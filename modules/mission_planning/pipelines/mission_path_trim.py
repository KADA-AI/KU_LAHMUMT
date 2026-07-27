from __future__ import annotations

import importlib
import json
import math
from typing import Any, Callable, Dict, List, Tuple
from types import ModuleType

from modules.common import db_paths
from modules.mission_planning.MissionPlanner.data_def.filming_altitude_guard import (
    normalize_filming_target_altitudes_in_waypoints,
)
try:
    from modules.mission_planning.MissionPlanner.runtime_settings import (
        fov_db_path,
        get_runtime_camera_adjust_fov_scale,
        get_runtime_float,
        load_fov_db_rows,
    )
except Exception:
    fov_db_path = None  # type: ignore[assignment]
    get_runtime_camera_adjust_fov_scale = None  # type: ignore[assignment]
    get_runtime_float = None  # type: ignore[assignment]
    load_fov_db_rows = None  # type: ignore[assignment]
from modules.mission_planning.pipelines.line_search_speed_guard import (
    clamp_line_search_speed_mps,
    effective_line_search_transit_m,
)


_ID_ALLOCATOR_MOD: ModuleType | None = None
# Keep the sweep completion margin aligned with collaborative replan entry lookahead.
DEFAULT_SWEEP_SPLIT_LOOKAHEAD_SECONDS = 5.0
_EARTH_RADIUS_M = 6_371_000.0
_FOV_DB_MIN_SEP_CACHE: Dict[Tuple[float, float, str, int, int], float] = {}


def _new_route_offset_lookup_context() -> Dict[str, Any]:
    return {}


def load_sweep_progress() -> Dict[int, Dict[str, Any]]:
    """Load sweep progress cache keyed by pathID."""
    try:
        base = db_paths.get_db_subpath("DSS_Internal")
    except Exception:
        return {}
    path = base / "sweep_progress.json"
    result: Dict[int, Dict[str, Any]] = {}
    if path.exists():
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            payload = {}
        entries = payload.get("entries") if isinstance(payload, dict) else None
        if isinstance(entries, list):
            for entry in entries:
                if not isinstance(entry, dict):
                    continue
                path_id = _to_int(entry.get("path_id"))
                if path_id is None:
                    continue
                result[path_id] = entry
    for path_id, line_entry in load_line_scan_progress().items():
        merged = dict(result.get(int(path_id)) or {})
        progress_percent = _to_int(line_entry.get("progressPercent"))
        sweep_points = _to_int(line_entry.get("sweepPointCount")) or 0
        progress_points = _to_int(line_entry.get("progressPoints"))
        if progress_points is None and sweep_points > 0 and progress_percent is not None:
            progress_points = int(round((max(0, min(100, int(progress_percent))) / 100.0) * sweep_points))
        buffer_points = _to_int(line_entry.get("bufferPoints"))
        if buffer_points is None:
            buffer_points = progress_points
        line_summary = _line_scan_summary(line_entry)
        merged.update(
            {
                "timestamp_ms": line_entry.get("timestampMs"),
                "aircraft_id": line_entry.get("aircraftID"),
                "mission_id": line_entry.get("missionID"),
                "input_mission_id": line_entry.get("inputMissionID"),
                "mission_plan_id": line_entry.get("missionPlanID"),
                "path_id": int(path_id),
                "sweep_point_count": int(sweep_points),
                "progress_percent": max(0, min(100, int(progress_percent or 0))),
                "progress_points": max(0, int(progress_points or 0)),
                "buffer_points": max(0, int(buffer_points or 0)),
                "line_count": line_summary.get("line_count", 0),
                "visited_line_count": line_summary.get("visited_line_count", 0),
                "completed_line_count": line_summary.get("completed_line_count", 0),
                "remaining_line_count": line_summary.get("remaining_line_count", 0),
                "current_line_index": line_summary.get("current_line_index"),
                "line_transition_count": line_summary.get("line_transition_count", 0),
                "line_direction_change_count": line_summary.get("line_direction_change_count", 0),
                "planned_length_m": line_summary.get("planned_length_m", 0.0),
                "covered_length_m": line_summary.get("covered_length_m", 0.0),
                "remaining_length_m": line_summary.get("remaining_length_m", 0.0),
                "line_remaining_fragment_count": line_summary.get("line_remaining_fragment_count", 0),
                "line_remaining_intervals": line_summary.get("line_remaining_intervals", []),
                "line_completed_indexes": line_summary.get("line_completed_indexes", []),
                "line_scan": dict(line_entry),
                "line_scan_reassignment": line_summary,
                "progress_source": "line_scan",
            }
        )
        result[int(path_id)] = merged
    return result


def load_line_scan_progress() -> Dict[int, Dict[str, Any]]:
    """Load line-only scan progress cache keyed by pathID."""
    try:
        base = db_paths.get_db_subpath("DSS_Internal")
    except Exception:
        return {}
    path = base / "line_scan_progress.json"
    if not path.exists():
        return {}
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    entries = payload.get("entries") if isinstance(payload, dict) else None
    if not isinstance(entries, list):
        return {}
    result: Dict[int, Dict[str, Any]] = {}
    for entry in entries:
        if not isinstance(entry, dict):
            continue
        path_id = _to_int(entry.get("pathID") or entry.get("path_id"))
        if path_id is None:
            continue
        result[int(path_id)] = entry
    return result


def _line_scan_summary(line_entry: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(line_entry, dict):
        return {}
    line_rows = [row for row in (line_entry.get("lineList") or []) if isinstance(row, dict)]
    remaining_intervals: List[Dict[str, Any]] = []
    completed_indexes: List[int] = []
    remaining_fragment_count = 0
    for fallback_idx, row in enumerate(line_rows):
        line_idx = _to_int(row.get("lineIndex"))
        if line_idx is None:
            line_idx = int(fallback_idx)
        coverage = _to_float(row.get("coveragePercent"))
        if coverage is not None and coverage >= 95.0:
            completed_indexes.append(int(line_idx))
        intervals = row.get("remainingIntervals")
        if isinstance(intervals, list):
            for item in intervals:
                if not isinstance(item, dict):
                    continue
                start_m = _to_float(item.get("startM"))
                end_m = _to_float(item.get("endM"))
                if start_m is None or end_m is None or end_m <= start_m:
                    continue
                remaining_fragment_count += 1
                remaining_intervals.append(
                    {
                        "lineIndex": int(line_idx),
                        "startM": round(float(start_m), 3),
                        "endM": round(float(end_m), 3),
                        "lengthM": round(float(end_m) - float(start_m), 3),
                    }
                )
    line_count = _to_int(line_entry.get("lineCount"))
    if line_count is None:
        line_count = len(line_rows)
    completed_line_count = _to_int(line_entry.get("completedLineCount"))
    if completed_line_count is None:
        completed_line_count = len(set(completed_indexes))
    remaining_line_count = _to_int(line_entry.get("remainingLineCount"))
    if remaining_line_count is None:
        remaining_line_count = max(0, int(line_count or 0) - int(completed_line_count or 0))
    return {
        "mission_plan_id": _to_int(line_entry.get("missionPlanID")),
        "input_mission_id": _to_int(line_entry.get("inputMissionID")),
        "aircraft_id": _to_int(line_entry.get("aircraftID")),
        "mission_id": _to_int(line_entry.get("missionID")),
        "path_id": _to_int(line_entry.get("pathID")),
        "line_count": max(0, int(line_count or 0)),
        "visited_line_count": max(0, int(_to_int(line_entry.get("visitedLineCount")) or 0)),
        "completed_line_count": max(0, int(completed_line_count or 0)),
        "remaining_line_count": max(0, int(remaining_line_count or 0)),
        "current_line_index": _to_int(line_entry.get("currentLineIndex")),
        "line_transition_count": max(0, int(_to_int(line_entry.get("lineTransitionCount")) or 0)),
        "line_direction_change_count": max(0, int(_to_int(line_entry.get("lineDirectionChangeCount")) or 0)),
        "planned_length_m": float(_to_float(line_entry.get("plannedLengthM")) or 0.0),
        "covered_length_m": float(_to_float(line_entry.get("coveredLengthM")) or 0.0),
        "remaining_length_m": float(_to_float(line_entry.get("remainingLengthM")) or 0.0),
        "line_remaining_fragment_count": int(remaining_fragment_count),
        "line_remaining_intervals": remaining_intervals,
        "line_completed_indexes": sorted(set(completed_indexes)),
    }


def sweep_progress_points(entry: Dict[str, Any] | None) -> int:
    if not isinstance(entry, dict):
        return 0
    value = _to_int(entry.get("progress_points") or entry.get("progressPoints"))
    points = max(0, value or 0)
    line_scan = entry.get("line_scan") if isinstance(entry.get("line_scan"), dict) else entry
    if isinstance(line_scan, dict):
        sweep_points = _to_int(
            entry.get("sweep_point_count")
            or entry.get("sweepPointCount")
            or line_scan.get("sweepPointCount")
        ) or 0
        planned_m = _to_float(line_scan.get("plannedLengthM") or entry.get("planned_length_m"))
        covered_m = _to_float(line_scan.get("coveredLengthM") or entry.get("covered_length_m"))
        if sweep_points > 0 and planned_m is not None and planned_m > 0.0 and covered_m is not None:
            ratio_points = int(round((max(0.0, min(float(covered_m), float(planned_m))) / float(planned_m)) * sweep_points))
            points = max(points, max(0, min(int(sweep_points), int(ratio_points))))
    return points


def is_line_scan_progress_entry(entry: Dict[str, Any] | None) -> bool:
    if not isinstance(entry, dict):
        return False
    if str(entry.get("progress_source") or "").strip().lower() == "line_scan":
        return True
    line_scan = entry.get("line_scan")
    if isinstance(line_scan, dict):
        return True
    source = str(entry.get("source") or "").strip().lower()
    return source == "line_scan_progress_monitor"


def estimate_sweep_buffer_points(
    entry: Dict[str, Any] | None,
    buffer_seconds: float,
) -> int:
    if not isinstance(entry, dict):
        return 0
    try:
        extra_seconds = float(buffer_seconds)
    except Exception:
        extra_seconds = 0.0
    progress_points = sweep_progress_points(entry)
    if extra_seconds <= 0.0:
        return progress_points

    sweep_points = _to_int(entry.get("sweep_point_count") or entry.get("sweepPointCount")) or 0
    planned_seconds = _to_float(entry.get("planned_seconds") or entry.get("plannedSeconds"))
    elapsed_seconds = _to_float(entry.get("elapsed_seconds") or entry.get("elapsedSeconds"))
    if sweep_points > 0 and planned_seconds is not None and planned_seconds > 0.0:
        elapsed = max(0.0, float(elapsed_seconds or 0.0))
        buffer_ratio = max(0.0, min(1.0, (elapsed + extra_seconds) / planned_seconds))
        return max(0, min(sweep_points, int(buffer_ratio * sweep_points)))

    sec_per_point = _to_float(entry.get("seconds_per_point") or entry.get("secondsPerPoint"))
    if sec_per_point is None or sec_per_point <= 0.0:
        return progress_points

    # Match the sweep-progress producer's truncation semantics for consistency.
    extra_points = max(0, int(extra_seconds / sec_per_point))
    estimated = max(0, progress_points + extra_points)
    if sweep_points > 0:
        return min(sweep_points, estimated)
    return estimated


def physical_sweep_buffer_points(
    entry: Dict[str, Any] | None,
    buffer_seconds: float,
) -> int:
    if is_line_scan_progress_entry(entry):
        return 0
    return estimate_sweep_buffer_points(entry, buffer_seconds)


def sweep_cut_points(
    entry: Dict[str, Any] | None,
    *,
    default_buffer_seconds: float = 0.0,
) -> int:
    if not isinstance(entry, dict):
        return 0
    base_points: int | None = None
    for key in ("buffer_points", "bufferPoints"):
        value = _to_int(entry.get(key))
        if value is not None:
            base_points = max(0, value)
            break
    if base_points is None:
        sweep_points = _to_int(entry.get("sweep_point_count") or entry.get("sweepPointCount")) or 0
        buffer_pct = _to_int(entry.get("buffer_percent") or entry.get("bufferPercent"))
        if sweep_points > 0 and buffer_pct is not None:
            base_points = max(0, int(round((buffer_pct / 100.0) * sweep_points)))
        else:
            base_points = 0

    if default_buffer_seconds > 0.0:
        estimated_points = estimate_sweep_buffer_points(entry, default_buffer_seconds)
        if estimated_points > base_points:
            return estimated_points
    return base_points


def physical_sweep_cut_points(
    entry: Dict[str, Any] | None,
    *,
    default_buffer_seconds: float = 0.0,
) -> int:
    if is_line_scan_progress_entry(entry):
        return 0
    return sweep_cut_points(entry, default_buffer_seconds=default_buffer_seconds)


def count_sweep_points_in_waypoints(waypoints: List[Dict[str, Any]]) -> int:
    total = 0
    for wp in waypoints or []:
        if not isinstance(wp, dict):
            continue
        fp = wp.get("filmingProperty")
        if not isinstance(fp, dict):
            continue
        line_search = fp.get("lineSearch")
        if not isinstance(line_search, dict):
            continue
        coords = line_search.get("coordinateList")
        if not isinstance(coords, list):
            continue
        total += len(coords)
    return total


def _coord_distance_m(a: Dict[str, Any] | None, b: Dict[str, Any] | None) -> float | None:
    if not isinstance(a, dict) or not isinstance(b, dict):
        return None
    lat1 = _to_float(a.get("latitude"))
    lon1 = _to_float(a.get("longitude"))
    lat2 = _to_float(b.get("latitude"))
    lon2 = _to_float(b.get("longitude"))
    if None in (lat1, lon1, lat2, lon2):
        return None
    phi1 = math.radians(float(lat1))
    phi2 = math.radians(float(lat2))
    d_phi = math.radians(float(lat2) - float(lat1))
    d_lambda = math.radians(float(lon2) - float(lon1))
    a_val = (
        math.sin(d_phi / 2.0) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2.0) ** 2
    )
    return 6_371_000.0 * 2.0 * math.atan2(math.sqrt(a_val), math.sqrt(max(0.0, 1.0 - a_val)))


def _line_search_distance_m(coords: List[Dict[str, Any]]) -> float:
    total = 0.0
    prev_coord: Dict[str, Any] | None = None
    for coord in coords:
        if not isinstance(coord, dict):
            continue
        if prev_coord is not None:
            dist_m = _coord_distance_m(prev_coord, coord)
            if dist_m is not None and dist_m > 0.0:
                total += float(dist_m)
        prev_coord = coord
    return float(total)


def _valid_coord(coord: Dict[str, Any] | None) -> Dict[str, Any] | None:
    if not isinstance(coord, dict):
        return None
    lat = _to_float(coord.get("latitude"))
    lon = _to_float(coord.get("longitude"))
    if lat is None or lon is None:
        return None
    out = dict(coord)
    out["latitude"] = float(lat)
    out["longitude"] = float(lon)
    return out


def _coord_to_local_xy(coord: Dict[str, Any], origin: Dict[str, Any]) -> Tuple[float, float] | None:
    coord_valid = _valid_coord(coord)
    origin_valid = _valid_coord(origin)
    if coord_valid is None or origin_valid is None:
        return None
    lat0 = math.radians(float(origin_valid["latitude"]))
    d_lat = math.radians(float(coord_valid["latitude"]) - float(origin_valid["latitude"]))
    d_lon = math.radians(float(coord_valid["longitude"]) - float(origin_valid["longitude"]))
    return (
        float(d_lon) * _EARTH_RADIUS_M * math.cos(lat0),
        float(d_lat) * _EARTH_RADIUS_M,
    )


def _local_xy_to_coord(
    x_m: float,
    y_m: float,
    origin: Dict[str, Any],
    *,
    altitude_source: Dict[str, Any] | None = None,
) -> Dict[str, Any] | None:
    origin_valid = _valid_coord(origin)
    if origin_valid is None:
        return None
    lat0 = math.radians(float(origin_valid["latitude"]))
    lon0 = math.radians(float(origin_valid["longitude"]))
    lat = lat0 + (float(y_m) / _EARTH_RADIUS_M)
    cos_lat = math.cos(lat0)
    if abs(cos_lat) <= 1e-9:
        return None
    lon = lon0 + (float(x_m) / (_EARTH_RADIUS_M * cos_lat))
    out: Dict[str, Any] = {
        "latitude": round(math.degrees(lat), 6),
        "longitude": round(math.degrees(lon), 6),
    }
    alt_source = altitude_source if isinstance(altitude_source, dict) else origin
    alt = _to_float((alt_source or {}).get("altitude"))
    if alt is not None:
        out["altitude"] = int(round(float(alt)))
    return out


def _first_sweep_line_coords(
    coords: List[Dict[str, Any]] | None,
    interpolation_points: int | None,
) -> List[Dict[str, Any]]:
    valid = [_valid_coord(coord) for coord in (coords or [])]
    valid = [coord for coord in valid if coord is not None]
    if len(valid) < 2:
        return []
    try:
        points_per_line = int(interpolation_points or 0)
    except Exception:
        points_per_line = 0
    if points_per_line < 2:
        points_per_line = 2
    chunk = valid[: min(points_per_line, len(valid))]
    if len(chunk) < 2:
        return []
    return [chunk[0], chunk[-1]]


def _signed_offset_m_from_line_anchor(
    anchor_coord: Dict[str, Any] | None,
    line_coords: List[Dict[str, Any]] | None,
    interpolation_points: int | None,
) -> float | None:
    anchor = _valid_coord(anchor_coord)
    line = _first_sweep_line_coords(line_coords, interpolation_points)
    if anchor is None or len(line) < 2:
        return None
    origin = line[0]
    end_xy = _coord_to_local_xy(line[-1], origin)
    anchor_xy = _coord_to_local_xy(anchor, origin)
    if end_xy is None or anchor_xy is None:
        return None
    dx = float(end_xy[0])
    dy = float(end_xy[1])
    norm = math.hypot(dx, dy)
    if norm <= 1e-6:
        return None
    mid_x = dx * 0.5
    mid_y = dy * 0.5
    normal_x = dy / norm
    normal_y = -dx / norm
    signed_offset_m = (
        (float(anchor_xy[0]) - mid_x) * normal_x
        + (float(anchor_xy[1]) - mid_y) * normal_y
    )
    if not math.isfinite(float(signed_offset_m)):
        return None
    return float(signed_offset_m)


def _runtime_line_route_offset_scale() -> float:
    if get_runtime_float is None:
        return 1.0
    try:
        value = float(get_runtime_float("line_route_offset_scale", 1.0))
    except Exception:
        value = 1.0
    return max(float(value), 0.0)


def _runtime_camera_adjust_scale_no_log() -> float:
    if get_runtime_camera_adjust_fov_scale is None:
        return 1.0
    try:
        scale = float(get_runtime_camera_adjust_fov_scale())
    except Exception:
        scale = 1.0
    if not math.isfinite(float(scale)) or scale <= 0.0:
        return 1.0
    return max(float(scale), 0.1)


def _route_offset_context_value(context: Dict[str, Any] | None, key: str, loader: Callable[[], Any]) -> Any:
    if isinstance(context, dict) and key in context:
        return context.get(key)
    value = loader()
    if isinstance(context, dict):
        context[key] = value
    return value


def _route_offset_context_scale(context: Dict[str, Any] | None) -> float:
    return float(
        _route_offset_context_value(
            context,
            "route_offset_scale",
            _runtime_line_route_offset_scale,
        )
    )


def _route_offset_context_camera_adjust_scale(context: Dict[str, Any] | None) -> float:
    return float(
        _route_offset_context_value(
            context,
            "camera_adjust_scale",
            _runtime_camera_adjust_scale_no_log,
        )
    )


def _route_offset_context_fov_db_signature(context: Dict[str, Any] | None) -> Tuple[str, int, int] | None:
    value = _route_offset_context_value(context, "fov_db_signature", _fov_db_cache_signature)
    if isinstance(value, tuple) and len(value) == 3:
        try:
            return str(value[0]), int(value[1]), int(value[2])
        except Exception:
            return None
    return None


def _route_offset_context_fov_rows(context: Dict[str, Any] | None) -> List[Dict[str, Any]]:
    def _load_rows() -> List[Dict[str, Any]]:
        if load_fov_db_rows is None:
            return []
        try:
            return list(load_fov_db_rows())
        except Exception:
            return []

    rows = _route_offset_context_value(context, "fov_rows", _load_rows)
    return rows if isinstance(rows, list) else []


def _fov_db_cache_signature() -> Tuple[str, int, int] | None:
    if fov_db_path is None:
        return None
    try:
        path = fov_db_path()
        stat = path.stat()
        return str(path.resolve()), int(stat.st_mtime_ns), int(stat.st_size)
    except Exception:
        return None


def _fov_db_min_sep_for_fov(
    fov_deg: Any,
    *,
    lookup_context: Dict[str, Any] | None = None,
) -> float:
    try:
        target_fov = float(fov_deg)
    except Exception:
        return 0.0
    if target_fov <= 0.0 or load_fov_db_rows is None:
        return 0.0
    rows = _route_offset_context_fov_rows(lookup_context)
    if not rows:
        return 0.0

    adjust_scale = _route_offset_context_camera_adjust_scale(lookup_context)
    db_sig = _route_offset_context_fov_db_signature(lookup_context)
    cache_key: Tuple[float, float, str, int, int] | None = None
    if db_sig is not None:
        cache_key = (
            round(float(target_fov), 4),
            round(float(adjust_scale), 6),
            db_sig[0],
            db_sig[1],
            db_sig[2],
        )
        cached = _FOV_DB_MIN_SEP_CACHE.get(cache_key)
        if cached is not None:
            return float(cached)

    matches: List[float] = []
    for row in rows:
        try:
            row_fov = float(row.get("fov", 0.0) or 0.0)
            row_sep = float(row.get("sep", 0.0) or 0.0)
        except Exception:
            continue
        if row_fov <= 0.0 or row_sep <= 0.0:
            continue
        if abs(row_fov - target_fov) <= 0.05:
            matches.append(float(row_sep))

    if not matches and abs(float(adjust_scale) - 1.0) > 1e-9:
        for row in rows:
            try:
                row_fov = float(row.get("fov", 0.0) or 0.0)
                row_sep = float(row.get("sep", 0.0) or 0.0)
            except Exception:
                continue
            if row_fov <= 0.0 or row_sep <= 0.0:
                continue
            try:
                adjusted_fov = max(float(row_fov) * float(adjust_scale), 0.1)
            except Exception:
                adjusted_fov = row_fov
            if abs(adjusted_fov - target_fov) <= 0.05:
                matches.append(float(row_sep))

    result = min(matches) if matches else 0.0
    if cache_key is not None:
        if len(_FOV_DB_MIN_SEP_CACHE) > 512:
            _FOV_DB_MIN_SEP_CACHE.clear()
        _FOV_DB_MIN_SEP_CACHE[cache_key] = float(result)
    return float(result)


def _route_offset_cap_m_for_waypoint(
    waypoint: Dict[str, Any],
    *,
    lookup_context: Dict[str, Any] | None = None,
) -> float:
    fp = waypoint.get("filmingProperty") if isinstance(waypoint, dict) else None
    if not isinstance(fp, dict):
        return 0.0
    fov_deg = _to_float(fp.get("fieldOfView"))
    if fov_deg is None or fov_deg <= 0.0:
        return 0.0
    sep_m = _fov_db_min_sep_for_fov(float(fov_deg), lookup_context=lookup_context)
    if sep_m <= 0.0:
        return 0.0
    return max(float(sep_m) * _route_offset_context_scale(lookup_context), 1.0)


def _clamp_signed_offset_for_waypoint(
    signed_offset_m: float,
    waypoint: Dict[str, Any],
    *,
    lookup_context: Dict[str, Any] | None = None,
) -> float:
    try:
        signed = float(signed_offset_m)
    except Exception:
        return 0.0
    if not math.isfinite(float(signed)):
        return 0.0
    cap_m = _route_offset_cap_m_for_waypoint(waypoint, lookup_context=lookup_context)
    if cap_m <= 0.0 or abs(float(signed)) <= float(cap_m) + 1e-9:
        return float(signed)
    return math.copysign(float(cap_m), float(signed))


def _signed_route_offset_from_reference(
    waypoint: Dict[str, Any],
    line_coords: List[Dict[str, Any]] | None,
    interpolation_points: int | None,
    reference_coord: Dict[str, Any] | None,
    fallback_signed_offset_m: float | None,
    *,
    lookup_context: Dict[str, Any] | None = None,
) -> float | None:
    reference_signed = _signed_offset_m_from_line_anchor(
        reference_coord if isinstance(reference_coord, dict) else None,
        line_coords,
        interpolation_points,
    )
    sign_source = reference_signed
    if sign_source is None or abs(float(sign_source)) <= 1e-6:
        sign_source = fallback_signed_offset_m
    if sign_source is None or abs(float(sign_source)) <= 1e-6:
        sign_source = 1.0

    cap_m = _route_offset_cap_m_for_waypoint(waypoint, lookup_context=lookup_context)
    if cap_m > 0.0:
        magnitude_m = float(cap_m)
    elif fallback_signed_offset_m is not None and abs(float(fallback_signed_offset_m)) > 1e-6:
        magnitude_m = abs(float(fallback_signed_offset_m))
    elif reference_signed is not None and abs(float(reference_signed)) > 1e-6:
        magnitude_m = abs(float(reference_signed))
    else:
        return None
    return math.copysign(float(magnitude_m), float(sign_source))


def _line_anchor_coord_from_signed_offset(
    line_coords: List[Dict[str, Any]] | None,
    interpolation_points: int | None,
    signed_offset_m: float,
    *,
    altitude_source: Dict[str, Any] | None = None,
) -> Dict[str, Any] | None:
    line = _first_sweep_line_coords(line_coords, interpolation_points)
    if len(line) < 2:
        return None
    origin = line[0]
    end_xy = _coord_to_local_xy(line[-1], origin)
    if end_xy is None:
        return None
    dx = float(end_xy[0])
    dy = float(end_xy[1])
    norm = math.hypot(dx, dy)
    if norm <= 1e-6:
        return None
    normal_x = dy / norm
    normal_y = -dx / norm
    anchor_x = (dx * 0.5) + (normal_x * float(signed_offset_m))
    anchor_y = (dy * 0.5) + (normal_y * float(signed_offset_m))
    return _local_xy_to_coord(
        anchor_x,
        anchor_y,
        origin,
        altitude_source=altitude_source,
    )


def realign_line_search_waypoint_to_first_sweep(
    waypoint: Dict[str, Any],
    *,
    signed_offset_m: float | None = None,
    reference_coord_for_offset: Dict[str, Any] | None = None,
    lookup_context: Dict[str, Any] | None = None,
) -> bool:
    if not isinstance(waypoint, dict):
        return False
    fp = waypoint.get("filmingProperty")
    if not isinstance(fp, dict):
        return False
    line_search = fp.get("lineSearch")
    if not isinstance(line_search, dict):
        return False
    coords = _line_search_coordinate_list(line_search)
    if not coords or len(coords) < 2:
        return False
    interp_points = _line_search_interpolation_points(line_search)
    if interp_points is None:
        interp_points = _to_int(line_search.get("interpolationPoints")) or 2
    if signed_offset_m is None:
        signed_offset_m = _signed_offset_m_from_line_anchor(
            waypoint.get("coordinate") if isinstance(waypoint.get("coordinate"), dict) else None,
            coords,
            interp_points,
        )
    if reference_coord_for_offset is not None:
        reference_signed_offset_m = _signed_route_offset_from_reference(
            waypoint,
            coords,
            interp_points,
            reference_coord_for_offset,
            signed_offset_m,
            lookup_context=lookup_context,
        )
        if reference_signed_offset_m is not None:
            signed_offset_m = reference_signed_offset_m
    if signed_offset_m is None:
        return False
    signed_offset_m = _clamp_signed_offset_for_waypoint(
        float(signed_offset_m),
        waypoint,
        lookup_context=lookup_context,
    )
    anchor_coord = _line_anchor_coord_from_signed_offset(
        coords,
        interp_points,
        float(signed_offset_m),
        altitude_source=waypoint.get("coordinate") if isinstance(waypoint.get("coordinate"), dict) else None,
    )
    if anchor_coord is None:
        return False
    waypoint["coordinate"] = anchor_coord
    return True


def realign_line_search_waypoints_to_first_sweep(
    waypoints: List[Dict[str, Any]],
    *,
    reference_coord_for_offset: Dict[str, Any] | None = None,
) -> int:
    changed = 0
    lookup_context = _new_route_offset_lookup_context()
    for waypoint in waypoints or []:
        if realign_line_search_waypoint_to_first_sweep(
            waypoint,
            reference_coord_for_offset=reference_coord_for_offset,
            lookup_context=lookup_context,
        ):
            changed += 1
    return changed


def recompute_line_search_speed_from_geometry(
    waypoints: List[Dict[str, Any]],
    *,
    first_reference_coord: Dict[str, Any] | None = None,
    speed_scale: float = 1.0,
    only_increase: bool = True,
    multiplier_cap_enabled: bool = True,
) -> int:
    if not waypoints:
        return 0
    try:
        effective_scale = max(float(speed_scale), 0.1)
    except Exception:
        effective_scale = 1.0

    changed = 0
    prev_coord = first_reference_coord if isinstance(first_reference_coord, dict) else None
    for wp in waypoints:
        if not isinstance(wp, dict):
            continue
        fp = wp.get("filmingProperty")
        if not isinstance(fp, dict):
            coord = wp.get("coordinate")
            if isinstance(coord, dict):
                prev_coord = coord
            continue
        line_search = fp.get("lineSearch")
        if not isinstance(line_search, dict):
            coord = wp.get("coordinate")
            if isinstance(coord, dict):
                prev_coord = coord
            continue
        coords = [coord for coord in (line_search.get("coordinateList") or []) if isinstance(coord, dict)]
        if len(coords) < 2:
            continue
        anchor_coord = wp.get("coordinate") if isinstance(wp.get("coordinate"), dict) else coords[0]
        transit_distance_m = _coord_distance_m(prev_coord, anchor_coord)
        if transit_distance_m is None or transit_distance_m <= 1e-6:
            transit_distance_m = _coord_distance_m(prev_coord, coords[0])
        if transit_distance_m is None or transit_distance_m <= 1e-6:
            coord = wp.get("coordinate")
            if isinstance(coord, dict):
                prev_coord = coord
            continue
        sweep_distance_m = _line_search_distance_m(coords)
        if sweep_distance_m <= 1e-6:
            continue
        cruise_speed = _to_float(wp.get("speed")) or 0.0
        if cruise_speed <= 0.0:
            cruise_speed = 40.0
        effective_transit_m = effective_line_search_transit_m(transit_distance_m)
        if effective_transit_m <= 1e-6:
            continue
        estimated_speed = (sweep_distance_m / (effective_transit_m / float(cruise_speed))) * float(effective_scale)
        estimated_speed = round(
            clamp_line_search_speed_mps(
                estimated_speed,
                cruise_speed_mps=float(cruise_speed),
                speed_scale=float(effective_scale),
                multiplier_cap_enabled=bool(multiplier_cap_enabled),
            ),
            2,
        )
        current_speed = _to_float(line_search.get("searchSpeed")) or 0.0
        if only_increase and current_speed > 0.0 and estimated_speed <= current_speed + 1e-9:
            coord = wp.get("coordinate")
            if isinstance(coord, dict):
                prev_coord = coord
            continue
        line_search["searchSpeed"] = float(estimated_speed)
        fp["lineSearch"] = line_search
        wp["filmingProperty"] = fp
        changed += 1
        coord = wp.get("coordinate")
        if isinstance(coord, dict):
            prev_coord = coord
    return changed


def preserve_first_waypoint_altitude_from_reference(
    waypoints: List[Dict[str, Any]],
    reference_coord: Dict[str, Any] | None,
) -> bool:
    if not waypoints or not isinstance(reference_coord, dict):
        return False
    reference_alt = _to_float(reference_coord.get("altitude"))
    if reference_alt is None:
        return False
    first_wp = waypoints[0] if isinstance(waypoints[0], dict) else None
    if not isinstance(first_wp, dict):
        return False
    coord = first_wp.get("coordinate") if isinstance(first_wp.get("coordinate"), dict) else None
    if not isinstance(coord, dict):
        return False
    lat = _to_float(coord.get("latitude"))
    lon = _to_float(coord.get("longitude"))
    if lat is None or lon is None:
        return False
    old_alt = _to_float(coord.get("altitude"))
    new_alt = int(round(float(reference_alt)))
    if old_alt is not None and int(round(float(old_alt))) == int(new_alt):
        return False
    first_wp["coordinate"] = {
        "latitude": round(float(lat), 6),
        "longitude": round(float(lon), 6),
        "altitude": int(new_alt),
    }
    return True


def scale_line_search_speed(
    waypoints: List[Dict[str, Any]],
    factor: float,
) -> int:
    if not waypoints:
        return 0
    try:
        scale = float(factor)
    except Exception:
        scale = 1.0
    if scale <= 0.0 or abs(scale - 1.0) <= 1e-9:
        return 0

    changed = 0
    for wp in waypoints:
        if not isinstance(wp, dict):
            continue
        fp = wp.get("filmingProperty")
        if not isinstance(fp, dict):
            continue
        line_search = fp.get("lineSearch")
        if not isinstance(line_search, dict):
            continue
        speed = line_search.get("searchSpeed")
        try:
            speed_value = float(speed)
        except Exception:
            continue
        if speed_value <= 0.0:
            continue
        cruise_speed = max(
            float(_to_float(wp.get("speed")) or 0.0),
            float(speed_value),
        )
        line_search["searchSpeed"] = round(
            clamp_line_search_speed_mps(
                speed_value * scale,
                cruise_speed_mps=cruise_speed,
                speed_scale=scale,
                minimum_speed_mps=float(speed_value),
            ),
            2,
        )
        fp["lineSearch"] = line_search
        wp["filmingProperty"] = fp
        changed += 1
    return changed


def trim_waypoints_by_is_done_prefix(
    waypoints: List[Dict[str, Any]],
) -> Tuple[List[Dict[str, Any]], int | None]:
    if not waypoints:
        return waypoints, None
    idx = 0
    while idx < len(waypoints) and bool(waypoints[idx].get("isDone")):
        idx += 1
    if idx <= 0:
        return waypoints, None
    last_removed = waypoints[idx - 1] if idx - 1 < len(waypoints) else None
    last_removed_id = _to_int(last_removed.get("waypointID")) if isinstance(last_removed, dict) else None
    return waypoints[idx:], last_removed_id


def trim_waypoints_by_sweep_points(
    waypoints: List[Dict[str, Any]],
    cut_points: int,
    *,
    preserve_waypoints: bool = False,
    reference_coord_for_offset: Dict[str, Any] | None = None,
) -> Tuple[List[Dict[str, Any]], int]:
    if not waypoints or cut_points <= 0:
        return waypoints, 0
    remaining = cut_points
    removed_points = 0
    new_list: List[Dict[str, Any]] = []
    lookup_context = _new_route_offset_lookup_context()
    for wp in waypoints:
        if remaining <= 0:
            new_list.append(wp)
            continue
        fp = wp.get("filmingProperty") if isinstance(wp, dict) else None
        if not isinstance(fp, dict):
            new_list.append(wp)
            continue
        line_search = fp.get("lineSearch")
        if not isinstance(line_search, dict):
            new_list.append(wp)
            continue
        coords = line_search.get("coordinateList")
        if not isinstance(coords, list) or not coords:
            new_list.append(wp)
            continue

        original_coords = list(coords)
        original_interp_points = _line_search_interpolation_points(line_search)
        if original_interp_points is None:
            original_interp_points = _to_int(line_search.get("interpolationPoints")) or 2
        original_signed_offset_m = _signed_offset_m_from_line_anchor(
            wp.get("coordinate") if isinstance(wp.get("coordinate"), dict) else None,
            original_coords,
            original_interp_points,
        )
        if preserve_waypoints:
            # Resume는 "잘린 지점 이후 좌표"를 그대로 이어서 수행해야 한다.
            # 따라서 남은 컷 포인트를 현재 waypoint sweep에 우선 적용하고,
            # sweep가 완전히 소진되면 해당 waypoint는 resume에서 제거한다.
            if remaining >= len(original_coords):
                removed_points += len(original_coords)
                remaining -= len(original_coords)
                continue

            coords = original_coords[remaining:]
            removed_points += remaining
            remaining = 0

            if len(coords) >= 2:
                line_search["coordinateList"] = coords
                fp["lineSearch"] = line_search
                wp["filmingProperty"] = fp
                realign_line_search_waypoint_to_first_sweep(
                    wp,
                    signed_offset_m=original_signed_offset_m,
                    reference_coord_for_offset=reference_coord_for_offset,
                    lookup_context=lookup_context,
                )
                new_list.append(wp)
                continue

            # 1점만 남으면 lineSearch는 제거하고 일반 이동 waypoint로 유지.
            first = coords[0] if coords else None
            if isinstance(first, dict):
                coord = wp.get("coordinate") if isinstance(wp, dict) else None
                if not isinstance(coord, dict):
                    coord = {}
                if "latitude" in first:
                    coord["latitude"] = first.get("latitude")
                if "longitude" in first:
                    coord["longitude"] = first.get("longitude")
                if "altitude" in first:
                    first_alt = first.get("altitude")
                    coord_alt = coord.get("altitude") if isinstance(coord, dict) else None
                    if coord_alt in (None, 0) and first_alt not in (None, 0):
                        coord["altitude"] = first_alt
                wp["coordinate"] = coord
            try:
                fp.pop("lineSearch", None)
            except Exception:
                pass
            fp["operationMode"] = 1
            if isinstance(first, dict):
                fp["coordinateOrientation"] = {"coordinate": dict(first)}
            elif isinstance(wp.get("coordinate"), dict):
                fp["coordinateOrientation"] = {"coordinate": dict(wp["coordinate"])}
            wp["filmingProperty"] = fp
            normalize_filming_target_altitudes_in_waypoints([wp])
            new_list.append(wp)
            continue

        if remaining >= len(original_coords):
            removed_points += len(original_coords)
            remaining -= len(original_coords)
            continue

        coords = original_coords[remaining:]
        removed_points += remaining
        remaining = 0
        if len(coords) < 2:
            continue
        line_search["coordinateList"] = coords
        fp["lineSearch"] = line_search
        wp["filmingProperty"] = fp
        realign_line_search_waypoint_to_first_sweep(
            wp,
            signed_offset_m=original_signed_offset_m,
            reference_coord_for_offset=reference_coord_for_offset,
            lookup_context=lookup_context,
        )
        new_list.append(wp)
    return new_list, removed_points


def merge_small_adjacent_line_search_waypoints(
    waypoints: List[Dict[str, Any]],
    *,
    max_sweeps: int = 2,
    skip_last_pair: bool = False,
    reference_coord_for_offset: Dict[str, Any] | None = None,
) -> Tuple[List[Dict[str, Any]], int]:
    if not waypoints or len(waypoints) < 2 or max_sweeps <= 0:
        return waypoints, 0

    merged_count = 0
    merged_waypoints: List[Dict[str, Any]] = []
    lookup_context = _new_route_offset_lookup_context()

    last_index = len(waypoints) - 1
    for idx, waypoint in enumerate(waypoints):
        if not isinstance(waypoint, dict):
            merged_waypoints.append(waypoint)
            continue
        if not merged_waypoints:
            merged_waypoints.append(waypoint)
            continue
        if skip_last_pair and idx == last_index:
            merged_waypoints.append(waypoint)
            continue

        prev_waypoint = merged_waypoints[-1]
        prev_line_search = _extract_line_search(prev_waypoint)
        curr_line_search = _extract_line_search(waypoint)
        if prev_line_search is None or curr_line_search is None:
            merged_waypoints.append(waypoint)
            continue

        prev_interp = _line_search_interpolation_points(prev_line_search)
        curr_interp = _line_search_interpolation_points(curr_line_search)
        if prev_interp is None or curr_interp is None or prev_interp != curr_interp:
            merged_waypoints.append(waypoint)
            continue

        curr_coords = _line_search_coordinate_list(curr_line_search)
        if curr_coords is None:
            merged_waypoints.append(waypoint)
            continue
        curr_sweeps = len(curr_coords) // curr_interp
        if curr_sweeps <= 0 or curr_sweeps > max_sweeps:
            merged_waypoints.append(waypoint)
            continue

        prev_coords = _line_search_coordinate_list(prev_line_search)
        if prev_coords is None:
            merged_waypoints.append(waypoint)
            continue

        append_coords = curr_coords
        if prev_coords and curr_coords and _coords_equivalent(prev_coords[-1], curr_coords[0]):
            append_coords = curr_coords[1:]
        if not append_coords:
            merged_count += 1
            continue

        prev_line_search["coordinateList"] = list(prev_coords) + list(append_coords)
        prev_fp = prev_waypoint.get("filmingProperty")
        if isinstance(prev_fp, dict):
            prev_fp["lineSearch"] = prev_line_search
            prev_waypoint["filmingProperty"] = prev_fp
            realign_line_search_waypoint_to_first_sweep(
                prev_waypoint,
                reference_coord_for_offset=reference_coord_for_offset,
                lookup_context=lookup_context,
            )
        merged_count += 1

    return merged_waypoints, merged_count


def relink_waypoints(waypoints: List[Dict[str, Any]]) -> None:
    if not waypoints:
        return
    for idx in range(len(waypoints) - 1):
        waypoints[idx]["nextWaypointID"] = waypoints[idx + 1].get("waypointID", 0)
    waypoints[-1]["nextWaypointID"] = 0


def reassign_unique_waypoint_ids_inplace(
    waypoints: List[Dict[str, Any]],
    *,
    waypoint_id_provider: Callable[[], int] | None = None,
) -> List[int]:
    waypoint_dicts = [wp for wp in (waypoints or []) if isinstance(wp, dict)]
    if not waypoint_dicts:
        return []

    assigned_ids: List[int] = []
    if waypoint_id_provider is not None:
        for waypoint in waypoint_dicts:
            waypoint_id = int(waypoint_id_provider())
            waypoint["waypointID"] = waypoint_id
            assigned_ids.append(waypoint_id)
    else:
        next_waypoint_id = int(_load_id_allocator().reserve_waypoint_block(len(waypoint_dicts)))
        for waypoint in waypoint_dicts:
            waypoint["waypointID"] = int(next_waypoint_id)
            assigned_ids.append(int(next_waypoint_id))
            next_waypoint_id += 1

    relink_waypoints(waypoints)
    return assigned_ids


def _to_int(value: Any) -> int | None:
    if value is None:
        return None
    try:
        return int(value)
    except Exception:
        return None


def _to_float(value: Any) -> float | None:
    if value is None:
        return None
    try:
        return float(value)
    except Exception:
        return None


def _extract_line_search(waypoint: Dict[str, Any] | None) -> Dict[str, Any] | None:
    if not isinstance(waypoint, dict):
        return None
    filming_property = waypoint.get("filmingProperty")
    if not isinstance(filming_property, dict):
        return None
    line_search = filming_property.get("lineSearch")
    return line_search if isinstance(line_search, dict) else None


def _line_search_coordinate_list(line_search: Dict[str, Any] | None) -> List[Dict[str, Any]] | None:
    if not isinstance(line_search, dict):
        return None
    coords = line_search.get("coordinateList")
    if not isinstance(coords, list) or not coords:
        return None
    valid_coords = [coord for coord in coords if isinstance(coord, dict)]
    return valid_coords if valid_coords else None


def _line_search_interpolation_points(line_search: Dict[str, Any] | None) -> int | None:
    if not isinstance(line_search, dict):
        return None
    interpolation_points = _to_int(line_search.get("interpolationPoints"))
    if interpolation_points is None or interpolation_points <= 0:
        return None
    coords = _line_search_coordinate_list(line_search)
    if not coords or len(coords) % interpolation_points != 0:
        return None
    return interpolation_points


def _coords_equivalent(lhs: Dict[str, Any] | None, rhs: Dict[str, Any] | None) -> bool:
    if not isinstance(lhs, dict) or not isinstance(rhs, dict):
        return False
    lat_l = _to_float(lhs.get("latitude"))
    lon_l = _to_float(lhs.get("longitude"))
    alt_l = _to_float(lhs.get("altitude")) or 0.0
    lat_r = _to_float(rhs.get("latitude"))
    lon_r = _to_float(rhs.get("longitude"))
    alt_r = _to_float(rhs.get("altitude")) or 0.0
    if lat_l is None or lon_l is None or lat_r is None or lon_r is None:
        return False
    return (
        abs(lat_l - lat_r) <= 1e-9
        and abs(lon_l - lon_r) <= 1e-9
        and abs(alt_l - alt_r) <= 1e-6
    )


def _load_id_allocator() -> ModuleType:
    global _ID_ALLOCATOR_MOD
    if _ID_ALLOCATOR_MOD is None:
        _ID_ALLOCATOR_MOD = importlib.import_module(
            "modules.mission_planning.MissionPlanner.data_def.id_allocator"
        )
    return _ID_ALLOCATOR_MOD
