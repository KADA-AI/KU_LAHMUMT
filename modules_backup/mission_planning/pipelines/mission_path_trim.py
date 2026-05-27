from __future__ import annotations

import importlib
import json
from typing import Any, Dict, List, Tuple
from types import ModuleType

from modules.common import db_paths
from modules.mission_planning.MissionPlanner.data_def.filming_altitude_guard import (
    normalize_filming_target_altitudes_in_waypoints,
)


_ID_ALLOCATOR_MOD: ModuleType | None = None
# Keep the sweep completion margin aligned with collaborative replan entry lookahead.
DEFAULT_SWEEP_SPLIT_LOOKAHEAD_SECONDS = 9.0


def load_sweep_progress() -> Dict[int, Dict[str, Any]]:
    """Load sweep progress cache keyed by pathID."""
    try:
        base = db_paths.get_db_subpath("DSS_Internal")
    except Exception:
        return {}
    path = base / "sweep_progress.json"
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
        path_id = _to_int(entry.get("path_id"))
        if path_id is None:
            continue
        result[path_id] = entry
    return result


def sweep_progress_points(entry: Dict[str, Any] | None) -> int:
    if not isinstance(entry, dict):
        return 0
    value = _to_int(entry.get("progress_points") or entry.get("progressPoints"))
    return max(0, value or 0)


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
        line_search["searchSpeed"] = round(speed_value * scale, 2)
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
) -> Tuple[List[Dict[str, Any]], int]:
    if not waypoints or cut_points <= 0:
        return waypoints, 0
    remaining = cut_points
    removed_points = 0
    new_list: List[Dict[str, Any]] = []
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
        if not preserve_waypoints:
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
                    # Only fill altitude when the existing value is missing/zero.
                    if coord_alt in (None, 0) and first_alt not in (None, 0):
                        coord["altitude"] = first_alt
                wp["coordinate"] = coord
        new_list.append(wp)
    return new_list, removed_points


def merge_small_adjacent_line_search_waypoints(
    waypoints: List[Dict[str, Any]],
    *,
    max_sweeps: int = 2,
    skip_last_pair: bool = False,
) -> Tuple[List[Dict[str, Any]], int]:
    if not waypoints or len(waypoints) < 2 or max_sweeps <= 0:
        return waypoints, 0

    merged_count = 0
    merged_waypoints: List[Dict[str, Any]] = []

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
        merged_count += 1

    return merged_waypoints, merged_count


def relink_waypoints(waypoints: List[Dict[str, Any]]) -> None:
    if not waypoints:
        return
    for idx in range(len(waypoints) - 1):
        waypoints[idx]["nextWaypointID"] = waypoints[idx + 1].get("waypointID", 0)
    waypoints[-1]["nextWaypointID"] = 0


def reassign_unique_waypoint_ids_inplace(waypoints: List[Dict[str, Any]]) -> List[int]:
    waypoint_dicts = [wp for wp in (waypoints or []) if isinstance(wp, dict)]
    if not waypoint_dicts:
        return []

    next_waypoint_id = int(_load_id_allocator().reserve_waypoint_block(len(waypoint_dicts)))
    assigned_ids: List[int] = []
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
