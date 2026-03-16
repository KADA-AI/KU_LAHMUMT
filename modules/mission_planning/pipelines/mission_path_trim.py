from __future__ import annotations

import json
from typing import Any, Dict, List, Tuple

from modules.common import db_paths


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


def sweep_cut_points(entry: Dict[str, Any] | None) -> int:
    if not isinstance(entry, dict):
        return 0
    for key in ("buffer_points", "bufferPoints"):
        value = _to_int(entry.get(key))
        if value is not None:
            return max(0, value)
    sweep_points = _to_int(entry.get("sweep_point_count") or entry.get("sweepPointCount")) or 0
    buffer_pct = _to_int(entry.get("buffer_percent") or entry.get("bufferPercent"))
    if sweep_points > 0 and buffer_pct is not None:
        return max(0, int(round((buffer_pct / 100.0) * sweep_points)))
    return 0


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
            wp["filmingProperty"] = fp
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


def relink_waypoints(waypoints: List[Dict[str, Any]]) -> None:
    if not waypoints:
        return
    for idx in range(len(waypoints) - 1):
        waypoints[idx]["nextWaypointID"] = waypoints[idx + 1].get("waypointID", 0)
    waypoints[-1]["nextWaypointID"] = 0


def _to_int(value: Any) -> int | None:
    if value is None:
        return None
    try:
        return int(value)
    except Exception:
        return None
