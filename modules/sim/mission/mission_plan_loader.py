from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Iterable

from modules.common import db_paths

EARTH_RADIUS_M = 6_371_008.8


def _coerce_int(value: Any) -> int | None:
    try:
        return int(value)
    except Exception:
        return None


def _coerce_bool(value: Any, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return default
    if isinstance(value, (int, float)):
        return bool(int(value))
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in ("1", "true", "yes", "y", "on"):
            return True
        if lowered in ("0", "false", "no", "n", "off"):
            return False
    return default


def _load_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}


def _input_package_id(data: dict[str, Any]) -> int | None:
    if not isinstance(data, dict):
        return None
    return _coerce_int(
        data.get("inputMissionPackageID")
        or data.get("InputMissionPackageID")
        or data.get("inputMissionPackageId")
    )


def _load_input_plan_for_package(
    base: Path,
    input_package_id: int | None,
) -> tuple[dict[str, Any], Path | None]:
    if input_package_id is None:
        return {}, None

    input_dir = Path(base) / "InputMissionPlan"
    direct_path = input_dir / f"{int(input_package_id)}.json"
    direct = _load_json(direct_path)
    if direct:
        return direct, direct_path

    try:
        candidates = sorted(
            input_dir.glob("*.json"),
            key=lambda path: path.stat().st_mtime,
            reverse=True,
        )
    except Exception:
        candidates = []

    for candidate in candidates:
        data = _load_json(candidate)
        if _input_package_id(data) == int(input_package_id):
            return data, candidate
    return {}, None


def normalize_input_mission_plan_float_fields(plan: dict[str, Any]) -> dict[str, Any]:
    """Normalize legacy numeric encodings to their current ICD JSON types.

    Coordinate.altitude is an int in the ICD, but older/new-target packages can
    contain integral JSON floats such as ``0.0``.  Convert only lossless values;
    non-integral floats remain untouched so validation still reports bad data.
    """

    def _walk(value: Any) -> None:
        if isinstance(value, dict):
            for key, item in list(value.items()):
                if (
                    str(key).lower() == "altitude"
                    and isinstance(item, float)
                    and math.isfinite(item)
                    and item.is_integer()
                ):
                    value[key] = int(item)
                else:
                    _walk(item)
        elif isinstance(value, list):
            for item in value:
                _walk(item)

    if isinstance(plan, dict):
        _walk(plan)
    return plan


def _extract_waypoints(data: dict[str, Any]) -> list[dict[str, Any]]:
    for key in ("lahWaypointList", "uavWaypointList", "waypointList"):
        lst = data.get(key)
        if isinstance(lst, list):
            return lst
    return []


def _extract_coord(item: dict[str, Any]) -> tuple[float, float, float | None] | None:
    coord = item.get("coordinate") or item.get("Coordinate")
    if not isinstance(coord, dict):
        return None
    lat = coord.get("latitude") if "latitude" in coord else coord.get("Latitude")
    lon = coord.get("longitude") if "longitude" in coord else coord.get("Longitude")
    alt = coord.get("altitude") if "altitude" in coord else coord.get("Altitude")
    if lat is None or lon is None:
        return None
    try:
        lat_v = float(lat)
        lon_v = float(lon)
        alt_v = float(alt) if alt is not None else None
    except Exception:
        return None
    return lat_v, lon_v, alt_v


def _extract_coord_dict(item: dict[str, Any]) -> tuple[float, float] | None:
    if not isinstance(item, dict):
        return None
    lat = item.get("latitude") if "latitude" in item else item.get("Latitude")
    lon = item.get("longitude") if "longitude" in item else item.get("Longitude")
    if lat is None or lon is None:
        return None
    try:
        lat_v = float(lat)
        lon_v = float(lon)
    except Exception:
        return None
    if not (-90.0 <= lat_v <= 90.0 and -180.0 <= lon_v <= 180.0):
        return None
    return lat_v, lon_v


def _line_search_block(wp: dict[str, Any]) -> dict[str, Any]:
    filming = wp.get("filmingProperty") or wp.get("FilmingProperty")
    if not isinstance(filming, dict):
        return {}
    line_search = filming.get("lineSearch") or filming.get("LineSearch")
    return line_search if isinstance(line_search, dict) else {}


def _extract_sweep_search_lines_from_waypoint(wp: dict[str, Any]) -> list[list[tuple[float, float]]]:
    line_search = _line_search_block(wp)
    coordinate_list = line_search.get("coordinateList") or line_search.get("CoordinateList")
    if not isinstance(coordinate_list, list):
        return []
    points = [
        coord
        for coord in (_extract_coord_dict(item) for item in coordinate_list if isinstance(item, dict))
        if coord is not None
    ]
    if len(points) < 2:
        return []

    chunk_size = _coerce_int(
        line_search.get("interpolationPoints")
        or line_search.get("InterpolationPoints")
        or line_search.get("interpolationPoint")
        or line_search.get("InterpolationPoint")
    )
    if chunk_size is not None and chunk_size > 2 and len(points) > chunk_size:
        chunks: list[list[tuple[float, float]]] = []
        for start in range(0, len(points), int(chunk_size)):
            chunk = points[start : start + int(chunk_size)]
            if len(chunk) >= 2:
                chunks.append(chunk)
        return chunks
    return [points]


def _extract_sweep_search_lines_from_path(path_data: dict[str, Any]) -> list[list[tuple[float, float]]]:
    lines: list[list[tuple[float, float]]] = []
    for wp in _order_waypoints(_extract_waypoints(path_data)):
        if not isinstance(wp, dict):
            continue
        lines.extend(_extract_sweep_search_lines_from_waypoint(wp))
    return lines


def _path_input_mission_index(individual_plans: Iterable[dict[str, Any]]) -> dict[int, dict[str, Any]]:
    index: dict[int, dict[str, Any]] = {}
    for plan in individual_plans:
        if not isinstance(plan, dict):
            continue
        aircraft_id = _coerce_int(plan.get("aircraftID") or plan.get("AircraftID"))
        for mission in plan.get("individualMissionList") or []:
            if not isinstance(mission, dict):
                continue
            path_id = _coerce_int(mission.get("pathID") or mission.get("PathID"))
            if path_id is None:
                continue
            related = mission.get("relatedMission") or mission.get("RelatedMission") or {}
            if not isinstance(related, dict):
                related = {}
            input_mission_id = _coerce_int(
                related.get("inputMissionID")
                or related.get("InputMissionID")
                or mission.get("inputMissionID")
                or mission.get("InputMissionID")
            )
            index[int(path_id)] = {
                "pathID": int(path_id),
                "aircraftID": aircraft_id,
                "inputMissionID": input_mission_id,
                "individualMissionID": _coerce_int(
                    mission.get("individualMissionID") or mission.get("IndividualMissionID")
                ),
            }
    return index


def _project_line_to_xy(
    line: list[tuple[float, float]],
    *,
    origin_lat: float,
    origin_lon: float,
) -> list[tuple[float, float]]:
    cos_lat = math.cos(math.radians(origin_lat))
    projected: list[tuple[float, float]] = []
    for lat, lon in line:
        x = math.radians(float(lon) - float(origin_lon)) * EARTH_RADIUS_M * cos_lat
        y = math.radians(float(lat) - float(origin_lat)) * EARTH_RADIUS_M
        projected.append((x, y))
    return projected


def _polyline_length_xy(points: list[tuple[float, float]]) -> float:
    total = 0.0
    for idx in range(1, len(points)):
        total += math.hypot(points[idx][0] - points[idx - 1][0], points[idx][1] - points[idx - 1][1])
    return total


def _point_at_fraction(points: list[tuple[float, float]], fraction: float) -> tuple[float, float]:
    if not points:
        return (0.0, 0.0)
    if len(points) == 1:
        return points[0]
    target = max(0.0, min(1.0, float(fraction))) * _polyline_length_xy(points)
    if target <= 0.0:
        return points[0]
    walked = 0.0
    for idx in range(1, len(points)):
        start = points[idx - 1]
        end = points[idx]
        seg_len = math.hypot(end[0] - start[0], end[1] - start[1])
        if seg_len <= 0.0:
            continue
        if walked + seg_len >= target:
            ratio = (target - walked) / seg_len
            return (start[0] + (end[0] - start[0]) * ratio, start[1] + (end[1] - start[1]) * ratio)
        walked += seg_len
    return points[-1]


def _sample_polyline(points: list[tuple[float, float]], sample_count: int = 9) -> list[tuple[float, float]]:
    if len(points) <= 2:
        return list(points)
    count = max(2, int(sample_count))
    return [_point_at_fraction(points, idx / float(count - 1)) for idx in range(count)]


def _point_segment_distance_xy(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    px, py = point
    ax, ay = start
    bx, by = end
    dx = bx - ax
    dy = by - ay
    denom = (dx * dx) + (dy * dy)
    if denom <= 0.0:
        return math.hypot(px - ax, py - ay)
    ratio = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / denom))
    cx = ax + dx * ratio
    cy = ay + dy * ratio
    return math.hypot(px - cx, py - cy)


def _point_polyline_distance_xy(point: tuple[float, float], line: list[tuple[float, float]]) -> float:
    if not line:
        return float("inf")
    if len(line) == 1:
        return math.hypot(point[0] - line[0][0], point[1] - line[0][1])
    return min(
        _point_segment_distance_xy(point, line[idx - 1], line[idx])
        for idx in range(1, len(line))
    )


def _mean_polyline_spacing_xy(
    left: list[tuple[float, float]],
    right: list[tuple[float, float]],
) -> float | None:
    if len(left) < 2 or len(right) < 2:
        return None
    left_samples = _sample_polyline(left)
    right_samples = _sample_polyline(right)
    distances = [
        _point_polyline_distance_xy(point, right)
        for point in left_samples
    ] + [
        _point_polyline_distance_xy(point, left)
        for point in right_samples
    ]
    distances = [value for value in distances if math.isfinite(value)]
    if not distances:
        return None
    return sum(distances) / float(len(distances))


def _build_sweep_line_spacing_summaries(
    flight_paths: Iterable[dict[str, Any]],
    path_mission_index: dict[int, dict[str, Any]],
) -> list[dict[str, Any]]:
    grouped: dict[int, dict[str, Any]] = {}
    for entry in flight_paths:
        if not isinstance(entry, dict):
            continue
        data = entry.get("data") if isinstance(entry.get("data"), dict) else entry
        if not isinstance(data, dict):
            continue
        path_id = _coerce_int(data.get("pathID") or data.get("PathID"))
        if path_id is None:
            continue
        mission_meta = path_mission_index.get(int(path_id), {})
        input_mission_id = _coerce_int(mission_meta.get("inputMissionID"))
        if input_mission_id is None:
            continue
        lines = _extract_sweep_search_lines_from_path(data)
        if not lines:
            continue
        group = grouped.setdefault(
            int(input_mission_id),
            {
                "inputMissionID": int(input_mission_id),
                "pathIds": set(),
                "aircraftIds": set(),
                "linesByPath": {},
                "allCoords": [],
            },
        )
        group["pathIds"].add(int(path_id))
        aircraft_id = _coerce_int(data.get("aircraftID") or data.get("AircraftID") or mission_meta.get("aircraftID"))
        if aircraft_id is not None:
            group["aircraftIds"].add(int(aircraft_id))
        group["linesByPath"].setdefault(int(path_id), []).extend(lines)
        for line in lines:
            group["allCoords"].extend(line)

    summaries: list[dict[str, Any]] = []
    for input_mission_id, group in sorted(grouped.items()):
        all_coords = group.get("allCoords") or []
        if not all_coords:
            continue
        origin_lat = sum(float(lat) for lat, _lon in all_coords) / float(len(all_coords))
        origin_lon = sum(float(lon) for _lat, lon in all_coords) / float(len(all_coords))
        distances: list[float] = []
        line_count = 0
        for path_id in sorted(group.get("linesByPath") or {}):
            raw_lines = group["linesByPath"].get(path_id) or []
            projected_lines = [
                _project_line_to_xy(line, origin_lat=origin_lat, origin_lon=origin_lon)
                for line in raw_lines
                if len(line) >= 2
            ]
            line_count += len(projected_lines)
            for idx in range(1, len(projected_lines)):
                spacing = _mean_polyline_spacing_xy(projected_lines[idx - 1], projected_lines[idx])
                if spacing is not None and math.isfinite(spacing):
                    distances.append(float(spacing))
        if not distances:
            continue
        avg_spacing = sum(distances) / float(len(distances))
        summaries.append(
            {
                "inputMissionID": int(input_mission_id),
                "averageLineSpacingM": float(avg_spacing),
                "minLineSpacingM": float(min(distances)),
                "maxLineSpacingM": float(max(distances)),
                "lineCount": int(line_count),
                "pairCount": int(len(distances)),
                "pathIds": sorted(int(value) for value in group.get("pathIds", set())),
                "aircraftIds": sorted(int(value) for value in group.get("aircraftIds", set())),
            }
        )
    return summaries


def _extract_loiter(item: dict[str, Any]) -> dict[str, Any] | None:
    loiter = (
        item.get("loiter")
        or item.get("Loiter")
        or item.get("loiterProperty")
        or item.get("LoiterProperty")
        or item.get("loiter_prop")
    )
    return loiter if isinstance(loiter, dict) else None


def _waypoint_mode(item: dict[str, Any]) -> str:
    loiter = _extract_loiter(item)
    pass_type = _coerce_int(item.get("waypointPassType") or item.get("WaypointPassType"))
    if loiter is not None or pass_type == 2:
        return "loiter"
    if pass_type == 1:
        return "fly-by"
    return "fly-over"


def _order_waypoints(raw: list[dict[str, Any]]) -> list[dict[str, Any]]:
    if not raw:
        return []
    by_id: dict[int, dict[str, Any]] = {}
    next_ids: set[int] = set()
    for wp in raw:
        if not isinstance(wp, dict):
            continue
        wid = wp.get("waypointID") or wp.get("WaypointID")
        if wid is None:
            continue
        try:
            wid_i = int(wid)
        except Exception:
            continue
        by_id[wid_i] = wp
        nxt = wp.get("nextWaypointID") or wp.get("NextWaypointID")
        if nxt is None:
            continue
        try:
            nxt_i = int(nxt)
        except Exception:
            continue
        if nxt_i > 0:
            next_ids.add(nxt_i)

    if not by_id:
        return list(raw)

    start_id = None
    for wid in by_id:
        if wid not in next_ids:
            start_id = wid
            break

    ordered: list[dict[str, Any]] = []
    visited: set[int] = set()
    if start_id is not None:
        curr = start_id
        while curr and curr in by_id and curr not in visited:
            wp = by_id[curr]
            ordered.append(wp)
            visited.add(curr)
            nxt = wp.get("nextWaypointID") or wp.get("NextWaypointID")
            try:
                curr = int(nxt)
            except Exception:
                break
            if curr == 0:
                break

    for wp in raw:
        if not isinstance(wp, dict):
            continue
        wid = wp.get("waypointID") or wp.get("WaypointID")
        if wid is None:
            ordered.append(wp)
            continue
        try:
            wid_i = int(wid)
        except Exception:
            ordered.append(wp)
            continue
        if wid_i not in visited:
            ordered.append(wp)
    return ordered


def _agent_label(aircraft_id: int) -> str:
    if 1 <= aircraft_id <= 3:
        return f"LAH{aircraft_id}"
    if 4 <= aircraft_id <= 6:
        return f"UAV{aircraft_id - 3}"
    return f"AC{aircraft_id}"


def _latest_reference_info(folder: Path) -> dict[str, Any]:
    if not folder.exists():
        return {}
    best = None
    best_ts = None
    for path in folder.glob("*.json"):
        data = _load_json(path)
        if not data:
            continue
        ts = _coerce_int(data.get("timestamp"))
        if ts is None:
            ts = _coerce_int(path.stem)
        if ts is None:
            continue
        if best is None or ts >= (best_ts or -1):
            best = data
            best_ts = ts
    return best or {}


def _mission_reference_list(reference: object, *keys: str) -> list[dict[str, Any]]:
    if not isinstance(reference, dict):
        return []
    for key in keys:
        rows = reference.get(key)
        if isinstance(rows, list):
            return [row for row in rows if isinstance(row, dict)]
    return []


def build_features_from_flight_paths(
    flight_paths: Iterable[dict[str, Any]],
    *,
    done_path_ids: set[int] | None = None,
) -> tuple[list[dict[str, Any]], dict[str, int]]:
    features: list[dict[str, Any]] = []
    agent_counts: dict[str, int] = {}
    feature_id = 1
    done_paths = done_path_ids or set()

    for entry in flight_paths:
        if not isinstance(entry, dict):
            continue
        data = entry.get("data") if isinstance(entry.get("data"), dict) else entry
        if not isinstance(data, dict):
            continue
        aircraft_id = data.get("aircraftID") or data.get("AircraftID")
        path_id = data.get("pathID") or data.get("PathID")
        try:
            aircraft_id = int(aircraft_id)
        except Exception:
            aircraft_id = -1
        try:
            path_id = int(path_id)
        except Exception:
            path_id = None

        waypoints = _extract_waypoints(data)
        if not waypoints:
            continue
        waypoints = _order_waypoints(waypoints)

        coords: list[list[float]] = []
        alts: list[float | None] = []
        wp_ids: list[int | None] = []
        wp_pass_types: list[int | None] = []
        wp_modes: list[str] = []
        alt_values: list[float] = []
        for wp in waypoints:
            if not isinstance(wp, dict):
                continue
            coord = _extract_coord(wp)
            if coord is None:
                continue
            lat, lon, alt = coord
            coords.append([float(lon), float(lat)])
            wp_id = wp.get("waypointID") or wp.get("WaypointID")
            try:
                wp_id = int(wp_id) if wp_id is not None else None
            except Exception:
                wp_id = None
            wp_ids.append(wp_id)
            wp_pass_types.append(_coerce_int(wp.get("waypointPassType") or wp.get("WaypointPassType")))
            wp_modes.append(_waypoint_mode(wp))
            alts.append(float(alt) if alt is not None else None)
            if alt is not None:
                alt_values.append(float(alt))

        if len(coords) < 1:
            continue

        agent = _agent_label(aircraft_id)
        agent_counts[agent] = agent_counts.get(agent, 0) + 1
        feature = {
            "id": feature_id,
            "agent": agent,
            "aircraftId": aircraft_id,
            "pathId": path_id,
            "isDone": bool(path_id is not None and path_id in done_paths),
            "points": len(coords),
            "coords": coords,
            "alts": alts,
            "wpIds": wp_ids,
            "wpPassTypes": wp_pass_types,
            "wpModes": wp_modes,
            "altMin": min(alt_values) if alt_values else None,
            "altMax": max(alt_values) if alt_values else None,
        }
        feature_id += 1
        features.append(feature)

    return features, agent_counts


def build_mission_plan_payload(
    mission_plan_id: int | None,
    *,
    db_root: Path | None = None,
) -> dict[str, Any]:
    if mission_plan_id is None:
        return {"ok": False, "error": "missionPlanID required"}
    if db_root is None:
        db_root = db_paths.get_active_db_root()
    base = Path(db_root)

    plan_path = base / "MissionPlan" / f"{int(mission_plan_id)}.json"
    mission_plan = _load_json(plan_path)
    if not mission_plan:
        return {
            "ok": False,
            "error": f"MissionPlan {mission_plan_id} not found",
            "planPath": str(plan_path),
        }

    input_package_id = _coerce_int(
        mission_plan.get("inputMissionPackageID")
        or mission_plan.get("InputMissionPackageID")
        or mission_plan.get("inputMissionPackageId")
    )
    input_plan, input_plan_path = _load_input_plan_for_package(base, input_package_id)
    normalize_input_mission_plan_float_fields(input_plan)
    input_plans = [input_plan] if input_plan else []

    aircraft_list = mission_plan.get("aircraftList") or []
    individual_plans: list[dict[str, Any]] = []
    path_ids: set[int] = set()
    individual_package_ids: list[int] = []
    for entry in aircraft_list:
        if not isinstance(entry, dict):
            continue
        package_id = _coerce_int(
            entry.get("individualMissionPackageID")
            or entry.get("individualMissionPackageId")
            or entry.get("IndividualMissionPackageID")
        )
        if package_id is None:
            continue
        individual_package_ids.append(package_id)
        plan = _load_json(base / "IndividualMissionPlan" / f"{int(package_id)}.json")
        if not plan:
            continue
        individual_plans.append(plan)
        for mission in plan.get("individualMissionList") or []:
            if not isinstance(mission, dict):
                continue
            pid = _coerce_int(mission.get("pathID") or mission.get("PathID"))
            if pid is not None:
                path_ids.add(pid)

    flight_paths: list[dict[str, Any]] = []
    missing_paths: list[int] = []
    for pid in sorted(path_ids):
        data = _load_json(base / "FlightPath" / f"{int(pid)}.json")
        if data:
            flight_paths.append(data)
        else:
            missing_paths.append(pid)

    mission_ref_id = _coerce_int(
        mission_plan.get("missionReferencePackageID")
        or mission_plan.get("missionReferencePackageId")
        or mission_plan.get("MissionReferencePackageID")
    )
    mission_ref_info = {}
    if mission_ref_id is not None:
        mission_ref_info = _load_json(base / "MissionReferenceInfo" / f"{int(mission_ref_id)}.json")
    if not mission_ref_info:
        mission_ref_info = _latest_reference_info(base / "MissionReferenceInfo")
    take_over_list = _mission_reference_list(
        mission_ref_info,
        "takeOverInfoList",
        "TakeOverInfoList",
    )
    hand_over_list = _mission_reference_list(
        mission_ref_info,
        "handOverInfoList",
        "HandOverInfoList",
    )
    rtb_coordinate_list = _mission_reference_list(
        mission_ref_info,
        "rtbCoordinateList",
        "RTBCoordinateList",
    )

    path_done_map: dict[int, bool] = {}
    for plan in individual_plans:
        if not isinstance(plan, dict):
            continue
        for mission in plan.get("individualMissionList") or []:
            if not isinstance(mission, dict):
                continue
            pid = _coerce_int(mission.get("pathID") or mission.get("PathID"))
            if pid is not None:
                mission_done = _coerce_bool(mission.get("isDone"), False)
                prev_done = path_done_map.get(pid)
                if prev_done is None:
                    path_done_map[pid] = mission_done
                else:
                    # If one mission referencing this path is active, keep it active.
                    path_done_map[pid] = bool(prev_done and mission_done)

    done_path_ids = {pid for pid, is_done in path_done_map.items() if is_done}
    path_mission_index = _path_input_mission_index(individual_plans)
    sweep_line_spacing_summaries = _build_sweep_line_spacing_summaries(
        flight_paths,
        path_mission_index,
    )
    sweep_line_spacing_by_input = {
        str(item["inputMissionID"]): item for item in sweep_line_spacing_summaries
    }

    features, agent_counts = build_features_from_flight_paths(
        flight_paths,
        done_path_ids=done_path_ids,
    )

    # Display-only roles the ICD cannot express (concealment points).  Absent
    # or unreadable is normal and simply means "no roles known".
    try:
        from modules.mission_planning.pipelines.lah_tactical_point_log import (
            load_tactical_points,
        )

        lah_tactical_points = load_tactical_points(base)
    except Exception:
        lah_tactical_points = {}

    payload = {
        "flightPaths": flight_paths,
        "inputMissionPlans": input_plans,
        "individualMissionPlans": individual_plans,
        "takeOverInfoList": take_over_list,
        "handOverInfoList": hand_over_list,
        "rtbCoordinateList": rtb_coordinate_list,
        "sweepLineSpacingSummaries": sweep_line_spacing_summaries,
        "sweepLineSpacingByInputMissionID": sweep_line_spacing_by_input,
        "lahTacticalPoints": lah_tactical_points,
    }

    return {
        "ok": True,
        "missionPlanID": int(mission_plan_id),
        "inputMissionPackageID": input_package_id,
        "inputMissionPlanPath": str(input_plan_path) if input_plan_path is not None else None,
        "missionReferencePackageID": mission_ref_id,
        "individualMissionPackageIDs": individual_package_ids,
        "missionPlan": mission_plan,
        "inputMissionPlans": input_plans,
        "individualMissionPlans": individual_plans,
        "flightPaths": flight_paths,
        "takeOverInfoList": take_over_list,
        "handOverInfoList": hand_over_list,
        "rtbCoordinateList": rtb_coordinate_list,
        "flightPathCount": len(flight_paths),
        "missingPathIds": missing_paths,
        "pathMissionIndex": path_mission_index,
        "lahTacticalPoints": lah_tactical_points,
        "sweepLineSpacingSummaries": sweep_line_spacing_summaries,
        "sweepLineSpacingByInputMissionID": sweep_line_spacing_by_input,
        "features": features,
        "agents": agent_counts,
        "count": len(features),
        "payload": payload,
    }
