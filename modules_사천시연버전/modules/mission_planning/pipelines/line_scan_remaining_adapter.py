from __future__ import annotations

import math
from copy import deepcopy
from typing import Any, Dict, Iterable, List, Set


def _to_int(value: object | None) -> int | None:
    try:
        return None if value is None else int(value)
    except Exception:
        return None


def _to_float(value: object | None) -> float | None:
    try:
        return None if value is None else float(value)
    except Exception:
        return None


def _normalize_coordinate(value: object | None) -> Dict[str, Any] | None:
    if not isinstance(value, dict):
        return None
    lat = _to_float(value.get("latitude") if "latitude" in value else value.get("Latitude"))
    lon = _to_float(value.get("longitude") if "longitude" in value else value.get("Longitude"))
    if lat is None or lon is None:
        return None
    if not (-90.0 <= float(lat) <= 90.0 and -180.0 <= float(lon) <= 180.0):
        return None
    coord: Dict[str, Any] = {"latitude": float(lat), "longitude": float(lon)}
    alt = _to_float(value.get("altitude") if "altitude" in value else value.get("Altitude"))
    if alt is not None:
        coord["altitude"] = int(round(float(alt)))
    return coord


def _normalize_coord_list(value: object | None, *, min_len: int = 0) -> List[Dict[str, Any]]:
    if not isinstance(value, list):
        return []
    out: List[Dict[str, Any]] = []
    for item in value:
        coord = _normalize_coordinate(item)
        if coord is not None:
            out.append(coord)
    if len(out) < int(min_len):
        return []
    return out


def _coord_step_length_m(left: Dict[str, Any], right: Dict[str, Any]) -> float:
    mean_lat_rad = math.radians((float(left["latitude"]) + float(right["latitude"])) * 0.5)
    m_per_deg_lat = 111_320.0
    m_per_deg_lon = 111_320.0 * math.cos(mean_lat_rad)
    dx = (float(right["longitude"]) - float(left["longitude"])) * m_per_deg_lon
    dy = (float(right["latitude"]) - float(left["latitude"])) * m_per_deg_lat
    return float(math.hypot(dx, dy))


def _coord_path_length_m(coords: List[Dict[str, Any]]) -> float:
    total = 0.0
    for idx in range(len(coords) - 1):
        total += _coord_step_length_m(coords[idx], coords[idx + 1])
    return float(total)


def _interpolate_coord(
    left: Dict[str, Any],
    right: Dict[str, Any],
    ratio: float,
) -> Dict[str, Any]:
    clamped = max(0.0, min(1.0, float(ratio)))
    coord: Dict[str, Any] = {
        "latitude": float(left["latitude"]) + ((float(right["latitude"]) - float(left["latitude"])) * clamped),
        "longitude": float(left["longitude"]) + ((float(right["longitude"]) - float(left["longitude"])) * clamped),
    }
    left_alt = _to_float(left.get("altitude"))
    right_alt = _to_float(right.get("altitude"))
    if left_alt is not None or right_alt is not None:
        alt0 = float(left_alt if left_alt is not None else right_alt or 0.0)
        alt1 = float(right_alt if right_alt is not None else left_alt or 0.0)
        coord["altitude"] = int(round(alt0 + ((alt1 - alt0) * clamped)))
    return coord


def _coord_at_distance(coords: List[Dict[str, Any]], distance_m: float) -> Dict[str, Any] | None:
    if len(coords) < 2:
        return None
    target = max(0.0, float(distance_m))
    walked = 0.0
    for idx in range(len(coords) - 1):
        left = coords[idx]
        right = coords[idx + 1]
        seg_len = _coord_step_length_m(left, right)
        if seg_len <= 1e-6:
            continue
        if walked + seg_len + 1e-6 >= target:
            return _interpolate_coord(left, right, (target - walked) / seg_len)
        walked += seg_len
    return dict(coords[-1])


def _slice_coord_path_by_distance(
    coords: List[Dict[str, Any]],
    start_m: float,
    end_m: float,
) -> List[Dict[str, Any]]:
    if len(coords) < 2:
        return []
    total_m = _coord_path_length_m(coords)
    if total_m <= 1e-6:
        return []
    start = max(0.0, min(float(start_m), total_m))
    end = max(0.0, min(float(end_m), total_m))
    if end <= start + 1e-6:
        return []
    out: List[Dict[str, Any]] = []
    start_coord = _coord_at_distance(coords, start)
    end_coord = _coord_at_distance(coords, end)
    if start_coord is None or end_coord is None:
        return []
    out.append(start_coord)

    walked = 0.0
    for idx in range(len(coords) - 1):
        left = coords[idx]
        right = coords[idx + 1]
        seg_len = _coord_step_length_m(left, right)
        seg_start = walked
        seg_end = walked + seg_len
        walked = seg_end
        if seg_len <= 1e-6:
            continue
        if seg_end <= start + 1e-6:
            continue
        if seg_start >= end - 1e-6:
            break
        if start < seg_end - 1e-6 and seg_end < end - 1e-6:
            if not _same_coord(out[-1], right):
                out.append(dict(right))

    if not out or not _same_coord(out[-1], end_coord):
        out.append(end_coord)
    return _dedupe_adjacent_coords(out)


def _same_coord(left: Dict[str, Any], right: Dict[str, Any]) -> bool:
    return (
        abs(float(left.get("latitude", 0.0)) - float(right.get("latitude", 0.0))) <= 1e-10
        and abs(float(left.get("longitude", 0.0)) - float(right.get("longitude", 0.0))) <= 1e-10
    )


def _dedupe_adjacent_coords(coords: Iterable[Dict[str, Any]]) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    for coord in coords:
        if not isinstance(coord, dict):
            continue
        if out and _same_coord(out[-1], coord):
            continue
        out.append(dict(coord))
    return out


def _detail_source_rows(source_detail: Dict[str, Any] | None) -> List[Dict[str, Any]]:
    detail = source_detail if isinstance(source_detail, dict) else {}
    rows: List[Dict[str, Any]] = []
    coords = _normalize_coord_list(detail.get("sourceCoordinateList"), min_len=2)
    if len(coords) >= 2:
        rows.append({"coordinateList": coords, "width": _to_float(detail.get("sourceLineWidthM"))})
        return rows
    for row in detail.get("sourceLineList") or []:
        if not isinstance(row, dict):
            continue
        coords = _normalize_coord_list(row.get("coordinateList"), min_len=2)
        if len(coords) >= 2:
            rows.append({"coordinateList": coords, "width": _to_float(row.get("width"))})
    if rows:
        return rows
    coords = _normalize_coord_list(detail.get("coordinateList"), min_len=2)
    if len(coords) >= 2:
        rows.append({"coordinateList": coords, "width": _to_float(detail.get("sourceLineWidthM"))})
        return rows
    for row in detail.get("lineList") or []:
        if not isinstance(row, dict):
            continue
        coords = _normalize_coord_list(row.get("coordinateList"), min_len=2)
        if len(coords) >= 2:
            rows.append({"coordinateList": coords, "width": _to_float(row.get("width"))})
    if rows:
        return rows
    return rows


def _entry_source_rows(line_entry: Dict[str, Any]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    coords = _normalize_coord_list(line_entry.get("sourceCoordinateList"), min_len=2)
    if len(coords) >= 2:
        rows.append({"coordinateList": coords, "width": _to_float(line_entry.get("sourceLineWidthM"))})
        return rows
    for row in line_entry.get("sourceLineList") or []:
        if not isinstance(row, dict):
            continue
        coords = _normalize_coord_list(row.get("coordinateList"), min_len=2)
        if len(coords) >= 2:
            rows.append({"coordinateList": coords, "width": _to_float(row.get("width"))})
    return rows


def _line_source_coords(
    *,
    line_row: Dict[str, Any],
    line_index: int,
    entry_source_rows: List[Dict[str, Any]],
    detail_source_rows: List[Dict[str, Any]],
    line_entry: Dict[str, Any],
) -> List[Dict[str, Any]]:
    coords = _normalize_coord_list(line_row.get("coordinateList"), min_len=2)
    if len(coords) >= 2:
        return coords
    if 0 <= int(line_index) < len(entry_source_rows):
        coords = _normalize_coord_list(entry_source_rows[int(line_index)].get("coordinateList"), min_len=2)
        if len(coords) >= 2:
            return coords
    if 0 <= int(line_index) < len(detail_source_rows):
        coords = _normalize_coord_list(detail_source_rows[int(line_index)].get("coordinateList"), min_len=2)
        if len(coords) >= 2:
            return coords
    if len(entry_source_rows) == 1:
        coords = _normalize_coord_list(entry_source_rows[0].get("coordinateList"), min_len=2)
        if len(coords) >= 2:
            return coords
    if len(detail_source_rows) == 1:
        coords = _normalize_coord_list(detail_source_rows[0].get("coordinateList"), min_len=2)
        if len(coords) >= 2:
            return coords
    return _normalize_coord_list(line_entry.get("sourceCoordinateList"), min_len=2)


def _line_width_m(
    *,
    line_row: Dict[str, Any],
    line_index: int,
    entry_source_rows: List[Dict[str, Any]],
    detail_source_rows: List[Dict[str, Any]],
    line_entry: Dict[str, Any],
    source_detail: Dict[str, Any] | None,
) -> float:
    candidates = [
        _to_float(line_row.get("width")),
        _to_float(line_entry.get("sourceLineWidthM")),
    ]
    detail = source_detail if isinstance(source_detail, dict) else {}
    candidates.append(_to_float(detail.get("sourceLineWidthM")))
    if 0 <= int(line_index) < len(entry_source_rows):
        candidates.append(_to_float(entry_source_rows[int(line_index)].get("width")))
    if 0 <= int(line_index) < len(detail_source_rows):
        candidates.append(_to_float(detail_source_rows[int(line_index)].get("width")))
    for value in candidates:
        if value is not None and value > 0.0:
            return float(value)
    return 1.0


def _remaining_interval_rows(line_row: Dict[str, Any]) -> List[Dict[str, float]]:
    intervals: List[Dict[str, float]] = []
    for item in line_row.get("remainingIntervals") or []:
        if not isinstance(item, dict):
            continue
        start_m = _to_float(item.get("startM"))
        end_m = _to_float(item.get("endM"))
        if start_m is None or end_m is None or end_m <= start_m:
            continue
        intervals.append({"startM": float(start_m), "endM": float(end_m)})
    if intervals:
        return intervals

    remaining_m = _to_float(line_row.get("remainingLengthM"))
    planned_m = _to_float(line_row.get("plannedLengthM"))
    coverage = _to_float(line_row.get("coveragePercent"))
    if (
        remaining_m is not None
        and remaining_m > 1e-6
        and planned_m is not None
        and planned_m > 1e-6
        and (coverage is None or coverage < 99.5)
    ):
        return [{"startM": 0.0, "endM": float(planned_m)}]
    return []


def _covered_interval_rows(line_row: Dict[str, Any]) -> List[Dict[str, float]]:
    intervals: List[Dict[str, float]] = []
    for item in line_row.get("coveredIntervals") or []:
        if not isinstance(item, dict):
            continue
        start_m = _to_float(item.get("startM"))
        end_m = _to_float(item.get("endM"))
        if start_m is None or end_m is None or end_m <= start_m:
            continue
        intervals.append({"startM": float(start_m), "endM": float(end_m)})
    if intervals:
        return intervals

    planned_m = _to_float(line_row.get("plannedLengthM"))
    covered_m = _to_float(line_row.get("coveredLengthM"))
    coverage = _to_float(line_row.get("coveragePercent"))
    if planned_m is not None and planned_m > 1e-6:
        if covered_m is None and coverage is not None and coverage > 0.0:
            covered_m = float(planned_m) * max(0.0, min(100.0, float(coverage))) / 100.0
        if covered_m is not None and covered_m > 1e-6:
            return [{"startM": 0.0, "endM": min(float(planned_m), float(covered_m))}]
    return []


def _merge_distance_intervals(
    intervals: Iterable[tuple[float, float]],
    *,
    length_m: float,
) -> List[tuple[float, float]]:
    length = max(0.0, float(length_m))
    normalized = []
    for start, end in intervals:
        start_v = max(0.0, min(float(start), length))
        end_v = max(0.0, min(float(end), length))
        if end_v > start_v + 1e-6:
            normalized.append((start_v, end_v))
    normalized.sort(key=lambda item: (float(item[0]), float(item[1])))
    merged: List[tuple[float, float]] = []
    for start, end in normalized:
        if not merged or start > merged[-1][1] + 1e-6:
            merged.append((float(start), float(end)))
        else:
            merged[-1] = (merged[-1][0], max(float(merged[-1][1]), float(end)))
    return merged


def _remaining_distance_intervals(
    *,
    length_m: float,
    covered_intervals: Iterable[tuple[float, float]],
) -> List[tuple[float, float]]:
    length = max(0.0, float(length_m))
    if length <= 1e-6:
        return []
    remaining: List[tuple[float, float]] = []
    cursor = 0.0
    for start, end in _merge_distance_intervals(covered_intervals, length_m=length):
        if start > cursor + 1e-6:
            remaining.append((float(cursor), float(start)))
        cursor = max(float(cursor), float(end))
    if cursor < length - 1e-6:
        remaining.append((float(cursor), float(length)))
    return remaining


def _remaining_distance_intervals_in_domains(
    *,
    length_m: float,
    domain_intervals: Iterable[tuple[float, float]],
    covered_intervals: Iterable[tuple[float, float]],
) -> List[tuple[float, float]]:
    domains = _merge_distance_intervals(domain_intervals, length_m=float(length_m))
    if not domains:
        return []
    covered = _merge_distance_intervals(covered_intervals, length_m=float(length_m))
    remaining: List[tuple[float, float]] = []
    for domain_start, domain_end in domains:
        cursor = float(domain_start)
        for covered_start, covered_end in covered:
            if covered_end <= cursor + 1e-6:
                continue
            if covered_start >= domain_end - 1e-6:
                break
            if covered_start > cursor + 1e-6:
                remaining.append((float(cursor), min(float(covered_start), float(domain_end))))
            cursor = max(float(cursor), min(float(covered_end), float(domain_end)))
            if cursor >= domain_end - 1e-6:
                break
        if cursor < domain_end - 1e-6:
            remaining.append((float(cursor), float(domain_end)))
    return remaining


def _domain_prefix_intervals(
    domain_intervals: Iterable[tuple[float, float]],
    *,
    ratio: float,
    length_m: float,
) -> List[tuple[float, float]]:
    domains = _merge_distance_intervals(domain_intervals, length_m=float(length_m))
    target_m = sum(max(0.0, float(end) - float(start)) for start, end in domains)
    target_m *= max(0.0, min(1.0, float(ratio)))
    covered: List[tuple[float, float]] = []
    remaining_target = float(target_m)
    for start, end in domains:
        if remaining_target <= 1e-6:
            break
        domain_len = max(0.0, float(end) - float(start))
        take = min(float(domain_len), float(remaining_target))
        if take > 1e-6:
            covered.append((float(start), float(start) + float(take)))
        remaining_target -= float(take)
    return covered


def _local_xy_frame(coords: List[Dict[str, Any]]) -> tuple[float, float]:
    valid = [coord for coord in coords if isinstance(coord, dict)]
    if not valid:
        return 0.0, 0.0
    ref_lat = sum(float(coord.get("latitude", 0.0)) for coord in valid) / float(len(valid))
    ref_lon = sum(float(coord.get("longitude", 0.0)) for coord in valid) / float(len(valid))
    return float(ref_lat), float(ref_lon)


def _coord_to_local_xy(
    coord: Dict[str, Any],
    *,
    ref_lat: float,
    ref_lon: float,
) -> tuple[float, float]:
    m_per_deg_lat = 111_320.0
    m_per_deg_lon = 111_320.0 * math.cos(math.radians(float(ref_lat)))
    return (
        (float(coord.get("longitude", 0.0)) - float(ref_lon)) * float(m_per_deg_lon),
        (float(coord.get("latitude", 0.0)) - float(ref_lat)) * float(m_per_deg_lat),
    )


def _project_coord_to_source_chainage_m(
    source_coords: List[Dict[str, Any]],
    coord: Dict[str, Any],
) -> tuple[float, float] | None:
    if len(source_coords) < 2 or not isinstance(coord, dict):
        return None
    ref_lat, ref_lon = _local_xy_frame([*source_coords, coord])
    source_xy = [
        _coord_to_local_xy(item, ref_lat=ref_lat, ref_lon=ref_lon)
        for item in source_coords
        if isinstance(item, dict)
    ]
    if len(source_xy) < 2:
        return None
    px, py = _coord_to_local_xy(coord, ref_lat=ref_lat, ref_lon=ref_lon)
    walked = 0.0
    best_chainage: float | None = None
    best_distance = float("inf")
    for idx in range(len(source_xy) - 1):
        x0, y0 = source_xy[idx]
        x1, y1 = source_xy[idx + 1]
        dx = float(x1) - float(x0)
        dy = float(y1) - float(y0)
        seg_len_sq = (dx * dx) + (dy * dy)
        seg_len = math.sqrt(seg_len_sq)
        if seg_len <= 1e-6:
            continue
        t = (((float(px) - float(x0)) * dx) + ((float(py) - float(y0)) * dy)) / seg_len_sq
        t = max(0.0, min(1.0, float(t)))
        qx = float(x0) + (dx * t)
        qy = float(y0) + (dy * t)
        distance = math.hypot(float(px) - qx, float(py) - qy)
        chainage = float(walked) + (float(seg_len) * t)
        if distance < best_distance:
            best_distance = float(distance)
            best_chainage = float(chainage)
        walked += float(seg_len)
    if best_chainage is None:
        return None
    return float(best_chainage), float(best_distance)


def _row_source_span_m(
    source_coords: List[Dict[str, Any]],
    row_coords: List[Dict[str, Any]],
) -> tuple[float, float, float] | None:
    if len(source_coords) < 2 or len(row_coords) < 2:
        return None
    start_projection = _project_coord_to_source_chainage_m(source_coords, row_coords[0])
    end_projection = _project_coord_to_source_chainage_m(source_coords, row_coords[-1])
    if start_projection is None or end_projection is None:
        return None
    start_m, start_dist = start_projection
    end_m, end_dist = end_projection
    if abs(float(end_m) - float(start_m)) <= 1e-6:
        return None
    return float(start_m), float(end_m), max(float(start_dist), float(end_dist))


def _map_row_interval_to_source_m(
    *,
    source_coords: List[Dict[str, Any]],
    row_coords: List[Dict[str, Any]],
    planned_m: float,
    start_m: float,
    end_m: float,
    max_projection_distance_m: float | None = None,
) -> tuple[float, float] | None:
    if planned_m <= 1e-6:
        return None
    span = _row_source_span_m(source_coords, row_coords)
    if span is None:
        return None
    source_start_m, source_end_m, distance_m = span
    if max_projection_distance_m is not None and float(distance_m) > float(max_projection_distance_m):
        return None
    ratio_start = max(0.0, min(1.0, float(start_m) / float(planned_m)))
    ratio_end = max(0.0, min(1.0, float(end_m) / float(planned_m)))
    mapped_start = float(source_start_m) + ((float(source_end_m) - float(source_start_m)) * ratio_start)
    mapped_end = float(source_start_m) + ((float(source_end_m) - float(source_start_m)) * ratio_end)
    return min(float(mapped_start), float(mapped_end)), max(float(mapped_start), float(mapped_end))


def _entry_coverage_ratio(line_entry: Dict[str, Any]) -> float | None:
    planned_m = _to_float(line_entry.get("plannedLengthM"))
    covered_m = _to_float(line_entry.get("coveredLengthM"))
    if planned_m is None or planned_m <= 1e-6:
        planned_sum = 0.0
        covered_sum = 0.0
        for row in line_entry.get("lineList") or []:
            if not isinstance(row, dict):
                continue
            row_planned = _to_float(row.get("plannedLengthM"))
            row_covered = _to_float(row.get("coveredLengthM"))
            if row_planned is None or row_planned <= 1e-6:
                continue
            planned_sum += float(row_planned)
            covered_sum += max(0.0, float(row_covered or 0.0))
        if planned_sum > 1e-6:
            return max(0.0, min(1.0, float(covered_sum) / float(planned_sum)))
    elif covered_m is not None:
        return max(0.0, min(1.0, float(covered_m) / float(planned_m)))

    progress_percent = _to_float(line_entry.get("progressPercent"))
    if progress_percent is not None:
        return max(0.0, min(1.0, float(progress_percent) / 100.0))
    return None


def _source_width_from_rows(
    *,
    source_rows: List[Dict[str, Any]],
    source_detail: Dict[str, Any] | None,
    matching_entries: List[Dict[str, Any]],
) -> float:
    candidates: List[float] = []
    detail = source_detail if isinstance(source_detail, dict) else {}
    for value in (
        _to_float(detail.get("sourceLineWidthM")),
        _to_float(detail.get("lineWidthM")),
        _to_float(detail.get("width")),
    ):
        if value is not None and value > 0.0:
            candidates.append(float(value))
    for row in source_rows or []:
        value = _to_float((row or {}).get("width"))
        if value is not None and value > 0.0:
            candidates.append(float(value))
    for entry in matching_entries or []:
        value = _to_float(entry.get("sourceLineWidthM"))
        if value is not None and value > 0.0:
            candidates.append(float(value))
        for row in entry.get("lineList") or []:
            if not isinstance(row, dict):
                continue
            value = _to_float(row.get("width"))
            if value is not None and value > 0.0:
                candidates.append(float(value))
    return max(candidates) if candidates else 1.0


def _entry_matches(
    line_entry: Dict[str, Any],
    *,
    source_plan_id: int | None,
    input_mission_id: int | None,
    aircraft_ids: Set[int] | None,
    path_ids: Set[int] | None,
) -> bool:
    if not isinstance(line_entry, dict):
        return False
    if not bool(line_entry.get("enabled", True)):
        return False
    if source_plan_id is not None:
        entry_plan = _to_int(line_entry.get("missionPlanID"))
        if entry_plan is not None and int(entry_plan) != int(source_plan_id):
            return False
    if input_mission_id is not None:
        entry_input = _to_int(line_entry.get("inputMissionID") or line_entry.get("input_id"))
        if entry_input is not None and int(entry_input) != int(input_mission_id):
            return False
    if aircraft_ids:
        entry_aircraft = _to_int(line_entry.get("aircraftID") or line_entry.get("aircraft_id"))
        if entry_aircraft is None or int(entry_aircraft) not in aircraft_ids:
            return False
    if path_ids:
        entry_path = _to_int(line_entry.get("pathID") or line_entry.get("path_id"))
        if entry_path is None or int(entry_path) not in path_ids:
            return False
    return True


def _raw_line_entry(progress_entry: Dict[str, Any]) -> Dict[str, Any]:
    if isinstance(progress_entry.get("line_scan"), dict):
        return dict(progress_entry.get("line_scan") or {})
    return dict(progress_entry)


def _coord_signature(coords: List[Dict[str, Any]]) -> tuple[tuple[int, int], ...]:
    return tuple(
        (
            int(round(float(coord.get("latitude", 0.0)) * 1_000_000_000.0)),
            int(round(float(coord.get("longitude", 0.0)) * 1_000_000_000.0)),
        )
        for coord in coords
        if isinstance(coord, dict)
    )


def build_line_scan_remaining_detail(
    sweep_progress: Dict[int, Dict[str, Any]] | None,
    *,
    source_plan_id: int | None = None,
    input_mission_id: int | None = None,
    aircraft_ids: Iterable[int] | None = None,
    path_ids: Iterable[int] | None = None,
    source_detail: Dict[str, Any] | None = None,
    min_fragment_m: float = 2.0,
) -> Dict[str, Any]:
    """Build a replan-ready LINE remaining detail on one mission centerline."""
    progress_map = sweep_progress if isinstance(sweep_progress, dict) else {}
    wanted_aircraft = {
        int(aid)
        for aid in (aircraft_ids or [])
        if _to_int(aid) is not None and int(aid) > 0
    }
    wanted_paths = {
        int(pid)
        for pid in (path_ids or [])
        if _to_int(pid) is not None and int(pid) > 0
    }
    detail_source_rows = _detail_source_rows(source_detail)
    matching_entries: List[Dict[str, Any]] = []
    contributing_aircraft: set[int] = set()
    contributing_paths: set[int] = set()
    contributing_missions: set[int] = set()
    source_rows: List[Dict[str, Any]] = list(detail_source_rows)

    for _path_id, progress_entry in sorted(progress_map.items(), key=lambda item: int(item[0])):
        if not isinstance(progress_entry, dict):
            continue
        line_entry = _raw_line_entry(progress_entry)
        if not _entry_matches(
            line_entry,
            source_plan_id=source_plan_id,
            input_mission_id=input_mission_id,
            aircraft_ids=wanted_aircraft if wanted_aircraft else None,
            path_ids=wanted_paths if wanted_paths else None,
        ):
            continue
        matching_entries.append(line_entry)
        entry_source_rows = _entry_source_rows(line_entry)
        if not source_rows and entry_source_rows:
            source_rows = list(entry_source_rows)
        aircraft_id = _to_int(line_entry.get("aircraftID") or line_entry.get("aircraft_id"))
        mission_id = _to_int(line_entry.get("missionID") or line_entry.get("mission_id"))
        path_id = _to_int(line_entry.get("pathID") or line_entry.get("path_id"))
        if aircraft_id is not None:
            contributing_aircraft.add(int(aircraft_id))
        if mission_id is not None:
            contributing_missions.add(int(mission_id))
        if path_id is not None:
            contributing_paths.add(int(path_id))

    if not matching_entries:
        return {"coordinateList": [], "lineList": [], "areaList": []}

    if not source_rows:
        for line_entry in matching_entries:
            for line_row in line_entry.get("lineList") or []:
                if not isinstance(line_row, dict):
                    continue
                coords = _normalize_coord_list(line_row.get("coordinateList"), min_len=2)
                if len(coords) >= 2:
                    source_rows.append({"coordinateList": coords, "width": _to_float(line_row.get("width"))})
                    break
            if source_rows:
                break

    source_coords = (
        _normalize_coord_list((source_rows[0] or {}).get("coordinateList"), min_len=2)
        if source_rows
        else []
    )
    source_len_m = _coord_path_length_m(source_coords)
    if len(source_coords) < 2 or source_len_m <= 1e-6:
        return {"coordinateList": [], "lineList": [], "areaList": []}

    source_width_m = _source_width_from_rows(
        source_rows=source_rows,
        source_detail=source_detail,
        matching_entries=matching_entries,
    )
    max_projection_distance_m = max(30.0, min(float(source_width_m) * 0.08, 80.0))
    restricted_domain = bool(wanted_aircraft or wanted_paths)
    domain_intervals: List[tuple[float, float]] = []
    covered_intervals: List[tuple[float, float]] = []
    fallback_ratios: List[float] = []

    for line_entry in matching_entries:
        line_rows = [row for row in (line_entry.get("lineList") or []) if isinstance(row, dict)]
        precise_added = False
        for line_row in line_rows:
            row_coords = _normalize_coord_list(line_row.get("coordinateList"), min_len=2)
            row_span = _row_source_span_m(source_coords, row_coords) if len(row_coords) >= 2 else None
            if row_span is not None and float(row_span[2]) <= float(max_projection_distance_m):
                domain_intervals.append(
                    (
                        min(float(row_span[0]), float(row_span[1])),
                        max(float(row_span[0]), float(row_span[1])),
                    )
                )
            planned_m = _to_float(line_row.get("plannedLengthM"))
            if planned_m is None or planned_m <= 1e-6:
                planned_m = _coord_path_length_m(row_coords)
            if (planned_m is None or planned_m <= 1e-6) and len(line_rows) == 1:
                planned_m = _to_float(line_entry.get("plannedLengthM")) or source_len_m
            if planned_m <= 1e-6:
                continue
            precise_rows = _covered_interval_rows(line_row)
            if precise_rows:
                for interval in precise_rows:
                    mapped = (
                        _map_row_interval_to_source_m(
                            source_coords=source_coords,
                            row_coords=row_coords,
                            planned_m=float(planned_m),
                            start_m=float(interval["startM"]),
                            end_m=float(interval["endM"]),
                            max_projection_distance_m=float(max_projection_distance_m),
                        )
                        if len(row_coords) >= 2
                        else None
                    )
                    if mapped is None and len(line_rows) == 1:
                        scale = float(source_len_m) / float(planned_m)
                        mapped = (
                            float(interval["startM"]) * scale,
                            float(interval["endM"]) * scale,
                        )
                    if mapped is None:
                        continue
                    covered_intervals.append((float(mapped[0]), float(mapped[1])))
                    precise_added = True
        if precise_added:
            continue

        ratio = _entry_coverage_ratio(line_entry)
        if ratio is not None:
            fallback_ratios.append(float(ratio))

    active_domain_intervals = (
        domain_intervals
        if restricted_domain and domain_intervals
        else [(0.0, float(source_len_m))]
    )
    if fallback_ratios:
        covered_intervals.extend(
            _domain_prefix_intervals(
                active_domain_intervals,
                ratio=max(fallback_ratios),
                length_m=float(source_len_m),
            )
        )

    remaining_intervals = _remaining_distance_intervals_in_domains(
        length_m=float(source_len_m),
        domain_intervals=active_domain_intervals,
        covered_intervals=covered_intervals,
    )
    line_rows: List[Dict[str, Any]] = []
    total_remaining_m = 0.0
    for interval_idx, (start_m, end_m) in enumerate(remaining_intervals):
        interval_len = max(0.0, float(end_m) - float(start_m))
        if interval_len < max(0.0, float(min_fragment_m)):
            continue
        coords = _slice_coord_path_by_distance(source_coords, start_m, end_m)
        if len(coords) < 2:
            continue
        total_remaining_m += float(interval_len)
        row = {
            "coordinateList": deepcopy(coords),
            "width": float(source_width_m),
            "lineIndex": 0,
            "remainingIntervalIndex": int(interval_idx),
            "remainingStartM": round(float(start_m), 3),
            "remainingEndM": round(float(end_m), 3),
            "remainingLengthM": round(float(interval_len), 3),
            "source": "line_scan_centerline_remaining_interval",
        }
        line_rows.append(row)

    if not line_rows:
        return {"coordinateList": [], "lineList": [], "areaList": []}

    covered_merged = _merge_distance_intervals(covered_intervals, length_m=float(source_len_m))
    total_covered_m = sum(max(0.0, float(end) - float(start)) for start, end in covered_merged)
    domain_merged = _merge_distance_intervals(active_domain_intervals, length_m=float(source_len_m))
    total_domain_m = sum(max(0.0, float(end) - float(start)) for start, end in domain_merged)
    coordinate_list = deepcopy(line_rows[0].get("coordinateList") or [])

    return {
        "coordinateList": coordinate_list,
        "lineList": line_rows,
        "areaList": [],
        "sourceLineWidthM": float(source_width_m),
        "sourceCoordinateList": deepcopy(source_coords),
        "sourceLineList": [
            {
                "coordinateList": deepcopy(source_coords),
                "width": float(source_width_m),
            }
        ],
        "lineRemainingSource": "line_scan_progress_centerline",
        "lineRemainingPolicy": "centerline_interval_union",
        "lineRemainingFragmentCount": len(line_rows),
        "linePlannedLengthM": round(float(source_len_m), 3),
        "lineDomainLengthM": round(float(total_domain_m), 3),
        "lineCoveredLengthM": round(float(total_covered_m), 3),
        "lineRemainingLengthM": round(float(total_remaining_m), 3),
        "lineScanContributingAircraftIDs": sorted(contributing_aircraft),
        "lineScanContributingPathIDs": sorted(contributing_paths),
        "lineScanContributingMissionIDs": sorted(contributing_missions),
    }


def load_line_scan_remaining_detail(
    *,
    source_plan_id: int | None = None,
    input_mission_id: int | None = None,
    aircraft_ids: Iterable[int] | None = None,
    path_ids: Iterable[int] | None = None,
    source_detail: Dict[str, Any] | None = None,
    min_fragment_m: float = 2.0,
) -> Dict[str, Any]:
    from modules.mission_planning.pipelines.mission_path_trim import load_sweep_progress

    return build_line_scan_remaining_detail(
        load_sweep_progress(),
        source_plan_id=source_plan_id,
        input_mission_id=input_mission_id,
        aircraft_ids=aircraft_ids,
        path_ids=path_ids,
        source_detail=source_detail,
        min_fragment_m=min_fragment_m,
    )


def _matching_line_entries_for_remaining(
    sweep_progress: Dict[int, Dict[str, Any]] | None,
    *,
    source_plan_id: int | None,
    input_mission_id: int | None,
    aircraft_ids: Iterable[int] | None,
    path_ids: Iterable[int] | None,
    allow_latest_plan_fallback: bool,
) -> tuple[List[Dict[str, Any]], bool]:
    progress_map = sweep_progress if isinstance(sweep_progress, dict) else {}
    wanted_aircraft = {
        int(aid)
        for aid in (aircraft_ids or [])
        if _to_int(aid) is not None and int(aid) > 0
    }
    wanted_paths = {
        int(pid)
        for pid in (path_ids or [])
        if _to_int(pid) is not None and int(pid) > 0
    }

    def collect(plan_id: int | None, *, ignore_paths: bool = False) -> List[Dict[str, Any]]:
        entries: List[Dict[str, Any]] = []
        for _path_id, progress_entry in sorted(progress_map.items(), key=lambda item: int(item[0])):
            if not isinstance(progress_entry, dict):
                continue
            line_entry = _raw_line_entry(progress_entry)
            if not isinstance(line_entry.get("lineList"), list):
                continue
            if plan_id is not None and _to_int(line_entry.get("missionPlanID")) is None:
                continue
            if input_mission_id is not None:
                entry_input = _to_int(line_entry.get("inputMissionID") or line_entry.get("input_id"))
                if entry_input is None:
                    continue
            if not _entry_matches(
                line_entry,
                source_plan_id=plan_id,
                input_mission_id=input_mission_id,
                aircraft_ids=wanted_aircraft if wanted_aircraft else None,
                path_ids=None if ignore_paths else (wanted_paths if wanted_paths else None),
            ):
                continue
            entries.append(line_entry)
        return entries

    exact_entries = collect(source_plan_id)
    if exact_entries or not allow_latest_plan_fallback or source_plan_id is None:
        return exact_entries, False

    fallback_entries = collect(None)
    if not fallback_entries and wanted_paths:
        fallback_entries = collect(None, ignore_paths=True)
    if not fallback_entries:
        return [], False

    plan_ids = [
        int(plan_id)
        for plan_id in (_to_int(entry.get("missionPlanID")) for entry in fallback_entries)
        if plan_id is not None and int(plan_id) > 0
    ]
    if plan_ids:
        latest_plan_id = max(plan_ids)
        fallback_entries = [
            entry
            for entry in fallback_entries
            if _to_int(entry.get("missionPlanID")) == int(latest_plan_id)
        ]
    return fallback_entries, True


def build_line_scan_aircraft_remaining_detail(
    sweep_progress: Dict[int, Dict[str, Any]] | None,
    *,
    source_plan_id: int | None = None,
    input_mission_id: int | None = None,
    aircraft_ids: Iterable[int] | None = None,
    path_ids: Iterable[int] | None = None,
    source_detail: Dict[str, Any] | None = None,
    min_fragment_m: float = 2.0,
    allow_latest_plan_fallback: bool = False,
) -> Dict[str, Any]:
    """Return row-level remaining LINE geometry for the active aircraft itself."""
    matching_entries, used_plan_fallback = _matching_line_entries_for_remaining(
        sweep_progress,
        source_plan_id=source_plan_id,
        input_mission_id=input_mission_id,
        aircraft_ids=aircraft_ids,
        path_ids=path_ids,
        allow_latest_plan_fallback=allow_latest_plan_fallback,
    )
    if not matching_entries:
        return {"coordinateList": [], "lineList": [], "areaList": []}

    detail_source_rows = _detail_source_rows(source_detail)
    line_rows: List[Dict[str, Any]] = []
    contributing_aircraft: set[int] = set()
    contributing_paths: set[int] = set()
    contributing_missions: set[int] = set()
    contributing_plans: set[int] = set()
    total_planned_m = 0.0
    total_covered_m = 0.0
    total_remaining_m = 0.0

    for line_entry in matching_entries:
        entry_source_rows = _entry_source_rows(line_entry)
        aircraft_id = _to_int(line_entry.get("aircraftID") or line_entry.get("aircraft_id"))
        mission_id = _to_int(line_entry.get("missionID") or line_entry.get("mission_id"))
        path_id = _to_int(line_entry.get("pathID") or line_entry.get("path_id"))
        plan_id = _to_int(line_entry.get("missionPlanID"))
        if aircraft_id is not None:
            contributing_aircraft.add(int(aircraft_id))
        if mission_id is not None:
            contributing_missions.add(int(mission_id))
        if path_id is not None:
            contributing_paths.add(int(path_id))
        if plan_id is not None:
            contributing_plans.add(int(plan_id))

        for row_idx, line_row in enumerate(line_entry.get("lineList") or []):
            if not isinstance(line_row, dict):
                continue
            raw_line_index = _to_int(line_row.get("lineIndex"))
            line_index = int(raw_line_index) if raw_line_index is not None else int(row_idx)
            source_coords = _line_source_coords(
                line_row=line_row,
                line_index=line_index,
                entry_source_rows=entry_source_rows,
                detail_source_rows=detail_source_rows,
                line_entry=line_entry,
            )
            if len(source_coords) < 2:
                continue
            actual_len_m = _coord_path_length_m(source_coords)
            planned_m = _to_float(line_row.get("plannedLengthM"))
            if planned_m is None or planned_m <= 1e-6:
                planned_m = actual_len_m
            if actual_len_m <= 1e-6 or planned_m <= 1e-6:
                continue
            covered_m = _to_float(line_row.get("coveredLengthM")) or 0.0
            total_planned_m += float(planned_m)
            total_covered_m += max(0.0, min(float(planned_m), float(covered_m)))

            width_m = _line_width_m(
                line_row=line_row,
                line_index=line_index,
                entry_source_rows=entry_source_rows,
                detail_source_rows=detail_source_rows,
                line_entry=line_entry,
                source_detail=source_detail,
            )
            distance_scale = float(actual_len_m) / float(planned_m)
            intervals = _remaining_interval_rows(line_row)
            for interval_idx, interval in enumerate(intervals):
                start_m = max(0.0, min(float(planned_m), float(interval["startM"])))
                end_m = max(0.0, min(float(planned_m), float(interval["endM"])))
                if end_m <= start_m:
                    continue
                actual_start_m = float(start_m) * float(distance_scale)
                actual_end_m = float(end_m) * float(distance_scale)
                fragment_len_m = max(0.0, float(actual_end_m) - float(actual_start_m))
                if fragment_len_m < max(0.0, float(min_fragment_m)):
                    continue
                coords = _slice_coord_path_by_distance(source_coords, actual_start_m, actual_end_m)
                if len(coords) < 2:
                    continue
                total_remaining_m += float(fragment_len_m)
                line_rows.append(
                    {
                        "coordinateList": deepcopy(coords),
                        "width": float(width_m),
                        "lineIndex": int(line_index),
                        "remainingIntervalIndex": int(interval_idx),
                        "remainingStartM": round(float(start_m), 3),
                        "remainingEndM": round(float(end_m), 3),
                        "remainingLengthM": round(float(fragment_len_m), 3),
                        "source": "line_scan_row_remaining_interval",
                    }
                )

    if not line_rows:
        return {
            "coordinateList": [],
            "lineList": [],
            "areaList": [],
            "lineRemainingCompleted": bool(matching_entries),
            "lineRemainingSource": "line_scan_progress_row",
            "lineRemainingPolicy": "row_remaining_intervals",
            "linePlannedLengthM": round(float(total_planned_m), 3),
            "lineCoveredLengthM": round(float(total_covered_m), 3),
            "lineScanSourcePlanFallback": bool(used_plan_fallback),
            "lineScanRequestedSourcePlanID": source_plan_id,
            "lineScanContributingAircraftIDs": sorted(contributing_aircraft),
            "lineScanContributingPathIDs": sorted(contributing_paths),
            "lineScanContributingMissionIDs": sorted(contributing_missions),
            "lineScanContributingMissionPlanIDs": sorted(contributing_plans),
        }

    coordinate_list = [
        dict(coord)
        for row in line_rows
        for coord in (row.get("coordinateList") or [])
        if isinstance(coord, dict)
    ]
    detail: Dict[str, Any] = {
        "coordinateList": _dedupe_adjacent_coords(coordinate_list),
        "lineList": line_rows,
        "areaList": [],
        "lineRemainingSource": "line_scan_progress_row",
        "lineRemainingPolicy": "row_remaining_intervals",
        "lineRemainingFragmentCount": len(line_rows),
        "linePlannedLengthM": round(float(total_planned_m), 3),
        "lineCoveredLengthM": round(float(total_covered_m), 3),
        "lineRemainingLengthM": round(float(total_remaining_m), 3),
        "lineScanContributingAircraftIDs": sorted(contributing_aircraft),
        "lineScanContributingPathIDs": sorted(contributing_paths),
        "lineScanContributingMissionIDs": sorted(contributing_missions),
        "lineScanContributingMissionPlanIDs": sorted(contributing_plans),
        "lineScanSourcePlanFallback": bool(used_plan_fallback),
        "lineScanRequestedSourcePlanID": source_plan_id,
    }
    source_rows = detail_source_rows or _entry_source_rows(matching_entries[0])
    if source_rows:
        detail["sourceLineList"] = deepcopy(source_rows)
        width = _source_width_from_rows(
            source_rows=source_rows,
            source_detail=source_detail,
            matching_entries=matching_entries,
        )
        detail["sourceLineWidthM"] = float(width)
        first_source = _normalize_coord_list(source_rows[0].get("coordinateList"), min_len=2)
        if len(first_source) >= 2:
            detail["sourceCoordinateList"] = deepcopy(first_source)
    return detail


def load_line_scan_aircraft_remaining_detail(
    *,
    source_plan_id: int | None = None,
    input_mission_id: int | None = None,
    aircraft_ids: Iterable[int] | None = None,
    path_ids: Iterable[int] | None = None,
    source_detail: Dict[str, Any] | None = None,
    min_fragment_m: float = 2.0,
    allow_latest_plan_fallback: bool = False,
) -> Dict[str, Any]:
    from modules.mission_planning.pipelines.mission_path_trim import load_sweep_progress

    return build_line_scan_aircraft_remaining_detail(
        load_sweep_progress(),
        source_plan_id=source_plan_id,
        input_mission_id=input_mission_id,
        aircraft_ids=aircraft_ids,
        path_ids=path_ids,
        source_detail=source_detail,
        min_fragment_m=min_fragment_m,
        allow_latest_plan_fallback=allow_latest_plan_fallback,
    )


def has_line_remaining_geometry(detail: object) -> bool:
    if not isinstance(detail, dict):
        return False
    line_list = detail.get("lineList")
    if isinstance(line_list, list):
        for row in line_list:
            if not isinstance(row, dict):
                continue
            if len(_normalize_coord_list(row.get("coordinateList"), min_len=2)) >= 2:
                return True
    return len(_normalize_coord_list(detail.get("coordinateList"), min_len=2)) >= 2


def apply_line_scan_remaining_to_input_mission(
    input_mission: Dict[str, Any],
    *,
    source_plan_id: int | None,
    input_mission_id: int | None = None,
    aircraft_ids: Iterable[int] | None = None,
    path_ids: Iterable[int] | None = None,
) -> tuple[Dict[str, Any], Dict[str, Any]]:
    mission = deepcopy(input_mission if isinstance(input_mission, dict) else {})
    resolved_input_id = _to_int(input_mission_id)
    if resolved_input_id is None:
        resolved_input_id = _to_int(mission.get("inputMissionID") or mission.get("input_mission_id"))
    source_detail = (
        mission.get("missionDetail")
        if isinstance(mission.get("missionDetail"), dict)
        else {}
    )
    line_detail = load_line_scan_remaining_detail(
        source_plan_id=source_plan_id,
        input_mission_id=resolved_input_id,
        aircraft_ids=aircraft_ids,
        path_ids=path_ids,
        source_detail=dict(source_detail or {}),
    )
    if not has_line_remaining_geometry(line_detail):
        return mission, {}
    mission_detail = dict(source_detail or {})
    mission_detail.update(line_detail)
    mission["missionDetail"] = mission_detail
    mission["isDone"] = False
    mission["lineTakeoverSource"] = "line_scan_progress"
    mission["lineRemainingPolicy"] = str(
        line_detail.get("lineRemainingPolicy") or "centerline_interval_union"
    )
    mission["lineTakeoverSourceAircraftIDs"] = list(line_detail.get("lineScanContributingAircraftIDs") or [])
    return mission, line_detail
