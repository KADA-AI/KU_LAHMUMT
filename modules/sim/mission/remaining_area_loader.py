from __future__ import annotations

import hashlib
import json
import math
import time
from pathlib import Path
from typing import Any

from modules.common import db_paths, mission_area_replan_store

try:
    from shapely.geometry import GeometryCollection, MultiPolygon, Polygon, mapping
    from shapely.ops import unary_union
except Exception:  # The explicit depth contract still renders without Shapely.
    GeometryCollection = MultiPolygon = Polygon = mapping = unary_union = None


SNAPSHOT_PREFIX = "mission_area_snapshot_"
_DEFAULT_LINE_WIDTH_M = 25.0
_M_PER_DEG_LAT = 111_320.0


def _to_int(value: Any) -> int | None:
    try:
        if value is None:
            return None
        return int(value)
    except Exception:
        return None


def _to_float(value: Any) -> float | None:
    try:
        if value is None:
            return None
        parsed = float(value)
    except Exception:
        return None
    if parsed != parsed:
        return None
    return parsed


def _pick(mapping: Any, *keys: str) -> Any:
    if not isinstance(mapping, dict):
        return None
    for key in keys:
        if key in mapping:
            return mapping.get(key)
    lowered = {str(key).lower(): value for key, value in mapping.items()}
    for key in keys:
        if str(key).lower() in lowered:
            return lowered[str(key).lower()]
    return None


def _agent_label(aircraft_id: int | None) -> str | None:
    if aircraft_id is None:
        return None
    if 1 <= int(aircraft_id) <= 3:
        return f"LAH{int(aircraft_id)}"
    if 4 <= int(aircraft_id) <= 6:
        return f"UAV{int(aircraft_id) - 3}"
    return f"AC{int(aircraft_id)}"


def _snapshot_dir() -> Path:
    return db_paths.get_db_subpath("DSS_Internal", "mission_area_replan")


def _snapshot_plan_id(path: Path) -> int | None:
    try:
        return _to_int(str(path.stem).rsplit("_", 1)[-1])
    except Exception:
        return None


def _latest_snapshot_path() -> Path | None:
    try:
        candidates = [
            path
            for path in _snapshot_dir().glob(f"{SNAPSHOT_PREFIX}*.json")
            if _snapshot_plan_id(path) is not None
        ]
    except Exception:
        candidates = []
    if not candidates:
        return None
    return max(
        candidates,
        key=lambda path: (
            path.stat().st_mtime if path.exists() else 0.0,
            path.name,
        ),
    )


def _load_snapshot(mission_plan_id: int | None) -> tuple[dict[str, Any] | None, Path | None, float | None]:
    if mission_plan_id is None:
        return None, None, None
    path = _snapshot_dir() / f"{SNAPSHOT_PREFIX}{int(mission_plan_id)}.json"
    if not path.exists():
        return None, path, None
    data = mission_area_replan_store.load_snapshot(int(mission_plan_id))
    try:
        mtime = float(path.stat().st_mtime)
    except Exception:
        mtime = None
    return data if isinstance(data, dict) else None, path, mtime


def _coordinate_ring(coords: Any) -> list[list[float]]:
    if not isinstance(coords, list):
        return []
    ring: list[list[float]] = []
    for coord in coords:
        if not isinstance(coord, dict):
            continue
        lat = _to_float(_pick(coord, "latitude", "Latitude", "lat"))
        lon = _to_float(_pick(coord, "longitude", "Longitude", "lon", "lng"))
        if lat is None or lon is None:
            continue
        if not (-90.0 <= float(lat) <= 90.0 and -180.0 <= float(lon) <= 180.0):
            continue
        ring.append([float(lon), float(lat)])
    if len(ring) < 3:
        return []
    first = ring[0]
    last = ring[-1]
    if first[0] != last[0] or first[1] != last[1]:
        ring.append([first[0], first[1]])
    return ring if len(ring) >= 4 else []


def _coordinate_path(coords: Any) -> list[list[float]]:
    """열린 폴리라인([lon, lat] 목록, 2점 이상). 연속 중복점은 제거."""
    if not isinstance(coords, list):
        return []
    path: list[list[float]] = []
    for coord in coords:
        if not isinstance(coord, dict):
            continue
        lat = _to_float(_pick(coord, "latitude", "Latitude", "lat"))
        lon = _to_float(_pick(coord, "longitude", "Longitude", "lon", "lng"))
        if lat is None or lon is None:
            continue
        if not (-90.0 <= float(lat) <= 90.0 and -180.0 <= float(lon) <= 180.0):
            continue
        point = [float(lon), float(lat)]
        if path and abs(path[-1][0] - point[0]) <= 1e-9 and abs(path[-1][1] - point[1]) <= 1e-9:
            continue
        path.append(point)
    return path if len(path) >= 2 else []


def _line_ribbon_ring(path_lonlat: list[list[float]], width_m: float) -> list[list[float]]:
    """잔여 line 구간을 폭 width_m의 회랑(리본) 폴리곤 링으로 변환.

    area 잔여영역과 동일한 투명 폴리곤 스타일로 렌더링하기 위한 시각화 용도로,
    지역 등장방형 근사(위도 기준 m/deg) + miter 클램프 오프셋을 사용한다.
    """
    if not isinstance(path_lonlat, list) or len(path_lonlat) < 2:
        return []
    half_width_m = float(width_m) * 0.5
    if half_width_m <= 0.0:
        return []
    ref_lat = sum(float(pt[1]) for pt in path_lonlat) / float(len(path_lonlat))
    m_per_deg_lon = max(1e-6, _M_PER_DEG_LAT * math.cos(math.radians(ref_lat)))

    points_xy: list[tuple[float, float]] = []
    for lon, lat in path_lonlat:
        candidate = (float(lon) * m_per_deg_lon, float(lat) * _M_PER_DEG_LAT)
        if points_xy and math.hypot(
            candidate[0] - points_xy[-1][0],
            candidate[1] - points_xy[-1][1],
        ) <= 0.05:
            continue
        points_xy.append(candidate)
    if len(points_xy) < 2:
        return []

    segment_normals: list[tuple[float, float]] = []
    for idx in range(1, len(points_xy)):
        dx = points_xy[idx][0] - points_xy[idx - 1][0]
        dy = points_xy[idx][1] - points_xy[idx - 1][1]
        length = math.hypot(dx, dy)
        segment_normals.append((-dy / length, dx / length))

    left_xy: list[tuple[float, float]] = []
    right_xy: list[tuple[float, float]] = []
    for idx, point in enumerate(points_xy):
        if idx == 0:
            normal = segment_normals[0]
        elif idx == len(points_xy) - 1:
            normal = segment_normals[-1]
        else:
            nx = segment_normals[idx - 1][0] + segment_normals[idx][0]
            ny = segment_normals[idx - 1][1] + segment_normals[idx][1]
            length = math.hypot(nx, ny)
            if length <= 1e-9:
                normal = segment_normals[idx]
            else:
                unit = (nx / length, ny / length)
                # miter 길이를 급커브에서 2.5배로 제한해 뾰족한 스파이크를 막는다.
                cos_half = unit[0] * segment_normals[idx][0] + unit[1] * segment_normals[idx][1]
                scale = 1.0 / max(float(cos_half), 0.4)
                normal = (unit[0] * scale, unit[1] * scale)
        left_xy.append((point[0] + normal[0] * half_width_m, point[1] + normal[1] * half_width_m))
        right_xy.append((point[0] - normal[0] * half_width_m, point[1] - normal[1] * half_width_m))

    ring = [
        [x / m_per_deg_lon, y / _M_PER_DEG_LAT]
        for x, y in (left_xy + right_xy[::-1])
    ]
    ring.append(list(ring[0]))
    return ring if len(ring) >= 4 else []


def _line_blocks(mission: dict[str, Any]) -> list[tuple[list[list[float]], float]]:
    """잔여 lineList 블록을 (경로, 폭[m]) 목록으로 정규화."""
    detail = _pick(mission, "remainingDetail", "remaining_detail")
    if not isinstance(detail, dict):
        return []
    fallback_width = _to_float(_pick(mission, "sourceLineWidthM", "source_line_width_m"))
    if fallback_width is None or fallback_width <= 0.0:
        fallback_width = _DEFAULT_LINE_WIDTH_M

    blocks: list[tuple[list[list[float]], float]] = []
    line_list = _pick(detail, "lineList", "LineList")
    if isinstance(line_list, list):
        for line in line_list:
            if not isinstance(line, dict):
                continue
            path = _coordinate_path(_pick(line, "coordinateList", "CoordinateList"))
            if not path:
                continue
            width = _to_float(_pick(line, "width", "Width"))
            if width is None or width <= 0.0:
                width = float(fallback_width)
            blocks.append((path, float(width)))
    if not blocks:
        path = _coordinate_path(_pick(detail, "coordinateList", "CoordinateList"))
        if path:
            blocks.append((path, float(fallback_width)))
    return blocks


def _area_polygons(remaining_detail: Any) -> list[list[list[list[float]]]]:
    if not isinstance(remaining_detail, dict):
        return []

    polygons: list[list[list[list[float]]]] = []
    area_list = _pick(remaining_detail, "areaList", "AreaList")
    if isinstance(area_list, list):
        outer_rings: list[list[list[float]]] = []
        hole_rings: list[list[list[float]]] = []
        for area in area_list:
            if not isinstance(area, dict):
                continue
            ring = _coordinate_ring(_pick(area, "coordinateList", "CoordinateList"))
            if not ring:
                continue
            if bool(_pick(area, "isHole", "IsHole")):
                hole_rings.append(ring)
            else:
                outer_rings.append(ring)
        holes_by_outer: list[list[list[list[float]]]] = [[] for _outer in outer_rings]
        for hole in hole_rings:
            # A snapshot may contain multiple disjoint outer areas.  GeoJSON
            # holes belong only to their containing polygon; attaching every
            # hole to every outer produced the visible "empty middle" artifact
            # and invalid rings for unrelated regions.
            sample = hole[0]
            containing = [
                index
                for index, outer in enumerate(outer_rings)
                if _point_in_ring(float(sample[0]), float(sample[1]), outer)
            ]
            if containing:
                owner_index = min(containing, key=lambda index: _ring_area_degrees(outer_rings[index]))
                holes_by_outer[owner_index].append(hole)
        for outer_index, outer in enumerate(outer_rings):
            polygons.append([outer, *holes_by_outer[outer_index]])

    segment_list = _pick(remaining_detail, "areaSegmentList", "AreaSegmentList")
    if isinstance(segment_list, list):
        for segment in segment_list:
            if not isinstance(segment, dict):
                continue
            ring = _coordinate_ring(_pick(segment, "coordinateList", "CoordinateList"))
            if ring:
                polygons.append([ring])

    if not polygons:
        ring = _coordinate_ring(_pick(remaining_detail, "coordinateList", "CoordinateList"))
        if ring:
            polygons.append([ring])
    return polygons


def _point_in_ring(x: float, y: float, ring: list[list[float]]) -> bool:
    """Return True for a point inside or on the boundary of a closed ring."""

    if not isinstance(ring, list) or len(ring) < 4:
        return False
    inside = False
    for index in range(1, len(ring)):
        x1, y1 = float(ring[index - 1][0]), float(ring[index - 1][1])
        x2, y2 = float(ring[index][0]), float(ring[index][1])
        cross = (x - x1) * (y2 - y1) - (y - y1) * (x2 - x1)
        if abs(cross) <= 1e-12 and min(x1, x2) - 1e-12 <= x <= max(x1, x2) + 1e-12 and min(
            y1, y2
        ) - 1e-12 <= y <= max(y1, y2) + 1e-12:
            return True
        if (y1 > y) != (y2 > y):
            crossing_x = x1 + ((y - y1) * (x2 - x1) / (y2 - y1))
            if crossing_x >= x:
                inside = not inside
    return inside


def _ring_area_degrees(ring: list[list[float]]) -> float:
    if not isinstance(ring, list) or len(ring) < 4:
        return 0.0
    area = 0.0
    for idx in range(1, len(ring)):
        x1, y1 = ring[idx - 1]
        x2, y2 = ring[idx]
        area += (float(x1) * float(y2)) - (float(x2) * float(y1))
    return abs(float(area)) * 0.5


def _polygon_area_degrees(polygon: list[list[list[float]]]) -> float:
    if not polygon:
        return 0.0
    outer_area = _ring_area_degrees(polygon[0])
    hole_area = sum(_ring_area_degrees(ring) for ring in polygon[1:])
    return max(0.0, float(outer_area) - float(hole_area))


def _polygons_area_degrees(polygons: list[list[list[list[float]]]]) -> float:
    return sum(_polygon_area_degrees(polygon) for polygon in polygons or [])


def _normalize_coverage_depth(value: Any) -> int | None:
    depth = _to_int(value)
    return int(depth) if depth in {0, 1, 2} else None


def _coverage_depth_status(depth: int) -> str:
    return {0: "needs_two", 1: "needs_one", 2: "complete"}.get(int(depth), "unknown")


def _coverage_depth_label(depth: int) -> str:
    return {0: "0/2 captured", 1: "1/2 captured", 2: "2/2 complete"}.get(
        int(depth),
        "unknown",
    )


def _normalized_int_list(value: Any) -> list[int]:
    values = value if isinstance(value, list) else [value]
    return sorted({int(parsed) for parsed in (_to_int(item) for item in values) if parsed is not None})


def _normalized_pass_list(value: Any) -> list[str]:
    values = value if isinstance(value, list) else [value]
    return sorted(
        {
            normalized
            for normalized in (str(item or "").strip().lower() for item in values)
            if normalized in {"forward", "reverse"}
        }
    )


def _depth_attribution(
    mission: dict[str, Any],
    depth_detail: dict[str, Any] | None,
    *,
    fallback_passes: list[str] | None = None,
) -> tuple[list[int], list[str]]:
    row = depth_detail if isinstance(depth_detail, dict) else {}
    raw_aircraft_ids = _pick(
        row,
        "activeAircraftIDs",
        "activeAircraftIds",
        "aircraftIDs",
        "aircraftIds",
    )
    aircraft_ids = _normalized_int_list(raw_aircraft_ids)
    if raw_aircraft_ids is None:
        aircraft_ids = _normalized_int_list(_pick(mission, "aircraftIDs", "aircraftIds"))
    raw_passes = _pick(
        row,
        "activeCoveragePasses",
        "coveragePasses",
        "activeCoveragePass",
    )
    passes = _normalized_pass_list(raw_passes)
    if raw_passes is None:
        passes = _normalized_pass_list(fallback_passes or [])
    if raw_passes is None and not passes:
        passes = _normalized_pass_list(
            _pick(mission, "activeCoveragePass", "active_coverage_pass")
        )
    return aircraft_ids, passes


def _depth_feature_properties(
    *,
    snapshot_plan_id: int | None,
    mission: dict[str, Any],
    depth: int,
    area_index: int,
    geometry_source: str,
    depth_detail: dict[str, Any] | None = None,
    fallback_passes: list[str] | None = None,
) -> dict[str, Any]:
    aircraft_ids, passes = _depth_attribution(
        mission,
        depth_detail,
        fallback_passes=fallback_passes,
    )
    agents = [agent for agent in (_agent_label(item) for item in aircraft_ids) if agent]
    remaining_count = 2 - int(depth)
    row = depth_detail if isinstance(depth_detail, dict) else {}
    explicit_remaining = _to_int(
        _pick(row, "remainingCaptureCount", "remaining_capture_count")
    )
    if explicit_remaining in {0, 1, 2}:
        remaining_count = int(explicit_remaining)
    properties = _feature_properties(
        snapshot_plan_id=snapshot_plan_id,
        mission=mission,
        owner=None,
        geometry_source=geometry_source,
        area_index=area_index,
    )
    properties.update(
        {
            "visualizationRole": "coverageDepth",
            "coverageDepth": int(depth),
            "remainingCaptureCount": int(remaining_count),
            "requiredCoverageDepth": _to_int(
                _pick(mission, "requiredCoverageDepth", "required_coverage_depth")
            )
            or 2,
            "coverageDepthStatus": _coverage_depth_status(int(depth)),
            "coverageDepthLabel": _coverage_depth_label(int(depth)),
            # GeoJSON properties stay scalar for MapLibre compatibility.  The
            # comma-delimited identity is also convenient in map popups.
            "activeAircraftIDs": ",".join(str(item) for item in aircraft_ids),
            "activeAgents": ",".join(agents),
            "activeCoveragePasses": ",".join(passes),
            "attributionCount": len(aircraft_ids),
            "coveragePercent": _to_int(
                _pick(row, "coveragePercent", "coverage_percent")
            ),
        }
    )
    area_m2 = _to_float(
        _pick(row, "areaM2", "remainingAreaM2", "remaining_area_m2")
    )
    if area_m2 is not None:
        properties["remainingAreaM2"] = float(area_m2)
        properties["areaM2"] = float(area_m2)
    properties["isDone"] = 1 if int(depth) >= 2 else 0
    return properties


def _polygon_geometry(polygons: list[list[list[list[float]]]]) -> Any:
    if Polygon is None or unary_union is None:
        return None
    shapes = []
    for polygon in polygons or []:
        if not polygon:
            continue
        try:
            candidate = Polygon(polygon[0], polygon[1:])
            if not candidate.is_valid:
                candidate = candidate.buffer(0)
            if not candidate.is_empty:
                shapes.append(candidate)
        except Exception:
            continue
    if not shapes:
        return None
    try:
        return unary_union(shapes)
    except Exception:
        return None


def _mapped_polygon_features(geometry: Any) -> list[dict[str, Any]]:
    if geometry is None or getattr(geometry, "is_empty", True) or mapping is None:
        return []
    polygonal_geometry = geometry
    if str(getattr(geometry, "geom_type", "")) == "GeometryCollection":
        polygonal_parts = [
            part
            for part in getattr(geometry, "geoms", ())
            if str(getattr(part, "geom_type", "")) in {"Polygon", "MultiPolygon"}
            and not bool(getattr(part, "is_empty", True))
        ]
        if not polygonal_parts or unary_union is None:
            return []
        try:
            polygonal_geometry = unary_union(polygonal_parts)
        except Exception:
            return []
    try:
        mapped = mapping(polygonal_geometry)
    except Exception:
        return []
    if mapped.get("type") in {"Polygon", "MultiPolygon"}:
        return [mapped]
    return []


def _mapped_polygon_union(
    polygons: list[list[list[list[float]]]],
) -> dict[str, Any] | None:
    """Return one logical GeoJSON geometry for any number of polygon pieces.

    Shapely dissolves overlaps when available.  The direct MultiPolygon fallback
    keeps the API contract (one feature per logical band/owner-pass) even in a
    lightweight SIM bundle without Shapely.
    """

    usable = [polygon for polygon in polygons or [] if polygon]
    if not usable:
        return None
    mapped = _mapped_polygon_features(_polygon_geometry(usable))
    if mapped:
        return mapped[0]
    if len(usable) == 1:
        return {"type": "Polygon", "coordinates": usable[0]}
    return {"type": "MultiPolygon", "coordinates": usable}


def _mapped_geometry_component_count(geometry: dict[str, Any] | None) -> int:
    if not isinstance(geometry, dict):
        return 0
    geometry_type = str(geometry.get("type") or "")
    if geometry_type == "Polygon":
        return 1
    if geometry_type == "MultiPolygon":
        return len([item for item in geometry.get("coordinates") or [] if item])
    return 0


def _aggregate_depth_detail(
    mission: dict[str, Any],
    details: list[dict[str, Any]],
    *,
    depth: int,
) -> dict[str, Any]:
    aggregate = dict(details[0]) if details else {}
    aircraft_ids: set[int] = set()
    passes: set[str] = set()
    area_values: list[float] = []
    coverage_values: list[int] = []
    for detail in details:
        detail_aircraft, detail_passes = _depth_attribution(mission, detail)
        aircraft_ids.update(int(item) for item in detail_aircraft)
        passes.update(str(item) for item in detail_passes)
        area_m2 = _to_float(
            _pick(detail, "areaM2", "remainingAreaM2", "remaining_area_m2")
        )
        if area_m2 is not None:
            area_values.append(max(0.0, float(area_m2)))
        coverage = _to_int(_pick(detail, "coveragePercent", "coverage_percent"))
        if coverage is not None:
            coverage_values.append(max(0, min(100, int(coverage))))
    aggregate["coverageDepth"] = int(depth)
    aggregate["remainingCaptureCount"] = max(0, 2 - int(depth))
    aggregate["activeAircraftIDs"] = sorted(aircraft_ids)
    aggregate["activeCoveragePasses"] = sorted(passes)
    if area_values:
        aggregate["areaM2"] = float(sum(area_values))
        aggregate["remainingAreaM2"] = float(sum(area_values))
    if coverage_values:
        aggregate["coveragePercent"] = max(coverage_values)
    return aggregate


def _feature_properties(
    *,
    snapshot_plan_id: int | None,
    mission: dict[str, Any],
    owner: dict[str, Any] | None,
    geometry_source: str,
    area_index: int,
    mission_kind: str = "area",
    width_m: float | None = None,
    coverage_pass_detail: dict[str, Any] | None = None,
) -> dict[str, Any]:
    row = owner if isinstance(owner, dict) else mission
    aircraft_id = _to_int(_pick(row, "aircraftID", "aircraftId", "aircraft_id"))
    if aircraft_id is None:
        # mission 레벨 스냅샷은 단일 aircraftID 대신 aircraftIDs 목록을 갖는다 —
        # 담당 기체가 하나뿐이면 그 색으로 렌더링한다.
        id_list = _pick(row, "aircraftIDs", "aircraftIds", "aircraft_ids")
        if isinstance(id_list, list):
            unique_ids = {_to_int(v) for v in id_list} - {None}
            if len(unique_ids) == 1:
                aircraft_id = int(next(iter(unique_ids)))
    agent = _agent_label(aircraft_id)
    input_id = _to_int(_pick(row, "inputMissionID", "inputMissionId", "input_id"))
    individual_id = _to_int(_pick(row, "individualMissionID", "individualMissionId", "missionID"))
    remaining_area_m2 = _to_float(_pick(row, "remainingAreaM2", "remaining_area_m2"))
    if remaining_area_m2 is None:
        remaining_area_m2 = _to_float(_pick(mission, "remainingAreaM2", "remaining_area_m2"))
    progress_points = _to_int(_pick(mission, "sweepProgressPoints", "sweep_progress_points"))
    progress_total = _to_int(_pick(mission, "sweepPointCount", "sweep_point_count"))
    boundary_index = _to_int(_pick(mission, "mappedBoundaryLineIndex", "mapped_boundary_line_index"))
    properties = {
        "missionPlanID": _to_int(_pick(mission, "missionPlanID")) or snapshot_plan_id,
        "inputMissionID": input_id,
        "individualMissionID": individual_id,
        "aircraftID": aircraft_id,
        "agent": agent,
        "geometrySource": geometry_source,
        "areaIndex": int(area_index),
        "missionKind": str(mission_kind or "area"),
        "remainingAreaM2": remaining_area_m2,
        "coveragePercent": _to_int(_pick(mission, "coveragePercent", "coverage_percent")),
        "isDone": 1 if bool(_pick(row, "isDone", "done")) or bool(_pick(mission, "isDone", "done")) else 0,
        "progressSource": _pick(mission, "progressSource", "areaProgressSource"),
        "sweepProgressPoints": progress_points,
        "sweepPointCount": progress_total,
        "mappedBoundaryLineIndex": boundary_index,
    }
    if isinstance(coverage_pass_detail, dict):
        coverage_pass = str(
            _pick(coverage_pass_detail, "coveragePass", "coverage_pass") or ""
        ).strip().lower()
        pass_done = bool(_pick(coverage_pass_detail, "isDone", "done"))
        pass_progress = _to_int(
            _pick(coverage_pass_detail, "coveragePercent", "coverage_percent")
        )
        if pass_progress is None:
            pass_progress = 100 if pass_done else 0
        pass_progress = max(0, min(100, int(pass_progress)))
        active_pass = str(
            _pick(mission, "activeCoveragePass", "active_coverage_pass") or ""
        ).strip().lower()
        owner_is_current = bool(
            _pick(owner, "isCurrent", "is_current")
        ) if isinstance(owner, dict) else False
        pass_status = (
            "completed"
            if pass_done
            else "active"
            if coverage_pass == active_pass or pass_progress > 0 or owner_is_current
            else "planned"
        )
        properties.update(
            {
                "visualizationRole": "coveragePassAttribution",
                "coveragePassRequirementMode": "all_passes_required",
                "contributesToCoverageCompletion": 1,
                "coveragePass": coverage_pass or None,
                "passIndex": _to_int(_pick(coverage_pass_detail, "passIndex", "pass_index")),
                "coveragePassStatus": pass_status,
                "coveragePassProgress": int(pass_progress),
                "status": pass_status,
                "progress": int(pass_progress),
                "plannedAreaM2": _to_float(
                    _pick(coverage_pass_detail, "plannedAreaM2", "planned_area_m2")
                ),
                "coveredAreaM2": _to_float(
                    _pick(coverage_pass_detail, "coveredAreaM2", "covered_area_m2")
                ),
                "activeCoveragePasses": coverage_pass or "",
            }
        )
        pass_remaining = _to_float(
            _pick(coverage_pass_detail, "remainingAreaM2", "remaining_area_m2")
        )
        if pass_remaining is not None:
            properties["remainingAreaM2"] = float(pass_remaining)
        properties["coveragePercent"] = int(pass_progress)
        properties["isDone"] = 1 if pass_done else 0
    if width_m is not None:
        properties["widthM"] = float(width_m)
    return properties


def _coverage_pass_name(detail: Any) -> str | None:
    pass_name = str(
        _pick(
            detail,
            "coveragePass",
            "coverage_pass",
            "areaAssignedCoveragePass",
            "area_assigned_coverage_pass",
            "areaCoveragePass",
            "area_coverage_pass",
        )
        or ""
    ).strip().lower()
    return pass_name if pass_name in {"forward", "reverse"} else None


def _coverage_pass_groups(mission: dict[str, Any]) -> list[dict[str, Any]]:
    """Group pass ledgers by logical display identity: aircraft + OUT/RETURN."""

    groups: dict[tuple[int | None, str], dict[str, Any]] = {}
    ownership_rows = [
        row
        for row in (
            _pick(mission, "areaOwnershipDetails", "areaOwnershipDetailList") or []
        )
        if isinstance(row, dict)
    ]
    for owner in ownership_rows:
        aircraft_id = _to_int(_pick(owner, "aircraftID", "aircraftId", "aircraft_id"))
        owner_pass_details = [
            detail
            for detail in (
                _pick(owner, "coveragePassDetails", "coverage_pass_details") or []
            )
            if isinstance(detail, dict) and _coverage_pass_name(detail) is not None
        ]
        if not owner_pass_details:
            direct_pass_name = _coverage_pass_name(owner)
            if direct_pass_name is not None:
                owner_pass_details = [
                    {
                        "coveragePass": direct_pass_name,
                        "passIndex": 1 if direct_pass_name == "forward" else 2,
                        "plannedAreaM2": _pick(
                            owner,
                            "plannedAreaM2",
                            "planned_area_m2",
                        ),
                        "coveredAreaM2": _pick(
                            owner,
                            "coveredAreaM2",
                            "covered_area_m2",
                        ),
                        "remainingAreaM2": _pick(
                            owner,
                            "remainingAreaM2",
                            "remaining_area_m2",
                        ),
                        "coveragePercent": _pick(
                            owner,
                            "coveragePercent",
                            "coverage_percent",
                        ),
                        "isDone": bool(_pick(owner, "isDone", "done")),
                        "remainingDetail": _pick(
                            owner,
                            "remainingDetail",
                            "remaining_detail",
                        ),
                    }
                ]
        for detail in owner_pass_details:
            pass_name = _coverage_pass_name(detail)
            if pass_name is None:
                continue
            group = groups.setdefault(
                (aircraft_id, pass_name),
                {
                    "aircraftID": aircraft_id,
                    "coveragePass": pass_name,
                    "owners": [],
                    "details": [],
                },
            )
            group["owners"].append(owner)
            group["details"].append(detail)

    # New snapshots always have ownership rows.  Mission-level rows remain the
    # compatibility path for old/single-aircraft snapshots.
    if not groups:
        aircraft_ids = _normalized_int_list(
            _pick(mission, "aircraftIDs", "aircraftIds", "aircraft_ids")
        )
        aircraft_id = aircraft_ids[0] if len(aircraft_ids) == 1 else None
        for detail in (
            _pick(mission, "coveragePassDetails", "coverage_pass_details") or []
        ):
            if not isinstance(detail, dict):
                continue
            pass_name = _coverage_pass_name(detail)
            if pass_name is None:
                continue
            group = groups.setdefault(
                (aircraft_id, pass_name),
                {
                    "aircraftID": aircraft_id,
                    "coveragePass": pass_name,
                    "owners": [],
                    "details": [],
                },
            )
            group["details"].append(detail)

    pass_order = {"forward": 1, "reverse": 2}
    return sorted(
        groups.values(),
        key=lambda group: (
            int(group.get("aircraftID") or 0),
            pass_order.get(str(group.get("coveragePass") or ""), 99),
        ),
    )


def _aggregate_pass_detail(group: dict[str, Any]) -> dict[str, Any]:
    details = [row for row in group.get("details") or [] if isinstance(row, dict)]
    aggregate = dict(details[0]) if details else {}
    pass_name = str(group.get("coveragePass") or "")
    aggregate["coveragePass"] = pass_name
    aggregate["isDone"] = bool(details) and all(
        bool(_pick(detail, "isDone", "done")) for detail in details
    )
    pass_indexes = [
        value
        for value in (
            _to_int(_pick(detail, "passIndex", "pass_index")) for detail in details
        )
        if value is not None
    ]
    if pass_indexes:
        aggregate["passIndex"] = min(pass_indexes)

    def _sum_values(*keys: str) -> float | None:
        values = [
            value
            for value in (_to_float(_pick(detail, *keys)) for detail in details)
            if value is not None
        ]
        return float(sum(max(0.0, float(value)) for value in values)) if values else None

    planned = _sum_values("plannedAreaM2", "planned_area_m2")
    covered = _sum_values("coveredAreaM2", "covered_area_m2")
    remaining = _sum_values("remainingAreaM2", "remaining_area_m2")
    if planned is not None:
        aggregate["plannedAreaM2"] = planned
    if covered is not None:
        aggregate["coveredAreaM2"] = covered
    if remaining is not None:
        aggregate["remainingAreaM2"] = remaining
    if planned is not None and planned > 1e-9 and covered is not None:
        aggregate["coveragePercent"] = max(
            0,
            min(100, int(round((covered / planned) * 100.0))),
        )
    else:
        progress_values = [
            value
            for value in (
                _to_int(_pick(detail, "coveragePercent", "coverage_percent"))
                for detail in details
            )
            if value is not None
        ]
        if progress_values:
            aggregate["coveragePercent"] = max(
                0,
                min(100, int(round(sum(progress_values) / len(progress_values)))),
            )
    return aggregate


def _area_coverage_is_complete(mission: dict[str, Any]) -> bool:
    """Return whether pending depth bands are no longer actionable.

    The monitor can close an Area mission while its final tolerance-sized
    ``coverageDepthDetails`` fragments are still present in the portable
    snapshot.  Those fragments are useful audit evidence, but must not be
    rendered as fresh NEED 1/NEED 2 work.  Prefer the explicit mission result;
    the pass-owner fallback covers a short publication window where every
    OUT/RETURN owner is complete before the mission aggregate is refreshed.
    """

    if bool(_pick(mission, "isDone", "done")):
        return True
    depth_contract_present = bool(
        _pick(
            mission,
            "areaCoverageDepthContractVersion",
            "area_coverage_depth_contract_version",
        )
        is not None
        or isinstance(
            _pick(mission, "coverageDepthDetails", "coverage_depth_details"),
            list,
        )
    )
    depth_satisfied = _pick(
        mission,
        "coverageDepthSatisfied",
        "coverage_depth_satisfied",
    )
    if depth_contract_present and bool(depth_satisfied):
        return True
    if depth_contract_present:
        # Route completion is not capture completion.  When an explicit spatial
        # ledger still reports pending depth, retain it even if all pass owners
        # have reached their last waypoint.  This is what lets a real interior
        # miss survive into the next replan.  Explicitly completed legacy
        # snapshots are handled by the isDone branch above.
        return False

    pass_groups = _coverage_pass_groups(mission)
    required_passes = {"forward", "reverse"}
    published_passes = {
        str(group.get("coveragePass") or "").strip().lower()
        for group in pass_groups
    }
    if not required_passes.issubset(published_passes):
        return False
    expected_aircraft_ids = set(
        _normalized_int_list(
            _pick(mission, "aircraftIDs", "aircraftIds", "aircraft_ids")
        )
    )
    attributed_group_keys = {
        (_to_int(group.get("aircraftID")), str(group.get("coveragePass") or ""))
        for group in pass_groups
        if _to_int(group.get("aircraftID")) is not None
    }
    if expected_aircraft_ids and attributed_group_keys:
        required_group_keys = {
            (int(aircraft_id), pass_name)
            for aircraft_id in expected_aircraft_ids
            for pass_name in required_passes
        }
        if not required_group_keys.issubset(attributed_group_keys):
            return False
    return bool(pass_groups) and all(
        bool(_pick(_aggregate_pass_detail(group), "isDone", "done"))
        for group in pass_groups
    )


def _coverage_pass_features(
    *,
    snapshot_plan_id: int | None,
    mission: dict[str, Any],
) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    for area_index, group in enumerate(_coverage_pass_groups(mission), start=1):
        pass_name = str(group.get("coveragePass") or "")
        details = [row for row in group.get("details") or [] if isinstance(row, dict)]
        owners = [row for row in group.get("owners") or [] if isinstance(row, dict)]
        polygons: list[list[list[list[float]]]] = []
        geometry_role = "assignment"
        for owner in owners:
            polygons.extend(
                _area_polygons(
                    _pick(owner, "areaAssignmentDetail", "assignmentDetail")
                )
            )
        if not polygons:
            geometry_role = "remaining_fallback"
            for detail in details:
                polygons.extend(
                    _area_polygons(_pick(detail, "remainingDetail", "remaining_detail"))
                )
        mapped_geometry = _mapped_polygon_union(polygons)
        if mapped_geometry is None:
            continue
        aggregate_detail = _aggregate_pass_detail(group)
        representative_owner = owners[0] if owners else None
        aircraft_id = _to_int(group.get("aircraftID"))
        properties = _feature_properties(
            snapshot_plan_id=snapshot_plan_id,
            mission=mission,
            owner=representative_owner,
            geometry_source=(
                f"coveragePass:{pass_name}:aircraft:{aircraft_id}"
                if aircraft_id is not None
                else f"coveragePass:{pass_name}"
            ),
            area_index=area_index,
            coverage_pass_detail=aggregate_detail,
        )
        individual_ids = _normalized_int_list(
            [
                _pick(owner, "individualMissionID", "individualMissionId", "missionID")
                for owner in owners
            ]
        )
        properties.update(
            {
                "coveragePassFeatureKey": f"{aircraft_id or 0}:{pass_name}",
                "coveragePassGeometryRole": geometry_role,
                "coveragePassSourceCount": len(details),
                "geometryComponentCount": _mapped_geometry_component_count(
                    mapped_geometry
                ),
                "individualMissionIDs": ",".join(
                    str(item) for item in individual_ids
                ),
            }
        )
        result.append(
            {
                "type": "Feature",
                "geometry": mapped_geometry,
                "properties": properties,
            }
        )
    return result


def _coverage_depth_features(
    *,
    snapshot_plan_id: int | None,
    mission: dict[str, Any],
) -> list[dict[str, Any]]:
    """Build disjoint 0/1/2-capture bands for one Area mission.

    New snapshots publish ``coverageDepthDetails`` directly.  Older snapshots
    are upgraded in-memory: overlap of the two remaining pass geometries still
    needs two captures (depth 0), while their one-sided differences need one
    capture (depth 1).  OUT/RETURN identity is kept only as attribution.
    """

    result: list[dict[str, Any]] = []
    required_depth = _to_int(
        _pick(mission, "requiredCoverageDepth", "required_coverage_depth")
    )
    required_depth = max(0, min(2, int(required_depth or 2)))
    mission_complete = _area_coverage_is_complete(mission)
    explicit_details = [
        row
        for row in (
            _pick(mission, "coverageDepthDetails", "coverage_depth_details") or []
        )
        if isinstance(row, dict)
        and _normalize_coverage_depth(
            _pick(row, "coverageDepth", "coverage_depth")
        )
        is not None
    ]
    if explicit_details:
        details_by_depth: dict[int, list[dict[str, Any]]] = {}
        for detail in explicit_details:
            depth = _normalize_coverage_depth(
                _pick(detail, "coverageDepth", "coverage_depth")
            )
            if depth is not None:
                details_by_depth.setdefault(int(depth), []).append(detail)

        for area_index, depth in enumerate(sorted(details_by_depth), start=1):
            if mission_complete and int(depth) < int(required_depth):
                # A completed mission may retain tiny tolerance/audit pieces in
                # the raw depth ledger.  Do not resurrect them as pending work.
                continue
            depth_details = details_by_depth[int(depth)]
            polygons: list[list[list[list[float]]]] = []
            for detail in depth_details:
                remaining_detail = _pick(
                    detail,
                    "remainingDetail",
                    "coverageDetail",
                    "depthDetail",
                    "remaining_detail",
                )
                polygons.extend(_area_polygons(remaining_detail))
            mapped_geometry = _mapped_polygon_union(polygons)
            if mapped_geometry is None:
                continue
            aggregate_detail = _aggregate_depth_detail(
                mission,
                depth_details,
                depth=int(depth),
            )
            properties = _depth_feature_properties(
                snapshot_plan_id=snapshot_plan_id,
                mission=mission,
                depth=int(depth),
                area_index=area_index,
                geometry_source=f"coverageDepth:{depth}",
                depth_detail=aggregate_detail,
            )
            properties["coverageDepthDerived"] = 0
            properties["coverageDepthBandIndex"] = int(depth)
            properties["coverageDepthBandCount"] = len(depth_details)
            properties["geometryComponentCount"] = _mapped_geometry_component_count(
                mapped_geometry
            )
            properties["areaCoverageDepthContractVersion"] = _to_int(
                _pick(
                    mission,
                    "areaCoverageDepthContractVersion",
                    "area_coverage_depth_contract_version",
                )
            ) or 1
            result.append(
                {
                    "type": "Feature",
                    "geometry": mapped_geometry,
                    "properties": properties,
                }
            )
        return result

    pass_details = [
        row
        for row in (_pick(mission, "coveragePassDetails", "coverage_pass_details") or [])
        if isinstance(row, dict)
        and str(_pick(row, "coveragePass", "coverage_pass") or "").strip().lower()
        in {"forward", "reverse"}
        and not bool(_pick(row, "isDone", "done"))
    ]
    if not pass_details:
        for owner in _pick(mission, "areaOwnershipDetails", "areaOwnershipDetailList") or []:
            if not isinstance(owner, dict):
                continue
            pass_details.extend(
                row
                for row in (
                    _pick(owner, "coveragePassDetails", "coverage_pass_details") or []
                )
                if isinstance(row, dict)
                and str(_pick(row, "coveragePass", "coverage_pass") or "")
                .strip()
                .lower()
                in {"forward", "reverse"}
                and not bool(_pick(row, "isDone", "done"))
            )
    pass_polygon_groups: dict[str, list[list[list[list[float]]]]] = {}
    for detail in pass_details:
        pass_name = str(_pick(detail, "coveragePass", "coverage_pass") or "").strip().lower()
        pass_polygon_groups.setdefault(pass_name, []).extend(
            _area_polygons(_pick(detail, "remainingDetail", "remaining_detail"))
        )
    pass_geometries = {
        pass_name: geometry
        for pass_name, polygons in pass_polygon_groups.items()
        if (
            geometry := _polygon_geometry(polygons)
        ) is not None
        and not bool(getattr(geometry, "is_empty", True))
    }
    if not pass_geometries:
        return []

    derived: list[tuple[int, Any, list[str]]] = []
    forward = pass_geometries.get("forward")
    reverse = pass_geometries.get("reverse")
    if forward is not None and reverse is not None:
        try:
            derived.extend(
                [
                    (0, forward.intersection(reverse), ["forward", "reverse"]),
                    (1, forward.difference(reverse), ["forward"]),
                    (1, reverse.difference(forward), ["reverse"]),
                ]
            )
        except Exception:
            derived = []
    else:
        for pass_name, geometry in pass_geometries.items():
            derived.append((1, geometry, [pass_name]))

    derived_by_depth: dict[int, dict[str, Any]] = {}
    for depth, geometry, passes in derived:
        if geometry is None or bool(getattr(geometry, "is_empty", True)):
            continue
        group = derived_by_depth.setdefault(
            int(depth),
            {"geometries": [], "passes": set()},
        )
        group["geometries"].append(geometry)
        group["passes"].update(str(item) for item in passes)

    for area_index, depth in enumerate(sorted(derived_by_depth), start=1):
        group = derived_by_depth[int(depth)]
        try:
            geometry = unary_union(group["geometries"]) if unary_union is not None else None
        except Exception:
            geometry = None
        mapped_rows = _mapped_polygon_features(geometry)
        if not mapped_rows:
            continue
        mapped_geometry = mapped_rows[0]
        properties = _depth_feature_properties(
            snapshot_plan_id=snapshot_plan_id,
            mission=mission,
            depth=int(depth),
            area_index=area_index,
            geometry_source=f"coverageDepthDerived:{depth}",
            fallback_passes=sorted(group["passes"]),
        )
        properties["coverageDepthDerived"] = 1
        properties["coverageDepthBandIndex"] = int(depth)
        properties["coverageDepthBandCount"] = len(group["geometries"])
        properties["geometryComponentCount"] = _mapped_geometry_component_count(
            mapped_geometry
        )
        properties["areaCoverageDepthContractVersion"] = 0
        result.append(
            {
                "type": "Feature",
                "geometry": mapped_geometry,
                "properties": properties,
            }
        )
    return result


def _features_from_snapshot(snapshot: dict[str, Any]) -> list[dict[str, Any]]:
    snapshot_plan_id = _to_int(_pick(snapshot, "missionPlanID", "missionPlanId"))
    features: list[dict[str, Any]] = []
    feature_id = 1
    for mission in _pick(snapshot, "missions", "missionList") or []:
        if not isinstance(mission, dict):
            continue
        normalized_mission = mission_area_replan_store.normalize_area_single_capture_entry(
            mission
        )
        if isinstance(normalized_mission, dict):
            mission = normalized_mission
        mission_type = str(_pick(mission, "missionType", "type") or "").strip().lower()
        if mission_type == "line":
            if bool(_pick(mission, "isDone", "is_done", "done")):
                continue
            # 잔여 line 구간은 폭을 반영한 회랑(리본) 폴리곤 + 중심선으로 노출해
            # area 잔여영역과 동일한 투명 채움 스타일로 렌더링되게 한다.
            for line_index, (path, width_m) in enumerate(_line_blocks(mission), start=1):
                ribbon_ring = _line_ribbon_ring(path, width_m)
                if ribbon_ring:
                    features.append(
                        {
                            "type": "Feature",
                            "id": feature_id,
                            "geometry": {"type": "Polygon", "coordinates": [ribbon_ring]},
                            "properties": _feature_properties(
                                snapshot_plan_id=snapshot_plan_id,
                                mission=mission,
                                owner=None,
                                geometry_source="lineRemainingDetail",
                                area_index=line_index,
                                mission_kind="line",
                                width_m=width_m,
                            ),
                        }
                    )
                    feature_id += 1
                features.append(
                    {
                        "type": "Feature",
                        "id": feature_id,
                        "geometry": {"type": "LineString", "coordinates": path},
                        "properties": _feature_properties(
                            snapshot_plan_id=snapshot_plan_id,
                            mission=mission,
                            owner=None,
                            geometry_source="lineRemainingCenterline",
                            area_index=line_index,
                            mission_kind="line",
                            width_m=width_m,
                        ),
                    }
                )
                feature_id += 1
            continue
        if mission_type != "area":
            continue

        depth_features = _coverage_depth_features(
            snapshot_plan_id=snapshot_plan_id,
            mission=mission,
        )
        for depth_feature in depth_features:
            depth_feature["id"] = feature_id
            features.append(depth_feature)
            feature_id += 1

        pass_features = _coverage_pass_features(
            snapshot_plan_id=snapshot_plan_id,
            mission=mission,
        )
        for pass_feature in pass_features:
            pass_feature["id"] = feature_id
            features.append(pass_feature)
            feature_id += 1

        if depth_features and not pass_features:
            # A depth-native snapshot no longer needs the legacy central
            # remainingDetail fill.  It would mask the disjoint depth bands.
            continue
        if pass_features:
            # The central remainingDetail is the spatial union/intersection
            # view used by legacy consumers.  Rendering it together with the
            # pass ledgers masks their independent progress, so reciprocal
            # snapshots use the pass features exclusively in the SIM map.
            continue

        mission_polygons = _area_polygons(_pick(mission, "remainingDetail", "remaining_detail"))
        ownership = _pick(mission, "areaOwnershipDetails", "areaOwnershipDetailList")
        owner_polygons: list[tuple[dict[str, Any], list[list[list[float]]]]] = []
        if isinstance(ownership, list):
            for owner in ownership:
                if not isinstance(owner, dict):
                    continue
                detail = _pick(owner, "remainingDetail", "remaining_detail")
                for polygon in _area_polygons(detail):
                    owner_polygons.append((owner, polygon))

        mission_area = _polygons_area_degrees(mission_polygons)
        owner_area = _polygons_area_degrees([polygon for _owner, polygon in owner_polygons])
        use_owner_projection = bool(
            owner_polygons
            and not mission_polygons
            and owner_area > 0.0
            and mission_area <= 0.0
        )
        if use_owner_projection:
            owner_union = _mapped_polygon_union(
                [polygon for _owner, polygon in owner_polygons]
            )
            if owner_union is not None:
                features.append(
                    {
                        "type": "Feature",
                        "id": feature_id,
                        "geometry": owner_union,
                        "properties": _feature_properties(
                            snapshot_plan_id=snapshot_plan_id,
                            mission=mission,
                            owner=None,
                            geometry_source="areaOwnershipUnionFallback",
                            area_index=1,
                        ),
                    }
                )
                feature_id += 1
            continue

        for area_index, polygon in enumerate(mission_polygons, start=1):
            features.append(
                {
                    "type": "Feature",
                    "id": feature_id,
                    "geometry": {"type": "Polygon", "coordinates": polygon},
                    "properties": _feature_properties(
                        snapshot_plan_id=snapshot_plan_id,
                        mission=mission,
                        owner=None,
                        geometry_source="remainingDetail",
                        area_index=area_index,
                    ),
                }
            )
            feature_id += 1
    return features


def _features_revision(features: list[dict[str, Any]]) -> str:
    try:
        serialized = json.dumps(
            features,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        )
    except Exception:
        serialized = str(len(features))
    return hashlib.sha1(serialized.encode("utf-8", errors="ignore")).hexdigest()


def _coverage_pass_summaries(snapshot: dict[str, Any]) -> list[dict[str, Any]]:
    summaries: list[dict[str, Any]] = []
    for mission in _pick(snapshot, "missions", "missionList") or []:
        if not isinstance(mission, dict):
            continue
        for group in _coverage_pass_groups(mission):
            detail = _aggregate_pass_detail(group)
            pass_name = str(group.get("coveragePass") or "")
            aircraft_id = _to_int(group.get("aircraftID"))
            owners = [
                row for row in group.get("owners") or [] if isinstance(row, dict)
            ]
            representative_owner = owners[0] if owners else None
            done = bool(_pick(detail, "isDone", "done"))
            progress = _to_int(
                _pick(detail, "coveragePercent", "coverage_percent")
            )
            if progress is None:
                progress = 100 if done else 0
            progress = max(0, min(100, int(progress)))
            active_pass = str(
                _pick(mission, "activeCoveragePass", "active_coverage_pass") or ""
            ).strip().lower()
            owner_is_current = any(
                bool(_pick(owner, "isCurrent", "is_current")) for owner in owners
            )
            input_id = _to_int(
                _pick(
                    representative_owner,
                    "inputMissionID",
                    "inputMissionId",
                    "input_id",
                )
            )
            if input_id is None:
                input_id = _to_int(
                    _pick(mission, "inputMissionID", "inputMissionId", "input_id")
                )
            summaries.append(
                {
                    "inputMissionID": input_id,
                    "aircraftID": aircraft_id,
                    "agent": _agent_label(aircraft_id),
                    "coveragePass": pass_name,
                    "coveragePassRequirementMode": "all_passes_required",
                    "contributesToCoverageCompletion": True,
                    "passIndex": _to_int(_pick(detail, "passIndex", "pass_index")),
                    "status": (
                        "completed"
                        if done
                        else "active"
                        if pass_name == active_pass or progress > 0 or owner_is_current
                        else "planned"
                    ),
                    "progress": int(progress),
                    "isDone": bool(done),
                    "plannedAreaM2": _to_float(
                        _pick(detail, "plannedAreaM2", "planned_area_m2")
                    ),
                    "coveredAreaM2": _to_float(
                        _pick(detail, "coveredAreaM2", "covered_area_m2")
                    ),
                    "remainingAreaM2": _to_float(
                        _pick(detail, "remainingAreaM2", "remaining_area_m2")
                    ),
                }
            )
    summaries.sort(
        key=lambda row: (
            int(row.get("inputMissionID") or 0),
            int(row.get("aircraftID") or 0),
            int(row.get("passIndex") or 999),
            str(row.get("coveragePass") or ""),
        )
    )
    return summaries


def _coverage_depth_summaries(
    snapshot: dict[str, Any],
    *,
    features: list[dict[str, Any]] | None = None,
) -> list[dict[str, Any]]:
    counted_area_bands: set[tuple[int | None, int, int]] = set()
    source_features: list[dict[str, Any]] = []
    if features is not None:
        source_features = [item for item in features if isinstance(item, dict)]
    else:
        snapshot_plan_id = _to_int(_pick(snapshot, "missionPlanID", "missionPlanId"))
        for mission in _pick(snapshot, "missions", "missionList") or []:
            if not isinstance(mission, dict):
                continue
            source_features.extend(
                _coverage_depth_features(
                    snapshot_plan_id=snapshot_plan_id,
                    mission=mission,
                )
            )

    grouped: dict[tuple[int | None, int], dict[str, Any]] = {}
    for feature in source_features:
        props = feature.get("properties") if isinstance(feature, dict) else {}
        if str(_pick(props, "visualizationRole") or "") != "coverageDepth":
            continue
        input_id = _to_int(
            _pick(props, "inputMissionID", "inputMissionId", "input_id")
        )
        depth = _normalize_coverage_depth(_pick(props, "coverageDepth"))
        if depth is None:
            continue
        remaining_capture_count = _to_int(
            _pick(props, "remainingCaptureCount", "remaining_capture_count")
        )
        row = grouped.setdefault(
            (input_id, int(depth)),
            {
                "inputMissionID": input_id,
                "coverageDepth": int(depth),
                "remainingCaptureCount": remaining_capture_count
                if remaining_capture_count is not None
                else 2 - int(depth),
                "status": _coverage_depth_status(int(depth)),
                "label": _coverage_depth_label(int(depth)),
                "areaM2": 0.0,
                "geometryCount": 0,
                "activeAircraftIDs": set(),
                "activeAgents": set(),
                "activeCoveragePasses": set(),
                "derived": bool(_to_int(_pick(props, "coverageDepthDerived"))),
            },
        )
        row["geometryCount"] += 1
        row["derived"] = bool(row["derived"]) or bool(
            _to_int(_pick(props, "coverageDepthDerived"))
        )
        area_m2 = _to_float(_pick(props, "areaM2", "remainingAreaM2"))
        band_index = _to_int(_pick(props, "coverageDepthBandIndex"))
        band_key = (input_id, int(depth), int(band_index or 0))
        if area_m2 is not None and (
            bool(_to_int(_pick(props, "coverageDepthDerived")))
            or band_key not in counted_area_bands
        ):
            row["areaM2"] += float(area_m2)
            counted_area_bands.add(band_key)
        row["activeAircraftIDs"].update(
            _normalized_int_list(
                str(_pick(props, "activeAircraftIDs") or "").split(",")
            )
        )
        row["activeAgents"].update(
            item
            for item in str(_pick(props, "activeAgents") or "").split(",")
            if item
        )
        row["activeCoveragePasses"].update(
            _normalized_pass_list(
                str(_pick(props, "activeCoveragePasses") or "").split(",")
            )
        )

    summaries: list[dict[str, Any]] = []
    for group_key in sorted(
        grouped,
        key=lambda key: (int(key[0] or 0), int(key[1])),
    ):
        row = grouped[group_key]
        row["areaM2"] = round(float(row["areaM2"]), 3)
        row["activeAircraftIDs"] = sorted(row["activeAircraftIDs"])
        row["activeAgents"] = sorted(row["activeAgents"])
        row["activeCoveragePasses"] = sorted(row["activeCoveragePasses"])
        summaries.append(row)
    summaries.sort(
        key=lambda row: (
            int(row.get("inputMissionID") or 0),
            int(row.get("coverageDepth") or 0),
        )
    )
    return summaries


def build_remaining_area_snapshot(mission_plan_id: int | None = None) -> dict[str, Any]:
    snapshot, path, mtime = _load_snapshot(mission_plan_id)
    if not isinstance(snapshot, dict):
        return {
            "ok": True,
            "available": False,
            "missionPlanID": mission_plan_id,
            "snapshotPath": str(path) if path is not None else None,
            "timestamp": int(time.time() * 1000),
            "featureCollection": {"type": "FeatureCollection", "features": []},
            "features": [],
            "coveragePassSummaries": [],
            "coveragePassRequirementMode": "all_passes_required",
            "coverageDepthSummaries": [],
            "requiredCoverageDepth": 2,
            "count": 0,
        }
    snapshot_plan_id = _to_int(_pick(snapshot, "missionPlanID", "missionPlanId"))
    features = _features_from_snapshot(snapshot)
    coverage_pass_summaries = _coverage_pass_summaries(snapshot)
    coverage_depth_summaries = _coverage_depth_summaries(snapshot, features=features)
    data_revision = _features_revision(features)
    return {
        "ok": True,
        "available": True,
        "missionPlanID": snapshot_plan_id,
        "snapshotPath": str(path) if path is not None else None,
        "snapshotMtimeMs": int(float(mtime) * 1000.0) if mtime is not None else None,
        "dataRevision": data_revision,
        "timestamp": _to_int(_pick(snapshot, "timestamp")) or int(time.time() * 1000),
        "featureCollection": {
            "type": "FeatureCollection",
            "features": features,
        },
        "features": features,
        "coveragePassSummaries": coverage_pass_summaries,
        "coveragePassRequirementMode": "all_passes_required",
        "coverageDepthSummaries": coverage_depth_summaries,
        "requiredCoverageDepth": 2,
        "count": len(features),
    }
