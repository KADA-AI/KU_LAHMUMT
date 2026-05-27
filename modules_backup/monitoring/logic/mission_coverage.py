# -*- coding: utf-8 -*-
from __future__ import annotations

import math
from dataclasses import dataclass
from functools import lru_cache
from typing import Any, Iterable

from pyproj import Transformer
from shapely.geometry import GeometryCollection, LineString, MultiPoint, Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import unary_union

_MIN_AREA_M2 = 1e-3
_MAX_VERTEX_COUNT = 4096
_SIMPLIFY_TOLERANCE_M = 0.5
_SWEEP_BRIDGE_GAP_FACTOR = 3.0
_NO_COVERAGE_MISSION_TYPES = {5, 7}


def _coerce_float(value: object) -> float | None:
    try:
        return float(value)
    except Exception:
        return None


def _coerce_int(value: object) -> int | None:
    try:
        return int(value)
    except Exception:
        return None


def _normalize_geometry(geometry: BaseGeometry | None) -> BaseGeometry:
    if geometry is None:
        return GeometryCollection()
    if geometry.is_empty:
        return GeometryCollection()
    if not geometry.is_valid:
        try:
            geometry = geometry.buffer(0)
        except Exception:
            return GeometryCollection()
    if geometry.is_empty:
        return GeometryCollection()
    return geometry


def _vertex_count(geometry: BaseGeometry | None) -> int:
    if geometry is None or geometry.is_empty:
        return 0
    geom_type = geometry.geom_type
    if geom_type == "Polygon":
        count = len(getattr(geometry.exterior, "coords", []) or [])
        for ring in geometry.interiors:
            count += len(getattr(ring, "coords", []) or [])
        return int(count)
    if geom_type in ("MultiPolygon", "GeometryCollection"):
        total = 0
        for child in geometry.geoms:
            total += _vertex_count(child)
        return int(total)
    if hasattr(geometry, "coords"):
        return len(list(geometry.coords))
    return 0


@lru_cache(maxsize=64)
def _utm_transformer(zone: int, south: bool) -> Transformer:
    utm_code = f"+proj=utm +zone={zone} {'+south' if south else ''} +ellps=WGS84 +units=m +no_defs"
    return Transformer.from_crs("EPSG:4326", utm_code, always_xy=True)


def _extract_lat_lon(coord: object) -> tuple[float, float] | None:
    if not isinstance(coord, dict):
        return None
    lat = _coerce_float(coord.get("latitude"))
    lon = _coerce_float(coord.get("longitude"))
    if lat is None or lon is None:
        return None
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        return None
    return float(lat), float(lon)


def _coordinate_dicts(items: Iterable[object] | None) -> list[dict[str, Any]]:
    result: list[dict[str, Any]] = []
    if items is None:
        return result
    for item in items:
        if isinstance(item, dict):
            result.append(item)
    return result


def _resolve_transformer(
    coordinate_lists: Iterable[Iterable[object] | None],
) -> Transformer | None:
    coords: list[tuple[float, float]] = []
    for items in coordinate_lists:
        for item in items or []:
            coord = _extract_lat_lon(item)
            if coord is None:
                continue
            coords.append(coord)
    if not coords:
        return None
    avg_lat = sum(lat for lat, _lon in coords) / len(coords)
    avg_lon = sum(lon for _lat, lon in coords) / len(coords)
    zone = int((avg_lon + 180.0) // 6.0) + 1
    zone = max(1, min(zone, 60))
    south = avg_lat < 0.0
    return _utm_transformer(zone, south)


def _project_points(
    items: Iterable[object] | None,
    transformer: Transformer,
) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []
    seen: set[tuple[int, int]] = set()
    for item in items or []:
        coord = _extract_lat_lon(item)
        if coord is None:
            continue
        lat, lon = coord
        x, y = transformer.transform(lon, lat)
        key = (int(round(x * 1000.0)), int(round(y * 1000.0)))
        if key in seen:
            continue
        seen.add(key)
        points.append((float(x), float(y)))
    return points


def _build_area_geometry(area_list: list[dict[str, Any]], transformer: Transformer) -> BaseGeometry:
    outers: list[BaseGeometry] = []
    holes: list[BaseGeometry] = []
    for area in area_list:
        coords = _project_points(area.get("coordinateList"), transformer)
        if len(coords) < 3:
            continue
        geometry = _normalize_geometry(Polygon(coords))
        if geometry.is_empty or geometry.area <= _MIN_AREA_M2:
            continue
        if bool(area.get("isHole")):
            holes.append(geometry)
        else:
            outers.append(geometry)
    if not outers:
        return GeometryCollection()
    geometry = _normalize_geometry(unary_union(outers))
    if holes and not geometry.is_empty:
        geometry = _normalize_geometry(geometry.difference(unary_union(holes)))
    return geometry


def _build_line_geometry(line_list: list[dict[str, Any]], transformer: Transformer) -> BaseGeometry:
    pieces: list[BaseGeometry] = []
    for line in line_list:
        coords = _project_points(line.get("coordinateList"), transformer)
        if not coords:
            continue
        width = _coerce_float(line.get("width")) or 0.0
        if len(coords) == 1:
            if width <= 0.0:
                continue
            geometry = Point(coords[0]).buffer(width / 2.0)
        else:
            geometry = LineString(coords)
            if width > 0.0:
                geometry = geometry.buffer(width / 2.0, cap_style=2, join_style=2)
        geometry = _normalize_geometry(geometry)
        if geometry.is_empty or geometry.area <= _MIN_AREA_M2:
            continue
        pieces.append(geometry)
    if not pieces:
        return GeometryCollection()
    return _normalize_geometry(unary_union(pieces))


@dataclass(frozen=True)
class MissionCoverageDefinition:
    planned_area_m2: float
    assignment_geometry: BaseGeometry
    transformer: Transformer


def build_mission_coverage_definition(mission: dict[str, Any]) -> MissionCoverageDefinition | None:
    mission_type = _coerce_int(mission.get("individual_mission_type"))
    if mission_type in _NO_COVERAGE_MISSION_TYPES:
        return None
    sweep_point_count = _coerce_int(mission.get("sweep_point_count")) or 0
    if not bool(mission.get("is_done")) and sweep_point_count <= 0:
        return None

    line_list = _coordinate_dicts(mission.get("line_list"))
    area_list = _coordinate_dicts(mission.get("area_list"))
    coordinate_lists = []
    coordinate_lists.extend(line.get("coordinateList") for line in line_list)
    coordinate_lists.extend(area.get("coordinateList") for area in area_list)
    transformer = _resolve_transformer(coordinate_lists)
    if transformer is None:
        return None

    pieces: list[BaseGeometry] = []
    area_geometry = _build_area_geometry(area_list, transformer)
    if not area_geometry.is_empty:
        pieces.append(area_geometry)
    line_geometry = _build_line_geometry(line_list, transformer)
    if not line_geometry.is_empty:
        pieces.append(line_geometry)
    if not pieces:
        return None

    assignment = _normalize_geometry(unary_union(pieces))
    planned_area_m2 = float(max(0.0, assignment.area))
    if assignment.is_empty or planned_area_m2 <= _MIN_AREA_M2:
        return None
    return MissionCoverageDefinition(
        planned_area_m2=planned_area_m2,
        assignment_geometry=assignment,
        transformer=transformer,
    )


def build_footprint_geometry(
    footprint_corners: Iterable[object] | None,
    transformer: Transformer,
) -> BaseGeometry:
    points = _project_points(footprint_corners, transformer)
    if len(points) < 3:
        return GeometryCollection()
    geometry = _normalize_geometry(MultiPoint(points).convex_hull)
    if geometry.geom_type != "Polygon":
        return GeometryCollection()
    if geometry.area <= _MIN_AREA_M2:
        return GeometryCollection()
    return geometry


def merge_coverage_geometry(
    existing_geometry: BaseGeometry | None,
    additional_geometry: BaseGeometry | None,
) -> BaseGeometry:
    existing = _normalize_geometry(existing_geometry)
    additional = _normalize_geometry(additional_geometry)
    if additional.is_empty:
        return existing
    if existing.is_empty:
        merged = additional
    else:
        merged = _normalize_geometry(unary_union([existing, additional]))
    if merged.is_empty:
        return GeometryCollection()
    if _vertex_count(merged) > _MAX_VERTEX_COUNT:
        simplified = _normalize_geometry(
            merged.simplify(_SIMPLIFY_TOLERANCE_M, preserve_topology=True)
        )
        if not simplified.is_empty and simplified.area > 0.0:
            merged = simplified
    return merged


def build_swept_footprint_geometry(
    previous_footprint: BaseGeometry | None,
    current_footprint: BaseGeometry | None,
) -> BaseGeometry:
    previous = _normalize_geometry(previous_footprint)
    current = _normalize_geometry(current_footprint)
    if previous.is_empty:
        return current
    if current.is_empty:
        return previous

    merged = _normalize_geometry(unary_union([previous, current]))
    if merged.is_empty:
        return GeometryCollection()

    try:
        prev_bounds = previous.bounds
        curr_bounds = current.bounds
        prev_diag = math.hypot(prev_bounds[2] - prev_bounds[0], prev_bounds[3] - prev_bounds[1])
        curr_diag = math.hypot(curr_bounds[2] - curr_bounds[0], curr_bounds[3] - curr_bounds[1])
        max_diag = max(prev_diag, curr_diag, 1e-6)
        centroid_gap = previous.centroid.distance(current.centroid)
    except Exception:
        return merged

    if centroid_gap > (max_diag * _SWEEP_BRIDGE_GAP_FACTOR):
        return merged

    swept = _normalize_geometry(merged.convex_hull)
    if swept.is_empty or swept.area <= merged.area:
        return merged
    return swept
