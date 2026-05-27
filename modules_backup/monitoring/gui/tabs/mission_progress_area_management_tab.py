# -*- coding: utf-8 -*-
from __future__ import annotations

import json
import math
import time
from copy import deepcopy
from dataclasses import dataclass, field
from typing import Any

from PyQt5.QtCore import QPoint, QPointF, Qt, QTimer
from PyQt5.QtGui import QBrush, QColor, QPainter, QPainterPath, QPen
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)
from pyproj.enums import TransformDirection
from shapely.geometry import GeometryCollection, LineString, Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import linemerge, split as split_geometry

from modules.common import mission_area_replan_store
from modules.monitoring.logic.mission_coverage import (
    MissionCoverageDefinition,
    build_mission_coverage_definition,
    build_footprint_geometry,
    merge_coverage_geometry,
)
from modules.monitoring.logic.mission_progress import MissionProgressTracker
from modules.monitoring.logic.mission_update import build_uav_mission_view, format_timestamp_ms

_UAV_IDS = (4, 5, 6)
_UAV_COLORS = {4: "#2563eb", 5: "#0f766e", 6: "#b45309"}
_DEFAULT_STRIP_WIDTH_M = 25.0


def _gc() -> BaseGeometry:
    return GeometryCollection()


def _as_int(value: object | None) -> int | None:
    try:
        return None if value is None else int(value)
    except Exception:
        return None


def _as_float(value: object | None) -> float | None:
    try:
        return None if value is None else float(value)
    except Exception:
        return None


def _coord(value: object | None) -> dict[str, float] | None:
    if not isinstance(value, dict):
        return None
    lat = _as_float(value.get("latitude") or value.get("Latitude"))
    lon = _as_float(value.get("longitude") or value.get("Longitude"))
    alt = _as_float(value.get("altitude") or value.get("Altitude"))
    if lat is None or lon is None:
        return None
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        return None
    data = {"latitude": float(lat), "longitude": float(lon)}
    if alt is not None:
        data["altitude"] = float(alt)
    return data


def _ground_distance_m(left: dict[str, float] | None, right: dict[str, float] | None) -> float | None:
    if not left or not right:
        return None
    lat1 = math.radians(float(left["latitude"]))
    lon1 = math.radians(float(left["longitude"]))
    lat2 = math.radians(float(right["latitude"]))
    lon2 = math.radians(float(right["longitude"]))
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    return 6_371_000.0 * (2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a)))


def _geometry_intersection_area_m2(
    left: BaseGeometry | None,
    right: BaseGeometry | None,
) -> float:
    if left is None or right is None or left.is_empty or right.is_empty:
        return 0.0
    try:
        return float(left.intersection(right).area or 0.0)
    except Exception:
        return 0.0


def _footprint_width_m(corners: list[dict[str, Any]] | None) -> float | None:
    if not corners or len(corners) < 4:
        return None
    pts = [_coord(item) for item in corners[:4]]
    if any(pt is None for pt in pts):
        return None
    top = _ground_distance_m(pts[0], pts[1])
    bot = _ground_distance_m(pts[3], pts[2])
    if top is not None and bot is not None:
        return (top + bot) * 0.5
    return top if top is not None else bot


def _observation_overlap_threshold_m2(
    state: _MissionAreaState,
    *,
    width_m: float | None,
    footprint_area_m2: float | None,
) -> float:
    width_hint_m = max(float(width_m or 0.0), float(state.width_hint_m or 0.0), 1.0)
    half_width_m = max(float(state.cut_half_width_m or 0.0), 1.0)
    threshold_candidates = [
        18.0,
        width_hint_m * half_width_m * 0.22,
    ]
    if footprint_area_m2 is not None and footprint_area_m2 > 0.0:
        threshold_candidates.append(min(float(footprint_area_m2) * 0.12, 220.0))
    return float(max(threshold_candidates))


def _footprint_observes_assignment(
    state: _MissionAreaState,
    footprint_geometry: BaseGeometry | None,
    *,
    width_m: float | None,
) -> bool:
    if footprint_geometry is None or footprint_geometry.is_empty:
        return True
    overlap_area_m2 = _geometry_intersection_area_m2(
        footprint_geometry,
        state.assignment_geometry,
    )
    footprint_area_m2 = float(footprint_geometry.area or 0.0)
    threshold_m2 = _observation_overlap_threshold_m2(
        state,
        width_m=width_m,
        footprint_area_m2=footprint_area_m2,
    )
    if overlap_area_m2 >= threshold_m2:
        return True
    if footprint_area_m2 <= 1e-6:
        return False
    return (overlap_area_m2 / footprint_area_m2) >= 0.18


def _footprint_supports_planned_line(
    state: _MissionAreaState,
    *,
    line_index: int,
    footprint_geometry: BaseGeometry | None,
    width_m: float | None,
) -> bool:
    if footprint_geometry is None or footprint_geometry.is_empty:
        return True
    if line_index < 0 or line_index >= len(state.planned_cut_lines):
        return False
    (
        overlap_area_m2,
        _strip_area_m2,
        footprint_area_m2,
        _strip_overlap_ratio,
        footprint_overlap_ratio,
        threshold_m2,
    ) = _planned_line_overlap_metrics(
        state,
        line_index=int(line_index),
        footprint_geometry=footprint_geometry,
        width_m=width_m,
    )
    if overlap_area_m2 >= threshold_m2:
        return True
    if footprint_area_m2 <= 1e-6:
        return False
    return footprint_overlap_ratio >= 0.12


def _project_xy(transformer, value: dict[str, float] | None) -> tuple[float, float] | None:
    if not value:
        return None
    try:
        x, y = transformer.transform(float(value["longitude"]), float(value["latitude"]))
    except Exception:
        return None
    return float(x), float(y)


def _normalize_vector(dx: float, dy: float) -> tuple[float, float] | None:
    length = math.hypot(float(dx), float(dy))
    if length <= 1e-6:
        return None
    return float(dx) / length, float(dy) / length


def _mission_track_vector(mission: dict[str, Any], coverage_def: MissionCoverageDefinition) -> tuple[float, float] | None:
    best_vec: tuple[float, float] | None = None
    best_len = 0.0
    for line in mission.get("line_list") or []:
        if not isinstance(line, dict):
            continue
        points: list[tuple[float, float]] = []
        for coord_item in line.get("coordinateList") or []:
            xy = _project_xy(coverage_def.transformer, _coord(coord_item))
            if xy is not None:
                points.append(xy)
        for idx in range(1, len(points)):
            dx = points[idx][0] - points[idx - 1][0]
            dy = points[idx][1] - points[idx - 1][1]
            length = math.hypot(dx, dy)
            vec = _normalize_vector(dx, dy)
            if vec is not None and length > best_len:
                best_len = length
                best_vec = vec
    if best_vec is not None:
        return best_vec
    geometry = coverage_def.assignment_geometry
    if geometry.is_empty:
        return None
    try:
        rect = geometry.minimum_rotated_rectangle
        coords = list(rect.exterior.coords)
    except Exception:
        return None
    for idx in range(1, len(coords)):
        dx = coords[idx][0] - coords[idx - 1][0]
        dy = coords[idx][1] - coords[idx - 1][1]
        length = math.hypot(dx, dy)
        vec = _normalize_vector(dx, dy)
        if vec is not None and length > best_len:
            best_len = length
            best_vec = vec
    return best_vec


def _iter_line_strings(geometry: BaseGeometry | None) -> list[Any]:
    if geometry is None or geometry.is_empty:
        return []
    if geometry.geom_type == "LineString":
        return [geometry]
    if geometry.geom_type in ("MultiLineString", "GeometryCollection"):
        result: list[Any] = []
        for child in geometry.geoms:
            result.extend(_iter_line_strings(child))
        return result
    return []


def _iter_polygons(geometry: BaseGeometry | None) -> list[Any]:
    if geometry is None or geometry.is_empty:
        return []
    if geometry.geom_type == "Polygon":
        return [geometry]
    if geometry.geom_type in ("MultiPolygon", "GeometryCollection"):
        result: list[Any] = []
        for child in geometry.geoms:
            result.extend(_iter_polygons(child))
        return result
    return []


def _fill_geometry_holes(geometry: BaseGeometry | None) -> BaseGeometry:
    polygons = _iter_polygons(geometry)
    if not polygons:
        return geometry if geometry is not None else GeometryCollection()
    filled: BaseGeometry = GeometryCollection()
    for poly in polygons:
        try:
            outer = Polygon(poly.exterior)
        except Exception:
            continue
        if outer.is_empty:
            continue
        filled = merge_coverage_geometry(filled, outer)
    return filled


def _filter_small_polygons(
    geometry: BaseGeometry | None,
    *,
    area_threshold_m2: float,
) -> BaseGeometry:
    polygons = _iter_polygons(geometry)
    if not polygons:
        return geometry if geometry is not None else GeometryCollection()
    filtered: BaseGeometry = GeometryCollection()
    threshold = max(float(area_threshold_m2), 0.0)
    for poly in polygons:
        try:
            area_m2 = float(poly.area or 0.0)
        except Exception:
            area_m2 = 0.0
        if area_m2 < threshold:
            continue
        filtered = merge_coverage_geometry(filtered, poly)
    return filtered


def _inverse_project_coord(
    transformer,
    x_val: float,
    y_val: float,
    altitude: float | int | None = None,
) -> dict[str, Any] | None:
    try:
        lon, lat = transformer.transform(
            float(x_val),
            float(y_val),
            direction=TransformDirection.INVERSE,
        )
    except Exception:
        return None
    if not (-90.0 <= float(lat) <= 90.0 and -180.0 <= float(lon) <= 180.0):
        return None
    coord: dict[str, Any] = {
        "latitude": float(lat),
        "longitude": float(lon),
    }
    alt = _as_float(altitude)
    if alt is not None:
        coord["altitude"] = int(round(float(alt)))
    return coord


def _dedupe_coord_list(
    coords: list[dict[str, Any]],
    *,
    closed: bool,
) -> list[dict[str, Any]]:
    deduped: list[dict[str, Any]] = []
    last_key: tuple[int, int, int] | None = None
    for item in coords:
        if not isinstance(item, dict):
            continue
        lat = _as_float(item.get("latitude"))
        lon = _as_float(item.get("longitude"))
        alt = _as_float(item.get("altitude"))
        if lat is None or lon is None:
            continue
        key = (
            int(round(float(lat) * 1_000_000.0)),
            int(round(float(lon) * 1_000_000.0)),
            int(round(float(alt or 0.0))),
        )
        if last_key == key:
            continue
        last_key = key
        coord: dict[str, Any] = {
            "latitude": float(lat),
            "longitude": float(lon),
        }
        if alt is not None:
            coord["altitude"] = int(round(float(alt)))
        deduped.append(coord)
    if closed and len(deduped) >= 2:
        first = deduped[0]
        last = deduped[-1]
        if (
            abs(float(first["latitude"]) - float(last["latitude"])) <= 1e-8
            and abs(float(first["longitude"]) - float(last["longitude"])) <= 1e-8
        ):
            deduped.pop()
    return deduped


def _representative_altitude_from_coords(coord_lists: list[list[dict[str, Any]]]) -> float | None:
    for coords in coord_lists:
        for item in coords:
            alt = _as_float((item or {}).get("altitude"))
            if alt is not None:
                return float(alt)
    return None


def _build_source_line_geometries(
    source_line_list: list[dict[str, Any]],
    source_coordinate_list: list[dict[str, Any]],
    transformer,
    *,
    default_width_m: float,
) -> list[tuple[LineString, float]]:
    pieces: list[tuple[LineString, float]] = []
    for line in source_line_list:
        if not isinstance(line, dict):
            continue
        coords = [
            xy
            for xy in (
                _project_xy(transformer, _coord(item))
                for item in (line.get("coordinateList") or [])
            )
            if xy is not None
        ]
        if len(coords) < 2:
            continue
        width_m = _as_float(line.get("width")) or float(default_width_m)
        try:
            pieces.append((LineString(coords), max(0.1, float(width_m))))
        except Exception:
            continue
    if pieces:
        return pieces
    coords = [
        xy
        for xy in (_project_xy(transformer, _coord(item)) for item in (source_coordinate_list or []))
        if xy is not None
    ]
    if len(coords) >= 2:
        try:
            pieces.append((LineString(coords), max(0.1, float(default_width_m))))
        except Exception:
            pass
    return pieces


def _line_coord_lists_from_geometry(
    geometry: BaseGeometry,
    transformer,
    *,
    altitude: float | None,
    min_length_m: float,
) -> list[list[dict[str, Any]]]:
    out: list[list[dict[str, Any]]] = []
    if geometry.is_empty:
        return out
    merged = geometry
    try:
        merged = linemerge(geometry)
    except Exception:
        merged = geometry
    for line in _iter_line_strings(merged):
        try:
            if float(line.length or 0.0) < float(min_length_m):
                continue
        except Exception:
            continue
        coords_llh = [
            coord
            for coord in (
                _inverse_project_coord(transformer, x_val, y_val, altitude)
                for x_val, y_val in list(line.coords)
            )
            if coord is not None
        ]
        coords_llh = _dedupe_coord_list(coords_llh, closed=False)
        if len(coords_llh) >= 2:
            out.append(coords_llh)
    return out


def _nearest_source_line_index(
    source_lines: list[tuple[LineString, float]],
    current_xy: tuple[float, float] | None,
) -> int | None:
    if current_xy is None or not source_lines:
        return None
    current_point = Point(current_xy)
    best_index: int | None = None
    best_distance: float | None = None
    for source_idx, (source_line, _width_m) in enumerate(source_lines):
        try:
            source_distance = float(source_line.distance(current_point))
        except Exception:
            continue
        if best_distance is None or source_distance < best_distance:
            best_index = int(source_idx)
            best_distance = float(source_distance)
    return best_index


def _clip_line_to_geometry_fragments(
    line: LineString,
    clip_geometry: BaseGeometry,
    *,
    clip_margin_m: float,
    min_length_m: float,
) -> list[LineString]:
    if line.is_empty or clip_geometry is None or clip_geometry.is_empty:
        return []
    clip_region = clip_geometry
    margin_m = max(0.0, float(clip_margin_m))
    if margin_m > 0.0:
        try:
            clip_region = clip_geometry.buffer(
                margin_m,
                cap_style=2,
                join_style=2,
            )
        except Exception:
            clip_region = clip_geometry
    try:
        clipped = line.intersection(clip_region)
    except Exception:
        clipped = GeometryCollection()
    if clipped.is_empty:
        return []
    try:
        clipped = linemerge(clipped)
    except Exception:
        pass
    ranges: list[tuple[float, float]] = []
    for child in _iter_line_strings(clipped):
        coords = list(child.coords)
        if len(coords) < 2:
            continue
        try:
            start_m = float(line.project(Point(coords[0])))
            end_m = float(line.project(Point(coords[-1])))
        except Exception:
            continue
        if end_m < start_m:
            start_m, end_m = end_m, start_m
        if end_m - start_m < float(min_length_m):
            continue
        ranges.append((float(start_m), float(end_m)))
    if not ranges:
        return []
    ranges.sort(key=lambda item: (float(item[0]), float(item[1])))
    merged_ranges: list[list[float]] = []
    gap_threshold_m = max(1.0, float(clip_margin_m) * 1.5)
    for start_m, end_m in ranges:
        if not merged_ranges or float(start_m) > float(merged_ranges[-1][1]) + gap_threshold_m:
            merged_ranges.append([float(start_m), float(end_m)])
            continue
        merged_ranges[-1][1] = max(float(merged_ranges[-1][1]), float(end_m))
    fragments: list[LineString] = []
    for start_m, end_m in merged_ranges:
        fragment = _substring_line(line, float(start_m), float(end_m))
        for candidate in _iter_line_strings(fragment):
            try:
                if float(candidate.length or 0.0) < float(min_length_m):
                    continue
            except Exception:
                continue
            fragments.append(candidate)
    return fragments


def _build_remaining_line_blocks(
    source_lines: list[tuple[LineString, float]],
    *,
    clip_geometry: BaseGeometry,
    transformer,
    altitude: float | None,
    min_length_m: float,
    clip_margin_m: float,
    current_source_index: int | None = None,
    current_state: _MissionAreaState | None = None,
    source_indexes: list[int] | None = None,
) -> list[dict[str, Any]]:
    line_blocks: list[dict[str, Any]] = []
    indexes = (
        [int(idx) for idx in (source_indexes or [])]
        if source_indexes is not None
        else list(range(len(source_lines)))
    )
    for source_idx in indexes:
        if source_idx < 0 or source_idx >= len(source_lines):
            continue
        source_line, width_m = source_lines[int(source_idx)]
        working_line = source_line
        if (
            current_state is not None
            and current_source_index is not None
            and int(source_idx) == int(current_source_index)
        ):
            working_line = _trim_current_line_from_progress(
                current_state,
                working_line,
                width_m=float(width_m),
                min_length_m=float(min_length_m),
            )
        fragments = _clip_line_to_geometry_fragments(
            working_line,
            clip_geometry,
            clip_margin_m=float(clip_margin_m),
            min_length_m=float(min_length_m),
        )
        for fragment in fragments:
            coord_lists = _line_coord_lists_from_geometry(
                fragment,
                transformer,
                altitude=altitude,
                min_length_m=min_length_m,
            )
            if not coord_lists:
                continue
            coord_list = max(
                coord_lists,
                key=lambda coords: _coord_list_length_m(coords, transformer),
            )
            line_blocks.append(
                {
                    "width": float(width_m),
                    "coordinateList": coord_list,
                }
            )
    return line_blocks


def _line_preview_has_visible_delta(
    source_lines: list[BaseGeometry] | None,
    remaining_lines: list[BaseGeometry] | None,
) -> bool:
    source_segments = [
        segment
        for geometry in (source_lines or [])
        for segment in _iter_line_strings(geometry)
    ]
    remaining_segments = [
        segment
        for geometry in (remaining_lines or [])
        for segment in _iter_line_strings(geometry)
    ]
    if not source_segments or not remaining_segments:
        return False
    if len(source_segments) != len(remaining_segments):
        return True
    point_tolerance_m = 2.0
    shape_tolerance_m = 1.5
    length_tolerance_m = 3.0
    for source_line, remaining_line in zip(source_segments, remaining_segments):
        try:
            source_coords = list(source_line.coords)
            remaining_coords = list(remaining_line.coords)
        except Exception:
            return True
        if len(source_coords) < 2 or len(remaining_coords) < 2:
            continue
        direct_delta = max(
            math.hypot(
                float(source_coords[0][0]) - float(remaining_coords[0][0]),
                float(source_coords[0][1]) - float(remaining_coords[0][1]),
            ),
            math.hypot(
                float(source_coords[-1][0]) - float(remaining_coords[-1][0]),
                float(source_coords[-1][1]) - float(remaining_coords[-1][1]),
            ),
        )
        reverse_delta = max(
            math.hypot(
                float(source_coords[0][0]) - float(remaining_coords[-1][0]),
                float(source_coords[0][1]) - float(remaining_coords[-1][1]),
            ),
            math.hypot(
                float(source_coords[-1][0]) - float(remaining_coords[0][0]),
                float(source_coords[-1][1]) - float(remaining_coords[0][1]),
            ),
        )
        endpoint_delta = min(float(direct_delta), float(reverse_delta))
        if endpoint_delta > point_tolerance_m:
            return True
        try:
            if abs(float(source_line.length or 0.0) - float(remaining_line.length or 0.0)) > length_tolerance_m:
                return True
        except Exception:
            return True
        try:
            if float(source_line.hausdorff_distance(remaining_line)) > shape_tolerance_m:
                return True
        except Exception:
            return True
    return False


def _area_coord_lists_from_geometry(
    geometry: BaseGeometry,
    transformer,
    *,
    altitude: float | None,
    area_threshold_m2: float,
) -> list[list[dict[str, Any]]]:
    out: list[list[dict[str, Any]]] = []
    filtered = _filter_small_polygons(
        _fill_geometry_holes(geometry),
        area_threshold_m2=float(area_threshold_m2),
    )
    for poly in _iter_polygons(filtered):
        try:
            if float(poly.area or 0.0) < float(area_threshold_m2):
                continue
        except Exception:
            continue
        coords_llh = [
            coord
            for coord in (
                _inverse_project_coord(transformer, x_val, y_val, altitude)
                for x_val, y_val in list(poly.exterior.coords)
            )
            if coord is not None
        ]
        coords_llh = _dedupe_coord_list(coords_llh, closed=True)
        if len(coords_llh) >= 3:
            out.append(coords_llh)
    return out


def _filter_small_polygon_holes(
    geometry: BaseGeometry | None,
    *,
    area_threshold_m2: float,
) -> BaseGeometry:
    polygons = _iter_polygons(geometry)
    if not polygons:
        return geometry if geometry is not None else GeometryCollection()
    filtered: BaseGeometry = GeometryCollection()
    threshold = max(float(area_threshold_m2), 0.0)
    for poly in polygons:
        hole_coords: list[list[tuple[float, float]]] = []
        for ring in poly.interiors:
            try:
                hole_poly = Polygon(ring)
                hole_area_m2 = float(hole_poly.area or 0.0)
            except Exception:
                continue
            if hole_area_m2 < threshold:
                continue
            hole_coords.append(list(ring.coords))
        try:
            rebuilt = Polygon(list(poly.exterior.coords), holes=hole_coords)
            if not rebuilt.is_valid:
                rebuilt = rebuilt.buffer(0)
        except Exception:
            continue
        if rebuilt.is_empty:
            continue
        filtered = merge_coverage_geometry(filtered, rebuilt)
    return filtered


def _area_blocks_from_geometry(
    geometry: BaseGeometry,
    transformer,
    *,
    altitude: float | None,
    area_threshold_m2: float,
    hole_threshold_m2: float | None = None,
) -> list[dict[str, Any]]:
    filtered = _filter_small_polygons(
        geometry,
        area_threshold_m2=float(area_threshold_m2),
    )
    filtered = _filter_small_polygon_holes(
        filtered,
        area_threshold_m2=float(
            hole_threshold_m2 if hole_threshold_m2 is not None else area_threshold_m2
        ),
    )
    blocks: list[dict[str, Any]] = []
    polygons = _iter_polygons(filtered)
    polygons.sort(key=lambda poly: float(poly.area or 0.0), reverse=True)
    for poly in polygons:
        try:
            if float(poly.area or 0.0) < float(area_threshold_m2):
                continue
        except Exception:
            continue
        outer_coords = [
            coord
            for coord in (
                _inverse_project_coord(transformer, x_val, y_val, altitude)
                for x_val, y_val in list(poly.exterior.coords)
            )
            if coord is not None
        ]
        outer_coords = _dedupe_coord_list(outer_coords, closed=True)
        if len(outer_coords) >= 3:
            blocks.append(
                {
                    "isHole": False,
                    "coordinateList": outer_coords,
                }
            )
        for ring in poly.interiors:
            try:
                hole_poly = Polygon(ring)
                hole_area_m2 = float(hole_poly.area or 0.0)
            except Exception:
                continue
            threshold_m2 = float(
                hole_threshold_m2 if hole_threshold_m2 is not None else area_threshold_m2
            )
            if hole_area_m2 < threshold_m2:
                continue
            hole_coords = [
                coord
                for coord in (
                    _inverse_project_coord(transformer, x_val, y_val, altitude)
                    for x_val, y_val in list(ring.coords)
                )
                if coord is not None
            ]
            hole_coords = _dedupe_coord_list(hole_coords, closed=True)
            if len(hole_coords) >= 3:
                blocks.append(
                    {
                        "isHole": True,
                        "coordinateList": hole_coords,
                    }
                )
    return blocks


def _first_detail_coordinate_list(
    blocks: list[dict[str, Any]] | None,
    *,
    skip_holes: bool,
) -> list[dict[str, Any]]:
    for block in blocks or []:
        if not isinstance(block, dict):
            continue
        if skip_holes and bool(block.get("isHole")):
            continue
        coords = [
            item
            for item in (block.get("coordinateList") or [])
            if isinstance(item, dict)
        ]
        if coords:
            return deepcopy(coords)
    return []


def _mission_geometry_kind(mission: dict[str, Any]) -> str:
    if _as_int(mission.get("individual_mission_type")) in {5, 7}:
        return "-"
    if not bool(mission.get("is_done")) and (_as_int(mission.get("sweep_point_count")) or 0) <= 0:
        return "-"
    if mission.get("area_list") or mission.get("input_area_list"):
        return "area"
    if (
        mission.get("line_list")
        or mission.get("input_line_list")
        or mission.get("sweep_line_coordinate_lists")
    ):
        return "line"
    return "-"


def _build_split_path_source_detail(
    mission: dict[str, Any],
) -> tuple[str, dict[str, Any], MissionCoverageDefinition] | None:
    default_width_m = max(
        1.0,
        float(_as_float(mission.get("width_m")) or 0.0),
    )
    sweep_line_list: list[dict[str, Any]] = []
    for coord_items in mission.get("sweep_line_coordinate_lists") or []:
        coord_list = _dedupe_coord_list(
            [dict(item) for item in (coord_items or []) if isinstance(item, dict)],
            closed=False,
        )
        if len(coord_list) < 2:
            continue
        sweep_line_list.append(
            {
                "width": float(default_width_m),
                "coordinateList": coord_list,
            }
        )
    if not sweep_line_list:
        return None

    sweep_cov = build_mission_coverage_definition(
        {
            "line_list": sweep_line_list,
            "area_list": [],
        }
    )
    if sweep_cov is None:
        return None

    altitude = _representative_altitude_from_coords(
        [list(line.get("coordinateList") or []) for line in sweep_line_list]
        + [
            [
                coord
                for coord in (line.get("coordinateList") or [])
                if isinstance(coord, dict)
            ]
            for line in (mission.get("line_list") or [])
            if isinstance(line, dict)
        ]
        + [
            [
                coord
                for coord in (area.get("coordinateList") or [])
                if isinstance(coord, dict)
            ]
            for area in (mission.get("area_list") or [])
            if isinstance(area, dict)
        ]
        + [list(mission.get("coordinate_list") or [])]
        + [
            [
                coord
                for coord in (line.get("coordinateList") or [])
                if isinstance(coord, dict)
            ]
            for line in (mission.get("input_line_list") or [])
            if isinstance(line, dict)
        ]
        + [
            [
                coord
                for coord in (area.get("coordinateList") or [])
                if isinstance(coord, dict)
            ]
            for area in (mission.get("input_area_list") or [])
            if isinstance(area, dict)
        ]
        + [list(mission.get("input_coordinate_list") or [])]
    )

    geometry_kind = _mission_geometry_kind(mission)
    if geometry_kind == "line":
        waypoint_coords = _dedupe_coord_list(
            [
                {
                    "latitude": float(wp.get("latitude")),
                    "longitude": float(wp.get("longitude")),
                    **(
                        {"altitude": int(round(float(wp.get("altitude"))))}
                        if _as_float(wp.get("altitude")) is not None
                        else {}
                    ),
                }
                for wp in (mission.get("waypoints") or [])
                if isinstance(wp, dict)
                and _as_float(wp.get("latitude")) is not None
                and _as_float(wp.get("longitude")) is not None
            ],
            closed=False,
        )
        if len(waypoint_coords) >= 2:
            line_detail = {
                "coordinateList": deepcopy(waypoint_coords),
                "lineList": [
                    {
                        "width": float(default_width_m),
                        "coordinateList": deepcopy(waypoint_coords),
                    }
                ],
                "areaList": [],
            }
            line_cov = build_mission_coverage_definition(
                {
                    "line_list": list(line_detail["lineList"]),
                    "area_list": [],
                }
            )
            if line_cov is not None:
                return "line", line_detail, line_cov

    if geometry_kind == "area":
        area_threshold_m2 = max(
            8.0,
            float(default_width_m) * float(default_width_m) * 0.35,
        )
        hole_threshold_m2 = max(
            area_threshold_m2,
            min(float(sweep_cov.assignment_geometry.area or 0.0) * 0.002, 40.0),
        )
        area_blocks = _area_blocks_from_geometry(
            sweep_cov.assignment_geometry,
            sweep_cov.transformer,
            altitude=altitude,
            area_threshold_m2=float(area_threshold_m2),
            hole_threshold_m2=float(hole_threshold_m2),
        )
        if area_blocks:
            area_cov = build_mission_coverage_definition(
                {
                    "line_list": [],
                    "area_list": area_blocks,
                }
            ) or sweep_cov
            return (
                "area",
                {
                    "coordinateList": _first_detail_coordinate_list(area_blocks, skip_holes=True),
                    "lineList": [],
                    "areaList": area_blocks,
                },
                area_cov,
            )

    return (
        "line",
        {
            "coordinateList": deepcopy(
                sweep_line_list[0]["coordinateList"] if len(sweep_line_list) == 1 else []
            ),
            "lineList": sweep_line_list,
            "areaList": [],
        },
        sweep_cov,
    )


def _build_source_detail_from_state(state: _MissionAreaState) -> dict[str, Any]:
    if str(state.mission_type) == "line":
        line_list = deepcopy(state.input_line_list or state.source_line_list or [])
        coordinate_list = _first_detail_coordinate_list(line_list, skip_holes=False)
        if not coordinate_list:
            coordinate_list = deepcopy(
                [
                    item
                    for item in (state.input_coordinate_list or state.source_coordinate_list or [])
                    if isinstance(item, dict)
                ]
            )
        if not line_list and len(coordinate_list) >= 2:
            line_list = [
                {
                    "width": float(state.width_hint_m or _DEFAULT_STRIP_WIDTH_M),
                    "coordinateList": deepcopy(coordinate_list),
                }
            ]
        return {
            "coordinateList": coordinate_list,
            "lineList": line_list,
            "areaList": [],
        }
    area_list = deepcopy(state.source_area_list or state.input_area_list or [])
    coordinate_list = _first_detail_coordinate_list(area_list, skip_holes=True)
    return {
        "coordinateList": coordinate_list,
        "lineList": [],
        "areaList": area_list,
    }


def _project_coordinate_overlay_points(
    transformer,
    coordinate_list: list[dict[str, Any]] | None,
) -> list[dict[str, Any]]:
    points: list[dict[str, Any]] = []
    for index, item in enumerate(coordinate_list or [], start=1):
        coord = _coord(item)
        xy = _project_xy(transformer, coord)
        if coord is None or xy is None:
            continue
        points.append(
            {
                "index": int(index),
                "x": float(xy[0]),
                "y": float(xy[1]),
                "latitude": float(coord["latitude"]),
                "longitude": float(coord["longitude"]),
                "altitude": _as_float(coord.get("altitude")),
            }
        )
    return points


def _build_coordinate_overlay_groups(
    state: _MissionAreaState,
    *,
    source_detail: dict[str, Any],
    remaining_detail: dict[str, Any],
) -> list[dict[str, Any]]:
    groups: list[dict[str, Any]] = []

    def _append_groups(
        role: str,
        geometry_type: str,
        blocks: list[dict[str, Any]] | None,
    ) -> None:
        for block_index, block in enumerate(blocks or [], start=1):
            if not isinstance(block, dict):
                continue
            coordinate_list = [
                item
                for item in (block.get("coordinateList") or [])
                if isinstance(item, dict)
            ]
            points = _project_coordinate_overlay_points(
                state.coverage_def.transformer,
                coordinate_list,
            )
            min_points = 2 if geometry_type == "line" else 3
            if len(points) < min_points:
                continue
            groups.append(
                {
                    "role": str(role),
                    "geometry_type": str(geometry_type),
                    "block_index": int(block_index),
                    "is_hole": bool(block.get("isHole")),
                    "points": points,
                }
            )

    _append_groups("source", "line", list((source_detail or {}).get("lineList") or []))
    _append_groups("remaining", "line", list((remaining_detail or {}).get("lineList") or []))
    _append_groups("source", "area", list((source_detail or {}).get("areaList") or []))
    _append_groups("remaining", "area", list((remaining_detail or {}).get("areaList") or []))
    return groups


def _build_planned_sweep_lines(
    mission: dict[str, Any],
    coverage_def: MissionCoverageDefinition,
) -> tuple[list[BaseGeometry], float]:
    raw_lists = mission.get("sweep_line_coordinate_lists") or []
    planned_lines: list[BaseGeometry] = []
    spacing_samples: list[float] = []

    for coord_list in raw_lists:
        if not isinstance(coord_list, list):
            continue
        points = [
            xy
            for xy in (
                _project_xy(coverage_def.transformer, _coord(item))
                for item in coord_list
            )
            if xy is not None
        ]
        if len(points) < 2:
            continue
        segments: list[tuple[tuple[float, float], tuple[float, float], float, tuple[float, float]]] = []
        max_len = 0.0
        for idx in range(1, len(points)):
            start_xy = points[idx - 1]
            end_xy = points[idx]
            dx = end_xy[0] - start_xy[0]
            dy = end_xy[1] - start_xy[1]
            seg_len = math.hypot(dx, dy)
            if seg_len <= 0.5:
                continue
            seg_vec = _normalize_vector(dx, dy)
            if seg_vec is None:
                continue
            max_len = max(max_len, seg_len)
            segments.append((start_xy, end_xy, seg_len, seg_vec))
        if not segments or max_len <= 0.0:
            continue
        length_threshold = max(10.0, max_len * 0.45)
        direction_cos_threshold = math.cos(math.radians(28.0))
        group_points: list[tuple[float, float]] = []
        group_vec: tuple[float, float] | None = None
        group_best_len = 0.0

        def _flush_group() -> None:
            nonlocal group_points, group_vec, group_best_len
            if len(group_points) < 2:
                group_points = []
                group_vec = None
                group_best_len = 0.0
                return
            line = LineString([group_points[0], group_points[-1]])
            clipped = coverage_def.assignment_geometry.intersection(line)
            if not clipped.is_empty and float(getattr(clipped, "length", 0.0) or 0.0) > 3.0:
                planned_lines.append(clipped)
            group_points = []
            group_vec = None
            group_best_len = 0.0

        for start_xy, end_xy, seg_len, seg_vec in segments:
            if seg_len < length_threshold:
                _flush_group()
                continue
            if not group_points:
                group_points = [start_xy, end_xy]
                group_vec = seg_vec
                group_best_len = seg_len
                continue
            dot = (float(seg_vec[0]) * float(group_vec[0]) + float(seg_vec[1]) * float(group_vec[1])) if group_vec is not None else 0.0
            if dot >= direction_cos_threshold:
                group_points.append(end_xy)
                if seg_len > group_best_len:
                    group_best_len = seg_len
                    group_vec = seg_vec
            else:
                _flush_group()
                group_points = [start_xy, end_xy]
                group_vec = seg_vec
                group_best_len = seg_len
        _flush_group()

    filtered_lines: list[BaseGeometry] = []
    seen_keys: set[tuple[int, int, int, int]] = set()
    for line in planned_lines:
        if line.is_empty:
            continue
        for child in _iter_line_strings(line):
            coords = list(child.coords)
            if len(coords) < 2:
                continue
            start_xy = coords[0]
            end_xy = coords[-1]
            line_len = math.hypot(end_xy[0] - start_xy[0], end_xy[1] - start_xy[1])
            if line_len <= 3.0:
                continue
            sx = int(round(start_xy[0] * 10.0))
            sy = int(round(start_xy[1] * 10.0))
            ex = int(round(end_xy[0] * 10.0))
            ey = int(round(end_xy[1] * 10.0))
            key = (sx, sy, ex, ey)
            reverse_key = (ex, ey, sx, sy)
            if key in seen_keys or reverse_key in seen_keys:
                continue
            seen_keys.add(key)
            filtered_lines.append(LineString([start_xy, end_xy]))

    for idx in range(1, len(filtered_lines)):
        try:
            spacing = float(filtered_lines[idx - 1].distance(filtered_lines[idx]))
        except Exception:
            spacing = 0.0
        if spacing > 0.5:
            spacing_samples.append(spacing)

    if spacing_samples:
        spacing_samples.sort()
        median_spacing = spacing_samples[len(spacing_samples) // 2]
        cut_half_width_m = max(1.0, min(float(median_spacing) * 0.42, 12.0))
    else:
        cut_half_width_m = max(1.0, min(float(_as_float(mission.get("width_m")) or 8.0) * 0.12, 8.0))

    return filtered_lines, float(cut_half_width_m)


def _format_area(value: float | None) -> str:
    amount = float(value or 0.0)
    if amount >= 1_000_000.0:
        return f"{amount / 1_000_000.0:.2f} km²"
    if amount >= 10_000.0:
        return f"{amount / 10_000.0:.2f} ha"
    return f"{amount:.0f} m²"


@dataclass
class _MissionAreaState:
    mission_id: int
    aircraft_id: int
    input_id: int | None
    mission_type: str
    coverage_def: MissionCoverageDefinition
    width_hint_m: float | None
    assignment_geometry: BaseGeometry
    planned_area_m2: float
    source_line_list: list[dict[str, Any]] = field(default_factory=list)
    source_area_list: list[dict[str, Any]] = field(default_factory=list)
    source_coordinate_list: list[dict[str, Any]] = field(default_factory=list)
    input_line_list: list[dict[str, Any]] = field(default_factory=list)
    input_area_list: list[dict[str, Any]] = field(default_factory=list)
    input_coordinate_list: list[dict[str, Any]] = field(default_factory=list)
    is_current: bool = False
    covered_geometry: BaseGeometry = field(default_factory=_gc)
    centerline_points: list[tuple[float, float]] = field(default_factory=list)
    cut_lines: list[BaseGeometry] = field(default_factory=list)
    planned_cut_lines: list[BaseGeometry] = field(default_factory=list)
    cut_half_width_m: float = 2.0
    last_cut_line_index: int = -1
    completed_cut_line_indexes: set[int] = field(default_factory=set)
    progress_origin_line_index: int | None = None
    progress_boundary_line_index: int | None = None
    progress_direction_sign: int | None = None
    last_nearest_cut_line_index: int | None = None
    tracking_cut_line_index: int | None = None
    tracking_projection_min_m: float | None = None
    tracking_projection_max_m: float | None = None
    tracking_sample_count: int = 0
    tracking_path_length_m: float = 0.0
    tracking_last_center_xy: tuple[float, float] | None = None
    provisional_frontier_line_index: int | None = None
    preferred_track_vector: tuple[float, float] | None = None
    last_center_xy: tuple[float, float] | None = None
    last_update_ms: int | None = None
    done: bool = False


def _planned_cut_strip(state: _MissionAreaState, planned_line: BaseGeometry) -> BaseGeometry:
    line_index = -1
    for idx, candidate in enumerate(state.planned_cut_lines):
        if candidate is planned_line:
            line_index = idx
            break
    half_width_m = float(state.cut_half_width_m)
    if line_index >= 0:
        half_width_m = _planned_line_half_width_m(state, line_index)
    return state.assignment_geometry.intersection(
        planned_line.buffer(
            max(0.8, float(half_width_m)),
            cap_style=2,
            join_style=2,
        )
    )


def _planned_line_overlap_metrics(
    state: _MissionAreaState,
    *,
    line_index: int,
    footprint_geometry: BaseGeometry | None,
    width_m: float | None,
) -> tuple[float, float, float, float, float, float]:
    footprint_area_m2 = 0.0
    if footprint_geometry is not None and not footprint_geometry.is_empty:
        try:
            footprint_area_m2 = float(footprint_geometry.area or 0.0)
        except Exception:
            footprint_area_m2 = 0.0
    threshold_m2 = _observation_overlap_threshold_m2(
        state,
        width_m=width_m,
        footprint_area_m2=footprint_area_m2,
    )
    if (
        footprint_geometry is None
        or footprint_geometry.is_empty
        or line_index < 0
        or line_index >= len(state.planned_cut_lines)
    ):
        return 0.0, 0.0, footprint_area_m2, 0.0, 0.0, threshold_m2
    strip = _planned_cut_strip(
        state,
        state.planned_cut_lines[int(line_index)],
    )
    strip_area_m2 = 0.0
    try:
        strip_area_m2 = float(strip.area or 0.0)
    except Exception:
        strip_area_m2 = 0.0
    overlap_area_m2 = _geometry_intersection_area_m2(
        footprint_geometry,
        strip,
    )
    strip_overlap_ratio = (
        float(overlap_area_m2) / float(strip_area_m2)
        if strip_area_m2 > 1e-6
        else 0.0
    )
    footprint_overlap_ratio = (
        float(overlap_area_m2) / float(footprint_area_m2)
        if footprint_area_m2 > 1e-6
        else 0.0
    )
    return (
        float(overlap_area_m2),
        float(strip_area_m2),
        float(footprint_area_m2),
        float(strip_overlap_ratio),
        float(footprint_overlap_ratio),
        float(threshold_m2),
    )


def _coord_list_length_m(
    coord_list: list[dict[str, Any]] | None,
    transformer,
) -> float:
    points = [
        xy
        for xy in (_project_xy(transformer, _coord(item)) for item in (coord_list or []))
        if xy is not None
    ]
    if len(points) < 2:
        return 0.0
    total = 0.0
    for idx in range(1, len(points)):
        total += math.hypot(
            float(points[idx][0]) - float(points[idx - 1][0]),
            float(points[idx][1]) - float(points[idx - 1][1]),
        )
    return float(total)


def _merge_state_geometries(geometries: list[BaseGeometry]) -> BaseGeometry:
    merged: BaseGeometry = GeometryCollection()
    for geometry in geometries:
        if geometry is None or geometry.is_empty:
            continue
        merged = merge_coverage_geometry(merged, geometry)
    return merged


def _line_block_lateral_sort_key(
    line_block: dict[str, Any],
    transformer,
    track_vector: tuple[float, float] | None,
) -> tuple[float, float]:
    coords = line_block.get("coordinateList") or []
    points = [
        xy
        for xy in (_project_xy(transformer, _coord(item)) for item in coords)
        if xy is not None
    ]
    if not points:
        return (0.0, 0.0)
    cx = sum(float(pt[0]) for pt in points) / len(points)
    cy = sum(float(pt[1]) for pt in points) / len(points)
    if track_vector is None:
        return (cy, cx)
    tx, ty = float(track_vector[0]), float(track_vector[1])
    nx, ny = -ty, tx
    lateral = (cx * nx) + (cy * ny)
    along = (cx * tx) + (cy * ty)
    return (lateral, along)


def _line_block_start_distance_to_current_m(
    coord_list: list[dict[str, Any]] | None,
    transformer,
    current_xy: tuple[float, float] | None,
) -> float:
    if current_xy is None:
        return 0.0
    points = [
        xy
        for xy in (_project_xy(transformer, _coord(item)) for item in (coord_list or []))
        if xy is not None
    ]
    if not points:
        return float("inf")
    start_xy = points[0]
    return float(
        math.hypot(
            float(start_xy[0]) - float(current_xy[0]),
            float(start_xy[1]) - float(current_xy[1]),
        )
    )


def _state_progress_vector(state: _MissionAreaState) -> tuple[float, float] | None:
    boundary_index = state.progress_boundary_line_index
    if boundary_index is not None:
        vec = _planned_line_progress_vector(state, int(boundary_index))
        if vec is not None:
            return vec
    if len(state.centerline_points) >= 2:
        prev_xy = state.centerline_points[-2]
        curr_xy = state.centerline_points[-1]
        vec = _normalize_vector(
            float(curr_xy[0]) - float(prev_xy[0]),
            float(curr_xy[1]) - float(prev_xy[1]),
        )
        if vec is not None:
            return vec
    return state.preferred_track_vector


def _orient_line_toward_progress(
    line: LineString,
    progress_vec: tuple[float, float] | None,
) -> LineString:
    coords = list(line.coords)
    if len(coords) < 2 or progress_vec is None:
        return line
    line_vec = _normalize_vector(
        float(coords[-1][0]) - float(coords[0][0]),
        float(coords[-1][1]) - float(coords[0][1]),
    )
    if line_vec is None:
        return line
    dot = (float(line_vec[0]) * float(progress_vec[0])) + (float(line_vec[1]) * float(progress_vec[1]))
    if dot >= 0.0:
        return line
    try:
        return LineString(list(reversed(coords)))
    except Exception:
        return line


def _substring_line(
    line: LineString,
    start_m: float,
    end_m: float,
) -> BaseGeometry:
    try:
        total_length = float(line.length or 0.0)
    except Exception:
        total_length = 0.0
    if total_length <= 1e-6:
        return GeometryCollection()
    start_d = max(0.0, min(float(start_m), total_length))
    end_d = max(0.0, min(float(end_m), total_length))
    if end_d <= start_d + 1e-6:
        return GeometryCollection()
    coords = list(line.coords)
    if len(coords) < 2:
        return GeometryCollection()

    collected: list[tuple[float, float]] = []
    walked = 0.0
    for idx in range(1, len(coords)):
        ax, ay = coords[idx - 1]
        bx, by = coords[idx]
        seg_len = math.hypot(float(bx) - float(ax), float(by) - float(ay))
        if seg_len <= 1e-6:
            continue
        seg_start = walked
        seg_end = walked + seg_len
        if seg_end < start_d - 1e-6:
            walked = seg_end
            continue
        if seg_start > end_d + 1e-6:
            break
        local_start = max(start_d, seg_start)
        local_end = min(end_d, seg_end)
        start_ratio = max(0.0, min(1.0, (local_start - seg_start) / seg_len))
        end_ratio = max(0.0, min(1.0, (local_end - seg_start) / seg_len))
        start_xy = (
            float(ax) + (float(bx) - float(ax)) * start_ratio,
            float(ay) + (float(by) - float(ay)) * start_ratio,
        )
        end_xy = (
            float(ax) + (float(bx) - float(ax)) * end_ratio,
            float(ay) + (float(by) - float(ay)) * end_ratio,
        )
        if not collected or math.hypot(
            float(collected[-1][0]) - float(start_xy[0]),
            float(collected[-1][1]) - float(start_xy[1]),
        ) > 1e-6:
            collected.append(start_xy)
        if math.hypot(
            float(collected[-1][0]) - float(end_xy[0]),
            float(collected[-1][1]) - float(end_xy[1]),
        ) > 1e-6:
            collected.append(end_xy)
        walked = seg_end
    if len(collected) < 2:
        return GeometryCollection()
    try:
        return LineString(collected)
    except Exception:
        return GeometryCollection()


def _trim_current_line_from_progress(
    state: _MissionAreaState,
    line: LineString,
    *,
    width_m: float,
    min_length_m: float,
) -> LineString:
    if not bool(state.is_current):
        return line
    if state.last_center_xy is None:
        return line
    point = Point(state.last_center_xy)
    try:
        distance_m = float(line.distance(point))
    except Exception:
        return line
    distance_limit_m = max(
        18.0,
        float(width_m) * 0.9,
        float(state.cut_half_width_m) * 1.6,
    )
    if distance_m > distance_limit_m:
        return line
    progress_vec = _state_progress_vector(state)
    oriented = _orient_line_toward_progress(line, progress_vec)
    try:
        projection_m = float(oriented.project(point))
        total_length_m = float(oriented.length or 0.0)
    except Exception:
        return oriented
    if total_length_m <= 1e-6:
        return oriented
    backtrack_m = max(
        2.0,
        min(
            max(float(width_m) * 0.06, float(state.cut_half_width_m) * 0.12),
            12.0,
        ),
    )
    trimmed = _substring_line(
        oriented,
        max(0.0, projection_m - backtrack_m),
        total_length_m,
    )
    for candidate in _iter_line_strings(trimmed):
        try:
            if float(candidate.length or 0.0) >= float(min_length_m):
                return candidate
        except Exception:
            continue
    return oriented


def _commit_planned_cut_line(state: _MissionAreaState, index: int) -> None:
    if index < 0 or index >= len(state.planned_cut_lines):
        return
    if int(index) in state.completed_cut_line_indexes:
        return
    planned_line = state.planned_cut_lines[int(index)]
    clipped_strip = _planned_cut_strip(state, planned_line)
    state.covered_geometry = merge_coverage_geometry(
        state.covered_geometry,
        clipped_strip,
    )
    if not planned_line.is_empty:
        state.cut_lines.append(planned_line)
        if len(state.cut_lines) > 800:
            state.cut_lines = state.cut_lines[-800:]
    state.completed_cut_line_indexes.add(int(index))
    state.last_cut_line_index = max(int(state.last_cut_line_index), int(index))


def _restore_planned_cut_progress(state: _MissionAreaState) -> None:
    if state.covered_geometry.is_empty or not state.planned_cut_lines:
        return
    restored_lines: list[BaseGeometry] = []
    restored_indexes: set[int] = set()
    for idx, planned_line in enumerate(state.planned_cut_lines):
        try:
            strip = _planned_cut_strip(state, planned_line)
        except Exception:
            continue
        if strip.is_empty:
            continue
        try:
            strip_area = float(strip.area or 0.0)
            overlap_area = float(state.covered_geometry.intersection(strip).area or 0.0)
        except Exception:
            continue
        if strip_area <= 1e-3:
            continue
        if overlap_area >= max(strip_area * 0.35, 4.0):
            restored_indexes.add(int(idx))
            if not planned_line.is_empty:
                restored_lines.append(planned_line)
    if restored_indexes:
        state.completed_cut_line_indexes = restored_indexes
        state.cut_lines = restored_lines[-800:]
        state.last_cut_line_index = max(restored_indexes)
        state.progress_origin_line_index = min(restored_indexes)
        state.progress_boundary_line_index = max(restored_indexes)
        state.progress_direction_sign = 1
        _rebuild_completed_sweep_coverage(state)


def _reset_planned_line_tracking(
    state: _MissionAreaState,
    *,
    line_index: int | None = None,
    projection_m: float | None = None,
) -> None:
    state.tracking_cut_line_index = None if line_index is None else int(line_index)
    state.tracking_projection_min_m = None if projection_m is None else float(projection_m)
    state.tracking_projection_max_m = None if projection_m is None else float(projection_m)
    state.tracking_sample_count = 0 if line_index is None else 1
    state.tracking_path_length_m = 0.0
    state.tracking_last_center_xy = None


def _planned_line_projection_m(
    state: _MissionAreaState,
    index: int,
    point_xy: tuple[float, float],
) -> float | None:
    if index < 0 or index >= len(state.planned_cut_lines):
        return None
    line_strings = _iter_line_strings(state.planned_cut_lines[int(index)])
    if not line_strings:
        return None
    try:
        return float(line_strings[0].project(Point(point_xy)))
    except Exception:
        return None


def _planned_line_distance_m(
    state: _MissionAreaState,
    index: int,
    point: Point,
) -> float | None:
    if index < 0 or index >= len(state.planned_cut_lines):
        return None
    try:
        return float(state.planned_cut_lines[int(index)].distance(point))
    except Exception:
        return None


def _select_effective_cut_line_index(
    state: _MissionAreaState,
    *,
    nearest_index: int,
    center_point: Point,
    distance_limit_m: float,
) -> int | None:
    if not state.planned_cut_lines:
        return None
    max_index = len(state.planned_cut_lines) - 1
    nearest_index = max(0, min(int(nearest_index), max_index))
    tracking_index = state.tracking_cut_line_index
    if tracking_index is not None and abs(int(nearest_index) - int(tracking_index)) <= 1:
        tracking_distance = _planned_line_distance_m(state, int(tracking_index), center_point)
        if tracking_distance is not None and tracking_distance <= (float(distance_limit_m) * 0.85):
            return int(tracking_index)
    frontier_index = (
        state.progress_boundary_line_index
        if state.progress_boundary_line_index is not None
        else state.progress_origin_line_index
    )
    if frontier_index is None:
        return int(nearest_index)
    frontier_index = max(0, min(int(frontier_index), max_index))
    direction_sign = 1 if state.progress_direction_sign is None else (1 if int(state.progress_direction_sign) >= 0 else -1)
    nearby_candidates = [
        frontier_index,
        frontier_index + int(direction_sign),
        frontier_index + (int(direction_sign) * 2),
        frontier_index - int(direction_sign),
    ]
    best_index: int | None = None
    best_distance: float | None = None
    for candidate in nearby_candidates:
        if candidate < 0 or candidate > max_index:
            continue
        candidate_distance = _planned_line_distance_m(state, int(candidate), center_point)
        if candidate_distance is None:
            continue
        if candidate_distance > (float(distance_limit_m) * 1.15):
            continue
        if best_distance is None or candidate_distance < best_distance:
            best_distance = candidate_distance
            best_index = int(candidate)
    if best_index is not None:
        return best_index
    if abs(int(nearest_index) - int(frontier_index)) <= 2:
        return int(nearest_index)
    return None


def _resolve_commit_line_index(
    state: _MissionAreaState,
    *,
    tracked_index: int,
    center_point: Point,
    distance_limit_m: float,
) -> int:
    max_index = len(state.planned_cut_lines) - 1
    tracked_index = max(0, min(int(tracked_index), max_index))
    boundary_index = state.progress_boundary_line_index
    direction_sign = 1 if state.progress_direction_sign is None else (1 if int(state.progress_direction_sign) >= 0 else -1)
    if boundary_index is None:
        return int(tracked_index)
    boundary_index = max(0, min(int(boundary_index), max_index))
    next_index = boundary_index + direction_sign
    if next_index < 0 or next_index > max_index:
        return int(boundary_index)
    signed_delta = (int(tracked_index) - int(boundary_index)) * int(direction_sign)
    if signed_delta > 1:
        return int(next_index)
    if signed_delta == 1:
        return int(next_index)
    if signed_delta < 0:
        return int(boundary_index)
    current_distance = _planned_line_distance_m(state, int(boundary_index), center_point)
    next_distance = _planned_line_distance_m(state, int(next_index), center_point)
    lateral_margin_m = max(2.0, float(state.cut_half_width_m) * 0.8)
    if (
        next_distance is not None
        and next_distance <= float(distance_limit_m) * 1.35
        and (
            current_distance is None
            or next_distance <= current_distance + lateral_margin_m
        )
    ):
        return int(next_index)
    return int(boundary_index)


def _contiguous_boundary_index(
    origin_index: int,
    boundary_index: int,
    direction_sign: int,
    completed_indexes: set[int],
) -> int:
    direction = 1 if int(direction_sign) >= 0 else -1
    boundary = int(boundary_index)
    floor_index = min(int(origin_index), int(boundary_index))
    ceil_index = max(int(origin_index), int(boundary_index))
    while True:
        next_index = boundary + direction
        if next_index in completed_indexes:
            boundary = int(next_index)
            floor_index = min(floor_index, boundary)
            ceil_index = max(ceil_index, boundary)
            continue
        break
    return int(boundary)


def _footprint_strongly_supports_planned_line(
    state: _MissionAreaState,
    *,
    line_index: int,
    center_point: Point,
    footprint_geometry: BaseGeometry | None,
    width_m: float | None,
    nearest_distance_m: float | None,
) -> bool:
    if footprint_geometry is None or footprint_geometry.is_empty:
        return False
    if line_index < 0 or line_index >= len(state.planned_cut_lines):
        return False
    distance_m = nearest_distance_m
    if distance_m is None:
        distance_m = _planned_line_distance_m(state, int(line_index), center_point)
    if distance_m is not None:
        distance_limit_m = max(14.0, float(state.cut_half_width_m) * 3.0)
        if float(distance_m) > distance_limit_m:
            return False
    (
        overlap_area_m2,
        strip_area_m2,
        footprint_area_m2,
        strip_overlap_ratio,
        footprint_overlap_ratio,
        threshold_m2,
    ) = _planned_line_overlap_metrics(
        state,
        line_index=int(line_index),
        footprint_geometry=footprint_geometry,
        width_m=width_m,
    )
    if overlap_area_m2 <= 1e-6 or footprint_area_m2 <= 1e-6:
        return False
    strong_area_threshold_m2 = max(float(threshold_m2) * 1.6, 45.0)
    if strip_area_m2 > 1e-6:
        strong_area_threshold_m2 = max(
            strong_area_threshold_m2,
            min(float(strip_area_m2) * 0.18, 360.0),
        )
    if overlap_area_m2 >= strong_area_threshold_m2:
        return True
    if (
        strip_overlap_ratio >= 0.30
        and footprint_overlap_ratio >= 0.10
        and overlap_area_m2 >= float(threshold_m2)
    ):
        return True
    return (
        footprint_overlap_ratio >= 0.24
        and overlap_area_m2 >= float(threshold_m2) * 1.2
    )


def _single_sample_line_commit_allowed(
    state: _MissionAreaState,
    *,
    line_index: int,
    center_point: Point,
    footprint_geometry: BaseGeometry | None,
    width_m: float | None,
    nearest_distance_m: float | None,
) -> bool:
    if str(state.mission_type) != "line":
        return False
    if line_index < 0 or line_index >= len(state.planned_cut_lines):
        return False
    if int(line_index) in state.completed_cut_line_indexes:
        return False
    if not _footprint_strongly_supports_planned_line(
        state,
        line_index=int(line_index),
        center_point=center_point,
        footprint_geometry=footprint_geometry,
        width_m=width_m,
        nearest_distance_m=nearest_distance_m,
    ):
        return False
    frontier_index = (
        state.progress_boundary_line_index
        if state.progress_boundary_line_index is not None
        else state.progress_origin_line_index
    )
    if frontier_index is None:
        return True
    max_index = len(state.planned_cut_lines) - 1
    frontier_index = max(0, min(int(frontier_index), max_index))
    direction_sign = 1 if state.progress_direction_sign is None else (1 if int(state.progress_direction_sign) >= 0 else -1)
    signed_delta = (int(line_index) - int(frontier_index)) * int(direction_sign)
    return 0 <= signed_delta <= 2


def _commit_observed_planned_line(
    state: _MissionAreaState,
    *,
    tracked_index: int,
    center_point: Point,
    distance_limit_m: float,
) -> bool:
    if not state.planned_cut_lines:
        return False
    commit_index = _resolve_commit_line_index(
        state,
        tracked_index=int(tracked_index),
        center_point=center_point,
        distance_limit_m=float(distance_limit_m),
    )
    if commit_index < 0 or commit_index >= len(state.planned_cut_lines):
        return False
    changed = False
    if state.progress_origin_line_index is None:
        state.progress_origin_line_index = int(commit_index)
        state.progress_boundary_line_index = int(commit_index)
        state.progress_direction_sign = 1
        changed = True
    else:
        origin_index = int(state.progress_origin_line_index)
        current_boundary_index = int(
            state.progress_boundary_line_index
            if state.progress_boundary_line_index is not None
            else origin_index
        )
        if state.progress_direction_sign is None:
            state.progress_direction_sign = 1
        direction_sign = state.progress_direction_sign
        if direction_sign is None:
            if abs(int(commit_index) - int(origin_index)) <= 1 and current_boundary_index != commit_index:
                state.progress_boundary_line_index = int(commit_index)
                changed = True
        else:
            contiguous_boundary = _contiguous_boundary_index(
                origin_index=int(origin_index),
                boundary_index=int(current_boundary_index),
                direction_sign=int(direction_sign),
                completed_indexes=set(state.completed_cut_line_indexes) | {int(commit_index)},
            )
            if contiguous_boundary != current_boundary_index:
                state.progress_boundary_line_index = int(contiguous_boundary)
                changed = True
    completed_indexes = set(state.completed_cut_line_indexes)
    completed_indexes.add(int(commit_index))
    if completed_indexes != state.completed_cut_line_indexes:
        state.completed_cut_line_indexes = completed_indexes
        changed = True
    return bool(changed)


def _update_planned_line_tracking(
    state: _MissionAreaState,
    *,
    line_index: int,
    center_xy: tuple[float, float],
) -> bool:
    projection_m = _planned_line_projection_m(state, line_index, center_xy)
    if projection_m is None:
        _reset_planned_line_tracking(state)
        return False
    if state.tracking_cut_line_index != int(line_index):
        _reset_planned_line_tracking(
            state,
            line_index=int(line_index),
            projection_m=float(projection_m),
        )
        state.tracking_last_center_xy = center_xy
        return False
    if state.tracking_last_center_xy is not None:
        state.tracking_path_length_m += math.hypot(
            float(center_xy[0] - state.tracking_last_center_xy[0]),
            float(center_xy[1] - state.tracking_last_center_xy[1]),
        )
    state.tracking_last_center_xy = center_xy
    state.tracking_sample_count = min(int(state.tracking_sample_count) + 1, 128)
    if state.tracking_projection_min_m is None:
        state.tracking_projection_min_m = float(projection_m)
    else:
        state.tracking_projection_min_m = min(float(state.tracking_projection_min_m), float(projection_m))
    if state.tracking_projection_max_m is None:
        state.tracking_projection_max_m = float(projection_m)
    else:
        state.tracking_projection_max_m = max(float(state.tracking_projection_max_m), float(projection_m))
    line_strings = _iter_line_strings(state.planned_cut_lines[int(line_index)])
    if not line_strings:
        return False
    commit_threshold_m = _planned_line_commit_threshold_m(state, line_strings[0])
    projection_span_m = max(
        0.0,
        float(state.tracking_projection_max_m or 0.0) - float(state.tracking_projection_min_m or 0.0),
    )
    path_threshold_m = max(4.0, float(commit_threshold_m) * 0.32)
    span_threshold_m = max(3.0, float(commit_threshold_m) * 0.22)
    confirm_path_threshold_m = max(7.0, float(commit_threshold_m) * 0.5)
    confirm_span_threshold_m = max(5.0, float(commit_threshold_m) * 0.35)
    if (
        int(state.tracking_sample_count) >= 2
        and float(state.tracking_path_length_m) >= path_threshold_m
        and projection_span_m >= span_threshold_m
    ):
        return True
    if (
        int(state.tracking_sample_count) >= 3
        and float(state.tracking_path_length_m) >= confirm_path_threshold_m
        and projection_span_m >= confirm_span_threshold_m
    ):
        return True
    return False


def _planned_line_commit_threshold_m(state: _MissionAreaState, planned_line: BaseGeometry) -> float:
    line_length = float(getattr(planned_line, "length", 0.0) or 0.0)
    base_threshold = max(
        12.0,
        min(line_length * 0.14, 45.0),
        float(state.cut_half_width_m) * 3.0,
    )
    if line_length > 1.0:
        return min(base_threshold, max(4.0, line_length * 0.75))
    return 4.0


def _planned_line_half_width_m(state: _MissionAreaState, index: int) -> float:
    if index < 0 or index >= len(state.planned_cut_lines):
        return max(0.8, float(state.cut_half_width_m))
    planned_line = state.planned_cut_lines[int(index)]
    local_samples: list[float] = []
    if index > 0:
        try:
            gap_prev = float(planned_line.distance(state.planned_cut_lines[index - 1]))
        except Exception:
            gap_prev = 0.0
        if gap_prev > 0.8:
            local_samples.append(gap_prev * 0.58)
    if index + 1 < len(state.planned_cut_lines):
        try:
            gap_next = float(planned_line.distance(state.planned_cut_lines[index + 1]))
        except Exception:
            gap_next = 0.0
        if gap_next > 0.8:
            local_samples.append(gap_next * 0.58)
    if local_samples:
        local_samples.sort()
        return max(float(state.cut_half_width_m), min(local_samples[-1], 28.0))
    return max(0.8, float(state.cut_half_width_m))


def _fill_between_planned_lines(
    state: _MissionAreaState,
    first_index: int,
    second_index: int,
) -> BaseGeometry:
    if first_index < 0 or second_index < 0:
        return GeometryCollection()
    if first_index >= len(state.planned_cut_lines) or second_index >= len(state.planned_cut_lines):
        return GeometryCollection()
    first_lines = _iter_line_strings(state.planned_cut_lines[int(first_index)])
    second_lines = _iter_line_strings(state.planned_cut_lines[int(second_index)])
    if not first_lines or not second_lines:
        return GeometryCollection()
    first_coords = list(first_lines[0].coords)
    second_coords = list(second_lines[0].coords)
    if len(first_coords) < 2 or len(second_coords) < 2:
        return GeometryCollection()
    same_dir_cost = (
        math.hypot(first_coords[0][0] - second_coords[0][0], first_coords[0][1] - second_coords[0][1])
        + math.hypot(first_coords[-1][0] - second_coords[-1][0], first_coords[-1][1] - second_coords[-1][1])
    )
    reverse_dir_cost = (
        math.hypot(first_coords[0][0] - second_coords[-1][0], first_coords[0][1] - second_coords[-1][1])
        + math.hypot(first_coords[-1][0] - second_coords[0][0], first_coords[-1][1] - second_coords[0][1])
    )
    if reverse_dir_cost < same_dir_cost:
        second_coords = list(reversed(second_coords))
    shell = list(first_coords) + list(reversed(second_coords))
    if len(shell) < 4:
        return GeometryCollection()
    try:
        bridge = Polygon(shell)
        if not bridge.is_valid:
            bridge = bridge.buffer(0)
    except Exception:
        return GeometryCollection()
    if bridge.is_empty:
        return GeometryCollection()
    try:
        gap_distance = float(first_lines[0].distance(second_lines[0]))
        curve_margin = max(1.0, gap_distance * 0.18)
        bridge = bridge.buffer(curve_margin, join_style=2)
        if not bridge.is_valid:
            bridge = bridge.buffer(0)
    except Exception:
        pass
    try:
        clipped = state.assignment_geometry.intersection(bridge)
    except Exception:
        clipped = GeometryCollection()
    return clipped if clipped is not None else GeometryCollection()


def _terminal_cap_geometry(
    state: _MissionAreaState,
    index: int,
    *,
    toward_start: bool,
) -> BaseGeometry:
    if index < 0 or index >= len(state.planned_cut_lines):
        return GeometryCollection()
    base_lines = _iter_line_strings(state.planned_cut_lines[int(index)])
    if not base_lines:
        return GeometryCollection()
    base_coords = list(base_lines[0].coords)
    if len(base_coords) < 2:
        return GeometryCollection()
    progress_vec = _planned_line_progress_vector(state, int(index))
    if progress_vec is None:
        return GeometryCollection()
    direction = -1.0 if toward_start else 1.0
    shift_vec = (float(progress_vec[0]) * direction, float(progress_vec[1]) * direction)
    gap_samples: list[float] = []
    neighbor_indexes = [index - 1, index + 1]
    for neighbor_index in neighbor_indexes:
        if neighbor_index < 0 or neighbor_index >= len(state.planned_cut_lines):
            continue
        try:
            gap = float(state.planned_cut_lines[int(index)].distance(state.planned_cut_lines[int(neighbor_index)]))
        except Exception:
            gap = 0.0
        if gap > 0.6:
            gap_samples.append(gap)
    if gap_samples:
        gap_samples.sort()
        cap_depth_m = max(float(state.cut_half_width_m) * 1.2, gap_samples[len(gap_samples) // 2] * 1.1)
    else:
        cap_depth_m = max(float(state.cut_half_width_m) * 2.0, 8.0)
    shifted_coords = [
        (
            float(x_val) + shift_vec[0] * float(cap_depth_m),
            float(y_val) + shift_vec[1] * float(cap_depth_m),
        )
        for x_val, y_val in base_coords
    ]
    try:
        cap_line = LineString(shifted_coords)
        bridge = Polygon(list(base_coords) + list(reversed(shifted_coords)))
        if not bridge.is_valid:
            bridge = bridge.buffer(0)
    except Exception:
        return GeometryCollection()
    if bridge.is_empty:
        return GeometryCollection()
    clipped = state.assignment_geometry.intersection(bridge)
    return clipped if clipped is not None else GeometryCollection()


def _completed_fill_margin_m(state: _MissionAreaState, valid_indexes: list[int]) -> float:
    gap_samples: list[float] = []
    for pos in range(1, len(valid_indexes)):
        prev_index = int(valid_indexes[pos - 1])
        curr_index = int(valid_indexes[pos])
        if curr_index != prev_index + 1:
            continue
        try:
            gap = float(state.planned_cut_lines[prev_index].distance(state.planned_cut_lines[curr_index]))
        except Exception:
            gap = 0.0
        if gap > 0.6:
            gap_samples.append(gap * 0.58)
    if gap_samples:
        gap_samples.sort()
        return max(1.2, min(gap_samples[len(gap_samples) // 2], 12.0))
    return max(1.2, min(float(state.cut_half_width_m) * 0.95, 8.0))


def _bridge_completed_cut_strips(
    state: _MissionAreaState,
    left_strip: BaseGeometry | None,
    right_strip: BaseGeometry | None,
    *,
    fill_margin_m: float,
) -> BaseGeometry:
    left = left_strip if left_strip is not None else GeometryCollection()
    right = right_strip if right_strip is not None else GeometryCollection()
    if left.is_empty or right.is_empty:
        return GeometryCollection()
    seed = merge_coverage_geometry(left, right)
    if seed.is_empty:
        return GeometryCollection()
    try:
        bridge = seed.convex_hull
    except Exception:
        bridge = seed
    if bridge.is_empty:
        return GeometryCollection()
    # Keep the bridge tight. This closes tiny numeric seams between adjacent
    # completed sweep strips without recreating the large diagonal half-plane
    # artifacts that can appear on bent line-search missions.
    margin_m = min(max(float(fill_margin_m) * 0.08, 0.4), 2.0)
    try:
        bridge = bridge.buffer(margin_m, cap_style=2, join_style=2)
    except Exception:
        pass
    try:
        bridge = state.assignment_geometry.intersection(bridge)
    except Exception:
        return GeometryCollection()
    return bridge if bridge is not None and not bridge.is_empty else GeometryCollection()


def _coverage_sliver_threshold_m2(state: _MissionAreaState) -> float:
    return max(6.0, float(state.cut_half_width_m) * float(state.cut_half_width_m) * 0.45)


def _remaining_island_drop_threshold_m2(state: _MissionAreaState) -> float:
    width_hint_m = _as_float(state.width_hint_m) or _DEFAULT_STRIP_WIDTH_M
    return max(
        2500.0,
        _coverage_sliver_threshold_m2(state) * 12.0,
        float(state.cut_half_width_m) * float(max(width_hint_m, _DEFAULT_STRIP_WIDTH_M)) * 3.0,
    )


def _drop_completed_backtrack_islands(
    state: _MissionAreaState,
    geometry: BaseGeometry | None,
) -> BaseGeometry:
    polygons = _iter_polygons(geometry)
    if len(polygons) <= 1:
        return geometry if geometry is not None else GeometryCollection()
    boundary_index = state.progress_boundary_line_index
    if boundary_index is None:
        return geometry if geometry is not None else GeometryCollection()
    progress_vec = _planned_line_progress_vector(state, int(boundary_index))
    if progress_vec is None:
        progress_vec = state.preferred_track_vector
    if progress_vec is None:
        return geometry if geometry is not None else GeometryCollection()
    direction_sign = 1 if int(state.progress_direction_sign or 1) >= 0 else -1
    progress_dir = (
        float(progress_vec[0]) * float(direction_sign),
        float(progress_vec[1]) * float(direction_sign),
    )
    boundary_center = _planned_line_center_xy(state.planned_cut_lines[int(boundary_index)])
    if boundary_center is None:
        return geometry if geometry is not None else GeometryCollection()
    area_threshold_m2 = _remaining_island_drop_threshold_m2(state)
    behind_threshold_m = max(4.0, float(state.cut_half_width_m) * 0.8)
    kept: list[BaseGeometry] = []
    dropped_any = False
    for poly in polygons:
        try:
            area_m2 = float(poly.area or 0.0)
        except Exception:
            area_m2 = 0.0
        if area_m2 <= 0.0:
            dropped_any = True
            continue
        try:
            centroid = poly.centroid
            delta_x = float(centroid.x) - float(boundary_center[0])
            delta_y = float(centroid.y) - float(boundary_center[1])
            signed_progress_m = (
                delta_x * float(progress_dir[0]) + delta_y * float(progress_dir[1])
            )
        except Exception:
            signed_progress_m = 0.0
        if area_m2 <= float(area_threshold_m2) and signed_progress_m < -float(behind_threshold_m):
            dropped_any = True
            continue
        kept.append(poly)
    if not dropped_any:
        return geometry if geometry is not None else GeometryCollection()
    return _merge_state_geometries(kept)


def _sanitize_remaining_geometry(
    state: _MissionAreaState,
    geometry: BaseGeometry | None,
) -> BaseGeometry:
    remaining = geometry if geometry is not None else GeometryCollection()
    try:
        remaining = state.assignment_geometry.intersection(remaining)
    except Exception:
        remaining = GeometryCollection()
    if remaining.is_empty:
        return GeometryCollection()
    preserve_holes = str(state.mission_type) == "area"
    if not preserve_holes:
        remaining = _fill_geometry_holes(remaining)
    remaining = _filter_small_polygons(
        remaining,
        area_threshold_m2=_coverage_sliver_threshold_m2(state),
    )
    if preserve_holes:
        remaining = _filter_small_polygon_holes(
            remaining,
            area_threshold_m2=max(
                _coverage_sliver_threshold_m2(state),
                min(float(state.planned_area_m2) * 0.002, 40.0),
            ),
        )
    remaining = _drop_completed_backtrack_islands(state, remaining)
    return remaining if remaining is not None and not remaining.is_empty else GeometryCollection()


def _is_state_geometrically_done(
    state: _MissionAreaState,
    *,
    remaining: BaseGeometry | None = None,
) -> bool:
    if state.assignment_geometry.is_empty:
        return True
    if state.planned_cut_lines:
        last_index = len(state.planned_cut_lines) - 1
        boundary_index = (
            int(state.progress_boundary_line_index)
            if state.progress_boundary_line_index is not None
            else -1
        )
        if boundary_index < last_index:
            return False
    candidate_remaining = (
        _sanitize_remaining_geometry(state, remaining)
        if remaining is not None
        else _remaining_geometry_for_state(state)
    )
    remaining_area_m2 = float(candidate_remaining.area or 0.0) if not candidate_remaining.is_empty else 0.0
    area_threshold_m2 = max(
        _coverage_sliver_threshold_m2(state),
        min(float(state.planned_area_m2) * 0.006, 80.0),
    )
    return remaining_area_m2 <= area_threshold_m2


def _remaining_side_geometry(
    state: _MissionAreaState,
    boundary_index: int,
) -> BaseGeometry:
    if state.assignment_geometry.is_empty:
        return GeometryCollection()
    if boundary_index < 0 or boundary_index >= len(state.planned_cut_lines):
        return state.assignment_geometry
    progress_vec = _planned_line_progress_vector(state, int(boundary_index))
    if progress_vec is None:
        return state.assignment_geometry
    direction_sign = 1 if int(state.progress_direction_sign or 1) >= 0 else -1
    extended_line = _extended_planned_line(state, int(boundary_index))
    line_strings = _iter_line_strings(extended_line)
    if not line_strings:
        return state.assignment_geometry
    coords = list(line_strings[0].coords)
    if len(coords) < 2:
        return state.assignment_geometry
    bounds = state.assignment_geometry.bounds
    extent_m = max(
        120.0,
        math.hypot(bounds[2] - bounds[0], bounds[3] - bounds[1]) * 2.5,
    )
    shift_x = float(progress_vec[0]) * float(direction_sign) * float(extent_m)
    shift_y = float(progress_vec[1]) * float(direction_sign) * float(extent_m)
    ahead_shell = [
        (coords[0][0], coords[0][1]),
        (coords[-1][0], coords[-1][1]),
        (coords[-1][0] + shift_x, coords[-1][1] + shift_y),
        (coords[0][0] + shift_x, coords[0][1] + shift_y),
    ]
    try:
        ahead_polygon = Polygon(ahead_shell)
        if not ahead_polygon.is_valid:
            ahead_polygon = ahead_polygon.buffer(0)
        remaining = state.assignment_geometry.intersection(ahead_polygon)
    except Exception:
        return state.assignment_geometry
    if remaining.is_empty:
        return GeometryCollection()
    if str(state.mission_type) != "area":
        remaining = _fill_geometry_holes(remaining)
    sliver_threshold_m2 = max(6.0, float(state.cut_half_width_m) * float(state.cut_half_width_m) * 0.45)
    remaining = _filter_small_polygons(
        remaining,
        area_threshold_m2=sliver_threshold_m2,
    )
    if str(state.mission_type) == "area":
        remaining = _filter_small_polygon_holes(
            remaining,
            area_threshold_m2=sliver_threshold_m2,
        )
    return remaining


def _progress_frontier_index(
    state: _MissionAreaState,
    *,
    include_tracked: bool,
) -> int | None:
    if not state.planned_cut_lines:
        return None
    max_index = len(state.planned_cut_lines) - 1
    frontier_index = (
        state.progress_boundary_line_index
        if state.progress_boundary_line_index is not None
        else state.progress_origin_line_index
    )
    if frontier_index is None and state.completed_cut_line_indexes:
        frontier_index = max(int(idx) for idx in state.completed_cut_line_indexes)
    if frontier_index is not None:
        frontier_index = max(0, min(int(frontier_index), max_index))
    if not include_tracked or not bool(state.is_current):
        return frontier_index
    direction_sign = 1 if int(state.progress_direction_sign or 1) >= 0 else -1
    candidate_indexes = [
        state.provisional_frontier_line_index,
        state.last_nearest_cut_line_index,
        state.tracking_cut_line_index,
    ]
    for candidate in candidate_indexes:
        if candidate is None:
            continue
        candidate_index = max(0, min(int(candidate), max_index))
        if frontier_index is None:
            frontier_index = candidate_index
            continue
        if direction_sign >= 0:
            frontier_index = max(int(frontier_index), candidate_index)
        else:
            frontier_index = min(int(frontier_index), candidate_index)
    return frontier_index


def _line_frontier_window_allows(state: _MissionAreaState, index: int) -> bool:
    if not state.planned_cut_lines:
        return False
    max_index = len(state.planned_cut_lines) - 1
    index = max(0, min(int(index), max_index))
    if int(index) in state.completed_cut_line_indexes:
        return False
    frontier_index = (
        state.progress_boundary_line_index
        if state.progress_boundary_line_index is not None
        else state.progress_origin_line_index
    )
    if frontier_index is None:
        return True
    frontier_index = max(0, min(int(frontier_index), max_index))
    direction_sign = 1 if int(state.progress_direction_sign or 1) >= 0 else -1
    signed_delta = (int(index) - int(frontier_index)) * int(direction_sign)
    return 0 <= int(signed_delta) <= 2


def _line_provisional_frontier_allowed(state: _MissionAreaState) -> bool:
    if str(state.mission_type) != "line":
        return False
    if not bool(state.is_current):
        return False
    if not state.planned_cut_lines:
        return False
    max_index = len(state.planned_cut_lines) - 1
    sticky_index = (
        max(0, min(int(state.provisional_frontier_line_index), max_index))
        if state.provisional_frontier_line_index is not None
        else None
    )
    sticky_allowed = (
        sticky_index is not None
        and _line_frontier_window_allows(state, int(sticky_index))
    )
    tracking_index = state.tracking_cut_line_index
    if tracking_index is None:
        if not sticky_allowed:
            state.provisional_frontier_line_index = None
        return bool(sticky_allowed)
    tracking_index = max(0, min(int(tracking_index), max_index))
    if int(tracking_index) in state.completed_cut_line_indexes:
        if not sticky_allowed:
            state.provisional_frontier_line_index = None
        return bool(sticky_allowed)
    sample_count = int(state.tracking_sample_count or 0)
    if sample_count < 1:
        if not sticky_allowed:
            state.provisional_frontier_line_index = None
        return bool(sticky_allowed)
    projection_min = state.tracking_projection_min_m
    projection_max = state.tracking_projection_max_m
    projection_span_m = (
        abs(float(projection_max) - float(projection_min))
        if projection_min is not None and projection_max is not None
        else 0.0
    )
    path_length_m = float(state.tracking_path_length_m or 0.0)
    progressed_enough = (
        path_length_m >= max(0.8, float(state.cut_half_width_m) * 0.12)
        or projection_span_m >= max(0.8, float(state.cut_half_width_m) * 0.08)
    )
    if sample_count >= 2:
        if not progressed_enough:
            if not sticky_allowed:
                state.provisional_frontier_line_index = None
            return bool(sticky_allowed)
    elif state.last_nearest_cut_line_index is None or int(state.last_nearest_cut_line_index) != int(tracking_index):
        if not sticky_allowed:
            state.provisional_frontier_line_index = None
        return bool(sticky_allowed)
    if not _line_frontier_window_allows(state, int(tracking_index)):
        if not sticky_allowed:
            state.provisional_frontier_line_index = None
        return bool(sticky_allowed)
    direction_sign = 1 if int(state.progress_direction_sign or 1) >= 0 else -1
    if sticky_index is None:
        state.provisional_frontier_line_index = int(tracking_index)
    elif (int(tracking_index) - int(sticky_index)) * int(direction_sign) >= 0:
        state.provisional_frontier_line_index = int(tracking_index)
    return True


def _remaining_geometry_for_state(state: _MissionAreaState) -> BaseGeometry:
    if state.done:
        return GeometryCollection()
    if state.assignment_geometry.is_empty:
        return GeometryCollection()
    if not state.planned_cut_lines:
        return state.assignment_geometry
    mission_type = str(state.mission_type)
    line_provisional_allowed = _line_provisional_frontier_allowed(state)
    frontier_index = _progress_frontier_index(
        state,
        include_tracked=(mission_type == "area" or line_provisional_allowed),
    )
    if not state.completed_cut_line_indexes:
        if mission_type == "area" and frontier_index is not None:
            provisional_remaining = _sanitize_remaining_geometry(
                state,
                _remaining_side_geometry(state, int(frontier_index)),
            )
            if not provisional_remaining.is_empty:
                return provisional_remaining
        if mission_type == "line" and line_provisional_allowed and frontier_index is not None:
            provisional_remaining = _sanitize_remaining_geometry(
                state,
                _remaining_side_geometry(state, int(frontier_index)),
            )
            if not provisional_remaining.is_empty:
                return provisional_remaining
        return state.assignment_geometry
    try:
        remaining = state.assignment_geometry.difference(state.covered_geometry)
    except Exception:
        boundary_index = (
            int(state.progress_boundary_line_index)
            if state.progress_boundary_line_index is not None
            else max(int(idx) for idx in state.completed_cut_line_indexes)
        )
        remaining = _remaining_side_geometry(state, boundary_index)
    if mission_type == "area" and frontier_index is not None:
        frontier_remaining = _sanitize_remaining_geometry(
            state,
            _remaining_side_geometry(state, int(frontier_index)),
        )
        if frontier_remaining.is_empty:
            return GeometryCollection()
        try:
            remaining = remaining.intersection(frontier_remaining)
        except Exception:
            remaining = frontier_remaining
    elif mission_type == "line" and frontier_index is not None and state.completed_cut_line_indexes:
        frontier_remaining = _sanitize_remaining_geometry(
            state,
            _remaining_side_geometry(state, int(frontier_index)),
        )
        if not frontier_remaining.is_empty:
            try:
                clipped_remaining = remaining.intersection(frontier_remaining)
            except Exception:
                clipped_remaining = remaining
            clipped_remaining = _sanitize_remaining_geometry(state, clipped_remaining)
            if not clipped_remaining.is_empty or _is_state_geometrically_done(state, remaining=clipped_remaining):
                remaining = clipped_remaining
    remaining = _sanitize_remaining_geometry(state, remaining)
    if remaining.is_empty and not _is_state_geometrically_done(state, remaining=remaining):
        boundary_index = (
            int(state.progress_boundary_line_index)
            if state.progress_boundary_line_index is not None
            else max(int(idx) for idx in state.completed_cut_line_indexes)
        )
        fallback = _sanitize_remaining_geometry(
            state,
            _remaining_side_geometry(state, boundary_index),
        )
        if not fallback.is_empty:
            return fallback
    return remaining


def _planned_line_center_xy(planned_line: BaseGeometry) -> tuple[float, float] | None:
    try:
        center = planned_line.centroid
    except Exception:
        return None
    return float(center.x), float(center.y)


def _planned_line_progress_vector(
    state: _MissionAreaState,
    index: int,
) -> tuple[float, float] | None:
    if index < 0 or index >= len(state.planned_cut_lines):
        return None
    current_xy = _planned_line_center_xy(state.planned_cut_lines[int(index)])
    if current_xy is None:
        return None
    prev_xy = (
        _planned_line_center_xy(state.planned_cut_lines[index - 1])
        if index > 0
        else None
    )
    next_xy = (
        _planned_line_center_xy(state.planned_cut_lines[index + 1])
        if index + 1 < len(state.planned_cut_lines)
        else None
    )
    if prev_xy is not None and next_xy is not None:
        return _normalize_vector(next_xy[0] - prev_xy[0], next_xy[1] - prev_xy[1])
    if next_xy is not None:
        return _normalize_vector(next_xy[0] - current_xy[0], next_xy[1] - current_xy[1])
    if prev_xy is not None:
        return _normalize_vector(current_xy[0] - prev_xy[0], current_xy[1] - prev_xy[1])
    return None


def _extended_planned_line(state: _MissionAreaState, index: int) -> BaseGeometry:
    if index < 0 or index >= len(state.planned_cut_lines):
        return GeometryCollection()
    planned_line = state.planned_cut_lines[int(index)]
    line_strings = _iter_line_strings(planned_line)
    if not line_strings:
        return planned_line
    line = line_strings[0]
    coords = list(line.coords)
    if len(coords) < 2:
        return planned_line
    start_xy = coords[0]
    end_xy = coords[-1]
    line_vec = _normalize_vector(end_xy[0] - start_xy[0], end_xy[1] - start_xy[1])
    if line_vec is None:
        return planned_line
    bounds = state.assignment_geometry.bounds
    extend_m = max(
        80.0,
        math.hypot(bounds[2] - bounds[0], bounds[3] - bounds[1]) * 1.5,
    )
    return LineString(
        [
            (
                start_xy[0] - float(line_vec[0]) * extend_m,
                start_xy[1] - float(line_vec[1]) * extend_m,
            ),
            (
                end_xy[0] + float(line_vec[0]) * extend_m,
                end_xy[1] + float(line_vec[1]) * extend_m,
            ),
        ]
    )


def _completed_side_geometry(state: _MissionAreaState, boundary_index: int) -> BaseGeometry:
    _ = boundary_index
    if state.assignment_geometry.is_empty:
        return GeometryCollection()
    remaining = _remaining_geometry_for_state(state)
    try:
        covered = state.assignment_geometry.difference(remaining)
    except Exception:
        covered = GeometryCollection()
    covered = _fill_geometry_holes(covered)
    sliver_threshold_m2 = max(6.0, float(state.cut_half_width_m) * float(state.cut_half_width_m) * 0.45)
    covered = _filter_small_polygons(
        covered,
        area_threshold_m2=sliver_threshold_m2,
    )
    return covered


def _geometry_overlap_metrics(left: BaseGeometry, right: BaseGeometry) -> tuple[float, float, float]:
    if left.is_empty or right.is_empty:
        return 0.0, 0.0, 0.0
    try:
        intersection_area = float(left.intersection(right).area or 0.0)
        left_area = float(left.area or 0.0)
        right_area = float(right.area or 0.0)
        union_area = max(left_area + right_area - intersection_area, 0.0)
    except Exception:
        return 0.0, 0.0, 0.0
    if intersection_area <= 0.0 or left_area <= 0.0 or right_area <= 0.0:
        return 0.0, 0.0, 0.0
    overlap_small = intersection_area / max(min(left_area, right_area), 1e-6)
    iou = intersection_area / max(union_area, 1e-6)
    return intersection_area, overlap_small, iou


def _find_previous_matching_state(
    previous_states: dict[int, _MissionAreaState],
    *,
    used_mission_ids: set[int],
    aircraft_id: int,
    input_id: int | None,
    mission_type: str,
    assignment_geometry: BaseGeometry,
    planned_cut_lines: list[BaseGeometry],
) -> _MissionAreaState | None:
    if assignment_geometry.is_empty:
        return None
    is_line_mission = str(mission_type) == "line"
    best_state: _MissionAreaState | None = None
    best_score: tuple[float, float, float] | None = None
    try:
        target_center = assignment_geometry.centroid
        target_diag = math.hypot(
            float(assignment_geometry.bounds[2] - assignment_geometry.bounds[0]),
            float(assignment_geometry.bounds[3] - assignment_geometry.bounds[1]),
        )
    except Exception:
        target_center = None
        target_diag = 0.0
    target_cut_count = len(planned_cut_lines)
    for previous_mid, previous_state in previous_states.items():
        if int(previous_mid) in used_mission_ids:
            continue
        if int(previous_state.aircraft_id) != int(aircraft_id):
            continue
        if str(previous_state.mission_type) != str(mission_type):
            continue
        if input_id is not None and previous_state.input_id is not None and int(previous_state.input_id) != int(input_id):
            continue
        previous_cut_count = len(previous_state.planned_cut_lines)
        if target_cut_count > 0 and previous_cut_count > 0 and not is_line_mission:
            allowed_gap = max(1, int(max(target_cut_count, previous_cut_count) * 0.04))
            if abs(int(target_cut_count) - int(previous_cut_count)) > allowed_gap:
                continue
        intersection_area, overlap_small, iou = _geometry_overlap_metrics(
            assignment_geometry,
            previous_state.assignment_geometry,
        )
        if is_line_mission:
            if overlap_small < 0.82 and not (overlap_small >= 0.60 and iou >= 0.35):
                continue
        else:
            if overlap_small < 0.985 or iou < 0.95:
                continue
        if target_center is not None:
            try:
                previous_center = previous_state.assignment_geometry.centroid
                center_distance = float(target_center.distance(previous_center))
            except Exception:
                center_distance = 0.0
            center_limit_m = max(10.0, float(target_diag) * 0.03)
            if is_line_mission:
                center_limit_m = max(60.0, float(target_diag) * 0.50)
            if center_distance > center_limit_m:
                continue
        score = (float(overlap_small), float(iou), float(intersection_area))
        if best_score is None or score > best_score:
            best_score = score
            best_state = previous_state
    return best_state


def _rebuild_completed_sweep_coverage(state: _MissionAreaState) -> None:
    if state.done:
        state.covered_geometry = state.assignment_geometry
        state.cut_lines = list(state.planned_cut_lines[-800:])
        if state.planned_cut_lines:
            state.completed_cut_line_indexes = set(range(len(state.planned_cut_lines)))
            state.last_cut_line_index = len(state.planned_cut_lines) - 1
        return
    if (
        state.progress_origin_line_index is not None
        and state.progress_boundary_line_index is not None
        and state.planned_cut_lines
    ):
        lo = max(0, min(int(state.progress_origin_line_index), int(state.progress_boundary_line_index)))
        hi = min(len(state.planned_cut_lines) - 1, max(int(state.progress_origin_line_index), int(state.progress_boundary_line_index)))
        state.completed_cut_line_indexes = set(range(lo, hi + 1))
    rendered_cut_lines: list[BaseGeometry] = []
    valid_indexes = sorted(
        idx
        for idx in state.completed_cut_line_indexes
        if 0 <= int(idx) < len(state.planned_cut_lines)
    )
    if not valid_indexes:
        state.covered_geometry = GeometryCollection()
        state.cut_lines = []
        state.last_cut_line_index = -1
        return
    if len(valid_indexes) >= len(state.planned_cut_lines):
        state.covered_geometry = state.assignment_geometry
        state.cut_lines = list(state.planned_cut_lines[-800:])
        state.last_cut_line_index = len(state.planned_cut_lines) - 1
        return
    strip_cache: dict[int, BaseGeometry] = {}
    recomputed_covered: BaseGeometry = GeometryCollection()
    for idx in valid_indexes:
        planned_line = state.planned_cut_lines[int(idx)]
        if not planned_line.is_empty:
            rendered_cut_lines.append(planned_line)
        strip = _planned_cut_strip(state, planned_line)
        if strip.is_empty:
            continue
        strip_cache[int(idx)] = strip
        recomputed_covered = merge_coverage_geometry(recomputed_covered, strip)
    fill_margin_m = _completed_fill_margin_m(state, valid_indexes)
    for pos in range(1, len(valid_indexes)):
        prev_index = int(valid_indexes[pos - 1])
        curr_index = int(valid_indexes[pos])
        if curr_index != prev_index + 1:
            continue
        bridge = _bridge_completed_cut_strips(
            state,
            strip_cache.get(prev_index),
            strip_cache.get(curr_index),
            fill_margin_m=float(fill_margin_m),
        )
        if bridge.is_empty:
            continue
        recomputed_covered = merge_coverage_geometry(recomputed_covered, bridge)
    try:
        remaining = _sanitize_remaining_geometry(
            state,
            state.assignment_geometry.difference(recomputed_covered),
        )
        recomputed_covered = state.assignment_geometry.difference(remaining)
    except Exception:
        remaining = GeometryCollection()
    recomputed_covered = _fill_geometry_holes(recomputed_covered)
    recomputed_covered = _filter_small_polygons(
        recomputed_covered,
        area_threshold_m2=_coverage_sliver_threshold_m2(state),
    )
    if (
        valid_indexes
        and valid_indexes[-1] >= len(state.planned_cut_lines) - 1
        and _is_state_geometrically_done(state, remaining=remaining)
    ):
        recomputed_covered = state.assignment_geometry
    state.covered_geometry = recomputed_covered
    state.cut_lines = rendered_cut_lines[-800:]
    state.last_cut_line_index = valid_indexes[-1] if valid_indexes else -1


class _MissionAreaCanvas(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._payload: dict[str, Any] | None = None
        self._zoom = 1.0
        self._pan = QPointF(0.0, 0.0)
        self._is_panning = False
        self._last_pan_pos: QPoint | None = None
        self.setMinimumSize(560, 420)
        self.setMouseTracking(True)

    def set_payload(self, payload: dict[str, Any] | None) -> None:
        self._payload = payload if isinstance(payload, dict) else None
        self.update()

    def zoom_in(self) -> None:
        self._apply_zoom(1.2)

    def zoom_out(self) -> None:
        self._apply_zoom(1.0 / 1.2)

    def reset_view(self) -> None:
        self._zoom = 1.0
        self._pan = QPointF(0.0, 0.0)
        self.update()

    def _apply_zoom(self, factor: float) -> None:
        self._zoom = max(0.35, min(8.0, float(self._zoom) * float(factor)))
        self.update()

    def wheelEvent(self, event) -> None:  # noqa: N802
        delta = event.angleDelta().y()
        if delta == 0:
            event.ignore()
            return
        self._apply_zoom(1.15 if delta > 0 else (1.0 / 1.15))
        event.accept()

    def mousePressEvent(self, event) -> None:  # noqa: N802
        if event.button() == Qt.LeftButton:
            self._is_panning = True
            self._last_pan_pos = event.pos()
            self.setCursor(Qt.ClosedHandCursor)
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event) -> None:  # noqa: N802
        if self._is_panning and self._last_pan_pos is not None:
            delta = event.pos() - self._last_pan_pos
            self._pan += QPointF(float(delta.x()), float(delta.y()))
            self._last_pan_pos = event.pos()
            self.update()
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event) -> None:  # noqa: N802
        if event.button() == Qt.LeftButton and self._is_panning:
            self._is_panning = False
            self._last_pan_pos = None
            self.unsetCursor()
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def paintEvent(self, event) -> None:  # noqa: N802
        _ = event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.fillRect(self.rect(), QColor("#f8fafc"))
        payload = self._payload or {}
        missions = [
            item for item in (payload.get("missions") or [])
            if isinstance(item, dict)
            and isinstance(item.get("assignment_geometry"), BaseGeometry)
            and not item["assignment_geometry"].is_empty
        ]
        if not missions:
            painter.setPen(QColor("#64748b"))
            painter.drawText(self.rect(), Qt.AlignCenter, "표시할 임무 도형이 없습니다.")
            painter.end()
            return
        min_x = min(item["assignment_geometry"].bounds[0] for item in missions)
        min_y = min(item["assignment_geometry"].bounds[1] for item in missions)
        max_x = max(item["assignment_geometry"].bounds[2] for item in missions)
        max_y = max(item["assignment_geometry"].bounds[3] for item in missions)
        width = max(1.0, max_x - min_x)
        height = max(1.0, max_y - min_y)
        margin = 28.0
        avail_w = max(1.0, self.width() - margin * 2.0)
        avail_h = max(1.0, self.height() - margin * 2.0)
        fit_scale = max(0.001, min(avail_w / width, avail_h / height))
        scale = fit_scale * float(self._zoom)
        content_w = width * scale
        content_h = height * scale
        off_x = margin + (avail_w - content_w) * 0.5 + float(self._pan.x())
        off_y = margin + (avail_h - content_h) * 0.5 - float(self._pan.y())

        def _pt(x_val: float, y_val: float) -> QPointF:
            return QPointF(
                off_x + (x_val - min_x) * scale,
                self.height() - (off_y + (y_val - min_y) * scale),
            )

        def _path(geometry: BaseGeometry | None) -> QPainterPath:
            path = QPainterPath()
            for poly in _iter_polygons(geometry):
                ext = list(poly.exterior.coords)
                if len(ext) < 3:
                    continue
                path.moveTo(_pt(ext[0][0], ext[0][1]))
                for x_val, y_val in ext[1:]:
                    path.lineTo(_pt(x_val, y_val))
                path.closeSubpath()
                for ring in poly.interiors:
                    coords = list(ring.coords)
                    if len(coords) < 3:
                        continue
                    path.moveTo(_pt(coords[0][0], coords[0][1]))
                    for x_val, y_val in coords[1:]:
                        path.lineTo(_pt(x_val, y_val))
                    path.closeSubpath()
            return path

        painter.setPen(QPen(QColor("#e2e8f0"), 1))
        painter.drawRoundedRect(int(off_x - 10), int(off_y - 10), int(content_w + 20), int(content_h + 20), 14, 14)
        base_color = QColor(_UAV_COLORS.get(int(payload.get("aircraft_id") or 0), "#2563eb"))
        selected_mission_id = _as_int(payload.get("selected_mission_id"))

        def _overlay_pen(group: dict[str, Any]) -> QPen:
            role = str(group.get("role") or "")
            geom_type = str(group.get("geometry_type") or "")
            is_hole = bool(group.get("is_hole"))
            if role == "remaining":
                color = QColor("#dc2626" if geom_type == "line" else "#0f766e")
            else:
                color = QColor("#475569" if geom_type == "line" else "#1d4ed8")
            if is_hole:
                color = QColor("#b45309")
            style = Qt.SolidLine if role == "remaining" else Qt.DotLine
            return QPen(color, 2, style)

        def _draw_coordinate_overlay(group: dict[str, Any]) -> None:
            points = list(group.get("points") or [])
            if not points:
                return
            pen = _overlay_pen(group)
            painter.setPen(pen)
            painter.setBrush(QBrush(pen.color()))
            for idx in range(1, len(points)):
                painter.drawLine(
                    _pt(points[idx - 1]["x"], points[idx - 1]["y"]),
                    _pt(points[idx]["x"], points[idx]["y"]),
                )
            if str(group.get("geometry_type") or "") == "area" and len(points) >= 3:
                painter.drawLine(
                    _pt(points[-1]["x"], points[-1]["y"]),
                    _pt(points[0]["x"], points[0]["y"]),
                )
            role_label = "SRC" if str(group.get("role") or "") == "source" else "REM"
            geom_label = "L" if str(group.get("geometry_type") or "") == "line" else "A"
            if bool(group.get("is_hole")):
                geom_label = f"{geom_label}H"
            for idx, point_info in enumerate(points, start=1):
                point = _pt(point_info["x"], point_info["y"])
                radius = 4.5 if idx == 1 else 3.5
                painter.drawEllipse(point, radius, radius)
                label_offset = QPointF(10.0, -10.0 if idx % 2 else 16.0)
                altitude = _as_float(point_info.get("altitude"))
                label = (
                    f"{role_label}-{geom_label}{int(group.get('block_index') or 0)}"
                    f"-P{int(point_info.get('index') or idx)}"
                    f"\n{float(point_info.get('latitude') or 0.0):.6f}, {float(point_info.get('longitude') or 0.0):.6f}"
                )
                if altitude is not None:
                    label = f"{label}\nALT {int(round(float(altitude)))}"
                painter.drawText(point + label_offset, label)

        for item in missions:
            painter.fillPath(_path(item.get("assignment_geometry")), QBrush(QColor(base_color.red(), base_color.green(), base_color.blue(), 22)))
            painter.setPen(QPen(QColor(base_color.red(), base_color.green(), base_color.blue(), 110), 1, Qt.DashLine))
            painter.drawPath(_path(item.get("assignment_geometry")))
            planned_cut_lines = item.get("planned_cut_lines") or []
            if planned_cut_lines:
                painter.setPen(QPen(QColor(37, 99, 235, 90), 1, Qt.DashLine))
                for planned_geom in planned_cut_lines[-600:]:
                    for line in _iter_line_strings(planned_geom):
                        coords = list(line.coords)
                        for idx in range(1, len(coords)):
                            painter.drawLine(
                                _pt(coords[idx - 1][0], coords[idx - 1][1]),
                                _pt(coords[idx][0], coords[idx][1]),
                            )
            remaining = item.get("remaining_geometry")
            if isinstance(remaining, BaseGeometry) and not remaining.is_empty:
                painter.fillPath(_path(remaining), QBrush(QColor(base_color.red(), base_color.green(), base_color.blue(), 48)))
                painter.setPen(QPen(base_color, 3 if item.get("is_current") else 2))
                painter.drawPath(_path(remaining))
            covered = item.get("covered_geometry")
            if isinstance(covered, BaseGeometry) and not covered.is_empty:
                fill = QColor("#16a34a" if item.get("is_done") else "#22c55e")
                fill.setAlpha(128 if item.get("is_current") else 110)
                painter.fillPath(_path(covered), QBrush(fill))
            cut_lines = item.get("cut_lines") or []
            if cut_lines:
                painter.setPen(QPen(QColor("#1d4ed8"), 2))
                for cut_geom in cut_lines[-400:]:
                    for line in _iter_line_strings(cut_geom):
                        coords = list(line.coords)
                        for idx in range(1, len(coords)):
                            painter.drawLine(
                                _pt(coords[idx - 1][0], coords[idx - 1][1]),
                                _pt(coords[idx][0], coords[idx][1]),
                            )
            else:
                points = item.get("centerline_points") or []
                if len(points) >= 2:
                    painter.setPen(QPen(QColor("#1d4ed8"), 2, Qt.DashLine))
                    for idx in range(1, len(points)):
                        painter.drawLine(
                            _pt(points[idx - 1][0], points[idx - 1][1]),
                            _pt(points[idx][0], points[idx][1]),
                        )
            preview_source_lines = item.get("preview_source_lines") or []
            if preview_source_lines:
                painter.setPen(QPen(QColor("#64748b"), 1, Qt.DotLine))
                painter.setBrush(Qt.NoBrush)
                for preview_geom in preview_source_lines:
                    for line in _iter_line_strings(preview_geom):
                        coords = list(line.coords)
                        for idx in range(1, len(coords)):
                            painter.drawLine(
                                _pt(coords[idx - 1][0], coords[idx - 1][1]),
                                _pt(coords[idx][0], coords[idx][1]),
                            )
                        for idx, (x_val, y_val) in enumerate(coords):
                            point = _pt(x_val, y_val)
                            radius = 4.0 if idx == 0 else 3.0
                            painter.setPen(QPen(QColor("#475569"), 1))
                            painter.setBrush(QBrush(QColor(248, 250, 252, 220)))
                            painter.drawEllipse(point, radius, radius)
            preview_remaining_lines = item.get("preview_remaining_lines") or []
            if preview_remaining_lines:
                painter.setPen(QPen(QColor("#dc2626"), 2))
                painter.setBrush(QBrush(QColor("#dc2626")))
                for preview_geom in preview_remaining_lines:
                    for line in _iter_line_strings(preview_geom):
                        coords = list(line.coords)
                        for idx in range(1, len(coords)):
                            painter.drawLine(
                                _pt(coords[idx - 1][0], coords[idx - 1][1]),
                                _pt(coords[idx][0], coords[idx][1]),
                            )
                        for idx, (x_val, y_val) in enumerate(coords):
                            point = _pt(x_val, y_val)
                            radius = 4.5 if idx == 0 else 3.5
                            painter.drawEllipse(point, radius, radius)
            centroid = item["assignment_geometry"].centroid
            label = f"M{int(item.get('mission_id') or 0)} | {int(item.get('coverage_percent') or 0)}%"
            if item.get("is_done"):
                label = f"{label} | DONE"
            painter.setPen(QColor("#0f172a"))
            painter.drawText(_pt(centroid.x, centroid.y) + QPointF(8.0, -8.0), label)
            if selected_mission_id is not None and int(item.get("mission_id") or 0) == int(selected_mission_id):
                for overlay_group in item.get("coordinate_overlay_groups") or []:
                    if isinstance(overlay_group, dict):
                        _draw_coordinate_overlay(overlay_group)
        painter.end()


class MissionProgressAreaManagementTab(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._ui_updates_enabled = True
        self._dirty = False
        self._selected_aircraft_id = 4
        self._mission_view: dict[str, Any] | None = None
        self._progress_tracker = MissionProgressTracker()
        self._progress_snapshot: dict[str, Any] = {}
        self._refresh_interval_ms = 90
        self._last_view_refresh_monotonic = 0.0
        self._refresh_timer = QTimer(self)
        self._refresh_timer.setSingleShot(True)
        self._refresh_timer.timeout.connect(self._flush_deferred_refresh)
        self._state_cache_token = 0
        self._rows_cache: dict[int, dict[str, Any]] | None = None
        self._rows_cache_key: tuple[int, int] | None = None
        self._snapshot_cache: dict[str, Any] | None = None
        self._snapshot_cache_key: int | None = None
        self._remaining_geometry_cache: dict[int, BaseGeometry] = {}
        self._state_remaining_detail_cache: dict[int, tuple[dict[str, Any], float]] = {}
        self._group_remaining_detail_cache: dict[tuple[int, ...], tuple[dict[str, Any], float]] = {}
        self._preview_line_cache: dict[int, tuple[list[BaseGeometry], list[BaseGeometry]]] = {}
        self._states: dict[int, _MissionAreaState] = {}
        self._current_mission_by_aircraft: dict[int, int | None] = {}
        self._last_timestamp_ms: int | None = None
        self._last_snapshot_signature: str | None = None
        self._selected_mission_id: int | None = None
        self._summary_cards: dict[int, tuple[QFrame, QLabel, QLabel]] = {}
        self._aircraft_buttons: dict[int, QPushButton] = {}
        self._plan_summary_label: QLabel | None = None
        self._canvas_summary_label: QLabel | None = None
        self._mission_table: QTableWidget | None = None
        self._coordinate_summary_label: QLabel | None = None
        self._coordinate_table: QTableWidget | None = None
        self._canvas: _MissionAreaCanvas | None = None
        self._build_ui()

    def set_ui_updates_enabled(self, enabled: bool) -> None:
        self._ui_updates_enabled = bool(enabled)
        if not self._ui_updates_enabled and self._refresh_timer.isActive():
            self._refresh_timer.stop()
        if self._ui_updates_enabled and self._dirty:
            self._request_refresh(force=True)

    def _invalidate_runtime_cache(self, *, selection_only: bool = False) -> None:
        self._rows_cache = None
        self._rows_cache_key = None
        self._preview_line_cache.clear()
        if selection_only:
            return
        self._state_cache_token += 1
        self._snapshot_cache = None
        self._snapshot_cache_key = None
        self._remaining_geometry_cache.clear()
        self._state_remaining_detail_cache.clear()
        self._group_remaining_detail_cache.clear()

    def _get_cached_remaining_geometry(self, state: _MissionAreaState) -> BaseGeometry:
        key = int(state.mission_id)
        cached = self._remaining_geometry_cache.get(key)
        if cached is not None:
            return cached
        remaining = _remaining_geometry_for_state(state)
        self._remaining_geometry_cache[key] = remaining
        return remaining

    def _get_cached_state_remaining_detail(
        self,
        state: _MissionAreaState,
    ) -> tuple[dict[str, Any], float]:
        key = int(state.mission_id)
        cached = self._state_remaining_detail_cache.get(key)
        if cached is not None:
            return cached
        detail = self._build_state_remaining_detail(state)
        self._state_remaining_detail_cache[key] = detail
        return detail

    def _get_cached_group_remaining_detail(
        self,
        states: list[_MissionAreaState],
    ) -> tuple[dict[str, Any], float]:
        key = tuple(sorted(int(state.mission_id) for state in states))
        cached = self._group_remaining_detail_cache.get(key)
        if cached is not None:
            return cached
        detail = self._build_group_remaining_detail(states)
        self._group_remaining_detail_cache[key] = detail
        return detail

    def _get_cached_preview_lines(
        self,
        state: _MissionAreaState,
    ) -> tuple[list[BaseGeometry], list[BaseGeometry]]:
        key = int(state.mission_id)
        cached = self._preview_line_cache.get(key)
        if cached is not None:
            return cached
        preview_detail, _ = self._get_cached_state_remaining_detail(state)
        preview_line_list = list((preview_detail or {}).get("lineList") or [])
        preview_coordinate_list = list((preview_detail or {}).get("coordinateList") or [])
        preview_source_line_list = list(state.input_line_list or state.source_line_list or [])
        preview_source_coordinate_list = list(state.input_coordinate_list or state.source_coordinate_list or [])
        preview_source_lines = [
            geom
            for geom, _width_m in _build_source_line_geometries(
                preview_source_line_list,
                preview_source_coordinate_list,
                state.coverage_def.transformer,
                default_width_m=float(state.width_hint_m or 1.0),
            )
        ]
        preview_remaining_lines = [
            geom
            for geom, _width_m in _build_source_line_geometries(
                preview_line_list,
                preview_coordinate_list,
                state.coverage_def.transformer,
                default_width_m=float(state.width_hint_m or 1.0),
            )
        ]
        if not _line_preview_has_visible_delta(
            preview_source_lines,
            preview_remaining_lines,
        ):
            preview_source_lines = []
            preview_remaining_lines = []
        cached = (preview_source_lines, preview_remaining_lines)
        self._preview_line_cache[key] = cached
        return cached

    def _flush_deferred_refresh(self) -> None:
        if not self._ui_updates_enabled or not self._dirty:
            return
        self._last_view_refresh_monotonic = time.perf_counter()
        self._request_refresh()

    def _request_refresh(self, *, force: bool = False) -> None:
        if not self._ui_updates_enabled:
            self._dirty = True
            self._persist_replan_snapshot()
            return
        now = time.perf_counter()
        elapsed_ms = (
            (float(now) - float(self._last_view_refresh_monotonic)) * 1000.0
            if self._last_view_refresh_monotonic > 0.0
            else float("inf")
        )
        if force or elapsed_ms >= float(self._refresh_interval_ms):
            if self._refresh_timer.isActive():
                self._refresh_timer.stop()
            self._last_view_refresh_monotonic = float(now)
            self._refresh_view()
            return
        self._dirty = True
        remaining_ms = max(1, int(round(float(self._refresh_interval_ms) - float(elapsed_ms))))
        if not self._refresh_timer.isActive():
            self._refresh_timer.start(remaining_ms)
            return
        current_remaining_ms = int(self._refresh_timer.remainingTime())
        if current_remaining_ms <= 0 or current_remaining_ms > remaining_ms:
            self._refresh_timer.start(remaining_ms)

    def update_0903(self, *, timestamp_ms: int | None, mission_plan_id: int | None, source: str | None = None) -> None:
        _ = timestamp_ms, source
        self._load_mission_plan(mission_plan_id)

    def apply_mission_plan_decision(self, *, mission_plan_id: int | None) -> None:
        self._load_mission_plan(mission_plan_id)

    def update_agent_status(self, *, timestamp_ms: int | None, agent_states: list[dict[str, Any]], fuel_state_map: dict[int, str] | None = None) -> None:
        _ = fuel_state_map
        if not self._mission_view:
            return
        self._progress_snapshot = self._progress_tracker.update(timestamp_ms, agent_states or [])
        if timestamp_ms is not None:
            self._last_timestamp_ms = int(timestamp_ms)
        current_map = self._progress_snapshot.get("aircraft_current_mission") or {}
        for aid, mid in current_map.items():
            aid_i = _as_int(aid)
            if aid_i is not None:
                self._current_mission_by_aircraft[aid_i] = _as_int(mid)
        mission_progress = self._progress_snapshot.get("mission_progress") or {}
        states_requiring_rebuild: set[int] = set()
        for mid, state in self._states.items():
            progress_done = bool((mission_progress.get(mid) or {}).get("done"))
            geometry_done = _is_state_geometrically_done(state)
            if progress_done or geometry_done or not state.done or not state.planned_cut_lines:
                continue
            completed_count = len(
                [
                    idx
                    for idx in state.completed_cut_line_indexes
                    if 0 <= int(idx) < len(state.planned_cut_lines)
                ]
            )
            if completed_count >= len(state.planned_cut_lines):
                continue
            state.done = False
            states_requiring_rebuild.add(int(mid))
        for item in agent_states or []:
            aid = _as_int(item.get("aircraft_id"))
            if aid is None:
                continue
            mid = _as_int(self._current_mission_by_aircraft.get(aid))
            state = self._states.get(mid or -1)
            if state is None or state.done:
                continue
            center_xy = _project_xy(state.coverage_def.transformer, _coord(item.get("sensor_center_coordinate") or item.get("coordinate")))
            if center_xy is None:
                continue
            footprint_width_m = _footprint_width_m(item.get("footprint_corners")) or state.width_hint_m or _DEFAULT_STRIP_WIDTH_M
            footprint_geometry = build_footprint_geometry(
                item.get("footprint_corners"),
                state.coverage_def.transformer,
            )
            if state.planned_cut_lines:
                if not _footprint_observes_assignment(
                    state,
                    footprint_geometry,
                    width_m=footprint_width_m,
                ):
                    _reset_planned_line_tracking(state)
                    state.last_center_xy = center_xy
                    state.last_update_ms = int(timestamp_ms) if timestamp_ms is not None else state.last_update_ms
                    if not state.centerline_points or Point(state.centerline_points[-1]).distance(Point(center_xy)) > 0.25:
                        state.centerline_points.append(center_xy)
                    continue
                nearest_index = None
                nearest_distance = None
                center_point = Point(center_xy)
                for idx, planned_line in enumerate(state.planned_cut_lines):
                    if not _footprint_supports_planned_line(
                        state,
                        line_index=int(idx),
                        footprint_geometry=footprint_geometry,
                        width_m=footprint_width_m,
                    ):
                        continue
                    try:
                        distance = float(planned_line.distance(center_point))
                    except Exception:
                        continue
                    if nearest_distance is None or distance < nearest_distance:
                        nearest_distance = distance
                        nearest_index = idx
                if nearest_index is not None and nearest_distance is not None:
                    distance_limit_m = max(22.0, float(state.cut_half_width_m) * 4.5)
                    if float(nearest_distance) <= distance_limit_m:
                        nearest_index_i = int(nearest_index)
                        effective_index_i = _select_effective_cut_line_index(
                            state,
                            nearest_index=int(nearest_index_i),
                            center_point=center_point,
                            distance_limit_m=float(distance_limit_m),
                        )
                        if effective_index_i is None:
                            _reset_planned_line_tracking(state)
                            continue
                        committed = _update_planned_line_tracking(
                            state,
                            line_index=int(effective_index_i),
                            center_xy=center_xy,
                        )
                        if committed:
                            if _commit_observed_planned_line(
                                state,
                                tracked_index=int(effective_index_i),
                                center_point=center_point,
                                distance_limit_m=float(distance_limit_m),
                            ):
                                states_requiring_rebuild.add(int(state.mission_id))
                            _reset_planned_line_tracking(state)
                        elif _single_sample_line_commit_allowed(
                            state,
                            line_index=int(effective_index_i),
                            center_point=center_point,
                            footprint_geometry=footprint_geometry,
                            width_m=footprint_width_m,
                            nearest_distance_m=None,
                        ):
                            if _commit_observed_planned_line(
                                state,
                                tracked_index=int(effective_index_i),
                                center_point=center_point,
                                distance_limit_m=float(distance_limit_m),
                            ):
                                states_requiring_rebuild.add(int(state.mission_id))
                            _reset_planned_line_tracking(state)
                        state.last_nearest_cut_line_index = effective_index_i
                    else:
                        _reset_planned_line_tracking(state)
                        state.last_nearest_cut_line_index = None
                else:
                    if nearest_index is None:
                        _reset_planned_line_tracking(state)
                        state.last_nearest_cut_line_index = None
            else:
                prev_xy = state.last_center_xy
                step_distance_m = None
                track_vector = None
                if prev_xy is not None:
                    dx = center_xy[0] - prev_xy[0]
                    dy = center_xy[1] - prev_xy[1]
                    step_distance_m = math.hypot(dx, dy)
                    if step_distance_m > 0.25:
                        track_vector = _normalize_vector(dx, dy)
                if track_vector is None:
                    track_vector = state.preferred_track_vector
                if track_vector is not None:
                    cross_vector = (-float(track_vector[1]), float(track_vector[0]))
                    bounds = state.assignment_geometry.bounds
                    diag = math.hypot(bounds[2] - bounds[0], bounds[3] - bounds[1])
                    half_length = max(float(footprint_width_m) * 1.5, float(diag) * 0.8, 20.0)
                    start_xy = (
                        center_xy[0] - cross_vector[0] * half_length,
                        center_xy[1] - cross_vector[1] * half_length,
                    )
                    end_xy = (
                        center_xy[0] + cross_vector[0] * half_length,
                        center_xy[1] + cross_vector[1] * half_length,
                    )
                    cut_line = LineString([start_xy, end_xy])
                    slice_depth_m = max(
                        2.0,
                        float(step_distance_m or 0.0),
                        float(footprint_width_m) * 0.14,
                    )
                    cut_strip = cut_line.buffer(
                        slice_depth_m * 0.5,
                        cap_style=2,
                        join_style=2,
                    )
                    clipped_strip = state.assignment_geometry.intersection(cut_strip)
                    state.covered_geometry = merge_coverage_geometry(
                        state.covered_geometry,
                        clipped_strip,
                    )
                    clipped_line = state.assignment_geometry.intersection(cut_line)
                    if clipped_line is not None and not clipped_line.is_empty:
                        state.cut_lines.append(clipped_line)
                        if len(state.cut_lines) > 800:
                            state.cut_lines = state.cut_lines[-800:]
            state.last_center_xy = center_xy
            state.last_update_ms = int(timestamp_ms) if timestamp_ms is not None else state.last_update_ms
            if not state.centerline_points or Point(state.centerline_points[-1]).distance(Point(center_xy)) > 0.25:
                state.centerline_points.append(center_xy)
        for rebuild_mid in sorted(states_requiring_rebuild):
            rebuild_state = self._states.get(int(rebuild_mid))
            if rebuild_state is not None:
                _rebuild_completed_sweep_coverage(rebuild_state)
        for mid, state in self._states.items():
            progress_done = bool((mission_progress.get(mid) or {}).get("done"))
            geometry_done = _is_state_geometrically_done(state)
            if geometry_done:
                state.done = True
                state.covered_geometry = state.assignment_geometry
                continue
            if not progress_done:
                continue
            if not state.planned_cut_lines:
                state.done = True
                state.covered_geometry = state.assignment_geometry
        self._invalidate_runtime_cache()
        self._persist_replan_snapshot()
        self._request_refresh()

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)
        title = QLabel("임무 진행영역 관리")
        title.setStyleSheet("font-size: 18px; font-weight: 800; color: #0f172a;")
        subtitle = QLabel("0401 sensor center 궤적과 footprint 폭으로 실제 촬영된 영역을 잘라내듯 소거해서 관리합니다.")
        subtitle.setWordWrap(True)
        subtitle.setStyleSheet("color: #475569;")
        root.addWidget(title)
        root.addWidget(subtitle)
        splitter = QSplitter(Qt.Horizontal)
        splitter.setChildrenCollapsible(False)
        left = QWidget()
        left_layout = QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(10)
        self._plan_summary_label = QLabel("MissionPlan: - / Last Update: -")
        self._plan_summary_label.setStyleSheet("padding: 10px 12px; border: 1px solid #cbd5e1; border-radius: 10px; background: #ffffff;")
        left_layout.addWidget(self._plan_summary_label)
        button_row = QHBoxLayout()
        button_row.setSpacing(8)
        for aid in _UAV_IDS:
            btn = QPushButton(f"UAV {aid}")
            btn.setCheckable(True)
            btn.clicked.connect(lambda _checked, aircraft_id=int(aid): self._select_aircraft(aircraft_id))
            self._aircraft_buttons[int(aid)] = btn
            button_row.addWidget(btn)
        button_row.addStretch(1)
        left_layout.addLayout(button_row)
        card_group = QGroupBox("UAV별 요약")
        card_layout = QVBoxLayout(card_group)
        card_layout.setContentsMargins(10, 10, 10, 10)
        card_layout.setSpacing(8)
        for aid in _UAV_IDS:
            frame = QFrame()
            frame.setObjectName("missionAreaSummaryCard")
            frame.setStyleSheet("QFrame#missionAreaSummaryCard { border: 1px solid #cbd5e1; border-radius: 12px; background: #ffffff; }")
            layout = QVBoxLayout(frame)
            layout.setContentsMargins(14, 12, 14, 12)
            layout.setSpacing(4)
            title_label = QLabel(f"UAV {aid}")
            title_label.setStyleSheet("color: #0f172a; font: 700 11px 'Malgun Gothic'; border: none;")
            value_label = QLabel("소거율 -")
            value_label.setStyleSheet("color: #0f172a; font: 700 15px 'Malgun Gothic'; border: none;")
            detail_label = QLabel("할당 - / 잔여 -")
            detail_label.setWordWrap(True)
            detail_label.setStyleSheet("color: #475569; font: 10px 'Malgun Gothic'; border: none;")
            layout.addWidget(title_label)
            layout.addWidget(value_label)
            layout.addWidget(detail_label)
            self._summary_cards[int(aid)] = (frame, value_label, detail_label)
            card_layout.addWidget(frame)
        left_layout.addWidget(card_group)
        table_group = QGroupBox("선택 UAV 임무 소거 현황")
        table_layout = QVBoxLayout(table_group)
        table_layout.setContentsMargins(10, 10, 10, 10)
        table_layout.setSpacing(8)
        self._mission_table = QTableWidget(0, 7)
        self._mission_table.setHorizontalHeaderLabels(["Mission", "Type", "상태", "소거율", "잔여 영역", "할당 영역", "Last Update"])
        self._mission_table.verticalHeader().setVisible(False)
        self._mission_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self._mission_table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self._mission_table.setSelectionMode(QAbstractItemView.SingleSelection)
        self._mission_table.setAlternatingRowColors(True)
        self._mission_table.setWordWrap(False)
        self._mission_table.horizontalHeader().setStretchLastSection(True)
        self._mission_table.itemSelectionChanged.connect(self._on_mission_table_selection_changed)
        table_layout.addWidget(self._mission_table)
        left_layout.addWidget(table_group, 1)
        coordinate_group = QGroupBox("선택 임무 좌표 상세")
        coordinate_layout = QVBoxLayout(coordinate_group)
        coordinate_layout.setContentsMargins(10, 10, 10, 10)
        coordinate_layout.setSpacing(8)
        self._coordinate_summary_label = QLabel("Mission row를 선택하면 원본/재구성 점 좌표가 표시됩니다.")
        self._coordinate_summary_label.setWordWrap(True)
        self._coordinate_summary_label.setStyleSheet("color: #475569; font: 11px 'Malgun Gothic';")
        coordinate_layout.addWidget(self._coordinate_summary_label)
        self._coordinate_table = QTableWidget(0, 7)
        self._coordinate_table.setHorizontalHeaderLabels(["Role", "Geom", "Block/Point", "Latitude", "Longitude", "Altitude", "Note"])
        self._coordinate_table.verticalHeader().setVisible(False)
        self._coordinate_table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self._coordinate_table.setSelectionMode(QAbstractItemView.NoSelection)
        self._coordinate_table.setAlternatingRowColors(True)
        self._coordinate_table.setWordWrap(False)
        self._coordinate_table.horizontalHeader().setStretchLastSection(True)
        self._coordinate_table.setMinimumHeight(200)
        coordinate_layout.addWidget(self._coordinate_table)
        left_layout.addWidget(coordinate_group, 1)
        right = QWidget()
        right_layout = QVBoxLayout(right)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(8)
        zoom_row = QHBoxLayout()
        zoom_row.setSpacing(6)
        zoom_hint = QLabel("캔버스 확대: 휠 확대/축소, 좌클릭 드래그 이동")
        zoom_hint.setStyleSheet("color: #64748b; font: 11px 'Malgun Gothic';")
        zoom_in_btn = QPushButton("확대")
        zoom_out_btn = QPushButton("축소")
        zoom_reset_btn = QPushButton("맞춤")
        for button in (zoom_in_btn, zoom_out_btn, zoom_reset_btn):
            button.setStyleSheet(
                "QPushButton { padding: 5px 10px; border: 1px solid #cbd5e1; border-radius: 8px; background: #ffffff; font: 11px 'Malgun Gothic'; }"
            )
        zoom_row.addWidget(zoom_hint)
        zoom_row.addStretch(1)
        zoom_row.addWidget(zoom_out_btn)
        zoom_row.addWidget(zoom_in_btn)
        zoom_row.addWidget(zoom_reset_btn)
        right_layout.addLayout(zoom_row)
        self._canvas_summary_label = QLabel("도형 표시: 할당(연한색) / 소거(녹색) / 잔여(주황) / centerline(파란 점선)")
        self._canvas_summary_label.setWordWrap(True)
        self._canvas_summary_label.setStyleSheet("padding: 10px 12px; border: 1px solid #dbe5f0; border-radius: 10px; background: #f8fbff; color: #334155; font: 11px 'Malgun Gothic';")
        right_layout.addWidget(self._canvas_summary_label)
        self._canvas = _MissionAreaCanvas()
        zoom_in_btn.clicked.connect(self._canvas.zoom_in)
        zoom_out_btn.clicked.connect(self._canvas.zoom_out)
        zoom_reset_btn.clicked.connect(self._canvas.reset_view)
        right_layout.addWidget(self._canvas, 1)
        splitter.addWidget(left)
        splitter.addWidget(right)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([430, 950])
        root.addWidget(splitter, 1)
        self._select_aircraft(self._selected_aircraft_id)
        self._request_refresh(force=True)

    def _default_selected_mission_id(self, row: dict[str, Any] | None) -> int | None:
        missions = list((row or {}).get("missions") or [])
        if not missions:
            return None
        selected_id = _as_int(self._selected_mission_id)
        if selected_id is not None:
            for mission in missions:
                if int(mission.get("mission_id") or -1) == int(selected_id):
                    return int(selected_id)
        for mission in missions:
            if mission.get("is_current"):
                current_id = _as_int(mission.get("mission_id"))
                if current_id is not None:
                    self._selected_mission_id = int(current_id)
                    return int(current_id)
        first_id = _as_int(missions[0].get("mission_id"))
        self._selected_mission_id = None if first_id is None else int(first_id)
        return first_id

    def _selected_mission_from_row(self, row: dict[str, Any] | None) -> dict[str, Any] | None:
        mission_id = self._default_selected_mission_id(row)
        if mission_id is None:
            return None
        for mission in (row or {}).get("missions") or []:
            if int(mission.get("mission_id") or -1) == int(mission_id):
                return mission
        return None

    def _populate_coordinate_table(self, mission: dict[str, Any] | None) -> None:
        table = self._coordinate_table
        summary = self._coordinate_summary_label
        if table is None:
            return
        groups = list((mission or {}).get("coordinate_overlay_groups") or [])
        coordinate_rows: list[list[str]] = []
        for group in groups:
            if not isinstance(group, dict):
                continue
            role_text = "Original" if str(group.get("role") or "") == "source" else "Remaining"
            geom_text = str(group.get("geometry_type") or "-").upper()
            if bool(group.get("is_hole")):
                geom_text = f"{geom_text}-HOLE"
            block_text = f"B{int(group.get('block_index') or 0)}"
            for point_info in group.get("points") or []:
                if not isinstance(point_info, dict):
                    continue
                altitude = _as_float(point_info.get("altitude"))
                note = f"P{int(point_info.get('index') or 0)}"
                coordinate_rows.append(
                    [
                        role_text,
                        geom_text,
                        f"{block_text} / {note}",
                        f"{float(point_info.get('latitude') or 0.0):.6f}",
                        f"{float(point_info.get('longitude') or 0.0):.6f}",
                        "-" if altitude is None else str(int(round(float(altitude)))),
                        note,
                    ]
                )
        table.setRowCount(len(coordinate_rows))
        for row_index, values in enumerate(coordinate_rows):
            for col_index, value in enumerate(values):
                item = table.item(row_index, col_index)
                if item is None:
                    item = QTableWidgetItem()
                    table.setItem(row_index, col_index, item)
                item.setText(str(value))
        if summary is None:
            return
        if mission is None:
            summary.setText("Mission row를 선택하면 원본/재구성 점 좌표가 표시됩니다.")
            return
        summary.setText(
            f"M{mission.get('mission_id') or '-'} / {str(mission.get('mission_type') or '-').upper()} / "
            f"overlay {len(groups)}개 / point {len(coordinate_rows)}개"
        )

    def _on_mission_table_selection_changed(self) -> None:
        table = self._mission_table
        if table is None:
            return
        selected_items = table.selectedItems()
        if not selected_items:
            return
        mission_id = _as_int(selected_items[0].data(Qt.UserRole))
        if mission_id is None:
            return
        if self._selected_mission_id == int(mission_id):
            return
        self._selected_mission_id = int(mission_id)
        self._request_refresh(force=True)

    def _load_mission_plan(self, mission_plan_id: int | None) -> None:
        previous_plan_id = _as_int((self._mission_view or {}).get("mission_plan_id"))
        preserve_same_plan = (
            previous_plan_id is not None
            and mission_plan_id is not None
            and int(previous_plan_id) == int(mission_plan_id)
        )
        previous_state_by_mission = dict(self._states)
        used_previous_mission_ids: set[int] = set()
        self._mission_view = build_uav_mission_view(mission_plan_id, uav_ids=_UAV_IDS)
        self._progress_tracker.reset(self._mission_view)
        self._progress_snapshot = self._progress_tracker.update(None, None)
        self._states = {}
        self._current_mission_by_aircraft = {}
        self._last_timestamp_ms = None
        for entry in (self._mission_view or {}).get("uav_entries") or []:
            aid = _as_int(entry.get("aircraft_id"))
            if aid is None:
                continue
            self._current_mission_by_aircraft[aid] = _as_int(entry.get("current_individual_mission_id"))
            current_mid = _as_int(self._current_mission_by_aircraft.get(aid))
            split_geometry_counts: dict[tuple[int, str], int] = {}
            for mission in entry.get("missions") or []:
                input_id = _as_int(mission.get("input_id"))
                mission_kind = _mission_geometry_kind(mission)
                if input_id is None or input_id <= 0 or mission_kind == "-":
                    continue
                key = (int(input_id), str(mission_kind))
                split_geometry_counts[key] = int(split_geometry_counts.get(key, 0)) + 1
            for mission in entry.get("missions") or []:
                mid = _as_int(mission.get("individual_mission_id"))
                if mid is None:
                    continue
                mission_type = _mission_geometry_kind(mission)
                coverage_def: MissionCoverageDefinition | None = None
                effective_source_detail: dict[str, Any] | None = None
                input_id = _as_int(mission.get("input_id"))
                duplicate_count = (
                    int(split_geometry_counts.get((int(input_id), str(mission_type)), 0))
                    if input_id is not None and input_id > 0 and mission_type != "-"
                    else 0
                )
                if duplicate_count > 1:
                    split_detail = _build_split_path_source_detail(mission)
                    if split_detail is not None:
                        mission_type, effective_source_detail, coverage_def = split_detail
                if coverage_def is None:
                    coverage_def = build_mission_coverage_definition(mission)
                if coverage_def is None:
                    continue
                done = bool(mission.get("is_done"))
                planned_cut_lines, cut_half_width_m = _build_planned_sweep_lines(mission, coverage_def)
                previous_state = previous_state_by_mission.get(mid) if preserve_same_plan else None
                if previous_state is not None:
                    used_previous_mission_ids.add(int(previous_state.mission_id))
                elif not done:
                    previous_state = _find_previous_matching_state(
                        previous_state_by_mission,
                        used_mission_ids=used_previous_mission_ids,
                        aircraft_id=int(aid),
                        input_id=_as_int(mission.get("input_id")),
                        mission_type=mission_type,
                        assignment_geometry=coverage_def.assignment_geometry,
                        planned_cut_lines=planned_cut_lines,
                    )
                    if previous_state is not None:
                        used_previous_mission_ids.add(int(previous_state.mission_id))
                preserved_covered = GeometryCollection()
                if previous_state is not None and not done:
                    try:
                        preserved_covered = coverage_def.assignment_geometry.intersection(previous_state.covered_geometry)
                    except Exception:
                        preserved_covered = GeometryCollection()
                covered = coverage_def.assignment_geometry if done else preserved_covered
                state = _MissionAreaState(
                    mid,
                    aid,
                    _as_int(mission.get("input_id")),
                    mission_type,
                    coverage_def,
                    _as_float(mission.get("width_m")),
                    coverage_def.assignment_geometry,
                    float(coverage_def.planned_area_m2),
                    source_line_list=deepcopy(
                        list((effective_source_detail or {}).get("lineList") or mission.get("line_list") or [])
                    ),
                    source_area_list=deepcopy(
                        list((effective_source_detail or {}).get("areaList") or mission.get("area_list") or [])
                    ),
                    source_coordinate_list=deepcopy(
                        list((effective_source_detail or {}).get("coordinateList") or mission.get("coordinate_list") or [])
                    ),
                    input_line_list=deepcopy(mission.get("input_line_list") or []),
                    input_area_list=deepcopy(mission.get("input_area_list") or []),
                    input_coordinate_list=deepcopy(mission.get("input_coordinate_list") or []),
                    is_current=bool(current_mid == mid),
                    covered_geometry=covered,
                    planned_cut_lines=planned_cut_lines,
                    cut_half_width_m=cut_half_width_m,
                    last_cut_line_index=(len(planned_cut_lines) - 1) if done and planned_cut_lines else -1,
                    preferred_track_vector=_mission_track_vector(mission, coverage_def),
                    done=done,
                )
                if done and planned_cut_lines:
                    state.completed_cut_line_indexes = set(range(len(planned_cut_lines)))
                    state.cut_lines = list(planned_cut_lines[-800:])
                    state.progress_origin_line_index = 0 if planned_cut_lines else None
                    state.progress_boundary_line_index = (len(planned_cut_lines) - 1) if planned_cut_lines else None
                    state.progress_direction_sign = 1 if planned_cut_lines else None
                elif previous_state is not None and preserve_same_plan and int(previous_state.mission_id) == int(mid):
                    state.centerline_points = list(previous_state.centerline_points[-800:])
                    state.cut_lines = list(previous_state.cut_lines[-800:])
                    state.completed_cut_line_indexes = set(previous_state.completed_cut_line_indexes)
                    state.last_cut_line_index = int(previous_state.last_cut_line_index)
                    state.progress_origin_line_index = previous_state.progress_origin_line_index
                    state.progress_boundary_line_index = previous_state.progress_boundary_line_index
                    state.progress_direction_sign = previous_state.progress_direction_sign
                    state.last_nearest_cut_line_index = previous_state.last_nearest_cut_line_index
                    state.tracking_cut_line_index = previous_state.tracking_cut_line_index
                    state.tracking_projection_min_m = previous_state.tracking_projection_min_m
                    state.tracking_projection_max_m = previous_state.tracking_projection_max_m
                    state.tracking_sample_count = previous_state.tracking_sample_count
                    state.tracking_path_length_m = previous_state.tracking_path_length_m
                    state.tracking_last_center_xy = previous_state.tracking_last_center_xy
                    state.provisional_frontier_line_index = previous_state.provisional_frontier_line_index
                    state.last_center_xy = previous_state.last_center_xy
                    state.last_update_ms = previous_state.last_update_ms
                    if state.completed_cut_line_indexes:
                        _rebuild_completed_sweep_coverage(state)
                elif previous_state is not None:
                    state.centerline_points = list(previous_state.centerline_points[-800:])
                    state.last_center_xy = previous_state.last_center_xy
                    state.last_update_ms = previous_state.last_update_ms
                    _restore_planned_cut_progress(state)
                else:
                    _restore_planned_cut_progress(state)
                self._states[mid] = state
        self._invalidate_runtime_cache()
        self._request_refresh(force=True)

    def _select_aircraft(self, aircraft_id: int) -> None:
        self._selected_aircraft_id = int(aircraft_id)
        self._selected_mission_id = None
        self._invalidate_runtime_cache(selection_only=True)
        for aid, btn in self._aircraft_buttons.items():
            selected = int(aid) == self._selected_aircraft_id
            btn.setChecked(selected)
            if selected:
                color = _UAV_COLORS.get(int(aid), "#2563eb")
                btn.setStyleSheet(f"QPushButton {{ background: {color}; color: white; padding: 6px 14px; border-radius: 8px; font-weight: 700; }}")
            else:
                btn.setStyleSheet("QPushButton { background: #e2e8f0; color: #0f172a; padding: 6px 14px; border-radius: 8px; }")
        self._request_refresh(force=True)

    def _aircraft_rows(self) -> dict[int, dict[str, Any]]:
        cache_key = (int(self._state_cache_token), int(self._selected_aircraft_id))
        if self._rows_cache is not None and self._rows_cache_key == cache_key:
            return self._rows_cache
        rows: dict[int, dict[str, Any]] = {}
        progress = self._progress_snapshot.get("mission_progress") or {}
        for entry in (self._mission_view or {}).get("uav_entries") or []:
            aid = _as_int(entry.get("aircraft_id"))
            if aid is None:
                continue
            show_preview = int(aid) == int(self._selected_aircraft_id)
            missions_out: list[dict[str, Any]] = []
            plan_area = 0.0
            covered_area = 0.0
            done_count = 0
            current_mid = _as_int(self._current_mission_by_aircraft.get(aid))
            for mission in entry.get("missions") or []:
                mid = _as_int(mission.get("individual_mission_id"))
                if mid is None:
                    continue
                state = self._states.get(mid)
                assign = state.assignment_geometry if state is not None else GeometryCollection()
                progress_done = bool((progress.get(mid) or {}).get("done"))
                geometry_done = _is_state_geometrically_done(state) if state is not None else False
                done = (
                    bool(mission.get("is_done"))
                    or bool(state.done if state is not None else False)
                    or bool(geometry_done)
                    or bool(progress_done and geometry_done)
                )
                covered = state.assignment_geometry if done and state is not None else state.covered_geometry if state is not None else GeometryCollection()
                if done and state is not None:
                    state.done = True
                    state.covered_geometry = state.assignment_geometry
                planned_area_m2 = float(state.planned_area_m2) if state is not None else 0.0
                if state is not None and not assign.is_empty and not done:
                    try:
                        covered = assign.intersection(state.covered_geometry)
                    except Exception:
                        covered = state.covered_geometry
                    covered = _fill_geometry_holes(covered)
                    remaining = self._get_cached_remaining_geometry(state)
                    try:
                        covered = assign.difference(remaining)
                    except Exception:
                        covered = covered
                else:
                    remaining = GeometryCollection()
                covered_area_m2 = min(planned_area_m2, float(covered.area if state is not None else 0.0))
                percent = int(round((covered_area_m2 / planned_area_m2) * 100)) if planned_area_m2 > 0.0 else 0
                percent = max(0, min(100, percent))
                if done:
                    done_count += 1
                plan_area += planned_area_m2
                covered_area += covered_area_m2
                source_detail = {"coordinateList": [], "lineList": [], "areaList": []}
                remaining_detail = {"coordinateList": [], "lineList": [], "areaList": []}
                coordinate_overlay_groups: list[dict[str, Any]] = []
                preview_source_lines: list[BaseGeometry] = []
                preview_remaining_lines: list[BaseGeometry] = []
                if show_preview and state is not None:
                    source_detail = _build_source_detail_from_state(state)
                    if not done:
                        remaining_detail, _ = self._get_cached_state_remaining_detail(state)
                    coordinate_overlay_groups = _build_coordinate_overlay_groups(
                        state,
                        source_detail=source_detail,
                        remaining_detail=remaining_detail,
                    )
                    if str(state.mission_type) == "line" and not done:
                        preview_source_lines, preview_remaining_lines = self._get_cached_preview_lines(state)
                missions_out.append({
                    "mission_id": mid,
                    "mission_type": "line" if mission.get("line_list") else "area" if mission.get("area_list") else "-",
                    "is_done": done,
                    "is_current": current_mid == mid,
                    "coverage_percent": percent,
                    "planned_area_m2": planned_area_m2,
                    "covered_geometry": covered,
                    "assignment_geometry": assign,
                    "remaining_geometry": remaining,
                    "remaining_area_m2": max(0.0, planned_area_m2 - covered_area_m2),
                    "centerline_points": list(state.centerline_points) if state is not None else [],
                    "cut_lines": list(state.cut_lines) if state is not None else [],
                    "planned_cut_lines": list(state.planned_cut_lines) if state is not None else [],
                    "preview_source_lines": preview_source_lines,
                    "preview_remaining_lines": preview_remaining_lines,
                    "source_detail": source_detail,
                    "remaining_detail": remaining_detail,
                    "coordinate_overlay_groups": coordinate_overlay_groups,
                    "last_update_ms": state.last_update_ms if state is not None else None,
                })
            total_percent = int(round((covered_area / plan_area) * 100)) if plan_area > 0.0 else 0
            rows[aid] = {
                "aircraft_id": aid,
                "current_mission_id": current_mid,
                "mission_count": len(missions_out),
                "done_count": done_count,
                "planned_area_m2": plan_area,
                "covered_area_m2": covered_area,
                "remaining_area_m2": max(0.0, plan_area - covered_area),
                "coverage_percent": max(0, min(100, total_percent)),
                "missions": missions_out,
            }
        self._rows_cache_key = cache_key
        self._rows_cache = rows
        return rows

    def _build_state_remaining_detail(self, state: _MissionAreaState) -> tuple[dict[str, Any], float]:
        remaining = self._get_cached_remaining_geometry(state)
        if remaining.is_empty:
            return {"coordinateList": [], "lineList": [], "areaList": []}, 0.0

        transformer = state.coverage_def.transformer
        altitude = _representative_altitude_from_coords(
            [
                [
                    item
                    for item in (line.get("coordinateList") or [])
                    if isinstance(item, dict)
                ]
                for line in state.source_line_list
                if isinstance(line, dict)
            ]
            + [
                [
                    item
                    for item in (area.get("coordinateList") or [])
                    if isinstance(item, dict)
                ]
                for area in state.source_area_list
                if isinstance(area, dict)
            ]
            + [list(state.source_coordinate_list or [])]
        )

        if str(state.mission_type) == "line":
            source_lines = _build_source_line_geometries(
                list(state.source_line_list or state.input_line_list or []),
                list(state.source_coordinate_list or state.input_coordinate_list or []),
                transformer,
                default_width_m=float(state.width_hint_m or 1.0),
            )
            clip_margin_m = max(2.5, float(state.cut_half_width_m) * 0.9)
            min_length_m = max(3.0, float(state.cut_half_width_m) * 0.85)
            current_source_index = (
                _nearest_source_line_index(source_lines, state.last_center_xy)
                if bool(state.is_current)
                else None
            )
            source_indexes = (
                list(range(int(current_source_index), len(source_lines)))
                if current_source_index is not None
                else None
            )
            line_blocks = _build_remaining_line_blocks(
                source_lines,
                clip_geometry=remaining,
                transformer=transformer,
                altitude=altitude,
                min_length_m=float(min_length_m),
                clip_margin_m=float(clip_margin_m),
                current_source_index=current_source_index,
                current_state=state,
                source_indexes=source_indexes,
            )
            coordinate_list = deepcopy(line_blocks[0]["coordinateList"]) if line_blocks else []
            return {
                "coordinateList": coordinate_list,
                "lineList": line_blocks,
                "areaList": [],
            }, float(max(0.0, remaining.area or 0.0))

        area_threshold_m2 = max(8.0, float(state.cut_half_width_m) * float(state.cut_half_width_m) * 0.6)
        hole_threshold_m2 = max(
            area_threshold_m2,
            min(float(state.planned_area_m2) * 0.002, 40.0),
        )
        area_blocks = _area_blocks_from_geometry(
            remaining,
            transformer,
            altitude=altitude,
            area_threshold_m2=area_threshold_m2,
            hole_threshold_m2=hole_threshold_m2,
        )
        coordinate_list = deepcopy(
            next(
                (
                    block.get("coordinateList")
                    for block in area_blocks
                    if not bool(block.get("isHole")) and isinstance(block.get("coordinateList"), list)
                ),
                [],
            )
        )
        return {
            "coordinateList": coordinate_list,
            "lineList": [],
            "areaList": area_blocks,
        }, float(max(0.0, remaining.area or 0.0))

    def _build_group_remaining_detail(self, states: list[_MissionAreaState]) -> tuple[dict[str, Any], float]:
        if not states:
            return {"coordinateList": [], "lineList": [], "areaList": []}, 0.0

        reference_state = states[0]
        transformer = reference_state.coverage_def.transformer
        assignment_geometry = _merge_state_geometries([state.assignment_geometry for state in states])
        remaining_geometry = _merge_state_geometries([self._get_cached_remaining_geometry(state) for state in states])
        if not assignment_geometry.is_empty:
            try:
                remaining_geometry = assignment_geometry.intersection(remaining_geometry)
            except Exception:
                pass
        if remaining_geometry.is_empty:
            return {"coordinateList": [], "lineList": [], "areaList": []}, 0.0

        altitude = _representative_altitude_from_coords(
            [
                [
                    item
                    for item in (line.get("coordinateList") or [])
                    if isinstance(item, dict)
                ]
                for state in states
                for line in (state.input_line_list or [])
                if isinstance(line, dict)
            ]
            + [
                [
                    item
                    for item in (area.get("coordinateList") or [])
                    if isinstance(item, dict)
                ]
                for state in states
                for area in (state.input_area_list or [])
                if isinstance(area, dict)
            ]
            + [list(state.input_coordinate_list or []) for state in states]
            + [
                [
                    item
                    for item in (line.get("coordinateList") or [])
                    if isinstance(item, dict)
                ]
                for state in states
                for line in (state.source_line_list or [])
                if isinstance(line, dict)
            ]
            + [
                [
                    item
                    for item in (area.get("coordinateList") or [])
                    if isinstance(item, dict)
                ]
                for state in states
                for area in (state.source_area_list or [])
                if isinstance(area, dict)
            ]
            + [list(state.source_coordinate_list or []) for state in states]
        )

        if str(reference_state.mission_type) == "line":
            input_line_list: list[dict[str, Any]] = []
            input_coordinate_list: list[dict[str, Any]] = []
            for state in states:
                if not input_line_list and state.input_line_list:
                    input_line_list = deepcopy(state.input_line_list)
                if not input_coordinate_list and state.input_coordinate_list:
                    input_coordinate_list = deepcopy(state.input_coordinate_list)
                if input_line_list and input_coordinate_list:
                    break
            if not input_line_list and not input_coordinate_list:
                for state in states:
                    if not input_line_list and state.source_line_list:
                        input_line_list = deepcopy(state.source_line_list)
                    if not input_coordinate_list and state.source_coordinate_list:
                        input_coordinate_list = deepcopy(state.source_coordinate_list)
                    if input_line_list or input_coordinate_list:
                        break

            if input_line_list or input_coordinate_list:
                width_candidates = [
                    float(line.get("width"))
                    for line in input_line_list
                    if isinstance(line, dict) and _as_float(line.get("width")) is not None
                ]
                default_width_m = max(
                    (width_candidates if width_candidates else [0.0])
                    + [float(state.width_hint_m or 0.0) for state in states]
                    + [1.0]
                )
                source_lines = _build_source_line_geometries(
                    input_line_list,
                    input_coordinate_list,
                    transformer,
                    default_width_m=float(default_width_m),
                )
                min_length_m = max(3.0, min(float(default_width_m) * 0.08, 60.0))
                current_state = next(
                    (
                        state
                        for state in states
                        if bool(state.is_current) and state.last_center_xy is not None
                    ),
                    None,
                )
                if current_state is None:
                    current_state = next(
                        (
                            state
                            for state in sorted(
                                states,
                                key=lambda item: int(item.last_update_ms or 0),
                                reverse=True,
                            )
                            if state.last_center_xy is not None
                        ),
                        None,
                    )
                current_source_index: int | None = None
                if source_lines:
                    best_candidate: tuple[int, float, float, _MissionAreaState] | None = None
                    candidate_states = [
                        state
                        for state in states
                        if state.last_center_xy is not None
                    ]
                    if not candidate_states and current_state is not None:
                        candidate_states = [current_state]
                    for candidate_state in candidate_states:
                        if candidate_state.last_center_xy is None:
                            continue
                        current_point = Point(candidate_state.last_center_xy)
                        progress_vec = _state_progress_vector(candidate_state)
                        for source_idx, (source_line, _width_m) in enumerate(source_lines):
                            try:
                                source_distance = float(source_line.distance(current_point))
                            except Exception:
                                continue
                            oriented = _orient_line_toward_progress(source_line, progress_vec)
                            try:
                                projection_m = float(oriented.project(current_point))
                            except Exception:
                                projection_m = 0.0
                            candidate = (
                                int(source_idx),
                                float(projection_m),
                                -float(source_distance),
                                candidate_state,
                            )
                            if best_candidate is None or candidate > best_candidate:
                                best_candidate = candidate
                    if best_candidate is not None:
                        current_source_index = int(best_candidate[0])
                        current_state = best_candidate[3]
                ordered_indexes = (
                    list(range(int(current_source_index), len(source_lines)))
                    if current_source_index is not None
                    else list(range(len(source_lines)))
                )
                clip_margin_m = max(
                    2.5,
                    float(default_width_m) * 0.18,
                    max(float(state.cut_half_width_m) for state in states) * 0.9,
                )
                line_blocks = _build_remaining_line_blocks(
                    source_lines,
                    clip_geometry=remaining_geometry,
                    transformer=transformer,
                    altitude=altitude,
                    min_length_m=float(min_length_m),
                    clip_margin_m=float(clip_margin_m),
                    current_source_index=current_source_index,
                    current_state=current_state,
                    source_indexes=ordered_indexes,
                )
                if line_blocks:
                    coordinate_list = deepcopy(line_blocks[0]["coordinateList"]) if len(line_blocks) == 1 else []
                    return {
                        "coordinateList": coordinate_list,
                        "lineList": line_blocks,
                        "areaList": [],
                    }, float(max(0.0, remaining_geometry.area or 0.0))

            line_block_rows: list[tuple[int, float, tuple[float, float], dict[str, Any]]] = []
            strip_track_vector = reference_state.preferred_track_vector
            strip_default_widths = [float(state.width_hint_m or 0.0) for state in states] + [1.0]
            strip_min_length_m = max(3.0, min(max(strip_default_widths) * 0.08, 60.0))
            for state in states:
                state_remaining = self._get_cached_remaining_geometry(state)
                if state_remaining.is_empty:
                    continue
                state_line_list = list(state.source_line_list or [])
                state_coordinate_list = list(state.source_coordinate_list or [])
                if not state_line_list and not state_coordinate_list:
                    continue
                state_default_width_m = max(
                    [
                        float(line.get("width"))
                        for line in state_line_list
                        if isinstance(line, dict) and _as_float(line.get("width")) is not None
                    ]
                    + [float(state.width_hint_m or 0.0), 1.0]
                )
                state_source_lines = _build_source_line_geometries(
                    state_line_list,
                    state_coordinate_list,
                    transformer,
                    default_width_m=float(state_default_width_m),
                )
                current_source_index = (
                    _nearest_source_line_index(state_source_lines, state.last_center_xy)
                    if bool(state.is_current)
                    else None
                )
                source_indexes = (
                    list(range(int(current_source_index), len(state_source_lines)))
                    if current_source_index is not None
                    else None
                )
                state_line_blocks = _build_remaining_line_blocks(
                    state_source_lines,
                    clip_geometry=state_remaining,
                    transformer=transformer,
                    altitude=altitude,
                    min_length_m=float(strip_min_length_m),
                    clip_margin_m=max(1.0, float(state.cut_half_width_m) * 0.9),
                    current_source_index=current_source_index,
                    current_state=state,
                    source_indexes=source_indexes,
                )
                for block in state_line_blocks:
                    coord_list = list(block.get("coordinateList") or [])
                    width_m = float(block.get("width") or state_default_width_m)
                    if not coord_list:
                        continue
                    line_block_rows.append(
                        (
                            0 if bool(state.is_current) else 1,
                            _line_block_start_distance_to_current_m(
                                coord_list,
                                transformer,
                                state.last_center_xy,
                            ),
                            _line_block_lateral_sort_key(
                                {
                                    "coordinateList": coord_list,
                                },
                                transformer,
                                strip_track_vector,
                            ),
                            {
                                "width": float(width_m),
                                "coordinateList": coord_list,
                            },
                        )
                    )

            if line_block_rows:
                line_block_rows.sort(
                    key=lambda row: (
                        int(row[0]),
                        float(row[1]),
                        float(row[2][0]),
                        float(row[2][1]),
                    )
                )
                line_blocks = [row[3] for row in line_block_rows]
                coordinate_list = (
                    deepcopy(line_blocks[0]["coordinateList"])
                    if len(line_blocks) == 1
                    else []
                )
                return {
                    "coordinateList": coordinate_list,
                    "lineList": line_blocks,
                    "areaList": [],
                }, float(max(0.0, remaining_geometry.area or 0.0))

            return {
                "coordinateList": [],
                "lineList": [],
                "areaList": [],
            }, float(max(0.0, remaining_geometry.area or 0.0))

        area_threshold_m2 = max(
            8.0,
            min(
                [float(state.cut_half_width_m) * float(state.cut_half_width_m) * 0.6 for state in states]
                + [8.0]
            ),
        )
        hole_threshold_m2 = max(
            area_threshold_m2,
            min(float(max(float(state.planned_area_m2 or 0.0) for state in states)) * 0.002, 40.0),
        )
        area_blocks = _area_blocks_from_geometry(
            remaining_geometry,
            transformer,
            altitude=altitude,
            area_threshold_m2=float(area_threshold_m2),
            hole_threshold_m2=float(hole_threshold_m2),
        )
        coordinate_list = deepcopy(
            next(
                (
                    block.get("coordinateList")
                    for block in area_blocks
                    if not bool(block.get("isHole")) and isinstance(block.get("coordinateList"), list)
                ),
                [],
            )
        )
        return {
            "coordinateList": coordinate_list,
            "lineList": [],
            "areaList": area_blocks,
        }, float(max(0.0, remaining_geometry.area or 0.0))

    def _build_replan_snapshot(self) -> dict[str, Any] | None:
        if self._snapshot_cache is not None and self._snapshot_cache_key == int(self._state_cache_token):
            return self._snapshot_cache
        plan_id = _as_int((self._mission_view or {}).get("mission_plan_id"))
        if plan_id is None or plan_id <= 0:
            return None

        grouped: dict[int, dict[str, Any]] = {}
        for state in self._states.values():
            input_id = _as_int(state.input_id)
            if input_id is None or input_id <= 0:
                continue
            entry = grouped.setdefault(
                int(input_id),
                {
                    "inputMissionID": int(input_id),
                    "missionType": str(state.mission_type or "-"),
                    "individualMissionIDs": [],
                    "aircraftIDs": [],
                    "states": [],
                },
            )
            entry["individualMissionIDs"].append(int(state.mission_id))
            entry["aircraftIDs"].append(int(state.aircraft_id))
            entry["states"].append(state)

        missions: list[dict[str, Any]] = []
        for input_id in sorted(grouped):
            entry = grouped[int(input_id)]
            states = list(entry.get("states") or [])
            assignment_geometry = _merge_state_geometries([state.assignment_geometry for state in states])
            planned_area_m2 = float(max(0.0, assignment_geometry.area or 0.0))
            if planned_area_m2 <= 0.0:
                planned_area_m2 = float(sum(float(state.planned_area_m2 or 0.0) for state in states))
            remaining_detail, remaining_area_m2 = self._get_cached_group_remaining_detail(states)
            remaining_area_m2 = max(0.0, float(remaining_area_m2 or 0.0))
            covered_area_m2 = max(0.0, planned_area_m2 - remaining_area_m2)
            coverage_percent = int(round((covered_area_m2 / planned_area_m2) * 100.0)) if planned_area_m2 > 0.0 else 0
            line_list = list((remaining_detail or {}).get("lineList") or [])
            area_list = list((remaining_detail or {}).get("areaList") or [])
            coordinate_list = list((remaining_detail or {}).get("coordinateList") or [])
            source_line_width_m: float | None = None
            source_coordinate_list: list[dict[str, Any]] = []
            if str(entry.get("missionType") or "-") == "line":
                for state in states:
                    if source_line_width_m is None:
                        width_candidates = [
                            float(line.get("width"))
                            for line in (state.input_line_list or [])
                            if isinstance(line, dict) and _as_float(line.get("width")) is not None
                        ]
                        if not width_candidates:
                            width_candidates = [
                                float(line.get("width"))
                                for line in (state.source_line_list or [])
                                if isinstance(line, dict) and _as_float(line.get("width")) is not None
                            ]
                        if width_candidates:
                            source_line_width_m = float(max(width_candidates))
                        elif _as_float(state.width_hint_m) is not None and float(state.width_hint_m or 0.0) > 0.0:
                            source_line_width_m = float(state.width_hint_m or 0.0)
                    if len(source_coordinate_list) >= 2:
                        continue
                    for line in (state.input_line_list or []):
                        if not isinstance(line, dict):
                            continue
                        coords = [
                            dict(coord)
                            for coord in (line.get("coordinateList") or [])
                            if isinstance(coord, dict)
                        ]
                        if len(coords) >= 2:
                            source_coordinate_list = coords
                            break
                    if len(source_coordinate_list) >= 2:
                        continue
                    coords = [dict(coord) for coord in (state.input_coordinate_list or []) if isinstance(coord, dict)]
                    if len(coords) >= 2:
                        source_coordinate_list = coords
                        continue
                    coords = [dict(coord) for coord in (state.source_coordinate_list or []) if isinstance(coord, dict)]
                    if len(coords) >= 2:
                        source_coordinate_list = coords
            is_done = not line_list and not area_list and len(coordinate_list) < 2
            missions.append(
                {
                    "inputMissionID": int(input_id),
                    "missionType": str(entry.get("missionType") or "-"),
                    "individualMissionIDs": sorted({int(v) for v in entry.get("individualMissionIDs") or []}),
                    "aircraftIDs": sorted({int(v) for v in entry.get("aircraftIDs") or []}),
                    "plannedAreaM2": float(planned_area_m2),
                    "remainingAreaM2": float(remaining_area_m2),
                    "coveragePercent": max(0, min(100, coverage_percent)),
                    "isDone": bool(is_done),
                    "sourceLineWidthM": float(source_line_width_m) if source_line_width_m is not None else None,
                    "sourceCoordinateList": deepcopy(source_coordinate_list) if len(source_coordinate_list) >= 2 else [],
                    "remainingDetail": {
                        "coordinateList": coordinate_list,
                        "lineList": line_list,
                        "areaList": area_list,
                    },
                }
            )

        snapshot = {
            "missionPlanID": int(plan_id),
            "timestamp": int(self._last_timestamp_ms) if self._last_timestamp_ms is not None else None,
            "missionCount": len(missions),
            "missions": missions,
        }
        self._snapshot_cache_key = int(self._state_cache_token)
        self._snapshot_cache = snapshot
        return snapshot

    def _persist_replan_snapshot(self) -> None:
        snapshot = self._build_replan_snapshot()
        if not snapshot:
            self._last_snapshot_signature = None
            return
        plan_id = _as_int(snapshot.get("missionPlanID"))
        mission_count = int(snapshot.get("missionCount") or 0)
        if plan_id is not None and plan_id > 0 and mission_count <= 0:
            existing = mission_area_replan_store.load_snapshot(int(plan_id))
            existing_mission_count = int((existing or {}).get("missionCount") or 0)
            if existing_mission_count > 0:
                return
        try:
            signature = json.dumps(
                snapshot,
                ensure_ascii=False,
                sort_keys=True,
                separators=(",", ":"),
            )
        except Exception:
            return
        if signature == self._last_snapshot_signature:
            return
        if plan_id is None or plan_id <= 0:
            return
        try:
            mission_area_replan_store.save_snapshot(int(plan_id), snapshot)
        except Exception:
            return
        self._last_snapshot_signature = signature

    def _refresh_view(self) -> None:
        if not self._ui_updates_enabled:
            self._dirty = True
            self._persist_replan_snapshot()
            return
        self._dirty = False
        rows = self._aircraft_rows()
        self._persist_replan_snapshot()
        if self._plan_summary_label is not None:
            self._plan_summary_label.setText(f"MissionPlan: {(self._mission_view or {}).get('mission_plan_id') or '-'} / Last Update: {format_timestamp_ms(self._last_timestamp_ms)}")
        for aid, widgets in self._summary_cards.items():
            frame, value_label, detail_label = widgets
            row = rows.get(aid)
            border = _UAV_COLORS.get(aid, "#334155")
            frame.setStyleSheet(
                f"QFrame#missionAreaSummaryCard {{ border: 1px solid {border}; border-left: 5px solid {border}; border-radius: 12px; background: #ffffff; }}"
            )
            if row is None:
                value_label.setText("소거율 -")
                detail_label.setText("할당 - / 잔여 -")
                continue
            value_label.setText(f"소거율 {int(row.get('coverage_percent') or 0)}%")
            detail_label.setText(f"현재 임무: {row.get('current_mission_id') or '-'}\n완료: {row.get('done_count')}/{row.get('mission_count')} | 할당: {_format_area(row.get('planned_area_m2'))} | 잔여: {_format_area(row.get('remaining_area_m2'))}")
        selected = rows.get(self._selected_aircraft_id)
        selected_mission = self._selected_mission_from_row(selected)
        selected_mission_id = _as_int((selected_mission or {}).get("mission_id"))
        if self._canvas is not None:
            if selected is None:
                self._canvas.set_payload(None)
            else:
                canvas_payload = dict(selected)
                canvas_payload["selected_mission_id"] = selected_mission_id
                self._canvas.set_payload(canvas_payload)
        if self._canvas_summary_label is not None:
            if selected is None:
                self._canvas_summary_label.setText("도형 표시: 할당(연한색) / 소거(녹색) / 잔여(주황) / centerline(파란 점선)")
            else:
                mission_label = f" / 선택 임무 {selected_mission_id}" if selected_mission_id is not None else ""
                self._canvas_summary_label.setText(
                    f"UAV {self._selected_aircraft_id} / 현재 임무 {selected.get('current_mission_id') or '-'}"
                    f"{mission_label} / 소거율 {int(selected.get('coverage_percent') or 0)}% / 잔여 {_format_area(selected.get('remaining_area_m2'))}"
                )
        table = self._mission_table
        missions = selected.get("missions") if isinstance(selected, dict) else []
        if table is None:
            self._populate_coordinate_table(selected_mission)
            return
        table.setRowCount(len(missions or []))
        for row_idx, mission in enumerate(missions or []):
            status = "완료" if mission.get("is_done") else "진행중" if mission.get("is_current") else "대기"
            values = [
                str(mission.get("mission_id") or "-"),
                str(mission.get("mission_type") or "-"),
                status,
                f"{int(mission.get('coverage_percent') or 0)}%",
                _format_area(mission.get("remaining_area_m2")),
                _format_area(mission.get("planned_area_m2")),
                format_timestamp_ms(mission.get("last_update_ms")),
            ]
            for col_idx, value in enumerate(values):
                item = table.item(row_idx, col_idx)
                if item is None:
                    item = QTableWidgetItem()
                    table.setItem(row_idx, col_idx, item)
                item.setText(value)
                if col_idx == 0:
                    item.setData(Qt.UserRole, _as_int(mission.get("mission_id")))
                if mission.get("is_done"):
                    item.setBackground(QColor("#dcfce7"))
                elif mission.get("is_current"):
                    item.setBackground(QColor("#dbeafe"))
                else:
                    item.setBackground(QColor("#ffffff"))
        table.blockSignals(True)
        table.clearSelection()
        if selected_mission_id is not None:
            for row_idx, mission in enumerate(missions or []):
                if int(mission.get("mission_id") or -1) != int(selected_mission_id):
                    continue
                table.selectRow(row_idx)
                break
        table.blockSignals(False)
        self._populate_coordinate_table(selected_mission)
