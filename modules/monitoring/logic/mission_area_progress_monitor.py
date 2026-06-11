# -*- coding: utf-8 -*-
from __future__ import annotations

import json
import math
import os
import time
from copy import deepcopy
from dataclasses import dataclass, field
from typing import Any, Callable

from pyproj.enums import TransformDirection
from shapely.geometry import GeometryCollection, LineString, Point, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import linemerge, unary_union

from modules.common import mission_area_replan_store
from modules.monitoring.logic.mission_coverage import (
    MissionCoverageDefinition,
    build_mission_coverage_definition,
    build_footprint_geometry,
    merge_coverage_geometry,
)
from modules.monitoring.logic.mission_area_row_progress import (
    AreaRowProgressDefinition,
    AreaRowProgressState,
    AreaRowProgressUpdate,
    build_area_row_progress_definition,
    clone_area_row_progress_state,
    reset_area_row_live_state,
    update_area_row_progress_state,
)
from modules.monitoring.logic.coverage_progress_store import persist_coverage_progress
from modules.monitoring.logic.mission_progress import MissionProgressTracker
from modules.monitoring.logic.mission_update import build_uav_mission_view

_UAV_IDS = (4, 5, 6)
_UAV_COLORS = {4: "#2563eb", 5: "#0f766e", 6: "#b45309"}
_DEFAULT_STRIP_WIDTH_M = 25.0
_AREA_ROW_PROGRESS_MIN_INTERVAL_MS = 200
_AREA_FOOTPRINT_CONFIRM_SAMPLE_COUNT = 2
_AREA_FOOTPRINT_CONFIRM_MIN_INTERVAL_MS = 300
_AREA_FOOTPRINT_CONFIRM_DIRECT_ADVANCE_ROWS = 1
_AREA_PASSED_KEEPBACK_RATIO_ENV = "KU_AREA_PASSED_KEEPBACK_RATIO"
_AREA_PASSED_KEEPBACK_RATIO_DEFAULT = 0.10


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


def _area_capture_progress_allowed(
    state: _MissionAreaState,
    *,
    current_waypoint_id: int | None,
    filming_value: int | None,
) -> bool:
    if str(state.mission_type) != "area":
        return True
    if filming_value == 0:
        return False
    if current_waypoint_id is not None and state.sweep_waypoint_ids:
        return int(current_waypoint_id) in state.sweep_waypoint_ids
    if filming_value is not None and int(filming_value) > 0:
        return True
    return False


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
        float(_as_float(mission.get("width_m")) or _DEFAULT_STRIP_WIDTH_M),
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
    sweep_point_count = sum(
        len(line.get("coordinateList") or [])
        for line in sweep_line_list
        if isinstance(line, dict)
    )

    sweep_cov = build_mission_coverage_definition(
        {
            "line_list": sweep_line_list,
            "area_list": [],
            "sweep_point_count": int(max(1, sweep_point_count)),
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
                    "sweep_point_count": int(max(1, sweep_point_count)),
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
                    "sweep_point_count": int(max(1, sweep_point_count)),
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


@dataclass
class _MissionAreaState:
    mission_id: int
    aircraft_id: int
    input_id: int | None
    mission_type: str
    source_plan_id: int | None
    path_id: int | None
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
    area_row_definition: AreaRowProgressDefinition | None = None
    area_row_progress_state: AreaRowProgressState = field(default_factory=AreaRowProgressState)
    sweep_waypoint_ids: set[int] = field(default_factory=set)
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
    area_progress_source: str | None = None
    area_progress_sweep_points: int | None = None
    area_progress_sweep_point_count: int | None = None
    area_progress_boundary_line_index: int | None = None
    area_progress_current_waypoint_id: int | None = None
    area_progress_confidence: str | None = None
    area_row_progress_last_sample_ms: int | None = None
    area_footprint_candidate_line_index: int | None = None
    area_footprint_candidate_seen_count: int = 0
    area_footprint_candidate_first_ms: int | None = None
    area_footprint_candidate_last_ms: int | None = None
    coverage_rebuild_signature: tuple[Any, ...] | None = None


def _planned_cut_line_signature(planned_lines: list[BaseGeometry]) -> tuple[tuple[int, int, int], ...]:
    signature: list[tuple[int, int, int]] = []
    for line in planned_lines or []:
        center = _planned_line_center_xy(line)
        try:
            length_m = float(getattr(line, "length", 0.0) or 0.0)
        except Exception:
            length_m = 0.0
        if center is None:
            signature.append((0, 0, int(round(length_m * 10.0))))
            continue
        signature.append(
            (
                int(round(float(center[0]) * 10.0)),
                int(round(float(center[1]) * 10.0)),
                int(round(length_m * 10.0)),
            )
        )
    return tuple(signature)


def _area_state_matches_new_baseline(
    previous_state: _MissionAreaState,
    *,
    aircraft_id: int,
    input_id: int | None,
    path_id: int | None,
    assignment_geometry: BaseGeometry,
    planned_cut_lines: list[BaseGeometry],
) -> bool:
    if int(previous_state.aircraft_id) != int(aircraft_id):
        return False
    if previous_state.input_id != input_id:
        return False
    if previous_state.path_id != path_id:
        return False
    if len(previous_state.planned_cut_lines) != len(planned_cut_lines):
        return False
    if _planned_cut_line_signature(previous_state.planned_cut_lines) != _planned_cut_line_signature(planned_cut_lines):
        return False
    try:
        previous_area = float(previous_state.assignment_geometry.area or 0.0)
        current_area = float(assignment_geometry.area or 0.0)
    except Exception:
        return False
    area_tol = max(5.0, max(previous_area, current_area) * 0.001)
    if abs(previous_area - current_area) > area_tol:
        return False
    try:
        prev_bounds = previous_state.assignment_geometry.bounds
        curr_bounds = assignment_geometry.bounds
    except Exception:
        return False
    return all(abs(float(prev_bounds[idx]) - float(curr_bounds[idx])) <= 2.0 for idx in range(4))


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


def _merge_completed_coverage_parts(geometries: list[BaseGeometry]) -> BaseGeometry:
    valid_parts = [
        geometry
        for geometry in geometries
        if isinstance(geometry, BaseGeometry) and not geometry.is_empty
    ]
    if not valid_parts:
        return GeometryCollection()
    if len(valid_parts) == 1:
        return merge_coverage_geometry(GeometryCollection(), valid_parts[0])
    try:
        return merge_coverage_geometry(GeometryCollection(), unary_union(valid_parts))
    except Exception:
        merged: BaseGeometry = GeometryCollection()
        for geometry in valid_parts:
            merged = merge_coverage_geometry(merged, geometry)
        return merged


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


def _top_level_area_replan_geometry(
    remaining_geometry: BaseGeometry,
    assignment_geometry: BaseGeometry,
    *,
    area_threshold_m2: float,
    bridge_gap_m: float,
) -> BaseGeometry:
    polygons = _iter_polygons(remaining_geometry)
    try:
        merged = unary_union(polygons) if polygons else remaining_geometry
    except Exception:
        merged = remaining_geometry
    if merged is None or merged.is_empty:
        return GeometryCollection()
    try:
        if assignment_geometry is not None and not assignment_geometry.is_empty:
            merged = assignment_geometry.intersection(merged)
    except Exception:
        pass
    merged = _filter_small_polygons(merged, area_threshold_m2=float(area_threshold_m2))
    if merged is None or merged.is_empty:
        return GeometryCollection()

    base_area_m2 = max(0.0, float(merged.area or 0.0))
    base_count = max(1, len(_iter_polygons(merged)))
    gap_m = max(0.0, float(bridge_gap_m or 0.0))
    if gap_m > 0.0:
        try:
            closed = merged.buffer(gap_m, join_style=2).buffer(-gap_m, join_style=2)
            if assignment_geometry is not None and not assignment_geometry.is_empty:
                closed = assignment_geometry.intersection(closed)
            closed = _filter_small_polygons(closed, area_threshold_m2=float(area_threshold_m2))
            closed_area_m2 = max(0.0, float(closed.area or 0.0)) if closed is not None else 0.0
            max_allowed_area_m2 = max(
                float(base_area_m2) * 1.06,
                float(base_area_m2) + max(float(area_threshold_m2) * 6.0, 20_000.0),
            )
            if (
                closed is not None
                and not closed.is_empty
                and float(closed_area_m2) <= float(max_allowed_area_m2)
                and len(_iter_polygons(closed)) <= int(base_count)
            ):
                return closed
        except Exception:
            pass

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
        if str(state.mission_type) == "line":
            boundary_index = max(restored_indexes)
            restored_indexes.update(range(0, int(boundary_index) + 1))
        elif str(state.mission_type) == "area":
            contiguous_boundary = -1
            for idx in range(0, len(state.planned_cut_lines)):
                if int(idx) not in restored_indexes:
                    break
                contiguous_boundary = int(idx)
            if contiguous_boundary < 0:
                return
            restored_indexes = set(range(0, int(contiguous_boundary) + 1))
            restored_lines = [
                line
                for idx, line in enumerate(state.planned_cut_lines)
                if int(idx) in restored_indexes and not line.is_empty
            ]
        state.completed_cut_line_indexes = restored_indexes
        state.cut_lines = restored_lines[-800:]
        state.last_cut_line_index = max(restored_indexes)
        state.progress_origin_line_index = 0 if str(state.mission_type) in {"area", "line"} else min(restored_indexes)
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
    if state.progress_direction_sign is None and str(state.mission_type) == "area":
        nearby_candidates = [
            frontier_index,
            frontier_index + 1,
            frontier_index - 1,
            frontier_index + 2,
            frontier_index - 2,
        ]
    else:
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
    if boundary_index is None:
        return int(tracked_index)
    boundary_index = max(0, min(int(boundary_index), max_index))
    if state.progress_direction_sign is None and str(state.mission_type) == "area":
        if int(tracked_index) == int(boundary_index):
            return int(boundary_index)
        if abs(int(tracked_index) - int(boundary_index)) <= 2:
            return int(tracked_index)
        return int(boundary_index)
    direction_sign = 1 if state.progress_direction_sign is None else (1 if int(state.progress_direction_sign) >= 0 else -1)
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
    completed_indexes = set(state.completed_cut_line_indexes)
    if state.progress_origin_line_index is None:
        if str(state.mission_type) == "line":
            state.progress_origin_line_index = 0
            state.progress_boundary_line_index = int(commit_index)
            completed_indexes.update(range(0, int(commit_index) + 1))
        else:
            state.progress_origin_line_index = int(commit_index)
            state.progress_boundary_line_index = int(commit_index)
        state.progress_boundary_line_index = int(commit_index)
        state.progress_direction_sign = 1 if str(state.mission_type) == "line" else None
        changed = True
    else:
        origin_index = int(state.progress_origin_line_index)
        current_boundary_index = int(
            state.progress_boundary_line_index
            if state.progress_boundary_line_index is not None
            else origin_index
        )
        direction_sign = state.progress_direction_sign
        if direction_sign is None:
            if int(commit_index) != int(origin_index):
                direction_sign = 1 if int(commit_index) > int(origin_index) else -1
                state.progress_direction_sign = int(direction_sign)
                contiguous_boundary = _contiguous_boundary_index(
                    origin_index=int(origin_index),
                    boundary_index=int(current_boundary_index),
                    direction_sign=int(direction_sign),
                    completed_indexes=set(state.completed_cut_line_indexes) | {int(commit_index)},
                )
                if contiguous_boundary != current_boundary_index:
                    state.progress_boundary_line_index = int(contiguous_boundary)
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


def _area_remaining_hole_threshold_m2(state: _MissionAreaState) -> float:
    width_hint_m = _as_float(state.width_hint_m) or _DEFAULT_STRIP_WIDTH_M
    planned_area_m2 = max(0.0, float(state.planned_area_m2 or 0.0))
    return max(
        _coverage_sliver_threshold_m2(state),
        min(planned_area_m2 * 0.006, 80_000.0),
        float(state.cut_half_width_m) * max(float(width_hint_m), _DEFAULT_STRIP_WIDTH_M) * 2.0,
    )


def _area_remaining_bridge_gap_m(states: list[_MissionAreaState]) -> float:
    if not states:
        return 0.0
    max_half_width_m = max(float(state.cut_half_width_m or 0.0) for state in states)
    max_width_hint_m = max(float(state.width_hint_m or 0.0) for state in states)
    return max(
        4.0,
        min(max(max_half_width_m * 0.85, max_width_hint_m * 0.16), 35.0),
    )


def _area_remaining_simplify_tolerance_m(states: list[_MissionAreaState]) -> float:
    if not states:
        return 0.0
    max_half_width_m = max(float(state.cut_half_width_m or 0.0) for state in states)
    return max(0.8, min(max_half_width_m * 0.35, 8.0))


def _coarsen_area_remaining_geometry(
    geometry: BaseGeometry,
    assignment_geometry: BaseGeometry,
    *,
    area_threshold_m2: float,
    hole_threshold_m2: float,
    simplify_tolerance_m: float,
) -> BaseGeometry:
    result = geometry if geometry is not None else GeometryCollection()
    if result.is_empty:
        return GeometryCollection()
    result = _filter_small_polygons(result, area_threshold_m2=float(area_threshold_m2))
    result = _filter_small_polygon_holes(result, area_threshold_m2=float(hole_threshold_m2))
    if result is None or result.is_empty:
        return GeometryCollection()
    tolerance_m = max(0.0, float(simplify_tolerance_m or 0.0))
    if tolerance_m > 0.0:
        try:
            simplified = result.simplify(tolerance_m, preserve_topology=True)
            if assignment_geometry is not None and not assignment_geometry.is_empty:
                simplified = assignment_geometry.intersection(simplified)
            simplified = _filter_small_polygons(
                simplified,
                area_threshold_m2=float(area_threshold_m2),
            )
            if simplified is not None and not simplified.is_empty:
                result = simplified
        except Exception:
            pass
    return result if result is not None and not result.is_empty else GeometryCollection()


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
    if str(state.mission_type) == "area":
        remaining = geometry if geometry is not None else GeometryCollection()
        if remaining.is_empty:
            return GeometryCollection()
        boundary_index = state.progress_boundary_line_index
        if boundary_index is None and state.completed_cut_line_indexes:
            boundary_index = max(int(index) for index in state.completed_cut_line_indexes)
        if boundary_index is None:
            return remaining
        side_remaining = _remaining_side_geometry(state, int(boundary_index))
        if side_remaining is None or side_remaining.is_empty:
            return GeometryCollection() if int(boundary_index) >= len(state.planned_cut_lines) - 1 else remaining
        try:
            clipped = remaining.intersection(side_remaining)
        except Exception:
            return remaining
        clipped = _filter_small_polygons(
            clipped,
            area_threshold_m2=_coverage_sliver_threshold_m2(state),
        )
        return clipped if clipped is not None and not clipped.is_empty else GeometryCollection()

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
            area_threshold_m2=_area_remaining_hole_threshold_m2(state),
        )
    remaining = _drop_completed_backtrack_islands(state, remaining)
    if preserve_holes:
        remaining = _coarsen_area_remaining_geometry(
            remaining,
            state.assignment_geometry,
            area_threshold_m2=_coverage_sliver_threshold_m2(state),
            hole_threshold_m2=_area_remaining_hole_threshold_m2(state),
            simplify_tolerance_m=_area_remaining_simplify_tolerance_m([state]),
        )
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


def _area_group_can_snapshot_done(states: list[_MissionAreaState]) -> bool:
    area_states = [state for state in states if str(state.mission_type) == "area"]
    if not area_states:
        return False
    for state in area_states:
        if bool(state.done):
            continue
        if not _is_state_geometrically_done(state):
            return False
    return True


def _area_passed_keepback_ratio() -> float:
    try:
        value = float(os.environ.get(_AREA_PASSED_KEEPBACK_RATIO_ENV, _AREA_PASSED_KEEPBACK_RATIO_DEFAULT))
    except Exception:
        value = float(_AREA_PASSED_KEEPBACK_RATIO_DEFAULT)
    if not math.isfinite(float(value)):
        value = float(_AREA_PASSED_KEEPBACK_RATIO_DEFAULT)
    return max(0.0, min(float(value), 0.45))


def _planned_line_spacing_hint_m(state: _MissionAreaState, boundary_index: int) -> float:
    samples: list[float] = []
    if boundary_index > 0 and boundary_index < len(state.planned_cut_lines):
        try:
            gap_prev = float(
                state.planned_cut_lines[int(boundary_index)].distance(
                    state.planned_cut_lines[int(boundary_index) - 1]
                )
            )
        except Exception:
            gap_prev = 0.0
        if gap_prev > 0.5:
            samples.append(gap_prev)
    if boundary_index + 1 < len(state.planned_cut_lines):
        try:
            gap_next = float(
                state.planned_cut_lines[int(boundary_index)].distance(
                    state.planned_cut_lines[int(boundary_index) + 1]
                )
            )
        except Exception:
            gap_next = 0.0
        if gap_next > 0.5:
            samples.append(gap_next)
    if samples:
        samples.sort()
        return float(samples[len(samples) // 2])
    width_hint_m = _as_float(state.width_hint_m) or _DEFAULT_STRIP_WIDTH_M
    return max(float(state.cut_half_width_m) * 2.0, float(width_hint_m), _DEFAULT_STRIP_WIDTH_M)


def _area_passed_keepback_m(state: _MissionAreaState, boundary_index: int) -> float:
    ratio = _area_passed_keepback_ratio()
    if ratio <= 0.0:
        return 0.0
    spacing_m = _planned_line_spacing_hint_m(state, int(boundary_index))
    return max(0.0, float(spacing_m) * float(ratio))


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
    progress_x = float(progress_vec[0]) * float(direction_sign)
    progress_y = float(progress_vec[1]) * float(direction_sign)
    keepback_m = _area_passed_keepback_m(state, int(boundary_index)) if str(state.mission_type) == "area" else 0.0
    shift_x = float(progress_x) * float(extent_m)
    shift_y = float(progress_y) * float(extent_m)
    base_start = (
        float(coords[0][0]) - float(progress_x) * float(keepback_m),
        float(coords[0][1]) - float(progress_y) * float(keepback_m),
    )
    base_end = (
        float(coords[-1][0]) - float(progress_x) * float(keepback_m),
        float(coords[-1][1]) - float(progress_y) * float(keepback_m),
    )
    ahead_shell = [
        base_start,
        base_end,
        (base_end[0] + shift_x, base_end[1] + shift_y),
        (base_start[0] + shift_x, base_start[1] + shift_y),
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
    area_side_clip_allowed = mission_type == "area"
    frontier_index = _progress_frontier_index(
        state,
        include_tracked=bool(line_provisional_allowed),
    )
    if not state.completed_cut_line_indexes:
        if mission_type == "area" and area_side_clip_allowed and frontier_index is not None:
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
    if mission_type == "area" and area_side_clip_allowed and frontier_index is not None:
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
        signature = (
            True,
            id(state.assignment_geometry),
            len(state.planned_cut_lines),
            round(float(state.cut_half_width_m), 3),
        )
        if state.coverage_rebuild_signature == signature:
            return
        state.covered_geometry = state.assignment_geometry
        state.cut_lines = list(state.planned_cut_lines[-800:])
        if state.planned_cut_lines:
            state.completed_cut_line_indexes = set(range(len(state.planned_cut_lines)))
            state.last_cut_line_index = len(state.planned_cut_lines) - 1
        state.coverage_rebuild_signature = signature
        return
    if (
        str(state.mission_type) != "area"
        and state.progress_origin_line_index is not None
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
    state.completed_cut_line_indexes = set(int(idx) for idx in valid_indexes)
    signature = (
        False,
        id(state.assignment_geometry),
        len(state.planned_cut_lines),
        tuple(int(idx) for idx in valid_indexes),
        state.progress_origin_line_index,
        state.progress_boundary_line_index,
        state.progress_direction_sign,
        round(float(state.cut_half_width_m), 3),
    )
    if state.coverage_rebuild_signature == signature:
        return
    if not valid_indexes:
        state.covered_geometry = GeometryCollection()
        state.cut_lines = []
        state.last_cut_line_index = -1
        state.coverage_rebuild_signature = signature
        return
    if len(valid_indexes) >= len(state.planned_cut_lines):
        state.covered_geometry = state.assignment_geometry
        state.cut_lines = list(state.planned_cut_lines[-800:])
        state.last_cut_line_index = len(state.planned_cut_lines) - 1
        state.coverage_rebuild_signature = signature
        return
    strip_cache: dict[int, BaseGeometry] = {}
    coverage_parts: list[BaseGeometry] = []
    for idx in valid_indexes:
        planned_line = state.planned_cut_lines[int(idx)]
        if not planned_line.is_empty:
            rendered_cut_lines.append(planned_line)
        strip = _planned_cut_strip(state, planned_line)
        if strip.is_empty:
            continue
        strip_cache[int(idx)] = strip
        coverage_parts.append(strip)
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
        coverage_parts.append(bridge)
    recomputed_covered = _merge_completed_coverage_parts(coverage_parts)
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
    state.coverage_rebuild_signature = signature


def _apply_sweep_point_progress_to_line_state(
    state: _MissionAreaState,
    progress: dict[str, Any] | None,
) -> bool:
    if (
        str(state.mission_type) != "line"
        or state.done
        or not state.planned_cut_lines
        or not isinstance(progress, dict)
    ):
        return False
    sweep_point_count = _as_int(progress.get("sweep_point_count"))
    sweep_progress_points = _as_int(progress.get("sweep_progress_points"))
    if (
        sweep_point_count is None
        or sweep_point_count <= 0
        or sweep_progress_points is None
        or sweep_progress_points <= 0
    ):
        return False

    line_count = len(state.planned_cut_lines)
    if bool(progress.get("done")) or int(sweep_progress_points) >= int(sweep_point_count):
        target_boundary = line_count - 1
    else:
        progress_ratio = max(
            0.0,
            min(1.0, float(sweep_progress_points) / max(1.0, float(sweep_point_count))),
        )
        completed_count = int(math.floor(progress_ratio * float(line_count)))
        target_boundary = int(completed_count) - 1
    if target_boundary < 0:
        return False

    current_boundary = (
        int(state.progress_boundary_line_index)
        if state.progress_boundary_line_index is not None
        else -1
    )
    target_boundary = max(0, min(int(target_boundary), line_count - 1))
    if target_boundary <= current_boundary:
        return False

    state.progress_origin_line_index = 0
    state.progress_boundary_line_index = int(target_boundary)
    state.progress_direction_sign = 1
    state.completed_cut_line_indexes.update(range(0, int(target_boundary) + 1))
    state.last_cut_line_index = max(int(state.last_cut_line_index), int(target_boundary))
    return True


def _apply_sweep_point_progress_to_area_state(
    state: _MissionAreaState,
    progress: dict[str, Any] | None,
) -> bool:
    if (
        str(state.mission_type) != "area"
        or state.done
        or not state.planned_cut_lines
        or not isinstance(progress, dict)
    ):
        return False

    current_waypoint_id = _as_int(progress.get("current_waypoint_id"))
    if current_waypoint_id is None:
        return False
    if state.sweep_waypoint_ids and int(current_waypoint_id) not in state.sweep_waypoint_ids:
        return False
    sweep_point_count = _as_int(progress.get("sweep_point_count"))
    sweep_progress_points = _as_int(progress.get("sweep_progress_points"))
    if (
        sweep_point_count is None
        or sweep_point_count <= 0
        or sweep_progress_points is None
        or sweep_progress_points <= 0
    ):
        return False

    line_count = len(state.planned_cut_lines)
    if int(sweep_progress_points) >= int(sweep_point_count):
        target_boundary = line_count - 1
    else:
        progress_ratio = max(
            0.0,
            min(1.0, float(sweep_progress_points) / max(1.0, float(sweep_point_count))),
        )
        completed_count = int(math.floor(progress_ratio * float(line_count)))
        target_boundary = int(completed_count) - 1
    if target_boundary < 0:
        return False

    target_boundary = max(0, min(int(target_boundary), line_count - 1))
    current_boundary = (
        int(state.progress_boundary_line_index)
        if state.progress_boundary_line_index is not None
        else -1
    )
    if target_boundary <= current_boundary:
        return False

    state.area_progress_source = "0401_sweep_points"
    state.area_progress_sweep_points = int(sweep_progress_points)
    state.area_progress_sweep_point_count = int(sweep_point_count)
    state.area_progress_boundary_line_index = int(target_boundary)
    state.area_progress_current_waypoint_id = int(current_waypoint_id)
    state.area_progress_confidence = "current_waypoint_line_search_match"
    state.progress_origin_line_index = 0
    state.progress_boundary_line_index = int(target_boundary)
    state.progress_direction_sign = 1
    state.completed_cut_line_indexes.update(range(0, int(target_boundary) + 1))
    state.last_cut_line_index = max(int(state.last_cut_line_index), int(target_boundary))
    return True


def _apply_area_row_progress_to_state(
    state: _MissionAreaState,
    update: Any,
    *,
    current_waypoint_id: int | None,
) -> bool:
    if str(state.mission_type) != "area" or state.done or not state.planned_cut_lines:
        return False
    progress_waypoint_id = (
        int(current_waypoint_id)
        if current_waypoint_id is not None
        else (
            int(state.area_progress_current_waypoint_id)
            if state.area_progress_current_waypoint_id is not None
            else 0
        )
    )
    max_index = len(state.planned_cut_lines) - 1
    observed_index = _as_int(getattr(update, "row_index", None))
    if observed_index is not None:
        observed_index = max(0, min(int(observed_index), max_index))
        state.last_nearest_cut_line_index = int(observed_index)
        state.tracking_cut_line_index = None
        state.provisional_frontier_line_index = None

    frontier_index = _as_int(getattr(update, "frontier_index", None))
    completed_indexes = {
        max(0, min(int(idx), max_index))
        for idx in (getattr(update, "completed_indexes", None) or [])
        if _as_int(idx) is not None
    }
    if frontier_index is None and observed_index is not None:
        frontier_index = int(observed_index)
    if frontier_index is None or not completed_indexes:
        return False

    frontier_index = max(0, min(int(frontier_index), max_index))
    direction_sign = _as_int(getattr(update, "row_index_direction_sign", None))
    if direction_sign is None:
        return False
    direction_sign = 1 if int(direction_sign) >= 0 else -1
    current_boundary = (
        int(state.progress_boundary_line_index)
        if state.progress_boundary_line_index is not None
        else None
    )
    if current_boundary is not None:
        current_boundary = max(0, min(int(current_boundary), max_index))
        signed_advance = (int(frontier_index) - int(current_boundary)) * int(direction_sign)
        if signed_advance <= 0 and completed_indexes.issubset(state.completed_cut_line_indexes):
            return False

    next_completed_indexes = set(completed_indexes)
    completed_count = len(next_completed_indexes)
    state.area_progress_source = "0401_area_row_count"
    state.area_progress_sweep_points = int(completed_count)
    state.area_progress_sweep_point_count = len(state.planned_cut_lines)
    state.area_progress_boundary_line_index = int(frontier_index)
    state.area_progress_current_waypoint_id = int(progress_waypoint_id)
    state.area_progress_confidence = "recent_0401_row_consensus"

    state.progress_origin_line_index = 0 if int(direction_sign) >= 0 else max_index
    state.progress_boundary_line_index = int(frontier_index)
    state.progress_direction_sign = int(direction_sign)
    state.completed_cut_line_indexes = next_completed_indexes
    state.last_cut_line_index = int(frontier_index)
    state.provisional_frontier_line_index = None
    _reset_area_footprint_candidate(state)
    return True


def _reset_area_footprint_candidate(state: _MissionAreaState) -> None:
    state.area_footprint_candidate_line_index = None
    state.area_footprint_candidate_seen_count = 0
    state.area_footprint_candidate_first_ms = None
    state.area_footprint_candidate_last_ms = None


def _area_footprint_candidate_window(state: _MissionAreaState) -> tuple[int, int] | None:
    if str(state.mission_type) != "area" or not state.planned_cut_lines:
        return None
    max_index = len(state.planned_cut_lines) - 1
    current_boundary = (
        max(0, min(int(state.progress_boundary_line_index), max_index))
        if state.progress_boundary_line_index is not None
        else None
    )
    if current_boundary is not None:
        if int(current_boundary) >= max_index:
            return None
        lower = min(max_index, int(current_boundary) + 1)
        upper = min(max_index, int(current_boundary) + 2)
        return int(lower), int(max(lower, upper))

    return 0, min(max_index, 1)


def _area_footprint_candidate_confirmed(
    state: _MissionAreaState,
    *,
    observed_index: int,
    current_boundary: int,
    timestamp_ms: int | None,
) -> bool:
    direct_limit = max(0, int(_AREA_FOOTPRINT_CONFIRM_DIRECT_ADVANCE_ROWS))
    if int(observed_index) - int(current_boundary) <= int(direct_limit):
        _reset_area_footprint_candidate(state)
        return True

    sample_ts = int(timestamp_ms) if timestamp_ms is not None else 0
    if state.area_footprint_candidate_line_index != int(observed_index):
        state.area_footprint_candidate_line_index = int(observed_index)
        state.area_footprint_candidate_seen_count = 1
        state.area_footprint_candidate_first_ms = int(sample_ts)
        state.area_footprint_candidate_last_ms = int(sample_ts)
        return False

    state.area_footprint_candidate_seen_count = int(state.area_footprint_candidate_seen_count or 0) + 1
    state.area_footprint_candidate_last_ms = int(sample_ts)
    first_ms = (
        int(state.area_footprint_candidate_first_ms)
        if state.area_footprint_candidate_first_ms is not None
        else int(sample_ts)
    )
    if (
        int(state.area_footprint_candidate_seen_count) < int(_AREA_FOOTPRINT_CONFIRM_SAMPLE_COUNT)
        or int(sample_ts) - int(first_ms) < int(_AREA_FOOTPRINT_CONFIRM_MIN_INTERVAL_MS)
    ):
        return False
    _reset_area_footprint_candidate(state)
    return True


def _area_footprint_supported_line_index(
    state: _MissionAreaState,
    *,
    footprint_geometry: BaseGeometry | None,
    width_m: float | None,
    min_line_index: int | None = None,
    max_line_index: int | None = None,
) -> int | None:
    if str(state.mission_type) != "area" or not state.planned_cut_lines:
        return None
    if footprint_geometry is None or footprint_geometry.is_empty:
        return None
    max_index = len(state.planned_cut_lines) - 1
    scan_start = 0 if min_line_index is None else max(0, min(int(min_line_index), max_index))
    scan_end = max_index if max_line_index is None else max(0, min(int(max_line_index), max_index))
    if int(scan_start) > int(scan_end):
        return None
    best_index: int | None = None
    best_score: tuple[float, float, float, int] | None = None
    for idx in range(int(scan_start), int(scan_end) + 1):
        (
            overlap_area_m2,
            strip_area_m2,
            footprint_area_m2,
            strip_overlap_ratio,
            footprint_overlap_ratio,
            threshold_m2,
        ) = _planned_line_overlap_metrics(
            state,
            line_index=int(idx),
            footprint_geometry=footprint_geometry,
            width_m=width_m,
        )
        if overlap_area_m2 <= 1e-6 or footprint_area_m2 <= 1e-6:
            continue
        area_floor_m2 = max(float(threshold_m2) * 0.85, 12.0)
        strip_floor_ratio = 0.055
        footprint_floor_ratio = 0.035
        if strip_area_m2 > 1e-6:
            area_floor_m2 = max(area_floor_m2, min(float(strip_area_m2) * 0.045, 160.0))
        supported = (
            overlap_area_m2 >= area_floor_m2
            or (
                strip_overlap_ratio >= strip_floor_ratio
                and footprint_overlap_ratio >= footprint_floor_ratio
                and overlap_area_m2 >= max(float(threshold_m2) * 0.55, 8.0)
            )
        )
        if not supported:
            continue
        score = (
            float(strip_overlap_ratio),
            float(footprint_overlap_ratio),
            float(overlap_area_m2),
            int(idx),
        )
        if best_score is None or score > best_score:
            best_score = score
            best_index = int(idx)
    return best_index


def _apply_area_footprint_row_progress_to_state(
    state: _MissionAreaState,
    *,
    footprint_geometry: BaseGeometry | None,
    width_m: float | None,
    current_waypoint_id: int | None,
    timestamp_ms: int | None,
) -> bool:
    if str(state.mission_type) != "area" or state.done or not state.planned_cut_lines:
        return False
    candidate_window = _area_footprint_candidate_window(state)
    if candidate_window is None:
        _reset_area_footprint_candidate(state)
        return False
    min_line_index, max_line_index = candidate_window
    observed_index = _area_footprint_supported_line_index(
        state,
        footprint_geometry=footprint_geometry,
        width_m=width_m,
        min_line_index=int(min_line_index),
        max_line_index=int(max_line_index),
    )
    if observed_index is None:
        _reset_area_footprint_candidate(state)
        return False
    max_index = len(state.planned_cut_lines) - 1
    current_boundary = (
        int(state.progress_boundary_line_index)
        if state.progress_boundary_line_index is not None
        else -1
    )
    observed_index = max(0, min(int(observed_index), max_index))
    if observed_index <= current_boundary:
        state.last_nearest_cut_line_index = int(observed_index)
        _reset_area_footprint_candidate(state)
        return False
    if not _area_footprint_candidate_confirmed(
        state,
        observed_index=int(observed_index),
        current_boundary=int(current_boundary),
        timestamp_ms=timestamp_ms,
    ):
        state.last_nearest_cut_line_index = int(observed_index)
        return False
    completed_indexes = tuple(range(0, int(observed_index) + 1))
    update = AreaRowProgressUpdate(
        row_index=int(observed_index),
        chainage_m=None,
        distance_m=None,
        frontier_index=int(observed_index),
        row_index_direction_sign=1,
        completed_indexes=completed_indexes,
        changed=True,
    )
    changed = _apply_area_row_progress_to_state(
        state,
        update,
        current_waypoint_id=current_waypoint_id,
    )
    if changed:
        state.area_progress_source = "0401_area_footprint_row_count"
        state.area_progress_confidence = "footprint_planned_row_overlap"
    return bool(changed)


def _area_progress_details_for_states(
    states: list[_MissionAreaState],
    *,
    source_plan_id_fallback: int | None = None,
) -> list[dict[str, Any]]:
    details: list[dict[str, Any]] = []
    for state in states:
        if str(state.mission_type) != "area":
            continue
        progress_source = state.area_progress_source
        boundary_index = state.area_progress_boundary_line_index
        sweep_points = state.area_progress_sweep_points
        sweep_point_count = state.area_progress_sweep_point_count
        confidence = state.area_progress_confidence
        if not progress_source:
            completed_indexes = [
                int(idx)
                for idx in state.completed_cut_line_indexes
                if 0 <= int(idx) < len(state.planned_cut_lines)
            ]
            fallback_boundary = (
                int(state.progress_boundary_line_index)
                if state.progress_boundary_line_index is not None
                else (max(completed_indexes) if completed_indexes else None)
            )
            if fallback_boundary is None:
                continue
            progress_source = "planned_line_footprint"
            boundary_index = int(fallback_boundary)
            sweep_points = int(fallback_boundary) + 1
            sweep_point_count = len(state.planned_cut_lines)
            confidence = "planned_line_footprint"
        detail = {
            "aircraftID": int(state.aircraft_id),
            "individualMissionID": int(state.mission_id),
            "sourceMissionPlanID": state.source_plan_id or source_plan_id_fallback,
            "pathID": state.path_id,
            "progressSource": str(progress_source),
            "areaProgressSource": str(progress_source),
            "currentWaypointID": (
                int(state.area_progress_current_waypoint_id)
                if state.area_progress_current_waypoint_id is not None
                else 0
            ),
            "sweepProgressPoints": int(sweep_points or 0),
            "sweepPointCount": int(sweep_point_count or len(state.planned_cut_lines)),
            "mappedBoundaryLineIndex": int(boundary_index),
            "progressDirectionSign": (
                1
                if state.progress_direction_sign is None or int(state.progress_direction_sign) >= 0
                else -1
            ),
            "plannedLineCount": len(state.planned_cut_lines),
            "completedLineCount": len(
                [
                    int(idx)
                    for idx in state.completed_cut_line_indexes
                    if 0 <= int(idx) < len(state.planned_cut_lines)
                ]
            ),
            "completedLineIndexes": [
                int(idx)
                for idx in sorted(state.completed_cut_line_indexes)
                if 0 <= int(idx) < len(state.planned_cut_lines)
            ],
            "confidence": str(confidence or "planned_line_footprint"),
        }
        details.append(detail)
    details.sort(
        key=lambda item: (
            int(item.get("aircraftID") or 0),
            int(item.get("individualMissionID") or 0),
        )
    )
    return details


def _area_ownership_details_for_states(
    states: list[_MissionAreaState],
    *,
    source_plan_id_fallback: int | None = None,
) -> list[dict[str, Any]]:
    details: list[dict[str, Any]] = []
    for state in states:
        if str(state.mission_type) != "area":
            continue
        completed_count = len(
            [
                int(idx)
                for idx in state.completed_cut_line_indexes
                if 0 <= int(idx) < len(state.planned_cut_lines)
            ]
        )
        details.append(
            {
                "aircraftID": int(state.aircraft_id),
                "individualMissionID": int(state.mission_id),
                "inputMissionID": int(state.input_id) if state.input_id is not None else None,
                "sourceMissionPlanID": state.source_plan_id or source_plan_id_fallback,
                "pathID": state.path_id,
                "plannedAreaM2": float(max(0.0, state.planned_area_m2 or 0.0)),
                "plannedLineCount": len(state.planned_cut_lines),
                "completedLineCount": int(completed_count),
                "progressBoundaryLineIndex": state.progress_boundary_line_index,
                "progressDirectionSign": (
                    1
                    if state.progress_direction_sign is None or int(state.progress_direction_sign) >= 0
                    else -1
                ),
                "isCurrent": bool(state.is_current),
                "isDone": bool(state.done),
            }
        )
    details.sort(
        key=lambda item: (
            int(item.get("aircraftID") or 0),
            int(item.get("individualMissionID") or 0),
        )
    )
    return details


def _remaining_geometry_diagnostics(
    *,
    mission_type: str,
    remaining_detail: dict[str, Any],
    area_progress_details: list[dict[str, Any]],
    area_ownership_details: list[dict[str, Any]],
) -> dict[str, Any]:
    line_list = list((remaining_detail or {}).get("lineList") or [])
    area_list = list((remaining_detail or {}).get("areaList") or [])
    area_segment_list = list((remaining_detail or {}).get("areaSegmentList") or [])
    coordinate_list = list((remaining_detail or {}).get("coordinateList") or [])
    outer_area_count = len(
        [
            item
            for item in area_list
            if isinstance(item, dict) and not bool(item.get("isHole"))
        ]
    )
    hole_area_count = len(
        [
            item
            for item in area_list
            if isinstance(item, dict) and bool(item.get("isHole"))
        ]
    )
    replan_input_geometry = "none"
    if str(mission_type) == "area":
        if area_segment_list:
            replan_input_geometry = "area_segment_list"
        elif outer_area_count == 1 and hole_area_count == 0:
            replan_input_geometry = "single_area_polygon"
        elif outer_area_count > 1 or hole_area_count > 0:
            replan_input_geometry = "area_component_decomposition_multi_polygon_or_hole"
        elif len(coordinate_list) >= 3:
            replan_input_geometry = "coordinate_polygon"
    elif str(mission_type) == "line":
        if line_list:
            replan_input_geometry = "line_list"
        elif len(coordinate_list) >= 2:
            replan_input_geometry = "coordinate_line"
    operator_decisions: list[dict[str, Any]] = []
    if str(mission_type) == "area":
        if area_progress_details:
            strongest_progress = max(
                area_progress_details,
                key=lambda item: (
                    int(item.get("completedLineCount") or 0),
                    int(item.get("sweepProgressPoints") or 0),
                ),
            )
            operator_decisions.append(
                {
                    "category": "monotonic_progress_trim",
                    "source": strongest_progress.get("areaProgressSource") or strongest_progress.get("progressSource"),
                    "mappedBoundaryLineIndex": strongest_progress.get("mappedBoundaryLineIndex"),
                    "currentWaypointID": strongest_progress.get("currentWaypointID"),
                }
            )
        if area_ownership_details:
            operator_decisions.append(
                {
                    "category": "preserved_assignment",
                    "policy": "piece_only_takeover",
                    "ownerCount": len(area_ownership_details),
                }
            )
        if replan_input_geometry == "area_segment_list":
            operator_decisions.append(
                {
                    "category": "planner_redivision",
                    "reason": "planned_sweep_row_segments_available",
                    "segmentCount": len(area_segment_list),
                }
            )
        elif replan_input_geometry == "area_component_decomposition_multi_polygon_or_hole":
            operator_decisions.append(
                {
                    "category": "planner_redivision",
                    "reason": "component_decomposition_multi_polygon_or_hole",
                }
            )
        elif replan_input_geometry in {"single_area_polygon", "coordinate_polygon"}:
            operator_decisions.append(
                {
                    "category": "planner_redivision",
                    "reason": "single_polygon_remaining_area_available",
                }
            )
        elif replan_input_geometry == "none":
            operator_decisions.append(
                {
                    "category": "fail_closed_skip",
                    "reason": "remaining_area_geometry_unavailable",
                }
            )
    return {
        "displayCoverageSource": "planned_sweep_row_prefix",
        "snapshotRemainingGeometry": {
            "coordinatePointCount": len(coordinate_list),
            "lineBlockCount": len(line_list),
            "areaBlockCount": len(area_list),
            "areaOuterCount": int(outer_area_count),
            "areaHoleCount": int(hole_area_count),
            "areaSegmentCount": len(area_segment_list),
        },
        "replanInputGeometry": replan_input_geometry,
        "areaSegmentPolicy": str((remaining_detail or {}).get("areaSegmentPolicy") or ""),
        "areaProgressDetailCount": len(area_progress_details),
        "areaOwnershipDetailCount": len(area_ownership_details),
        "operatorDecisions": operator_decisions,
    }


def _attach_remaining_detail_to_area_ownership(
    area_ownership_details: list[dict[str, Any]],
    states: list[_MissionAreaState],
    remaining_detail_provider: Callable[[_MissionAreaState], tuple[dict[str, Any], float]],
) -> None:
    if not area_ownership_details:
        return
    state_by_mission_id = {
        int(state.mission_id): state
        for state in states
        if str(state.mission_type) == "area"
    }
    for ownership_detail in area_ownership_details:
        owner_mission_id = _as_int(ownership_detail.get("individualMissionID"))
        owner_state = state_by_mission_id.get(int(owner_mission_id)) if owner_mission_id is not None else None
        if owner_state is None:
            continue
        owner_remaining_detail, owner_remaining_area_m2 = remaining_detail_provider(owner_state)
        ownership_detail["remainingAreaM2"] = float(max(0.0, owner_remaining_area_m2 or 0.0))
        ownership_detail["remainingDetail"] = deepcopy(owner_remaining_detail)
        ownership_detail["takeoverPolicy"] = "piece_only"




class MissionProgressAreaSnapshotMonitor:
    """Headless area progress tracker for replan snapshots, independent from Qt/UI."""

    def __init__(self, *, snapshot_persist_interval_ms: int = 3000) -> None:
        self._ui_updates_enabled = False
        self._background_logic_enabled = True
        self._dirty = False
        self._selected_aircraft_id = int(_UAV_IDS[0])
        self._mission_view: dict[str, Any] | None = None
        self._progress_tracker = MissionProgressTracker()
        self._progress_snapshot: dict[str, Any] = {}
        self._snapshot_persist_interval_ms = max(0, int(snapshot_persist_interval_ms))
        self._last_snapshot_persist_monotonic = 0.0
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
        self._coverage_progress_last_signature: str | None = None
        self._selected_mission_id: int | None = None

    def set_ui_updates_enabled(self, enabled: bool) -> None:
        _ = enabled
        self._ui_updates_enabled = False

    def _request_refresh(self, *, force: bool = False) -> None:
        _ = force
        self._dirty = True
        if self._background_logic_enabled:
            self._persist_replan_snapshot()

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


    def update_0903(self, *, timestamp_ms: int | None, mission_plan_id: int | None, source: str | None = None) -> None:
        _ = timestamp_ms, source
        self._load_mission_plan(mission_plan_id)


    def apply_mission_plan_decision(self, *, mission_plan_id: int | None) -> None:
        self._load_mission_plan(mission_plan_id)


    def update_agent_status(self, *, timestamp_ms: int | None, agent_states: list[dict[str, Any]], fuel_state_map: dict[int, str] | None = None) -> None:
        _ = fuel_state_map
        if not self._mission_view:
            return
        if not self._ui_updates_enabled and not self._background_logic_enabled:
            self._dirty = True
            return
        self._progress_snapshot = self._progress_tracker.update(timestamp_ms, agent_states or [])
        self._coverage_progress_last_signature = persist_coverage_progress(
            mission_view=self._mission_view,
            snapshot=self._progress_snapshot,
            timestamp_ms=timestamp_ms,
            previous_signature=self._coverage_progress_last_signature,
        )
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
        for mid, state in self._states.items():
            progress_entry = mission_progress.get(mid) or mission_progress.get(str(mid))
            if _apply_sweep_point_progress_to_line_state(state, progress_entry):
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
            current_waypoint_id = _as_int(item.get("current_waypoint_id"))
            filming_value = _as_int(item.get("filming"))
            footprint_width_m = _footprint_width_m(item.get("footprint_corners")) or state.width_hint_m or _DEFAULT_STRIP_WIDTH_M
            footprint_geometry = build_footprint_geometry(
                item.get("footprint_corners"),
                state.coverage_def.transformer,
            )
            area_capture_active = _area_capture_progress_allowed(
                state,
                current_waypoint_id=current_waypoint_id,
                filming_value=filming_value,
            )
            if (
                str(state.mission_type) == "area"
                and not area_capture_active
                and filming_value is not None
                and int(filming_value) > 0
                and footprint_geometry is not None
                and not footprint_geometry.is_empty
                and _footprint_observes_assignment(
                    state,
                    footprint_geometry,
                    width_m=footprint_width_m,
                )
            ):
                area_capture_active = True
            if (
                str(state.mission_type) == "area"
                and current_waypoint_id is not None
                and area_capture_active
            ):
                state.area_progress_current_waypoint_id = int(current_waypoint_id)
            area_row_changed = False
            if (
                str(state.mission_type) == "area"
                and state.planned_cut_lines
                and state.area_row_definition is not None
            ):
                should_sample_area_row = bool(area_capture_active)
                if not area_capture_active:
                    reset_area_row_live_state(
                        state.area_row_progress_state,
                        timestamp_ms=timestamp_ms,
                    )
                    _reset_area_footprint_candidate(state)
                elif (
                    timestamp_ms is not None
                    and state.area_row_progress_last_sample_ms is not None
                    and int(timestamp_ms) - int(state.area_row_progress_last_sample_ms) < int(_AREA_ROW_PROGRESS_MIN_INTERVAL_MS)
                ):
                    should_sample_area_row = False
                if should_sample_area_row:
                    row_update = update_area_row_progress_state(
                        state.area_row_definition,
                        state.area_row_progress_state,
                        center_xy,
                        timestamp_ms=timestamp_ms,
                        filming=item.get("filming"),
                    )
                    if timestamp_ms is not None:
                        state.area_row_progress_last_sample_ms = int(timestamp_ms)
                    if _apply_area_row_progress_to_state(
                        state,
                        row_update,
                        current_waypoint_id=current_waypoint_id,
                    ):
                        area_row_changed = True
                        states_requiring_rebuild.add(int(state.mission_id))
            if (
                str(state.mission_type) == "area"
                and state.planned_cut_lines
                and area_capture_active
            ):
                if _apply_area_footprint_row_progress_to_state(
                    state,
                    footprint_geometry=footprint_geometry,
                    width_m=footprint_width_m,
                    current_waypoint_id=current_waypoint_id,
                    timestamp_ms=timestamp_ms,
                ):
                    states_requiring_rebuild.add(int(state.mission_id))
            if str(state.mission_type) == "area" and state.planned_cut_lines:
                _reset_planned_line_tracking(state)
                state.last_center_xy = center_xy
                state.last_update_ms = int(timestamp_ms) if timestamp_ms is not None else state.last_update_ms
                if not state.centerline_points or Point(state.centerline_points[-1]).distance(Point(center_xy)) > 0.25:
                    state.centerline_points.append(center_xy)
                    if len(state.centerline_points) > 800:
                        state.centerline_points = state.centerline_points[-800:]
                continue
            if str(state.mission_type) == "area" and not area_capture_active:
                _reset_planned_line_tracking(state)
                state.last_nearest_cut_line_index = None
                state.last_center_xy = center_xy
                state.last_update_ms = int(timestamp_ms) if timestamp_ms is not None else state.last_update_ms
                if not state.centerline_points or Point(state.centerline_points[-1]).distance(Point(center_xy)) > 0.25:
                    state.centerline_points.append(center_xy)
                    if len(state.centerline_points) > 800:
                        state.centerline_points = state.centerline_points[-800:]
                continue
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
                        if len(state.centerline_points) > 800:
                            state.centerline_points = state.centerline_points[-800:]
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
                if len(state.centerline_points) > 800:
                    state.centerline_points = state.centerline_points[-800:]
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
                area_row_definition = (
                    build_area_row_progress_definition(
                        planned_cut_lines,
                        width_hint_m=_as_float(mission.get("width_m")),
                        cut_half_width_m=float(cut_half_width_m),
                    )
                    if mission_type == "area" and planned_cut_lines
                    else None
                )
                previous_state = previous_state_by_mission.get(mid) if preserve_same_plan else None
                if previous_state is not None and mission_type == "area":
                    if not _area_state_matches_new_baseline(
                        previous_state,
                        aircraft_id=int(aid),
                        input_id=_as_int(mission.get("input_id")),
                        path_id=_as_int(mission.get("path_id")),
                        assignment_geometry=coverage_def.assignment_geometry,
                        planned_cut_lines=planned_cut_lines,
                    ):
                        previous_state = None
                if previous_state is None and not preserve_same_plan and mission_type != "area":
                    exact_previous_state = previous_state_by_mission.get(mid)
                    if (
                        exact_previous_state is not None
                        and int(exact_previous_state.aircraft_id) == int(aid)
                        and str(exact_previous_state.mission_type) == str(mission_type)
                        and exact_previous_state.input_id == _as_int(mission.get("input_id"))
                        and exact_previous_state.path_id == _as_int(mission.get("path_id"))
                    ):
                        previous_state = exact_previous_state
                if previous_state is not None:
                    used_previous_mission_ids.add(int(previous_state.mission_id))
                elif not done and mission_type != "area":
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
                    mission_id=mid,
                    aircraft_id=aid,
                    input_id=_as_int(mission.get("input_id")),
                    mission_type=mission_type,
                    source_plan_id=_as_int((self._mission_view or {}).get("mission_plan_id")),
                    path_id=_as_int(mission.get("path_id")),
                    coverage_def=coverage_def,
                    width_hint_m=_as_float(mission.get("width_m")),
                    assignment_geometry=coverage_def.assignment_geometry,
                    planned_area_m2=float(coverage_def.planned_area_m2),
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
                    area_row_definition=area_row_definition,
                    sweep_waypoint_ids={
                        int(wp.get("waypoint_id"))
                        for wp in (mission.get("waypoints") or [])
                        if _as_int(wp.get("waypoint_id")) is not None
                        and int(_as_int(wp.get("line_search_point_count")) or 0) > 0
                    },
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
                    state.area_row_progress_state = clone_area_row_progress_state(
                        previous_state.area_row_progress_state
                    )
                    state.area_row_progress_last_sample_ms = previous_state.area_row_progress_last_sample_ms
                    state.area_footprint_candidate_line_index = previous_state.area_footprint_candidate_line_index
                    state.area_footprint_candidate_seen_count = int(previous_state.area_footprint_candidate_seen_count or 0)
                    state.area_footprint_candidate_first_ms = previous_state.area_footprint_candidate_first_ms
                    state.area_footprint_candidate_last_ms = previous_state.area_footprint_candidate_last_ms
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
        self._last_snapshot_persist_monotonic = 0.0
        self._request_refresh(force=True)


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
            _area_remaining_hole_threshold_m2(state),
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
            max(_area_remaining_hole_threshold_m2(state) for state in states),
        )
        bridge_gap_m = _area_remaining_bridge_gap_m(states)
        replan_geometry = _top_level_area_replan_geometry(
            remaining_geometry,
            assignment_geometry,
            area_threshold_m2=float(area_threshold_m2),
            bridge_gap_m=float(bridge_gap_m),
        )
        replan_geometry = _coarsen_area_remaining_geometry(
            replan_geometry,
            assignment_geometry,
            area_threshold_m2=float(area_threshold_m2),
            hole_threshold_m2=float(hole_threshold_m2),
            simplify_tolerance_m=_area_remaining_simplify_tolerance_m(states),
        )
        area_blocks = _area_blocks_from_geometry(
            replan_geometry,
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
            area_segment_list = list((remaining_detail or {}).get("areaSegmentList") or [])
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
            is_done = not line_list and not area_list and not area_segment_list and len(coordinate_list) < 2
            if is_done and str(entry.get("missionType") or "-") == "area":
                is_done = _area_group_can_snapshot_done(states)
            remaining_detail_payload = {
                "coordinateList": coordinate_list,
                "lineList": line_list,
                "areaList": area_list,
            }
            if area_segment_list:
                remaining_detail_payload["areaSegmentList"] = area_segment_list
                remaining_detail_payload["areaSegmentPolicy"] = str(
                    (remaining_detail or {}).get("areaSegmentPolicy") or "planned_sweep_row_remaining"
                )
            mission_snapshot = {
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
                "remainingDetail": remaining_detail_payload,
            }
            area_progress_details = _area_progress_details_for_states(
                states,
                source_plan_id_fallback=int(plan_id),
            )
            area_ownership_details = _area_ownership_details_for_states(
                states,
                source_plan_id_fallback=int(plan_id),
            )
            _attach_remaining_detail_to_area_ownership(
                area_ownership_details,
                states,
                self._get_cached_state_remaining_detail,
            )
            if area_progress_details:
                strongest_progress = max(
                    area_progress_details,
                    key=lambda item: (
                        int(item.get("completedLineCount") or 0),
                        int(item.get("sweepProgressPoints") or 0),
                    ),
                )
                mission_snapshot.update(
                    {
                        "progressSource": strongest_progress.get("progressSource"),
                        "areaProgressSource": strongest_progress.get("areaProgressSource"),
                        "sourceMissionPlanID": strongest_progress.get("sourceMissionPlanID"),
                        "pathID": strongest_progress.get("pathID"),
                        "currentWaypointID": strongest_progress.get("currentWaypointID"),
                        "sweepProgressPoints": strongest_progress.get("sweepProgressPoints"),
                        "sweepPointCount": strongest_progress.get("sweepPointCount"),
                        "mappedBoundaryLineIndex": strongest_progress.get("mappedBoundaryLineIndex"),
                        "confidence": strongest_progress.get("confidence"),
                        "areaProgressDetails": area_progress_details,
                    }
                )
            if area_ownership_details:
                mission_snapshot["areaOwnershipDetails"] = area_ownership_details
                mission_snapshot["areaOwnershipPolicy"] = "piece_only_takeover"
                mission_snapshot["geometryDiagnostics"] = _remaining_geometry_diagnostics(
                    mission_type=str(entry.get("missionType") or "-"),
                    remaining_detail=remaining_detail_payload,
                    area_progress_details=area_progress_details,
                    area_ownership_details=area_ownership_details,
                )
            missions.append(mission_snapshot)

        snapshot = {
            "missionPlanID": int(plan_id),
            "timestamp": int(self._last_timestamp_ms) if self._last_timestamp_ms is not None else None,
            "missionCount": len(missions),
            "missions": missions,
        }
        self._snapshot_cache_key = int(self._state_cache_token)
        self._snapshot_cache = snapshot
        return snapshot


    def _persist_replan_snapshot(self, *, force: bool = False) -> None:
        now = time.perf_counter()
        if not force:
            interval_sec = max(0.0, float(getattr(self, "_snapshot_persist_interval_ms", 3000)) / 1000.0)
            last = float(getattr(self, "_last_snapshot_persist_monotonic", 0.0) or 0.0)
            if last > 0.0 and now - last < interval_sec:
                return
        self._last_snapshot_persist_monotonic = float(now)
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

