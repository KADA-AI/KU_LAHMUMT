"""Coordinate conversion, geometry helpers, Qt color utilities, math helpers."""
from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Sequence, Tuple

from PyQt5.QtGui import QColor
from shapely.geometry import LineString, MultiPolygon, Polygon

from ._constants import _R, _ORIGIN_LAT, _ORIGIN_LON, _DEFAULT_ALT, UAV_COLORS


def local_xy_to_llh(
    east_m: float,
    north_m: float,
    lat0: float = _ORIGIN_LAT,
    lon0: float = _ORIGIN_LON,
) -> Tuple[float, float]:
    lat0_r = math.radians(lat0)
    lat = lat0 + math.degrees(north_m / _R)
    lon = lon0 + math.degrees(east_m / (_R * math.cos(lat0_r)))
    return lat, lon


def llh_to_local_xy(
    lat: float,
    lon: float,
    lat0: float = _ORIGIN_LAT,
    lon0: float = _ORIGIN_LON,
) -> Tuple[float, float]:
    lat0_r = math.radians(lat0)
    east_m = math.radians(lon - lon0) * _R * math.cos(lat0_r)
    north_m = math.radians(lat - lat0) * _R
    return east_m, north_m


def meters_to_coord(east_m: float, north_m: float, alt_m: float = _DEFAULT_ALT) -> Dict[str, float]:
    lat, lon = local_xy_to_llh(east_m, north_m)
    return {"latitude": float(lat), "longitude": float(lon), "altitude": float(alt_m)}


def coord_to_xy(coord: Dict[str, Any]) -> Optional[Tuple[float, float]]:
    if not isinstance(coord, dict):
        return None
    if "latitude" not in coord or "longitude" not in coord:
        return None
    return llh_to_local_xy(float(coord["latitude"]), float(coord["longitude"]))


def coords_to_xy(coords: Sequence[Dict[str, Any]]) -> List[Tuple[float, float]]:
    out: List[Tuple[float, float]] = []
    for coord in coords:
        xy = coord_to_xy(coord)
        if xy is not None:
            out.append(xy)
    return out


def _distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _dedupe_points(
    points_xy: Sequence[Tuple[float, float]],
    min_dist_m: float = 1.0,
) -> List[Tuple[float, float]]:
    out: List[Tuple[float, float]] = []
    for point in points_xy:
        xy = (float(point[0]), float(point[1]))
        if out and _distance(out[-1], xy) < min_dist_m:
            continue
        out.append(xy)
    return out


def normalize_area_points(points_xy: Sequence[Tuple[float, float]]) -> Tuple[List[Tuple[float, float]], bool]:
    cleaned = _dedupe_points(points_xy)
    if len(cleaned) >= 2 and _distance(cleaned[0], cleaned[-1]) < 1.0:
        cleaned = cleaned[:-1]
    poly = Polygon(cleaned)
    corrected = False
    if not poly.is_valid:
        poly = poly.buffer(0)
        corrected = True
    if poly.is_empty:
        raise ValueError("영역이 비어 있습니다. 다시 입력해 주세요.")
    if isinstance(poly, MultiPolygon):
        poly = max((g for g in poly.geoms if isinstance(g, Polygon) and not g.is_empty), key=lambda g: g.area)
        corrected = True
    if not isinstance(poly, Polygon):
        raise ValueError("유효한 영역을 만들 수 없습니다. 교차 없이 다시 입력해 주세요.")
    result = [(float(x), float(y)) for (x, y) in list(poly.exterior.coords)[:-1]]
    if len(result) < 3:
        raise ValueError("영역 점이 3개 미만으로 정리되었습니다. 다시 입력해 주세요.")
    return result, corrected


def corridor_polygon_xy(points_xy: Sequence[Tuple[float, float]], width_m: float) -> List[Tuple[float, float]]:
    if len(points_xy) < 2 or width_m <= 0.0:
        return []
    line = LineString(points_xy)
    poly = line.buffer(float(width_m) * 0.5, cap_style=2, join_style=2, mitre_limit=10.0)
    if poly.is_empty:
        return []
    if isinstance(poly, MultiPolygon):
        poly = max((g for g in poly.geoms if isinstance(g, Polygon) and not g.is_empty), key=lambda g: g.area)
    if not isinstance(poly, Polygon):
        return []
    return [(float(x), float(y)) for (x, y) in list(poly.exterior.coords)[:-1]]


def centroid_xy(points_xy: Sequence[Tuple[float, float]]) -> Optional[Tuple[float, float]]:
    if not points_xy:
        return None
    sx = sum(p[0] for p in points_xy)
    sy = sum(p[1] for p in points_xy)
    n = float(len(points_xy))
    return sx / n, sy / n


def _qcolor(hex_color: str, alpha: int = 255) -> QColor:
    color = QColor(hex_color)
    color.setAlpha(max(0, min(255, int(alpha))))
    return color


def _uav_color(aircraft_id: int, alpha: int = 255) -> QColor:
    return _qcolor(UAV_COLORS.get(int(aircraft_id), "#64748b"), alpha)


def _format_coord(point_xy: Tuple[float, float]) -> str:
    lat, lon = local_xy_to_llh(point_xy[0], point_xy[1])
    return f"E {point_xy[0]:7.1f}m  N {point_xy[1]:7.1f}m  |  {lat:.6f}, {lon:.6f}"


def _undirected_angle_deg(
    vertex_xy: Tuple[float, float],
    point1_xy: Tuple[float, float],
    point2_xy: Tuple[float, float],
) -> Optional[float]:
    v1x = float(point1_xy[0]) - float(vertex_xy[0])
    v1y = float(point1_xy[1]) - float(vertex_xy[1])
    v2x = float(point2_xy[0]) - float(vertex_xy[0])
    v2y = float(point2_xy[1]) - float(vertex_xy[1])
    norm1 = math.hypot(v1x, v1y)
    norm2 = math.hypot(v2x, v2y)
    if norm1 <= 1e-9 or norm2 <= 1e-9:
        return None
    cos_theta = ((v1x * v2x) + (v1y * v2y)) / (norm1 * norm2)
    cos_theta = max(-1.0, min(1.0, float(cos_theta)))
    return float(math.degrees(math.acos(abs(cos_theta))))


def _bearing_deg_from_xy(
    start_xy: Tuple[float, float],
    end_xy: Tuple[float, float],
) -> Optional[float]:
    dx = float(end_xy[0]) - float(start_xy[0])
    dy = float(end_xy[1]) - float(start_xy[1])
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return None
    return float((math.degrees(math.atan2(dx, dy)) + 360.0) % 360.0)
