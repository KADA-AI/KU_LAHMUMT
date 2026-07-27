# -*- coding: utf-8 -*-
from __future__ import annotations

"""RTV 기준 시나리오를 GUI 상태로 변형하는 자동 임무 생성기.

이 모듈은 파일을 저장하거나 Random_mission 전역 설정을 변경하지 않는다. 같은 seed와
참조 파일에는 같은 결과를 반환하므로 HTTP 요청 및 테스트에서 안전하게 재사용할 수 있다.
"""

import copy
import json
import math
import random
import secrets
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Sequence


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_REFERENCE_ROOT = PROJECT_ROOT / "Logs" / "RTV_mission"
EARTH_M_PER_DEG = 111_320.0
LAYOUT_VERSION = 6


@dataclass(frozen=True)
class ScenarioProfile:
    package_type: int
    label: str
    reference_filename: str
    primary_region_types: tuple[int, ...]
    target_type_pool: tuple[int, ...]
    prefer_hole: bool = False


SCENARIO_PROFILES: dict[int, ScenarioProfile] = {
    1: ScenarioProfile(
        1,
        "대기갑항공타격",
        "260709_대기갑항공타격.json",
        (6,),
        (1, 1, 1, 2, 3, 5),
    ),
    2: ScenarioProfile(
        2,
        "지상작전부대",
        "260709_지상작전부대.json",
        (7,),
        (1, 2, 2, 4, 6),
    ),
    3: ScenarioProfile(
        3,
        "공중강습작전",
        "260709_공중강습작전.json",
        (7,),
        (2, 5, 6, 6),
    ),
    4: ScenarioProfile(
        4,
        "항공지원작전",
        "260709_항공지원작전.json",
        (7, 10),
        (2, 4, 5, 6),
        prefer_hole=True,
    ),
    5: ScenarioProfile(
        5,
        "도시지역",
        "260709_도시지역.json",
        (11,),
        (1, 2, 6, 6, 6),
    ),
}

# docs/reference/임무타입별 구현 상태.txt의 2026-07-09 확정 0201 계약.
# 일부 RTV 참조 파일은 예전 InputMissionType/MissionType 값을 담고 있으므로 도형은 RTV를
# 사용하되 0201 타입과 순서는 아래 계약으로 정규화한다.
MISSION_TYPE_SEQUENCES: dict[int, tuple[tuple[int, int], ...]] = {
    1: ((1, 3), (1, 4), (2, 4), (2, 6), (1, 3), (1, 2)),
    2: ((1, 3), (1, 6), (5, 6), (1, 7), (3, 7), (1, 6), (1, 3), (1, 2)),
    3: ((1, 8), (1, 3), (1, 9), (4, 9), (1, 7), (3, 7), (1, 9), (1, 3), (1, 2)),
    4: ((1, 4), (2, 4), (1, 7), (3, 7), (1, 4), (1, 3), (1, 2)),
    5: ((1, 4), (2, 4), (1, 11), (6, 11), (1, 4), (1, 3), (1, 2)),
}

# Semantic mission-stage progress.  Values deliberately rise to the main
# objective and then return instead of inheriting the source RTV's centroids.
# The generator uses these as a fresh deployment skeleton in layout v6.
MISSION_STAGE_PHASES: dict[int, tuple[float, ...]] = {
    1: (0.04, 0.19, 0.35, 0.56, 0.78, 0.96),
    2: (0.03, 0.14, 0.27, 0.40, 0.52, 0.64, 0.80, 0.97),
    3: (0.03, 0.12, 0.22, 0.32, 0.43, 0.54, 0.65, 0.80, 0.97),
    4: (0.03, 0.18, 0.34, 0.52, 0.68, 0.83, 0.97),
    5: (0.03, 0.18, 0.34, 0.52, 0.68, 0.83, 0.97),
}

# Line mission index -> (start phase, end phase) on the semantic spine.
# Area missions occupy the shared phase between their incoming/outgoing lines.
ROUTE_LINE_PHASE_RANGES: dict[int, dict[int, tuple[float, float]]] = {
    1: {0: (0.02, 0.18), 1: (0.18, 0.35), 4: (0.72, 0.84), 5: (0.84, 0.98)},
    2: {
        0: (0.02, 0.14), 1: (0.14, 0.27), 3: (0.27, 0.52),
        5: (0.52, 0.68), 6: (0.68, 0.82), 7: (0.82, 0.98),
    },
    3: {
        0: (0.02, 0.12), 1: (0.12, 0.22), 2: (0.22, 0.32),
        4: (0.32, 0.54), 6: (0.54, 0.68), 7: (0.68, 0.82),
        8: (0.82, 0.98),
    },
    4: {0: (0.02, 0.18), 2: (0.18, 0.52), 4: (0.52, 0.68), 5: (0.68, 0.83), 6: (0.83, 0.98)},
    5: {0: (0.02, 0.18), 2: (0.18, 0.52), 4: (0.52, 0.68), 5: (0.68, 0.83), 6: (0.83, 0.98)},
}

# Normalized (lateral, forward) control paths.  These are deliberately not
# affine variants of one another, so fitBounds cannot make them look alike.
SKELETON_CONTROL_PATHS: dict[str, tuple[tuple[float, float], ...]] = {
    "dogleg": (
        (-0.35, 0.00), (-0.42, 0.55), (0.15, 0.55), (0.18, 0.96),
        (0.52, 0.78), (0.02, 0.40), (0.42, 0.00),
    ),
    "diamond": (
        (0.00, 0.00), (-0.55, 0.35), (-0.28, 0.82), (0.00, 1.00),
        (0.55, 0.62), (0.25, 0.22), (0.55, 0.00),
    ),
    "split_lobe": (
        (-0.35, 0.00), (-0.08, 0.24), (-0.55, 0.52), (-0.18, 0.92),
        (0.48, 0.82), (0.60, 0.48), (0.16, 0.24), (0.48, 0.00),
    ),
    "transverse_bar": (
        (-0.45, 0.00), (-0.50, 0.38), (-0.43, 0.82), (0.35, 0.82),
        (0.55, 0.54), (0.12, 0.29), (0.48, 0.00),
    ),
    "stepped_z": (
        (-0.42, 0.00), (-0.42, 0.35), (0.12, 0.35), (0.12, 0.70),
        (-0.12, 0.70), (-0.12, 1.00), (0.50, 0.70), (0.50, 0.35),
        (0.43, 0.00),
    ),
    "keyhole": (
        (-0.42, 0.00), (-0.38, 0.50), (-0.05, 0.88), (0.35, 0.90),
        (0.48, 0.62), (0.18, 0.40), (-0.12, 0.52), (0.06, 0.23),
        (0.45, 0.00),
    ),
}


def scenario_label_for_package(package_type: int) -> str:
    return _profile(package_type).label


def reference_path_for_package(
    package_type: int,
    reference_root: Path | None = None,
) -> Path:
    profile = _profile(package_type)
    return (reference_root or DEFAULT_REFERENCE_ROOT) / profile.reference_filename


def _profile(package_type: int) -> ScenarioProfile:
    try:
        return SCENARIO_PROFILES[int(package_type)]
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("자동 생성 시나리오 Type은 1~5여야 합니다.") from exc


def _number(value: Any, default: float = 0.0) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return float(default)
    return number if math.isfinite(number) else float(default)


def _coord(source: Any) -> dict[str, float] | None:
    if not isinstance(source, dict):
        return None
    lat = source.get("latitude", source.get("Latitude"))
    lon = source.get("longitude", source.get("Longitude"))
    if lat is None or lon is None:
        return None
    lat_value = _number(lat, math.nan)
    lon_value = _number(lon, math.nan)
    if not (math.isfinite(lat_value) and math.isfinite(lon_value)):
        return None
    return {"latitude": lat_value, "longitude": lon_value}


def _coords(items: Any) -> list[dict[str, float]]:
    if not isinstance(items, list):
        return []
    return [point for item in items if (point := _coord(item)) is not None]


def _mean_point(points: Sequence[dict[str, float]]) -> dict[str, float]:
    if not points:
        raise ValueError("좌표 중심을 계산할 수 없습니다.")
    return {
        "latitude": sum(point["latitude"] for point in points) / len(points),
        "longitude": sum(point["longitude"] for point in points) / len(points),
    }


def _project(point: dict[str, float], origin: dict[str, float]) -> tuple[float, float]:
    cos_lat = max(0.1, math.cos(math.radians(origin["latitude"])))
    east = (point["longitude"] - origin["longitude"]) * EARTH_M_PER_DEG * cos_lat
    north = (point["latitude"] - origin["latitude"]) * EARTH_M_PER_DEG
    return east, north


def _unproject(east: float, north: float, origin: dict[str, float]) -> dict[str, float]:
    cos_lat = max(0.1, math.cos(math.radians(origin["latitude"])))
    return {
        "latitude": round(origin["latitude"] + north / EARTH_M_PER_DEG, 8),
        "longitude": round(origin["longitude"] + east / (EARTH_M_PER_DEG * cos_lat), 8),
    }


def distance_m(left: dict[str, float], right: dict[str, float]) -> float:
    east, north = _project(right, left)
    return math.hypot(east, north)


def _offset(point: dict[str, float], east_m: float, north_m: float) -> dict[str, float]:
    return _unproject(east_m, north_m, point)


def point_in_polygon(point: dict[str, float], polygon: Sequence[dict[str, float]]) -> bool:
    if len(polygon) < 3:
        return False
    origin = _mean_point(polygon)
    x, y = _project(point, origin)
    vertices = [_project(vertex, origin) for vertex in polygon]
    inside = False
    j = len(vertices) - 1
    for i, (xi, yi) in enumerate(vertices):
        xj, yj = vertices[j]
        if ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi
        ):
            inside = not inside
        j = i
    return inside


def point_to_polygon_boundary_m(
    point: dict[str, float],
    polygon: Sequence[dict[str, float]],
) -> float:
    if len(polygon) < 2:
        return math.inf
    origin = _mean_point(polygon)
    px, py = _project(point, origin)
    vertices = [_project(vertex, origin) for vertex in polygon]
    best = math.inf
    for idx, (ax, ay) in enumerate(vertices):
        bx, by = vertices[(idx + 1) % len(vertices)]
        dx, dy = bx - ax, by - ay
        denom = dx * dx + dy * dy
        t = 0.0 if denom <= 1e-12 else ((px - ax) * dx + (py - ay) * dy) / denom
        t = max(0.0, min(1.0, t))
        best = min(best, math.hypot(px - (ax + t * dx), py - (ay + t * dy)))
    return best


def _segment_polygon_intersections(
    start: dict[str, float],
    end: dict[str, float],
    polygon: Sequence[dict[str, float]],
) -> list[tuple[float, dict[str, float]]]:
    if len(polygon) < 3:
        return []
    origin = _mean_point([start, end, *polygon])
    sx, sy = _project(start, origin)
    ex, ey = _project(end, origin)
    rx, ry = ex - sx, ey - sy
    vertices = [_project(vertex, origin) for vertex in polygon]
    hits: list[tuple[float, dict[str, float]]] = []
    for idx, (ax, ay) in enumerate(vertices):
        bx, by = vertices[(idx + 1) % len(vertices)]
        qx, qy = bx - ax, by - ay
        denom = rx * qy - ry * qx
        if abs(denom) <= 1e-9:
            continue
        asx, asy = ax - sx, ay - sy
        t = (asx * qy - asy * qx) / denom
        u = (asx * ry - asy * rx) / denom
        if -1e-9 <= t <= 1.0 + 1e-9 and -1e-9 <= u <= 1.0 + 1e-9:
            hit = _unproject(sx + t * rx, sy + t * ry, origin)
            if not hits or all(abs(t - old_t) > 1e-7 for old_t, _ in hits):
                hits.append((max(0.0, min(1.0, t)), hit))
    return sorted(hits, key=lambda item: item[0])


def line_to_polygon_boundary(
    start: dict[str, float],
    polygon: Sequence[dict[str, float]],
) -> dict[str, float]:
    """start에서 polygon 중심으로 향하는 선을 첫 교점에서 자른다."""

    center = _mean_point(polygon)
    hits = _segment_polygon_intersections(start, center, polygon)
    if not hits:
        return center
    if point_in_polygon(start, polygon):
        return hits[-1][1]
    return hits[0][1]


def _exit_toward(
    polygon: Sequence[dict[str, float]],
    destination: dict[str, float],
) -> dict[str, float]:
    center = _mean_point(polygon)
    hits = _segment_polygon_intersections(center, destination, polygon)
    return hits[-1][1] if hits else center


def _polygon_boundary_toward(
    polygon: Sequence[dict[str, float]],
    toward: dict[str, float],
) -> dict[str, float]:
    """Return the outer boundary point facing ``toward`` using an extended ray.

    Unlike a center-to-destination segment, this still reaches the boundary
    when the destination happens to fall inside a moved or enlarged polygon.
    """

    if len(polygon) < 3:
        return _mean_point(polygon) if polygon else dict(toward)
    center = _mean_point(polygon)
    east, north = _project(toward, center)
    norm = math.hypot(east, north)
    if norm <= 1e-6:
        east, north, norm = 1.0, 0.0, 1.0
    radius_m = max((distance_m(center, point) for point in polygon), default=1.0)
    ray_length_m = max(norm + radius_m * 2.0, radius_m * 4.0, 100.0)
    ray_end = _offset(center, east / norm * ray_length_m, north / norm * ray_length_m)
    hits = _segment_polygon_intersections(center, ray_end, polygon)
    return hits[-1][1] if hits else min(polygon, key=lambda point: distance_m(point, toward))


def _closest_point_on_polygon_edge(
    toward: dict[str, float],
    polygon: Sequence[dict[str, float]],
    edge_index: int,
    *,
    inset_fraction: float = 0.0,
) -> dict[str, float]:
    if len(polygon) < 2:
        return dict(toward)
    origin = _mean_point([toward, *polygon])
    px, py = _project(toward, origin)
    start = polygon[edge_index % len(polygon)]
    end = polygon[(edge_index + 1) % len(polygon)]
    ax, ay = _project(start, origin)
    bx, by = _project(end, origin)
    dx, dy = bx - ax, by - ay
    denominator = dx * dx + dy * dy
    fraction = 0.0 if denominator <= 1e-12 else (
        ((px - ax) * dx + (py - ay) * dy) / denominator
    )
    inset = max(0.0, min(0.45, float(inset_fraction)))
    fraction = max(inset, min(1.0 - inset, fraction))
    return _unproject(ax + fraction * dx, ay + fraction * dy, origin)


def _nearest_polygon_edge_index(
    point: dict[str, float],
    polygon: Sequence[dict[str, float]],
) -> int:
    if len(polygon) < 2:
        return 0
    candidates = [
        (
            distance_m(
                point,
                _closest_point_on_polygon_edge(point, polygon, edge_index),
            ),
            edge_index,
        )
        for edge_index in range(len(polygon))
    ]
    return min(candidates)[1]


def _polygon_exit_anchor(
    polygon: Sequence[dict[str, float]],
    toward: dict[str, float],
    entry_anchor: dict[str, float] | None,
) -> dict[str, float]:
    """Choose an exit portal facing the next node on a different Area edge."""

    candidate = _polygon_boundary_toward(polygon, toward)
    if entry_anchor is None or len(polygon) < 3:
        return candidate
    entry_edge = _nearest_polygon_edge_index(entry_anchor, polygon)
    if _nearest_polygon_edge_index(candidate, polygon) != entry_edge:
        return candidate
    alternatives = [
        (
            distance_m(
                toward,
                _closest_point_on_polygon_edge(
                    toward,
                    polygon,
                    edge_index,
                    inset_fraction=0.06,
                ),
            ),
            edge_index,
        )
        for edge_index in range(len(polygon))
        if edge_index != entry_edge
    ]
    _distance, selected_edge = min(alternatives)
    return _closest_point_on_polygon_edge(
        toward,
        polygon,
        selected_edge,
        inset_fraction=0.06,
    )


def _trim_polyline_to_area_boundary(
    points: Sequence[dict[str, float]],
    polygon: Sequence[dict[str, float]],
    *,
    entering: bool,
) -> list[dict[str, float]]:
    """Clip a route at its first Area-boundary contact.

    For an outgoing route the operation is performed from the destination
    backwards and the result is reversed.  This is the important distinction
    from merely snapping the last coordinate: large Areas can contain one or
    more earlier skeleton waypoints.
    """

    working = [dict(point) for point in (reversed(points) if not entering else points)]
    if len(working) < 2 or len(polygon) < 3:
        return list(reversed(working)) if not entering else working
    clipped = [working[0]]
    for segment_index in range(len(working) - 1):
        start, end = working[segment_index], working[segment_index + 1]
        hits = _segment_polygon_intersections(start, end, polygon)
        # Ignore a numerical touch at the outside segment's starting point;
        # the next positive hit is the actual entry portal.
        positive_hits = [hit for fraction, hit in hits if fraction > 1e-7]
        if positive_hits:
            clipped.append(positive_hits[0])
            break
        if point_in_polygon(end, polygon):
            clipped.append(_polygon_boundary_toward(polygon, start))
            break
        clipped.append(end)
    result = list(reversed(clipped)) if not entering else clipped
    compact: list[dict[str, float]] = []
    for point in result:
        if not compact or distance_m(compact[-1], point) > 0.5:
            compact.append(point)
    return compact


def _outside_area_portal(
    polygon: Sequence[dict[str, float]],
    anchor: dict[str, float],
    clearance_m: float = 350.0,
) -> dict[str, float]:
    center = _mean_point(polygon)
    east, north = _project(anchor, center)
    norm = math.hypot(east, north)
    if norm <= 1e-6:
        east, north, norm = 1.0, 0.0, 1.0
    return _offset(
        anchor,
        east / norm * clearance_m,
        north / norm * clearance_m,
    )


def _line_entries(mission: dict[str, Any]) -> list[dict[str, Any]]:
    modern = mission.get("PolyLines")
    if isinstance(modern, dict):
        rows = modern.get("LineList") or modern.get("lineList") or []
        if isinstance(rows, dict):
            rows = [rows]
        return [row for row in rows if isinstance(row, dict)]
    legacy = mission.get("PolyLine")
    if isinstance(legacy, dict) and (legacy.get("CoordinateList") or legacy.get("coordinateList")):
        return [legacy]
    alternate = mission.get("Polylines")
    if isinstance(alternate, list):
        return [row for row in alternate if isinstance(row, dict)]
    return []


def _area_entries(mission: dict[str, Any]) -> list[dict[str, Any]]:
    polygons = mission.get("Polygons") or mission.get("polygons") or {}
    if not isinstance(polygons, dict):
        return []
    rows = polygons.get("AreaList") or polygons.get("areaList") or []
    if isinstance(rows, dict):
        rows = [rows]
    return [row for row in rows if isinstance(row, dict)]


def _entry_coords(entry: dict[str, Any]) -> list[dict[str, float]]:
    return _coords(entry.get("CoordinateList") or entry.get("coordinateList") or [])


def _reference_parts(payload: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    init = payload.get("InitScenario") or payload.get("initScenario") or {}
    if not isinstance(init, dict):
        raise ValueError("RTV 참조 파일의 InitScenario가 올바르지 않습니다.")
    package = init.get("InputMissionPackage") or init.get("inputMissionPackage") or {}
    mission_ref = init.get("MissionReferencePackage") or init.get("missionReferencePackage") or {}
    if not isinstance(package, dict) or not isinstance(mission_ref, dict):
        raise ValueError("RTV 참조 파일의 임무 패키지 구조가 올바르지 않습니다.")
    return package, mission_ref


def _reference_takeovers(mission_ref: dict[str, Any]) -> list[dict[str, float]]:
    rows = mission_ref.get("TakeOverInfoList") or mission_ref.get("takeOverInfoList") or []
    points: list[dict[str, float]] = []
    for row in rows if isinstance(rows, list) else []:
        if not isinstance(row, dict):
            continue
        point = _coord(row.get("CoordinateList") or row.get("coordinate"))
        if point:
            points.append(point)
    return points


def _reference_area_lists(mission_ref: dict[str, Any], key: str) -> list[list[dict[str, float]]]:
    rows = mission_ref.get(key) or []
    areas: list[list[dict[str, float]]] = []
    for row in rows if isinstance(rows, list) else []:
        if not isinstance(row, dict):
            continue
        points = _coords(row.get("AreaLatLonList") or row.get("areaLatLonList") or [])
        if len(points) >= 3:
            areas.append(points)
    return areas


def _mission_geometry(missions: Sequence[dict[str, Any]]) -> list[dict[str, float]]:
    points: list[dict[str, float]] = []
    for mission in missions:
        for line in _line_entries(mission):
            points.extend(_entry_coords(line))
        for area in _area_entries(mission):
            points.extend(_entry_coords(area))
        coordinate = _coord(mission.get("Coordinate") or mission.get("coordinate"))
        if coordinate and (abs(coordinate["latitude"]) > 1e-8 or abs(coordinate["longitude"]) > 1e-8):
            points.append(coordinate)
    return points


def _random_config_values() -> tuple[
    tuple[float, float],
    tuple[float, float],
    tuple[tuple[float, float], ...],
    float,
    float,
    float,
]:
    try:
        from modules.Random_mission.FPL_Random.fpl_random import config as random_config

        return (
            tuple(map(float, random_config.AUTO_MISSION_AREA_SW)),
            tuple(map(float, random_config.AUTO_MISSION_AREA_NE)),
            tuple(tuple(map(float, row)) for row in random_config.START_REFERENCE_POINTS_RAW),
            float(random_config.SIDE_M),
            float(random_config.HANDOVER_OFFSET_M),
            float(random_config.RTB_OFFSET_M),
        )
    except Exception:
        return (
            (38.037074, 127.206058),
            (38.220469, 127.429963),
            ((38.042, 127.23), (38.042, 127.32), (38.042, 127.41)),
            150.0,
            300.0,
            300.0,
        )


def _choose_start_anchor(rng: random.Random) -> dict[str, float]:
    sw, ne, reference_points, _side, _handover, _rtb = _random_config_values()
    min_lat = min((row[0] for row in reference_points), default=sw[0])
    south_limit = min_lat + max(600.0 / EARTH_M_PER_DEG, (ne[0] - sw[0]) * 0.08)
    candidates = [row for row in reference_points if row[0] <= south_limit]
    if not candidates:
        candidates = sorted(reference_points, key=lambda row: row[0])[:3]
    lat, lon = rng.choice(candidates)
    anchor = {"latitude": float(lat), "longitude": float(lon)}
    # Keep the deployment in the southern start band, but avoid a handful of
    # visibly repeated anchor coordinates. Triangular sampling keeps most
    # offsets near the reference point while still allowing useful variation.
    anchor = _offset(
        anchor,
        rng.triangular(-900.0, 900.0, 0.0),
        rng.triangular(-250.0, 450.0, 0.0),
    )
    anchor["latitude"] = max(sw[0] + 0.001, min(ne[0] - 0.001, anchor["latitude"]))
    anchor["longitude"] = max(sw[1] + 0.001, min(ne[1] - 0.001, anchor["longitude"]))
    return anchor


def _rotate_offset_by_bearing(
    east_m: float,
    north_m: float,
    bearing_deg: float,
) -> tuple[float, float]:
    """Rotate an east/north offset clockwise from north by bearing_deg."""

    angle = math.radians(float(bearing_deg))
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    return (
        east_m * cos_a + north_m * sin_a,
        -east_m * sin_a + north_m * cos_a,
    )


def _formation_points(
    anchor: dict[str, float],
    rng: random.Random,
    *,
    deployment_heading_deg: float = 0.0,
) -> tuple[list[dict[str, float]], list[dict[str, float]], list[dict[str, float]]]:
    _sw, _ne, _refs, side_m, handover_m, rtb_m = _random_config_values()
    height = math.sqrt(3.0) * side_m / 2.0
    formation_bearing_deg = float(deployment_heading_deg) + rng.triangular(-7.0, 7.0, 0.0)
    takeover_offsets = (
        (0.0, 2.0 * height / 3.0),
        (-side_m / 2.0, -height / 3.0),
        (side_m / 2.0, -height / 3.0),
    )
    takeovers = []
    for east_m, north_m in takeover_offsets:
        rotated_east, rotated_north = _rotate_offset_by_bearing(
            east_m,
            north_m,
            formation_bearing_deg,
        )
        takeovers.append(_offset(anchor, rotated_east, rotated_north))

    direction = rng.choice((-1.0, 1.0))
    lateral_east, lateral_north = _rotate_offset_by_bearing(1.0, 0.0, deployment_heading_deg)
    handovers = [
        _offset(
            point,
            direction * handover_m * lateral_east,
            direction * handover_m * lateral_north,
        )
        for point in takeovers
    ]
    rtbs = [
        _offset(
            point,
            -direction * rtb_m * lateral_east,
            -direction * rtb_m * lateral_north,
        )
        for point in takeovers
    ]
    return takeovers, handovers, rtbs


@dataclass(frozen=True)
class _LayoutRecipe:
    variant: str
    heading_deg: float
    forward_scale: float
    lateral_scale: float
    mirrored: bool
    shear_ratio: float
    curve_amplitude_m: float
    curve_cycles: int
    stage_forward_factors: tuple[float, ...]
    stage_lateral_offsets_m: tuple[float, ...]
    stage_lateral_scales: tuple[float, ...]
    turn_style: str
    density_style: str
    area_placement_style: str
    center_spacing_scale: float
    area_footprint_scale: float
    mission_turn_deltas_deg: tuple[float, ...]
    mission_length_factors: tuple[float, ...]
    mission_lateral_offsets_m: tuple[float, ...]
    mission_forward_offsets_m: tuple[float, ...]
    mission_local_rotations_deg: tuple[float, ...]
    mission_area_aspects: tuple[float, ...]
    skeleton_style: str
    skeleton_lateral_ratio: float
    stage_spacing_power: float
    branch_spacing_scale: float
    branch_fan_rotation_deg: float
    branch_splay_deg: float
    branch_stagger_m: float


class _DeploymentTransform:
    def __init__(
        self,
        source_origin: dict[str, float],
        target_origin: dict[str, float],
        source_axis_rad: float,
        source_extent_m: float,
        recipe: _LayoutRecipe,
    ) -> None:
        self.source_origin = source_origin
        self.target_origin = target_origin
        self.source_axis_rad = float(source_axis_rad)
        self.source_extent_m = max(float(source_extent_m), 1.0)
        self.recipe = recipe
        self.target_axis_rad = math.pi / 2.0 - math.radians(float(recipe.heading_deg))
        self.angle_rad = self.target_axis_rad - self.source_axis_rad
        self.scale = math.sqrt(float(recipe.forward_scale) * float(recipe.lateral_scale))

    def _curve_offset_m(self, progress: float) -> float:
        if progress <= 0.0 or progress >= 1.0:
            return 0.0
        cycles = max(1, int(self.recipe.curve_cycles))
        return float(self.recipe.curve_amplitude_m) * math.sin(math.pi * cycles * progress)

    @staticmethod
    def _interpolate_stage_values(values: Sequence[float], progress: float, default: float) -> float:
        if not values:
            return float(default)
        if len(values) == 1 or progress <= 0.0:
            return float(values[0])
        if progress >= 1.0:
            return float(values[-1])
        scaled = float(progress) * float(len(values) - 1)
        index = min(int(math.floor(scaled)), len(values) - 2)
        fraction = scaled - float(index)
        # Smooth interpolation avoids sharp kinks inside a polygon while still
        # letting successive mission stages move to distinct lateral positions.
        smooth = fraction * fraction * (3.0 - 2.0 * fraction)
        return float(values[index]) + (float(values[index + 1]) - float(values[index])) * smooth

    def _warped_forward_progress(self, progress: float) -> float:
        factors = self.recipe.stage_forward_factors
        if not factors or progress <= 0.0 or progress >= 1.0:
            return float(progress)
        scaled = float(progress) * float(len(factors))
        index = min(int(math.floor(scaled)), len(factors) - 1)
        fraction = scaled - float(index)
        total = sum(max(float(value), 0.05) for value in factors)
        completed = sum(max(float(value), 0.05) for value in factors[:index])
        completed += fraction * max(float(factors[index]), 0.05)
        return completed / max(total, 1e-9)

    def point(self, point: dict[str, float]) -> dict[str, float]:
        east, north = _project(point, self.source_origin)
        source_cos = math.cos(self.source_axis_rad)
        source_sin = math.sin(self.source_axis_rad)
        forward_m = east * source_cos + north * source_sin
        lateral_m = -east * source_sin + north * source_cos
        progress = forward_m / self.source_extent_m

        mirrored_lateral_m = (-lateral_m if self.recipe.mirrored else lateral_m)
        warped_progress = self._warped_forward_progress(progress)
        warped_forward_m = self.source_extent_m * warped_progress * float(self.recipe.forward_scale)
        stage_lateral_scale = self._interpolate_stage_values(
            self.recipe.stage_lateral_scales,
            progress,
            1.0,
        )
        stage_lateral_offset_m = self._interpolate_stage_values(
            self.recipe.stage_lateral_offsets_m,
            progress,
            0.0,
        )
        warped_lateral_m = (
            mirrored_lateral_m * float(self.recipe.lateral_scale) * stage_lateral_scale
            + forward_m * float(self.recipe.shear_ratio)
            + self._curve_offset_m(progress)
            + stage_lateral_offset_m
        )

        target_cos = math.cos(self.target_axis_rad)
        target_sin = math.sin(self.target_axis_rad)
        out_east = warped_forward_m * target_cos - warped_lateral_m * target_sin
        out_north = warped_forward_m * target_sin + warped_lateral_m * target_cos
        return _unproject(out_east, out_north, self.target_origin)

    def points(self, points: Sequence[dict[str, float]]) -> list[dict[str, float]]:
        return [self.point(point) for point in points]


def _sample_layout_recipe(
    rng: random.Random,
    *,
    package_type: int,
) -> _LayoutRecipe:
    heading_limit_deg = 18.0 if int(package_type) == 3 else 22.0
    forward_low, forward_high = ((0.93, 1.09) if int(package_type) == 3 else (0.90, 1.12))
    lateral_low, lateral_high = ((0.86, 1.16) if int(package_type) == 3 else (0.82, 1.20))

    variant = rng.choice(("straight", "arc_left", "arc_right", "s_left", "s_right"))
    if variant == "straight":
        curve_amplitude_m = rng.triangular(-260.0, 260.0, 0.0)
        curve_cycles = 1
    elif variant.startswith("arc_"):
        sign = 1.0 if variant.endswith("left") else -1.0
        curve_amplitude_m = sign * rng.triangular(550.0, 1_150.0, 760.0)
        curve_cycles = 1
    else:
        sign = 1.0 if variant.endswith("left") else -1.0
        curve_amplitude_m = sign * rng.triangular(420.0, 900.0, 620.0)
        curve_cycles = 2

    mission_sequence = MISSION_TYPE_SEQUENCES[int(package_type)]
    mission_count = len(mission_sequence)
    stage_interval_count = max(5, mission_count - 1)
    forward_spread = 0.12 if int(package_type) == 3 else 0.18
    stage_forward_factors = tuple(
        rng.triangular(1.0 - forward_spread, 1.0 + forward_spread, 1.0)
        for _ in range(stage_interval_count)
    )
    lateral_offset_limit_m = 360.0 if int(package_type) == 3 else 520.0
    stage_lateral_offsets = [0.0]
    previous_offset_m = 0.0
    for _ in range(stage_interval_count - 1):
        target_offset_m = rng.triangular(
            -lateral_offset_limit_m,
            lateral_offset_limit_m,
            0.0,
        )
        previous_offset_m = 0.30 * previous_offset_m + 0.70 * target_offset_m
        stage_lateral_offsets.append(previous_offset_m)
    stage_lateral_offsets.append(0.0)

    lateral_stage_spread = 0.10 if int(package_type) == 3 else 0.14
    stage_lateral_scales = [1.0]
    stage_lateral_scales.extend(
        rng.triangular(
            1.0 - lateral_stage_spread,
            1.0 + lateral_stage_spread,
            1.0,
        )
        for _ in range(stage_interval_count - 1)
    )
    stage_lateral_scales.append(1.0)

    # A layout-v6 skeleton is selected independently of the RTV's stage
    # centroids.  Left/right variants are kept explicit so a seed replay is
    # easy to diagnose from metadata.
    skeleton_style = rng.choice(
        tuple(SKELETON_CONTROL_PATHS)
    )
    if int(package_type) == 3:
        skeleton_lateral_ratio = rng.triangular(0.58, 0.88, 0.70)
    else:
        skeleton_lateral_ratio = rng.triangular(0.62, 0.96, 0.76)
    stage_spacing_power = rng.triangular(0.88, 1.12, 1.0)
    if int(package_type) in (2, 3):
        branch_spacing_scale = rng.triangular(0.72, 1.42, 1.0)
        branch_fan_rotation_deg = rng.choice((-1.0, 1.0)) * rng.uniform(12.0, 38.0)
        branch_splay_deg = rng.uniform(8.0, 24.0)
        branch_stagger_m = rng.uniform(-700.0, 700.0)
    else:
        branch_spacing_scale = 1.0
        branch_fan_rotation_deg = 0.0
        branch_splay_deg = 0.0
        branch_stagger_m = 0.0

    # The spatial warp above changes the overall silhouette.  These styles are
    # sampled separately for the semantic mission stages so an outbound line
    # and a return line occupying the same map band do not receive the same
    # deformation again.
    turn_style = rng.choice(
        ("aligned", "sweep_left", "sweep_right", "alternating_left", "alternating_right")
    )
    turn_limit_deg = 10.0 if int(package_type) == 3 else 13.0
    mission_turn_deltas_deg: list[float] = []
    for interval_idx in range(mission_count - 1):
        if turn_style == "aligned":
            delta_deg = rng.triangular(-4.5, 4.5, 0.0)
        elif turn_style.startswith("sweep_"):
            sign = 1.0 if turn_style.endswith("left") else -1.0
            delta_deg = sign * rng.triangular(3.5, turn_limit_deg, 7.0)
        else:
            starts_left = turn_style.endswith("left")
            sign = 1.0 if (interval_idx % 2 == 0) == starts_left else -1.0
            delta_deg = sign * rng.triangular(5.5, turn_limit_deg, 8.5)
        mission_turn_deltas_deg.append(delta_deg)

    density_style = rng.choice(("compact", "compact", "balanced", "balanced", "balanced", "open", "open"))
    if int(package_type) == 3:
        density_ranges = {
            "compact": ((0.86, 0.93), (1.06, 1.12)),
            "balanced": ((0.97, 1.04), (0.96, 1.04)),
            "open": ((1.10, 1.18), (0.82, 0.92)),
        }
        mission_length_low, mission_length_high = 0.93, 1.09
    else:
        density_ranges = {
            "compact": ((0.84, 0.93), (1.03, 1.10)),
            "balanced": ((0.96, 1.04), (0.94, 1.07)),
            "open": ((1.08, 1.17), (0.88, 0.98)),
        }
        mission_length_low, mission_length_high = 0.90, 1.12
    spacing_range, footprint_range = density_ranges[density_style]
    center_spacing_scale = rng.triangular(*spacing_range, sum(spacing_range) / 2.0)
    area_footprint_scale = rng.triangular(*footprint_range, sum(footprint_range) / 2.0)
    mission_length_factors = tuple(
        rng.triangular(mission_length_low, mission_length_high, 1.0)
        for _ in range(mission_count - 1)
    )

    area_placement_style = rng.choice(("left_bias", "right_bias", "staggered", "centered"))
    mission_lateral_offsets_m: list[float] = []
    mission_forward_offsets_m: list[float] = []
    mission_local_rotations_deg: list[float] = []
    mission_area_aspects: list[float] = []
    area_ordinal = 0
    offset_gain = 0.78 if int(package_type) == 3 else 1.0
    for mission_idx, (mission_type, _region_type) in enumerate(mission_sequence):
        progress = mission_idx / max(mission_count - 1, 1)
        envelope = math.sin(math.pi * progress)
        if mission_idx in (0, mission_count - 1):
            lateral_offset_m = 0.0
            forward_offset_m = 0.0
        else:
            if turn_style == "aligned":
                lateral_offset_m = rng.triangular(-260.0, 260.0, 0.0)
            elif turn_style.startswith("sweep_"):
                sign = 1.0 if turn_style.endswith("left") else -1.0
                lateral_offset_m = sign * envelope * rng.triangular(240.0, 700.0, 440.0)
                lateral_offset_m += rng.triangular(-140.0, 140.0, 0.0)
            else:
                starts_left = turn_style.endswith("left")
                sign = 1.0 if (mission_idx % 2 == 0) == starts_left else -1.0
                lateral_offset_m = sign * envelope * rng.triangular(320.0, 820.0, 520.0)
            forward_offset_m = envelope * rng.triangular(-420.0, 420.0, 0.0)

        is_area_mission = int(mission_type) != 1
        if is_area_mission and mission_idx not in (0, mission_count - 1):
            if area_placement_style == "left_bias":
                lateral_offset_m += rng.triangular(260.0, 820.0, 480.0)
            elif area_placement_style == "right_bias":
                lateral_offset_m -= rng.triangular(260.0, 820.0, 480.0)
            elif area_placement_style == "staggered":
                area_sign = 1.0 if area_ordinal % 2 == 0 else -1.0
                lateral_offset_m += area_sign * rng.triangular(340.0, 900.0, 560.0)
            else:
                lateral_offset_m += rng.triangular(-240.0, 240.0, 0.0)
            forward_offset_m += rng.triangular(-320.0, 320.0, 0.0)
            area_ordinal += 1

        previous_turn = mission_turn_deltas_deg[max(0, mission_idx - 1)] if mission_turn_deltas_deg else 0.0
        next_turn = mission_turn_deltas_deg[min(mission_idx, len(mission_turn_deltas_deg) - 1)] if mission_turn_deltas_deg else 0.0
        local_rotation_deg = 0.35 * (previous_turn + next_turn)
        local_rotation_deg += rng.triangular(-4.5, 4.5, 0.0)
        if is_area_mission:
            local_rotation_deg += rng.triangular(-7.0, 7.0, 0.0)
        local_rotation_limit = 12.0 if int(package_type) == 3 else 16.0

        mission_lateral_offsets_m.append(offset_gain * lateral_offset_m)
        mission_forward_offsets_m.append(offset_gain * forward_offset_m)
        mission_local_rotations_deg.append(
            max(-local_rotation_limit, min(local_rotation_limit, local_rotation_deg))
        )
        if is_area_mission:
            aspect_low, aspect_high = ((0.88, 1.14) if int(package_type) == 3 else (0.82, 1.22))
            mission_area_aspects.append(rng.triangular(aspect_low, aspect_high, 1.0))
        else:
            mission_area_aspects.append(1.0)

    return _LayoutRecipe(
        variant=variant,
        heading_deg=rng.triangular(-heading_limit_deg, heading_limit_deg, 0.0),
        forward_scale=rng.triangular(forward_low, forward_high, 1.0),
        lateral_scale=rng.triangular(lateral_low, lateral_high, 1.0),
        mirrored=bool(rng.getrandbits(1)),
        shear_ratio=rng.triangular(-0.055, 0.055, 0.0),
        curve_amplitude_m=curve_amplitude_m,
        curve_cycles=curve_cycles,
        stage_forward_factors=stage_forward_factors,
        stage_lateral_offsets_m=tuple(stage_lateral_offsets),
        stage_lateral_scales=tuple(stage_lateral_scales),
        turn_style=turn_style,
        density_style=density_style,
        area_placement_style=area_placement_style,
        center_spacing_scale=center_spacing_scale,
        area_footprint_scale=area_footprint_scale,
        mission_turn_deltas_deg=tuple(mission_turn_deltas_deg),
        mission_length_factors=mission_length_factors,
        mission_lateral_offsets_m=tuple(mission_lateral_offsets_m),
        mission_forward_offsets_m=tuple(mission_forward_offsets_m),
        mission_local_rotations_deg=tuple(mission_local_rotations_deg),
        mission_area_aspects=tuple(mission_area_aspects),
        skeleton_style=skeleton_style,
        skeleton_lateral_ratio=skeleton_lateral_ratio,
        stage_spacing_power=stage_spacing_power,
        branch_spacing_scale=branch_spacing_scale,
        branch_fan_rotation_deg=branch_fan_rotation_deg,
        branch_splay_deg=branch_splay_deg,
        branch_stagger_m=branch_stagger_m,
    )


def _build_transform(
    reference_anchor: dict[str, float],
    target_anchor: dict[str, float],
    geometry: Sequence[dict[str, float]],
    rng: random.Random,
    *,
    package_type: int,
) -> tuple[_DeploymentTransform, float, float]:
    geometry_center = _mean_point(geometry)
    east, north = _project(geometry_center, reference_anchor)
    source_axis_rad = math.atan2(north, east)
    source_cos = math.cos(source_axis_rad)
    source_sin = math.sin(source_axis_rad)
    forward_values = []
    for point in geometry:
        point_east, point_north = _project(point, reference_anchor)
        forward_values.append(point_east * source_cos + point_north * source_sin)
    source_extent_m = max(forward_values, default=1.0)
    recipe = _sample_layout_recipe(rng, package_type=int(package_type))
    transform = _DeploymentTransform(
        reference_anchor,
        target_anchor,
        source_axis_rad,
        source_extent_m,
        recipe,
    )
    return transform, transform.scale, float(recipe.heading_deg)


def _deform_polygon(
    polygon: Sequence[dict[str, float]],
    rng: random.Random,
    *,
    mild: bool = False,
    deformation: tuple[float, float, float] | None = None,
) -> list[dict[str, float]]:
    if len(polygon) < 3:
        return list(polygon)
    center = _mean_point(polygon)
    if deformation is None:
        scale_low, scale_high = ((0.97, 1.04) if mild else (0.86, 1.18))
        rotation_limit_deg = 4.0 if mild else 12.0
        east_scale = rng.triangular(scale_low, scale_high, 1.0)
        north_scale = rng.triangular(scale_low, scale_high, 1.0)
        rotation_deg = rng.triangular(-rotation_limit_deg, rotation_limit_deg, 0.0)
    else:
        east_scale, north_scale, rotation_deg = deformation
    rotation_rad = math.radians(float(rotation_deg))
    cos_a = math.cos(rotation_rad)
    sin_a = math.sin(rotation_rad)
    result: list[dict[str, float]] = []
    for point in polygon:
        east, north = _project(point, center)
        vertex_scale = (
            rng.triangular(0.97, 1.03, 1.0)
            if not mild
            else rng.triangular(0.99, 1.01, 1.0)
        )
        scaled_east = east * float(east_scale) * vertex_scale
        scaled_north = north * float(north_scale) * vertex_scale
        rotated_east = scaled_east * cos_a - scaled_north * sin_a
        rotated_north = scaled_east * sin_a + scaled_north * cos_a
        result.append(_unproject(rotated_east, rotated_north, center))
    return result


def _ensure_holes_inside(areas: list[dict[str, Any]]) -> None:
    outers = [area for area in areas if not area.get("isHole") and len(area.get("points") or []) >= 3]
    if not outers:
        return
    outer = outers[0]["points"]
    outer_center = _mean_point(outer)
    for area in areas:
        points = area.get("points") or []
        if not area.get("isHole") or all(point_in_polygon(point, outer) for point in points):
            continue
        center = _mean_point(points)
        repaired: list[dict[str, float]] = []
        for point in points:
            east, north = _project(point, center)
            candidate = _unproject(east * 0.65, north * 0.65, center)
            if not point_in_polygon(candidate, outer):
                toward_east, toward_north = _project(outer_center, candidate)
                candidate = _offset(candidate, toward_east * 0.55, toward_north * 0.55)
            repaired.append(candidate)
        area["points"] = repaired


def _transform_missions(
    reference_missions: Sequence[dict[str, Any]],
    transform: _DeploymentTransform,
    rng: random.Random,
    *,
    package_type: int,
) -> list[dict[str, Any]]:
    missions: list[dict[str, Any]] = []
    expected_sequence = MISSION_TYPE_SEQUENCES[package_type]
    if len(reference_missions) != len(expected_sequence):
        raise ValueError(
            f"Type {package_type} RTV 참조 임무 수({len(reference_missions)})와 "
            f"0201 계약({len(expected_sequence)})이 다릅니다."
        )
    for mission_index, reference in enumerate(reference_missions):
        mission_type, region_type = expected_sequence[mission_index]
        mission: dict[str, Any] = {
            "inputMissionType": mission_type,
            "regionType": region_type,
            "isDone": False,
            "lineList": [],
            "areaList": [],
            "coordinateList": [],
        }
        for line in _line_entries(reference):
            points = transform.points(_entry_coords(line))
            if len(points) < 2:
                continue
            if len(points) > 2:
                for idx in range(1, len(points) - 1):
                    points[idx] = _offset(points[idx], rng.uniform(-100.0, 100.0), rng.uniform(-100.0, 100.0))
            width = max(350, min(2_000, int(round(_number(line.get("Width", line.get("width")), 1000) * rng.uniform(0.8, 1.25) / 10.0) * 10)))
            mission["lineList"].append({"width": width, "points": points})
        area_rows = _area_entries(reference)
        # One bounded aspect/rotation recipe per mission keeps branch groups and
        # outer/hole polygons coherent instead of deforming each independently.
        area_deformation = None
        if area_rows:
            area_deformation = (
                rng.triangular(0.88, 1.15, 1.0),
                rng.triangular(0.88, 1.15, 1.0),
                rng.triangular(-10.0, 10.0, 0.0),
            )
        for area in area_rows:
            points = transform.points(_entry_coords(area))
            if len(points) < 3:
                continue
            mission["areaList"].append(
                {
                    "isHole": bool(area.get("IsHole", area.get("isHole", False))),
                    "points": _deform_polygon(
                        points,
                        rng,
                        deformation=area_deformation,
                    ),
                }
            )
        _ensure_holes_inside(mission["areaList"])
        coordinate = _coord(reference.get("Coordinate") or reference.get("coordinate"))
        if coordinate and (abs(coordinate["latitude"]) > 1e-8 or abs(coordinate["longitude"]) > 1e-8):
            mission["coordinateList"] = [transform.point(coordinate)]
        missions.append(mission)
    return missions


def _mission_geometry_points(mission: dict[str, Any]) -> list[dict[str, float]]:
    points: list[dict[str, float]] = []
    for line in mission.get("lineList") or []:
        points.extend(line.get("points") or [])
    for area in mission.get("areaList") or []:
        points.extend(area.get("points") or [])
    points.extend(mission.get("coordinateList") or [])
    return points


def _sample_skeleton_path(
    control_points: Sequence[tuple[float, float]],
    phase: float,
) -> tuple[float, float]:
    if not control_points:
        return 0.0, 0.0
    if len(control_points) == 1:
        return control_points[0]
    segment_lengths = [
        math.hypot(
            control_points[index + 1][0] - control_points[index][0],
            control_points[index + 1][1] - control_points[index][1],
        )
        for index in range(len(control_points) - 1)
    ]
    total_length = sum(segment_lengths)
    if total_length <= 1e-9:
        return control_points[0]
    remaining = max(0.0, min(1.0, float(phase))) * total_length
    for index, length in enumerate(segment_lengths):
        if remaining <= length or index == len(segment_lengths) - 1:
            fraction = 0.0 if length <= 1e-9 else min(1.0, remaining / length)
            start = control_points[index]
            end = control_points[index + 1]
            return (
                start[0] + (end[0] - start[0]) * fraction,
                start[1] + (end[1] - start[1]) * fraction,
            )
        remaining -= length
    return control_points[-1]


def _skeleton_control_phases(
    control_points: Sequence[tuple[float, float]],
    spacing_power: float,
) -> list[float]:
    """Return semantic phases at each control-path corner.

    ``_sample_skeleton_path`` is parameterized by arc length and layout v6
    applies ``phase ** spacing_power`` before sampling.  Keeping the inverse
    mapping here lets each generated line retain the selected skeleton's real
    corners instead of replacing it with one long chord.
    """

    if len(control_points) < 2:
        return [0.0]
    lengths = [
        math.hypot(
            control_points[index + 1][0] - control_points[index][0],
            control_points[index + 1][1] - control_points[index][1],
        )
        for index in range(len(control_points) - 1)
    ]
    total = sum(lengths)
    if total <= 1e-9:
        return [0.0 for _ in control_points]
    inverse_power = 1.0 / max(0.05, float(spacing_power))
    phases = [0.0]
    walked = 0.0
    for length in lengths:
        walked += length
        phases.append(max(0.0, min(1.0, walked / total)) ** inverse_power)
    phases[-1] = 1.0
    return phases


def _interpolate_xy_at_phase(
    phases: Sequence[float],
    points: Sequence[tuple[float, float]],
    phase: float,
) -> tuple[float, float]:
    if not phases or not points:
        return 0.0, 0.0
    phase = float(phase)
    if phase <= phases[0]:
        return points[0]
    if phase >= phases[-1]:
        return points[-1]
    for index in range(len(phases) - 1):
        start_phase, end_phase = phases[index], phases[index + 1]
        if phase > end_phase:
            continue
        fraction = 0.0 if end_phase <= start_phase else (
            (phase - start_phase) / (end_phase - start_phase)
        )
        start, end = points[index], points[index + 1]
        return (
            start[0] + (end[0] - start[0]) * fraction,
            start[1] + (end[1] - start[1]) * fraction,
        )
    return points[-1]


def _semantic_subpath(
    semantic_spine: Sequence[dict[str, Any]],
    start_phase: float,
    end_phase: float,
) -> list[dict[str, float]]:
    """Slice a phase-tagged spine while retaining every internal corner."""

    ordered = sorted(
        (
            (float(row.get("phase", 0.0)), _coord(row.get("point")))
            for row in semantic_spine
            if isinstance(row, dict)
        ),
        key=lambda row: row[0],
    )
    ordered = [(phase, point) for phase, point in ordered if point is not None]
    if not ordered:
        return []

    def point_at(phase: float) -> dict[str, float]:
        if phase <= ordered[0][0]:
            return dict(ordered[0][1])
        if phase >= ordered[-1][0]:
            return dict(ordered[-1][1])
        for index in range(len(ordered) - 1):
            left_phase, left = ordered[index]
            right_phase, right = ordered[index + 1]
            if phase > right_phase:
                continue
            fraction = 0.0 if right_phase <= left_phase else (
                (phase - left_phase) / (right_phase - left_phase)
            )
            east, north = _project(right, left)
            return _offset(left, east * fraction, north * fraction)
        return dict(ordered[-1][1])

    low, high = sorted((float(start_phase), float(end_phase)))
    points = [point_at(low)]
    points.extend(
        dict(point)
        for phase, point in ordered
        if low + 1e-9 < phase < high - 1e-9
    )
    points.append(point_at(high))
    if start_phase > end_phase:
        points.reverse()
    compact: list[dict[str, float]] = []
    for point in points:
        if not compact or distance_m(compact[-1], point) > 0.5:
            compact.append(point)
    return compact


def _apply_mission_stage_variation(
    missions: list[dict[str, Any]],
    transform: _DeploymentTransform,
    *,
    package_type: int,
    gain: float = 1.0,
) -> dict[str, Any]:
    """Place mission groups on a fresh semantic skeleton after the global warp.

    This deliberately does not accumulate the source RTV's center vectors.
    Doing so only produced a bent copy of the original.  Instead, package-type
    progress and a loop/zigzag/dogleg/S-curve profile define new centers.  A
    gain below one blends back toward the source only for safety fallback.
    Multiple branch polygons and an outer/hole pair still receive one affine
    transform and remain a coherent group.
    """

    if not missions:
        return {"missionCenterOffsets": [], "maxMissionCenterOffsetM": 0.0}
    recipe = transform.recipe
    gain = max(0.0, min(1.0, float(gain)))
    source_centers = [
        _mean_point(points) if (points := _mission_geometry_points(mission)) else transform.target_origin
        for mission in missions
    ]
    layout_origin = _mean_point(source_centers)
    source_xy = [_project(center, layout_origin) for center in source_centers]
    spacing = 1.0 + gain * (float(recipe.center_spacing_scale) - 1.0)
    # First/last mission groups remain hard anchored to the transformed RTV.
    # Density changes the internal reach, not the handover/return band.
    base_xy = list(source_xy)

    forward_x = math.cos(transform.target_axis_rad)
    forward_y = math.sin(transform.target_axis_rad)
    lateral_x = -forward_y
    lateral_y = forward_x
    phase_profile = MISSION_STAGE_PHASES[int(package_type)]
    if len(phase_profile) != len(missions):
        raise ValueError(f"Type {package_type} semantic stage profile length mismatch")
    control_path = SKELETON_CONTROL_PATHS[recipe.skeleton_style]
    all_geometry = [
        point
        for mission in missions
        for point in _mission_geometry_points(mission)
    ]
    geometry_xy = [_project(point, layout_origin) for point in all_geometry]
    geometry_forward = [east * forward_x + north * forward_y for east, north in geometry_xy]
    source_route_span_m = max(geometry_forward, default=0.0) - min(
        geometry_forward,
        default=0.0,
    )
    route_span_m = max(9_000.0, min(15_000.0, source_route_span_m * 0.68)) * spacing
    lateral_amplitude_m = route_span_m * float(recipe.skeleton_lateral_ratio)
    spacing_power = float(recipe.stage_spacing_power)
    sampled_control = [
        _sample_skeleton_path(control_path, max(0.0, phase) ** spacing_power)
        for phase in phase_profile
    ]
    anchor_east, anchor_north = base_xy[0]
    first_lateral, first_forward = sampled_control[0]
    mirror = -1.0 if recipe.mirrored else 1.0

    # One affine map sends the selected skeleton's first and last semantic
    # nodes to the transformed handover/return band.  Unlike a phase-varying
    # endpoint correction, an affine map cannot turn a simple skeleton into a
    # self-crossing one.
    last_lateral, last_forward = sampled_control[-1]
    control_delta_lateral = last_lateral - first_lateral
    control_delta_forward = last_forward - first_forward
    control_delta_norm = math.hypot(control_delta_lateral, control_delta_forward)
    if control_delta_norm <= 1e-6:
        control_delta_lateral, control_delta_forward, control_delta_norm = 1.0, 0.0, 1.0
    control_u_lateral = control_delta_lateral / control_delta_norm
    control_u_forward = control_delta_forward / control_delta_norm
    control_v_lateral = -control_u_forward * mirror
    control_v_forward = control_u_lateral * mirror
    end_delta_east = base_xy[-1][0] - anchor_east
    end_delta_north = base_xy[-1][1] - anchor_north
    end_delta_norm = math.hypot(end_delta_east, end_delta_north)
    if end_delta_norm <= 500.0:
        end_u_east, end_u_north = lateral_x, lateral_y
        end_delta_norm = 500.0
    else:
        end_u_east = end_delta_east / end_delta_norm
        end_u_north = end_delta_north / end_delta_norm
    end_v_east, end_v_north = -end_u_north, end_u_east
    sampled_across = [
        (lateral - first_lateral) * control_v_lateral
        + (forward - first_forward) * control_v_forward
        for lateral, forward in sampled_control[1:-1]
    ]
    mean_sampled_across = (
        sum(sampled_across) / len(sampled_across)
        if sampled_across
        else 1.0
    )
    if (
        mean_sampled_across
        * (end_v_east * forward_x + end_v_north * forward_y)
        < 0.0
    ):
        end_v_east, end_v_north = -end_v_east, -end_v_north
    cross_scale_m = route_span_m * (0.72 + 0.20 * float(recipe.skeleton_lateral_ratio))

    def map_skeleton_point(lateral: float, forward: float) -> tuple[float, float]:
        relative_lateral = lateral - first_lateral
        relative_forward = forward - first_forward
        along = (
            relative_lateral * control_u_lateral
            + relative_forward * control_u_forward
        )
        across = (
            relative_lateral * control_v_lateral
            + relative_forward * control_v_forward
        )
        return (
            anchor_east
            + end_u_east * along / control_delta_norm * end_delta_norm
            + end_v_east * across * cross_scale_m,
            anchor_north
            + end_u_north * along / control_delta_norm * end_delta_norm
            + end_v_north * across * cross_scale_m,
        )

    template_xy = [
        map_skeleton_point(*sampled_control[index])
        for index in range(len(missions))
    ]
    template_xy[0] = base_xy[0]
    target_xy = [
        (
            base_xy[index][0] + gain * (template_xy[index][0] - base_xy[index][0]),
            base_xy[index][1] + gain * (template_xy[index][1] - base_xy[index][1]),
        )
        for index in range(len(missions))
    ]

    target_centers: list[dict[str, float]] = []
    center_diagnostics: list[dict[str, float | int]] = []
    for mission_idx, (east, north) in enumerate(target_xy):
        # Skeleton topology carries the large-scale variety.  Mission-local
        # offsets remain deliberately small so an Area cannot pull one route
        # node across a nonadjacent leg of that skeleton.
        forward_offset = gain * float(recipe.mission_forward_offsets_m[mission_idx]) * 0.25
        lateral_offset = gain * float(recipe.mission_lateral_offsets_m[mission_idx]) * 0.25
        target_east = east + forward_offset * forward_x + lateral_offset * lateral_x
        target_north = north + forward_offset * forward_y + lateral_offset * lateral_y
        target_center = _unproject(target_east, target_north, layout_origin)
        target_centers.append(target_center)
        delta_east, delta_north = _project(target_center, source_centers[mission_idx])
        center_diagnostics.append(
            {
                "missionID": mission_idx + 1,
                "eastM": round(delta_east, 1),
                "northM": round(delta_north, 1),
                "distanceM": round(math.hypot(delta_east, delta_north), 1),
            }
        )

    for mission_idx, mission in enumerate(missions):
        source_center = source_centers[mission_idx]
        target_center = target_centers[mission_idx]
        rotation_rad = math.radians(gain * float(recipe.mission_local_rotations_deg[mission_idx]))
        cos_r, sin_r = math.cos(rotation_rad), math.sin(rotation_rad)
        raw_aspect = max(0.70, min(1.35, float(recipe.mission_area_aspects[mission_idx])))
        aspect = 1.0 + gain * (raw_aspect - 1.0)
        footprint = 1.0 + gain * (float(recipe.area_footprint_scale) - 1.0)

        def transform_point(point: dict[str, float], *, area_point: bool) -> dict[str, float]:
            east, north = _project(point, source_center)
            local_forward = east * forward_x + north * forward_y
            local_lateral = east * lateral_x + north * lateral_y
            if area_point:
                local_forward *= footprint / math.sqrt(aspect)
                local_lateral *= footprint * math.sqrt(aspect)
            scaled_east = local_forward * forward_x + local_lateral * lateral_x
            scaled_north = local_forward * forward_y + local_lateral * lateral_y
            rotated_east = scaled_east * cos_r - scaled_north * sin_r
            rotated_north = scaled_east * sin_r + scaled_north * cos_r
            return _offset(target_center, rotated_east, rotated_north)

        for line in mission.get("lineList") or []:
            line["points"] = [
                transform_point(point, area_point=False)
                for point in (line.get("points") or [])
            ]
        for area in mission.get("areaList") or []:
            area["points"] = [
                transform_point(point, area_point=True)
                for point in (area.get("points") or [])
            ]
        mission["coordinateList"] = [
            transform_point(point, area_point=False)
            for point in (mission.get("coordinateList") or [])
        ]
        _ensure_holes_inside(mission.get("areaList") or [])

    branch_spacing_m: list[float] = []
    branch_area_mission_idx = {2: 4, 3: 5}.get(int(package_type))
    if branch_area_mission_idx is not None:
        branch_areas = _outer_mission_areas(missions[branch_area_mission_idx])
        if len(branch_areas) >= 2:
            group_points = [point for area in branch_areas for point in area.get("points") or []]
            group_center = _mean_point(group_points)
            area_centers = [_mean_point(area["points"]) for area in branch_areas]
            center_index = (len(branch_areas) - 1) / 2.0
            fan_angle = math.radians(gain * float(recipe.branch_fan_rotation_deg))
            fan_cos, fan_sin = math.cos(fan_angle), math.sin(fan_angle)
            branch_scale = 1.0 + gain * (float(recipe.branch_spacing_scale) - 1.0)
            new_centers: list[dict[str, float]] = []
            for area_idx, (area, old_center) in enumerate(zip(branch_areas, area_centers)):
                east, north = _project(old_center, group_center)
                rotated_east = (east * fan_cos - north * fan_sin) * branch_scale
                rotated_north = (east * fan_sin + north * fan_cos) * branch_scale
                ordinal = area_idx - center_index
                stagger = gain * ordinal * float(recipe.branch_stagger_m)
                rotated_east += stagger * forward_x
                rotated_north += stagger * forward_y
                new_center = _offset(group_center, rotated_east, rotated_north)
                new_centers.append(new_center)
                splay_angle = math.radians(
                    gain * ordinal * float(recipe.branch_splay_deg)
                )
                splay_cos, splay_sin = math.cos(splay_angle), math.sin(splay_angle)
                rebuilt: list[dict[str, float]] = []
                for point in area.get("points") or []:
                    local_east, local_north = _project(point, old_center)
                    rebuilt.append(
                        _offset(
                            new_center,
                            local_east * splay_cos - local_north * splay_sin,
                            local_east * splay_sin + local_north * splay_cos,
                        )
                    )
                area["points"] = rebuilt

            # The former implementation checked center-to-center spacing only.
            # A 4 km polygon could therefore overlap a sibling whose center was
            # 800 m away.  Lay branch footprints out on one ordered fan axis,
            # using projected polygon half-widths plus the composite-route
            # clearance.  Keeping the same order from split to merge also
            # prevents branch chords from swapping sides and crossing.
            upstream_areas = _outer_mission_areas(missions[branch_area_mission_idx - 2])
            upstream_center = (
                _mean_point(
                    [
                        point
                        for upstream_area in upstream_areas
                        for point in (upstream_area.get("points") or [])
                    ]
                )
                if upstream_areas
                else _offset(group_center, -forward_x * 5_000.0, -forward_y * 5_000.0)
            )
            branch_exit_mission_idx = 5 if int(package_type) == 2 else 6
            merge_phase = ROUTE_LINE_PHASE_RANGES[int(package_type)][branch_exit_mission_idx][1]
            merge_template_xy = map_skeleton_point(
                *_sample_skeleton_path(
                    control_path,
                    max(0.0, merge_phase) ** spacing_power,
                )
            )
            merge_source_xy = _interpolate_xy_at_phase(
                phase_profile,
                source_xy,
                merge_phase,
            )
            merge_xy = (
                merge_source_xy[0] + gain * (merge_template_xy[0] - merge_source_xy[0]),
                merge_source_xy[1] + gain * (merge_template_xy[1] - merge_source_xy[1]),
            )
            merge_reference = _unproject(*merge_xy, layout_origin)
            route_east, route_north = _project(merge_reference, upstream_center)
            route_norm = math.hypot(route_east, route_north)
            if route_norm <= 1e-6:
                route_east, route_north, route_norm = forward_x, forward_y, 1.0
            route_east /= route_norm
            route_north /= route_norm
            # Branch areas occupy a cross-section normal to the local
            # split-to-merge direction.  Retain only a mild fan rotation so
            # ordering cannot invert between entry and exit.
            branch_axis_east, branch_axis_north = -route_north, route_east
            mild_fan_angle = fan_angle * 0.20
            mild_fan_cos, mild_fan_sin = math.cos(mild_fan_angle), math.sin(mild_fan_angle)
            branch_axis_east, branch_axis_north = (
                branch_axis_east * mild_fan_cos - branch_axis_north * mild_fan_sin,
                branch_axis_east * mild_fan_sin + branch_axis_north * mild_fan_cos,
            )
            axis_norm = math.hypot(branch_axis_east, branch_axis_north) or 1.0
            branch_axis_east /= axis_norm
            branch_axis_north /= axis_norm
            normal_east, normal_north = -branch_axis_north, branch_axis_east
            if normal_east * route_east + normal_north * route_north < 0.0:
                normal_east, normal_north = -normal_east, -normal_north

            current_centers = [_mean_point(area["points"]) for area in branch_areas]
            half_widths: list[float] = []
            for area, center in zip(branch_areas, current_centers):
                projections = []
                for point in area.get("points") or []:
                    east, north = _project(point, center)
                    projections.append(east * branch_axis_east + north * branch_axis_north)
                half_widths.append(
                    max(250.0, (max(projections, default=0.0) - min(projections, default=0.0)) / 2.0)
                )
            branch_edge_gap_m = 700.0
            total_width_m = sum(width * 2.0 for width in half_widths)
            total_width_m += branch_edge_gap_m * max(0, len(branch_areas) - 1)
            cursor_m = -total_width_m / 2.0
            ordered_centers: list[dict[str, float]] = []
            for area_idx, (area, old_center, half_width) in enumerate(
                zip(branch_areas, current_centers, half_widths)
            ):
                axis_position_m = cursor_m + half_width
                cursor_m += half_width * 2.0 + branch_edge_gap_m
                ordinal = area_idx - center_index
                # A small longitudinal stagger is retained for visual variety,
                # but it cannot change the branch's lateral ordering.
                normal_position_m = gain * ordinal * float(recipe.branch_stagger_m) * 0.30
                new_center = _offset(
                    group_center,
                    branch_axis_east * axis_position_m + normal_east * normal_position_m,
                    branch_axis_north * axis_position_m + normal_north * normal_position_m,
                )
                delta_east, delta_north = _project(new_center, old_center)
                area["points"] = [
                    _offset(point, delta_east, delta_north)
                    for point in (area.get("points") or [])
                ]
                ordered_centers.append(new_center)
            new_centers = ordered_centers

            upstream_points = [
                point
                for upstream_area in upstream_areas
                for point in (upstream_area.get("points") or [])
            ]
            branch_points = [
                point
                for area in branch_areas
                for point in (area.get("points") or [])
            ]
            if upstream_points and branch_points:
                updated_group_center = _mean_point(branch_points)
                upstream_center = _mean_point(upstream_points)
                center_east, center_north = _project(updated_group_center, upstream_center)
                current_forward_m = center_east * route_east + center_north * route_north
                upstream_extent_m = max(
                    (
                        _project(point, upstream_center)[0] * route_east
                        + _project(point, upstream_center)[1] * route_north
                        for point in upstream_points
                    ),
                    default=0.0,
                )
                branch_backward_extent_m = max(
                    (
                        -(
                            _project(point, updated_group_center)[0] * route_east
                            + _project(point, updated_group_center)[1] * route_north
                        )
                        for point in branch_points
                    ),
                    default=0.0,
                )
                required_forward_m = (
                    upstream_extent_m + branch_backward_extent_m + 1_500.0
                )
                forward_shift_m = max(0.0, required_forward_m - current_forward_m)
                if forward_shift_m > 0.0:
                    for area in branch_areas:
                        area["points"] = [
                            _offset(
                                point,
                                route_east * forward_shift_m,
                                route_north * forward_shift_m,
                            )
                            for point in (area.get("points") or [])
                        ]
                    new_centers = [
                        _offset(
                            center,
                            route_east * forward_shift_m,
                            route_north * forward_shift_m,
                        )
                        for center in new_centers
                    ]
            branch_spacing_m = [
                distance_m(new_centers[left], new_centers[right])
                for left in range(len(new_centers))
                for right in range(left + 1, len(new_centers))
            ]

    # Materialize the semantic spine used by the topology builder.  Lines are
    # generated from this ordered route, not translated as independent mission
    # objects.  Area centers replace the matching route node so every incoming
    # and outgoing corridor is derived from the Area it actually serves.
    route_ranges = ROUTE_LINE_PHASE_RANGES[int(package_type)]
    sample_phases = {
        0.0,
        1.0,
        *phase_profile,
        *(
            phase
            for phase_range in route_ranges.values()
            for phase in phase_range
        ),
        *_skeleton_control_phases(control_path, spacing_power),
    }

    def raw_template_at(phase: float) -> tuple[float, float]:
        lateral, forward = _sample_skeleton_path(
            control_path,
            max(0.0, min(1.0, float(phase))) ** spacing_power,
        )
        return map_skeleton_point(lateral, forward)

    spine_xy: dict[float, tuple[float, float]] = {}
    for phase in sorted(sample_phases):
        template_east, template_north = raw_template_at(phase)
        source_east, source_north = _interpolate_xy_at_phase(
            phase_profile,
            source_xy,
            phase,
        )
        spine_xy[phase] = (
            source_east + gain * (template_east - source_east),
            source_north + gain * (template_north - source_north),
        )

    for mission_idx, mission in enumerate(missions):
        outer_areas = _outer_mission_areas(mission)
        if not outer_areas:
            continue
        group_points = [
            point
            for area in outer_areas
            for point in (area.get("points") or [])
        ]
        if group_points:
            spine_xy[phase_profile[mission_idx]] = _project(
                _mean_point(group_points),
                layout_origin,
            )
    semantic_spine = [
        {
            "phase": round(float(phase), 8),
            "point": _unproject(east, north, layout_origin),
        }
        for phase, (east, north) in sorted(spine_xy.items())
    ]

    return {
        "localVariationGain": round(gain, 3),
        "semanticRouteSpanM": round(route_span_m, 1),
        "semanticLateralAmplitudeM": round(lateral_amplitude_m, 1),
        "minimumBranchCenterSpacingM": round(min(branch_spacing_m), 1) if branch_spacing_m else None,
        "missionCenterOffsets": center_diagnostics,
        "maxMissionCenterOffsetM": max(
            (float(row["distanceM"]) for row in center_diagnostics),
            default=0.0,
        ),
        "_semanticSpine": semantic_spine,
        "_reverseBranchReturn": bool(
            int(package_type) == 3
            and recipe.turn_style in ("alternating_left", "alternating_right")
        ),
    }


def _mission_terminal(mission: dict[str, Any]) -> dict[str, float] | None:
    lines = mission.get("lineList") or []
    for line in reversed(lines):
        points = line.get("points") or []
        if points:
            return points[-1]
    areas = mission.get("areaList") or []
    for area in reversed(areas):
        points = area.get("points") or []
        if points:
            return _mean_point(points)
    coordinates = mission.get("coordinateList") or []
    return coordinates[-1] if coordinates else None


def _normalize_branch_missions(
    missions: list[dict[str, Any]],
    package_type: int,
) -> int | None:
    """Type 2/3 분기의 Line-Area-후속 Line 개수를 최신 0201 계약에 맞춘다."""

    indexes = {2: (3, 4, 5), 3: (4, 5, 6)}.get(package_type)
    if indexes is None:
        return None
    entry_idx, area_idx, exit_idx = indexes
    entry_mission = missions[entry_idx]
    area_mission = missions[area_idx]
    exit_mission = missions[exit_idx]
    areas = [area for area in (area_mission.get("areaList") or []) if not area.get("isHole")][:3]
    if not areas:
        raise ValueError(f"Type {package_type} 분기 Area가 없습니다.")
    area_mission["areaList"] = areas
    branch_count = len(areas)

    entry_lines = list(entry_mission.get("lineList") or [])
    entry_template = entry_lines[0] if entry_lines else {"width": 1000, "points": []}
    while len(entry_lines) < branch_count:
        polygon = areas[len(entry_lines)].get("points") or []
        entry_lines.append(
            {
                "width": int(entry_template.get("width") or 1000),
                "points": [_mean_point(polygon), _mean_point(polygon)],
            }
        )
    entry_mission["lineList"] = entry_lines[:branch_count]

    existing_exit_lines = list(exit_mission.get("lineList") or [])
    following = missions[exit_idx + 1] if exit_idx + 1 < len(missions) else {}
    following_lines = following.get("lineList") or []
    common_destination = None
    if following_lines and (following_lines[0].get("points") or []):
        common_destination = following_lines[0]["points"][0]
    if common_destination is None and existing_exit_lines:
        points = existing_exit_lines[0].get("points") or []
        common_destination = points[-1] if points else None
    if common_destination is None:
        common_destination = _offset(_mean_point(areas[-1]["points"]), 0.0, 3_500.0)

    normalized_exit_lines: list[dict[str, Any]] = []
    for branch_idx, area in enumerate(areas):
        source = existing_exit_lines[min(branch_idx, len(existing_exit_lines) - 1)] if existing_exit_lines else {}
        source_points = source.get("points") or []
        destination = source_points[-1] if len(existing_exit_lines) >= branch_count and source_points else common_destination
        lateral = (branch_idx - (branch_count - 1) / 2.0) * 220.0
        destination = _offset(destination, lateral, 0.0)
        polygon = area.get("points") or []
        normalized_exit_lines.append(
            {
                "width": int(source.get("width") or entry_template.get("width") or 1000),
                "points": [_exit_toward(polygon, destination), destination],
            }
        )
    exit_mission["lineList"] = normalized_exit_lines
    return branch_count


def _clip_transition_lines(missions: list[dict[str, Any]]) -> list[dict[str, int]]:
    pairs: list[dict[str, int]] = []
    for mission_idx in range(len(missions) - 1):
        mission = missions[mission_idx]
        next_mission = missions[mission_idx + 1]
        lines = mission.get("lineList") or []
        areas = next_mission.get("areaList") or []
        if not lines or not areas:
            continue
        candidate_areas = [area for area in areas if not area.get("isHole")] or areas
        previous = missions[mission_idx - 1] if mission_idx > 0 else None
        for line_idx, line in enumerate(lines):
            polygon_idx = min(line_idx, len(candidate_areas) - 1)
            polygon = candidate_areas[polygon_idx].get("points") or []
            if len(polygon) < 3:
                continue
            original_points = line.get("points") or []
            start = original_points[0] if original_points else None
            if previous is not None:
                previous_areas = previous.get("areaList") or []
                if previous_areas:
                    previous_polygon = previous_areas[-1].get("points") or []
                    if len(previous_polygon) >= 3:
                        start = _polygon_boundary_toward(previous_polygon, _mean_point(polygon))
                else:
                    start = _mission_terminal(previous) or start
            if start is None:
                continue
            endpoint = _polygon_boundary_toward(polygon, start)
            line["points"] = [start, endpoint]
            pairs.append(
                {
                    "lineMissionID": mission_idx + 1,
                    "lineIndex": line_idx,
                    "areaMissionID": mission_idx + 2,
                    "areaIndex": areas.index(candidate_areas[polygon_idx]),
                }
            )
    return pairs


def _set_line_endpoint(
    line: dict[str, Any],
    endpoint_index: int,
    point: dict[str, float],
) -> None:
    points = list(line.get("points") or [])
    if not points:
        points = [dict(point), dict(point)]
    elif len(points) == 1:
        points = [dict(points[0]), dict(points[0])]
    points[0 if endpoint_index == 0 else -1] = dict(point)
    line["points"] = points


def _reshape_line_between(
    line: dict[str, Any],
    start: dict[str, float],
    end: dict[str, float],
) -> None:
    old_points = list(line.get("points") or [])
    if len(old_points) <= 2:
        line["points"] = [dict(start), dict(end)]
        return
    old_start, old_end = old_points[0], old_points[-1]
    old_east, old_north = _project(old_end, old_start)
    old_length = math.hypot(old_east, old_north)
    new_east, new_north = _project(end, start)
    new_length = math.hypot(new_east, new_north)
    if old_length <= 1e-6 or new_length <= 1e-6:
        line["points"] = [
            _offset(start, new_east * index / (len(old_points) - 1), new_north * index / (len(old_points) - 1))
            for index in range(len(old_points))
        ]
        line["points"][0] = dict(start)
        line["points"][-1] = dict(end)
        return
    old_unit_east, old_unit_north = old_east / old_length, old_north / old_length
    new_unit_east, new_unit_north = new_east / new_length, new_north / new_length
    rebuilt = [dict(start)]
    for point in old_points[1:-1]:
        east, north = _project(point, old_start)
        fraction = max(0.0, min(1.0, (east * old_unit_east + north * old_unit_north) / old_length))
        lateral_ratio = (-east * old_unit_north + north * old_unit_east) / old_length
        lateral_ratio = max(-0.15, min(0.15, lateral_ratio))
        rebuilt.append(
            _offset(
                start,
                new_east * fraction - new_unit_north * lateral_ratio * new_length,
                new_north * fraction + new_unit_east * lateral_ratio * new_length,
            )
        )
    rebuilt.append(dict(end))
    line["points"] = rebuilt


def _outer_mission_areas(mission: dict[str, Any]) -> list[dict[str, Any]]:
    areas = mission.get("areaList") or []
    return [area for area in areas if not area.get("isHole")] or list(areas)


def _reroute_branch_return_tail(
    missions: list[dict[str, Any]],
    package_type: int,
    route_return_anchor: dict[str, float] | None,
) -> bool:
    """Route the post-branch tail around the occupied mission envelope.

    A semantic skeleton can place its return lobe close to a wide branch fan.
    The old endpoint repair then drew one long chord through the fan.  Four
    inexpensive perimeter candidates are sufficient here and mirror the
    occupied-geometry rejection used by Random_mission.
    """

    indexes = {2: (5, 6, 7), 3: (6, 7, 8)}.get(int(package_type))
    if indexes is None or route_return_anchor is None:
        return False
    branch_exit_idx, first_tail_idx, final_tail_idx = indexes
    branch_exit_lines = missions[branch_exit_idx].get("lineList") or []
    first_tail_lines = missions[first_tail_idx].get("lineList") or []
    final_tail_lines = missions[final_tail_idx].get("lineList") or []
    if not branch_exit_lines or not first_tail_lines or not final_tail_lines:
        return False
    merge = dict(branch_exit_lines[0]["points"][-1])
    end = dict(route_return_anchor)

    obstacle_lines = [
        line.get("points") or []
        for mission_idx, mission in enumerate(missions)
        if mission_idx not in (first_tail_idx, final_tail_idx)
        for line in mission.get("lineList") or []
    ]
    obstacle_areas = [
        area.get("points") or []
        for mission in missions
        for area in mission.get("areaList") or []
        if not area.get("isHole")
    ]
    occupied_points = [
        point
        for points in [*obstacle_lines, *obstacle_areas]
        for point in points
    ]
    if not occupied_points:
        return False
    origin = _mean_point([*occupied_points, merge, end])
    occupied_xy = [_project(point, origin) for point in occupied_points]
    min_east = min(point[0] for point in occupied_xy)
    max_east = max(point[0] for point in occupied_xy)
    min_north = min(point[1] for point in occupied_xy)
    max_north = max(point[1] for point in occupied_xy)
    merge_east, merge_north = _project(merge, origin)
    end_east, end_north = _project(end, origin)
    margin_m = 1_500.0

    raw_candidates = [
        ((min_east - margin_m, merge_north), (min_east - margin_m, end_north)),
        ((max_east + margin_m, merge_north), (max_east + margin_m, end_north)),
        ((merge_east, min_north - margin_m), (end_east, min_north - margin_m)),
        ((merge_east, max_north + margin_m), (end_east, max_north + margin_m)),
    ]
    candidates: list[list[dict[str, float]]] = []
    for first_xy, second_xy in raw_candidates:
        candidate = [
            merge,
            _unproject(*first_xy, origin),
            _unproject(*second_xy, origin),
            end,
        ]
        if any(
            _polyline_penetrates_polygon(candidate, polygon)
            for polygon in obstacle_areas
        ):
            continue
        if any(
            _polyline_crosses_polyline(candidate, line)
            for line in obstacle_lines
            if len(line) >= 2
        ):
            continue
        if _polyline_crosses_polyline(candidate, candidate, same_line=True):
            continue
        if distance_m(candidate[0], candidate[1]) < 800.0:
            continue
        if distance_m(candidate[2], candidate[3]) < 800.0:
            continue
        candidates.append(candidate)
    if not candidates:
        return False
    selected = min(
        candidates,
        key=lambda candidate: sum(
            distance_m(candidate[index], candidate[index + 1])
            for index in range(len(candidate) - 1)
        ),
    )
    first_tail_lines[0]["points"] = [
        dict(selected[0]),
        dict(selected[1]),
        dict(selected[2]),
    ]
    final_tail_lines[0]["points"] = [dict(selected[2]), dict(selected[3])]
    return True


def _reconnect_mission_topology(
    missions: list[dict[str, Any]],
    package_type: int,
    *,
    route_start_anchor: dict[str, float] | None = None,
    route_return_anchor: dict[str, float] | None = None,
    semantic_spine: Sequence[dict[str, Any]] = (),
    reverse_branch_return: bool = False,
) -> list[dict[str, Any]]:
    """Build corridor edges from the package topology and Area portals.

    This mirrors ``Random_mission.generate_composite_route``: Area geometry is
    fixed first, ingress/egress portals are chosen on its boundary (different
    edges when both exist), and only then are Line missions routed.  Type 1's
    mission 4 -> 5 operational gap remains intentionally disconnected.
    """

    package_type = int(package_type)
    diagnostics: list[dict[str, Any]] = []
    line_phase_ranges = ROUTE_LINE_PHASE_RANGES[package_type]
    branch_indexes = {2: (3, 4, 5), 3: (4, 5, 6)}.get(package_type)
    branch_line_missions = set()
    if branch_indexes is not None:
        branch_line_missions = {branch_indexes[0], branch_indexes[2]}

    # Start every ordinary corridor as the exact phase slice of one simple
    # skeleton.  Branch edges are initialized separately because each must fan
    # to its own Area rather than overlap on the common centerline.
    if semantic_spine:
        for mission_idx, phase_range in line_phase_ranges.items():
            lines = missions[mission_idx].get("lineList") or []
            if not lines:
                continue
            path = _semantic_subpath(semantic_spine, *phase_range)
            if len(path) < 2:
                continue
            if mission_idx not in branch_line_missions:
                for line in lines:
                    line["points"] = [dict(point) for point in path]
                continue
            branch_area_idx = branch_indexes[1] if branch_indexes else -1
            branch_areas = _outer_mission_areas(missions[branch_area_idx])
            for line_idx, line in enumerate(lines):
                area = branch_areas[min(line_idx, len(branch_areas) - 1)]
                area_center = _mean_point(area.get("points") or [])
                if mission_idx == branch_indexes[0]:
                    line["points"] = [dict(path[0]), area_center]
                else:
                    line["points"] = [area_center, dict(path[-1])]

    endpoints: dict[tuple[int, int], list[dict[str, float]]] = {}
    for mission_idx, mission in enumerate(missions):
        for line_idx, line in enumerate(mission.get("lineList") or []):
            points = line.get("points") or []
            if points:
                endpoints[(mission_idx, line_idx)] = [dict(points[0]), dict(points[-1])]

    first_key = (0, 0)
    final_key = (len(missions) - 1, 0)
    if route_start_anchor is not None and first_key in endpoints:
        endpoints[first_key][0] = dict(route_start_anchor)
    if route_return_anchor is not None and final_key in endpoints:
        endpoints[final_key][1] = dict(route_return_anchor)

    direct_pairs = {
        1: ((0, 1), (4, 5)),
        2: ((0, 1), (6, 7)),
        3: ((0, 1), (1, 2), (7, 8)),
        4: ((4, 5), (5, 6)),
        5: ((4, 5), (5, 6)),
    }.get(package_type, ())
    for previous_idx, next_idx in direct_pairs:
        previous_key, next_key = (previous_idx, 0), (next_idx, 0)
        if previous_key not in endpoints or next_key not in endpoints:
            continue
        junction = _mean_point((endpoints[previous_key][1], endpoints[next_key][0]))
        endpoints[previous_key][1] = dict(junction)
        endpoints[next_key][0] = dict(junction)

    branch_merge = {2: (5, 6), 3: (6, 7)}.get(package_type)
    if branch_merge is not None:
        branch_exit_idx, return_idx = branch_merge
        return_key = (return_idx, 0)
        if return_key in endpoints:
            branch_area_idx = 4 if package_type == 2 else 5
            branch_areas = _outer_mission_areas(missions[branch_area_idx])
            upstream_areas = _outer_mission_areas(missions[branch_area_idx - 2])
            branch_points = [
                point
                for area in branch_areas
                for point in (area.get("points") or [])
            ]
            upstream_points = [
                point
                for area in upstream_areas
                for point in (area.get("points") or [])
            ]
            merge = dict(endpoints[return_key][0])
            if branch_points and upstream_points:
                branch_center = _mean_point(branch_points)
                upstream_center = _mean_point(upstream_points)
                forward_east, forward_north = _project(branch_center, upstream_center)
                forward_norm = math.hypot(forward_east, forward_north)
                if forward_norm <= 1e-6:
                    forward_east, forward_north = _project(merge, branch_center)
                    forward_norm = math.hypot(forward_east, forward_north)
                if forward_norm <= 1e-6:
                    forward_east, forward_north, forward_norm = 1.0, 0.0, 1.0
                forward_east /= forward_norm
                forward_north /= forward_norm
                return_line = missions[return_idx]["lineList"][0]
                old_return_points = list(return_line.get("points") or [])
                if reverse_branch_return and package_type == 3:
                    lateral_east, lateral_north = -forward_north, forward_east
                    occupied_points = [
                        point
                        for mission_idx, mission in enumerate(missions)
                        if mission_idx <= branch_area_idx
                        for point in _mission_geometry_points(mission)
                    ]
                    local_coordinates = []
                    for point in occupied_points:
                        east, north = _project(point, branch_center)
                        local_coordinates.append(
                            (
                                east * lateral_east + north * lateral_north,
                                east * forward_east + north * forward_north,
                            )
                        )
                    min_lateral = min((row[0] for row in local_coordinates), default=-4_000.0)
                    max_lateral = max((row[0] for row in local_coordinates), default=4_000.0)
                    min_forward = min((row[1] for row in local_coordinates), default=-4_000.0)
                    max_forward = max((row[1] for row in local_coordinates), default=4_000.0)
                    merge_forward = min_forward - 1_500.0

                    def local_point(lateral_m: float, forward_m: float) -> dict[str, float]:
                        return _offset(
                            branch_center,
                            lateral_east * lateral_m + forward_east * forward_m,
                            lateral_north * lateral_m + forward_north * forward_m,
                        )

                    merge = local_point(0.0, merge_forward)
                    exit_lines = missions[branch_exit_idx].get("lineList") or []
                    for line_idx, (area, line) in enumerate(zip(branch_areas, exit_lines)):
                        area_center = _mean_point(area.get("points") or [])
                        center_east, center_north = _project(area_center, branch_center)
                        center_lateral = (
                            center_east * lateral_east + center_north * lateral_north
                        )
                        side_sign = -1.0 if line_idx < len(exit_lines) / 2.0 else 1.0
                        side_lateral = (
                            min_lateral - 1_500.0 - line_idx * 350.0
                            if side_sign < 0.0
                            else max_lateral + 1_500.0 + line_idx * 350.0
                        )
                        downstream_forward = max_forward + 1_500.0
                        line["points"] = [
                            area_center,
                            local_point(center_lateral, downstream_forward),
                            local_point(side_lateral, downstream_forward),
                            local_point(side_lateral, merge_forward),
                            merge,
                        ]
                    return_line["points"] = [
                        merge,
                        old_return_points[-1] if old_return_points else merge,
                    ]
                else:
                    maximum_forward_extent_m = max(
                        (
                            _project(point, branch_center)[0] * forward_east
                            + _project(point, branch_center)[1] * forward_north
                            for point in branch_points
                        ),
                        default=0.0,
                    )
                    requested_east, requested_north = _project(merge, branch_center)
                    requested_forward_m = (
                        requested_east * forward_east + requested_north * forward_north
                    )
                    merge_distance_m = max(
                        maximum_forward_extent_m + 1_500.0,
                        requested_forward_m,
                    )
                    merge = _offset(
                        branch_center,
                        forward_east * merge_distance_m,
                        forward_north * merge_distance_m,
                    )
                    outward = _offset(
                        merge,
                        forward_east * 1_000.0,
                        forward_north * 1_000.0,
                    )
                    return_line["points"] = [merge, outward, *old_return_points[1:]]
            endpoints[return_key][0] = merge
            _set_line_endpoint(missions[return_idx]["lineList"][0], 0, merge)
            for line_idx, _line in enumerate(missions[branch_exit_idx].get("lineList") or []):
                key = (branch_exit_idx, line_idx)
                if key in endpoints:
                    endpoints[key][1] = dict(merge)
                    _set_line_endpoint(
                        missions[branch_exit_idx]["lineList"][line_idx],
                        -1,
                        merge,
                    )

    # (Area mission, incoming Line mission, outgoing Line mission).  A single
    # Area may fan to several lines; branch Area lists use matching indexes.
    area_contracts = {
        1: ((2, 1, None),),
        2: ((2, 1, 3), (4, 3, 5)),
        3: ((3, 2, 4), (5, 4, 6)),
        4: ((1, 0, 2), (3, 2, 4)),
        5: ((1, 0, 2), (3, 2, 4)),
    }.get(package_type, ())
    portal_records: list[dict[str, Any]] = []
    for area_mission_idx, incoming_mission_idx, outgoing_mission_idx in area_contracts:
        areas = _outer_mission_areas(missions[area_mission_idx])
        incoming_lines = missions[incoming_mission_idx].get("lineList") or []
        outgoing_lines = (
            missions[outgoing_mission_idx].get("lineList") or []
            if outgoing_mission_idx is not None
            else []
        )
        for area_idx, area in enumerate(areas):
            polygon = area.get("points") or []
            if len(polygon) < 3:
                continue
            incoming_line_idx = min(area_idx, len(incoming_lines) - 1)
            incoming_key = (incoming_mission_idx, incoming_line_idx)
            entry_anchor = None
            if incoming_key in endpoints:
                incoming_line = missions[incoming_mission_idx]["lineList"][incoming_line_idx]
                incoming_points = incoming_line.get("points") or []
                clipped_incoming = _trim_polyline_to_area_boundary(
                    incoming_points,
                    polygon,
                    entering=True,
                )
                if len(clipped_incoming) >= 2:
                    # Keep the public composite transition contract used by
                    # the GUI/exporter: an Area-ingress corridor is one
                    # start/end segment whose endpoint lies on the boundary.
                    incoming_line["points"] = [
                        dict(clipped_incoming[0]),
                        dict(clipped_incoming[-1]),
                    ]
                    entry_anchor = dict(clipped_incoming[-1])
                else:
                    entry_anchor = _polygon_boundary_toward(
                        polygon,
                        endpoints[incoming_key][0],
                    )
                endpoints[incoming_key][1] = dict(entry_anchor)

            relevant_outgoing = []
            if outgoing_mission_idx is not None:
                if len(areas) == 1:
                    relevant_outgoing = list(enumerate(outgoing_lines))
                elif area_idx < len(outgoing_lines):
                    relevant_outgoing = [(area_idx, outgoing_lines[area_idx])]
            exit_anchors: list[dict[str, float]] = []
            for outgoing_line_idx, outgoing_line in relevant_outgoing:
                outgoing_key = (outgoing_mission_idx, outgoing_line_idx)
                if outgoing_key not in endpoints:
                    continue
                outgoing_points = outgoing_line.get("points") or []
                clipped_outgoing = _trim_polyline_to_area_boundary(
                    outgoing_points,
                    polygon,
                    entering=False,
                )
                if len(clipped_outgoing) >= 2:
                    outgoing_line["points"] = clipped_outgoing
                    exit_anchor = dict(clipped_outgoing[0])
                else:
                    exit_anchor = _polygon_boundary_toward(
                        polygon,
                        endpoints[outgoing_key][1],
                    )
                if (
                    entry_anchor is not None
                    and _nearest_polygon_edge_index(exit_anchor, polygon)
                    == _nearest_polygon_edge_index(entry_anchor, polygon)
                ):
                    toward_next = (
                        clipped_outgoing[1]
                        if len(clipped_outgoing) >= 2
                        else endpoints[outgoing_key][1]
                    )
                    exit_anchor = _polygon_exit_anchor(
                        polygon,
                        toward_next,
                        entry_anchor,
                    )
                    outside_portal = _outside_area_portal(polygon, exit_anchor)
                    rebuilt_points = [dict(exit_anchor), outside_portal]
                    rebuilt_points.extend(
                        dict(point)
                        for point in clipped_outgoing[1:]
                        if distance_m(outside_portal, point) > 1.0
                    )
                    if len(rebuilt_points) < 2:
                        rebuilt_points.append(dict(endpoints[outgoing_key][1]))
                    outgoing_line["points"] = rebuilt_points
                endpoints[outgoing_key][0] = dict(exit_anchor)
                exit_anchors.append(exit_anchor)
            portal_records.append(
                {
                    "areaMissionID": area_mission_idx + 1,
                    "areaIndex": area_idx,
                    "entry": entry_anchor,
                    "exits": exit_anchors,
                    "polygon": polygon,
                }
            )

    # Preserve the reference Type-1 gap after the disconnected Area mission.
    if package_type == 1 and (4, 0) in endpoints:
        separated_areas = _outer_mission_areas(missions[3])
        if separated_areas:
            polygon = separated_areas[0].get("points") or []
            return_start = endpoints[(4, 0)][0]
            if len(polygon) >= 3 and point_to_polygon_boundary_m(return_start, polygon) < 2_500.0:
                center = _mean_point(polygon)
                boundary = _polygon_boundary_toward(polygon, return_start)
                east, north = _project(boundary, center)
                norm = math.hypot(east, north)
                if norm <= 1e-6:
                    east, north = _project(endpoints[(4, 0)][1], center)
                    norm = math.hypot(east, north)
                if norm <= 1e-6:
                    east, north, norm = 1.0, 0.0, 1.0
                endpoints[(4, 0)][0] = _offset(
                    boundary,
                    east / norm * 2_500.0,
                    north / norm * 2_500.0,
                )

    for (mission_idx, line_idx), (start, end) in endpoints.items():
        # The line already follows the semantic skeleton.  Moving only its
        # two portals clips that path at the Area boundary; re-affining all
        # intermediate points against the new chord can fold a valid route
        # back across an earlier corridor.
        line = missions[mission_idx]["lineList"][line_idx]
        _set_line_endpoint(line, 0, start)
        _set_line_endpoint(line, -1, end)

    if branch_merge is not None:
        _reroute_branch_return_tail(
            missions,
            package_type,
            route_return_anchor,
        )

    for previous_idx, next_idx in direct_pairs:
        previous = missions[previous_idx]["lineList"][0]["points"][-1]
        following = missions[next_idx]["lineList"][0]["points"][0]
        diagnostics.append(
            {
                "kind": "lineToLine",
                "fromMissionID": previous_idx + 1,
                "toMissionID": next_idx + 1,
                "errorM": round(distance_m(previous, following), 3),
            }
        )
    for record in portal_records:
        polygon = record["polygon"]
        entry_anchor = record["entry"]
        if entry_anchor is not None:
            diagnostics.append(
                {
                    "kind": "lineToArea",
                    "areaMissionID": record["areaMissionID"],
                    "areaIndex": record["areaIndex"],
                    "errorM": round(point_to_polygon_boundary_m(entry_anchor, polygon), 3),
                }
            )
        for exit_anchor in record["exits"]:
            diagnostics.append(
                {
                    "kind": "areaToLine",
                    "areaMissionID": record["areaMissionID"],
                    "areaIndex": record["areaIndex"],
                    "errorM": round(point_to_polygon_boundary_m(exit_anchor, polygon), 3),
                }
            )
            if entry_anchor is not None:
                diagnostics.append(
                    {
                        "kind": "areaPortalEdge",
                        "areaMissionID": record["areaMissionID"],
                        "areaIndex": record["areaIndex"],
                        "entryEdge": _nearest_polygon_edge_index(entry_anchor, polygon),
                        "exitEdge": _nearest_polygon_edge_index(exit_anchor, polygon),
                        "errorM": 0.0,
                    }
                )
    if branch_merge is not None:
        branch_idx, return_idx = branch_merge
        merge = missions[return_idx]["lineList"][0]["points"][0]
        for line_idx, line in enumerate(missions[branch_idx].get("lineList") or []):
            diagnostics.append(
                {
                    "kind": "branchMerge",
                    "fromMissionID": branch_idx + 1,
                    "toMissionID": return_idx + 1,
                    "lineIndex": line_idx,
                    "errorM": round(distance_m(line["points"][-1], merge), 3),
                }
            )
    if route_start_anchor is not None and first_key in endpoints:
        diagnostics.append(
            {
                "kind": "routeStartAnchor",
                "missionID": 1,
                "errorM": round(distance_m(missions[0]["lineList"][0]["points"][0], route_start_anchor), 3),
            }
        )
    if route_return_anchor is not None and final_key in endpoints:
        diagnostics.append(
            {
                "kind": "routeReturnAnchor",
                "missionID": len(missions),
                "errorM": round(distance_m(missions[-1]["lineList"][0]["points"][-1], route_return_anchor), 3),
            }
        )
    return diagnostics


def _point_segment_distance_m(
    point: dict[str, float],
    start: dict[str, float],
    end: dict[str, float],
) -> float:
    origin = start
    px, py = _project(point, origin)
    ex, ey = _project(end, origin)
    denominator = ex * ex + ey * ey
    fraction = 0.0 if denominator <= 1e-12 else (px * ex + py * ey) / denominator
    fraction = max(0.0, min(1.0, fraction))
    return math.hypot(px - ex * fraction, py - ey * fraction)


def _segment_relation(
    a: dict[str, float],
    b: dict[str, float],
    c: dict[str, float],
    d: dict[str, float],
) -> str:
    """Classify two metre-frame segments as none/point/proper/overlap."""

    origin = _mean_point((a, b, c, d))
    ax, ay = _project(a, origin)
    bx, by = _project(b, origin)
    cx, cy = _project(c, origin)
    dx, dy = _project(d, origin)

    def orient(
        p: tuple[float, float],
        q: tuple[float, float],
        r: tuple[float, float],
    ) -> float:
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])

    pa, pb, pc, pd = (ax, ay), (bx, by), (cx, cy), (dx, dy)
    o1, o2 = orient(pa, pb, pc), orient(pa, pb, pd)
    o3, o4 = orient(pc, pd, pa), orient(pc, pd, pb)
    epsilon = 1e-5
    if o1 * o2 < -epsilon and o3 * o4 < -epsilon:
        return "proper"
    collinear = all(abs(value) <= epsilon for value in (o1, o2, o3, o4))
    if collinear:
        use_east = abs(bx - ax) >= abs(by - ay)
        left = sorted((ax, bx)) if use_east else sorted((ay, by))
        right = sorted((cx, dx)) if use_east else sorted((cy, dy))
        overlap_m = min(left[1], right[1]) - max(left[0], right[0])
        if overlap_m > 0.5:
            return "overlap"
        if overlap_m >= -0.5:
            return "point"
        return "none"
    if (
        _point_segment_distance_m(a, c, d) <= 0.5
        or _point_segment_distance_m(b, c, d) <= 0.5
        or _point_segment_distance_m(c, a, b) <= 0.5
        or _point_segment_distance_m(d, a, b) <= 0.5
    ):
        return "point"
    return "none"


def _segments_share_endpoint(
    a: dict[str, float],
    b: dict[str, float],
    c: dict[str, float],
    d: dict[str, float],
    tolerance_m: float = 1.0,
) -> bool:
    return any(
        distance_m(left, right) <= tolerance_m
        for left in (a, b)
        for right in (c, d)
    )


def _polyline_crosses_polyline(
    left: Sequence[dict[str, float]],
    right: Sequence[dict[str, float]],
    *,
    same_line: bool = False,
) -> bool:
    for left_index in range(len(left) - 1):
        for right_index in range(len(right) - 1):
            if same_line and abs(left_index - right_index) <= 1:
                continue
            a, b = left[left_index], left[left_index + 1]
            c, d = right[right_index], right[right_index + 1]
            relation = _segment_relation(a, b, c, d)
            if relation in ("proper", "overlap"):
                return True
            if relation == "point" and not _segments_share_endpoint(a, b, c, d):
                return True
    return False


def _polyline_penetrates_polygon(
    points: Sequence[dict[str, float]],
    polygon: Sequence[dict[str, float]],
) -> bool:
    if len(points) < 2 or len(polygon) < 3:
        return False
    # Eight-decimal lat/lon serialization introduces only millimetres of
    # uncertainty.  Keep a 5 mm numerical allowance, but reject even shallow
    # corridor slivers through an unrelated Area.
    boundary_tolerance_m = 0.005
    global_endpoints = (points[0], points[-1])
    for point in points:
        if (
            point_in_polygon(point, polygon)
            and point_to_polygon_boundary_m(point, polygon) > boundary_tolerance_m
        ):
            return True
    for segment_index in range(len(points) - 1):
        start, end = points[segment_index], points[segment_index + 1]
        midpoint = _mean_point((start, end))
        if (
            point_in_polygon(midpoint, polygon)
            and point_to_polygon_boundary_m(midpoint, polygon) > boundary_tolerance_m
        ):
            return True
        for _fraction, hit in _segment_polygon_intersections(start, end, polygon):
            if (
                min(distance_m(hit, endpoint) for endpoint in global_endpoints)
                > boundary_tolerance_m
            ):
                return True
    return False


def _polygons_overlap(
    left: Sequence[dict[str, float]],
    right: Sequence[dict[str, float]],
) -> bool:
    if len(left) < 3 or len(right) < 3:
        return False
    for left_index in range(len(left)):
        for right_index in range(len(right)):
            relation = _segment_relation(
                left[left_index],
                left[(left_index + 1) % len(left)],
                right[right_index],
                right[(right_index + 1) % len(right)],
            )
            if relation in ("proper", "overlap"):
                return True
    return any(
        point_in_polygon(point, right)
        and point_to_polygon_boundary_m(point, right) > 0.5
        for point in left
    ) or any(
        point_in_polygon(point, left)
        and point_to_polygon_boundary_m(point, left) > 0.5
        for point in right
    )


def _polygon_self_intersects(polygon: Sequence[dict[str, float]]) -> bool:
    if len(polygon) < 4:
        return False
    origin = _mean_point(polygon)
    vertices = [_project(point, origin) for point in polygon]

    def orientation(
        left: tuple[float, float],
        middle: tuple[float, float],
        right: tuple[float, float],
    ) -> float:
        return (middle[0] - left[0]) * (right[1] - left[1]) - (
            middle[1] - left[1]
        ) * (right[0] - left[0])

    def crosses(
        a: tuple[float, float],
        b: tuple[float, float],
        c: tuple[float, float],
        d: tuple[float, float],
    ) -> bool:
        ab_c = orientation(a, b, c)
        ab_d = orientation(a, b, d)
        cd_a = orientation(c, d, a)
        cd_b = orientation(c, d, b)
        return ab_c * ab_d < -1e-6 and cd_a * cd_b < -1e-6

    edge_count = len(vertices)
    for left_idx in range(edge_count):
        left_next = (left_idx + 1) % edge_count
        for right_idx in range(left_idx + 1, edge_count):
            right_next = (right_idx + 1) % edge_count
            if left_idx == right_idx or left_next == right_idx or right_next == left_idx:
                continue
            if crosses(
                vertices[left_idx],
                vertices[left_next],
                vertices[right_idx],
                vertices[right_next],
            ):
                return True
    return False


def _mission_layout_error(
    missions: Sequence[dict[str, Any]],
    package_type: int,
    *,
    minimum_line_length_m: float = 800.0,
    topology_connections: Sequence[dict[str, Any]] = (),
    clipped_pairs: Sequence[dict[str, int]] = (),
) -> str | None:
    for mission_idx, mission in enumerate(missions):
        for line_idx, line in enumerate(mission.get("lineList") or []):
            points = line.get("points") or []
            if len(points) < 2:
                return f"mission {mission_idx + 1} line {line_idx + 1} has fewer than two points"
            total_length_m = sum(
                distance_m(points[index], points[index + 1])
                for index in range(len(points) - 1)
            )
            if total_length_m < minimum_line_length_m:
                return (
                    f"mission {mission_idx + 1} line {line_idx + 1} is "
                    f"{total_length_m:.1f}m"
                )
        areas = mission.get("areaList") or []
        for area_idx, area in enumerate(areas):
            polygon = area.get("points") or []
            if len(polygon) < 3 or _polygon_self_intersects(polygon):
                return f"mission {mission_idx + 1} area {area_idx + 1} is invalid"
        outers = [area for area in areas if not area.get("isHole")]
        holes = [area for area in areas if area.get("isHole")]
        if holes and not outers:
            return f"mission {mission_idx + 1} has a hole without an outer polygon"
        for hole in holes:
            if not all(
                point_in_polygon(point, outers[0].get("points") or [])
                for point in (hole.get("points") or [])
            ):
                return f"mission {mission_idx + 1} hole is outside its outer polygon"

    # Composite-route hard rules: only declared endpoint contact is allowed.
    # Every other Line-Line crossing, Line-Area penetration and outer-Area
    # overlap rejects the candidate so the bounded layout retry can resample.
    all_lines = [
        (mission_idx, line_idx, line.get("points") or [])
        for mission_idx, mission in enumerate(missions)
        for line_idx, line in enumerate(mission.get("lineList") or [])
    ]
    outer_areas = [
        (mission_idx, area_idx, area.get("points") or [])
        for mission_idx, mission in enumerate(missions)
        for area_idx, area in enumerate(mission.get("areaList") or [])
        if not area.get("isHole")
    ]
    for mission_idx, line_idx, points in all_lines:
        if _polyline_crosses_polyline(points, points, same_line=True):
            return f"mission {mission_idx + 1} line {line_idx + 1} self-crosses"
    for left_index, (left_mission, left_line, left_points) in enumerate(all_lines):
        for right_mission, right_line, right_points in all_lines[left_index + 1 :]:
            if _polyline_crosses_polyline(left_points, right_points):
                return (
                    f"mission {left_mission + 1} line {left_line + 1} crosses "
                    f"mission {right_mission + 1} line {right_line + 1}"
                )
    for line_mission, line_idx, line_points in all_lines:
        for area_mission, area_idx, polygon in outer_areas:
            if _polyline_penetrates_polygon(line_points, polygon):
                return (
                    f"mission {line_mission + 1} line {line_idx + 1} penetrates "
                    f"mission {area_mission + 1} area {area_idx + 1}"
                )
    for left_index, (left_mission, left_area, left_polygon) in enumerate(outer_areas):
        for right_mission, right_area, right_polygon in outer_areas[left_index + 1 :]:
            if _polygons_overlap(left_polygon, right_polygon):
                return (
                    f"mission {left_mission + 1} area {left_area + 1} overlaps "
                    f"mission {right_mission + 1} area {right_area + 1}"
                )

    branch_indexes = {2: (3, 4, 5), 3: (4, 5, 6)}.get(int(package_type))
    if branch_indexes is not None:
        entry_idx, area_idx, exit_idx = branch_indexes
        counts = (
            len(missions[entry_idx].get("lineList") or []),
            len(_outer_mission_areas(missions[area_idx])),
            len(missions[exit_idx].get("lineList") or []),
        )
        if not (counts[0] == counts[1] == counts[2] and counts[0] > 0):
            return f"branch count mismatch: {counts}"
        branch_areas = _outer_mission_areas(missions[area_idx])
        branch_centers = [_mean_point(area["points"]) for area in branch_areas]
        if len(branch_centers) >= 2:
            minimum_spacing_m = min(
                distance_m(branch_centers[left], branch_centers[right])
                for left in range(len(branch_centers))
                for right in range(left + 1, len(branch_centers))
            )
            if minimum_spacing_m < 800.0:
                return f"branch center spacing is only {minimum_spacing_m:.1f}m"
    for connection in topology_connections:
        if float(connection.get("errorM") or 0.0) >= 1.0:
            return f"{connection.get('kind') or 'topology'} connection error"
        if (
            connection.get("kind") == "areaPortalEdge"
            and connection.get("entryEdge") == connection.get("exitEdge")
        ):
            return "area ingress and egress use the same edge"
    for pair in clipped_pairs:
        line = missions[pair["lineMissionID"] - 1]["lineList"][pair["lineIndex"]]
        area = missions[pair["areaMissionID"] - 1]["areaList"][pair["areaIndex"]]
        if point_to_polygon_boundary_m(line["points"][-1], area["points"]) >= 1.0:
            return "line-to-area boundary error"
    if int(package_type) == 1:
        separated_areas = _outer_mission_areas(missions[3])
        return_lines = missions[4].get("lineList") or []
        if separated_areas and return_lines:
            gap_m = point_to_polygon_boundary_m(
                return_lines[0]["points"][0],
                separated_areas[0]["points"],
            )
            if gap_m < 1_500.0:
                return f"type 1 intentional separation is only {gap_m:.1f}m"
    return None


def _expanded_polygon(
    polygon: Sequence[dict[str, float]],
    factor: float,
) -> list[dict[str, float]]:
    center = _mean_point(polygon)
    return [
        _unproject(*(component * factor for component in _project(point, center)), center)
        for point in polygon
    ]


def _fallback_flight_area(
    points: Sequence[dict[str, float]],
    rng: random.Random,
) -> list[dict[str, float]]:
    center = _mean_point(points)
    projected = [_project(point, center) for point in points]
    min_east = min(row[0] for row in projected) - rng.uniform(2_800.0, 4_200.0)
    max_east = max(row[0] for row in projected) + rng.uniform(2_800.0, 4_200.0)
    min_north = min(row[1] for row in projected) - rng.uniform(2_200.0, 3_500.0)
    max_north = max(row[1] for row in projected) + rng.uniform(2_800.0, 4_500.0)
    skew = rng.uniform(-450.0, 450.0)
    return [
        _unproject(min_east, min_north, center),
        _unproject(min_east + skew, max_north, center),
        _unproject(max_east, max_north - skew, center),
        _unproject(max_east - skew, min_north, center),
    ]


def _build_reference_areas(
    mission_ref: dict[str, Any],
    transform: _DeploymentTransform,
    mission_points: Sequence[dict[str, float]],
    formation_points: Sequence[dict[str, float]],
    rng: random.Random,
) -> tuple[list[list[dict[str, float]]], list[list[dict[str, float]]]]:
    raw_flight = _reference_area_lists(mission_ref, "FlightAreaList")
    all_required = [*mission_points, *formation_points]
    flight_areas: list[list[dict[str, float]]] = []
    for raw in raw_flight:
        transformed = transform.points(raw)
        for factor in (rng.uniform(1.05, 1.14), 1.22, 1.38):
            candidate = _expanded_polygon(transformed, factor)
            if all(point_in_polygon(point, candidate) for point in all_required):
                transformed = candidate
                break
        flight_areas.append(transformed)
    if not flight_areas or not all(
        any(point_in_polygon(point, area) for area in flight_areas) for point in all_required
    ):
        flight_areas = [_fallback_flight_area(all_required, rng)]

    prohibited_areas = [
        _deform_polygon(transform.points(raw), rng, mild=True)
        for raw in _reference_area_lists(mission_ref, "ProhibitedAreaList")
    ]
    if not prohibited_areas:
        flight_center = _mean_point(flight_areas[0])
        for direction in (-1.0, 1.0):
            center = _offset(flight_center, direction * rng.uniform(8_000.0, 11_000.0), rng.uniform(1_000.0, 6_000.0))
            radius = rng.uniform(650.0, 1_150.0)
            prohibited_areas.append(
                [
                    _offset(
                        center,
                        radius * math.sin(math.radians(18.0 + idx * 72.0)),
                        radius * math.cos(math.radians(18.0 + idx * 72.0)),
                    )
                    for idx in range(5)
                ]
            )
    return flight_areas, prohibited_areas


def _sample_in_polygon(
    polygon: Sequence[dict[str, float]],
    rng: random.Random,
    *,
    holes: Sequence[Sequence[dict[str, float]]] = (),
) -> dict[str, float] | None:
    center = _mean_point(polygon)
    projected = [_project(point, center) for point in polygon]
    min_east, max_east = min(row[0] for row in projected), max(row[0] for row in projected)
    min_north, max_north = min(row[1] for row in projected), max(row[1] for row in projected)
    for _ in range(300):
        candidate = _unproject(rng.uniform(min_east, max_east), rng.uniform(min_north, max_north), center)
        if point_in_polygon(candidate, polygon) and not any(point_in_polygon(candidate, hole) for hole in holes):
            return candidate
    return center if not any(point_in_polygon(center, hole) for hole in holes) else None


def _sample_on_line(
    lines: Sequence[dict[str, Any]],
    rng: random.Random,
) -> dict[str, float] | None:
    segments: list[tuple[dict[str, float], dict[str, float], float, float]] = []
    for line in lines:
        points = line.get("points") or []
        width = float(line.get("width") or 600.0)
        for idx in range(len(points) - 1):
            length = distance_m(points[idx], points[idx + 1])
            if length > 1.0:
                segments.append((points[idx], points[idx + 1], length, width))
    if not segments:
        return None
    total = sum(row[2] for row in segments)
    pick = rng.uniform(0.0, total)
    selected = segments[-1]
    for segment in segments:
        pick -= segment[2]
        if pick <= 0.0:
            selected = segment
            break
    start, end, _length, width = selected
    origin = start
    east, north = _project(end, origin)
    t = rng.uniform(0.08, 0.92)
    norm = math.hypot(east, north) or 1.0
    lateral = rng.uniform(-0.38, 0.38) * width
    out_east = east * t - north / norm * lateral
    out_north = north * t + east / norm * lateral
    return _unproject(out_east, out_north, origin)


def _far_enough(
    point: dict[str, float],
    existing: Sequence[dict[str, float]],
    minimum_m: float,
) -> bool:
    return all(distance_m(point, old) >= minimum_m for old in existing)


def _primary_area_candidates(
    missions: Sequence[dict[str, Any]],
    profile: ScenarioProfile,
) -> list[tuple[int, int, list[dict[str, float]]]]:
    candidates: list[tuple[int, int, list[dict[str, float]]]] = []
    for mission_id, mission in enumerate(missions, start=1):
        if int(mission.get("regionType") or 0) not in profile.primary_region_types:
            continue
        areas = mission.get("areaList") or []
        preferred = [
            (idx, area)
            for idx, area in enumerate(areas)
            if bool(area.get("isHole")) == profile.prefer_hole
        ]
        if not preferred:
            preferred = list(enumerate(areas))
        for area_idx, area in preferred:
            points = area.get("points") or []
            if len(points) >= 3:
                candidates.append((mission_id, area_idx, points))
    if candidates:
        return candidates
    fallback: list[tuple[int, int, list[dict[str, float]]]] = []
    for mission_id, mission in enumerate(missions, start=1):
        for area_idx, area in enumerate(mission.get("areaList") or []):
            points = area.get("points") or []
            if len(points) >= 3 and not area.get("isHole"):
                fallback.append((mission_id, area_idx, points))
    if not fallback:
        return []
    return [max(fallback, key=lambda row: _mean_point(row[2])["latitude"])]


def _build_targets(
    missions: Sequence[dict[str, Any]],
    takeovers: Sequence[dict[str, float]],
    profile: ScenarioProfile,
    rng: random.Random,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    targets: list[dict[str, Any]] = []
    locations: list[dict[str, float]] = []
    counts_by_mission: dict[int, int] = {}
    primary = _primary_area_candidates(missions, profile)
    if not primary:
        raise ValueError(f"{profile.label} 참조 임무에서 표적 영역을 찾지 못했습니다.")
    primary_total = rng.randint(2, 5)

    def append_target(mission_id: int, point: dict[str, float]) -> None:
        targets.append(
            {
                "targetID": len(targets) + 1,
                "targetType": int(rng.choice(profile.target_type_pool)),
                "inputMissionID": mission_id,
                "location": dict(point),
            }
        )
        locations.append(point)
        counts_by_mission[mission_id] = counts_by_mission.get(mission_id, 0) + 1

    for index in range(primary_total):
        mission_id, _area_idx, polygon = primary[index % len(primary)]
        point = None
        for _ in range(120):
            candidate = _sample_in_polygon(polygon, rng)
            if candidate and _far_enough(candidate, locations, 320.0) and _far_enough(candidate, takeovers, 2_000.0):
                point = candidate
                break
        if point is None:
            point = _sample_in_polygon(polygon, rng) or _mean_point(polygon)
        append_target(mission_id, point)

    primary_mission_ids = {row[0] for row in primary}
    for mission_id, mission in enumerate(missions, start=1):
        if mission_id in primary_mission_ids or rng.random() >= 0.55:
            continue
        areas = [area for area in (mission.get("areaList") or []) if not area.get("isHole")]
        lines = mission.get("lineList") or []
        if not areas and not lines:
            continue
        count = rng.randint(1, 5)
        holes = [area.get("points") or [] for area in (mission.get("areaList") or []) if area.get("isHole")]
        for _ in range(count):
            point = None
            for _attempt in range(100):
                if areas:
                    area = rng.choice(areas)
                    candidate = _sample_in_polygon(area.get("points") or [], rng, holes=holes)
                else:
                    candidate = _sample_on_line(lines, rng)
                if candidate and _far_enough(candidate, locations, 260.0) and _far_enough(candidate, takeovers, 1_500.0):
                    point = candidate
                    break
            if point is not None:
                append_target(mission_id, point)

    metadata = {
        "primaryTargetMissionIDs": sorted(primary_mission_ids),
        "primaryTargetCount": primary_total,
        "targetCountsByMission": {str(key): value for key, value in sorted(counts_by_mission.items())},
    }
    return targets, metadata


def _all_state_mission_points(missions: Sequence[dict[str, Any]]) -> list[dict[str, float]]:
    points: list[dict[str, float]] = []
    for mission in missions:
        for line in mission.get("lineList") or []:
            points.extend(line.get("points") or [])
        for area in mission.get("areaList") or []:
            points.extend(area.get("points") or [])
        points.extend(mission.get("coordinateList") or [])
    return points


def _fit_missions_to_auto_bounds(
    missions: Sequence[dict[str, Any]],
    target_anchor: dict[str, float],
    *,
    inset_m: float = 400.0,
) -> tuple[dict[str, float], dict[str, float]]:
    """Fit the connected mission graph inside the legacy AUTO_MISSION_AREA.

    A single uniform affine transform is applied to every Line/Area/Coordinate
    point.  Consequently all area portals, intentional gaps and non-crossing
    relations established by ``_reconnect_mission_topology`` remain intact.
    Translation is only applied on axes that overflow the configured envelope,
    so random placement variation is retained whenever it already fits.
    """

    mission_points = _all_state_mission_points(missions)
    if not mission_points:
        return dict(target_anchor), {
            "envelopeFitScale": 1.0,
            "envelopeFitEastM": 0.0,
            "envelopeFitNorthM": 0.0,
        }

    sw, ne, _refs, _side, _handover, _rtb = _random_config_values()
    origin = {
        "latitude": (float(sw[0]) + float(ne[0])) / 2.0,
        "longitude": (float(sw[1]) + float(ne[1])) / 2.0,
    }
    sw_east, sw_north = _project(
        {"latitude": float(sw[0]), "longitude": float(sw[1])}, origin
    )
    ne_east, ne_north = _project(
        {"latitude": float(ne[0]), "longitude": float(ne[1])}, origin
    )
    allowed_min_east = min(sw_east, ne_east) + inset_m
    allowed_max_east = max(sw_east, ne_east) - inset_m
    allowed_min_north = min(sw_north, ne_north) + inset_m
    allowed_max_north = max(sw_north, ne_north) - inset_m
    allowed_width = allowed_max_east - allowed_min_east
    allowed_height = allowed_max_north - allowed_min_north
    if allowed_width <= 0.0 or allowed_height <= 0.0:
        raise ValueError("AUTO_MISSION_AREA is too small for its safety inset")

    # The formation is created around target_anchor after this step.  Keeping
    # the anchor itself inside the same 400 m inset also leaves enough room for
    # the legacy 150 m triangle plus 300 m handover/RTB offsets.
    projected = [
        *(_project(point, origin) for point in mission_points),
        _project(target_anchor, origin),
    ]
    min_east = min(row[0] for row in projected)
    max_east = max(row[0] for row in projected)
    min_north = min(row[1] for row in projected)
    max_north = max(row[1] for row in projected)
    width = max(max_east - min_east, 1.0)
    height = max(max_north - min_north, 1.0)
    # Leave a centimetre-scale numerical guard after 8-decimal serialization.
    scale = min(1.0, allowed_width / width, allowed_height / height) * 0.99999
    center_east = (min_east + max_east) / 2.0
    center_north = (min_north + max_north) / 2.0

    scaled_min_east = center_east + (min_east - center_east) * scale
    scaled_max_east = center_east + (max_east - center_east) * scale
    scaled_min_north = center_north + (min_north - center_north) * scale
    scaled_max_north = center_north + (max_north - center_north) * scale

    east_shift_low = allowed_min_east - scaled_min_east
    east_shift_high = allowed_max_east - scaled_max_east
    north_shift_low = allowed_min_north - scaled_min_north
    north_shift_high = allowed_max_north - scaled_max_north
    east_shift = min(max(0.0, east_shift_low), east_shift_high)
    north_shift = min(max(0.0, north_shift_low), north_shift_high)

    def fitted(point: dict[str, float]) -> dict[str, float]:
        east, north = _project(point, origin)
        return _unproject(
            center_east + (east - center_east) * scale + east_shift,
            center_north + (north - center_north) * scale + north_shift,
            origin,
        )

    visited: set[int] = set()
    for point in mission_points:
        identity = id(point)
        if identity in visited:
            continue
        visited.add(identity)
        replacement = fitted(point)
        point["latitude"] = replacement["latitude"]
        point["longitude"] = replacement["longitude"]

    return fitted(target_anchor), {
        "envelopeFitScale": round(scale, 6),
        "envelopeFitEastM": round(east_shift, 1),
        "envelopeFitNorthM": round(north_shift, 1),
    }


def _layout_geometry_diagnostics(
    points: Sequence[dict[str, float]],
) -> dict[str, float | int]:
    if not points:
        return {
            "boundingWidthM": 0.0,
            "boundingHeightM": 0.0,
            "outsideEnvelopePointCount": 0,
            "maxEnvelopeOverflowM": 0.0,
        }

    center = _mean_point(points)
    projected = [_project(point, center) for point in points]
    east_values = [row[0] for row in projected]
    north_values = [row[1] for row in projected]
    sw, ne, _refs, _side, _handover, _rtb = _random_config_values()
    outside_count = 0
    max_overflow_m = 0.0
    for point in points:
        clamped = {
            "latitude": max(float(sw[0]), min(float(ne[0]), float(point["latitude"]))),
            "longitude": max(float(sw[1]), min(float(ne[1]), float(point["longitude"]))),
        }
        overflow_m = distance_m(point, clamped)
        if overflow_m > 0.1:
            outside_count += 1
            max_overflow_m = max(max_overflow_m, overflow_m)

    return {
        "boundingWidthM": round(max(east_values) - min(east_values), 1),
        "boundingHeightM": round(max(north_values) - min(north_values), 1),
        "outsideEnvelopePointCount": int(outside_count),
        "maxEnvelopeOverflowM": round(max_overflow_m, 1),
    }


def _guided_meta(package_type: int) -> dict[str, Any]:
    if package_type == 2:
        return {"package2BoundaryLineDone": True, "package2BoundaryDone": True}
    if package_type == 3:
        return {"package3BoundaryLineDone": True, "package3BoundaryDone": True}
    if package_type == 4:
        return {"package4BoundaryDone": True}
    return {}


def generate_random_mission_state(
    package_type: int,
    *,
    seed: int | None = None,
    reference_root: Path | None = None,
) -> dict[str, Any]:
    """시나리오별 RTV 참조를 변형해 GUI state와 생성 메타데이터를 반환한다."""

    profile = _profile(package_type)
    resolved_seed = int(seed) if seed is not None else secrets.randbits(31)
    rng = random.Random(resolved_seed)
    reference_path = reference_path_for_package(package_type, reference_root)
    if not reference_path.exists():
        raise FileNotFoundError(f"RTV 참조 시나리오를 찾을 수 없습니다: {reference_path}")
    try:
        payload = json.loads(reference_path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise ValueError(f"RTV 참조 시나리오를 읽을 수 없습니다: {reference_path.name}: {exc}") from exc
    if not isinstance(payload, dict):
        raise ValueError(f"RTV 참조 시나리오가 JSON 객체가 아닙니다: {reference_path.name}")

    package, mission_ref = _reference_parts(payload)
    reference_missions = package.get("InputMissionList") or package.get("inputMissionList") or []
    if not isinstance(reference_missions, list) or not reference_missions:
        raise ValueError(f"RTV 참조 시나리오에 InputMissionList가 없습니다: {reference_path.name}")
    geometry = _mission_geometry(reference_missions)
    if not geometry:
        raise ValueError(f"RTV 참조 시나리오에 임무 좌표가 없습니다: {reference_path.name}")
    reference_takeovers = _reference_takeovers(mission_ref)
    reference_anchor = _mean_point(reference_takeovers) if reference_takeovers else min(geometry, key=lambda row: row["latitude"])
    target_anchor = _choose_start_anchor(rng)
    transform, scale, heading_deg = _build_transform(
        reference_anchor,
        target_anchor,
        geometry,
        rng,
        package_type=profile.package_type,
    )
    requested_skeleton_style = transform.recipe.skeleton_style

    layout_error: str | None = None
    first_layout_error: str | None = None
    attempted_gains: list[float] = []
    layout_recipe_attempts = 0
    layout_succeeded = False
    for recipe_attempt in range(20):
        layout_recipe_attempts = recipe_attempt + 1
        if recipe_attempt:
            transform, scale, heading_deg = _build_transform(
                reference_anchor,
                target_anchor,
                geometry,
                rng,
                package_type=profile.package_type,
            )
            if transform.recipe.skeleton_style != requested_skeleton_style:
                retained_recipe = replace(
                    transform.recipe,
                    skeleton_style=requested_skeleton_style,
                )
                transform = _DeploymentTransform(
                    transform.source_origin,
                    transform.target_origin,
                    transform.source_axis_rad,
                    transform.source_extent_m,
                    retained_recipe,
                )
                scale = transform.scale
                heading_deg = float(retained_recipe.heading_deg)
        transformed_missions = _transform_missions(
            reference_missions,
            transform,
            rng,
            package_type=profile.package_type,
        )
        transformed_first_lines = transformed_missions[0].get("lineList") or []
        transformed_last_lines = transformed_missions[-1].get("lineList") or []
        route_start_anchor = (
            dict(transformed_first_lines[0]["points"][0])
            if transformed_first_lines and (transformed_first_lines[0].get("points") or [])
            else None
        )
        route_return_anchor = (
            dict(transformed_last_lines[0]["points"][-1])
            if transformed_last_lines and (transformed_last_lines[0].get("points") or [])
            else None
        )
        for local_gain in (1.0, 0.85, 0.70, 0.55, 0.40, 0.25, 0.12, 0.0):
            attempted_gains.append(local_gain)
            missions = copy.deepcopy(transformed_missions)
            local_layout_metadata = _apply_mission_stage_variation(
                missions,
                transform,
                package_type=profile.package_type,
                gain=local_gain,
            )
            semantic_spine = local_layout_metadata.pop("_semanticSpine", [])
            reverse_branch_return = bool(
                local_layout_metadata.pop("_reverseBranchReturn", False)
            )
            branch_count = _normalize_branch_missions(missions, profile.package_type)
            clipped_pairs = _clip_transition_lines(missions)
            topology_connections = _reconnect_mission_topology(
                missions,
                profile.package_type,
                route_start_anchor=route_start_anchor,
                route_return_anchor=route_return_anchor,
                semantic_spine=semantic_spine,
                reverse_branch_return=reverse_branch_return,
            )
            fitted_target_anchor, envelope_fit_metadata = _fit_missions_to_auto_bounds(
                missions,
                target_anchor,
            )
            local_layout_metadata.update(envelope_fit_metadata)
            layout_error = _mission_layout_error(
                missions,
                profile.package_type,
                topology_connections=topology_connections,
                clipped_pairs=clipped_pairs,
            )
            if layout_error is None:
                target_anchor = fitted_target_anchor
                layout_succeeded = True
                break
            if first_layout_error is None:
                first_layout_error = layout_error
        if layout_succeeded:
            break
    if not layout_succeeded:  # pragma: no cover - bounded rejection is defensive.
        raise ValueError(f"자동 배치 연결 검증에 실패했습니다: {layout_error}")
    takeovers, handovers, rtbs = _formation_points(
        target_anchor,
        rng,
        deployment_heading_deg=heading_deg,
    )
    local_layout_metadata["localVariationAttempts"] = len(attempted_gains)
    local_layout_metadata["layoutRecipeAttempts"] = layout_recipe_attempts
    local_layout_metadata["localVariationFallbackReason"] = first_layout_error
    mission_points = _all_state_mission_points(missions)
    if not mission_points:
        raise ValueError("변형된 임무 좌표가 없습니다.")
    flight_areas, prohibited_areas = _build_reference_areas(
        mission_ref,
        transform,
        mission_points,
        [*takeovers, *handovers, *rtbs],
        rng,
    )
    targets, target_metadata = _build_targets(missions, takeovers, profile, rng)
    layout_diagnostics = _layout_geometry_diagnostics(
        [*mission_points, *takeovers, *handovers, *rtbs]
    )
    layout_recipe = transform.recipe

    auto_meta = {
        "seed": resolved_seed,
        "scenarioLabel": profile.label,
        "referenceFilename": reference_path.name,
        "layoutVersion": LAYOUT_VERSION,
        "layoutVariant": layout_recipe.variant,
        "mirrored": bool(layout_recipe.mirrored),
        "startAnchor": {
            "latitude": round(float(target_anchor["latitude"]), 8),
            "longitude": round(float(target_anchor["longitude"]), 8),
        },
        "scale": round(scale, 6),
        "northHeadingDeg": round(heading_deg, 6),
        "forwardScale": round(float(layout_recipe.forward_scale), 6),
        "lateralScale": round(float(layout_recipe.lateral_scale), 6),
        "shearRatio": round(float(layout_recipe.shear_ratio), 6),
        "curveAmplitudeM": round(float(layout_recipe.curve_amplitude_m), 1),
        "curveCycles": int(layout_recipe.curve_cycles),
        "stageForwardFactors": [
            round(float(value), 4) for value in layout_recipe.stage_forward_factors
        ],
        "stageLateralOffsetsM": [
            round(float(value), 1) for value in layout_recipe.stage_lateral_offsets_m
        ],
        "stageLateralScales": [
            round(float(value), 4) for value in layout_recipe.stage_lateral_scales
        ],
        "turnStyle": layout_recipe.turn_style,
        "skeletonStyle": layout_recipe.skeleton_style,
        "skeletonLateralRatio": round(float(layout_recipe.skeleton_lateral_ratio), 4),
        "stageSpacingPower": round(float(layout_recipe.stage_spacing_power), 4),
        "branchSpacingScale": round(float(layout_recipe.branch_spacing_scale), 4),
        "branchFanRotationDeg": round(float(layout_recipe.branch_fan_rotation_deg), 3),
        "branchSplayDeg": round(float(layout_recipe.branch_splay_deg), 3),
        "branchStaggerM": round(float(layout_recipe.branch_stagger_m), 1),
        "densityStyle": layout_recipe.density_style,
        "areaPlacementStyle": layout_recipe.area_placement_style,
        "centerSpacingScale": round(float(layout_recipe.center_spacing_scale), 4),
        "areaFootprintScale": round(float(layout_recipe.area_footprint_scale), 4),
        "missionTurnDeltasDeg": [
            round(float(value), 3) for value in layout_recipe.mission_turn_deltas_deg
        ],
        "missionLengthFactors": [
            round(float(value), 4) for value in layout_recipe.mission_length_factors
        ],
        "missionLateralOffsetsM": [
            round(float(value), 1) for value in layout_recipe.mission_lateral_offsets_m
        ],
        "missionForwardOffsetsM": [
            round(float(value), 1) for value in layout_recipe.mission_forward_offsets_m
        ],
        "missionLocalRotationsDeg": [
            round(float(value), 3) for value in layout_recipe.mission_local_rotations_deg
        ],
        "missionAreaAspects": [
            round(float(value), 4) for value in layout_recipe.mission_area_aspects
        ],
        "clippedLinePairs": clipped_pairs,
        "topologyConnections": topology_connections,
        "branchCount": branch_count,
        **local_layout_metadata,
        **layout_diagnostics,
        **target_metadata,
    }
    state = {
        "packageType": profile.package_type,
        "packageID": None,
        "takeOver": takeovers,
        "handOver": handovers,
        "rtb": rtbs,
        "flightAreas": flight_areas,
        "prohibitedAreas": prohibited_areas,
        "missions": missions,
        "targets": targets,
        "guidedMeta": {**_guided_meta(profile.package_type), "autoGeneration": auto_meta},
        "demEnabled": True,
        "refAgl": 600,
        "missionAlt": 700,
        "areaLower": 0,
        "areaUpper": 5000,
    }
    return {"state": state, "metadata": auto_meta}


__all__ = [
    "DEFAULT_REFERENCE_ROOT",
    "LAYOUT_VERSION",
    "MISSION_TYPE_SEQUENCES",
    "SCENARIO_PROFILES",
    "distance_m",
    "generate_random_mission_state",
    "line_to_polygon_boundary",
    "point_in_polygon",
    "point_to_polygon_boundary_m",
    "reference_path_for_package",
    "scenario_label_for_package",
]
