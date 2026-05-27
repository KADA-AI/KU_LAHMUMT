from __future__ import annotations

import math
from typing import Any, Callable

from .BF import BFPlanner
from .area_nadir_planner import build_nadir_overflight_coords

try:
    from modules.mission_planning.MissionPlanner.data_def.coord_transform import llh_to_xy, xy_to_llh
except Exception as exc:  # pragma: no cover - import guard for standalone usage
    raise ImportError("Failed to import MissionPlanner coordinate transforms") from exc


def _coerce_anchor_llh(anchor_llh: Any, fallback_llh: tuple[float, float]) -> tuple[float, float]:
    lat_fallback, lon_fallback = fallback_llh
    if isinstance(anchor_llh, dict):
        try:
            lat = float(anchor_llh.get("latitude", lat_fallback))
        except Exception:
            lat = float(lat_fallback)
        try:
            lon = float(anchor_llh.get("longitude", lon_fallback))
        except Exception:
            lon = float(lon_fallback)
        return lat, lon
    if isinstance(anchor_llh, (list, tuple)) and len(anchor_llh) >= 2:
        try:
            lat = float(anchor_llh[0])
        except Exception:
            lat = float(lat_fallback)
        try:
            lon = float(anchor_llh[1])
        except Exception:
            lon = float(lon_fallback)
        return lat, lon
    return float(lat_fallback), float(lon_fallback)


def _bearing_goal_xy(anchor_xy: tuple[float, float], bearing_deg: float, length_m: float) -> tuple[float, float]:
    th = math.radians(float(bearing_deg))
    dx = math.sin(th) * float(length_m)
    dy = math.cos(th) * float(length_m)
    return float(anchor_xy[0] + dx), float(anchor_xy[1] + dy)


def build_nadir_bf_overflight_coords(
    *,
    polygon_llh: list[tuple[float, float]],
    anchor_llh: Any,
    bearing_deg: float,
    separation_m: float | None = None,
    fov_deg: float | None = None,
    min_segment_m: float = 3.0,
    spacing_margin: float = 1.0,
    altitude_fn: Callable[[float, float], int],
    cruise_alt_m: float = 1000.0,
    r_min: float = 200.0,
    goal_length_m: float = 300.0,
) -> tuple[list[dict], float]:
    """
    BF.py 알고리즘(BFPlanner.plan) 그대로 호출해 좌표를 생성한다.
    - Dubins 전이점(branch 2 + entry 1)을 포함한 full_path를 그대로 사용
    - fallback/보정/중복 제거를 하지 않는다
    """
    _ = separation_m
    _ = min_segment_m
    _ = spacing_margin

    if len(polygon_llh) < 3:
        return [], 0.0

    lat0, lon0 = polygon_llh[0]
    points_xy = [llh_to_xy(lat, lon, lat0, lon0) for lat, lon in polygon_llh]

    anchor_lat, anchor_lon = _coerce_anchor_llh(anchor_llh, (lat0, lon0))
    start_x, start_y = llh_to_xy(anchor_lat, anchor_lon, lat0, lon0)
    goal_x, goal_y = _bearing_goal_xy((start_x, start_y), bearing_deg, goal_length_m)

    start = (float(start_x), float(start_y), float(cruise_alt_m))
    goal = (float(goal_x), float(goal_y), float(cruise_alt_m))

    planner = BFPlanner(
        points_xy,
        start,
        goal,
        R_min=float(r_min),
        insert_transition_points=True,
    )
    if fov_deg is not None and float(fov_deg) > 0.0:
        planner.fov = float(fov_deg)

    full_path_xyh, runtime_fov = planner.plan()
    if not full_path_xyh:
        fallback_coords = build_nadir_overflight_coords(
            polygon_llh=polygon_llh,
            anchor_llh=anchor_llh,
            bearing_deg=bearing_deg,
            separation_m=float(separation_m or cruise_alt_m),
            fov_deg=float(fov_deg or planner.fov),
            min_segment_m=float(min_segment_m),
            spacing_margin=float(spacing_margin),
            altitude_fn=altitude_fn,
        )
        return fallback_coords, float(fov_deg or planner.fov)

    coords: list[dict] = []
    for x, y, _h in full_path_xyh:
        lat, lon = xy_to_llh(float(x), float(y), lat0, lon0)
        coords.append(
            {
                "latitude": float(lat),
                "longitude": float(lon),
                "altitude": int(altitude_fn(float(lat), float(lon))),
            }
        )
    return coords, float(runtime_fov)
