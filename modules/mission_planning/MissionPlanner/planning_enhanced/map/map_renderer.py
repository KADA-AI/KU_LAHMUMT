from __future__ import annotations

import math
from typing import Any, Dict, Iterable, List, Optional, Tuple

import folium
from folium import DivIcon
from folium.plugins import AntPath

from ..models import DirectionDebug, SplitPiece, SplitRunResult


LINE_TYPES = {1, 4, 5, 7}
AREA_TYPES = {2, 3, 6}

COLORS = [
    "#e53935",
    "#1e88e5",
    "#43a047",
    "#fb8c00",
    "#8e24aa",
    "#00897b",
    "#6d4c41",
    "#3949ab",
    "#c2185b",
    "#7cb342",
]


def _pick_color(idx: int) -> str:
    return COLORS[idx % len(COLORS)]


def _coord_to_latlon(coord: Dict[str, Any]) -> Tuple[float, float]:
    return float(coord["latitude"]), float(coord["longitude"])


def _coords_to_latlon(coords: Iterable[Dict[str, Any]]) -> List[Tuple[float, float]]:
    out: List[Tuple[float, float]] = []
    for c in coords:
        if not isinstance(c, dict):
            continue
        if "latitude" not in c or "longitude" not in c:
            continue
        out.append((float(c["latitude"]), float(c["longitude"])))
    return out


def _centroid(coords: List[Tuple[float, float]]) -> Tuple[float, float]:
    lat = sum(c[0] for c in coords) / len(coords)
    lon = sum(c[1] for c in coords) / len(coords)
    return lat, lon


def _corridor_polygon(path_ll: List[Tuple[float, float]], width_m: float) -> List[Tuple[float, float]]:
    half = float(width_m) / 2.0
    poly_left: List[Tuple[float, float]] = []
    poly_right: List[Tuple[float, float]] = []

    for i in range(len(path_ll) - 1):
        lat1, lon1 = path_ll[i]
        lat2, lon2 = path_ll[i + 1]
        dx = (lon2 - lon1) * 111_000.0 * math.cos(math.radians((lat1 + lat2) / 2.0))
        dy = (lat2 - lat1) * 111_000.0
        length = math.hypot(dx, dy)
        if length == 0:
            continue
        ux, uy = dx / length, dy / length
        px, py = uy, -ux

        dlat = (py * half) / 111_000.0
        dlon = (px * half) / (111_000.0 * math.cos(math.radians(lat1)))

        left_start = (lat1 + dlat, lon1 + dlon)
        right_start = (lat1 - dlat, lon1 - dlon)
        left_end = (lat2 + dlat, lon2 + dlon)
        right_end = (lat2 - dlat, lon2 - dlon)

        if i == 0:
            poly_left.append(left_start)
            poly_right.append(right_start)
        poly_left.append(left_end)
        poly_right.append(right_end)

    return poly_left + poly_right[::-1]


def _offset_latlon(lat: float, lon: float, north_m: float = 0.0, east_m: float = 0.0) -> Tuple[float, float]:
    k = 111_132.92
    dlat = north_m / k
    dlon = east_m / (k * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon


def _bearing_to_vec(bearing_deg: float, length_m: float) -> Tuple[float, float]:
    th = math.radians(float(bearing_deg))
    east = math.sin(th) * length_m
    north = math.cos(th) * length_m
    return north, east


def _mission_geometry_points(mission: Dict[str, Any]) -> List[Tuple[float, float]]:
    detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else {}
    mtype = int(mission.get("inputMissionType", 0) or 0)
    if mtype in LINE_TYPES:
        lines = detail.get("lineList") if isinstance(detail, dict) else []
        if isinstance(lines, list) and lines:
            coords = lines[0].get("coordinateList")
            if isinstance(coords, list):
                return _coords_to_latlon(coords)
    if mtype in AREA_TYPES:
        areas = detail.get("areaList") if isinstance(detail, dict) else []
        if isinstance(areas, list) and areas:
            coords = areas[0].get("coordinateList")
            if isinstance(coords, list):
                return _coords_to_latlon(coords)
    return []


def _initial_center(cmpk: Optional[Dict[str, Any]], mrpk: Optional[Dict[str, Any]]) -> Tuple[float, float]:
    if isinstance(mrpk, dict):
        take_over = mrpk.get("takeOverInfoList")
        if isinstance(take_over, list) and take_over:
            first = take_over[0].get("coordinate") if isinstance(take_over[0], dict) else None
            if isinstance(first, dict) and "latitude" in first and "longitude" in first:
                return float(first["latitude"]), float(first["longitude"])
    if isinstance(cmpk, dict):
        missions = cmpk.get("inputMissionList")
        if isinstance(missions, list):
            for mission in missions:
                if isinstance(mission, dict):
                    pts = _mission_geometry_points(mission)
                    if pts:
                        return pts[0]
    return 37.5665, 126.9780


def _add_0203_layers(m: folium.Map, mrpk: Dict[str, Any]) -> None:
    take_over = mrpk.get("takeOverInfoList")
    if isinstance(take_over, list):
        for idx, item in enumerate(take_over, start=1):
            coord = item.get("coordinate") if isinstance(item, dict) else None
            if not isinstance(coord, dict) or "latitude" not in coord or "longitude" not in coord:
                continue
            loc = (float(coord["latitude"]), float(coord["longitude"]))
            folium.CircleMarker(
                location=loc,
                radius=5,
                color="#0d47a1",
                fill=True,
                fill_color="#0d47a1",
                tooltip=f"0203 TakeOver #{idx}",
            ).add_to(m)

    hand_over = mrpk.get("handOverInfoList")
    if isinstance(hand_over, list):
        for idx, item in enumerate(hand_over, start=1):
            coord = item.get("coordinate") if isinstance(item, dict) else None
            if not isinstance(coord, dict) or "latitude" not in coord or "longitude" not in coord:
                continue
            loc = (float(coord["latitude"]), float(coord["longitude"]))
            folium.CircleMarker(
                location=loc,
                radius=5,
                color="#1b5e20",
                fill=True,
                fill_color="#1b5e20",
                tooltip=f"0203 HandOver #{idx}",
            ).add_to(m)


def _add_original_missions(m: folium.Map, cmpk: Dict[str, Any]) -> None:
    missions = cmpk.get("inputMissionList")
    if not isinstance(missions, list):
        return

    for order, mission in enumerate(missions, start=1):
        if not isinstance(mission, dict):
            continue
        color = _pick_color(order - 1)
        mission_id = mission.get("inputMissionID", order)
        mtype = int(mission.get("inputMissionType", 0) or 0)
        detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else {}
        label_point: Optional[Tuple[float, float]] = None

        if mtype in LINE_TYPES:
            line_list = detail.get("lineList") if isinstance(detail, dict) else []
            if isinstance(line_list, list):
                for line in line_list:
                    if not isinstance(line, dict):
                        continue
                    coords = _coords_to_latlon(line.get("coordinateList", []))
                    if len(coords) < 2:
                        continue
                    label_point = label_point or coords[0]
                    folium.PolyLine(
                        locations=coords,
                        color=color,
                        weight=4,
                        tooltip=f"0201 Mission#{order} ID={mission_id} type={mtype}",
                    ).add_to(m)

                    try:
                        width = float(line.get("width", 0.0))
                    except (TypeError, ValueError):
                        width = 0.0
                    if width > 0:
                        corridor = _corridor_polygon(coords, width)
                        if len(corridor) >= 3:
                            folium.Polygon(
                                locations=corridor,
                                color=color,
                                weight=1,
                                fill=True,
                                fill_color=color,
                                fill_opacity=0.10,
                            ).add_to(m)

        elif mtype in AREA_TYPES:
            area_list = detail.get("areaList") if isinstance(detail, dict) else []
            if isinstance(area_list, list):
                for area in area_list:
                    if not isinstance(area, dict):
                        continue
                    coords = _coords_to_latlon(area.get("coordinateList", []))
                    if len(coords) < 3:
                        continue
                    label_point = label_point or _centroid(coords)
                    folium.Polygon(
                        locations=coords,
                        color=color,
                        weight=2,
                        fill=True,
                        fill_color=color,
                        fill_opacity=0.22,
                        tooltip=f"0201 Mission#{order} ID={mission_id} type={mtype}",
                    ).add_to(m)

        if label_point is not None:
            folium.Marker(
                location=label_point,
                icon=DivIcon(
                    icon_size=(24, 24),
                    icon_anchor=(12, 12),
                    html=(
                        f"<div style='font-size:12px;font-weight:bold;color:{color};"
                        f"background:#fff;border:1px solid {color};border-radius:10px;"
                        f"padding:1px 6px;'>M{order}</div>"
                    ),
                ),
            ).add_to(m)


def _add_split_result_layers(m: folium.Map, split_result: SplitRunResult) -> None:
    for piece in split_result.pieces:
        color = _pick_color(piece.parent_order - 1)
        coords = _coords_to_latlon(piece.data.get("coordinateList", []))
        if len(coords) < 3:
            continue
        folium.Polygon(
            locations=coords,
            color="#000000",
            weight=1,
            fill=True,
            fill_color=color,
            fill_opacity=0.35,
            tooltip=(
                f"Split M{piece.parent_order} piece#{piece.piece_index} "
                f"UAV={piece.assigned_uav}"
            ),
        ).add_to(m)

    for dbg in split_result.directions:
        color = _pick_color(dbg.parent_order - 1)
        if dbg.prev_point and dbg.center_point:
            p = _coord_to_latlon(dbg.prev_point)
            c = _coord_to_latlon(dbg.center_point)
            AntPath(
                locations=[p, c],
                color="#ff1744",
                weight=3,
                opacity=0.85,
                delay=500,
                tooltip=f"M{dbg.parent_order} move bearing={dbg.bearing_move_deg:.1f} deg",
            ).add_to(m)

        if dbg.center_point and dbg.bearing_split_deg is not None:
            c_lat, c_lon = _coord_to_latlon(dbg.center_point)
            n1, e1 = _bearing_to_vec(dbg.bearing_split_deg, 2200.0)
            n2, e2 = _bearing_to_vec((dbg.bearing_split_deg + 180.0) % 360.0, 2200.0)
            p1 = _offset_latlon(c_lat, c_lon, north_m=n1, east_m=e1)
            p2 = _offset_latlon(c_lat, c_lon, north_m=n2, east_m=e2)
            folium.PolyLine(
                locations=[p1, p2],
                color="#6a1b9a",
                weight=2,
                dash_array="8,6",
                tooltip=f"M{dbg.parent_order} split axis={dbg.bearing_split_deg:.1f} deg",
            ).add_to(m)

        if dbg.line_start and dbg.line_end:
            s = _coord_to_latlon(dbg.line_start)
            e = _coord_to_latlon(dbg.line_end)
            AntPath(
                locations=[s, e],
                color="#00acc1",
                weight=3,
                opacity=0.8,
                delay=650,
                tooltip=f"M{dbg.parent_order} line direction",
            ).add_to(m)


def _add_legend(m: folium.Map, split_result: Optional[SplitRunResult]) -> None:
    pieces = len(split_result.pieces) if split_result is not None else 0
    uav_count = split_result.uav_count if split_result is not None else 0
    html = f"""
    <div style="
        position: fixed;
        bottom: 16px;
        left: 16px;
        z-index: 9999;
        background: white;
        border: 1px solid #333;
        border-radius: 6px;
        padding: 8px 10px;
        font-size: 12px;
        box-shadow: 0 2px 6px rgba(0,0,0,0.25);
    ">
      <div><b>Mission Split Tester</b></div>
      <div>Original: mission order color-coded</div>
      <div>Split: filled darker + black border</div>
      <div>Direction: red(move), purple(split-axis)</div>
      <div>Pieces: {pieces}, UAV count: {uav_count}</div>
    </div>
    """
    m.get_root().html.add_child(folium.Element(html))


def build_map_html(
    cmpk: Optional[Dict[str, Any]],
    mrpk: Optional[Dict[str, Any]],
    split_result: Optional[SplitRunResult] = None,
) -> str:
    center = _initial_center(cmpk, mrpk)
    m = folium.Map(location=center, zoom_start=12, control_scale=True, tiles="OpenStreetMap")

    if isinstance(mrpk, dict):
        _add_0203_layers(m, mrpk)
    if isinstance(cmpk, dict):
        _add_original_missions(m, cmpk)
    if split_result is not None:
        _add_split_result_layers(m, split_result)

    _add_legend(m, split_result)
    return m.get_root().render()
