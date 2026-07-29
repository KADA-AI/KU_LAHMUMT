from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
from shapely.affinity import scale as geom_scale, translate
from shapely.geometry import LineString, Polygon
from shapely.ops import linemerge, unary_union
from shapely.strtree import STRtree

try:
    import cv2
except Exception:
    cv2 = None  # type: ignore

try:
    from ...capture_physics import (
        area_sequential_split_max_width_m as _area_sequential_split_max_width_m,
        area_sequential_split_width_m as _area_sequential_split_width_m,
        max_sweep_row_chord_m_llh as _max_sweep_row_chord_m_llh,
    )
except Exception:
    try:
        from modules.mission_planning.MissionPlanner.capture_physics import (  # type: ignore
            area_sequential_split_max_width_m as _area_sequential_split_max_width_m,
            area_sequential_split_width_m as _area_sequential_split_width_m,
            max_sweep_row_chord_m_llh as _max_sweep_row_chord_m_llh,
        )
    except Exception:
        def _area_sequential_split_max_width_m() -> float:  # type: ignore
            return 0.0

        def _area_sequential_split_width_m() -> float:  # type: ignore
            return 0.0

        def _max_sweep_row_chord_m_llh(coords, bearing_deg) -> float:  # type: ignore
            return 0.0
try:
    from ...runtime_settings import get_runtime_str
except Exception:
    try:
        from modules.mission_planning.MissionPlanner.runtime_settings import get_runtime_str  # type: ignore
    except Exception:
        get_runtime_str = None  # type: ignore


_R = 6_378_137.0
_DEFAULT_AREA_BEARING_DEG = 90.0
_AREA_EXPAND_SCALE = 1.10
_AREA_SIMPLIFY_BASE_RATIO = 0.028
_AREA_SIMPLIFY_MAX_RATIO = 0.32
_PROJECT_ROOT = Path(__file__).resolve().parents[5]
_DEM_PATH = _PROJECT_ROOT / "modules" / "mission_planning" / "MissionPlanner" / "AnS" / "DEM.jpg"
DEM_RESOLUTION = 100.0
DEM_IMG = cv2.imread(str(_DEM_PATH), cv2.IMREAD_GRAYSCALE) if (cv2 is not None and _DEM_PATH.exists()) else None


def _runtime_area_sweep_mode() -> str:
    if get_runtime_str is None:
        return "vertical"
    try:
        raw = str(get_runtime_str("area_sweep_mode", "vertical") or "vertical").strip().lower()
    except Exception:
        return "vertical"
    if raw in {"vertical", "ver", "perpendicular", "orthogonal"}:
        return "vertical"
    if raw in {"nadir", "directdown", "bf_nadir"}:
        return "nadir"
    return "parallel"


def _runtime_area_split_mode() -> str:
    if get_runtime_str is None:
        return "single_stage"
    try:
        raw = str(get_runtime_str("area_split_mode", "single_stage") or "single_stage").strip().lower()
    except Exception:
        return "single_stage"
    if raw in {"single", "single_stage", "one_stage", "1stage", "single_only"}:
        return "single_stage"
    return "two_stage"


def llh_to_xy(lat: float, lon: float, lat0: float, lon0: float) -> Tuple[float, float]:
    lat0_r = math.radians(lat0)
    x = math.radians(lon - lon0) * _R * math.cos(lat0_r)
    y = math.radians(lat - lat0) * _R
    return x, y


def xy_to_llh(x: float, y: float, lat0: float, lon0: float) -> Tuple[float, float]:
    lat0_r = math.radians(lat0)
    lat = lat0 + math.degrees(y / _R)
    lon = lon0 + math.degrees(x / (_R * math.cos(lat0_r)))
    return lat, lon


def _llh2xy(lat: float, lon: float, lat0: float, lon0: float) -> Tuple[float, float]:
    return llh_to_xy(lat, lon, lat0, lon0)


def _xy2llh(x: float, y: float, lat0: float, lon0: float) -> Tuple[float, float]:
    return xy_to_llh(x, y, lat0, lon0)


def altitude_stats_llh(llh_list: List[Dict[str, Any]]) -> Tuple[float, float]:
    if DEM_IMG is None or not llh_list:
        return 0.0, 0.0
    h, w = DEM_IMG.shape
    lat0, lon0 = llh_list[0]["latitude"], llh_list[0]["longitude"]
    vals = []
    for p in llh_list:
        x, y = llh_to_xy(p["latitude"], p["longitude"], lat0, lon0)
        px = int(round(x / DEM_RESOLUTION))
        py = int(round(y / DEM_RESOLUTION))
        if 0 <= px < w and 0 <= py < h:
            vals.append(float(DEM_IMG[py, px]))
    return (float(np.mean(vals)), float(np.var(vals))) if vals else (0.0, 0.0)


def _clip_poly(poly: List[Tuple[float, float]], A: float, B: float, C: float) -> List[Tuple[float, float]]:
    out: List[Tuple[float, float]] = []
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        f1 = A * x1 + B * y1 + C
        f2 = A * x2 + B * y2 + C
        if f1 <= 0:
            out.append((x1, y1))
        if f1 * f2 < 0:
            t = f1 / (f1 - f2)
            out.append((x1 + t * (x2 - x1), y1 + t * (y2 - y1)))
    return out


def divide_corridor_polyline(line_seg: dict, uav_cnt: int) -> list[dict]:
    def _non_adjacent_cross_points(xy_np: np.ndarray):
        segs = [LineString([xy_np[i], xy_np[i + 1]]) for i in range(len(xy_np) - 1)]
        pts = []
        if not segs:
            return pts

        try:
            tree = STRtree(segs)
            import numpy as _np

            pairs = _np.array(tree.query_bulk(segs)).T
            seen = set()
            for i, j in pairs:
                if i == j:
                    continue
                if abs(i - j) <= 1:
                    continue
                if i > j:
                    i, j = j, i
                if (i, j) in seen:
                    continue
                seen.add((i, j))
                inter = segs[i].intersection(segs[j])
                if inter.is_empty:
                    continue
                if inter.geom_type == "Point":
                    pts.append(inter)
                elif inter.geom_type == "MultiPoint":
                    pts.extend(list(inter.geoms))
            return pts
        except Exception:
            for i in range(len(segs)):
                for j in range(i + 2, len(segs)):
                    inter = segs[i].intersection(segs[j])
                    if inter.is_empty:
                        continue
                    if inter.geom_type == "Point":
                        pts.append(inter)
                    elif inter.geom_type == "MultiPoint":
                        pts.extend(list(inter.geoms))
            return pts

    def _build_cross_cutmask(xy_np: np.ndarray, cut_radius: float):
        pts = _non_adjacent_cross_points(xy_np)
        if not pts:
            return None
        disks = [p.buffer(cut_radius, cap_style=1, join_style=1) for p in pts]
        return unary_union(disks) if len(disks) > 1 else disks[0]

    def _spurious_overlap_mask(center_xy: np.ndarray, width: float):
        if center_xy is None or len(center_xy) < 3:
            return None

        r = width * 0.5
        segs = [LineString([center_xy[i], center_xy[i + 1]]) for i in range(len(center_xy) - 1)]
        buffs = [s.buffer(r, cap_style=CAP_SQUARE, join_style=JOIN_MITRE, mitre_limit=MITRE_LIMIT) for s in segs]

        try:
            import numpy as _np

            tree = STRtree(buffs)
            pairs = _np.array(tree.query_bulk(buffs)).T
        except Exception:
            pairs = [(i, j) for i in range(len(buffs)) for j in range(i + 1, len(buffs))]

        cuts, seen = [], set()
        for i, j in pairs:
            if i == j or abs(i - j) <= 1:
                continue
            if i > j:
                i, j = j, i
            if (i, j) in seen:
                continue
            seen.add((i, j))

            inter = buffs[i].intersection(buffs[j])
            if inter.is_empty:
                continue
            if inter.area < (width * width * 0.02):
                continue
            cuts.append(inter)

        if not cuts:
            return None

        mask = unary_union(cuts).buffer(width * 0.06, cap_style=1, join_style=1)
        return mask if (mask and not mask.is_empty) else None

    if uav_cnt < 1:
        raise ValueError("uav_cnt must be >= 1")

    coords_llh = line_seg["coordinateList"]
    if len(coords_llh) < 2:
        raise ValueError("coordinateList must contain >= 2 points")

    lat0, lon0 = coords_llh[0]["latitude"], coords_llh[0]["longitude"]
    xy = np.array([llh_to_xy(p["latitude"], p["longitude"], lat0, lon0) for p in coords_llh], dtype=float)
    line_world = LineString(xy)

    total_w = float(line_seg["width"])
    indiv_w = total_w / uav_cnt
    half_w = total_w * 0.5

    JOIN_MITRE = 2
    CAP_SQUARE = 2
    MITRE_LIMIT = 10.0

    def _offset_line(ls: LineString, dist: float):
        if abs(float(dist)) <= 1e-9:
            return LineString(list(ls.coords))
        side = "left" if dist >= 0 else "right"
        return ls.parallel_offset(abs(dist), side, join_style=JOIN_MITRE, mitre_limit=MITRE_LIMIT)

    corridor_poly = line_world.buffer(half_w, cap_style=CAP_SQUARE, join_style=JOIN_MITRE, mitre_limit=MITRE_LIMIT)
    if not corridor_poly.is_valid:
        corridor_poly = corridor_poly.buffer(0)

    cross_cutmask = _build_cross_cutmask(xy, cut_radius=indiv_w * 0.51)

    strips: list[dict] = []
    for i in range(uav_cnt):
        d_outer = half_w - i * indiv_w
        d_inner = d_outer - indiv_w
        d_center = d_outer - indiv_w * 0.5

        try:
            center_ls = _offset_line(line_world, d_center)

            center_geom = center_ls
            if center_geom.geom_type == "MultiLineString":
                center_geom = linemerge(center_geom)
            if center_geom.geom_type == "GeometryCollection":
                lines = [g for g in center_geom.geoms if isinstance(g, LineString)]
                center_geom = max(lines, key=lambda g: g.length) if lines else LineString()

            if isinstance(center_geom, LineString) and not center_geom.is_empty:
                center_xy = np.array(center_geom.coords, dtype=float)
            else:
                cs_xy = np.array(center_ls.coords[0], dtype=float)
                ce_xy = np.array(center_ls.coords[-1], dtype=float)
                center_xy = np.vstack([cs_xy, ce_xy])

            if len(center_xy) >= 2:
                dist_start = np.linalg.norm(center_xy[0] - xy[0])
                dist_end = np.linalg.norm(center_xy[-1] - xy[0])
                if dist_start > dist_end:
                    center_xy = center_xy[::-1]

            strip_poly = LineString(center_xy).buffer(
                indiv_w * 0.5, cap_style=CAP_SQUARE, join_style=JOIN_MITRE, mitre_limit=MITRE_LIMIT
            )
            if not strip_poly.is_valid:
                strip_poly = strip_poly.buffer(0)

            poly = strip_poly.intersection(corridor_poly)
            if not poly.is_valid:
                poly = poly.buffer(0)

        except Exception:
            buf_poly = line_world.buffer(indiv_w * 0.5, cap_style=CAP_SQUARE, join_style=JOIN_MITRE, mitre_limit=MITRE_LIMIT)

            v = xy[1] - xy[0]
            v /= np.linalg.norm(v)
            w = np.array([-v[1], v[0]])
            offset_dist = -half_w + (i + 0.5) * indiv_w
            strip_poly = translate(buf_poly, xoff=w[0] * offset_dist, yoff=w[1] * offset_dist)
            poly = strip_poly.intersection(corridor_poly)
            if not poly.is_valid:
                poly = poly.buffer(0)
            center_xy = np.array([pt + w * offset_dist for pt in xy], dtype=float)

        if poly.is_empty or (not isinstance(poly, Polygon)) or len(poly.exterior.coords) < 4:
            raise ValueError(f"strip #{i + 1} polygon empty after generation")

        if (cross_cutmask is not None) and (not cross_cutmask.is_empty):
            cut_result = poly.difference(cross_cutmask)
            if not cut_result.is_empty:
                poly = cut_result

        spmask = _spurious_overlap_mask(center_xy, indiv_w)
        if (spmask is not None) and (not spmask.is_empty):
            cut_geom = poly.difference(spmask)
            if not cut_geom.is_empty:
                poly = cut_geom

        pieces = []
        if isinstance(poly, Polygon):
            pieces = [poly]
        elif poly.geom_type in ("MultiPolygon", "GeometryCollection"):
            pieces = [g for g in poly.geoms if isinstance(g, Polygon)]

        if not pieces:
            raise ValueError(f"strip #{i + 1} polygon empty after cutting")

        for piece in pieces:
            if piece.is_empty or len(piece.exterior.coords) < 4:
                continue

            coord_llh = [
                {
                    "latitude": _xy2llh(float(x), float(y), lat0, lon0)[0],
                    "longitude": _xy2llh(float(x), float(y), lat0, lon0)[1],
                    "altitude": coords_llh[0].get("altitude", 0),
                }
                for (x, y) in list(piece.exterior.coords)[:-1]
            ]

            center_llh = [
                {
                    "latitude": _xy2llh(float(px), float(py), lat0, lon0)[0],
                    "longitude": _xy2llh(float(px), float(py), lat0, lon0)[1],
                    "altitude": coords_llh[0].get("altitude", 0),
                }
                for (px, py) in center_xy
            ] or [
                {
                    "latitude": _xy2llh(*xy[0], lat0, lon0)[0],
                    "longitude": _xy2llh(*xy[0], lat0, lon0)[1],
                    "altitude": coords_llh[0].get("altitude", 0),
                },
                {
                    "latitude": _xy2llh(*xy[-1], lat0, lon0)[0],
                    "longitude": _xy2llh(*xy[-1], lat0, lon0)[1],
                    "altitude": coords_llh[-1].get("altitude", 0),
                },
            ]

            mean_alt, var_alt = altitude_stats_llh(coord_llh)

            strips.append(
                {
                    "Geometry": "Area",
                    "width": indiv_w,
                    "Centerline": center_llh,
                    "coordinateList": coord_llh,
                    "meanAltitude": mean_alt,
                    "altitudeVariance": var_alt,
                }
            )

    return strips


def _bearing_to_unit_xy(bearing_deg: float) -> Tuple[float, float]:
    th = math.radians(float(bearing_deg))
    # local XY: x=east, y=north
    return math.sin(th), math.cos(th)


def _unit_xy_to_bearing(vx: float, vy: float) -> float:
    return (math.degrees(math.atan2(vx, vy)) + 360.0) % 360.0


def _blend_bearings_deg(bearings: List[float]) -> float:
    sx = 0.0
    sy = 0.0
    for b in bearings:
        vx, vy = _bearing_to_unit_xy(b)
        sx += vx
        sy += vy
    norm = math.hypot(sx, sy)
    if norm < 1e-9:
        return float(bearings[0])
    return _unit_xy_to_bearing(sx / norm, sy / norm)


def _angle_diff_deg(a: float, b: float) -> float:
    d = (a - b + 180.0) % 360.0 - 180.0
    return abs(d)


def _largest_polygon(geom: Any) -> Optional[Polygon]:
    if isinstance(geom, Polygon):
        return geom
    if geom is None or geom.is_empty:
        return None
    if geom.geom_type in ("MultiPolygon", "GeometryCollection"):
        polys = [g for g in geom.geoms if isinstance(g, Polygon) and not g.is_empty]
        if polys:
            return max(polys, key=lambda g: g.area)
    return None


def _longest_line(geom: Any) -> Optional[LineString]:
    if isinstance(geom, LineString):
        return geom
    if geom is None or geom.is_empty:
        return None
    if geom.geom_type in ("MultiLineString", "GeometryCollection"):
        lines = [g for g in geom.geoms if isinstance(g, LineString) and not g.is_empty and g.length > 0]
        if lines:
            return max(lines, key=lambda g: g.length)
    return None


def _representative_point_xy(geom: Any) -> Optional[Tuple[float, float]]:
    if geom is None or geom.is_empty:
        return None
    if geom.geom_type == "Point":
        return float(geom.x), float(geom.y)
    if geom.geom_type == "MultiPoint":
        pts = list(geom.geoms)
        if not pts:
            return None
        x = sum(float(p.x) for p in pts) / len(pts)
        y = sum(float(p.y) for p in pts) / len(pts)
        return x, y
    if geom.geom_type in ("LineString", "LinearRing"):
        p = geom.interpolate(0.5, normalized=True)
        return float(p.x), float(p.y)
    if geom.geom_type in ("MultiLineString", "GeometryCollection"):
        line = _longest_line(geom)
        if line is not None:
            p = line.interpolate(0.5, normalized=True)
            return float(p.x), float(p.y)
    p = geom.representative_point()
    return float(p.x), float(p.y)


def _project_t_on_line(pt_xy: Tuple[float, float], line: LineString) -> float:
    p0 = line.coords[0]
    p1 = line.coords[-1]
    vx = float(p1[0] - p0[0])
    vy = float(p1[1] - p0[1])
    den = vx * vx + vy * vy
    if den < 1e-12:
        return 0.0
    return ((pt_xy[0] - float(p0[0])) * vx + (pt_xy[1] - float(p0[1])) * vy) / den


def _piece_polygon_xy(
    piece: Dict[str, Any],
    ref_lat0: float,
    ref_lon0: float,
    use_raw_preferred: bool = True,
) -> Optional[Polygon]:
    raw = piece.get("rawCoordinateList")
    if use_raw_preferred:
        coords = raw if isinstance(raw, list) and len(raw) >= 3 else piece.get("coordinateList")
    else:
        coords = piece.get("coordinateList")
        if not isinstance(coords, list) or len(coords) < 3:
            coords = raw
    if not isinstance(coords, list) or len(coords) < 3:
        return None
    poly_xy = [_llh2xy(float(p["latitude"]), float(p["longitude"]), ref_lat0, ref_lon0) for p in coords]
    poly = Polygon(poly_xy)
    if not poly.is_valid:
        poly = poly.buffer(0)
    return _largest_polygon(poly)


def _sort_piece_dicts_by_shared_touch(
    pieces: List[Dict[str, Any]],
    shared_line: Optional[LineString],
    ref_lat0: float,
    ref_lon0: float,
) -> List[Dict[str, Any]]:
    if shared_line is None or len(pieces) <= 1:
        return pieces

    def _touch_t(piece: Dict[str, Any]) -> float:
        poly = _piece_polygon_xy(piece, ref_lat0, ref_lon0)
        if poly is None:
            return 0.0
        inter = poly.boundary.intersection(shared_line)
        rep = _representative_point_xy(inter)
        if rep is None:
            c = poly.centroid
            foot = shared_line.interpolate(shared_line.project(c))
            rep = (float(foot.x), float(foot.y))
        return _project_t_on_line(rep, shared_line)

    return sorted(pieces, key=_touch_t)


def _xy_polygon_to_llh(poly: Polygon, lat0: float, lon0: float, alt0: float) -> List[Dict[str, float]]:
    coords = list(poly.exterior.coords)
    if len(coords) < 4:
        return []
    return [
        {
            "latitude": _xy2llh(float(x), float(y), lat0, lon0)[0],
            "longitude": _xy2llh(float(x), float(y), lat0, lon0)[1],
            "altitude": alt0,
        }
        for (x, y) in coords[:-1]
    ]


def _vertex_count(poly: Polygon) -> int:
    try:
        return max(0, len(list(poly.exterior.coords)) - 1)
    except Exception:
        return 0


def _target_vertex_count(raw_vertex_count: int) -> int:
    if raw_vertex_count >= 9:
        return 6
    if raw_vertex_count >= 7:
        return 5
    return max(4, raw_vertex_count)


def _inflate_and_reduce_polygon(raw_poly: Polygon, strip_span_m: float) -> Polygon:
    geom = raw_poly if raw_poly.is_valid else raw_poly.buffer(0)
    if geom.is_empty:
        return raw_poly

    expanded = geom_scale(geom, xfact=_AREA_EXPAND_SCALE, yfact=_AREA_EXPAND_SCALE, origin="centroid")
    if expanded.is_empty:
        expanded = geom
    if not expanded.is_valid:
        expanded = expanded.buffer(0)

    raw_n = _vertex_count(raw_poly)
    target_n = _target_vertex_count(raw_n)

    base_tol = max(0.25, float(strip_span_m) * _AREA_SIMPLIFY_BASE_RATIO)
    max_tol = max(base_tol, float(strip_span_m) * _AREA_SIMPLIFY_MAX_RATIO)
    trial_tols = [base_tol * m for m in (1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0)]
    trial_tols = [min(t, max_tol) for t in trial_tols]

    best_geom: Optional[Polygon] = None
    best_count = 1_000_000
    for tol in trial_tols:
        cand = expanded.simplify(tol, preserve_topology=True)
        if cand.is_empty:
            continue
        if not cand.is_valid:
            cand = cand.buffer(0)
        picked_cand = _largest_polygon(cand)
        if picked_cand is None:
            continue
        n = _vertex_count(picked_cand)
        if n >= 3 and n < best_count:
            best_count = n
            best_geom = picked_cand
        if n <= target_n and n >= 3:
            best_geom = picked_cand
            break

    picked = best_geom
    if picked is None:
        picked = _largest_polygon(expanded)
    if picked is None:
        picked = _largest_polygon(geom)
    return picked if picked is not None else raw_poly


def _remove_close_points_xy(poly_xy: List[Tuple[float, float]], eps: float = 1e-6) -> List[Tuple[float, float]]:
    if not poly_xy:
        return []
    out = [poly_xy[0]]
    for p in poly_xy[1:]:
        if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > eps:
            out.append(p)
    if len(out) >= 2 and math.hypot(out[0][0] - out[-1][0], out[0][1] - out[-1][1]) <= eps:
        out.pop()
    return out


def _split_area_by_axis(
    area_poly: List[Dict[str, Any]],
    axis_bearing_deg: float,
    prev_pt: Optional[Dict[str, Any]] = None,
    axis_anchor_pt: Optional[Dict[str, Any]] = None,
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    lat0, lon0 = area_poly[0]["latitude"], area_poly[0]["longitude"]
    alt0 = area_poly[0].get("altitude", 0)
    poly_xy = [_llh2xy(p["latitude"], p["longitude"], lat0, lon0) for p in area_poly]
    if len(poly_xy) < 3:
        return [], []

    th = math.radians(axis_bearing_deg)
    dx, dy = math.sin(th), math.cos(th)  # axis direction
    nx, ny = -dy, dx  # axis normal
    # A line through the vertex-average centre does not bisect an irregular
    # polygon's area.  Triangles and tapered quadrilaterals then produce a very
    # short sequential stage.  Locate the cut by cumulative polygon area along
    # the same axis instead; the entry/exit directions stay unchanged.
    balanced_bounds = _equal_area_projection_bounds_xy(
        poly_xy,
        nx,
        ny,
        2,
    )
    if len(balanced_bounds) == 3:
        cut_projection = float(balanced_bounds[1])
    elif (
        isinstance(axis_anchor_pt, dict)
        and "latitude" in axis_anchor_pt
        and "longitude" in axis_anchor_pt
    ):
        cx, cy = _llh2xy(
            float(axis_anchor_pt["latitude"]),
            float(axis_anchor_pt["longitude"]),
            lat0,
            lon0,
        )
        cut_projection = nx * cx + ny * cy
    else:
        cx = sum(x for x, _ in poly_xy) / len(poly_xy)
        cy = sum(y for _, y in poly_xy) / len(poly_xy)
        cut_projection = nx * cx + ny * cy
    C = -float(cut_projection)

    side_a_xy = _remove_close_points_xy(_clip_poly(poly_xy, nx, ny, C))
    side_b_xy = _remove_close_points_xy(_clip_poly(poly_xy, -nx, -ny, -C))

    def _to_llh(xy: List[Tuple[float, float]]) -> List[Dict[str, Any]]:
        out: List[Dict[str, Any]] = []
        for x, y in xy:
            lat, lon = _xy2llh(float(x), float(y), lat0, lon0)
            out.append({"latitude": lat, "longitude": lon, "altitude": alt0})
        return out

    a_llh = _to_llh(side_a_xy) if len(side_a_xy) >= 3 else []
    b_llh = _to_llh(side_b_xy) if len(side_b_xy) >= 3 else []

    if not a_llh or not b_llh or prev_pt is None:
        return a_llh, b_llh

    try:
        px, py = _llh2xy(float(prev_pt["latitude"]), float(prev_pt["longitude"]), lat0, lon0)
        s_prev = nx * px + ny * py + C
        # first polygon = entry side(near prev), second polygon = opposite side
        if s_prev <= 0:
            return a_llh, b_llh
        return b_llh, a_llh
    except Exception:
        return a_llh, b_llh


def _normal_from_cut_bearing(bearing_deg: float) -> Tuple[float, float]:
    # Cut-line direction -> clipping normal (perpendicular vector)
    th = math.radians(float(bearing_deg))
    return math.cos(th), -math.sin(th)


def _annotate_area_outer_owner_pieces(
    pieces: List[Dict],
    owner_count: int,
    bearing_deg: float,
    *,
    ref_lat0: float,
    ref_lon0: float,
    first_sweep: bool,
) -> Dict[int, str]:
    """Mark the low/high edge owners without relying on list ordering."""

    owner_count = max(1, int(owner_count))
    if owner_count < 2 or len(pieces) != owner_count:
        return {}
    nx, ny = _normal_from_cut_bearing(float(bearing_deg))
    centers: List[Tuple[float, int]] = []
    for index, item in enumerate(pieces):
        if not isinstance(item, dict):
            continue
        coords = (
            item.get("rawCoordinateList")
            or item.get("coordinateList")
            or []
        )
        points_xy = [
            _llh2xy(
                float(coord["latitude"]),
                float(coord["longitude"]),
                float(ref_lat0),
                float(ref_lon0),
            )
            for coord in coords
            if isinstance(coord, dict)
            and "latitude" in coord
            and "longitude" in coord
        ]
        if not points_xy:
            continue
        center_x = sum(point[0] for point in points_xy) / len(points_xy)
        center_y = sum(point[1] for point in points_xy) / len(points_xy)
        centers.append((nx * center_x + ny * center_y, index))
    if len(centers) != owner_count:
        return {}

    low_index = min(centers)[1]
    high_index = max(centers)[1]
    outer_side_by_slot: Dict[int, str] = {}
    for index, item in enumerate(pieces):
        owner_slot = index + 1
        item["areaSequentialOwnerSlot"] = int(owner_slot)
        item["areaSequentialOwnerCount"] = int(owner_count)
        outer_side = (
            "min"
            if index == low_index
            else "max"
            if index == high_index
            else None
        )
        if outer_side is None:
            continue
        item["areaOuterOwner"] = True
        item["areaOuterSide"] = str(outer_side)
        item["areaOuterFirstSweep"] = bool(first_sweep)
        outer_side_by_slot[int(owner_slot)] = str(outer_side)
    return outer_side_by_slot


def _equal_area_projection_bounds_xy(
    poly_xy: List[Tuple[float, float]],
    nx: float,
    ny: float,
    part_count: int,
) -> List[float]:
    """Projection cuts whose clipped polygon areas are equal.

    Equal projection *width* only balances rectangles.  For tapered or concave
    mission shapes it can leave a thin end piece with very little filming
    workload.  This monotonic bisection keeps the requested sweep direction but
    chooses each internal boundary from cumulative polygon area.
    """

    if int(part_count) < 1 or len(poly_xy) < 3:
        return []
    subject = Polygon(poly_xy)
    if not subject.is_valid:
        subject = subject.buffer(0)
    subject_poly = _largest_polygon(subject)
    if subject_poly is None or float(subject_poly.area) <= 1e-6:
        return []

    projections = [float(nx) * float(x) + float(ny) * float(y) for x, y in poly_xy]
    d_min = min(projections)
    d_max = max(projections)
    if d_max - d_min <= 1e-9:
        return [float(d_min), float(d_max)]
    if int(part_count) == 1:
        return [float(d_min), float(d_max)]

    # Tangent to the clipping normal.  The generous extension makes the band
    # effectively infinite for this local XY polygon.
    dx, dy = -float(ny), float(nx)
    min_x, min_y, max_x, max_y = subject_poly.bounds
    extent = max(
        10_000.0,
        math.hypot(float(max_x) - float(min_x), float(max_y) - float(min_y))
        * 4.0,
    )
    low_extent = float(d_min) - extent

    def _area_through(cut: float) -> float:
        half_plane = Polygon(
            [
                (
                    float(nx) * low_extent + dx * extent,
                    float(ny) * low_extent + dy * extent,
                ),
                (
                    float(nx) * float(cut) + dx * extent,
                    float(ny) * float(cut) + dy * extent,
                ),
                (
                    float(nx) * float(cut) - dx * extent,
                    float(ny) * float(cut) - dy * extent,
                ),
                (
                    float(nx) * low_extent - dx * extent,
                    float(ny) * low_extent - dy * extent,
                ),
            ]
        )
        try:
            return float(subject_poly.intersection(half_plane).area)
        except Exception:
            return 0.0

    total_area = float(subject_poly.area)
    bounds: List[float] = [float(d_min)]
    for part_index in range(1, int(part_count)):
        target_area = total_area * float(part_index) / float(part_count)
        low = float(bounds[-1])
        high = float(d_max)
        for _ in range(52):
            mid = (low + high) * 0.5
            if _area_through(mid) < target_area:
                low = mid
            else:
                high = mid
        bounds.append((low + high) * 0.5)
    bounds.append(float(d_max))
    return bounds


def _divide_search_area_clip_with_bounds(
    area_poly: List[Dict[str, Any]],
    bearing_deg: float,
    bounds: List[float],
    ref_lat0: Optional[float] = None,
    ref_lon0: Optional[float] = None,
) -> List[Dict]:
    if len(bounds) < 2:
        raise ValueError("bounds must have at least two values")

    lat0 = float(ref_lat0) if ref_lat0 is not None else float(area_poly[0]["latitude"])
    lon0 = float(ref_lon0) if ref_lon0 is not None else float(area_poly[0]["longitude"])
    alt0 = area_poly[0].get("altitude", 0)
    poly_xy = [_llh2xy(p["latitude"], p["longitude"], lat0, lon0) for p in area_poly]
    nx, ny = _normal_from_cut_bearing(bearing_deg)
    th = math.radians(float(bearing_deg))
    dx, dy = math.sin(th), math.cos(th)  # cut-line direction (unit)

    subject = Polygon(poly_xy)
    if not subject.is_valid:
        subject = subject.buffer(0)
    subject_poly = _largest_polygon(subject)
    if subject_poly is None:
        raise ValueError("invalid source polygon for clipping")

    max_abs = max(max(abs(x), abs(y)) for x, y in poly_xy) if poly_xy else 1000.0
    extent = max(10_000.0, max_abs * 4.0)

    out: List[Dict] = []
    for i in range(len(bounds) - 1):
        b0 = float(bounds[i])
        b1 = float(bounds[i + 1])
        low = min(b0, b1)
        high = max(b0, b1)

        # Infinite strip between n·p=low and n·p=high.
        band_poly = Polygon(
            [
                (nx * low + dx * extent, ny * low + dy * extent),
                (nx * high + dx * extent, ny * high + dy * extent),
                (nx * high - dx * extent, ny * high - dy * extent),
                (nx * low - dx * extent, ny * low - dy * extent),
            ]
        )
        clipped = subject_poly.intersection(band_poly)
        if not clipped.is_valid:
            clipped = clipped.buffer(0)
        picked = _largest_polygon(clipped)
        if picked is None:
            raise ValueError(f"clip strip #{i + 1} invalid polygon")

        raw_coord_llh = _xy_polygon_to_llh(picked, lat0, lon0, alt0)
        if len(raw_coord_llh) < 3:
            raise ValueError(f"clip strip #{i + 1} degenerate polygon")

        strip_span = max(0.1, abs(high - low))
        post_poly = _inflate_and_reduce_polygon(picked, strip_span)
        coord_llh = _xy_polygon_to_llh(post_poly, lat0, lon0, alt0)
        if len(coord_llh) < 3:
            coord_llh = raw_coord_llh

        mean_alt, var_alt = altitude_stats_llh(coord_llh)
        out.append(
            {
                "Geometry": "Area",
                "coordinateList": coord_llh,
                "rawCoordinateList": raw_coord_llh,
                "postProcess": {
                    "expandScale": _AREA_EXPAND_SCALE,
                    "style": "vertex_reduce",
                    "targetVertexCount": _target_vertex_count(len(raw_coord_llh)),
                    "simplifyToleranceBaseM": max(0.25, float(strip_span) * _AREA_SIMPLIFY_BASE_RATIO),
                },
                "meanAltitude": mean_alt,
                "altitudeVariance": var_alt,
            }
        )
    return out


def divide_search_area_two_stage(
    area_poly: List[Dict[str, Any]],
    uav_cnt: int,
    boundary_axis_bearing_deg: float,
    entry_move_bearing_deg: float,
    exit_move_bearing_deg: float,
    prev_pt: Optional[Dict[str, Any]] = None,
    boundary_anchor_pt: Optional[Dict[str, Any]] = None,
) -> List[Dict]:
    if uav_cnt < 1:
        raise ValueError("uav_cnt must be >= 1")
    lat0 = float(area_poly[0]["latitude"])
    lon0 = float(area_poly[0]["longitude"])

    entry_side, exit_side = _split_area_by_axis(
        area_poly,
        boundary_axis_bearing_deg,
        prev_pt=prev_pt,
        axis_anchor_pt=boundary_anchor_pt,
    )
    if len(entry_side) < 3 or len(exit_side) < 3:
        out = divide_search_area_clip(
            area_poly,
            uav_cnt,
            float(entry_move_bearing_deg),
            ref_lat0=float(area_poly[0]["latitude"]),
            ref_lon0=float(area_poly[0]["longitude"]),
        )
        for r in out:
            r["splitStage"] = 1
            r["splitCount"] = 1
        _annotate_area_outer_owner_pieces(
            out,
            uav_cnt,
            float(entry_move_bearing_deg),
            ref_lat0=lat0,
            ref_lon0=lon0,
            first_sweep=True,
        )
        return out

    # Master split on entry-side first, then transfer strip widths across boundary axis.
    entry_xy = [_llh2xy(p["latitude"], p["longitude"], lat0, lon0) for p in entry_side]
    exit_xy = [_llh2xy(p["latitude"], p["longitude"], lat0, lon0) for p in exit_side]
    n1x, n1y = _normal_from_cut_bearing(float(entry_move_bearing_deg))
    pvals1 = [n1x * x + n1y * y for x, y in entry_xy]
    p1_min, p1_max = min(pvals1), max(pvals1)
    bounds1 = _equal_area_projection_bounds_xy(
        entry_xy,
        n1x,
        n1y,
        uav_cnt,
    )
    if len(bounds1) != int(uav_cnt) + 1:
        bounds1 = [
            p1_min + (p1_max - p1_min) * i / uav_cnt
            for i in range(uav_cnt + 1)
        ]

    n2x, n2y = _normal_from_cut_bearing(float(exit_move_bearing_deg))
    p2 = [n2x * x + n2y * y for x, y in exit_xy]
    e_min, e_max = min(p2), max(p2)

    # Transfer entry-side internal split boundaries to exit-side using
    # actual entry/exit shared boundary intersections. This makes S1/S2
    # boundaries connect continuously with different cut directions.
    entry_poly = Polygon(entry_xy)
    exit_poly = Polygon(exit_xy)
    if not entry_poly.is_valid:
        entry_poly = entry_poly.buffer(0)
    if not exit_poly.is_valid:
        exit_poly = exit_poly.buffer(0)
    entry_poly = _largest_polygon(entry_poly)
    exit_poly = _largest_polygon(exit_poly)
    shared_line: Optional[LineString] = None
    if entry_poly is not None and exit_poly is not None:
        shared_line = _longest_line(entry_poly.boundary.intersection(exit_poly.boundary))

    bounds2: List[float]
    if shared_line is not None and uav_cnt >= 2:
        th1 = math.radians(float(entry_move_bearing_deg))
        d1x, d1y = math.sin(th1), math.cos(th1)
        max_abs = 0.0
        for x, y in entry_xy + exit_xy:
            max_abs = max(max_abs, abs(x), abs(y))
        extent = max(10_000.0, max_abs * 4.0)
        transfer_vals: List[float] = []
        for k in bounds1[1:-1]:
            # Any point with n1*p=k lies on this split boundary line.
            px, py = n1x * float(k), n1y * float(k)
            cut_line = LineString([(px - d1x * extent, py - d1y * extent), (px + d1x * extent, py + d1y * extent)])
            inter = cut_line.intersection(shared_line)
            rep = _representative_point_xy(inter)
            if rep is None:
                transfer_vals = []
                break
            transfer_vals.append(n2x * rep[0] + n2y * rep[1])
        if len(transfer_vals) == (uav_cnt - 1):
            clamped = [max(e_min, min(e_max, v)) for v in transfer_vals]
            bounds2 = [e_min] + sorted(clamped) + [e_max]
        else:
            bounds2 = _equal_area_projection_bounds_xy(
                exit_xy,
                n2x,
                n2y,
                uav_cnt,
            )
    else:
        bounds2 = _equal_area_projection_bounds_xy(
            exit_xy,
            n2x,
            n2y,
            uav_cnt,
        )
    if len(bounds2) != int(uav_cnt) + 1:
        bounds2 = [
            e_min + (e_max - e_min) * i / uav_cnt
            for i in range(uav_cnt + 1)
        ]

    # Keep bounds strictly increasing to avoid zero-width strips.
    eps = max(1e-6, (e_max - e_min) * 1e-6)
    for i in range(1, len(bounds2)):
        if bounds2[i] <= bounds2[i - 1] + eps:
            bounds2[i] = bounds2[i - 1] + eps
    if bounds2[-1] <= bounds2[0] + eps:
        bounds2 = [e_min + (e_max - e_min) * i / uav_cnt for i in range(uav_cnt + 1)]

    try:
        s1 = _divide_search_area_clip_with_bounds(
            entry_side,
            float(entry_move_bearing_deg),
            bounds1,
            ref_lat0=lat0,
            ref_lon0=lon0,
        )
    except Exception:
        out = divide_search_area_clip(area_poly, uav_cnt, float(entry_move_bearing_deg), ref_lat0=lat0, ref_lon0=lon0)
        for r in out:
            r["splitStage"] = 1
            r["splitCount"] = 1
        _annotate_area_outer_owner_pieces(
            out,
            uav_cnt,
            float(entry_move_bearing_deg),
            ref_lat0=lat0,
            ref_lon0=lon0,
            first_sweep=True,
        )
        return out

    try:
        s2 = _divide_search_area_clip_with_bounds(
            exit_side,
            float(exit_move_bearing_deg),
            bounds2,
            ref_lat0=lat0,
            ref_lon0=lon0,
        )
    except Exception:
        out = divide_search_area_clip(area_poly, uav_cnt, float(entry_move_bearing_deg), ref_lat0=lat0, ref_lon0=lon0)
        for r in out:
            r["splitStage"] = 1
            r["splitCount"] = 1
        _annotate_area_outer_owner_pieces(
            out,
            uav_cnt,
            float(entry_move_bearing_deg),
            ref_lat0=lat0,
            ref_lon0=lon0,
            first_sweep=True,
        )
        return out

    if len(s1) != uav_cnt or len(s2) != uav_cnt:
        out = divide_search_area_clip(area_poly, uav_cnt, float(entry_move_bearing_deg), ref_lat0=lat0, ref_lon0=lon0)
        for r in out:
            r["splitStage"] = 1
            r["splitCount"] = 1
        _annotate_area_outer_owner_pieces(
            out,
            uav_cnt,
            float(entry_move_bearing_deg),
            ref_lat0=lat0,
            ref_lon0=lon0,
            first_sweep=True,
        )
        return out

    s1 = _sort_piece_dicts_by_shared_touch(s1, shared_line, lat0, lon0)
    s2 = _sort_piece_dicts_by_shared_touch(s2, shared_line, lat0, lon0)
    outer_side_by_slot = _annotate_area_outer_owner_pieces(
        s1,
        uav_cnt,
        float(entry_move_bearing_deg),
        ref_lat0=lat0,
        ref_lon0=lon0,
        first_sweep=True,
    )

    for item in s1:
        item["splitStage"] = 1
        item["splitCount"] = 2
        item["phaseMoveBearing_deg"] = float(entry_move_bearing_deg)
        item["phaseSplitBearing_deg"] = float(entry_move_bearing_deg)
        item["boundaryAxisBearing_deg"] = float(boundary_axis_bearing_deg)

    for owner_slot, item in enumerate(s2, start=1):
        item["splitStage"] = 2
        item["splitCount"] = 2
        item["areaSequentialOwnerSlot"] = int(owner_slot)
        item["areaSequentialOwnerCount"] = int(uav_cnt)
        if int(owner_slot) in outer_side_by_slot:
            item["areaOuterOwner"] = True
            item["areaOuterSide"] = str(
                outer_side_by_slot[int(owner_slot)]
            )
            item["areaOuterFirstSweep"] = False
        item["phaseMoveBearing_deg"] = float(exit_move_bearing_deg)
        item["phaseSplitBearing_deg"] = float(exit_move_bearing_deg)
        item["boundaryAxisBearing_deg"] = float(boundary_axis_bearing_deg)

    return s1 + s2


def divide_search_area_clip(
    area_poly: List[Dict],
    uav_cnt: int,
    bearing_deg: float,
    ref_lat0: Optional[float] = None,
    ref_lon0: Optional[float] = None,
) -> List[Dict]:
    if uav_cnt < 1:
        raise ValueError("uav_cnt must be >= 1")
    lat0 = float(ref_lat0) if ref_lat0 is not None else float(area_poly[0]["latitude"])
    lon0 = float(ref_lon0) if ref_lon0 is not None else float(area_poly[0]["longitude"])
    poly_xy = [_llh2xy(p["latitude"], p["longitude"], lat0, lon0) for p in area_poly]
    nx, ny = _normal_from_cut_bearing(float(bearing_deg))
    projs = [nx * x + ny * y for x, y in poly_xy]
    d_min, d_max = min(projs), max(projs)
    bounds = _equal_area_projection_bounds_xy(poly_xy, nx, ny, uav_cnt)
    if len(bounds) != int(uav_cnt) + 1:
        bounds = [
            d_min + (d_max - d_min) * i / uav_cnt
            for i in range(uav_cnt + 1)
        ]
    return _divide_search_area_clip_with_bounds(
        area_poly,
        float(bearing_deg),
        bounds,
        ref_lat0=lat0,
        ref_lon0=lon0,
    )


def _divide_search_area_uniform_projection(
    area_poly: List[Dict[str, Any]],
    part_count: int,
    bearing_deg: float,
) -> List[Dict]:
    """Cut an owner strip into equal physical widths along the sweep axis."""

    part_count = max(1, int(part_count))
    if len(area_poly) < 3:
        return []
    lat0 = float(area_poly[0]["latitude"])
    lon0 = float(area_poly[0]["longitude"])
    poly_xy = [
        _llh2xy(p["latitude"], p["longitude"], lat0, lon0)
        for p in area_poly
    ]
    nx, ny = _normal_from_cut_bearing(float(bearing_deg))
    projections = [nx * x + ny * y for x, y in poly_xy]
    if not projections:
        return []
    d_min = min(projections)
    d_max = max(projections)
    bounds = [
        d_min + (d_max - d_min) * index / part_count
        for index in range(part_count + 1)
    ]
    return _divide_search_area_clip_with_bounds(
        area_poly,
        float(bearing_deg),
        bounds,
        ref_lat0=lat0,
        ref_lon0=lon0,
    )


def _divide_area_by_sequential_width_limit(
    area_poly: List[Dict],
    owner_count: int,
    bearing_deg: float,
    *,
    minimum_stage_count: int = 1,
) -> Tuple[List[Dict], int, float]:
    """Split each owner's strip into enough sequential stages for the width limit."""

    owner_count = max(1, int(owner_count))
    minimum_stage_count = max(1, int(minimum_stage_count))
    base_parts = divide_search_area_clip(area_poly, owner_count, float(bearing_deg))
    if len(base_parts) != owner_count:
        return base_parts, 1, 0.0

    # ``divide_search_area_clip`` is ordered from the low to the high side of
    # the split normal.  Only the two edge owners touch the original AREA
    # exterior.  Persist that ownership so the camera planner can start each
    # edge strip at the convex-hull boundary instead of whichever endpoint is
    # closest to the entry waypoint.
    for owner_slot, item in enumerate(base_parts, start=1):
        if not isinstance(item, dict):
            continue
        item["areaSequentialOwnerSlot"] = int(owner_slot)
        item["areaSequentialOwnerCount"] = int(owner_count)
        outer_side = None
        if owner_count >= 2:
            if owner_slot == 1:
                outer_side = "min"
            elif owner_slot == owner_count:
                outer_side = "max"
        if outer_side is not None:
            item["areaOuterOwner"] = True
            item["areaOuterSide"] = str(outer_side)
            item["areaOuterFirstSweep"] = True

    width_target_m = float(_area_sequential_split_width_m() or 0.0)
    width_limit_m = float(_area_sequential_split_max_width_m() or 0.0)
    widest_owner_span_m = max(
        (
            _max_sweep_row_chord_m_llh(
                item.get("coordinateList"),
                float(bearing_deg),
            )
            for item in base_parts
            if isinstance(item, dict)
        ),
        default=0.0,
    )
    width_stage_count = (
        int(math.ceil(widest_owner_span_m / width_limit_m))
        if width_limit_m > 0.0 and widest_owner_span_m > width_limit_m
        else 1
    )
    stage_count = max(minimum_stage_count, width_stage_count)
    if stage_count <= 1:
        return base_parts, 1, float(widest_owner_span_m)

    owner_stage_parts: List[List[Dict]] = []
    for _attempt in range(16):
        candidate_by_owner: List[List[Dict]] = []
        for owner_slot, base_part in enumerate(base_parts, start=1):
            base_coords = (
                base_part.get("rawCoordinateList")
                or base_part.get("coordinateList")
                or []
            )
            owner_parts = _divide_search_area_uniform_projection(
                base_coords,
                stage_count,
                float(bearing_deg),
            )
            if len(owner_parts) != stage_count:
                candidate_by_owner = []
                break
            for item in owner_parts:
                item["areaSequentialOwnerSlot"] = int(owner_slot)
                item["areaSequentialOwnerCount"] = int(owner_count)
                if owner_count >= 2 and owner_slot in (1, owner_count):
                    item["areaOuterOwner"] = True
                    item["areaOuterSide"] = (
                        "min" if owner_slot == 1 else "max"
                    )
            candidate_by_owner.append(owner_parts)
        if len(candidate_by_owner) != owner_count:
            return base_parts, 1, float(widest_owner_span_m)
        owner_stage_parts = candidate_by_owner
        if width_limit_m <= 0.0:
            break
        widest_stage_span_m = max(
            (
                _max_sweep_row_chord_m_llh(
                    item.get("coordinateList"),
                    float(bearing_deg),
                )
                for owner_parts in candidate_by_owner
                for item in owner_parts
                if isinstance(item, dict)
            ),
            default=0.0,
        )
        if widest_stage_span_m <= width_limit_m + 1.0:
            break
        grown_count = int(
            math.ceil(stage_count * widest_stage_span_m / width_limit_m)
        )
        next_stage_count = min(64, max(stage_count + 1, grown_count))
        if next_stage_count == stage_count:
            break
        stage_count = next_stage_count

    if (
        len(owner_stage_parts) != owner_count
        or any(len(parts) != stage_count for parts in owner_stage_parts)
    ):
        return base_parts, 1, float(widest_owner_span_m)

    # The low edge naturally runs low->high (outside->inside).  The high edge
    # must run high->low so that its first sequential piece also touches the
    # original convex hull.  Middle owners retain the canonical ordering.
    stage_groups: List[List[Dict]] = []
    for logical_stage_index in range(stage_count):
        group: List[Dict] = []
        for owner_index in range(owner_count):
            owner_slot = owner_index + 1
            physical_stage_index = int(logical_stage_index)
            if owner_count >= 2 and owner_slot == owner_count:
                physical_stage_index = stage_count - logical_stage_index - 1
            group.append(
                owner_stage_parts[owner_index][physical_stage_index]
            )
        stage_groups.append(group)
    ordered_parts = [item for group in stage_groups for item in group]
    for stage_index, group in enumerate(stage_groups, start=1):
        for item in group:
            item["splitStage"] = int(stage_index)
            item["splitCount"] = int(stage_count)
            item["areaSequentialWidthSplit"] = True
            item["areaSequentialWidthSpanM"] = round(
                float(widest_owner_span_m), 1
            )
            item["areaSequentialWidthTargetM"] = round(
                float(width_target_m), 1
            )
            item["areaSequentialWidthLimitM"] = round(
                float(width_limit_m), 1
            )
            item["areaOuterFirstSweep"] = bool(
                item.get("areaOuterOwner")
                and int(stage_index) == 1
            )
    return ordered_parts, int(stage_count), float(widest_owner_span_m)


def _centroid_llh(ll: list[dict]) -> dict:
    lat = sum(p["latitude"] for p in ll) / len(ll)
    lon = sum(p["longitude"] for p in ll) / len(ll)
    return {"latitude": lat, "longitude": lon}


def _bearing_deg(p0: dict, p1: dict) -> float:
    lat0, lon0 = math.radians(p0["latitude"]), math.radians(p0["longitude"])
    lat1, lon1 = math.radians(p1["latitude"]), math.radians(p1["longitude"])
    d_lon = lon1 - lon0
    x = math.sin(d_lon) * math.cos(lat1)
    y = math.cos(lat0) * math.sin(lat1) - math.sin(lat0) * math.cos(lat1) * math.cos(d_lon)
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


def _resolve_area_bearing(
    prev_pt: dict | None,
    next_pt: dict | None,
    poly_llh: list[dict],
) -> tuple[dict, float, Optional[float], Optional[float]]:
    center = _centroid_llh(poly_llh)
    bearing_in = _bearing_deg(prev_pt, center) if prev_pt is not None else None
    bearing_out = _bearing_deg(center, next_pt) if next_pt is not None else None

    if bearing_in is not None and bearing_out is not None:
        # Entry and exit are blended; if they are near-opposite, prioritize entry.
        if _angle_diff_deg(bearing_in, bearing_out) >= 160.0:
            bearing_move = float(bearing_in)
        else:
            bearing_move = _blend_bearings_deg([bearing_in, bearing_out])
    elif bearing_in is not None:
        bearing_move = float(bearing_in)
    elif bearing_out is not None:
        bearing_move = float(bearing_out)
    else:
        bearing_move = _DEFAULT_AREA_BEARING_DEG

    return center, bearing_move, bearing_in, bearing_out


def _branch_split_count(
    branch_owner_counts: list[int] | None,
    branch_index: int,
    default_cnt: int,
) -> int:
    """Owner count for a single branch, or the pool size when not per-branch.

    In Type-2 각자도생 mode ``branch_owner_counts[i]`` is how many UAVs own branch
    ``i`` (1 normally; >=2 only when spare UAVs sub-divide that branch). Outside
    per-branch mode (``None``) every element is width-split across the whole pool.

    Zero means every owner of that branch is permanently gone, so the element is
    dropped from the plan rather than handed to someone else.
    """
    if branch_owner_counts is None:
        return max(1, int(default_cnt))
    if 0 <= branch_index < len(branch_owner_counts):
        return max(0, int(branch_owner_counts[branch_index]))
    return 1


_DUP_EPS_DEG = 1e-8  # ~1 mm; consecutive points closer than this count as one point


def _dedupe_consecutive_llh(coords: Any) -> list[dict]:
    out: list[dict] = []
    for p in coords if isinstance(coords, list) else []:
        if not (isinstance(p, dict) and "latitude" in p and "longitude" in p):
            continue
        if out and (
            abs(float(p["latitude"]) - float(out[-1]["latitude"])) < _DUP_EPS_DEG
            and abs(float(p["longitude"]) - float(out[-1]["longitude"])) < _DUP_EPS_DEG
        ):
            continue
        out.append(p)
    return out


def valid_line_segments(detail: Any) -> list[dict]:
    """lineList segments that still span >= 2 distinct points after collapsing
    consecutive duplicates (upstream sometimes sends one point duplicated as a line)."""
    segs = detail.get("lineList") if isinstance(detail, dict) else None
    out: list[dict] = []
    for seg in segs if isinstance(segs, list) else []:
        if isinstance(seg, dict) and len(_dedupe_consecutive_llh(seg.get("coordinateList"))) >= 2:
            out.append(seg)
    return out


def valid_area_entries(detail: Any) -> list[dict]:
    areas = detail.get("areaList") if isinstance(detail, dict) else None
    out: list[dict] = []
    for area in areas if isinstance(areas, list) else []:
        if not isinstance(area, dict) or bool(area.get("isHole")):
            continue
        if len(_dedupe_consecutive_llh(area.get("coordinateList"))) >= 3:
            out.append(area)
    return out


def mission_geometry(mission: Any) -> str | None:
    """Resolve the plannable geometry of a 0201 mission from its shape data.

    inputMissionType only breaks the tie when a mission carries both line and
    area shapes; otherwise whatever valid shape exists wins, regardless of type.
    Returns "line" | "area" | "coordinate" | None.
    """
    if not isinstance(mission, dict):
        return None
    detail = mission.get("missionDetail")
    if not isinstance(detail, dict):
        return None
    has_line = bool(valid_line_segments(detail))
    has_area = bool(valid_area_entries(detail))
    if has_line and has_area:
        try:
            mtype = int(mission.get("inputMissionType") or 0)
        except Exception:
            mtype = 0
        return "area" if mtype in (2, 3, 4, 5, 6) else "line"
    if has_line:
        return "line"
    if has_area:
        return "area"
    if len(_dedupe_consecutive_llh(detail.get("coordinateList"))) >= 2:
        return "coordinate"
    return None


def _coordinate_only_pieces(
    md: Any,
    mtype: int,
    mission_id: Any,
    prev_pt: dict | None,
) -> list[dict]:
    # Coordinate mission fallback: keep as a single piece (no width split).
    coord_list = md.get("coordinateList") if isinstance(md, dict) else None
    if not isinstance(coord_list, list):
        return []
    coords = [
        {
            "latitude": float(p["latitude"]),
            "longitude": float(p["longitude"]),
            "altitude": int(round(float(p.get("altitude", 0) or 0))),
        }
        for p in coord_list
        if isinstance(p, dict) and ("latitude" in p) and ("longitude" in p)
    ]
    if len(coords) < 2:
        return []
    mean_alt, var_alt = altitude_stats_llh(coords)
    piece = {
        "Geometry": "Coordinate",
        "coordinateList": coords,
        "Centerline": coords,
        "width": 1.0,
        "meanAltitude": mean_alt,
        "altitudeVariance": var_alt,
        "fromCoordinateList": True,
    }
    if prev_pt is not None:
        piece["bearingFromPrev"] = _bearing_deg(prev_pt, coords[0])
    piece.update({"inputMissionType": mtype, "MissionID": mission_id})
    return [piece]


def split_mission_into_subareas(
    input_m: dict,
    uav_cnt: int,
    prev_pt: dict | None,
    next_pt: dict | None = None,
    branch_owner_counts: list[int] | None = None,
    *,
    split_branch_area_into_two: bool = False,
) -> list[dict]:
    try:
        mtype = int(input_m.get("inputMissionType") or 0)
    except Exception:
        mtype = 0
    mission_id = input_m.get("inputMissionID")
    md = input_m.get("missionDetail") if isinstance(input_m.get("missionDetail"), dict) else {}
    geometry = mission_geometry(input_m)
    per_branch = branch_owner_counts is not None
    subs: list[dict] = []

    if geometry == "line":
        line_list = md.get("lineList") if isinstance(md, dict) else None
        if not isinstance(line_list, list):
            line_list = []

        for seg_index, seg in enumerate(line_list):
            if not isinstance(seg, dict):
                continue
            deduped = _dedupe_consecutive_llh(seg.get("coordinateList"))
            if len(deduped) < 2:
                # Degenerate segment (single point sent as a line): skip it
                # instead of failing the whole plan.
                continue
            if len(deduped) != len(seg.get("coordinateList") or []):
                seg["coordinateList"] = deduped
            try:
                width_val = float(seg.get("width", 0))
            except Exception:
                width_val = 0.0
            if width_val <= 0:
                seg["width"] = 1.0
            if prev_pt is not None:
                start_pt = seg["coordinateList"][0]
                seg["bearingFromPrev"] = _bearing_deg(prev_pt, start_pt)

            seg_split_cnt = _branch_split_count(branch_owner_counts, seg_index, uav_cnt)
            if seg_split_cnt <= 0:
                # Every owner of this branch is permanently unavailable: drop the
                # element instead of redistributing it to another UAV.
                continue
            try:
                pieces = divide_corridor_polyline(seg, seg_split_cnt)
            except Exception:
                # One malformed segment must not abort the remaining missions.
                continue
            for r in pieces:
                r.update({"inputMissionType": mtype, "MissionID": mission_id})
                if per_branch:
                    r["branchIndex"] = int(seg_index)
                    r["branchOwnerCount"] = int(seg_split_cnt)
                subs.append(r)

        if not subs:
            subs = _coordinate_only_pieces(md, mtype, mission_id, prev_pt)

    elif geometry == "coordinate":
        subs = _coordinate_only_pieces(md, mtype, mission_id, prev_pt)

    elif geometry == "area":
        area_list = md.get("areaList") if isinstance(md, dict) else None
        if not isinstance(area_list, list) or not area_list:
            return subs

        for area_idx, area in enumerate(area_list, start=1):
            if not isinstance(area, dict):
                continue
            if bool(area.get("isHole")):
                continue
            poly = area.get("coordinateList")
            if not isinstance(poly, list) or len(poly) < 3:
                continue

            center, bearing_move, bearing_in, bearing_out = _resolve_area_bearing(prev_pt, next_pt, poly)
            bearing_split = (bearing_move + 90.0) % 360.0
            bearing_entry = float(bearing_in) if bearing_in is not None else float(bearing_move)
            bearing_exit = float(bearing_out) if bearing_out is not None else float(bearing_move)
            area_mode = _runtime_area_sweep_mode()
            area_split_mode = _runtime_area_split_mode()
            area_split_cnt = _branch_split_count(branch_owner_counts, area_idx - 1, uav_cnt)
            if area_split_cnt <= 0:
                # Owner(s) permanently gone: the area leaves the plan with the
                # rest of its branch set rather than being taken over.
                continue
            # Type-2/3 self-reliance boundary Areas use two spatial halves per
            # sticky owner.  Other branch/package types keep their existing
            # single-stage behavior, while ordinary pooled Areas still obey
            # the runtime split-mode option.
            use_two_stage_split = (
                (
                    (per_branch and bool(split_branch_area_into_two))
                    or (
                        not per_branch
                        and area_mode != "nadir"
                        and area_split_mode == "two_stage"
                    )
                )
                and next_pt is not None
                and bearing_out is not None
            )

            # Two-stage split only when there is a valid next mission direction.
            if use_two_stage_split:
                area_parts = divide_search_area_two_stage(
                    poly,
                    area_split_cnt,
                    boundary_axis_bearing_deg=bearing_split,
                    entry_move_bearing_deg=bearing_entry,
                    exit_move_bearing_deg=bearing_exit,
                    prev_pt=prev_pt,
                    boundary_anchor_pt=center,
                )
                # The two-stage helper intentionally falls back to one piece
                # per owner when its entry/exit axis is degenerate.  A
                # self-reliance boundary Area still owes two unique halves, so
                # use adjacent clip pairs as a deterministic secondary split.
                if (
                    per_branch
                    and bool(split_branch_area_into_two)
                    and len(area_parts) != 2 * int(area_split_cnt)
                ):
                    fallback_parts = divide_search_area_clip(
                        poly,
                        2 * int(area_split_cnt),
                        float(bearing_entry),
                    )
                    if len(fallback_parts) == 2 * int(area_split_cnt):
                        stage_one = fallback_parts[0::2]
                        stage_two = fallback_parts[1::2]
                        if int(area_split_cnt) >= 2:
                            # The highest owner pair is [inner, exterior] in
                            # canonical low->high order. Put its exterior half
                            # in stage 1, matching the low edge owner's contract.
                            stage_one[-1], stage_two[-1] = (
                                stage_two[-1],
                                stage_one[-1],
                            )
                        outer_side_by_slot = _annotate_area_outer_owner_pieces(
                            stage_one,
                            int(area_split_cnt),
                            float(bearing_entry),
                            ref_lat0=float(poly[0]["latitude"]),
                            ref_lon0=float(poly[0]["longitude"]),
                            first_sweep=True,
                        )
                        for item in stage_one:
                            item["splitStage"] = 1
                            item["splitCount"] = 2
                        for owner_slot, item in enumerate(stage_two, start=1):
                            item["splitStage"] = 2
                            item["splitCount"] = 2
                            item["areaSequentialOwnerSlot"] = int(owner_slot)
                            item["areaSequentialOwnerCount"] = int(
                                area_split_cnt
                            )
                            if int(owner_slot) in outer_side_by_slot:
                                item["areaOuterOwner"] = True
                                item["areaOuterSide"] = str(
                                    outer_side_by_slot[int(owner_slot)]
                                )
                                item["areaOuterFirstSweep"] = False
                        area_parts = stage_one + stage_two
                # Two fixed stages only satisfy the width contract while the
                # original owner strip is at most twice the configured limit.
                sequential_width_limit_m = float(
                    _area_sequential_split_max_width_m() or 0.0
                )
                if sequential_width_limit_m > 0.0:
                    owner_parts = divide_search_area_clip(
                        poly,
                        int(area_split_cnt),
                        float(bearing_entry),
                    )
                    widest_owner_span_m = max(
                        (
                            _max_sweep_row_chord_m_llh(
                                item.get("coordinateList"),
                                float(bearing_entry),
                            )
                            for item in owner_parts
                            if isinstance(item, dict)
                        ),
                        default=0.0,
                    )
                    if widest_owner_span_m > 2.0 * sequential_width_limit_m:
                        expanded_parts, _stage_count, _span_m = (
                            _divide_area_by_sequential_width_limit(
                                poly,
                                int(area_split_cnt),
                                float(bearing_entry),
                                minimum_stage_count=2,
                            )
                        )
                        if len(expanded_parts) > len(area_parts):
                            area_parts = expanded_parts
                            for item in area_parts:
                                item["phaseMoveBearing_deg"] = float(
                                    bearing_entry
                                )
                                item["phaseSplitBearing_deg"] = float(
                                    bearing_entry
                                )
                                item["boundaryAxisBearing_deg"] = float(
                                    bearing_split
                                )
            elif area_mode == "nadir":
                area_parts, _stage_count, _span_m = (
                    _divide_area_by_sequential_width_limit(
                        poly,
                        int(area_split_cnt),
                        float(bearing_move),
                    )
                )
                for item in area_parts:
                    item["phaseMoveBearing_deg"] = float(bearing_move)
                    item["phaseSplitBearing_deg"] = float(bearing_move)
                    item["boundaryAxisBearing_deg"] = float(bearing_move)
            else:
                area_parts, _stage_count, _span_m = (
                    _divide_area_by_sequential_width_limit(
                        poly,
                        int(area_split_cnt),
                        float(bearing_entry),
                    )
                )
                for item in area_parts:
                    item["phaseMoveBearing_deg"] = float(bearing_entry)
                    item["phaseSplitBearing_deg"] = float(bearing_entry)
                    item["boundaryAxisBearing_deg"] = float(bearing_split)

            for r in area_parts:
                r.update({"inputMissionType": mtype, "MissionID": mission_id})
                r["sourceAreaIndex"] = int(area_idx)
                if per_branch:
                    r["branchIndex"] = int(area_idx - 1)
                    r["branchOwnerCount"] = int(area_split_cnt)
                    if (
                        bool(split_branch_area_into_two)
                        and len(area_parts) >= 2 * int(area_split_cnt)
                    ):
                        r["branchAreaSequentialSplit"] = True
                        r["branchAreaSequence"] = int(r.get("splitStage", 1) or 1)
                r["bearing_deg"] = float(r.get("phaseMoveBearing_deg", bearing_move))
                r["splitBearing_deg"] = float(r.get("phaseSplitBearing_deg", (r["bearing_deg"] + 90.0) % 360.0))
                r["boundaryAxisBearing_deg"] = float(bearing_split)
                if bearing_in is not None:
                    r["bearingIn_deg"] = float(bearing_in)
                if bearing_out is not None:
                    r["bearingOut_deg"] = float(bearing_out)
                if isinstance(prev_pt, dict):
                    r["prevPoint"] = {
                        "latitude": float(prev_pt.get("latitude", center["latitude"])),
                        "longitude": float(prev_pt.get("longitude", center["longitude"])),
                        "altitude": int(round(float(prev_pt.get("altitude", 0) or 0))),
                    }
                else:
                    r["prevPoint"] = {
                        "latitude": float(center["latitude"]),
                        "longitude": float(center["longitude"]),
                        "altitude": 0,
                    }
                if isinstance(next_pt, dict):
                    r["nextPoint"] = {
                        "latitude": float(next_pt.get("latitude", center["latitude"])),
                        "longitude": float(next_pt.get("longitude", center["longitude"])),
                        "altitude": int(round(float(next_pt.get("altitude", 0) or 0))),
                    }
                subs.append(r)
    # geometry None: no plannable shape at all — return empty so the caller can
    # skip this mission instead of aborting the whole plan.
    return subs
