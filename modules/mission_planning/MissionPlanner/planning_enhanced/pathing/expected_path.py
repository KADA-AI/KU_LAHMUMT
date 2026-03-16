from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from shapely.geometry import LineString, Polygon
from shapely.ops import nearest_points

from ..models import DirectionDebug, SplitPiece, SplitRunResult


_R = 6_378_137.0
_LINE_TYPES = {1, 4, 5, 7}
_AREA_TYPES = {2, 3, 6}
_PROJECT_ROOT = Path(__file__).resolve().parents[5]
_DEFAULT_DB_PATH = _PROJECT_ROOT / "resource" / "db" / "fov_db.csv"
_AREA_OFFSET_SIGN = -1.0


def _llh_to_xy(lat: float, lon: float, lat0: float, lon0: float) -> Tuple[float, float]:
    lat0_r = math.radians(lat0)
    x = math.radians(lon - lon0) * _R * math.cos(lat0_r)
    y = math.radians(lat - lat0) * _R
    return x, y


def _xy_to_llh(x: float, y: float, lat0: float, lon0: float) -> Tuple[float, float]:
    lat0_r = math.radians(lat0)
    lat = lat0 + math.degrees(y / _R)
    lon = lon0 + math.degrees(x / (_R * math.cos(lat0_r)))
    return lat, lon


def _unit(vx: float, vy: float) -> Tuple[float, float]:
    n = math.hypot(vx, vy)
    if n < 1e-9:
        return 0.0, 0.0
    return vx / n, vy / n


def _to_coord(lat: float, lon: float, alt: float = 0.0) -> Dict[str, float]:
    return {"latitude": float(lat), "longitude": float(lon), "altitude": float(alt)}


def _to_float(v: Any, default: float = 0.0) -> float:
    try:
        return float(v)
    except Exception:
        return default


def _mid_coord(c0: Dict[str, Any], c1: Dict[str, Any]) -> Dict[str, float]:
    lat = (_to_float(c0.get("latitude")) + _to_float(c1.get("latitude"))) * 0.5
    lon = (_to_float(c0.get("longitude")) + _to_float(c1.get("longitude"))) * 0.5
    alt = (_to_float(c0.get("altitude")) + _to_float(c1.get("altitude"))) * 0.5
    return _to_coord(lat, lon, alt)


def _annotate_path_point_set(row: Dict[str, Any]) -> None:
    coords = row.get("coordinateList")
    if not isinstance(coords, list) or len(coords) < 2:
        return

    parent = int(row.get("parentOrder", 0) or 0)
    idx = int(row.get("index", 0) or 0)
    set_name = str(row.get("setName", f"E{parent}-{idx}"))
    point_list: List[Dict[str, Any]] = []
    seq = 1

    def _push(coord: Dict[str, Any], role: str) -> None:
        nonlocal seq
        c = _to_coord(
            _to_float(coord.get("latitude")),
            _to_float(coord.get("longitude")),
            _to_float(coord.get("altitude"), 0.0),
        )
        point_list.append(
            {
                "name": f"{set_name}-P{seq:02d}",
                "role": role,
                "coordinate": c,
            }
        )
        seq += 1

    # Segment rule:
    # - first segment: start, mid, end
    # - connected segments: (overlapped start skipped) mid, end
    for i in range(len(coords) - 1):
        c0 = coords[i] if isinstance(coords[i], dict) else {}
        c1 = coords[i + 1] if isinstance(coords[i + 1], dict) else {}
        if i == 0:
            _push(c0, "start")
        _push(_mid_coord(c0, c1), "mid")
        _push(c1, "end")

    row["setName"] = set_name
    row["setGroup"] = str(row.get("setGroup", row.get("baseSetName", set_name)))
    row["baseSetName"] = str(row.get("baseSetName", set_name))
    row["pathRole"] = str(row.get("pathRole", "base"))
    row["pointList"] = point_list
    row["pointCount"] = len(point_list)


def _load_sep_db(path: Path) -> List[Tuple[float, float]]:
    rows: List[Tuple[float, float]] = []
    if not path.exists():
        return rows
    with path.open("r", encoding="utf-8-sig", newline="") as f:
        reader = csv.DictReader(f)
        for r in reader:
            w = _to_float(r.get("width"), -1.0)
            s = _to_float(r.get("sep"), -1.0)
            if w > 0 and s > 0:
                rows.append((w, s))
    return rows


def _pick_sep_m(rows: List[Tuple[float, float]], width_m: float) -> float:
    target = max(1.0, float(width_m))
    if not rows:
        return max(80.0, target)
    sorted_rows = sorted(rows, key=lambda x: x[0])
    max_w = sorted_rows[-1][0]
    if target > max_w + 1e-9:
        # Requirement: when width exceeds DB max threshold, do not offset.
        return 0.0
    capped = target

    # DB width means "maximum allowed width" threshold for that row.
    # Pick the smallest DB width that can still cover target (ceiling rule).
    cands = [(w, s) for w, s in sorted_rows if w + 1e-9 >= capped]
    if cands:
        chosen_w = min(w for w, _ in cands)
        chosen_sep = [s for w, s in cands if abs(w - chosen_w) <= 1e-9]
        if chosen_sep:
            return max(chosen_sep)

    # Fallback (should be rare after capping): use largest-width row.
    max_row_w = max(w for w, _ in sorted_rows)
    max_row_sep = [s for w, s in sorted_rows if abs(w - max_row_w) <= 1e-9]
    if max_row_sep:
        return max(max_row_sep)
    return max(sep for _, sep in sorted_rows)


def _line_expected_path(
    piece: SplitPiece,
    direction_debug: Optional[DirectionDebug],
    db_rows: List[Tuple[float, float]],
) -> Optional[Dict[str, Any]]:
    data = piece.data if isinstance(piece.data, dict) else {}
    center = data.get("Centerline")
    if not isinstance(center, list) or len(center) < 2:
        return None

    width_m = max(1.0, _to_float(data.get("width"), 1.0))
    sep_m = _pick_sep_m(db_rows, float(math.ceil(width_m)))

    lat0 = _to_float(center[0].get("latitude"))
    lon0 = _to_float(center[0].get("longitude"))
    alt0 = _to_float(center[0].get("altitude"), 0.0)
    xy = [(_llh_to_xy(_to_float(p.get("latitude")), _to_float(p.get("longitude")), lat0, lon0)) for p in center]

    off_x: float
    off_y: float
    # Offset rule (all missions including first):
    # use mission-start direction side from centerline first segment (reverse direction).
    dx = xy[1][0] - xy[0][0]
    dy = xy[1][1] - xy[0][1]
    ux, uy = _unit(dx, dy)
    if abs(ux) < 1e-9 and abs(uy) < 1e-9:
        return None
    off_x = -ux * sep_m
    off_y = -uy * sep_m

    out: List[Dict[str, float]] = []
    for x, y in xy:
        lat, lon = _xy_to_llh(x + off_x, y + off_y, lat0, lon0)
        out.append(_to_coord(lat, lon, alt0))

    return {
        "parentOrder": piece.parent_order,
        "index": piece.piece_index,
        "missionType": piece.mission_type,
        "source": "line_center_offset_dir",
        "pathRole": "base",
        "widthRefM": float(width_m),
        "pathWidthM": float(width_m),
        "sepM": float(sep_m),
        "coordinateList": out,
    }


def _piece_polygon_xy(piece: SplitPiece, ref_lat0: float, ref_lon0: float) -> Optional[Polygon]:
    data = piece.data if isinstance(piece.data, dict) else {}
    coords = data.get("coordinateList")
    if not isinstance(coords, list) or len(coords) < 3:
        return None
    xy = [
        _llh_to_xy(_to_float(p.get("latitude")), _to_float(p.get("longitude")), ref_lat0, ref_lon0)
        for p in coords
    ]
    poly = Polygon(xy)
    if not poly.is_valid:
        poly = poly.buffer(0)
    if poly.is_empty:
        return None
    if poly.geom_type == "Polygon":
        return poly
    if poly.geom_type in ("MultiPolygon", "GeometryCollection"):
        polys = [g for g in poly.geoms if isinstance(g, Polygon) and not g.is_empty]
        if polys:
            return max(polys, key=lambda g: g.area)
    return None


def _piece_move_bearing_deg(piece: SplitPiece, fallback: float = 90.0) -> float:
    data = piece.data if isinstance(piece.data, dict) else {}
    return _to_float(
        data.get(
            "phaseMoveBearing_deg",
            data.get("bearing_deg", data.get("splitBearing_deg", fallback)),
        ),
        fallback,
    )


def _piece_boundary_bearing_deg(
    piece: SplitPiece,
    direction_debug: Optional[DirectionDebug],
    fallback: float = 90.0,
) -> float:
    data = piece.data if isinstance(piece.data, dict) else {}
    if "boundaryAxisBearing_deg" in data:
        return _to_float(data.get("boundaryAxisBearing_deg"), fallback)
    if direction_debug is not None and direction_debug.bearing_split_deg is not None:
        return float(direction_debug.bearing_split_deg)
    return fallback


def _area_offset_bearing_deg(
    direction_debug: Optional[DirectionDebug],
    fallback: float,
) -> float:
    if direction_debug is not None and direction_debug.bearing_in_deg is not None:
        return float(direction_debug.bearing_in_deg)
    if direction_debug is not None and direction_debug.bearing_move_deg is not None:
        return float(direction_debug.bearing_move_deg)
    return float(fallback)


def _bearing_unit_xy(bearing_deg: float) -> Tuple[float, float]:
    th = math.radians(float(bearing_deg))
    return math.sin(th), math.cos(th)


def _bearing_normal_xy(bearing_deg: float) -> Tuple[float, float]:
    th = math.radians(float(bearing_deg))
    return math.cos(th), -math.sin(th)


def _line_strings_from_geom(geom: Any) -> List[LineString]:
    if geom is None or geom.is_empty:
        return []
    gt = getattr(geom, "geom_type", "")
    if gt == "LineString":
        return [geom]
    if gt in ("MultiLineString", "GeometryCollection"):
        return [g for g in geom.geoms if getattr(g, "geom_type", "") == "LineString" and not g.is_empty]
    return []


def _shared_cut_center_xy(poly_a: Polygon, poly_b: Polygon) -> Optional[Tuple[float, float]]:
    inter = poly_a.boundary.intersection(poly_b.boundary)
    lines = _line_strings_from_geom(inter)
    if lines:
        seg = max(lines, key=lambda g: g.length)
        p = seg.interpolate(0.5, normalized=True)
        return float(p.x), float(p.y)

    gt = getattr(inter, "geom_type", "")
    if gt == "Point":
        return float(inter.x), float(inter.y)
    if gt == "MultiPoint":
        pts = list(inter.geoms)
        if pts:
            x = sum(float(p.x) for p in pts) / len(pts)
            y = sum(float(p.y) for p in pts) / len(pts)
            return x, y

    try:
        p1, p2 = nearest_points(poly_a.boundary, poly_b.boundary)
    except Exception:
        return None
    return (float((p1.x + p2.x) * 0.5), float((p1.y + p2.y) * 0.5))


def _ray_endpoint_from_anchor(
    poly: Polygon,
    anchor_xy: Tuple[float, float],
    dir_xy: Tuple[float, float],
) -> Optional[Tuple[Tuple[float, float], float]]:
    dx, dy = _unit(dir_xy[0], dir_xy[1])
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return None

    minx, miny, maxx, maxy = poly.bounds
    ext = max(maxx - minx, maxy - miny, 1.0) * 4.0
    ray = LineString([anchor_xy, (anchor_xy[0] + dx * ext, anchor_xy[1] + dy * ext)])
    inter = poly.intersection(ray)
    lines = _line_strings_from_geom(inter)
    if not lines:
        return None

    best_pt: Optional[Tuple[float, float]] = None
    best_t = -1.0
    for seg in lines:
        coords = list(seg.coords)
        for x, y in (coords[0], coords[-1]):
            t = dx * (float(x) - anchor_xy[0]) + dy * (float(y) - anchor_xy[1])
            if t > best_t:
                best_t = t
                best_pt = (float(x), float(y))

    if best_pt is None or best_t <= 1e-6:
        return None
    return best_pt, float(best_t)


def _piece_section_end_from_anchor(
    poly: Polygon,
    anchor_xy: Tuple[float, float],
    move_bearing_deg: float,
    boundary_bearing_deg: float,
) -> Optional[Tuple[Tuple[float, float], float]]:
    ux, uy = _bearing_unit_xy(move_bearing_deg)
    nx, ny = _bearing_normal_xy(boundary_bearing_deg)

    c = poly.centroid
    cx, cy = float(c.x), float(c.y)
    # Pick the direction that goes to the same split-axis side as the piece centroid.
    sign_c = nx * (cx - anchor_xy[0]) + ny * (cy - anchor_xy[1])
    dir_xy = (ux, uy)
    if sign_c < 0.0:
        dir_xy = (-ux, -uy)

    pos = _ray_endpoint_from_anchor(poly, anchor_xy, dir_xy)
    neg = _ray_endpoint_from_anchor(poly, anchor_xy, (-dir_xy[0], -dir_xy[1]))
    if pos is None and neg is None:
        return None
    if pos is None:
        return neg
    if neg is None:
        return pos
    return pos if pos[1] >= neg[1] else neg


def _cross_section_through_centroid(
    poly: Polygon,
    bearing_deg: float,
) -> Optional[Tuple[Tuple[float, float], Tuple[float, float], float]]:
    ux, uy = _bearing_unit_xy(bearing_deg)
    if abs(ux) < 1e-9 and abs(uy) < 1e-9:
        return None
    c = poly.centroid
    cx, cy = float(c.x), float(c.y)
    minx, miny, maxx, maxy = poly.bounds
    ext = max(maxx - minx, maxy - miny, 1.0) * 4.0
    line = LineString([(cx - ux * ext, cy - uy * ext), (cx + ux * ext, cy + uy * ext)])
    inter = poly.intersection(line)
    lines = _line_strings_from_geom(inter)
    if not lines:
        return None
    seg = max(lines, key=lambda g: g.length)
    coords = list(seg.coords)
    p0 = (float(coords[0][0]), float(coords[0][1]))
    p1 = (float(coords[-1][0]), float(coords[-1][1]))
    return p0, p1, float(seg.length)


def _fallback_piece_end_from_centroid(
    poly: Polygon,
    anchor_xy: Tuple[float, float],
    move_bearing_deg: float,
) -> Optional[Tuple[Tuple[float, float], float]]:
    cross = _cross_section_through_centroid(poly, move_bearing_deg)
    if cross is None:
        return None
    p0, p1, _ = cross
    d0 = math.hypot(p0[0] - anchor_xy[0], p0[1] - anchor_xy[1])
    d1 = math.hypot(p1[0] - anchor_xy[0], p1[1] - anchor_xy[1])
    if d0 >= d1:
        return p0, float(d0)
    return p1, float(d1)


def _piece_centroid(piece: SplitPiece) -> Optional[Dict[str, float]]:
    data = piece.data if isinstance(piece.data, dict) else {}
    coords = data.get("coordinateList")
    if not isinstance(coords, list) or len(coords) < 3:
        return None
    lat = sum(_to_float(p.get("latitude")) for p in coords) / len(coords)
    lon = sum(_to_float(p.get("longitude")) for p in coords) / len(coords)
    alt = _to_float(coords[0].get("altitude"), 0.0)
    return _to_coord(lat, lon, alt)


def _piece_source_area_index(piece: SplitPiece) -> Optional[int]:
    data = piece.data if isinstance(piece.data, dict) else {}
    if int(piece.mission_type) not in _AREA_TYPES:
        return None
    try:
        v = int(data.get("sourceAreaIndex", 0) or 0)
    except Exception:
        return None
    return v if v > 0 else None


def _poly_width_m_by_move_bearing(poly: Polygon, move_bearing_deg: float) -> float:
    nx, ny = _bearing_normal_xy(move_bearing_deg)
    ex = list(poly.exterior.coords)
    if not ex:
        return 0.0
    vals = [nx * float(x) + ny * float(y) for x, y in ex]
    return float(max(vals) - min(vals)) if vals else 0.0


def _area_pair_expected_path(
    parent_order: int,
    pair_index: int,
    s1_piece: SplitPiece,
    s2_piece: SplitPiece,
    direction_debug: Optional[DirectionDebug],
    db_rows: List[Tuple[float, float]],
) -> Optional[Dict[str, Any]]:
    c1 = _piece_centroid(s1_piece)
    if c1 is None:
        return None
    lat0 = _to_float(c1.get("latitude"))
    lon0 = _to_float(c1.get("longitude"))
    alt0 = _to_float(c1.get("altitude"), 0.0)

    poly1 = _piece_polygon_xy(s1_piece, lat0, lon0)
    poly2 = _piece_polygon_xy(s2_piece, lat0, lon0)
    if poly1 is None or poly2 is None:
        return None

    kink_xy = _shared_cut_center_xy(poly1, poly2)
    if kink_xy is None:
        return None

    b1 = _piece_move_bearing_deg(s1_piece)
    b2 = _piece_move_bearing_deg(s2_piece)
    split_b = _piece_boundary_bearing_deg(s1_piece, direction_debug)

    end1 = _piece_section_end_from_anchor(poly1, kink_xy, b1, split_b)
    end2 = _piece_section_end_from_anchor(poly2, kink_xy, b2, split_b)
    if end1 is None:
        end1 = _fallback_piece_end_from_centroid(poly1, kink_xy, b1)
    if end2 is None:
        end2 = _fallback_piece_end_from_centroid(poly2, kink_xy, b2)
    if end1 is None or end2 is None:
        return None

    p1_xy, len1 = end1
    p2_xy, len2 = end2
    r1 = (s1_piece.data or {}).get("reviewArea")
    r2 = (s2_piece.data or {}).get("reviewArea")
    review_len1 = _to_float(r1.get("segmentLenM"), 0.0) if isinstance(r1, dict) else 0.0
    review_len2 = _to_float(r2.get("segmentLenM"), 0.0) if isinstance(r2, dict) else 0.0
    if review_len1 > 0.0 and review_len2 > 0.0:
        width_ref_m = max(float(review_len1), float(review_len2), 1.0)
    else:
        width_ref_m = max(float(len1), float(len2), 1.0)
    sep_m = _pick_sep_m(db_rows, float(math.ceil(width_ref_m)))
    off_bearing_deg = _area_offset_bearing_deg(direction_debug, b1)
    oux, ouy = _bearing_unit_xy(off_bearing_deg)
    off_x = oux * sep_m * _AREA_OFFSET_SIGN
    off_y = ouy * sep_m * _AREA_OFFSET_SIGN

    p1_ll = _xy_to_llh(p1_xy[0] + off_x, p1_xy[1] + off_y, lat0, lon0)
    k_ll = _xy_to_llh(kink_xy[0] + off_x, kink_xy[1] + off_y, lat0, lon0)
    p2_ll = _xy_to_llh(p2_xy[0] + off_x, p2_xy[1] + off_y, lat0, lon0)

    return {
        "parentOrder": int(parent_order),
        "index": int(pair_index),
        "missionType": s1_piece.mission_type,
        "source": "area_section_chain_by_split_axis",
        "pathRole": "base",
        "widthRefM": float(width_ref_m),
        "pathWidthM": float(width_ref_m),
        "sepM": float(sep_m),
        "segmentLenS1M": float(len1),
        "segmentLenS2M": float(len2),
        "offsetBearingDeg": float(off_bearing_deg),
        "coordinateList": [
            _to_coord(p1_ll[0], p1_ll[1], alt0),
            _to_coord(k_ll[0], k_ll[1], alt0),
            _to_coord(p2_ll[0], p2_ll[1], alt0),
        ],
        "pairPieces": [int(s1_piece.piece_index), int(s2_piece.piece_index)],
    }


def _area_single_expected_path(
    parent_order: int,
    piece: SplitPiece,
    direction_debug: Optional[DirectionDebug],
    db_rows: List[Tuple[float, float]],
) -> Optional[Dict[str, Any]]:
    c = _piece_centroid(piece)
    if c is None:
        return None
    lat0 = _to_float(c.get("latitude"))
    lon0 = _to_float(c.get("longitude"))
    alt0 = _to_float(c.get("altitude"), 0.0)

    poly = _piece_polygon_xy(piece, lat0, lon0)
    if poly is None:
        return None

    b = _piece_move_bearing_deg(piece)
    cross = _cross_section_through_centroid(poly, b)
    if cross is None:
        return None

    p0_xy, p1_xy, length_m = cross
    r = (piece.data or {}).get("reviewArea")
    review_len = _to_float(r.get("segmentLenM"), 0.0) if isinstance(r, dict) else 0.0
    width_ref_m = max(float(review_len if review_len > 0.0 else length_m), 1.0)
    sep_m = _pick_sep_m(db_rows, float(math.ceil(width_ref_m)))
    off_bearing_deg = _area_offset_bearing_deg(direction_debug, b)
    oux, ouy = _bearing_unit_xy(off_bearing_deg)
    off_x = oux * sep_m * _AREA_OFFSET_SIGN
    off_y = ouy * sep_m * _AREA_OFFSET_SIGN

    p0_ll = _xy_to_llh(p0_xy[0] + off_x, p0_xy[1] + off_y, lat0, lon0)
    p1_ll = _xy_to_llh(p1_xy[0] + off_x, p1_xy[1] + off_y, lat0, lon0)

    return {
        "parentOrder": int(parent_order),
        "index": int(piece.piece_index),
        "missionType": piece.mission_type,
        "source": "area_single_section_line",
        "pathRole": "base",
        "widthRefM": float(width_ref_m),
        "pathWidthM": float(width_ref_m),
        "sepM": float(sep_m),
        "segmentLenM": float(length_m),
        "offsetBearingDeg": float(off_bearing_deg),
        "coordinateList": [
            _to_coord(p0_ll[0], p0_ll[1], alt0),
            _to_coord(p1_ll[0], p1_ll[1], alt0),
        ],
    }


def _iter_area_pairs(split_result: SplitRunResult) -> List[Tuple[int, int, SplitPiece, SplitPiece]]:
    by_parent: Dict[int, List[SplitPiece]] = {}
    for piece in split_result.pieces:
        if piece.mission_type not in _AREA_TYPES:
            continue
        by_parent.setdefault(int(piece.parent_order), []).append(piece)

    out: List[Tuple[int, int, SplitPiece, SplitPiece]] = []
    for parent_order in sorted(by_parent):
        pieces = by_parent[parent_order]
        by_source_area: Dict[int, List[SplitPiece]] = {}
        for p in pieces:
            src_idx = _piece_source_area_index(p) or 1
            by_source_area.setdefault(int(src_idx), []).append(p)
        pair_idx = 1
        for source_area_idx in sorted(by_source_area):
            group = by_source_area[source_area_idx]
            s1 = sorted(
                [p for p in group if int((p.data or {}).get("splitStage", 0) or 0) == 1],
                key=lambda p: p.piece_index,
            )
            s2 = sorted(
                [p for p in group if int((p.data or {}).get("splitStage", 0) or 0) == 2],
                key=lambda p: p.piece_index,
            )
            if not s1 or not s2:
                continue

            # 1) Prefer explicit review-pair metadata when available.
            s1_meta: Dict[str, Dict[int, SplitPiece]] = {}
            s2_meta: Dict[str, Dict[int, SplitPiece]] = {}
            for p in s1:
                rd = (p.data or {}).get("reviewArea")
                if not isinstance(rd, dict):
                    continue
                pid = rd.get("pairID")
                sub = rd.get("pairSubIndex")
                if pid is None:
                    continue
                try:
                    sub_i = int(sub)
                except Exception:
                    continue
                s1_meta.setdefault(str(pid), {})[sub_i] = p
            for p in s2:
                rd = (p.data or {}).get("reviewArea")
                if not isinstance(rd, dict):
                    continue
                pid = rd.get("pairID")
                sub = rd.get("pairSubIndex")
                if pid is None:
                    continue
                try:
                    sub_i = int(sub)
                except Exception:
                    continue
                s2_meta.setdefault(str(pid), {})[sub_i] = p

            used_s1: set[int] = set()
            used_s2: set[int] = set()
            for pid in sorted(set(s1_meta.keys()) & set(s2_meta.keys())):
                i1 = s1_meta[pid]
                i2 = s2_meta[pid]
                common = sorted(set(i1.keys()) & set(i2.keys()))
                for sub_i in common:
                    p1 = i1[sub_i]
                    p2 = i2[sub_i]
                    out.append((parent_order, pair_idx, p1, p2))
                    pair_idx += 1
                    used_s1.add(id(p1))
                    used_s2.add(id(p2))

            # 2) Fallback legacy pairing for remaining pieces in same source area.
            s1_left = [p for p in s1 if id(p) not in used_s1]
            s2_left = [p for p in s2 if id(p) not in used_s2]
            for i in range(min(len(s1_left), len(s2_left))):
                out.append((parent_order, pair_idx, s1_left[i], s2_left[i]))
                pair_idx += 1
    return out


def generate_expected_paths(
    split_result: SplitRunResult,
    mrpk: Optional[Dict[str, Any]],
    db_csv_path: Optional[str | Path] = None,
) -> List[Dict[str, Any]]:
    db_path = Path(db_csv_path) if db_csv_path is not None else _DEFAULT_DB_PATH
    db_rows = _load_sep_db(db_path)
    dir_by_parent: Dict[int, DirectionDebug] = {}
    dir_by_area: Dict[Tuple[int, int], DirectionDebug] = {}
    for d in split_result.directions:
        parent = int(d.parent_order)
        if parent not in dir_by_parent:
            dir_by_parent[parent] = d
        sidx = getattr(d, "source_area_index", None)
        if sidx is not None:
            try:
                area_idx = int(sidx)
            except Exception:
                area_idx = 0
            if area_idx > 0:
                dir_by_area[(parent, area_idx)] = d

    def _pick_direction(piece: SplitPiece) -> Optional[DirectionDebug]:
        parent = int(piece.parent_order)
        sidx = _piece_source_area_index(piece)
        if sidx is not None:
            d = dir_by_area.get((parent, int(sidx)))
            if d is not None:
                return d
        return dir_by_parent.get(parent)

    expected_paths: List[Dict[str, Any]] = []

    for piece in sorted(split_result.pieces, key=lambda p: (p.parent_order, p.piece_index)):
        if piece.mission_type not in _LINE_TYPES:
            continue
        path = _line_expected_path(piece, _pick_direction(piece), db_rows)
        if path is not None and len(path.get("coordinateList", [])) >= 2:
            expected_paths.append(path)

    area_by_parent: Dict[int, List[SplitPiece]] = {}
    for piece in split_result.pieces:
        if piece.mission_type in _AREA_TYPES:
            area_by_parent.setdefault(int(piece.parent_order), []).append(piece)

    pairs = _iter_area_pairs(split_result)
    paired_parents = {int(parent_order) for parent_order, _, _, _ in pairs}
    for parent_order, pair_index, s1_piece, s2_piece in pairs:
        dbg = _pick_direction(s1_piece)
        path = _area_pair_expected_path(
            parent_order,
            pair_index,
            s1_piece,
            s2_piece,
            dbg,
            db_rows,
        )
        if path is not None and len(path.get("coordinateList", [])) >= 2:
            expected_paths.append(path)

    for parent_order in sorted(area_by_parent):
        if parent_order in paired_parents:
            continue
        for piece in sorted(area_by_parent[parent_order], key=lambda p: p.piece_index):
            path = _area_single_expected_path(parent_order, piece, _pick_direction(piece), db_rows)
            if path is not None and len(path.get("coordinateList", [])) >= 2:
                expected_paths.append(path)

    expected_paths.sort(
        key=lambda r: (
            int(r.get("parentOrder", 0)),
            int(r.get("index", 0)),
            str(r.get("source", "")),
        )
    )
    for row in expected_paths:
        if isinstance(row, dict):
            _annotate_path_point_set(row)
    return expected_paths
