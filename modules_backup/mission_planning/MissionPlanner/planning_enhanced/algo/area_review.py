from __future__ import annotations

import copy
import math
from typing import Any, Dict, List, Optional, Tuple

from shapely.geometry import LineString, Polygon
from shapely.ops import split as geom_split

from . import split_algorithms as sa
from ..models import DirectionDebug, SplitPiece, SplitRunResult
try:
    from ...runtime_settings import get_runtime_str
except Exception:
    from modules.mission_planning.MissionPlanner.runtime_settings import get_runtime_str  # type: ignore


_AREA_TYPES = {2, 3, 6}


def _runtime_area_sweep_mode() -> str:
    raw = str(get_runtime_str("area_sweep_mode", "vertical") or "vertical").strip().lower()
    if raw in {"vertical", "ver", "perpendicular", "orthogonal"}:
        return "vertical"
    if raw in {"nadir", "directdown", "bf_nadir"}:
        return "nadir"
    return "parallel"


def _to_float(v: Any, default: float = 0.0) -> float:
    try:
        return float(v)
    except Exception:
        return default


def _clone_coord_list(coords: Any) -> List[Dict[str, Any]]:
    if not isinstance(coords, list):
        return []
    out: List[Dict[str, Any]] = []
    for item in coords:
        if isinstance(item, dict):
            out.append(copy.copy(item))
    return out


def _unit(vx: float, vy: float) -> Tuple[float, float]:
    n = math.hypot(vx, vy)
    if n < 1e-9:
        return 0.0, 0.0
    return vx / n, vy / n


def _largest_polygon(geom: Any) -> Optional[Polygon]:
    if isinstance(geom, Polygon):
        return geom
    if geom is None or geom.is_empty:
        return None
    gt = getattr(geom, "geom_type", "")
    if gt in ("MultiPolygon", "GeometryCollection"):
        polys = [g for g in geom.geoms if isinstance(g, Polygon) and not g.is_empty]
        if polys:
            return max(polys, key=lambda g: g.area)
    return None


def _piece_polygon_xy(
    piece: SplitPiece,
    ref_lat0: float,
    ref_lon0: float,
) -> Optional[Polygon]:
    data = piece.data if isinstance(piece.data, dict) else {}
    coords = data.get("coordinateList")
    if not isinstance(coords, list) or len(coords) < 3:
        return None
    xy = [sa.llh_to_xy(float(p["latitude"]), float(p["longitude"]), ref_lat0, ref_lon0) for p in coords]
    poly = Polygon(xy)
    if not poly.is_valid:
        poly = poly.buffer(0)
    return _largest_polygon(poly)


def _axis_bearing_deg(piece: SplitPiece, direction_debug: Optional[DirectionDebug]) -> float:
    data = piece.data if isinstance(piece.data, dict) else {}
    area_mode = _runtime_area_sweep_mode()
    if area_mode in {"vertical", "nadir"}:
        if "phaseMoveBearing_deg" in data:
            return _to_float(data.get("phaseMoveBearing_deg"), 90.0)
        if "bearing_deg" in data:
            return _to_float(data.get("bearing_deg"), 90.0)
        if direction_debug is not None and direction_debug.bearing_move_deg is not None:
            return float(direction_debug.bearing_move_deg)
        return 90.0
    if "boundaryAxisBearing_deg" in data:
        return _to_float(data.get("boundaryAxisBearing_deg"), 90.0)
    if direction_debug is not None and direction_debug.bearing_split_deg is not None:
        return float(direction_debug.bearing_split_deg)
    move = _to_float(data.get("phaseMoveBearing_deg", data.get("bearing_deg", 90.0)), 90.0)
    return (move + 90.0) % 360.0


def _split_lines_from_segment(
    start_xy: Tuple[float, float],
    end_xy: Tuple[float, float],
    axis_bearing_deg: float,
    max_segment_m: float,
    extent_m: float,
    split_count: Optional[int] = None,
) -> Tuple[List[LineString], int, float]:
    dx = end_xy[0] - start_xy[0]
    dy = end_xy[1] - start_xy[1]
    total_len = math.hypot(dx, dy)
    if total_len <= 1e-6:
        return [], 1, 0.0

    if isinstance(split_count, int) and split_count >= 1:
        n = int(split_count)
    else:
        n = max(1, int(math.ceil(total_len / float(max_segment_m))))
    if n <= 1:
        return [], 1, total_len

    ux, uy = dx / total_len, dy / total_len
    th = math.radians(float(axis_bearing_deg))
    ax, ay = math.sin(th), math.cos(th)

    lines: List[LineString] = []
    for i in range(1, n):
        t = total_len * (float(i) / float(n))
        px = start_xy[0] + ux * t
        py = start_xy[1] + uy * t
        lines.append(
            LineString(
                [
                    (px - ax * extent_m, py - ay * extent_m),
                    (px + ax * extent_m, py + ay * extent_m),
                ]
            )
        )
    return lines, n, total_len


def _split_polygon_by_lines(poly: Polygon, lines: List[LineString]) -> List[Polygon]:
    parts: List[Polygon] = [poly]
    for line in lines:
        next_parts: List[Polygon] = []
        for part in parts:
            try:
                out = geom_split(part, line)
            except Exception:
                next_parts.append(part)
                continue
            polys = [g for g in out.geoms if isinstance(g, Polygon) and not g.is_empty and g.area > 1e-6]
            if polys:
                next_parts.extend(polys)
            else:
                next_parts.append(part)
        parts = next_parts
    return parts


def _subdivide_piece_by_segment(
    piece: SplitPiece,
    seg_start: Dict[str, Any],
    seg_end: Dict[str, Any],
    axis_bearing_deg: float,
    max_segment_m: float,
    split_count: Optional[int] = None,
    max_split_count: int = 0,
    min_segment_m: float = 0.0,
) -> Tuple[List[SplitPiece], Dict[str, Any]]:
    data = piece.data if isinstance(piece.data, dict) else {}
    coords = data.get("coordinateList")
    if not isinstance(coords, list) or len(coords) < 3:
        return [piece], {"reason": "invalid_polygon_coords", "changed": False}

    lat0 = float(coords[0]["latitude"])
    lon0 = float(coords[0]["longitude"])
    alt0 = float(coords[0].get("altitude", 0.0))

    poly = _piece_polygon_xy(piece, lat0, lon0)
    if poly is None:
        return [piece], {"reason": "invalid_polygon_geom", "changed": False}

    sx, sy = sa.llh_to_xy(float(seg_start["latitude"]), float(seg_start["longitude"]), lat0, lon0)
    ex, ey = sa.llh_to_xy(float(seg_end["latitude"]), float(seg_end["longitude"]), lat0, lon0)

    minx, miny, maxx, maxy = poly.bounds
    extent = max(maxx - minx, maxy - miny, 1.0) * 4.0
    requested_split_count = split_count
    raw_split_count: Optional[int] = None
    split_capped = False
    split_cap_reason = "-"
    if not (isinstance(requested_split_count, int) and requested_split_count >= 1):
        total_len_guess = math.hypot(ex - sx, ey - sy)
        requested_split_count, raw_split_count, split_capped, split_cap_reason = _resolve_review_split_count(
            total_len_guess,
            max_segment_m,
            max_split_count=max_split_count,
            min_segment_m=min_segment_m,
        )
    else:
        raw_split_count = int(requested_split_count)
        if int(max_split_count or 0) > 0 or float(min_segment_m or 0.0) > 0.0:
            requested_split_count, raw_split_count, split_capped, split_cap_reason = _resolve_review_split_count(
                raw_split_count * float(max(max_segment_m, 1.0)),
                max_segment_m,
                max_split_count=max_split_count,
                min_segment_m=min_segment_m,
            )
    lines, n, total_len = _split_lines_from_segment(
        (sx, sy),
        (ex, ey),
        axis_bearing_deg,
        max_segment_m,
        extent,
        split_count=requested_split_count,
    )
    if not lines or n <= 1:
        return [piece], {
            "reason": "no_need_split",
            "changed": False,
            "segmentLenM": total_len,
            "splitCount": int(n),
            "rawSplitCount": int(raw_split_count if raw_split_count is not None else n),
            "splitCapped": bool(split_capped),
            "splitCapReason": split_cap_reason,
            "maxSplitCountLimit": int(max(0, int(max_split_count or 0))),
            "minSegmentM": float(max(0.0, min_segment_m or 0.0)),
        }

    parts = _split_polygon_by_lines(poly, lines)
    if len(parts) <= 1:
        return [piece], {
            "reason": "split_failed",
            "changed": False,
            "segmentLenM": total_len,
            "splitCount": int(n),
            "rawSplitCount": int(raw_split_count if raw_split_count is not None else n),
            "splitCapped": bool(split_capped),
            "splitCapReason": split_cap_reason,
            "maxSplitCountLimit": int(max(0, int(max_split_count or 0))),
            "minSegmentM": float(max(0.0, min_segment_m or 0.0)),
        }

    ux, uy = _unit(ex - sx, ey - sy)
    parts_sorted = sorted(
        parts,
        key=lambda g: (float(g.centroid.x) - sx) * ux + (float(g.centroid.y) - sy) * uy,
    )

    eq_len = total_len / float(n)
    new_pieces: List[SplitPiece] = []
    for idx, g in enumerate(parts_sorted, start=1):
        raw_llh = sa._xy_polygon_to_llh(g, lat0, lon0, alt0)
        post = sa._inflate_and_reduce_polygon(g, eq_len)
        coord_llh = sa._xy_polygon_to_llh(post, lat0, lon0, alt0)
        if len(coord_llh) < 3:
            coord_llh = raw_llh
        mean_alt, var_alt = sa.altitude_stats_llh(coord_llh)

        new_data = copy.deepcopy(piece.data)
        new_data["coordinateList"] = coord_llh
        new_data["rawCoordinateList"] = raw_llh
        new_data["meanAltitude"] = mean_alt
        new_data["altitudeVariance"] = var_alt
        new_data["reviewArea"] = {
            "subdivided": True,
            "fromPieceIndex": int(piece.piece_index),
            "subIndex": int(idx),
            "splitCount": int(n),
            "rawSplitCount": int(raw_split_count if raw_split_count is not None else n),
            "segmentLenM": float(eq_len),
            "maxSegmentM": float(max_segment_m),
            "maxSplitCountLimit": int(max(0, int(max_split_count or 0))),
            "minSegmentM": float(max(0.0, min_segment_m or 0.0)),
            "splitCapped": bool(split_capped),
            "splitCapReason": split_cap_reason,
            "axisBearingDeg": float(axis_bearing_deg),
            # Keep pre-review polygon so export can restore original
            # (split-before-review, already 10% expanded/simplified) geometry.
            "fromPieceCoordinateList": _clone_coord_list(data.get("coordinateList")),
            "fromPieceRawCoordinateList": _clone_coord_list(data.get("rawCoordinateList")),
        }

        new_pieces.append(
            SplitPiece(
                parent_order=piece.parent_order,
                mission_id=piece.mission_id,
                mission_type=piece.mission_type,
                piece_index=piece.piece_index,
                data=new_data,
                assigned_uav=piece.assigned_uav,
            )
        )

    return new_pieces, {
        "changed": True,
        "oldPieceIndex": int(piece.piece_index),
        "newPieceCount": len(new_pieces),
        "splitCount": int(n),
        "rawSplitCount": int(raw_split_count if raw_split_count is not None else n),
        "splitCapped": bool(split_capped),
        "splitCapReason": split_cap_reason,
        "maxSplitCountLimit": int(max(0, int(max_split_count or 0))),
        "minSegmentM": float(max(0.0, min_segment_m or 0.0)),
        "segmentLenM": float(eq_len),
        "totalLenM": float(total_len),
    }


def _is_overflow_area_row(row: Dict[str, Any]) -> bool:
    if str(row.get("pathRole", "base")) != "base":
        return False
    src = str(row.get("source", ""))
    if src not in ("area_section_chain_by_split_axis", "area_single_section_line"):
        return False
    sep = _to_float(row.get("sepM"), 0.0)
    return sep <= 1e-6


def _segment_len_m_llh(p0: Dict[str, Any], p1: Dict[str, Any]) -> float:
    lat0 = _to_float(p0.get("latitude"))
    lon0 = _to_float(p0.get("longitude"))
    x, y = sa.llh_to_xy(_to_float(p1.get("latitude")), _to_float(p1.get("longitude")), lat0, lon0)
    return float(math.hypot(x, y))


def _piece_source_area_index(piece: SplitPiece) -> Optional[int]:
    data = piece.data if isinstance(piece.data, dict) else {}
    try:
        v = int(data.get("sourceAreaIndex", 0) or 0)
    except Exception:
        return None
    return v if v > 0 else None


def _piece_pattern_type(piece: SplitPiece) -> int:
    data = piece.data if isinstance(piece.data, dict) else {}
    try:
        return int(data.get("patternType", 0) or 0)
    except Exception:
        return 0


def _rebuild_pieces_with_replacements(
    pieces: List[SplitPiece],
    replacements: Dict[Tuple[int, int], List[SplitPiece]],
) -> List[SplitPiece]:
    bucket: Dict[int, List[Tuple[int, int, int, SplitPiece]]] = {}
    for p in pieces:
        parent = int(p.parent_order)
        key = (parent, int(p.piece_index))
        stage = int((p.data or {}).get("splitStage", 0) or 0)
        repl = replacements.get(key)
        if repl:
            for sub_idx, np in enumerate(repl, start=1):
                nstage = int((np.data or {}).get("splitStage", stage) or 0)
                bucket.setdefault(parent, []).append((nstage, int(p.piece_index), sub_idx, np))
        else:
            bucket.setdefault(parent, []).append((stage, int(p.piece_index), 0, p))

    out: List[SplitPiece] = []
    for parent in sorted(bucket):
        rows = sorted(bucket[parent], key=lambda x: (x[0], x[1], x[2]))
        for new_idx, (_, _, _, piece) in enumerate(rows, start=1):
            piece.piece_index = int(new_idx)
            out.append(piece)
    return out


def _coord_copy(coord: Optional[Dict[str, Any]]) -> Optional[Dict[str, float]]:
    if not isinstance(coord, dict):
        return None
    if "latitude" not in coord or "longitude" not in coord:
        return None
    return {
        "latitude": _to_float(coord.get("latitude")),
        "longitude": _to_float(coord.get("longitude")),
        "altitude": _to_float(coord.get("altitude"), 0.0),
    }


def _takeover_map(mrpk: Optional[Dict[str, Any]]) -> Dict[int, Dict[str, float]]:
    if not isinstance(mrpk, dict):
        return {}
    infos = mrpk.get("takeOverInfoList")
    if not isinstance(infos, list):
        return {}
    out: Dict[int, Dict[str, float]] = {}
    for item in infos:
        if not isinstance(item, dict):
            continue
        try:
            aid = int(item.get("aircraftID"))
        except Exception:
            continue
        coord = _coord_copy(item.get("coordinate"))
        if coord is None:
            continue
        out[int(aid)] = coord
    return out


def _piece_centroid_coord(piece: SplitPiece) -> Optional[Dict[str, float]]:
    data = piece.data if isinstance(piece.data, dict) else {}
    coords = data.get("coordinateList")
    if not isinstance(coords, list) or len(coords) < 3:
        return None
    lat0 = _to_float(coords[0].get("latitude"))
    lon0 = _to_float(coords[0].get("longitude"))
    alt0 = _to_float(coords[0].get("altitude"), 0.0)
    poly = _piece_polygon_xy(piece, lat0, lon0)
    if poly is None:
        return None
    clat, clon = sa._xy2llh(float(poly.centroid.x), float(poly.centroid.y), lat0, lon0)
    return {
        "latitude": float(clat),
        "longitude": float(clon),
        "altitude": float(alt0),
    }


def _piece_projection_span_m(piece: SplitPiece, cut_bearing_deg: float) -> float:
    data = piece.data if isinstance(piece.data, dict) else {}
    coords = data.get("coordinateList")
    if not isinstance(coords, list) or len(coords) < 3:
        return 0.0
    lat0 = _to_float(coords[0].get("latitude"))
    lon0 = _to_float(coords[0].get("longitude"))
    poly = _piece_polygon_xy(piece, lat0, lon0)
    if poly is None:
        return 0.0
    th = math.radians(float(cut_bearing_deg))
    nx, ny = math.cos(th), -math.sin(th)
    vals = [float(nx * x + ny * y) for x, y in list(poly.exterior.coords)]
    if not vals:
        return 0.0
    return float(max(vals) - min(vals))


def _piece_within_segment_limit(
    piece: SplitPiece,
    *,
    axis_bearing_deg: float,
    max_segment_m: float,
) -> tuple[bool, float]:
    span_m = float(_piece_projection_span_m(piece, axis_bearing_deg))
    limit_m = max(float(max_segment_m), 1.0)
    return span_m <= (limit_m + 1e-6), span_m


def _resolve_review_split_count(
    projected_span_m: float,
    max_segment_m: float,
    *,
    max_split_count: int = 0,
    min_segment_m: float = 0.0,
) -> Tuple[int, int, bool, str]:
    raw_count = max(1, int(math.ceil(float(projected_span_m) / float(max(max_segment_m, 1.0)))))
    split_count = int(raw_count)
    reasons: List[str] = []
    if int(max_split_count or 0) > 0 and split_count > int(max_split_count):
        split_count = int(max_split_count)
        reasons.append("max_split_count")
    if float(min_segment_m or 0.0) > 0.0 and split_count > 1:
        max_by_min_segment = max(1, int(math.floor(float(projected_span_m) / float(min_segment_m))))
        if split_count > max_by_min_segment:
            split_count = int(max_by_min_segment)
            reasons.append("min_segment_m")
    split_count = max(1, int(split_count))
    return split_count, raw_count, bool(split_count < raw_count), "|".join(reasons) if reasons else "-"


def _sort_parts_near_uav(
    parts: List[Dict[str, Any]],
    uav_coord: Dict[str, float],
    cut_bearing_deg: float,
    lat0: float,
    lon0: float,
) -> List[Dict[str, Any]]:
    if len(parts) <= 1:
        return parts
    th = math.radians(float(cut_bearing_deg))
    nx, ny = math.cos(th), -math.sin(th)

    def _centroid_proj(item: Dict[str, Any]) -> float:
        coords = item.get("coordinateList")
        if not isinstance(coords, list) or len(coords) < 3:
            return 0.0
        poly_xy = [sa.llh_to_xy(_to_float(p.get("latitude")), _to_float(p.get("longitude")), lat0, lon0) for p in coords]
        poly = Polygon(poly_xy)
        if not poly.is_valid:
            poly = poly.buffer(0)
        picked = _largest_polygon(poly)
        if picked is None:
            return 0.0
        return float(nx * picked.centroid.x + ny * picked.centroid.y)

    ux, uy = sa.llh_to_xy(_to_float(uav_coord.get("latitude")), _to_float(uav_coord.get("longitude")), lat0, lon0)
    u_proj = float(nx * ux + ny * uy)
    ordered = sorted(parts, key=_centroid_proj)
    if not ordered:
        return parts
    first_proj = _centroid_proj(ordered[0])
    last_proj = _centroid_proj(ordered[-1])
    if abs(u_proj - last_proj) < abs(u_proj - first_proj):
        ordered.reverse()
    return ordered


def _apply_replan_local_bearing(
    data: Dict[str, Any],
    move_bearing_deg: float,
    uav_coord: Optional[Dict[str, float]],
    *,
    split_count: int,
    projected_span_m: float,
    max_segment_m: float,
    piece_index: int,
    sub_index: Optional[int] = None,
    source_piece_data: Optional[Dict[str, Any]] = None,
    raw_split_count: Optional[int] = None,
    split_capped: bool = False,
    split_cap_reason: str = "-",
    max_split_count: int = 0,
    min_segment_m: float = 0.0,
) -> None:
    boundary_axis_deg = (float(move_bearing_deg) + 90.0) % 360.0
    data["bearingFromPrev"] = float(move_bearing_deg)
    data["bearing_deg"] = float(move_bearing_deg)
    data["phaseMoveBearing_deg"] = float(move_bearing_deg)
    data["phaseSplitBearing_deg"] = float(move_bearing_deg)
    data["splitBearing_deg"] = float(move_bearing_deg)
    data["boundaryAxisBearing_deg"] = float(boundary_axis_deg)
    data["bearingIn_deg"] = float(move_bearing_deg)
    if uav_coord is not None:
        data["prevPoint"] = {
            "latitude": float(uav_coord.get("latitude", 0.0)),
            "longitude": float(uav_coord.get("longitude", 0.0)),
            "altitude": int(round(float(uav_coord.get("altitude", 0.0) or 0.0))),
        }

    review = data.get("reviewArea") if isinstance(data.get("reviewArea"), dict) else {}
    review.update(
        {
            "mode": "replan_local_assigned",
            "localAssigned": True,
            "subdivided": bool(int(split_count) > 1),
            "fromPieceIndex": int(piece_index),
            "splitCount": int(max(1, split_count)),
            "projectedSpanM": float(max(0.0, projected_span_m)),
            "segmentLenM": float(max(0.0, projected_span_m) / float(max(1, split_count))),
            "maxSegmentM": float(max_segment_m),
            "axisBearingDeg": float(move_bearing_deg),
            "rawSplitCount": int(raw_split_count if raw_split_count is not None else split_count),
            "splitCapped": bool(split_capped),
            "splitCapReason": str(split_cap_reason or "-"),
            "maxSplitCountLimit": int(max(0, int(max_split_count or 0))),
            "minSegmentM": float(max(0.0, min_segment_m or 0.0)),
        }
    )
    if sub_index is not None:
        review["subIndex"] = int(sub_index)
    if isinstance(source_piece_data, dict):
        review["fromPieceCoordinateList"] = _clone_coord_list(source_piece_data.get("coordinateList"))
        review["fromPieceRawCoordinateList"] = _clone_coord_list(source_piece_data.get("rawCoordinateList"))
    data["reviewArea"] = review


def _subdivide_piece_by_local_assignment(
    piece: SplitPiece,
    uav_coord: Dict[str, float],
    max_segment_m: float,
    *,
    max_split_count: int = 0,
    min_segment_m: float = 0.0,
) -> Tuple[List[SplitPiece], Dict[str, Any]]:
    data = piece.data if isinstance(piece.data, dict) else {}
    coords = data.get("coordinateList")
    if not isinstance(coords, list) or len(coords) < 3:
        return [piece], {"changed": False, "reason": "invalid_polygon_coords"}

    center = _piece_centroid_coord(piece)
    if center is None:
        return [piece], {"changed": False, "reason": "invalid_polygon_geom"}

    local_bearing_deg = float(sa._bearing_deg(uav_coord, center))
    projected_span_m = _piece_projection_span_m(piece, local_bearing_deg)
    split_count, raw_split_count, split_capped, split_cap_reason = _resolve_review_split_count(
        projected_span_m,
        max_segment_m,
        max_split_count=max_split_count,
        min_segment_m=min_segment_m,
    )

    _apply_replan_local_bearing(
        data,
        local_bearing_deg,
        uav_coord,
        split_count=split_count,
        projected_span_m=projected_span_m,
        max_segment_m=max_segment_m,
        piece_index=int(piece.piece_index),
        raw_split_count=raw_split_count,
        split_capped=split_capped,
        split_cap_reason=split_cap_reason,
        max_split_count=max_split_count,
        min_segment_m=min_segment_m,
    )

    if split_count <= 1:
        return [piece], {
            "changed": False,
            "localized": True,
            "splitCount": int(split_count),
            "rawSplitCount": int(raw_split_count),
            "splitCapped": bool(split_capped),
            "splitCapReason": str(split_cap_reason),
            "maxSplitCountLimit": int(max(0, int(max_split_count or 0))),
            "minSegmentM": float(max(0.0, min_segment_m or 0.0)),
            "projectedSpanM": float(projected_span_m),
            "axisBearingDeg": float(local_bearing_deg),
        }

    lat0 = _to_float(coords[0].get("latitude"))
    lon0 = _to_float(coords[0].get("longitude"))
    try:
        parts = sa.divide_search_area_clip(
            coords,
            split_count,
            float(local_bearing_deg),
            ref_lat0=lat0,
            ref_lon0=lon0,
        )
    except Exception:
        return [piece], {
            "changed": False,
            "localized": True,
            "reason": "split_failed",
            "splitCount": int(split_count),
            "rawSplitCount": int(raw_split_count),
            "splitCapped": bool(split_capped),
            "splitCapReason": str(split_cap_reason),
            "maxSplitCountLimit": int(max(0, int(max_split_count or 0))),
            "minSegmentM": float(max(0.0, min_segment_m or 0.0)),
            "projectedSpanM": float(projected_span_m),
            "axisBearingDeg": float(local_bearing_deg),
        }

    parts = _sort_parts_near_uav(parts, uav_coord, local_bearing_deg, lat0, lon0)
    if len(parts) <= 1:
        return [piece], {
            "changed": False,
            "localized": True,
            "reason": "split_not_effective",
            "splitCount": int(split_count),
            "rawSplitCount": int(raw_split_count),
            "splitCapped": bool(split_capped),
            "splitCapReason": str(split_cap_reason),
            "maxSplitCountLimit": int(max(0, int(max_split_count or 0))),
            "minSegmentM": float(max(0.0, min_segment_m or 0.0)),
            "projectedSpanM": float(projected_span_m),
            "axisBearingDeg": float(local_bearing_deg),
        }

    new_pieces: List[SplitPiece] = []
    for idx, part in enumerate(parts, start=1):
        new_data = copy.deepcopy(piece.data)
        new_data["coordinateList"] = _clone_coord_list(part.get("coordinateList"))
        new_data["rawCoordinateList"] = _clone_coord_list(part.get("rawCoordinateList"))
        if "postProcess" in part:
            new_data["postProcess"] = copy.deepcopy(part.get("postProcess"))
        if "meanAltitude" in part:
            new_data["meanAltitude"] = part.get("meanAltitude")
        if "altitudeVariance" in part:
            new_data["altitudeVariance"] = part.get("altitudeVariance")
        _apply_replan_local_bearing(
            new_data,
            local_bearing_deg,
            uav_coord,
            split_count=split_count,
            projected_span_m=projected_span_m,
            max_segment_m=max_segment_m,
            piece_index=int(piece.piece_index),
            sub_index=int(idx),
            source_piece_data=data,
            raw_split_count=raw_split_count,
            split_capped=split_capped,
            split_cap_reason=split_cap_reason,
            max_split_count=max_split_count,
            min_segment_m=min_segment_m,
        )
        new_pieces.append(
            SplitPiece(
                parent_order=int(piece.parent_order),
                mission_id=piece.mission_id,
                mission_type=int(piece.mission_type),
                piece_index=int(piece.piece_index),
                data=new_data,
                assigned_uav=piece.assigned_uav,
            )
        )

    return new_pieces, {
        "changed": True,
        "localized": True,
        "splitCount": int(split_count),
        "rawSplitCount": int(raw_split_count),
        "splitCapped": bool(split_capped),
        "splitCapReason": str(split_cap_reason),
        "maxSplitCountLimit": int(max(0, int(max_split_count or 0))),
        "minSegmentM": float(max(0.0, min_segment_m or 0.0)),
        "projectedSpanM": float(projected_span_m),
        "segmentLenM": float(projected_span_m / float(max(1, split_count))),
        "axisBearingDeg": float(local_bearing_deg),
        "newPieceCount": int(len(new_pieces)),
    }


def review_assigned_areas_local(
    split_result: SplitRunResult,
    mrpk: Dict[str, Any],
    max_segment_m: float = 3000.0,
    *,
    max_split_count: int = 0,
    min_segment_m: float = 0.0,
) -> Dict[str, Any]:
    if split_result is None:
        return {
            "changed": False,
            "targets": 0,
            "oldPieceCount": 0,
            "newPieceCount": 0,
            "details": [],
        }

    takeover_map = _takeover_map(mrpk)
    replacements: Dict[Tuple[int, int], List[SplitPiece]] = {}
    details: List[Dict[str, Any]] = []
    targets = 0
    localized = 0

    for piece in split_result.pieces:
        if int(piece.mission_type) not in _AREA_TYPES:
            continue
        aid = int(piece.assigned_uav or 0)
        detail = {
            "parentOrder": int(piece.parent_order),
            "pieceIndex": int(piece.piece_index),
            "splitStage": int((piece.data or {}).get("splitStage", 0) or 0),
            "assignedUav": int(aid),
        }
        if aid <= 0:
            detail.update({"changed": False, "reason": "unassigned_piece"})
            details.append(detail)
            continue
        uav_coord = takeover_map.get(aid)
        if uav_coord is None:
            detail.update({"changed": False, "reason": "missing_takeover"})
            details.append(detail)
            continue

        targets += 1
        new_pieces, meta = _subdivide_piece_by_local_assignment(
            piece,
            uav_coord=uav_coord,
            max_segment_m=max_segment_m,
            max_split_count=max(0, int(max_split_count or 0)),
            min_segment_m=max(0.0, float(min_segment_m or 0.0)),
        )
        if meta.get("localized"):
            localized += 1
        if meta.get("changed"):
            replacements[(int(piece.parent_order), int(piece.piece_index))] = new_pieces
        detail.update(meta)
        details.append(detail)

    old_count = len(split_result.pieces)
    if replacements:
        split_result.pieces = _rebuild_pieces_with_replacements(split_result.pieces, replacements)
    new_count = len(split_result.pieces)
    return {
        "changed": bool(replacements or localized > 0),
        "targets": int(targets),
        "localized": int(localized),
        "oldPieceCount": int(old_count),
        "newPieceCount": int(new_count),
        "details": details,
        "mode": "replan_local_assigned",
    }


def review_overflow_areas(
    split_result: SplitRunResult,
    expected_paths: List[Dict[str, Any]],
    max_segment_m: float = 3000.0,
    max_split_count: int = 0,
    min_segment_m: float = 0.0,
) -> Dict[str, Any]:
    if _runtime_area_sweep_mode() == "nadir":
        return {
            "changed": False,
            "overflowRows": 0,
            "targets": 0,
            "oldPieceCount": len(split_result.pieces) if split_result is not None else 0,
            "newPieceCount": len(split_result.pieces) if split_result is not None else 0,
            "details": [],
            "skipped": True,
            "reason": "nadir_mode",
        }
    if split_result is None:
        return {
            "changed": False,
            "overflowRows": 0,
            "targets": 0,
            "oldPieceCount": 0,
            "newPieceCount": 0,
            "details": [],
        }

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

    def _pick_dir(piece: SplitPiece) -> Optional[DirectionDebug]:
        parent = int(piece.parent_order)
        sidx = _piece_source_area_index(piece)
        if sidx is not None:
            d = dir_by_area.get((parent, int(sidx)))
            if d is not None:
                return d
        return dir_by_parent.get(parent)
    piece_map: Dict[Tuple[int, int], SplitPiece] = {
        (int(p.parent_order), int(p.piece_index)): p
        for p in split_result.pieces
        if int(p.mission_type) in _AREA_TYPES
    }

    overflow_rows = [r for r in expected_paths if isinstance(r, dict) and _is_overflow_area_row(r)]
    if not overflow_rows:
        return {
            "changed": False,
            "overflowRows": 0,
            "targets": 0,
            "oldPieceCount": len(split_result.pieces),
            "newPieceCount": len(split_result.pieces),
            "details": [],
        }

    replacements: Dict[Tuple[int, int], List[SplitPiece]] = {}
    details: List[Dict[str, Any]] = []

    for row in overflow_rows:
        parent = int(row.get("parentOrder", 0) or 0)
        source = str(row.get("source", ""))
        coords = row.get("coordinateList")
        if not isinstance(coords, list):
            continue

        if source == "area_section_chain_by_split_axis":
            pair = row.get("pairPieces")
            if not (isinstance(pair, list) and len(pair) >= 2 and len(coords) >= 3):
                continue

            key1 = (parent, int(pair[0]))
            key2 = (parent, int(pair[1]))
            if key1 in replacements or key2 in replacements:
                continue

            piece1 = piece_map.get(key1)
            piece2 = piece_map.get(key2)
            if piece1 is None or piece2 is None:
                continue
            pt1 = _piece_pattern_type(piece1)
            pt2 = _piece_pattern_type(piece2)
            if pt1 != 6 and pt2 != 6:
                details.append(
                    {
                        "parentOrder": int(parent),
                        "source": source,
                        "pairPieces": [int(pair[0]), int(pair[1])],
                        "changed": False,
                        "reason": "skip_patternType_not_6",
                        "patternTypes": [int(pt1), int(pt2)],
                    }
                )
                continue

            seg1_s, seg1_e = coords[0], coords[1]
            seg2_s, seg2_e = coords[1], coords[2]
            len1 = _segment_len_m_llh(seg1_s, seg1_e)
            len2 = _segment_len_m_llh(seg2_s, seg2_e)

            dbg1 = _pick_dir(piece1)
            dbg2 = _pick_dir(piece2)
            axis1 = _axis_bearing_deg(piece1, dbg1)
            axis2 = _axis_bearing_deg(piece2, dbg2)

            # If only one side is patternType==6, split that side only.
            if pt1 == 6 and pt2 != 6:
                within1, span1 = _piece_within_segment_limit(
                    piece1,
                    axis_bearing_deg=axis1,
                    max_segment_m=max_segment_m,
                )
                if within1:
                    details.append(
                        {
                            "parentOrder": int(piece1.parent_order),
                            "pieceIndex": int(piece1.piece_index),
                            "splitStage": int((piece1.data or {}).get("splitStage", 0) or 0),
                            "source": source,
                            "changed": False,
                            "reason": "skip_piece_span_within_limit",
                            "patternType": int(pt1),
                            "axisBearingDeg": float(axis1),
                            "projectedSpanM": float(span1),
                            "maxSegmentM": float(max_segment_m),
                        }
                    )
                    details.append(
                        {
                            "parentOrder": int(piece2.parent_order),
                            "pieceIndex": int(piece2.piece_index),
                            "splitStage": int((piece2.data or {}).get("splitStage", 0) or 0),
                            "source": source,
                            "changed": False,
                            "reason": "skip_patternType_not_6",
                            "patternType": int(pt2),
                        }
                    )
                    continue
                new1, meta1 = _subdivide_piece_by_segment(
                    piece1,
                    seg_start=seg1_s,
                    seg_end=seg1_e,
                    axis_bearing_deg=axis1,
                    max_segment_m=max_segment_m,
                    max_split_count=max_split_count,
                    min_segment_m=min_segment_m,
                )
                if meta1.get("changed"):
                    replacements[key1] = new1
                d1 = {
                    "parentOrder": int(piece1.parent_order),
                    "pieceIndex": int(piece1.piece_index),
                    "splitStage": int((piece1.data or {}).get("splitStage", 0) or 0),
                    "source": source,
                    "axisBearingDeg": float(axis1),
                    "pairSegmentLenM": float(len1),
                    "patternType": int(pt1),
                }
                d1.update(meta1)
                details.append(d1)
                details.append(
                    {
                        "parentOrder": int(piece2.parent_order),
                        "pieceIndex": int(piece2.piece_index),
                        "splitStage": int((piece2.data or {}).get("splitStage", 0) or 0),
                        "source": source,
                        "changed": False,
                        "reason": "skip_patternType_not_6",
                        "patternType": int(pt2),
                    }
                )
                continue

            if pt1 != 6 and pt2 == 6:
                within2, span2 = _piece_within_segment_limit(
                    piece2,
                    axis_bearing_deg=axis2,
                    max_segment_m=max_segment_m,
                )
                if within2:
                    details.append(
                        {
                            "parentOrder": int(piece1.parent_order),
                            "pieceIndex": int(piece1.piece_index),
                            "splitStage": int((piece1.data or {}).get("splitStage", 0) or 0),
                            "source": source,
                            "changed": False,
                            "reason": "skip_patternType_not_6",
                            "patternType": int(pt1),
                        }
                    )
                    details.append(
                        {
                            "parentOrder": int(piece2.parent_order),
                            "pieceIndex": int(piece2.piece_index),
                            "splitStage": int((piece2.data or {}).get("splitStage", 0) or 0),
                            "source": source,
                            "changed": False,
                            "reason": "skip_piece_span_within_limit",
                            "patternType": int(pt2),
                            "axisBearingDeg": float(axis2),
                            "projectedSpanM": float(span2),
                            "maxSegmentM": float(max_segment_m),
                        }
                    )
                    continue
                new2, meta2 = _subdivide_piece_by_segment(
                    piece2,
                    seg_start=seg2_s,
                    seg_end=seg2_e,
                    axis_bearing_deg=axis2,
                    max_segment_m=max_segment_m,
                    max_split_count=max_split_count,
                    min_segment_m=min_segment_m,
                )
                if meta2.get("changed"):
                    replacements[key2] = new2
                details.append(
                    {
                        "parentOrder": int(piece1.parent_order),
                        "pieceIndex": int(piece1.piece_index),
                        "splitStage": int((piece1.data or {}).get("splitStage", 0) or 0),
                        "source": source,
                        "changed": False,
                        "reason": "skip_patternType_not_6",
                        "patternType": int(pt1),
                    }
                )
                d2 = {
                    "parentOrder": int(piece2.parent_order),
                    "pieceIndex": int(piece2.piece_index),
                    "splitStage": int((piece2.data or {}).get("splitStage", 0) or 0),
                    "source": source,
                    "axisBearingDeg": float(axis2),
                    "pairSegmentLenM": float(len2),
                    "patternType": int(pt2),
                }
                d2.update(meta2)
                details.append(d2)
                continue

            split_n, raw_split_n, split_capped, split_cap_reason = _resolve_review_split_count(
                max(len1, len2),
                max_segment_m,
                max_split_count=max_split_count,
                min_segment_m=min_segment_m,
            )
            within1, span1 = _piece_within_segment_limit(
                piece1,
                axis_bearing_deg=axis1,
                max_segment_m=max_segment_m,
            )
            within2, span2 = _piece_within_segment_limit(
                piece2,
                axis_bearing_deg=axis2,
                max_segment_m=max_segment_m,
            )
            if within1 and within2:
                details.append(
                    {
                        "parentOrder": int(piece1.parent_order),
                        "pieceIndex": int(piece1.piece_index),
                        "splitStage": int((piece1.data or {}).get("splitStage", 0) or 0),
                        "source": source,
                        "changed": False,
                        "reason": "skip_piece_span_within_limit",
                        "axisBearingDeg": float(axis1),
                        "pairSegmentLenM": float(len1),
                        "patternType": int(pt1),
                        "projectedSpanM": float(span1),
                        "maxSegmentM": float(max_segment_m),
                    }
                )
                details.append(
                    {
                        "parentOrder": int(piece2.parent_order),
                        "pieceIndex": int(piece2.piece_index),
                        "splitStage": int((piece2.data or {}).get("splitStage", 0) or 0),
                        "source": source,
                        "changed": False,
                        "reason": "skip_piece_span_within_limit",
                        "axisBearingDeg": float(axis2),
                        "pairSegmentLenM": float(len2),
                        "patternType": int(pt2),
                        "projectedSpanM": float(span2),
                        "maxSegmentM": float(max_segment_m),
                    }
                )
                continue

            new1, meta1 = _subdivide_piece_by_segment(
                piece1,
                seg_start=seg1_s,
                seg_end=seg1_e,
                axis_bearing_deg=axis1,
                max_segment_m=max_segment_m,
                split_count=split_n,
                max_split_count=max_split_count,
                min_segment_m=min_segment_m,
            )
            new2, meta2 = _subdivide_piece_by_segment(
                piece2,
                seg_start=seg2_s,
                seg_end=seg2_e,
                axis_bearing_deg=axis2,
                max_segment_m=max_segment_m,
                split_count=split_n,
                max_split_count=max_split_count,
                min_segment_m=min_segment_m,
            )

            # Prefer paired metadata when both sides succeed; otherwise keep changed side only.
            if meta1.get("changed") and meta2.get("changed"):
                pair_id = f"{parent}:{int(pair[0])}:{int(pair[1])}"
                n1 = len(new1)
                n2 = len(new2)
                for j, p in enumerate(new1, start=1):
                    d = p.data if isinstance(p.data, dict) else {}
                    rd = d.get("reviewArea") if isinstance(d.get("reviewArea"), dict) else {}
                    rd["pairID"] = pair_id
                    rd["pairSubIndex"] = int(max(1, n1 - j + 1))  # near-kink first on S1 side
                    d["reviewArea"] = rd
                for j, p in enumerate(new2, start=1):
                    d = p.data if isinstance(p.data, dict) else {}
                    rd = d.get("reviewArea") if isinstance(d.get("reviewArea"), dict) else {}
                    rd["pairID"] = pair_id
                    rd["pairSubIndex"] = int(j)  # near-kink first on S2 side
                    d["reviewArea"] = rd
                replacements[key1] = new1
                replacements[key2] = new2
            else:
                if meta1.get("changed"):
                    replacements[key1] = new1
                if meta2.get("changed"):
                    replacements[key2] = new2

            d1 = {
                "parentOrder": int(piece1.parent_order),
                "pieceIndex": int(piece1.piece_index),
                "splitStage": int((piece1.data or {}).get("splitStage", 0) or 0),
                "source": source,
                "axisBearingDeg": float(axis1),
                "pairSplitCount": int(split_n),
                "pairRawSplitCount": int(raw_split_n),
                "pairSplitCapped": bool(split_capped),
                "pairSplitCapReason": split_cap_reason,
                "pairSegmentLenM": float(len1),
                "patternType": int(pt1),
            }
            d1.update(meta1)
            details.append(d1)

            d2 = {
                "parentOrder": int(piece2.parent_order),
                "pieceIndex": int(piece2.piece_index),
                "splitStage": int((piece2.data or {}).get("splitStage", 0) or 0),
                "source": source,
                "axisBearingDeg": float(axis2),
                "pairSplitCount": int(split_n),
                "pairRawSplitCount": int(raw_split_n),
                "pairSplitCapped": bool(split_capped),
                "pairSplitCapReason": split_cap_reason,
                "pairSegmentLenM": float(len2),
                "patternType": int(pt2),
            }
            d2.update(meta2)
            details.append(d2)
        elif source == "area_single_section_line":
            if len(coords) < 2:
                continue
            key = (parent, int(row.get("index", 0) or 0))
            if key in replacements:
                continue
            piece = piece_map.get(key)
            if piece is None:
                continue
            pt = _piece_pattern_type(piece)
            if pt != 6:
                details.append(
                    {
                        "parentOrder": int(parent),
                        "pieceIndex": int(row.get("index", 0) or 0),
                        "source": source,
                        "changed": False,
                        "reason": "skip_patternType_not_6",
                        "patternType": int(pt),
                    }
                )
                continue
            dbg = _pick_dir(piece)
            axis = _axis_bearing_deg(piece, dbg)
            within, span = _piece_within_segment_limit(
                piece,
                axis_bearing_deg=axis,
                max_segment_m=max_segment_m,
            )
            if within:
                details.append(
                    {
                        "parentOrder": int(piece.parent_order),
                        "pieceIndex": int(piece.piece_index),
                        "splitStage": int((piece.data or {}).get("splitStage", 0) or 0),
                        "source": source,
                        "changed": False,
                        "reason": "skip_piece_span_within_limit",
                        "axisBearingDeg": float(axis),
                        "projectedSpanM": float(span),
                        "maxSegmentM": float(max_segment_m),
                        "patternType": int(pt),
                    }
                )
                continue
            new_pieces, meta = _subdivide_piece_by_segment(
                piece,
                seg_start=coords[0],
                seg_end=coords[1],
                axis_bearing_deg=axis,
                max_segment_m=max_segment_m,
                max_split_count=max_split_count,
                min_segment_m=min_segment_m,
            )
            if meta.get("changed"):
                replacements[key] = new_pieces
            detail = {
                "parentOrder": int(piece.parent_order),
                "pieceIndex": int(piece.piece_index),
                "splitStage": int((piece.data or {}).get("splitStage", 0) or 0),
                "source": source,
                "axisBearingDeg": float(axis),
            }
            detail.update(meta)
            details.append(detail)
        else:
            continue

    if not replacements:
        return {
            "changed": False,
            "overflowRows": len(overflow_rows),
            "targets": 0,
            "oldPieceCount": len(split_result.pieces),
            "newPieceCount": len(split_result.pieces),
            "details": details,
        }

    old_count = len(split_result.pieces)
    split_result.pieces = _rebuild_pieces_with_replacements(split_result.pieces, replacements)
    new_count = len(split_result.pieces)
    return {
        "changed": True,
        "overflowRows": len(overflow_rows),
        "targets": len(replacements),
        "oldPieceCount": old_count,
        "newPieceCount": new_count,
        "details": details,
    }

