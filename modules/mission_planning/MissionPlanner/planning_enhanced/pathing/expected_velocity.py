from __future__ import annotations

import math
import threading
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from shapely.geometry import Polygon

from ..models import SplitPiece, SplitRunResult
try:
    from ...runtime_settings import (
        fov_db_path as runtime_fov_db_path,
        read_fov_db_rows_from_path,
    )
except Exception:
    from modules.mission_planning.MissionPlanner.runtime_settings import (  # type: ignore
        fov_db_path as runtime_fov_db_path,
        read_fov_db_rows_from_path,
    )


_R = 6_378_137.0
_LINE_TYPES = {1, 7}
_AREA_TYPES = {2, 3, 4, 5, 6}
_FOV_DB_CACHE_LOCK = threading.Lock()
_FOV_DB_CACHE_SIG: Tuple[str, int, int] | None = None
_FOV_DB_CACHE_ROWS: List[Dict[str, float]] | None = None


def _db_sig(path: Path) -> Tuple[str, int, int] | None:
    try:
        resolved = Path(path).resolve()
        stat = resolved.stat()
    except Exception:
        return None
    return str(resolved), int(stat.st_mtime_ns), int(stat.st_size)


def _to_float(v: Any, default: float = 0.0) -> float:
    try:
        return float(v)
    except Exception:
        return default


def _to_int(v: Any, default: int = 0) -> int:
    try:
        return int(v)
    except Exception:
        return default


def _llh_to_xy(lat: float, lon: float, lat0: float, lon0: float) -> Tuple[float, float]:
    lat0_r = math.radians(lat0)
    x = math.radians(lon - lon0) * _R * math.cos(lat0_r)
    y = math.radians(lat - lat0) * _R
    return x, y


def _bearing_unit_xy(bearing_deg: float) -> Tuple[float, float]:
    th = math.radians(float(bearing_deg))
    return math.sin(th), math.cos(th)


def _coords_to_xy(coords: List[Dict[str, Any]]) -> Tuple[List[Tuple[float, float]], Optional[Tuple[float, float]]]:
    if not isinstance(coords, list) or len(coords) < 2:
        return [], None
    lat0 = _to_float(coords[0].get("latitude"))
    lon0 = _to_float(coords[0].get("longitude"))
    out: List[Tuple[float, float]] = []
    for c in coords:
        if not isinstance(c, dict):
            continue
        out.append(
            _llh_to_xy(
                _to_float(c.get("latitude")),
                _to_float(c.get("longitude")),
                lat0,
                lon0,
            )
        )
    return out, (lat0, lon0)


def _polyline_len_m(coords: Any) -> float:
    if not isinstance(coords, list) or len(coords) < 2:
        return 0.0
    xy, _ = _coords_to_xy(coords)
    if len(xy) < 2:
        return 0.0
    total = 0.0
    for i in range(len(xy) - 1):
        total += math.hypot(xy[i + 1][0] - xy[i][0], xy[i + 1][1] - xy[i][1])
    return float(total)


def _projection_span(points_xy: List[Tuple[float, float]], bearing_deg: float) -> float:
    if len(points_xy) < 2:
        return 0.0
    ux, uy = _bearing_unit_xy(bearing_deg)
    vals = [p[0] * ux + p[1] * uy for p in points_xy]
    return float(max(vals) - min(vals)) if vals else 0.0


def _polygon_short_long_spans(coords: Any) -> Tuple[float, float]:
    if not isinstance(coords, list) or len(coords) < 3:
        return 0.0, 0.0
    xy, _ = _coords_to_xy(coords)
    if len(xy) < 3:
        return 0.0, 0.0
    poly = Polygon(xy)
    if not poly.is_valid:
        poly = poly.buffer(0)
    if poly.is_empty:
        return 0.0, 0.0
    mrr = poly.minimum_rotated_rectangle
    if mrr.is_empty:
        return 0.0, 0.0
    pts = list(mrr.exterior.coords)
    if len(pts) < 4:
        return 0.0, 0.0
    edges: List[float] = []
    for i in range(len(pts) - 1):
        edges.append(math.hypot(float(pts[i + 1][0] - pts[i][0]), float(pts[i + 1][1] - pts[i][1])))
    if not edges:
        return 0.0, 0.0
    return float(min(edges)), float(max(edges))


def _piece_is_line(piece: SplitPiece) -> bool:
    data = piece.data if isinstance(piece.data, dict) else {}
    center = data.get("Centerline")
    if isinstance(center, list) and len(center) >= 2:
        return True
    return int(piece.mission_type) in _LINE_TYPES


def _piece_is_area(piece: SplitPiece) -> bool:
    return int(piece.mission_type) in _AREA_TYPES


def _build_path_lookup(expected_paths: List[Dict[str, Any]]) -> Dict[Tuple[int, int], Dict[str, Any]]:
    lookup: Dict[Tuple[int, int], Dict[str, Any]] = {}
    for row in expected_paths:
        if not isinstance(row, dict):
            continue
        parent = _to_int(row.get("parentOrder"), 0)
        if parent <= 0:
            continue
        src = str(row.get("source", ""))
        if src == "area_section_chain_by_split_axis":
            pair = row.get("pairPieces")
            if isinstance(pair, list) and len(pair) >= 2:
                p1 = _to_int(pair[0], 0)
                p2 = _to_int(pair[1], 0)
                if p1 > 0:
                    lookup[(parent, p1)] = {"row": row, "role": "pair_s1"}
                if p2 > 0:
                    lookup[(parent, p2)] = {"row": row, "role": "pair_s2"}
            continue
        idx = _to_int(row.get("index"), 0)
        if idx > 0 and (parent, idx) not in lookup:
            lookup[(parent, idx)] = {"row": row, "role": "direct"}
    return lookup


def _piece_distance_m(
    piece: SplitPiece,
    path_info: Optional[Dict[str, Any]],
) -> float:
    data = piece.data if isinstance(piece.data, dict) else {}
    if _piece_is_line(piece):
        if isinstance(path_info, dict):
            row = path_info.get("row")
            if isinstance(row, dict):
                d = _polyline_len_m(row.get("coordinateList"))
                if d > 0.0:
                    return d
        center = data.get("Centerline")
        if isinstance(center, list) and len(center) >= 2:
            return _polyline_len_m(center)
        coords = data.get("coordinateList")
        if isinstance(coords, list) and len(coords) >= 2:
            return _polyline_len_m(coords)
        return 0.0

    # Area time is estimated on the long-side travel length.
    coords = data.get("coordinateList")
    short_span, long_span = _polygon_short_long_spans(coords)
    if long_span > 0.0:
        return long_span
    return short_span


def _piece_width_ref_m(
    piece: SplitPiece,
    path_info: Optional[Dict[str, Any]],
) -> Tuple[float, str]:
    data = piece.data if isinstance(piece.data, dict) else {}
    if _piece_is_line(piece):
        w = _to_float(data.get("width"), 0.0)
        if w > 0.0:
            return w, "line_width"
        if isinstance(path_info, dict):
            row = path_info.get("row")
            if isinstance(row, dict):
                rw = _to_float(row.get("widthRefM"), 0.0)
                if rw > 0.0:
                    return rw, "line_path_widthRef"
        coords = data.get("coordinateList")
        short_span, _ = _polygon_short_long_spans(coords)
        if short_span > 0.0:
            return short_span, "line_poly_short_span"
        return 1.0, "line_fallback"

    if _piece_is_area(piece):
        coords = data.get("coordinateList")
        review = data.get("reviewArea") if isinstance(data.get("reviewArea"), dict) else {}
        if isinstance(review, dict) and bool(review.get("subdivided", False)):
            axis = _to_float(review.get("axisBearingDeg"), float("nan"))
            if math.isfinite(axis):
                xy, _ = _coords_to_xy(coords if isinstance(coords, list) else [])
                if len(xy) >= 3:
                    # Use review split progression direction (axis + 90), not the cut-face direction.
                    split_bearing = (axis + 90.0) % 360.0
                    w = _projection_span(xy, split_bearing)
                    if w > 0.0:
                        return w, "area_review_split_dir"
        if isinstance(path_info, dict):
            row = path_info.get("row")
            if isinstance(row, dict):
                rw = _to_float(row.get("widthRefM"), 0.0)
                if rw > 0.0:
                    return rw, "area_path_widthRef"
        short_span, _ = _polygon_short_long_spans(coords)
        if short_span > 0.0:
            return short_span, "area_short_span"
        return 1.0, "area_fallback"

    return 1.0, "unknown_fallback"


def _load_fov_db(path: Path) -> List[Dict[str, float]]:
    global _FOV_DB_CACHE_SIG, _FOV_DB_CACHE_ROWS
    sig = _db_sig(path)
    if sig is not None:
        with _FOV_DB_CACHE_LOCK:
            if _FOV_DB_CACHE_SIG == sig and isinstance(_FOV_DB_CACHE_ROWS, list):
                return _FOV_DB_CACHE_ROWS

    out: List[Dict[str, float]] = []
    with _FOV_DB_CACHE_LOCK:
        if sig is not None and _FOV_DB_CACHE_SIG == sig and isinstance(_FOV_DB_CACHE_ROWS, list):
            return _FOV_DB_CACHE_ROWS
        for row in read_fov_db_rows_from_path(path):
            w = _to_float(row.get("width"), -1.0)
            v = _to_float(row.get("vel"), -1.0)
            if w > 0.0 and v > 0.0:
                out.append({"width": w, "vel": v})
        if sig is not None:
            _FOV_DB_CACHE_SIG = sig
            _FOV_DB_CACHE_ROWS = out
    return out


def _candidate_vel_range(db_rows: List[Dict[str, float]], width_ref_m: float) -> Tuple[List[float], float, float]:
    req = max(0.0, float(width_ref_m))
    if req <= 0.0:
        return [], 0.0, 0.0
    cands = [r for r in db_rows if float(r["width"]) + 1e-9 >= req]
    if not cands:
        return [], 0.0, 0.0
    vel_list = sorted(set(float(r["vel"]) for r in cands))
    if not vel_list:
        return [], 0.0, 0.0
    return vel_list, float(min(vel_list)), float(max(vel_list))


def _candidate_vel_range_nearest_width(
    db_rows: List[Dict[str, float]],
    width_ref_m: float,
) -> Tuple[List[float], float, float]:
    req = max(0.0, float(width_ref_m))
    if req <= 0.0 or not db_rows:
        return [], 0.0, 0.0
    nearest_w = min((float(r["width"]) for r in db_rows), key=lambda w: abs(w - req))
    cands = [r for r in db_rows if abs(float(r["width"]) - nearest_w) <= 1e-9]
    vel_list = sorted(set(float(r["vel"]) for r in cands))
    if not vel_list:
        return [], 0.0, 0.0
    return vel_list, float(min(vel_list)), float(max(vel_list))


def _is_review_subdivided_area(piece: SplitPiece) -> bool:
    if not _piece_is_area(piece):
        return False
    data = piece.data if isinstance(piece.data, dict) else {}
    review = data.get("reviewArea")
    return isinstance(review, dict) and bool(review.get("subdivided", False))


def _piece_pattern_type(piece: SplitPiece) -> int:
    data = piece.data if isinstance(piece.data, dict) else {}
    try:
        return int(data.get("patternType", 0) or 0)
    except Exception:
        return 0


def _area_time_group_key(piece: SplitPiece) -> Optional[Tuple[int, int]]:
    if not _piece_is_area(piece):
        return None
    parent = int(piece.parent_order)
    data = piece.data if isinstance(piece.data, dict) else {}
    review = data.get("reviewArea")
    if isinstance(review, dict) and bool(review.get("subdivided", False)):
        try:
            base_idx = int(review.get("fromPieceIndex", 0) or 0)
        except Exception:
            base_idx = 0
        if base_idx > 0:
            return (parent, base_idx)
    return (parent, int(piece.piece_index))


def _virtual_area_scan_distance_m(
    piece: SplitPiece,
    split_width_m: float = 50.0,
) -> Optional[Dict[str, float]]:
    if not _piece_is_area(piece):
        return None
    data = piece.data if isinstance(piece.data, dict) else {}
    coords = data.get("coordinateList")
    short_span, long_span = _polygon_short_long_spans(coords)
    if short_span <= 0.0 or long_span <= 0.0:
        return None
    strip_n = max(1, int(math.ceil(short_span / float(max(split_width_m, 1e-6)))))
    dist_m = float(long_span * float(strip_n))
    return {
        "shortSpanM": float(short_span),
        "longSpanM": float(long_span),
        "virtualSplitWidthM": float(split_width_m),
        "virtualStripCount": float(strip_n),
        "virtualDistanceM": float(dist_m),
    }


def _time_range_sec(distance_m: float, vel_min_kmh: float, vel_max_kmh: float) -> Tuple[Optional[float], Optional[float]]:
    d = max(0.0, float(distance_m))
    v0 = max(0.0, float(vel_min_kmh))
    v1 = max(0.0, float(vel_max_kmh))
    if d <= 0.0 or v0 <= 0.0 or v1 <= 0.0:
        return None, None
    t_fast_sec = d / (v1 * 1000.0 / 3600.0)
    t_slow_sec = d / (v0 * 1000.0 / 3600.0)
    return float(t_fast_sec), float(t_slow_sec)


def _time_single_sec(distance_m: float, vel_kmh: float) -> Optional[float]:
    d = max(0.0, float(distance_m))
    v = max(0.0, float(vel_kmh))
    if d <= 0.0 or v <= 0.0:
        return None
    t_sec = d / (v * 1000.0 / 3600.0)
    return float(t_sec)


def calculate_expected_velocity(
    split_result: SplitRunResult,
    expected_paths: Optional[List[Dict[str, Any]]] = None,
    db_csv_path: Optional[str | Path] = None,
) -> Dict[str, Any]:
    db_path = Path(db_csv_path) if db_csv_path is not None else runtime_fov_db_path()
    db_rows = _load_fov_db(db_path)
    db_max_width = max((float(r["width"]) for r in db_rows), default=0.0)
    path_lookup = _build_path_lookup(expected_paths or [])

    report_rows: List[Dict[str, Any]] = []
    piece_payload_rows: List[Tuple[SplitPiece, Dict[str, Any], Dict[str, Any]]] = []
    for piece in sorted(split_result.pieces, key=lambda p: (int(p.parent_order), int(p.piece_index))):
        key = (int(piece.parent_order), int(piece.piece_index))
        path_info = path_lookup.get(key)
        width_ref_m, width_source = _piece_width_ref_m(piece, path_info)
        distance_m = _piece_distance_m(piece, path_info)
        is_review_subdiv = _is_review_subdivided_area(piece)
        pattern_type = _piece_pattern_type(piece)
        vel_candidates, vel_min, vel_max = _candidate_vel_range(db_rows, width_ref_m)
        vel_approx = False
        vel_source = "width_ge"
        virtual_meta: Dict[str, float] = {}

        # For area pattern 3/4 (not review-subdivided), estimate mission time
        # on virtual 50m strip decomposition with fixed 140km/h.
        if _piece_is_area(piece) and (pattern_type in (3, 4)) and (not is_review_subdiv):
            vm = _virtual_area_scan_distance_m(piece, split_width_m=50.0)
            if vm is not None:
                virtual_meta = vm
                distance_m = float(vm["virtualDistanceM"])
                vel_candidates = [140.0]
                vel_min = 140.0
                vel_max = 140.0
                vel_source = "area_virtual_split50_fixed140"

        if not vel_candidates and is_review_subdiv:
            # Requested policy:
            # for review-subdivided area only, use nearest-width DB row as approximate velocity.
            vel_candidates, vel_min, vel_max = _candidate_vel_range_nearest_width(db_rows, width_ref_m)
            vel_approx = bool(vel_candidates)
            if vel_approx:
                vel_source = "nearest_width_approx"
        t_min_s, t_max_s = _time_range_sec(distance_m, vel_min, vel_max)
        v_sel = float(vel_max) if vel_max > 0.0 else 0.0
        t_sel_s = _time_single_sec(distance_m, v_sel)
        geom = "line" if _piece_is_line(piece) else ("area" if _piece_is_area(piece) else "unknown")

        vel_payload = {
            "dbPath": str(db_path),
            "widthRefM": float(width_ref_m),
            "widthSource": str(width_source),
            "distanceRefM": float(distance_m),
            "candidateCount": int(len(vel_candidates)),
            "velCandidatesKmh": [float(v) for v in vel_candidates],
            "velMinKmh": float(vel_min),
            "velMaxKmh": float(vel_max),
            "velSelectedKmh": float(v_sel),
            "velApprox": bool(vel_approx),
            "velSource": str(vel_source),
            "timeMinSec": float(t_min_s) if t_min_s is not None else None,
            "timeMaxSec": float(t_max_s) if t_max_s is not None else None,
            "timeSelectedSec": float(t_sel_s) if t_sel_s is not None else None,
            "timeMinMin": float(t_min_s / 60.0) if t_min_s is not None else None,
            "timeMaxMin": float(t_max_s / 60.0) if t_max_s is not None else None,
            "timeSelectedMin": float(t_sel_s / 60.0) if t_sel_s is not None else None,
            "dbMaxWidthM": float(db_max_width),
            "isOverflowWidth": bool(width_ref_m > db_max_width + 1e-9),
            "geometry": geom,
            "isReviewSubdividedArea": bool(is_review_subdiv),
            "patternType": int(pattern_type),
            "distanceMode": "long_side_length",
        }
        if virtual_meta:
            vel_payload.update(virtual_meta)
        if isinstance(piece.data, dict):
            piece.data["expVel"] = vel_payload

        row = {
            "parentOrder": int(piece.parent_order),
            "pieceIndex": int(piece.piece_index),
            "geometry": geom,
            "widthRefM": float(width_ref_m),
            "widthSource": str(width_source),
            "distanceRefM": float(distance_m),
            "candidateCount": int(len(vel_candidates)),
            "velMinKmh": float(vel_min),
            "velMaxKmh": float(vel_max),
            "velSelectedKmh": float(v_sel),
            "velApprox": bool(vel_approx),
            "velSource": str(vel_source),
            "timeMinSec": float(t_min_s) if t_min_s is not None else None,
            "timeMaxSec": float(t_max_s) if t_max_s is not None else None,
            "timeSelectedSec": float(t_sel_s) if t_sel_s is not None else None,
            "timeMinMin": float(t_min_s / 60.0) if t_min_s is not None else None,
            "timeMaxMin": float(t_max_s / 60.0) if t_max_s is not None else None,
            "timeSelectedMin": float(t_sel_s / 60.0) if t_sel_s is not None else None,
            "isOverflowWidth": bool(width_ref_m > db_max_width + 1e-9),
            "isReviewSubdividedArea": bool(is_review_subdiv),
            "patternType": int(pattern_type),
            "distanceMode": "long_side_length",
        }
        report_rows.append(row)
        piece_payload_rows.append((piece, vel_payload, row))

    # Aggregate AREA time by pre-review piece group and expose one group-time label.
    area_groups: Dict[Tuple[int, int], List[Tuple[SplitPiece, Dict[str, Any], Dict[str, Any]]]] = {}
    for piece, payload, row in piece_payload_rows:
        gk = _area_time_group_key(piece)
        if gk is None:
            continue
        area_groups.setdefault(gk, []).append((piece, payload, row))

    for gk, entries in area_groups.items():
        if not entries:
            continue
        parent, base_idx = gk
        group_id = f"A{parent}-{base_idx}"
        leader_piece_idx = min(int(p.piece_index) for p, _, _ in entries)
        vals_min: List[float] = []
        vals_max: List[float] = []
        valid = True
        for _, payload, _ in entries:
            tmin_s = payload.get("timeMinSec")
            tmax_s = payload.get("timeMaxSec")
            if tmin_s is None or tmax_s is None:
                valid = False
                continue
            vals_min.append(float(tmin_s))
            vals_max.append(float(tmax_s))
        gmin_s = float(sum(vals_min)) if valid and vals_min else None
        gmax_s = float(sum(vals_max)) if valid and vals_max else None
        gcount = int(len(entries))

        for piece, payload, row in entries:
            is_leader = int(piece.piece_index) == leader_piece_idx
            payload["areaTimeGroupKey"] = group_id
            payload["areaTimeGroupCount"] = gcount
            payload["areaTimeGroupLeader"] = bool(is_leader)
            payload["groupTimeMinSec"] = gmin_s
            payload["groupTimeMaxSec"] = gmax_s
            payload["groupTimeMinMin"] = (gmin_s / 60.0) if gmin_s is not None else None
            payload["groupTimeMaxMin"] = (gmax_s / 60.0) if gmax_s is not None else None

            row["areaTimeGroupKey"] = group_id
            row["areaTimeGroupCount"] = gcount
            row["areaTimeGroupLeader"] = bool(is_leader)
            row["groupTimeMinSec"] = gmin_s
            row["groupTimeMaxSec"] = gmax_s
            row["groupTimeMinMin"] = (gmin_s / 60.0) if gmin_s is not None else None
            row["groupTimeMaxMin"] = (gmax_s / 60.0) if gmax_s is not None else None

    return {
        "dbPath": str(db_path),
        "dbRowCount": int(len(db_rows)),
        "dbMaxWidthM": float(db_max_width),
        "pieceCount": len(report_rows),
        "rows": report_rows,
    }
