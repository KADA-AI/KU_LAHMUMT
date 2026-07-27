from __future__ import annotations



import json
import math
import random
import re
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple



if __name__ == "__main__" and __package__ is None:
    # Allow running as a script without installing the package.
    import sys
    sys.path.append(str(Path(__file__).resolve().parent.parent))
    __package__ = "fpl_random"



from .areas import AUTO_MISSION_AREA, START_REFERENCE_POINTS, LatLon
from . import dem_corridor
from . import config, paths
from .utils import now_ms_2000, offset_lat_lon

# 생성 파라미터는 config.py에서 관리한다.



PROJECT_ROOT = Path(__file__).resolve().parent.parent  # 모듈 루트(FPL_Random).



# 거리 계산을 안정화하기 위한 지역 좌표 기준점.
REF_LAT = (AUTO_MISSION_AREA.southwest.latitude + AUTO_MISSION_AREA.northeast.latitude) / 2.0
REF_LON = (AUTO_MISSION_AREA.southwest.longitude + AUTO_MISSION_AREA.northeast.longitude) / 2.0

def _db_dir() -> Path:
    return paths.db_root() / "InputMissionPlan"

def _bounds_with_margin(margin_m: float) -> Tuple[float, float, float, float]:
    sw_lat, sw_lon = offset_lat_lon(AUTO_MISSION_AREA.southwest.latitude, AUTO_MISSION_AREA.southwest.longitude, margin_m, margin_m)
    ne_lat, ne_lon = offset_lat_lon(AUTO_MISSION_AREA.northeast.latitude, AUTO_MISSION_AREA.northeast.longitude, -margin_m, -margin_m)
    return sw_lat, sw_lon, ne_lat, ne_lon

def _inside_bounds(pt: LatLon, bounds: Tuple[float, float, float, float]) -> bool:
    sw_lat, sw_lon, ne_lat, ne_lon = bounds
    return sw_lat <= pt.latitude <= ne_lat and sw_lon <= pt.longitude <= ne_lon


def _to_xy(pt: LatLon) -> Tuple[float, float]:
    east = (pt.longitude - REF_LON) * 111_320.0 * math.cos(math.radians(REF_LAT))
    north = (pt.latitude - REF_LAT) * 111_320.0
    return east, north


def _distance_m(p1: LatLon, p2: LatLon) -> float:
    e1, n1 = _to_xy(p1)
    e2, n2 = _to_xy(p2)
    return math.hypot(e2 - e1, n2 - n1)

def _bearing_deg(p1: LatLon, p2: LatLon) -> float:
    e1, n1 = _to_xy(p1)
    e2, n2 = _to_xy(p2)
    ang = math.degrees(math.atan2(e2 - e1, n2 - n1))
    return (ang + 360.0) % 360.0


def _move(point: LatLon, distance_m: float, bearing_deg: float) -> LatLon:
    rad = math.radians(bearing_deg)
    east = distance_m * math.sin(rad)
    north = distance_m * math.cos(rad)
    lat, lon = offset_lat_lon(point.latitude, point.longitude, east, north)
    return LatLon(lat, lon)


def _from_xy(east: float, north: float) -> LatLon:
    lat, lon = offset_lat_lon(REF_LAT, REF_LON, east, north)
    return LatLon(lat, lon)


def _offset_point(point: LatLon, east_m: float, north_m: float) -> LatLon:
    lat, lon = offset_lat_lon(point.latitude, point.longitude, east_m, north_m)
    return LatLon(lat, lon)


def _rect_inside_bounds(center: LatLon, width_m: float, height_m: float, bounds: Tuple[float, float, float, float]) -> bool:
    half_w = width_m / 2.0
    half_h = height_m / 2.0
    sw_lat, sw_lon = offset_lat_lon(center.latitude, center.longitude, -half_w, -half_h)
    ne_lat, ne_lon = offset_lat_lon(center.latitude, center.longitude, half_w, half_h)
    swb_lat, swb_lon, neb_lat, neb_lon = bounds
    return swb_lat <= sw_lat and swb_lon <= sw_lon and ne_lat <= neb_lat and ne_lon <= neb_lon


def _rect_corners(center: LatLon, width_m: float, height_m: float) -> List[LatLon]:
    half_w = width_m / 2.0
    half_h = height_m / 2.0
    offsets = (
        (-half_w, -half_h),
        (-half_w, half_h),
        (half_w, half_h),
        (half_w, -half_h),
    )
    corners: List[LatLon] = []
    for east, north in offsets:
        lat, lon = offset_lat_lon(center.latitude, center.longitude, east, north)
        corners.append(LatLon(lat, lon))
    return corners

def _segments_from_points(points: Sequence[LatLon]) -> List[Tuple[LatLon, LatLon]]:
    return [(points[i], points[i + 1]) for i in range(len(points) - 1)]


def _rect_edges(corners: Sequence[LatLon]) -> List[Tuple[LatLon, LatLon]]:
    return list(zip(corners, corners[1:] + corners[:1]))


def _heading_delta(a: float, b: float) -> float:
    """Smallest absolute difference between two bearings in degrees."""
    return abs(((b - a + 180.0) % 360.0) - 180.0)


def _quadrant_from_heading(heading: float) -> str:
    """Map heading to one of N/E/S/W quadrants."""
    h = heading % 360.0
    if 45.0 <= h < 135.0:
        return "E"
    if 135.0 <= h < 225.0:
        return "S"
    if 225.0 <= h < 315.0:
        return "W"
    return "N"

def _orient(a: LatLon, b: LatLon, c: LatLon) -> float:
    ae, an = _to_xy(a)
    be, bn = _to_xy(b)
    ce, cn = _to_xy(c)
    return (be - ae) * (cn - an) - (bn - an) * (ce - ae)


def _segments_intersect(a1: LatLon, a2: LatLon, b1: LatLon, b2: LatLon) -> bool:
    o1 = _orient(a1, a2, b1)
    o2 = _orient(a1, a2, b2)
    o3 = _orient(b1, b2, a1)
    o4 = _orient(b1, b2, a2)
    if (o1 == 0 and _point_on_segment(b1, a1, a2)) or (o2 == 0 and _point_on_segment(b2, a1, a2)):
        return True
    if (o3 == 0 and _point_on_segment(a1, b1, b2)) or (o4 == 0 and _point_on_segment(a2, b1, b2)):
        return True
    return (o1 > 0) != (o2 > 0) and (o3 > 0) != (o4 > 0)


def _point_on_segment(p: LatLon, a: LatLon, b: LatLon) -> bool:

    e, n = _to_xy(p)

    ae, an = _to_xy(a)

    be, bn = _to_xy(b)

    return min(ae, be) - 1e-6 <= e <= max(ae, be) + 1e-6 and min(an, bn) - 1e-6 <= n <= max(an, bn) + 1e-6 and abs(_orient(a, b, p)) < 1e-6





def _point_segment_distance_m(p: LatLon, a: LatLon, b: LatLon) -> float:

    pe, pn = _to_xy(p)

    ae, an = _to_xy(a)

    be, bn = _to_xy(b)

    ab_e = be - ae

    ab_n = bn - an

    ab2 = ab_e * ab_e + ab_n * ab_n

    if ab2 == 0.0:

        return math.hypot(pe - ae, pn - an)

    t = ((pe - ae) * ab_e + (pn - an) * ab_n) / ab2

    t = max(0.0, min(1.0, t))

    proj_e = ae + ab_e * t

    proj_n = an + ab_n * t

    return math.hypot(pe - proj_e, pn - proj_n)





def _segment_distance_m(a1: LatLon, a2: LatLon, b1: LatLon, b2: LatLon) -> float:

    if _segments_intersect(a1, a2, b1, b2):

        return 0.0

    return min(

        _point_segment_distance_m(a1, b1, b2),

        _point_segment_distance_m(a2, b1, b2),

        _point_segment_distance_m(b1, a1, a2),

        _point_segment_distance_m(b2, a1, a2),

    )





def _segment_intersects_list(seg: Tuple[LatLon, LatLon], others: Sequence[Tuple[LatLon, LatLon]], skip_shared_endpoint: bool = False) -> bool:

    for o1, o2 in others:

        if skip_shared_endpoint and _shares_endpoint(seg, (o1, o2)):

            continue

        if _segments_intersect(seg[0], seg[1], o1, o2):

            return True

    return False





def _segment_min_distance(seg: Tuple[LatLon, LatLon], others: Sequence[Tuple[LatLon, LatLon]], skip_shared_endpoint: bool = False) -> float:

    mind = float("inf")

    for o1, o2 in others:

        if skip_shared_endpoint and _shares_endpoint(seg, (o1, o2)):

            continue

        mind = min(mind, _segment_distance_m(seg[0], seg[1], o1, o2))

        if mind == 0.0:

            return 0.0

    return mind





def _shares_endpoint(seg1: Tuple[LatLon, LatLon], seg2: Tuple[LatLon, LatLon], tol_m: float = 0.1) -> bool:

    return (

        _distance_m(seg1[0], seg2[0]) < tol_m

        or _distance_m(seg1[0], seg2[1]) < tol_m

        or _distance_m(seg1[1], seg2[0]) < tol_m

        or _distance_m(seg1[1], seg2[1]) < tol_m

    )





def _min_distance_polyline_edges(points: Sequence[LatLon], edges: Sequence[Tuple[LatLon, LatLon]]) -> float:

    mind = float("inf")

    for s in _segments_from_points(points):

        mind = min(mind, _segment_min_distance(s, edges))

        if mind == 0.0:

            return 0.0

    return mind





def _min_distance_point_edges(pt: LatLon, edges: Sequence[Tuple[LatLon, LatLon]]) -> float:

    return min(_point_segment_distance_m(pt, e1, e2) for e1, e2 in edges)





def _min_distance_point_polyline(pt: LatLon, points: Sequence[LatLon]) -> float:

    return min(_point_segment_distance_m(pt, s1, s2) for s1, s2 in _segments_from_points(points))





def _polyline_intersects_segments(points: Sequence[LatLon], segments: Sequence[Tuple[LatLon, LatLon]]) -> bool:

    for s in _segments_from_points(points):

        if _segment_intersects_list(s, segments):

            return True

    return False





def _pick_main_sensor(rng: random.Random) -> int:

    weights = [(sensor, weight) for sensor, weight in config.MAIN_SENSOR_WEIGHTS.items() if weight > 0]

    if not weights:

        return 1

    total = sum(weight for _, weight in weights)

    pick = rng.uniform(0, total)

    upto = 0.0

    for sensor, weight in weights:

        upto += weight

        if pick <= upto:

            return sensor

    return weights[-1][0]

def _weighted_choice(rng: random.Random, weights: Dict[int, float], default: int) -> int:

    items = [(int(k), float(v)) for k, v in weights.items() if float(v) > 0]

    if not items:

        return default

    total = sum(weight for _, weight in items)

    pick = rng.uniform(0, total)

    upto = 0.0

    for key, weight in items:

        upto += weight

        if pick <= upto:

            return key

    return items[-1][0]


def _int_value(value: float | int) -> int:

    try:

        return int(round(float(value)))

    except Exception:

        return 0


def _line_width(rng: random.Random, aircraft_count: int) -> int:

    """Width scales by aircraft count; each aircraft contributes 200~500m in 50m steps."""

    steps = int((config.PER_AIRCRAFT_WIDTH_MAX_M - config.PER_AIRCRAFT_WIDTH_MIN_M) / config.WIDTH_STEP_M)

    per_aircraft = config.PER_AIRCRAFT_WIDTH_MIN_M + config.WIDTH_STEP_M * rng.randint(0, steps)

    total = per_aircraft * max(1, aircraft_count)

    return _int_value(total)

def _rand_step(rng: random.Random, min_m: float, max_m: float, step_m: float) -> float:

    if step_m <= 0:

        return rng.uniform(min_m, max_m)

    if max_m < min_m:

        min_m, max_m = max_m, min_m

    steps = int((max_m - min_m) / step_m)

    return min_m + step_m * rng.randint(0, max(0, steps))





def _generate_line_path(

    start: LatLon,

    bounds: Tuple[float, float, float, float],

    rng: random.Random,

    avoid_segments: Sequence[Tuple[LatLon, LatLon]],

    min_gap_m: float,

    heading: Optional[float] = None,

    aircraft_count: int = 1,

) -> Tuple[Optional[List[LatLon]], Optional[float]]:

    point_count = rng.randint(config.LINE_POINT_COUNT_RANGE[0], config.LINE_POINT_COUNT_RANGE[1])

    width = _line_width(rng, aircraft_count)

    pts: List[LatLon] = [start]

    heading_cur = rng.uniform(0.0, 360.0) if heading is None else heading

    segments: List[Tuple[LatLon, LatLon]] = []

    for _ in range(point_count - 1):

        success = False

        for _ in range(config.SEGMENT_ATTEMPTS):

            delta = rng.uniform(-config.HEADING_DELTA_MAX_DEG, config.HEADING_DELTA_MAX_DEG)

            heading_new = (heading_cur + delta) % 360.0

            distance = rng.uniform(config.LINE_SEGMENT_MIN_M, config.LINE_SEGMENT_MAX_M)

            candidate = _move(pts[-1], distance, heading_new)

            if not _inside_bounds(candidate, bounds):

                continue

            new_seg = (pts[-1], candidate)

            if _segment_intersects_list(new_seg, segments, skip_shared_endpoint=True):

                continue

            if avoid_segments and _segment_intersects_list(new_seg, avoid_segments):

                continue

            if avoid_segments and _segment_min_distance(new_seg, avoid_segments) < min_gap_m:

                continue

            segments.append(new_seg)

            pts.append(candidate)

            heading_cur = heading_new

            success = True

            break

        if not success:

            return None, None

    return pts, width


def _generate_line_chain(
    start: LatLon,
    bounds: Tuple[float, float, float, float],
    rng: random.Random,
    segment_count: int,
    heading: Optional[float] = None,
    *,
    min_gap_m: float = 0.0,
    takeovers: Optional[Sequence[LatLon]] = None,
    min_takeover_m: float = 0.0,
    min_seg_m: Optional[float] = None,
    max_seg_m: Optional[float] = None,
    heading_delta_max: Optional[float] = None,
    end_point: Optional[LatLon] = None,
) -> Tuple[Optional[List[LatLon]], Optional[List[Tuple[LatLon, LatLon]]], Optional[List[float]]]:
    pts: List[LatLon] = [start]
    segments: List[Tuple[LatLon, LatLon]] = []
    headings: List[float] = []
    heading_cur = rng.uniform(0.0, 360.0) if heading is None else heading

    for _ in range(segment_count):
        success = False
        for _ in range(config.SEGMENT_ATTEMPTS):
            delta_max = config.HEADING_DELTA_MAX_DEG if heading_delta_max is None else heading_delta_max
            if end_point is not None:
                desired = _bearing_deg(pts[-1], end_point)
                delta_to_desired = ((desired - heading_cur + 180.0) % 360.0) - 180.0
                delta = max(-delta_max, min(delta_to_desired, delta_max))
                delta += rng.uniform(-delta_max * 0.25, delta_max * 0.25)
                delta = max(-delta_max, min(delta, delta_max))
            else:
                delta = rng.uniform(-delta_max, delta_max)
            heading_new = (heading_cur + delta) % 360.0
            seg_min = config.LINE_SEGMENT_MIN_M if min_seg_m is None else min_seg_m
            seg_max = config.LINE_SEGMENT_MAX_M if max_seg_m is None else max_seg_m
            if seg_max < seg_min:
                seg_max = seg_min
            distance = rng.uniform(seg_min, seg_max)
            candidate = _move(pts[-1], distance, heading_new)
            if not _inside_bounds(candidate, bounds):
                continue
            new_seg = (pts[-1], candidate)
            if _segment_intersects_list(new_seg, segments, skip_shared_endpoint=True):
                continue
            if min_gap_m > 0 and _segment_min_distance(new_seg, segments, skip_shared_endpoint=True) < min_gap_m:
                continue
            if takeovers and min_takeover_m > 0:
                too_close = False
                for pt in takeovers:
                    if _point_segment_distance_m(pt, new_seg[0], new_seg[1]) < min_takeover_m:
                        too_close = True
                        break
                if too_close:
                    continue
            segments.append(new_seg)
            headings.append(heading_new)
            pts.append(candidate)
            heading_cur = heading_new
            success = True
            break
        if not success:
            return None, None, None

    return pts, segments, headings


def _path_length_m(points: Sequence[LatLon]) -> float:
    total = 0.0
    for i in range(1, len(points)):
        total += _distance_m(points[i - 1], points[i])
    return total


def _resample_path_points(points: Sequence[LatLon], count: int) -> List[LatLon]:
    if not points:
        return []
    if count <= 1:
        return [points[0]]
    cum = _cumulative_distances(points)
    total = cum[-1] if cum else 0.0
    if total <= 0.0:
        return [points[0]] * count
    step = total / max(1, count - 1)
    sampled: List[LatLon] = []
    for i in range(count):
        sampled.append(_point_at_distance(points, cum, step * i))
    return sampled


def _simplify_path_by_turns(
    points: Sequence[LatLon],
    min_seg_m: float,
    turn_deg: float,
) -> List[LatLon]:
    if len(points) <= 2:
        return list(points)
    kept: List[LatLon] = [points[0]]
    dist_since = 0.0
    for i in range(1, len(points) - 1):
        dist_since += _distance_m(points[i - 1], points[i])
        if dist_since < min_seg_m:
            continue
        b1 = _bearing_deg(points[i - 1], points[i])
        b2 = _bearing_deg(points[i], points[i + 1])
        if _heading_delta(b1, b2) >= turn_deg:
            kept.append(points[i])
            dist_since = 0.0
    kept.append(points[-1])
    deduped: List[LatLon] = []
    for pt in kept:
        if not deduped or _distance_m(deduped[-1], pt) > 0.5:
            deduped.append(pt)
    return deduped


def _shape_corridor_path_coords(points: Sequence[LatLon]) -> List[LatLon]:
    if len(points) <= 2:
        return list(points)
    min_count, max_count = config.LINE_POINT_COUNT_RANGE
    corridor_min = min(max_count, max(min_count, 3)) if max_count >= 3 else max_count
    turn_deg = float(getattr(config, "DEM_PATH_TURN_DEG", 20.0))
    min_seg_m = float(getattr(config, "DEM_PATH_MIN_SEG_M", 1000.0))
    simplified = _simplify_path_by_turns(points, min_seg_m, turn_deg)
    if len(simplified) < corridor_min:
        return _resample_path_points(points, corridor_min)
    if len(simplified) > max_count:
        return _resample_path_points(simplified, max_count)
    return simplified




def _path_span_m(points: Sequence[LatLon]) -> Tuple[float, float]:
    if not points:
        return 0.0, 0.0
    lats = [p.latitude for p in points]
    lons = [p.longitude for p in points]
    min_lat = min(lats)
    max_lat = max(lats)
    min_lon = min(lons)
    max_lon = max(lons)
    center_lat = (min_lat + max_lat) / 2.0
    meters_per_deg_lat = 111_320.0
    meters_per_deg_lon = meters_per_deg_lat * math.cos(math.radians(center_lat))
    span_x = (max_lon - min_lon) * meters_per_deg_lon
    span_y = (max_lat - min_lat) * meters_per_deg_lat
    return abs(span_x), abs(span_y)


def _cumulative_distances(points: Sequence[LatLon]) -> List[float]:
    if not points:
        return []
    out = [0.0]
    for i in range(1, len(points)):
        out.append(out[-1] + _distance_m(points[i - 1], points[i]))
    return out


def _point_at_distance(points: Sequence[LatLon], cum: Sequence[float], dist: float) -> LatLon:
    if not points:
        raise ValueError("no points")
    if dist <= 0.0:
        return points[0]
    if dist >= cum[-1]:
        return points[-1]
    for i in range(1, len(points)):
        if cum[i] >= dist:
            prev = points[i - 1]
            cur = points[i]
            span = cum[i] - cum[i - 1]
            if span <= 1e-9:
                return cur
            t = (dist - cum[i - 1]) / span
            lat = prev.latitude + (cur.latitude - prev.latitude) * t
            lon = prev.longitude + (cur.longitude - prev.longitude) * t
            return LatLon(lat, lon)
    return points[-1]


def _subpath(points: Sequence[LatLon], cum: Sequence[float], start_d: float, end_d: float) -> List[LatLon]:
    if not points:
        return []
    if end_d < start_d:
        start_d, end_d = end_d, start_d
    start_d = max(0.0, start_d)
    end_d = min(cum[-1], end_d)
    if end_d <= start_d:
        return []
    start_pt = _point_at_distance(points, cum, start_d)
    end_pt = _point_at_distance(points, cum, end_d)
    start_idx = 0
    end_idx = 0
    for i in range(1, len(points)):
        if cum[i] >= start_d:
            start_idx = i
            break
    for i in range(1, len(points)):
        if cum[i] >= end_d:
            end_idx = i
            break
    coords: List[LatLon] = [start_pt]
    for i in range(start_idx, end_idx):
        coords.append(points[i])
    coords.append(end_pt)
    deduped: List[LatLon] = []
    for pt in coords:
        if not deduped or _distance_m(deduped[-1], pt) > 0.5:
            deduped.append(pt)
    return deduped


def _heading_at_distance(points: Sequence[LatLon], cum: Sequence[float], dist: float) -> float:
    if len(points) < 2:
        return 0.0
    if dist <= 0.0:
        return _bearing_deg(points[0], points[1])
    if dist >= cum[-1]:
        return _bearing_deg(points[-2], points[-1])
    for i in range(1, len(points)):
        if cum[i] >= dist:
            return _bearing_deg(points[i - 1], points[i])
    return _bearing_deg(points[-2], points[-1])


def _select_intervals(
    total_len: float,
    lengths: Sequence[float],
    min_gap: float,
    rng: random.Random,
    attempts: int,
    forbidden: Optional[Sequence[Tuple[float, float]]] = None,
) -> Optional[List[Tuple[float, float]]]:
    if total_len <= 0:
        return None
    ordered = sorted(((i, l) for i, l in enumerate(lengths)), key=lambda x: x[1], reverse=True)
    placed: List[Tuple[float, float, int]] = []
    for idx, length in ordered:
        length = max(1.0, min(length, total_len))
        success = False
        for _ in range(attempts):
            start = rng.uniform(0.0, max(0.0, total_len - length))
            end = start + length
            if forbidden:
                blocked = False
                for s, e in forbidden:
                    if not (end + min_gap <= s or start >= e + min_gap):
                        blocked = True
                        break
                if blocked:
                    continue
            ok = True
            for s, e, _ in placed:
                if end + min_gap <= s or start >= e + min_gap:
                    continue
                ok = False
                break
            if ok:
                placed.append((start, end, idx))
                success = True
                break
        if not success:
            return None
    placed.sort(key=lambda x: x[2])
    return [(s, e) for s, e, _ in placed]


def _pick_anchor_distances(
    total_len: float,
    count: int,
    forbidden: Sequence[Tuple[float, float]],
    min_gap: float,
    rng: random.Random,
    attempts: int,
    predicate: Optional[Callable[[float], bool]] = None,
) -> Optional[List[float]]:
    if count <= 0:
        return []
    picks: List[float] = []
    for _ in range(count):
        success = False
        for _ in range(attempts):
            cand = rng.uniform(0.0, total_len)
            if any(s <= cand <= e for s, e in forbidden):
                continue
            if any(abs(cand - p) < min_gap for p in picks):
                continue
            if predicate and not predicate(cand):
                continue
            picks.append(cand)
            success = True
            break
        if not success:
            return None
    return picks


def _nearest_allowed_distance(
    target: float,
    total_len: float,
    forbidden: Sequence[Tuple[float, float]],
    step: float,
    tries: int,
    predicate: Optional[Callable[[float], bool]] = None,
) -> Optional[float]:
    for i in range(tries):
        for sign in (1, -1):
            cand = target + sign * step * i
            if cand < 0.0 or cand > total_len:
                continue
            if any(s <= cand <= e for s, e in forbidden):
                continue
            if predicate and not predicate(cand):
                continue
            return cand
    return None


def _generate_guideline(
    start: LatLon,
    bounds: Tuple[float, float, float, float],
    rng: random.Random,
    segment_count: int,
    *,
    min_takeover_m: float,
    takeovers: Sequence[LatLon],
    min_length_m: float,
    max_length_m: float,
    min_span_major_ratio: float,
    min_span_minor_ratio: float,
    min_seg_m: float,
    max_seg_m: float,
    min_gap_m: float = 0.0,
    end_point: Optional[LatLon] = None,
    end_tolerance_m: float = 0.0,
) -> Tuple[Optional[List[LatLon]], Optional[List[Tuple[LatLon, LatLon]]], Optional[List[float]]]:
    network = None
    if config.DEM_CORRIDOR_ENABLE:
        try:
            network = dem_corridor.build_corridor_network(
                dem_path=Path(config.DEM_CORRIDOR_FILE),
                flow_threshold=config.DEM_CORRIDOR_FLOW_ACC_THRESHOLD,
            )
        except Exception:
            network = None

    for _ in range(config.MAX_GEN_ATTEMPTS):
        pts, segments, headings = _generate_line_chain(
            start,
            bounds,
            rng,
            segment_count=segment_count,
            heading=None,
            min_gap_m=min_gap_m,
            takeovers=takeovers,
            min_takeover_m=min_takeover_m,
            min_seg_m=min_seg_m,
            max_seg_m=max_seg_m,
            heading_delta_max=config.HEADING_DELTA_MAX_DEG * rng.uniform(1.0, 1.3),
            end_point=end_point,
        )
        if not pts or not segments or not headings:
            continue
        if end_point is not None and end_tolerance_m > 0:
            if _distance_m(pts[-1], end_point) > end_tolerance_m:
                continue
        length = _path_length_m(pts)
        if min_length_m > 0 and length < min_length_m:
            continue
        if max_length_m > 0 and length > max_length_m:
            continue
        span_x, span_y = _path_span_m(pts)
        if span_x <= 0 or span_y <= 0:
            continue
        sw_lat, sw_lon, ne_lat, ne_lon = bounds
        center_lat = (sw_lat + ne_lat) / 2.0
        meters_per_deg_lat = 111_320.0
        meters_per_deg_lon = meters_per_deg_lat * math.cos(math.radians(center_lat))
        bounds_w = abs(ne_lon - sw_lon) * meters_per_deg_lon
        bounds_h = abs(ne_lat - sw_lat) * meters_per_deg_lat
        max_dim = max(bounds_w, bounds_h)
        min_dim = min(bounds_w, bounds_h)
        if max_dim > 0 and max(span_x, span_y) < max_dim * min_span_major_ratio:
            continue
        if min_dim > 0 and min(span_x, span_y) < min_dim * min_span_minor_ratio:
            continue
        return pts, segments, headings
    return None, None, None


def _split_guideline(points: Sequence[LatLon], group_count: int) -> List[List[LatLon]]:
    if group_count <= 0:
        return []
    segment_total = max(0, len(points) - 1)
    if segment_total <= 0:
        return []
    group_count = min(group_count, segment_total)
    base = segment_total // group_count
    extra = segment_total % group_count
    groups: List[List[LatLon]] = []
    idx = 0
    for i in range(group_count):
        segs = base + (1 if i < extra else 0)
        segs = max(1, segs)
        end_idx = min(segment_total, idx + segs)
        coords = list(points[idx : end_idx + 1])
        if len(coords) >= 2:
            groups.append(coords)
        idx = end_idx
    return groups


def _simple_line_from_point(
    anchor: LatLon,
    bounds: Tuple[float, float, float, float],
    rng: random.Random,
    heading_hint: Optional[float] = None,
) -> Optional[List[LatLon]]:
    for _ in range(config.SEGMENT_ATTEMPTS):
        heading = heading_hint if heading_hint is not None else rng.uniform(0.0, 360.0)
        heading = (heading + rng.uniform(-30.0, 30.0)) % 360.0
        distance = rng.uniform(config.LINE_SEGMENT_MIN_M, config.LINE_SEGMENT_MAX_M)
        end = _move(anchor, distance, heading)
        if _inside_bounds(end, bounds):
            return [anchor, end]
    return None



def _place_area_near_point(

    ref_point: LatLon,

    bounds: Tuple[float, float, float, float],

    rng: random.Random,

    line_points: Sequence[LatLon],

    heading_ref: float,

) -> Optional[Tuple[LatLon, float, float, List[LatLon]]]:

    line_segments = _segments_from_points(line_points)

    preferred_dir = _quadrant_from_heading(heading_ref)

    dir_cycle = {

        "N": ("N", "E", "W", "S"),

        "S": ("S", "E", "W", "N"),

        "E": ("E", "N", "S", "W"),

        "W": ("W", "N", "S", "E"),

    }[preferred_dir]

    for attempt in range(config.RECT_ATTEMPTS):

        width = _rand_step(rng, config.AREA_SIDE_MIN_M, config.AREA_SIDE_MAX_M, config.AREA_SIDE_STEP_M)

        height = _rand_step(rng, config.AREA_SIDE_MIN_M, config.AREA_SIDE_MAX_M, config.AREA_SIDE_STEP_M)

        gap = rng.uniform(config.EDGE_GAP_MIN_M, config.EDGE_GAP_MAX_M)

        # Try forward-facing directions first; fallback to others later.

        if attempt < config.RECT_ATTEMPTS // 2:

            direction = dir_cycle[attempt % len(dir_cycle)]

        else:

            direction = rng.choice(dir_cycle)

        if direction in ("E", "W"):

            east_offset = (width / 2.0 + gap) * (1 if direction == "E" else -1)

            north_offset = rng.uniform(-height / 2.0, height / 2.0)

        else:

            north_offset = (height / 2.0 + gap) * (1 if direction == "N" else -1)

            east_offset = rng.uniform(-width / 2.0, width / 2.0)

        lat, lon = offset_lat_lon(ref_point.latitude, ref_point.longitude, east_offset, north_offset)

        center = LatLon(lat, lon)

        if not _rect_inside_bounds(center, width, height, bounds):

            continue

        bearing_to_center = _bearing_deg(ref_point, center)

        if _heading_delta(heading_ref, bearing_to_center) > config.FORWARD_ALIGN_DEG:

            continue

        corners = _rect_corners(center, width, height)

        edges = _rect_edges(corners)

        if _polyline_intersects_segments(line_points, edges):

            continue

        min_dist_line = _min_distance_polyline_edges(line_points, edges)

        if min_dist_line < config.EDGE_GAP_MIN_M:

            continue

        dist_end = _min_distance_point_edges(ref_point, edges)

        if not (config.EDGE_GAP_MIN_M <= dist_end <= config.EDGE_GAP_MAX_M):

            continue

        return center, width, height, corners

    return None


def _area_near_point_fallback(
    anchor: LatLon,
    bounds: Tuple[float, float, float, float],
    rng: random.Random,
) -> Optional[List[LatLon]]:
    for _ in range(config.RECT_ATTEMPTS):
        width = _rand_step(rng, config.AREA_SIDE_MIN_M, config.AREA_SIDE_MAX_M, config.AREA_SIDE_STEP_M)
        height = _rand_step(rng, config.AREA_SIDE_MIN_M, config.AREA_SIDE_MAX_M, config.AREA_SIDE_STEP_M)
        if _rect_inside_bounds(anchor, width, height, bounds):
            return _rect_corners(anchor, width, height)
    return None


def _area_centered_on_point(
    anchor: LatLon,
    bounds: Tuple[float, float, float, float],
    rng: random.Random,
    takeovers: Optional[Sequence[LatLon]] = None,
    min_gap_m: float = 0.0,
) -> Optional[List[LatLon]]:
    sw_lat, sw_lon, ne_lat, ne_lon = bounds
    west = LatLon(anchor.latitude, sw_lon)
    east = LatLon(anchor.latitude, ne_lon)
    south = LatLon(sw_lat, anchor.longitude)
    north = LatLon(ne_lat, anchor.longitude)
    max_half_w = min(_distance_m(anchor, west), _distance_m(anchor, east))
    max_half_h = min(_distance_m(anchor, south), _distance_m(anchor, north))
    max_width = min(config.AREA_SIDE_MAX_M, max_half_w * 2.0)
    max_height = min(config.AREA_SIDE_MAX_M, max_half_h * 2.0)
    if takeovers:
        nearest = min(_distance_m(anchor, pt) for pt in takeovers)
        if nearest > 0:
            safe_span = max(0.0, (nearest - min_gap_m) * 2.0)
            max_width = min(max_width, safe_span)
            max_height = min(max_height, safe_span)
    if max_width <= 0.0 or max_height <= 0.0:
        return None
    if max_width < config.AREA_SIDE_MIN_M or max_height < config.AREA_SIDE_MIN_M:
        return None
    min_width = min(config.AREA_SIDE_MIN_M, max_width)
    min_height = min(config.AREA_SIDE_MIN_M, max_height)
    for _ in range(config.RECT_ATTEMPTS):
        width = _rand_step(rng, min_width, max_width, config.AREA_SIDE_STEP_M)
        height = _rand_step(rng, min_height, max_height, config.AREA_SIDE_STEP_M)
        if not _rect_inside_bounds(anchor, width, height, bounds):
            continue
        use_pent = rng.random() < config.AREA_PENTAGON_RATIO
        if use_pent:
            radius = min(width, height) / 2.0
            points = _pentagon_points(anchor, radius, rng)
            if all(_inside_bounds(p, bounds) for p in points):
                return points
            continue
        return _rect_corners(anchor, width, height)
    return None


def _anchor_clear_for_area(
    anchor: LatLon,
    bounds: Tuple[float, float, float, float],
    min_side_m: float,
) -> bool:
    if min_side_m <= 0:
        return True
    sw_lat, sw_lon, ne_lat, ne_lon = bounds
    west = LatLon(anchor.latitude, sw_lon)
    east = LatLon(anchor.latitude, ne_lon)
    south = LatLon(sw_lat, anchor.longitude)
    north = LatLon(ne_lat, anchor.longitude)
    max_half_w = min(_distance_m(anchor, west), _distance_m(anchor, east))
    max_half_h = min(_distance_m(anchor, south), _distance_m(anchor, north))
    return max_half_w * 2.0 >= min_side_m and max_half_h * 2.0 >= min_side_m


def _area_from_point(
    anchor: LatLon,
    bounds: Tuple[float, float, float, float],
    rng: random.Random,
    line_points: Sequence[LatLon],
    heading_ref: float,
) -> Optional[List[LatLon]]:
    area_params = _place_area_near_point(anchor, bounds, rng, line_points, heading_ref)
    if area_params:
        _, _, _, area_corners = area_params
        return area_corners
    return _area_near_point_fallback(anchor, bounds, rng)




def _point_off_rect(

    center: LatLon,

    width_m: float,

    height_m: float,

    corners: Sequence[LatLon],

    bounds: Tuple[float, float, float, float],

    rng: random.Random,

    avoid_segments: Sequence[Tuple[LatLon, LatLon]],

    heading_ref: Optional[float] = None,

    preferred_direction: Optional[str] = None,

    force_direction: bool = False,

) -> Optional[LatLon]:

    edges = _rect_edges(corners)

    for _ in range(config.RECT_ATTEMPTS):

        gap = rng.uniform(config.EDGE_GAP_MIN_M, config.EDGE_GAP_MAX_M)

        if preferred_direction and (force_direction or _ < config.RECT_ATTEMPTS // 2):

            direction = preferred_direction

        else:

            direction = rng.choice(("E", "W", "N", "S"))

        if direction in ("E", "W"):

            east_offset = (width_m / 2.0 + gap) * (1 if direction == "E" else -1)

            north_offset = rng.uniform(-height_m / 2.0, height_m / 2.0)

        else:

            north_offset = (height_m / 2.0 + gap) * (1 if direction == "N" else -1)

            east_offset = rng.uniform(-width_m / 2.0, width_m / 2.0)

        lat, lon = offset_lat_lon(center.latitude, center.longitude, east_offset, north_offset)

        pt = LatLon(lat, lon)

        if not _inside_bounds(pt, bounds):

            continue

        if heading_ref is not None and _heading_delta(heading_ref, _bearing_deg(center, pt)) > config.CONTINUE_HEADING_MAX_DEG:

            continue

        dist_rect = _min_distance_point_edges(pt, edges)

        if not (config.EDGE_GAP_MIN_M <= dist_rect <= config.EDGE_GAP_MAX_M):

            continue

        if avoid_segments and _min_distance_point_segments(pt, avoid_segments) < config.EDGE_GAP_MIN_M:

            continue

        return pt

    return None





def _min_distance_point_segments(pt: LatLon, segments: Sequence[Tuple[LatLon, LatLon]]) -> float:

    return min(_point_segment_distance_m(pt, s1, s2) for s1, s2 in segments)


def _poly_edges(points: Sequence[LatLon]) -> List[Tuple[LatLon, LatLon]]:
    if len(points) < 2:
        return []
    return [(points[i], points[(i + 1) % len(points)]) for i in range(len(points))]


def _point_in_poly(pt: LatLon, poly: Sequence[LatLon]) -> bool:
    """Ray casting; works for simple polygons (including rectangles)."""
    x, y = pt.longitude, pt.latitude
    inside = False
    n = len(poly)
    if n < 3:
        return False
    for i in range(n):
        x1, y1 = poly[i - 1].longitude, poly[i - 1].latitude
        x2, y2 = poly[i].longitude, poly[i].latitude
        if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1 + 1e-12) + x1):
            inside = not inside
    return inside


def _poly_intersects_poly(a: Sequence[LatLon], b: Sequence[LatLon]) -> bool:
    if len(a) < 3 or len(b) < 3:
        return False
    edges_a = _poly_edges(a)
    edges_b = _poly_edges(b)
    for seg in edges_a:
        if _segment_intersects_list(seg, edges_b):
            return True
    if _point_in_poly(a[0], b) or _point_in_poly(b[0], a):
        return True
    return False


def _min_distance_poly_poly(a: Sequence[LatLon], b: Sequence[LatLon]) -> float:
    if len(a) < 2 or len(b) < 2:
        return float("inf")
    edges_a = _poly_edges(a)
    edges_b = _poly_edges(b)
    mind = float("inf")
    for s1 in edges_a:
        for s2 in edges_b:
            mind = min(mind, _segment_distance_m(s1[0], s1[1], s2[0], s2[1]))
            if mind == 0.0:
                return 0.0
    return mind


def _line_clear_of_lines(
    line_points: Sequence[LatLon],
    segments: Sequence[Tuple[LatLon, LatLon]],
    min_gap_m: float,
) -> bool:
    if not segments:
        return True
    for seg in _segments_from_points(line_points):
        if _segment_intersects_list(seg, segments, skip_shared_endpoint=False):
            return False
        if _segment_min_distance(seg, segments, skip_shared_endpoint=False) < min_gap_m:
            return False
    return True


def _line_clear_of_areas(
    line_points: Sequence[LatLon],
    areas: Sequence[Sequence[LatLon]],
    min_gap_m: float,
) -> bool:
    for area in areas:
        if len(area) < 3:
            continue
        edges = _poly_edges(area)
        if _polyline_intersects_segments(line_points, edges):
            return False
        if any(_point_in_poly(p, area) for p in line_points):
            return False
        if _min_distance_polyline_edges(line_points, edges) < min_gap_m:
            return False
    return True


def _area_clear_of_lines(
    area: Sequence[LatLon],
    segments: Sequence[Tuple[LatLon, LatLon]],
    min_gap_m: float,
) -> bool:
    if len(area) < 3:
        return False
    edges = _poly_edges(area)
    for seg in segments:
        if _point_in_poly(seg[0], area) or _point_in_poly(seg[1], area):
            return False
        if _segment_intersects_list(seg, edges):
            return False
        if _segment_min_distance(seg, edges) < min_gap_m:
            return False
    return True


def _area_clear_of_areas(
    area: Sequence[LatLon],
    areas: Sequence[Sequence[LatLon]],
    min_gap_m: float,
) -> bool:
    if len(area) < 3:
        return False
    for other in areas:
        if len(other) < 3:
            continue
        if _poly_intersects_poly(area, other):
            return False
        if _min_distance_poly_poly(area, other) < min_gap_m:
            return False
    return True


def _line_clear_of_takeovers(
    line_points: Sequence[LatLon],
    takeovers: Sequence[LatLon],
    min_gap_m: float,
) -> bool:
    if not takeovers:
        return True
    for seg in _segments_from_points(line_points):
        for pt in takeovers:
            if _point_segment_distance_m(pt, seg[0], seg[1]) < min_gap_m:
                return False
    return True


def _area_clear_of_takeovers(
    area: Sequence[LatLon],
    takeovers: Sequence[LatLon],
    min_gap_m: float,
) -> bool:
    if not takeovers:
        return True
    if len(area) < 3:
        return False
    edges = _poly_edges(area)
    for pt in takeovers:
        if _point_in_poly(pt, area):
            return False
        if _min_distance_point_edges(pt, edges) < min_gap_m:
            return False
    return True


def _try_line_mission(
    anchor: LatLon,
    bounds: Tuple[float, float, float, float],
    rng: random.Random,
    heading_ref: Optional[float],
    existing_segments: Sequence[Tuple[LatLon, LatLon]],
    existing_areas: Sequence[Sequence[LatLon]],
    takeovers: Sequence[LatLon],
    min_gap_m: float,
    min_takeover_m: float,
) -> Optional[List[LatLon]]:
    for _ in range(config.SEGMENT_ATTEMPTS):
        coords = _simple_line_from_point(anchor, bounds, rng, heading_ref)
        if not coords:
            continue
        if not _line_clear_of_lines(coords, existing_segments, min_gap_m):
            continue
        if not _line_clear_of_areas(coords, existing_areas, min_gap_m):
            continue
        if not _line_clear_of_takeovers(coords, takeovers, min_takeover_m):
            continue
        return coords
    return None


def _try_area_mission(
    anchor: LatLon,
    bounds: Tuple[float, float, float, float],
    rng: random.Random,
    line_points: Sequence[LatLon],
    heading_ref: float,
    existing_segments: Sequence[Tuple[LatLon, LatLon]],
    existing_areas: Sequence[Sequence[LatLon]],
    takeovers: Sequence[LatLon],
    min_gap_m: float,
    min_takeover_m: float,
    center_on_anchor: bool = False,
) -> Optional[List[LatLon]]:
    if center_on_anchor:
        coords = _area_centered_on_point(anchor, bounds, rng, takeovers=takeovers, min_gap_m=min_takeover_m)
        if coords and _area_clear_of_lines(coords, existing_segments, min_gap_m) and _area_clear_of_areas(
            coords, existing_areas, min_gap_m
        ) and _area_clear_of_takeovers(coords, takeovers, min_takeover_m):
            return coords
        return None
    for _ in range(config.RECT_ATTEMPTS):
        coords = _area_from_point(anchor, bounds, rng, line_points, heading_ref)
        if not coords:
            continue
        if not _area_clear_of_lines(coords, existing_segments, min_gap_m):
            continue
        if not _area_clear_of_areas(coords, existing_areas, min_gap_m):
            continue
        if not _area_clear_of_takeovers(coords, takeovers, min_takeover_m):
            continue
        return coords
    return None





def _type_base(package_type: Optional[int]) -> int:
    if package_type is None:
        return 0
    try:
        pkg = int(package_type)
    except Exception:
        return 0
    if pkg <= 0:
        return 0
    return pkg * 1000


def _next_ids(package_type: Optional[int] = None) -> Tuple[int, int]:

    base = _type_base(package_type)
    current = _existing_max_seq(package_type)
    if base and current < base:
        return base, base
    seq = current + 1
    return seq, seq





def _existing_max_seq(package_type: Optional[int] = None) -> int:
    base = _type_base(package_type)
    upper = base + 999 if base else 0
    max_seq = 0

    dir_path = _db_dir()
    roots = []
    if dir_path.exists():
        roots.extend(dir_path.glob("*.json"))
    bundle_root = paths.db_root()
    roots.extend(bundle_root.glob("Random_Scenario_*/*/InputMissionPlan/*.json"))

    for p in roots:
        seq = _extract_seq(p.stem)
        if seq is None:
            continue
        if base and not (base <= seq <= upper):
            continue
        max_seq = max(max_seq, seq)

    return max_seq


def _extract_seq(stem: str) -> int | None:
    if not stem:
        return None
    if stem.isdigit():
        return int(stem)
    match = re.search(r"(\d+)$", stem)
    if not match:
        return None
    try:
        return int(match.group(1))
    except Exception:
        return None





def _bootstrap_state_if_needed() -> None:

    # Previously used a state file; now sequence is derived solely from existing files.

    return None





def _line_mission(
    mission_id: int,
    points: List[LatLon],
    width: float,
    altitude_m: float,
    mission_type: int = 1,
    region_type: int = 0,
) -> Dict:

    return {

        "inputMissionID": mission_id,

        "inputMissionType": mission_type,

        "regionType": int(region_type),

        "isDone": False,

        "missionDetail": {

            "coordinateList": None,

            "lineList": [

                {

                    "width": float(width),

                    "coordinateList": [

                        {
                            "latitude": p.latitude,
                            "longitude": p.longitude,
                            "altitude": _int_value(altitude_m),
                        }
                        for p in points

                    ],

                }

            ],

            "areaList": None,

        },

    }





def _area_mission(
    mission_id: int,
    coords: List[LatLon],
    mission_type: int = 2,
    region_type: int = 0,
) -> Dict:

    return {

        "inputMissionID": mission_id,

        "inputMissionType": mission_type,

        "regionType": int(region_type),

        "isDone": False,

        "missionDetail": {

            "coordinateList": None,

            "lineList": None,

            "areaList": [

                {

                    "isHole": False,

                    "coordinateList": [

                        {"latitude": p.latitude, "longitude": p.longitude, "altitude": 0} for p in coords

                    ],

                }

            ],

        },

    }

def _point_mission(
    mission_id: int,
    point: LatLon,
    mission_type: int,
    region_type: int = 0,
) -> Dict:

    return {

        "inputMissionID": mission_id,

        "inputMissionType": mission_type,

        "regionType": int(region_type),

        "isDone": False,

        "missionDetail": {

            "coordinateList": [

                {"latitude": point.latitude, "longitude": point.longitude, "altitude": 0}

            ],

            "lineList": None,

            "areaList": None,

        },

    }


def _composite_route_anchor(takeovers: Optional[Sequence[LatLon]]) -> LatLon:
    if takeovers:
        return takeovers[0]
    return START_REFERENCE_POINTS[0]


def _composite_area_coords(
    center: LatLon,
    rng: random.Random,
    bounds: Tuple[float, float, float, float],
    *,
    width_range_m: Tuple[float, float],
    height_range_m: Tuple[float, float],
    pentagon_ratio: float = 0.35,
) -> List[LatLon]:
    for _ in range(30):
        if rng.random() < pentagon_ratio:
            radius = rng.uniform(min(width_range_m), max(width_range_m)) * rng.uniform(0.48, 0.62)
            coords = _pentagon_points(center, radius, rng)
        else:
            width_m = rng.uniform(*width_range_m)
            height_m = rng.uniform(*height_range_m)
            coords = _rect_corners(center, width_m, height_m)
        if coords and all(_inside_bounds(pt, bounds) for pt in coords):
            return coords
    return _rect_corners(center, min(width_range_m), min(height_range_m))


def _composite_line_candidate_ok(
    points: Sequence[LatLon],
    *,
    bounds: Tuple[float, float, float, float],
    existing_segments: Sequence[Tuple[LatLon, LatLon]],
    blocked_areas: Sequence[Sequence[LatLon]],
    blocked_points: Sequence[LatLon],
    takeovers: Sequence[LatLon],
    min_line_gap_m: float,
    min_area_gap_m: float,
    min_point_gap_m: float,
    min_takeover_m: float,
) -> bool:
    if len(points) < 2:
        return False
    if any(not _inside_bounds(point, bounds) for point in points):
        return False
    if existing_segments and not _line_clear_of_lines(points, existing_segments, min_line_gap_m):
        return False
    if blocked_areas and not _line_clear_of_areas(points, blocked_areas, min_area_gap_m):
        return False
    if blocked_points and not _line_clear_of_takeovers(points, blocked_points, min_point_gap_m):
        return False
    if takeovers and not _line_clear_of_takeovers(points, takeovers, min_takeover_m):
        return False
    ratio = float(getattr(config, "DEM_PATH_MAX_ROUTE_RATIO", 0.0))
    if ratio > 0.0:
        path_len = _path_length_m(points)
        direct = _distance_m(points[0], points[-1])
        if direct > 1e-6 and path_len > direct * ratio:
            return False
    return True


def _composite_line_points(
    start: LatLon,
    end: LatLon,
    rng: random.Random,
    *,
    bounds: Tuple[float, float, float, float],
    existing_segments: Sequence[Tuple[LatLon, LatLon]],
    blocked_areas: Sequence[Sequence[LatLon]],
    blocked_points: Sequence[LatLon],
    takeovers: Sequence[LatLon],
    min_line_gap_m: float = 350.0,
    min_area_gap_m: float = 700.0,
    min_point_gap_m: float = 500.0,
    min_takeover_m: float = config.TAKEOVER_CLEARANCE_M,
) -> List[LatLon]:
    direct = [start, end]
    if not config.DEM_CORRIDOR_ENABLE:
        return direct

    try:
        dem_grid = int(getattr(config, "DEM_PATH_GRID_SIZE", config.DEM_3D_GRID_SIZE))
        dem_buffer = float(getattr(config, "DEM_PATH_BUFFER_M", config.DEM_3D_BUFFER_M))
        elev_weight = float(getattr(config, "DEM_PATH_ELEV_WEIGHT", 6.0))
        latlon_path = dem_corridor.low_terrain_path(
            start,
            end,
            dem_path=Path(config.DEM_CORRIDOR_FILE),
            grid_size=dem_grid,
            buffer_m=dem_buffer,
            elev_weight=elev_weight,
        )
        if latlon_path:
            coords_candidate = _shape_corridor_path_coords(latlon_path)
            if _composite_line_candidate_ok(
                coords_candidate,
                bounds=bounds,
                existing_segments=existing_segments,
                blocked_areas=blocked_areas,
                blocked_points=blocked_points,
                takeovers=takeovers,
                min_line_gap_m=min_line_gap_m,
                min_area_gap_m=min_area_gap_m,
                min_point_gap_m=min_point_gap_m,
                min_takeover_m=min_takeover_m,
            ):
                return coords_candidate
    except Exception:
        pass

    return direct


def _composite_area_edge_anchor(ref_point: LatLon, area: Sequence[LatLon], fallback: LatLon) -> LatLon:
    _dist, anchor = _area_edge_distance(ref_point, area)
    return anchor or fallback


def _nearest_area_edge_index(point: LatLon, area: Sequence[LatLon]) -> Optional[int]:
    edges = _poly_edges(area)
    if not edges:
        return None
    best_idx: Optional[int] = None
    best_dist = float("inf")
    for idx, (a, b) in enumerate(edges):
        dist = _point_segment_distance_m(point, a, b)
        if dist < best_dist:
            best_dist = dist
            best_idx = idx
    return best_idx


def _anchors_share_edge(a: LatLon, b: LatLon, area: Sequence[LatLon]) -> bool:
    edge_a = _nearest_area_edge_index(a, area)
    edge_b = _nearest_area_edge_index(b, area)
    return edge_a is not None and edge_a == edge_b


def _composite_area_exit_anchor(
    ref_point: LatLon,
    entry_anchor: LatLon,
    area: Sequence[LatLon],
    fallback: LatLon,
) -> LatLon:
    exit_anchor = _composite_area_edge_anchor(ref_point, area, fallback)
    entry_edge = _nearest_area_edge_index(entry_anchor, area)
    exit_edge = _nearest_area_edge_index(exit_anchor, area)
    if entry_edge is None or exit_edge is None or entry_edge != exit_edge:
        return exit_anchor

    best_anchor: Optional[LatLon] = None
    best_dist = float("inf")
    for idx, (a, b) in enumerate(_poly_edges(area)):
        if idx == entry_edge:
            continue
        candidate = _closest_point_on_segment(ref_point, a, b)
        dist = _distance_m(ref_point, candidate)
        if dist < best_dist:
            best_dist = dist
            best_anchor = candidate
    return best_anchor or exit_anchor


def _point_clear_of_areas(point: LatLon, areas: Sequence[Sequence[LatLon]], min_gap_m: float) -> bool:
    for area in areas:
        if len(area) < 3:
            continue
        if _point_in_poly(point, area):
            return False
        dist, _anchor = _area_edge_distance(point, area)
        if dist < min_gap_m:
            return False
    return True


def _points_clear(points: Sequence[LatLon], min_gap_m: float) -> bool:
    for idx, point in enumerate(points):
        for other in points[idx + 1 :]:
            if _distance_m(point, other) < min_gap_m:
                return False
    return True


def _composite_route_clear(
    route_nodes: Sequence[Dict],
    *,
    min_line_gap_m: float,
    min_area_gap_m: float,
    min_point_gap_m: float,
) -> bool:
    line_segments: List[Tuple[LatLon, LatLon]] = []
    for idx in range(len(route_nodes) - 1):
        start = route_nodes[idx]["out_anchor"]
        end = route_nodes[idx + 1]["in_anchor"]
        line_segments.append((start, end))

    for idx, seg in enumerate(line_segments):
        for other_idx in range(idx + 1, len(line_segments)):
            if other_idx == idx + 1:
                continue
            other_seg = line_segments[other_idx]
            if _segment_intersects_list(seg, [other_seg], skip_shared_endpoint=True):
                return False
            if _segment_min_distance(seg, [other_seg], skip_shared_endpoint=True) < min_line_gap_m:
                return False

    for node_idx, node in enumerate(route_nodes):
        if node["kind"] == "area":
            connected_lines = {node_idx - 1, node_idx}
            for line_idx, seg in enumerate(line_segments):
                if line_idx in connected_lines:
                    continue
                if not _line_clear_of_areas([seg[0], seg[1]], [node["coords"]], min_area_gap_m):
                    return False
            continue

        connected_lines = set()
        if node_idx > 0:
            connected_lines.add(node_idx - 1)
        if node_idx < len(route_nodes) - 1:
            connected_lines.add(node_idx)
        for line_idx, seg in enumerate(line_segments):
            if line_idx in connected_lines:
                continue
            if _point_segment_distance_m(node["coords"], seg[0], seg[1]) < min_point_gap_m:
                return False

    return True


def generate_composite_route(
    seed: Optional[int] = None,
    *,
    anchor: Optional[LatLon] = None,
    takeovers: Optional[Sequence[LatLon]] = None,
    max_start_offset_m: float = 2500.0,
    package_type: int = 2,
) -> Dict:
    _bootstrap_state_if_needed()

    rng = random.Random(seed)
    bounds = _bounds_with_margin(config.BORDER_MARGIN_M)
    route_anchor = anchor or _composite_route_anchor(takeovers)
    area_center = LatLon(
        (AUTO_MISSION_AREA.southwest.latitude + AUTO_MISSION_AREA.northeast.latitude) / 2.0,
        (AUTO_MISSION_AREA.southwest.longitude + AUTO_MISSION_AREA.northeast.longitude) / 2.0,
    )
    route_anchor_east, _route_anchor_north = _to_xy(route_anchor)
    layout_shift_east = max(-1_200.0, min(1_200.0, route_anchor_east * 0.18))
    if abs(route_anchor_east) < 1_800.0:
        outbound_sign = rng.choice((-1.0, 1.0))
    else:
        outbound_sign = -1.0 if route_anchor_east < 0.0 else 1.0
    return_sign = -outbound_sign

    route_nodes: Optional[List[Dict]] = None
    route_line_paths: Optional[List[List[LatLon]]] = None
    for _ in range(40):
        route_start = _start_from_anchor(route_anchor, bounds, rng, None, max_start_offset_m)
        if route_start is None:
            continue
        region_1 = route_start
        ccr_band_north = rng.uniform(-6_900.0, -5_200.0)
        acp_band_north = ccr_band_north + rng.uniform(2_000.0, 3_100.0)
        attack_band_north = acp_band_north + rng.uniform(2_200.0, 3_900.0)
        battle_band_north = acp_band_north + rng.uniform(1_900.0, 3_400.0)
        target_band_north = rng.uniform(5_900.0, 7_500.0)
        region_2 = _offset_point(
            area_center,
            layout_shift_east + outbound_sign * rng.uniform(5_600.0, 8_600.0),
            ccr_band_north + rng.uniform(-220.0, 220.0),
        )
        region_3 = _offset_point(
            area_center,
            layout_shift_east + outbound_sign * rng.uniform(4_200.0, 7_800.0),
            acp_band_north + rng.uniform(-280.0, 280.0),
        )
        attack_center = _offset_point(
            area_center,
            layout_shift_east * 0.8 + outbound_sign * rng.uniform(2_800.0, 6_900.0),
            attack_band_north + rng.uniform(-380.0, 380.0),
        )
        battle_center = _offset_point(
            area_center,
            layout_shift_east * 0.6 + return_sign * rng.uniform(2_000.0, 5_200.0),
            battle_band_north + rng.uniform(-420.0, 420.0),
        )
        target_center = _offset_point(
            area_center,
            rng.uniform(-500.0, 500.0),
            target_band_north + rng.uniform(-120.0, 120.0),
        )
        region_7 = _offset_point(
            area_center,
            layout_shift_east * 0.8 + return_sign * rng.uniform(5_600.0, 8_600.0),
            acp_band_north + rng.uniform(-320.0, 320.0),
        )
        region_8 = _offset_point(
            area_center,
            layout_shift_east * 0.8 + return_sign * rng.uniform(4_200.0, 7_800.0),
            ccr_band_north + rng.uniform(-260.0, 260.0),
        )

        attack_area = _composite_area_coords(
            attack_center,
            rng,
            bounds,
            width_range_m=(2_500.0, 3_600.0),
            height_range_m=(2_000.0, 2_900.0),
            pentagon_ratio=0.45,
        )
        battle_area = _composite_area_coords(
            battle_center,
            rng,
            bounds,
            width_range_m=(2_700.0, 3_800.0),
            height_range_m=(2_000.0, 3_000.0),
            pentagon_ratio=0.45,
        )
        target_area = _composite_area_coords(
            target_center,
            rng,
            bounds,
            width_range_m=(6_200.0, 6_200.0),
            height_range_m=(4_700.0, 4_700.0),
            pentagon_ratio=0.0,
        )

        point_nodes = [region_1, region_2, region_3, region_7, region_8]
        area_nodes = [attack_area, battle_area, target_area]
        if not all(_inside_bounds(point, bounds) for point in point_nodes):
            continue
        if not all(all(_inside_bounds(pt, bounds) for pt in area) for area in area_nodes):
            continue
        if not _points_clear(point_nodes, 1_500.0):
            continue
        if not _point_clear_of_areas(region_2, area_nodes, 1_200.0):
            continue
        if not _point_clear_of_areas(region_3, area_nodes, 1_700.0):
            continue
        if not _point_clear_of_areas(region_7, area_nodes, 1_500.0):
            continue
        if not _point_clear_of_areas(region_8, area_nodes, 1_200.0):
            continue
        if not _area_clear_of_areas(attack_area, [battle_area, target_area], 1_700.0):
            continue
        if not _area_clear_of_areas(battle_area, [attack_area, target_area], 1_700.0):
            continue
        if not _area_clear_of_areas(target_area, [attack_area, battle_area], 2_400.0):
            continue
        candidate_nodes = [
            {"kind": "point", "region_type": 1, "mission_type": 2, "anchor": region_1, "coords": region_1},
            {"kind": "point", "region_type": 2, "mission_type": 2, "anchor": region_2, "coords": region_2},
            {"kind": "point", "region_type": 3, "mission_type": 2, "anchor": region_3, "coords": region_3},
            {"kind": "area", "region_type": 4, "mission_type": 2, "anchor": attack_center, "coords": attack_area},
            {"kind": "area", "region_type": 5, "mission_type": 2, "anchor": battle_center, "coords": battle_area},
            {"kind": "area", "region_type": 6, "mission_type": 2, "anchor": target_center, "coords": target_area},
            {"kind": "point", "region_type": 3, "mission_type": 2, "anchor": region_7, "coords": region_7},
            {"kind": "point", "region_type": 2, "mission_type": 2, "anchor": region_8, "coords": region_8},
        ]

        for node in candidate_nodes:
            node["in_anchor"] = node["anchor"]
            node["out_anchor"] = node["anchor"]

        for idx, node in enumerate(candidate_nodes):
            if node["kind"] != "area":
                continue
            prev_ref = candidate_nodes[idx - 1]["out_anchor"] if idx > 0 else node["anchor"]
            next_ref = candidate_nodes[idx + 1]["anchor"] if idx < len(candidate_nodes) - 1 else node["anchor"]
            node["in_anchor"] = _composite_area_edge_anchor(prev_ref, node["coords"], node["anchor"])
            node["out_anchor"] = _composite_area_exit_anchor(
                next_ref,
                node["in_anchor"],
                node["coords"],
                node["anchor"],
            )

        if not _composite_route_clear(
            candidate_nodes,
            min_line_gap_m=350.0,
            min_area_gap_m=700.0,
            min_point_gap_m=500.0,
        ):
            continue

        candidate_line_paths: List[List[LatLon]] = []
        candidate_segments: List[Tuple[LatLon, LatLon]] = []
        line_build_failed = False
        for idx in range(len(candidate_nodes) - 1):
            blocked_areas = [
                node["coords"]
                for node_idx, node in enumerate(candidate_nodes)
                if node["kind"] == "area" and node_idx not in (idx, idx + 1)
            ]
            blocked_points = [
                node["coords"]
                for node_idx, node in enumerate(candidate_nodes)
                if node["kind"] == "point" and node_idx not in (idx, idx + 1)
            ]
            line_points = _composite_line_points(
                candidate_nodes[idx]["out_anchor"],
                candidate_nodes[idx + 1]["in_anchor"],
                rng,
                bounds=bounds,
                existing_segments=candidate_segments,
                blocked_areas=blocked_areas,
                blocked_points=blocked_points,
                takeovers=(),
            )
            if len(line_points) < 2:
                line_build_failed = True
                break
            candidate_line_paths.append(line_points)
            candidate_segments.extend(_segments_from_points(line_points))
        if line_build_failed:
            continue

        route_nodes = candidate_nodes
        route_line_paths = candidate_line_paths
        break

    if route_nodes is None or route_line_paths is None:
        raise RuntimeError("Failed to place composite-route missions with required spacing")

    missions: List[Dict] = []
    mission_id = 1
    aircraft_count = len(config.UAV_IDS)

    for idx, node in enumerate(route_nodes):
        if node["kind"] == "point":
            missions.append(
                _point_mission(
                    mission_id,
                    node["coords"],
                    mission_type=int(node["mission_type"]),
                    region_type=int(node["region_type"]),
                )
            )
        else:
            missions.append(
                _area_mission(
                    mission_id,
                    list(node["coords"]),
                    mission_type=int(node["mission_type"]),
                    region_type=int(node["region_type"]),
                )
            )
        mission_id += 1

        if idx >= len(route_nodes) - 1:
            continue

        line_alt = rng.uniform(config.LINE_ALT_MIN_M, config.LINE_ALT_MAX_M)
        line_points = route_line_paths[idx]
        missions.append(
            _line_mission(
                mission_id,
                line_points,
                _line_width(rng, aircraft_count),
                line_alt,
                mission_type=1,
                region_type=0,
            )
        )
        mission_id += 1

    pkg_id, file_seq = _next_ids(package_type)
    return {
        "timestamp": now_ms_2000(),
        "inputMissionPackageID": pkg_id,
        "inputMissionPackageType": int(package_type),
        "mainSensor": _pick_main_sensor(rng),
        "availableAircraftList": [{"aircraftID": i} for i in config.AIRCRAFT_IDS],
        "inputMissionList": missions,
        "_meta": {"fileSeq": file_seq, "seed": seed, "generationMode": "composite_route"},
    }


def _mission_anchor_point(mission: Dict) -> Optional[LatLon]:
    detail = mission.get("missionDetail") or {}
    line_list = detail.get("lineList") or []
    if line_list:
        coords = line_list[0].get("coordinateList") or []
        if coords:
            return LatLon(coords[0]["latitude"], coords[0]["longitude"])
    area_list = detail.get("areaList") or []
    if area_list:
        coords = area_list[0].get("coordinateList") or []
        if coords:
            lat = sum(c["latitude"] for c in coords) / len(coords)
            lon = sum(c["longitude"] for c in coords) / len(coords)
            return LatLon(lat, lon)
    point_list = detail.get("coordinateList") or []
    if point_list:
        return LatLon(point_list[0]["latitude"], point_list[0]["longitude"])
    return None


def _reassign_ids_by_distance(missions: List[Dict], ref_point: LatLon) -> List[Dict]:
    def key_fn(mission: Dict) -> float:
        pt = _mission_anchor_point(mission)
        if not pt:
            return float("inf")
        return _distance_m(ref_point, pt)

    ordered = sorted(missions, key=key_fn)
    for idx, mission in enumerate(ordered, start=1):
        mission["inputMissionID"] = idx
    return ordered


def _line_endpoints(mission: Dict) -> Optional[Tuple[LatLon, LatLon]]:
    detail = mission.get("missionDetail") or {}
    line_list = detail.get("lineList") or []
    if not line_list:
        return None
    coords = line_list[0].get("coordinateList") or []
    if len(coords) < 2:
        return None
    start = LatLon(coords[0]["latitude"], coords[0]["longitude"])
    end = LatLon(coords[-1]["latitude"], coords[-1]["longitude"])
    return start, end


def _reverse_line_coords(mission: Dict) -> None:
    detail = mission.get("missionDetail") or {}
    for line in detail.get("lineList") or []:
        coords = line.get("coordinateList")
        if coords and len(coords) > 1:
            coords.reverse()


def _area_coords(mission: Dict) -> Optional[List[LatLon]]:
    detail = mission.get("missionDetail") or {}
    area_list = detail.get("areaList") or []
    if not area_list:
        return None
    coords = area_list[0].get("coordinateList") or []
    if len(coords) < 3:
        return None
    return [LatLon(c["latitude"], c["longitude"]) for c in coords]


def _closest_point_on_segment(p: LatLon, a: LatLon, b: LatLon) -> LatLon:
    pe, pn = _to_xy(p)
    ae, an = _to_xy(a)
    be, bn = _to_xy(b)
    ab_e = be - ae
    ab_n = bn - an
    ab2 = ab_e * ab_e + ab_n * ab_n
    if ab2 == 0.0:
        return a
    t = ((pe - ae) * ab_e + (pn - an) * ab_n) / ab2
    t = max(0.0, min(1.0, t))
    lat = a.latitude + (b.latitude - a.latitude) * t
    lon = a.longitude + (b.longitude - a.longitude) * t
    return LatLon(lat, lon)


def _area_edge_distance(pt: LatLon, area: Sequence[LatLon]) -> Tuple[float, Optional[LatLon]]:
    if len(area) < 3:
        return float("inf"), None
    inside = _point_in_poly(pt, area)
    edges = _poly_edges(area)
    best_dist = float("inf")
    best_pt: Optional[LatLon] = None
    for a, b in edges:
        dist = _point_segment_distance_m(pt, a, b)
        if dist < best_dist:
            best_dist = dist
            best_pt = _closest_point_on_segment(pt, a, b)
    if inside:
        return 0.0, best_pt
    return best_dist, best_pt


def _ray_segment_intersection(
    origin: Tuple[float, float],
    direction: Tuple[float, float],
    a: Tuple[float, float],
    b: Tuple[float, float],
) -> Optional[Tuple[float, float, float]]:
    ox, oy = origin
    dx, dy = direction
    ax, ay = a
    bx, by = b
    rx = bx - ax
    ry = by - ay
    denom = dx * ry - dy * rx
    if abs(denom) < 1e-9:
        return None
    t = ((ax - ox) * ry - (ay - oy) * rx) / denom
    u = ((ax - ox) * dy - (ay - oy) * dx) / denom
    if t < 0.0 or u < 0.0 or u > 1.0:
        return None
    return t, u, denom


def _area_exit_point(entry: LatLon, area: Sequence[LatLon]) -> LatLon:
    if len(area) < 3:
        return entry
    centroid = LatLon(
        sum(p.latitude for p in area) / len(area),
        sum(p.longitude for p in area) / len(area),
    )
    ce, cn = _to_xy(centroid)
    ee, en = _to_xy(entry)
    vec_e = ee - ce
    vec_n = en - cn
    norm = math.hypot(vec_e, vec_n)
    if norm < 1e-6:
        farthest = max(area, key=lambda p: _distance_m(entry, p))
        return farthest
    dir_e = -vec_e / norm
    dir_n = -vec_n / norm
    origin = (ce, cn)
    direction = (dir_e, dir_n)
    edges = _poly_edges(area)
    best_t = float("inf")
    best_point: Optional[LatLon] = None
    for a, b in edges:
        ax, ay = _to_xy(a)
        bx, by = _to_xy(b)
        hit = _ray_segment_intersection(origin, direction, (ax, ay), (bx, by))
        if not hit:
            continue
        t, u, _ = hit
        if t < best_t:
            best_t = t
            lat = a.latitude + (b.latitude - a.latitude) * u
            lon = a.longitude + (b.longitude - a.longitude) * u
            best_point = LatLon(lat, lon)
    if best_point:
        return best_point
    farthest = max(area, key=lambda p: _distance_m(entry, p))
    return farthest


def _pentagon_points(center: LatLon, radius_m: float, rng: random.Random) -> List[LatLon]:
    base_angle = rng.uniform(0.0, 360.0)
    angles = []
    for i in range(5):
        angles.append(base_angle + i * 72.0 + rng.uniform(-10.0, 10.0))
    angles.sort()
    points: List[LatLon] = []
    for ang in angles:
        rad = math.radians(ang)
        r = radius_m * rng.uniform(0.8, 1.0)
        east = r * math.cos(rad)
        north = r * math.sin(rad)
        lat, lon = offset_lat_lon(center.latitude, center.longitude, east, north)
        points.append(LatLon(lat, lon))
    return points


def _reorder_missions_nearest(missions: List[Dict], start_point: LatLon) -> List[Dict]:
    remaining = list(missions)
    ordered: List[Dict] = []
    current = start_point

    while remaining:
        best_idx = 0
        best_dist = float("inf")
        best_end: Optional[LatLon] = None
        best_reverse = False

        for idx, mission in enumerate(remaining):
            endpoints = _line_endpoints(mission)
            if endpoints:
                start, end = endpoints
                dist_start = _distance_m(current, start)
                dist_end = _distance_m(current, end)
                if dist_end < dist_start:
                    dist = dist_end
                    end_point = start
                    reverse = True
                else:
                    dist = dist_start
                    end_point = end
                    reverse = False
            else:
                area = _area_coords(mission)
                if area:
                    dist, nearest = _area_edge_distance(current, area)
                    if nearest:
                        end_point = _area_exit_point(nearest, area)
                    else:
                        end_point = _mission_anchor_point(mission)
                    reverse = False
                else:
                    anchor = _mission_anchor_point(mission)
                    if not anchor:
                        continue
                    dist = _distance_m(current, anchor)
                    end_point = anchor
                    reverse = False

            if dist < best_dist:
                best_dist = dist
                best_idx = idx
                best_end = end_point
                best_reverse = reverse

        mission = remaining.pop(best_idx)
        if best_reverse:
            _reverse_line_coords(mission)
        ordered.append(mission)
        if best_end:
            current = best_end

    for idx, mission in enumerate(ordered, start=1):
        mission["inputMissionID"] = idx
    return ordered





def _start_from_anchor(

    anchor: LatLon,

    bounds: Tuple[float, float, float, float],

    rng: random.Random,

    heading_hint: Optional[float],

    max_start_offset_m: float,

) -> Optional[LatLon]:

    """

    Choose a start point near the anchor while leaving room for the first leg.

    """

    min_offset = max(0.0, float(config.START_OFFSET_MIN_M))
    _, _, ne_lat, _ = bounds
    north_bound = LatLon(ne_lat, anchor.longitude)
    available_north_m = _distance_m(anchor, north_bound)
    config_max_offset = max(0.0, float(config.START_OFFSET_MAX_M))
    request_max_offset = max(0.0, float(max_start_offset_m))
    max_offset = min(config_max_offset, request_max_offset, available_north_m)
    if max_offset < min_offset:
        return None

    def _forward_room_m(point: LatLon, heading_deg: float) -> float:
        sw_lat, sw_lon, ne_lat, ne_lon = bounds
        center_lat = (sw_lat + ne_lat) / 2.0
        meters_per_deg_lat = 111_320.0
        meters_per_deg_lon = meters_per_deg_lat * math.cos(math.radians(center_lat))
        bounds_w = abs(ne_lon - sw_lon) * meters_per_deg_lon
        bounds_h = abs(ne_lat - sw_lat) * meters_per_deg_lat
        hi = max(1000.0, math.hypot(bounds_w, bounds_h))
        lo = 0.0
        for _ in range(18):
            mid = (lo + hi) / 2.0
            if _inside_bounds(_move(point, mid, heading_deg), bounds):
                lo = mid
            else:
                hi = mid
        return lo

    preferred_heading = None if heading_hint is None else (float(heading_hint) + 180.0) % 360.0
    best_candidate: Optional[LatLon] = None
    best_room = -1.0
    spread = min(85.0, max(20.0, float(config.HEADING_DELTA_MAX_DEG)))

    for attempt in range(config.RECT_ATTEMPTS):
        offset_m = rng.uniform(min_offset, max_offset)
        if preferred_heading is None:
            bearing = 0.0 if attempt == 0 else rng.uniform(0.0, 360.0)
        else:
            if attempt == 0:
                bearing = preferred_heading
            else:
                bearing = (preferred_heading + rng.uniform(-spread, spread)) % 360.0
        cand = _move(anchor, offset_m, bearing)
        if not _inside_bounds(cand, bounds):
            continue
        if heading_hint is None:
            return cand
        room = _forward_room_m(cand, float(heading_hint))
        if room > best_room:
            best_candidate = cand
            best_room = room
            if room >= max(200.0, float(config.LINE_SEGMENT_MIN_M) / 2.0):
                return cand

    if best_candidate is not None:
        return best_candidate

    for _ in range(config.RECT_ATTEMPTS):
        offset_m = rng.uniform(min_offset, max_offset)
        cand = _move(anchor, offset_m, 0.0)
        if _inside_bounds(cand, bounds):
            return cand
    return None





def _build_missions(

    rng: random.Random,

    bounds: Tuple[float, float, float, float],

    anchor: Optional[LatLon] = None,

    heading_hint: Optional[float] = None,

    max_start_offset_m: float = 2500.0,

) -> Tuple[List[LatLon], float, List[LatLon], List[LatLon], float]:

    aircraft_count = len(config.UAV_IDS)

    for _ in range(config.MAX_GEN_ATTEMPTS):

        if anchor:

            start = _start_from_anchor(anchor, bounds, rng, heading_hint, max_start_offset_m) or _random_point(bounds, rng)

        else:

            start = _random_point(bounds, rng)

        line1, width1 = _generate_line_path(

            start,

            bounds,

            rng,

            avoid_segments=[],

            min_gap_m=0.0,

            heading=heading_hint,

            aircraft_count=aircraft_count,

        )

        if not line1:

            continue

        heading_ref = _bearing_deg(line1[-2], line1[-1]) if len(line1) >= 2 else rng.uniform(0.0, 360.0)

        area_params = _place_area_near_point(line1[-1], bounds, rng, line1, heading_ref)

        if not area_params:

            continue

        area_center, area_w, area_h, area_corners = area_params

        area_edges = _rect_edges(area_corners)

        # Choose the opposite edge of the one closest to the incoming line end to keep handoff cleaner.

        edge_dirs = ("W", "N", "E", "S")

        dist_by_edge = [_min_distance_point_edges(line1[-1], [edge]) for edge in area_edges]

        nearest_idx = dist_by_edge.index(min(dist_by_edge))

        preferred_dir = edge_dirs[(nearest_idx + 2) % 4]

        line1_segments = _segments_from_points(line1)

        line2_start = _point_off_rect(

            area_center,

            area_w,

            area_h,

            area_corners,

            bounds,

            rng,

            avoid_segments=line1_segments,

            heading_ref=heading_ref,

            preferred_direction=preferred_dir,

            force_direction=True,

        )

        if not line2_start:

            continue

        avoid_for_line2 = list(line1_segments) + area_edges

        line2, width2 = _generate_line_path(

            line2_start,

            bounds,

            rng,

            avoid_segments=avoid_for_line2,

            min_gap_m=config.EDGE_GAP_MIN_M,

            heading=_bearing_deg(area_center, line2_start),

            aircraft_count=aircraft_count,

        )

        if not line2:

            continue

        if _min_distance_polyline_edges(line2, area_edges) < config.EDGE_GAP_MIN_M:

            continue

        return line1, width1, area_corners, line2, width2

    raise RuntimeError("Failed to generate missions within attempt budget")


def _random_point(bounds: Tuple[float, float, float, float], rng: random.Random) -> LatLon:

    sw_lat, sw_lon, ne_lat, ne_lon = bounds

    lat = rng.uniform(sw_lat, ne_lat)

    lon = rng.uniform(sw_lon, ne_lon)

    return LatLon(lat, lon)

def _city_bounds() -> Tuple[float, float, float, float]:
    lat1, lon1 = config.CITY_MISSION_AREA_NW
    lat2, lon2 = config.CITY_MISSION_AREA_SE
    sw_lat = min(lat1, lat2)
    ne_lat = max(lat1, lat2)
    sw_lon = min(lon1, lon2)
    ne_lon = max(lon1, lon2)
    return sw_lat, sw_lon, ne_lat, ne_lon


def _city_center(bounds: Tuple[float, float, float, float]) -> LatLon:
    sw_lat, sw_lon, ne_lat, ne_lon = bounds
    return LatLon((sw_lat + ne_lat) / 2.0, (sw_lon + ne_lon) / 2.0)


def _point_in_any_area(pt: LatLon, areas: Sequence[Sequence[LatLon]]) -> bool:
    for area in areas:
        if len(area) < 3:
            continue
        if _point_in_poly(pt, area):
            return True
    return False


def _area_radius(center: LatLon, area: Sequence[LatLon]) -> float:
    if not area:
        return 0.0
    return max(_distance_m(center, corner) for corner in area)


def _next_start_after_area(
    center: LatLon,
    area: Sequence[LatLon],
    heading_ref: Optional[float],
    bounds: Tuple[float, float, float, float],
    rng: random.Random,
    min_gap_m: float,
) -> LatLon:
    radius = _area_radius(center, area)
    heading = heading_ref if heading_ref is not None else rng.uniform(0.0, 360.0)
    candidate = _move(center, radius + min_gap_m, heading)
    if _inside_bounds(candidate, bounds) and not _point_in_poly(candidate, area):
        return candidate
    fallback = _random_point_near(
        bounds,
        center,
        rng,
        max_offset_m=radius + min_gap_m * 2.0,
        min_offset_m=max(0.0, radius + min_gap_m * 0.5),
    )
    if fallback and not _point_in_poly(fallback, area):
        return fallback
    return center





def _random_point_near(

    bounds: Tuple[float, float, float, float],

    anchor: LatLon,

    rng: random.Random,

    max_offset_m: float,

    min_offset_m: float = 0.0,

) -> Optional[LatLon]:

    """

    Pick a point near an anchor (within max_offset_m, outside min_offset_m) that still respects bounds.

    Returns None if no suitable point is found.

    """

    if max_offset_m <= 0:

        return None

    for _ in range(config.RECT_ATTEMPTS):

        dist = rng.uniform(min_offset_m, max_offset_m)

        bearing = rng.uniform(0.0, 360.0)

        east = dist * math.sin(math.radians(bearing))

        north = dist * math.cos(math.radians(bearing))

        lat, lon = offset_lat_lon(anchor.latitude, anchor.longitude, east, north)

        pt = LatLon(lat, lon)

        if _inside_bounds(pt, bounds):

            return pt

    return None





def generate(

    seed: Optional[int] = None,

    *,

    anchor: Optional[LatLon] = None,

    heading_hint: Optional[float] = None,

    max_start_offset_m: float = 2500.0,

    package_type: Optional[int] = None,
    takeovers: Optional[Sequence[LatLon]] = None,

) -> Dict:

    """

    Build an InputMissionPlan payload with a line-area-line structure.



    Args:

        seed: Optional random seed.

        anchor: Optional starting bias (e.g., takeOver point). First line start stays near this point when possible.

        heading_hint: Optional initial heading in degrees for the first leg.

        max_start_offset_m: Max distance from anchor when selecting the start point.

    """

    # If anchor is given but no heading_hint, default to northbound (triangle tip points north).

    if anchor is not None and heading_hint is None:

        heading_hint = 0.0

    _bootstrap_state_if_needed()

    rng = random.Random(seed)
    bounds = _bounds_with_margin(config.BORDER_MARGIN_M)
    if package_type is None:
        package_type = rng.randint(1, 5)
    network = None
    if config.DEM_CORRIDOR_ENABLE and getattr(config, "DEM_CORRIDOR_USE_NETWORK", True):
        try:
            network = dem_corridor.build_corridor_network(
                dem_path=Path(config.DEM_CORRIDOR_FILE),
                flow_threshold=config.DEM_CORRIDOR_FLOW_ACC_THRESHOLD,
            )
        except Exception:
            network = None
    allow_astar = bool(config.DEM_CORRIDOR_ENABLE)
    mission_count = rng.randint(config.INPUT_MISSION_COUNT_RANGE[0], config.INPUT_MISSION_COUNT_RANGE[1])
    if mission_count < 1:
        mission_count = 1

    line_min, line_max = config.LINE_MISSION_COUNT_RANGE
    line_max = min(line_max, mission_count)
    line_min = min(line_min, line_max)
    if package_type == 5:
        # Exactly one area mission; lines are allowed.
        line_max = min(line_max, mission_count - 1)
        if line_max < 0:
            line_max = 0
        line_min = min(line_min, line_max)
        line_count = rng.randint(line_min, line_max) if line_max > 0 else 0
        point_count = 1
        mission_count = line_count + point_count
    else:
        line_count = rng.randint(line_min, line_max) if line_max > 0 else mission_count
        point_count = max(0, mission_count - line_count)
        if point_count <= 0 and line_count > line_min:
            line_count -= 1
            point_count = max(1, mission_count - line_count)

    speed_scale = 1.0
    if mission_count >= 7:
        speed_scale = 0.5
    elif mission_count >= 5:
        speed_scale = 0.7

    takeovers = list(takeovers) if takeovers else []
    min_gap_m = config.EDGE_GAP_MIN_M
    min_takeover_m = config.START_OFFSET_MIN_M

    line_type_set = {1, 7}
    if package_type == 5:
        non_line_types = {k: v for k, v in config.POINT_MISSION_TYPE_WEIGHTS.items() if k not in {1, 7}}
        point_types = [_weighted_choice(rng, non_line_types, default=2)]
    else:
        point_types = [
            _weighted_choice(rng, config.POINT_MISSION_TYPE_WEIGHTS, default=2)
            for _ in range(point_count)
        ]
    extra_line_types = [t for t in point_types if t in line_type_set]
    area_types = [t for t in point_types if t not in line_type_set]
    if not area_types and point_types:
        point_types[0] = rng.choice([2, 3, 6])
        extra_line_types = [t for t in point_types if t in line_type_set]
        area_types = [t for t in point_types if t not in line_type_set]

    missions: List[Dict] = []
    city_center = _city_center(_city_bounds()) if package_type == 5 else None
    for _ in range(config.MAX_GEN_ATTEMPTS):
        start = _start_from_anchor(anchor, bounds, rng, heading_hint, max_start_offset_m) if anchor else None
        if not start:
            if anchor:
                continue
            start = _random_point(bounds, rng)

        line_min_len = float(config.LINE_SEGMENT_MIN_M)
        line_max_len = float(config.LINE_SEGMENT_MAX_M)
        if line_max_len < line_min_len:
            line_max_len = line_min_len
        seg_min = max(200.0, line_min_len / 2.0)
        seg_max = max(seg_min, line_max_len / 2.0)
        total_line_count = line_count + len(extra_line_types)

        missions = []
        existing_segments: List[Tuple[LatLon, LatLon]] = []
        existing_areas: List[List[LatLon]] = []
        mission_id = 1
        aircraft_count = len(config.UAV_IDS)
        ok = True

        line_types: List[int] = []
        for _ in range(line_count):
            line_types.append(_weighted_choice(rng, config.LINE_MISSION_TYPE_WEIGHTS, default=1))
        line_types.extend(extra_line_types)

        sequence: List[Tuple[str, int]] = []
        line_queue = list(line_types)
        area_queue = list(area_types)
        while line_queue or area_queue:
            if line_queue:
                sequence.append(("line", line_queue.pop(0)))
            if area_queue:
                sequence.append(("area", area_queue.pop(0)))

        cursor = start
        heading_ref = heading_hint
        city_area_used = False

        for kind, mission_type in sequence:
            if kind == "line":
                success = False
                attempts = config.SEGMENT_ATTEMPTS * 2
                desired_len = rng.uniform(line_min_len, line_max_len)
                hard_fail = False
                for attempt in range(attempts):
                    start_pt = cursor
                    if attempt > 0:
                        if mission_id == 1 and anchor:
                            alt = _start_from_anchor(anchor, bounds, rng, heading_hint, max_start_offset_m)
                        else:
                            alt = _random_point_near(
                                bounds,
                                cursor,
                                rng,
                                max_offset_m=config.EDGE_GAP_MAX_M,
                                min_offset_m=min_gap_m,
                            )
                        if alt:
                            start_pt = alt
                    if existing_segments and _min_distance_point_segments(start_pt, existing_segments) < min_gap_m:
                        continue
                    if existing_areas and _point_in_any_area(start_pt, existing_areas):
                        offset = _random_point_near(
                            bounds,
                            start_pt,
                            rng,
                            max_offset_m=config.EDGE_GAP_MAX_M,
                            min_offset_m=config.EDGE_GAP_MIN_M,
                        )
                        if offset:
                            start_pt = offset
                    coords = None
                    if config.DEM_CORRIDOR_ENABLE:
                        sample_count = max(20, int(getattr(config, "DEM_PATH_ELEV_SAMPLES", 120) * speed_scale))
                        elev_limit = float(getattr(config, "DEM_PATH_MAX_ELEV_M", 0.0))
                        if network:
                            start_xy, start_node = dem_corridor.snap_to_network(network, start_pt)
                            component = dem_corridor.network_component(network, start_node) if start_node else []
                            if component:
                                min_len = line_min_len
                                max_len = line_max_len
                                candidates = [
                                    n
                                    for n in component
                                    if min_len <= ((n[0] - start_xy[0]) ** 2 + (n[1] - start_xy[1]) ** 2) ** 0.5 <= max_len
                                ]
                                if not candidates:
                                    candidates = component
                                rng.shuffle(candidates)
                                max_goal_tries = max(8, int(getattr(config, "DEM_PATH_GOAL_ATTEMPTS", 60) * speed_scale))
                                best_score = None
                                for node in candidates[:max_goal_tries]:
                                    goal_latlon = dem_corridor.latlon_from_utm(network, [node])[0]
                                    if not _inside_bounds(goal_latlon, bounds):
                                        continue
                                    if takeovers and min_takeover_m > 0:
                                        if any(_distance_m(goal_latlon, pt) < min_takeover_m for pt in takeovers):
                                            continue
                                    path_nodes = dem_corridor.path_between_nodes(
                                        network,
                                        start_node,
                                        node,
                                        start_xy=start_xy,
                                        goal_xy=node,
                                    )
                                    if not path_nodes:
                                        continue
                                    latlon_path = dem_corridor.latlon_from_utm(network, path_nodes)
                                    if any(not _inside_bounds(p, bounds) for p in latlon_path):
                                        continue
                                    coords_candidate = _shape_corridor_path_coords(latlon_path)
                                    if not coords_candidate:
                                        continue
                                    if existing_segments and not _line_clear_of_lines(coords_candidate, existing_segments, min_gap_m):
                                        continue
                                    if existing_areas and not _line_clear_of_areas(coords_candidate, existing_areas, min_gap_m):
                                        continue
                                    if takeovers and not _line_clear_of_takeovers(coords_candidate, takeovers, min_takeover_m):
                                        continue
                                    ratio = float(getattr(config, "DEM_PATH_MAX_ROUTE_RATIO", 0.0))
                                    if ratio > 0.0:
                                        path_len = _path_length_m(coords_candidate)
                                        direct = _distance_m(coords_candidate[0], coords_candidate[-1])
                                        if direct > 1e-6 and path_len > direct * ratio:
                                            continue
                                    stats = dem_corridor.path_elevation_stats(
                                        Path(config.DEM_CORRIDOR_FILE),
                                        latlon_path,
                                        sample_count=sample_count,
                                    )
                                    if stats:
                                        mean_elev, max_elev = stats
                                        if elev_limit > 0.0 and max_elev > elev_limit:
                                            continue
                                        score = (max_elev, mean_elev, _path_length_m(coords_candidate))
                                    else:
                                        score = (float("inf"), float("inf"), _path_length_m(coords_candidate))
                                    if best_score is None or score < best_score:
                                        best_score = score
                                        coords = coords_candidate
                        if coords is None and network and getattr(config, "DEM_CORRIDOR_ALLOW_LINE_FALLBACK", True):
                            corridor_line = dem_corridor.build_corridor_line(
                                start_pt,
                                dem_path=Path(config.DEM_CORRIDOR_FILE),
                                flow_threshold=config.DEM_CORRIDOR_FLOW_ACC_THRESHOLD,
                                min_length_m=0.0,
                            )
                            if corridor_line and corridor_line.points:
                                cum = _cumulative_distances(corridor_line.points)
                                if cum and cum[-1] > 0.0:
                                    segment = _subpath(corridor_line.points, cum, 0.0, desired_len)
                                else:
                                    segment = corridor_line.points
                                coords = _shape_corridor_path_coords(segment)
                        if coords is None and allow_astar:
                            dem_grid = int(getattr(config, "DEM_PATH_GRID_SIZE", config.DEM_3D_GRID_SIZE))
                            dem_buffer = float(getattr(config, "DEM_PATH_BUFFER_M", config.DEM_3D_BUFFER_M))
                            elev_weight = float(getattr(config, "DEM_PATH_ELEV_WEIGHT", 6.0))
                            max_goal_tries = max(8, int(getattr(config, "DEM_PATH_GOAL_ATTEMPTS", 60) * speed_scale))
                            best_score = None
                            for _ in range(max_goal_tries):
                                bearing = rng.uniform(0.0, 360.0)
                                dist = rng.uniform(line_min_len, line_max_len)
                                goal_pt = _move(start_pt, dist, bearing)
                                if not _inside_bounds(goal_pt, bounds):
                                    continue
                                if takeovers and min_takeover_m > 0:
                                    if any(_distance_m(goal_pt, pt) < min_takeover_m for pt in takeovers):
                                        continue
                                latlon_path = dem_corridor.low_terrain_path(
                                    start_pt,
                                    goal_pt,
                                    dem_path=Path(config.DEM_CORRIDOR_FILE),
                                    grid_size=dem_grid,
                                    buffer_m=dem_buffer,
                                    elev_weight=elev_weight,
                                )
                                if not latlon_path:
                                    continue
                                if any(not _inside_bounds(p, bounds) for p in latlon_path):
                                    continue
                                coords_candidate = _shape_corridor_path_coords(latlon_path)
                                if not coords_candidate:
                                    continue
                                if existing_segments and not _line_clear_of_lines(coords_candidate, existing_segments, min_gap_m):
                                    continue
                                if existing_areas and not _line_clear_of_areas(coords_candidate, existing_areas, min_gap_m):
                                    continue
                                if takeovers and not _line_clear_of_takeovers(coords_candidate, takeovers, min_takeover_m):
                                    continue
                                stats = dem_corridor.path_elevation_stats(
                                    Path(config.DEM_CORRIDOR_FILE),
                                    latlon_path,
                                    sample_count=sample_count,
                                )
                                if stats:
                                    mean_elev, max_elev = stats
                                    if elev_limit > 0.0 and max_elev > elev_limit:
                                        continue
                                    score = (max_elev, mean_elev, _path_length_m(coords_candidate))
                                else:
                                    score = (float("inf"), float("inf"), _path_length_m(coords_candidate))
                                if best_score is None or score < best_score:
                                    best_score = score
                                    coords = coords_candidate
                    if coords is None:
                        coords, _, _ = _generate_line_chain(
                            start_pt,
                            bounds,
                            rng,
                            segment_count=2,
                            heading=heading_ref,
                            min_gap_m=0.0,
                            takeovers=takeovers,
                            min_takeover_m=min_takeover_m,
                            min_seg_m=seg_min,
                            max_seg_m=seg_max,
                            heading_delta_max=config.HEADING_DELTA_MAX_DEG,
                        )
                    if not coords:
                        continue
                    direct_len = _distance_m(coords[0], coords[-1])
                    if direct_len < line_min_len or direct_len > line_max_len:
                        continue
                    if not _line_clear_of_lines(coords, existing_segments, min_gap_m):
                        continue
                    if not _line_clear_of_areas(coords, existing_areas, min_gap_m):
                        continue
                    if takeovers and not _line_clear_of_takeovers(coords, takeovers, min_takeover_m):
                        continue
                    width = _line_width(rng, aircraft_count)
                    altitude = rng.uniform(config.LINE_ALT_MIN_M, config.LINE_ALT_MAX_M)
                    missions.append(_line_mission(mission_id, coords, width, altitude, mission_type))
                    existing_segments.extend(_segments_from_points(coords))
                    mission_id += 1
                    cursor = coords[-1]
                    if len(coords) >= 2:
                        heading_ref = _bearing_deg(coords[-2], coords[-1])
                    success = True
                    break
                if hard_fail or not success:
                    ok = False
                    break
            else:
                success = False
                if package_type == 5 and not city_area_used and city_center is not None:
                    coords = _area_centered_on_point(
                        city_center,
                        bounds,
                        rng,
                        takeovers=takeovers,
                        min_gap_m=min_takeover_m,
                    )
                    if coords:
                        if not _area_clear_of_lines(coords, existing_segments, min_gap_m):
                            coords = None
                        elif not _area_clear_of_areas(coords, existing_areas, min_gap_m):
                            coords = None
                        elif not _area_clear_of_takeovers(coords, takeovers, min_takeover_m):
                            coords = None
                    if coords:
                        missions.append(_area_mission(mission_id, coords, mission_type))
                        existing_areas.append(coords)
                        mission_id += 1
                        cursor = _next_start_after_area(city_center, coords, heading_ref, bounds, rng, min_gap_m)
                        city_area_used = True
                        success = True
                    else:
                        ok = False
                        break
                if not success:
                    for _ in range(config.RECT_ATTEMPTS * 2):
                        anchor_pt = _random_point_near(
                            bounds,
                            cursor,
                            rng,
                            max_offset_m=config.EDGE_GAP_MAX_M,
                            min_offset_m=config.EDGE_GAP_MIN_M,
                        ) or cursor
                        coords = _area_centered_on_point(
                            anchor_pt,
                            bounds,
                            rng,
                            takeovers=takeovers,
                            min_gap_m=min_takeover_m,
                        )
                        if not coords:
                            continue
                        if not _area_clear_of_lines(coords, existing_segments, min_gap_m):
                            continue
                        if not _area_clear_of_areas(coords, existing_areas, min_gap_m):
                            continue
                        if not _area_clear_of_takeovers(coords, takeovers, min_takeover_m):
                            continue
                        missions.append(_area_mission(mission_id, coords, mission_type))
                        existing_areas.append(coords)
                        mission_id += 1
                        cursor = _next_start_after_area(anchor_pt, coords, heading_ref, bounds, rng, min_gap_m)
                        success = True
                        break
                if not success:
                    ok = False
                    break

        if ok and missions:
            if start:
                missions = _reorder_missions_nearest(missions, start)
            break

    if not missions:
        if not getattr(config, "_RELAXED_GEN", False):
            prev_gap = config.EDGE_GAP_MIN_M
            prev_start = config.START_OFFSET_MIN_M
            prev_seg = config.SEGMENT_ATTEMPTS
            prev_rect = config.RECT_ATTEMPTS
            prev_max = config.MAX_GEN_ATTEMPTS
            try:
                config._RELAXED_GEN = True
                config.EDGE_GAP_MIN_M = max(500.0, prev_gap * 0.7)
                config.START_OFFSET_MIN_M = max(500.0, prev_start * 0.7)
                config.SEGMENT_ATTEMPTS = max(40, int(prev_seg * 0.7))
                config.RECT_ATTEMPTS = max(40, int(prev_rect * 0.7))
                config.MAX_GEN_ATTEMPTS = max(60, int(prev_max * 0.7))
                return generate(
                    seed=seed,
                    anchor=anchor,
                    heading_hint=heading_hint,
                    max_start_offset_m=max_start_offset_m,
                    package_type=package_type,
                    takeovers=takeovers,
                )
            finally:
                config.EDGE_GAP_MIN_M = prev_gap
                config.START_OFFSET_MIN_M = prev_start
                config.SEGMENT_ATTEMPTS = prev_seg
                config.RECT_ATTEMPTS = prev_rect
                config.MAX_GEN_ATTEMPTS = prev_max
                try:
                    delattr(config, "_RELAXED_GEN")
                except Exception:
                    pass
        raise RuntimeError("Failed to generate missions within attempt budget")

    if package_type is None:
        package_type = rng.randint(1, 5)

    pkg_id, file_seq = _next_ids(package_type)

    payload = {

        "timestamp": now_ms_2000(),
        "inputMissionPackageID": pkg_id,
        "inputMissionPackageType": int(package_type),
        "mainSensor": _pick_main_sensor(rng),
        "availableAircraftList": [{"aircraftID": i} for i in config.AIRCRAFT_IDS],
        "inputMissionList": missions,
        "_meta": {"fileSeq": file_seq, "seed": seed},
    }
    return payload


def save(payload: Dict) -> Path:
    dir_path = _db_dir()
    dir_path.mkdir(parents=True, exist_ok=True)
    meta = payload.get("_meta", {})
    file_seq = meta.get("fileSeq") or payload.get("inputMissionPackageID", 1)
    path = dir_path / f"{int(file_seq):04d}.json"
    payload = dict(payload)
    payload.pop("_meta", None)
    with path.open("w", encoding="utf-8") as fh:
        json.dump(payload, fh, ensure_ascii=False, indent=2)
    return path





if __name__ == "__main__":
    obj = generate()
    out = save(obj)
    print(f"generated: {out}")
