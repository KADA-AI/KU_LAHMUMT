from __future__ import annotations

"""
One-shot scenario generation that ties together:
- FlightReferenceInfo (takeOver/handOver/prohibited area)
- InputMissionPlan (line -> area -> line), biased near takeOver points
- Target list with mission-aware placement rules

Usage:
    from fpl_random.pipeline import generate_sequence
    result = generate_sequence(seed=123)
    # result["paths"] holds saved file paths
"""

import math
import random
from functools import lru_cache
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple

if __name__ == "__main__" and __package__ is None:
    # Allow running as a script without installing the package.
    import sys

    sys.path.append(str(Path(__file__).resolve().parent.parent))
    __package__ = "fpl_random"

from .areas import AUTO_MISSION_AREA, LatLon, START_REFERENCE_POINTS
from . import config, dem, flight_ref, mission_plan, paths
from .utils import now_ms_2000, offset_lat_lon

PROJECT_ROOT = Path(__file__).resolve().parent.parent  # 모듈 루트(FPL_Random).

# 생성 파라미터는 config.py에서 관리한다.

def _target_dir() -> Path:
    return paths.db_root() / "TargetInfo"


def _bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle bearing from (lat1, lon1) to (lat2, lon2) in degrees."""
    lat1_r, lat2_r = math.radians(lat1), math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    x = math.sin(dlon) * math.cos(lat2_r)
    y = math.cos(lat1_r) * math.sin(lat2_r) - math.sin(lat1_r) * math.cos(lat2_r) * math.cos(dlon)
    ang = math.degrees(math.atan2(x, y))
    return (ang + 360.0) % 360.0


def _distance_m(p1: LatLon, p2: LatLon) -> float:
    """Equirectangular approximation; good enough for local spacing checks."""
    mean_lat = math.radians((p1.latitude + p2.latitude) / 2.0)
    dlat = (p2.latitude - p1.latitude) * 111_320.0
    dlon = (p2.longitude - p1.longitude) * 111_320.0 * math.cos(mean_lat)
    return math.hypot(dlat, dlon)


def _far_enough(existing: Sequence[LatLon], cand: LatLon, min_dist_m: float) -> bool:
    return all(_distance_m(pt, cand) >= min_dist_m for pt in existing)


@lru_cache(maxsize=1)
def _composite_start_reference_candidates() -> Tuple[LatLon, ...]:
    min_lat = min(pt.latitude for pt in START_REFERENCE_POINTS)
    south_band = min_lat + 0.005
    candidates = [pt for pt in START_REFERENCE_POINTS if pt.latitude <= south_band]
    if len(candidates) > 4:
        mid_lon = (AUTO_MISSION_AREA.southwest.longitude + AUTO_MISSION_AREA.northeast.longitude) / 2.0
        center_idx = min(range(len(candidates)), key=lambda idx: abs(candidates[idx].longitude - mid_lon))
        candidates.pop(center_idx)
    candidates.sort(key=lambda pt: (pt.longitude, pt.latitude))
    return tuple(candidates or (START_REFERENCE_POINTS[0],))


def _random_point_within_radius(base: LatLon, rng: random.Random, radius_m: float) -> LatLon:
    distance = rng.uniform(0.0, max(0.0, radius_m))
    bearing = rng.uniform(0.0, 360.0)
    east = distance * math.sin(math.radians(bearing))
    north = distance * math.cos(math.radians(bearing))
    lat, lon = offset_lat_lon(base.latitude, base.longitude, east, north)
    return LatLon(lat, lon)


def _target_altitude(point: LatLon) -> int:
    return dem.altitude_agl_m(point.latitude, point.longitude, config.TARGET_ALT_AGL_M)


def _terrain_targeting_module():
    try:
        from . import terrain_targeting as targeting_mod
    except Exception:
        return None
    return targeting_mod


def _resolve_target_random_dem_path() -> Optional[Path]:
    raw = getattr(config, "TARGET_RANDOM_DEM_FILE", None) or getattr(config, "DEM_CORRIDOR_FILE", None)
    if not raw:
        return None
    dem_path = Path(str(raw))
    if dem_path.is_absolute():
        return dem_path.resolve()
    project_root = Path(getattr(config, "_PROJECT_ROOT", Path(__file__).resolve().parents[4]))
    return (project_root / dem_path).resolve()


def _mission_center(coords: Sequence[LatLon]) -> Optional[LatLon]:
    if not coords:
        return None
    lat = sum(float(pt.latitude) for pt in coords) / len(coords)
    lon = sum(float(pt.longitude) for pt in coords) / len(coords)
    return LatLon(lat, lon)


def _collect_static_target_bounds(
    static_missions: Sequence[Tuple[str, int, Sequence[LatLon]]],
) -> Optional[Tuple[float, float, float, float]]:
    points: List[LatLon] = []
    for _kind, _mission_id, coords in static_missions:
        points.extend(list(coords))
    if not points:
        return None
    lats = [float(pt.latitude) for pt in points]
    lons = [float(pt.longitude) for pt in points]
    return min(lats), max(lats), min(lons), max(lons)


def _auto_mission_area_bounds() -> Tuple[float, float, float, float]:
    return (
        float(AUTO_MISSION_AREA.southwest.latitude),
        float(AUTO_MISSION_AREA.northeast.latitude),
        float(AUTO_MISSION_AREA.southwest.longitude),
        float(AUTO_MISSION_AREA.northeast.longitude),
    )


def _point_in_bounds(point: LatLon, bounds: Tuple[float, float, float, float]) -> bool:
    lat_min, lat_max, lon_min, lon_max = bounds
    return lat_min <= float(point.latitude) <= lat_max and lon_min <= float(point.longitude) <= lon_max


def _targeting_bounds_key(
    bounds: Tuple[float, float, float, float],
    margin_m: float,
) -> Tuple[float, float, float, float, float]:
    return (
        round(float(bounds[0]), 6),
        round(float(bounds[1]), 6),
        round(float(bounds[2]), 6),
        round(float(bounds[3]), 6),
        round(float(margin_m), 1),
    )


@lru_cache(maxsize=4)
def _load_targeting_bundle_cached(
    dem_path_text: str,
    bounds_key: Tuple[float, float, float, float, float],
):
    targeting_mod = _terrain_targeting_module()
    if targeting_mod is None:
        raise RuntimeError("terrain_targeting module unavailable")
    lat_min, lat_max, lon_min, lon_max, margin_m = bounds_key
    return targeting_mod.load_terrain_bundle(
        Path(dem_path_text),
        targeting_mod.default_mask_config(),
        clip_bounds_wgs84=(lat_min, lat_max, lon_min, lon_max),
        clip_margin_m=margin_m,
    )


def _lonlat_to_dem_local(point: LatLon, bundle) -> Optional[Tuple[float, float]]:
    targeting_mod = _terrain_targeting_module()
    if targeting_mod is None:
        return None
    return targeting_mod.lonlat_to_local(float(point.latitude), float(point.longitude), bundle)


def _dem_world_to_latlon(x_world: float, y_world: float, bundle) -> Optional[LatLon]:
    targeting_mod = _terrain_targeting_module()
    if targeting_mod is None:
        return None
    lat_lon = targeting_mod.world_to_lonlat(float(x_world), float(y_world), bundle)
    if lat_lon is None:
        return None
    return LatLon(float(lat_lon[0]), float(lat_lon[1]))


def _try_make_reference_point(
    point: LatLon,
    bundle,
    terrain,
) -> Optional[object]:
    targeting_mod = _terrain_targeting_module()
    if targeting_mod is None:
        return None
    local_point = _lonlat_to_dem_local(point, bundle)
    if local_point is None:
        return None
    try:
        return targeting_mod.make_reference_point(terrain, float(local_point[0]), float(local_point[1]))
    except Exception:
        return None


def _build_terrain_candidate_map(
    static_missions: Sequence[Tuple[str, int, Sequence[LatLon]]],
    rng: random.Random,
    avoid_points: Sequence[LatLon],
    min_avoid_dist_m: float,
) -> Dict[int, List[LatLon]]:
    if not static_missions:
        return {}

    targeting_mod = _terrain_targeting_module()
    dem_path = _resolve_target_random_dem_path()
    mission_bounds = _collect_static_target_bounds(static_missions)
    analysis_bounds = _auto_mission_area_bounds()
    if targeting_mod is None or dem_path is None or not dem_path.exists() or mission_bounds is None:
        return {}

    margin_m = 0.0
    jitter_attempts = max(1, int(getattr(config, "TARGET_RANDOM_REFERENCE_ATTEMPTS", 12) or 12))
    sam_limit = max(1, int(getattr(config, "TARGET_RANDOM_SAM_CANDIDATE_LIMIT", 50) or 50))
    radar_limit = max(0, int(getattr(config, "TARGET_RANDOM_RADAR_CANDIDATE_LIMIT", 20) or 20))
    jitter_radius_m = max(300.0, min(margin_m, 1200.0))

    try:
        bundle = _load_targeting_bundle_cached(str(dem_path), _targeting_bounds_key(analysis_bounds, margin_m))
    except Exception:
        return {}

    terrain = bundle.terrain
    radar_config = targeting_mod.default_radar_config()
    sam_config = targeting_mod.default_sam_config()
    candidate_map: Dict[int, Dict[Tuple[int, int], Tuple[LatLon, float]]] = {}

    for kind, mission_id, coords in static_missions:
        if not coords:
            continue
        reference = None
        refs: List[LatLon] = []
        center = _mission_center(coords)
        if center is not None:
            refs.append(center)
        for _ in range(max(0, jitter_attempts - len(refs))):
            if kind == "area":
                pt = _pick_point_in_area(coords, rng)
            else:
                base = center or coords[0]
                pt = _random_point_within_radius(base, rng, jitter_radius_m)
            if pt is not None:
                refs.append(pt)
        for ref_point in refs:
            if not _point_in_bounds(ref_point, analysis_bounds):
                continue
            reference = _try_make_reference_point(ref_point, bundle, terrain)
            if reference is not None:
                break
        if reference is None:
            continue

        try:
            radar_result = targeting_mod.search_radar_candidates(terrain, reference, radar_config)
            sam_candidates = targeting_mod.search_sam_candidates(
                terrain,
                reference,
                radar_result.selected_candidates,
                sam_config,
            )
        except Exception:
            continue

        ranked = list(sam_candidates)[:sam_limit] + list(radar_result.selected_candidates)[:radar_limit]
        if not ranked:
            continue

        per_mission = candidate_map.setdefault(mission_id, {})
        for candidate in ranked:
            point = _dem_world_to_latlon(float(candidate.x), float(candidate.y), bundle)
            if point is None:
                continue
            if not _point_in_bounds(point, analysis_bounds):
                continue
            if not _terrain_candidate_matches_mission(kind, point, coords):
                continue
            if avoid_points and not _far_enough(avoid_points, point, min_avoid_dist_m):
                continue
            key = (
                int(round(float(point.latitude) * 1_000_000)),
                int(round(float(point.longitude) * 1_000_000)),
            )
            score = float(getattr(candidate, "total_score", 0.0))
            prev = per_mission.get(key)
            if prev is None or score > prev[1]:
                per_mission[key] = (point, score)

    out: Dict[int, List[LatLon]] = {}
    for mission_id, scored in candidate_map.items():
        ranked_points = sorted(scored.values(), key=lambda item: item[1], reverse=True)
        out[mission_id] = [point for point, _score in ranked_points]
    return out


def _take_terrain_site(
    terrain_sites: Optional[List[LatLon]],
    existing_pts: Sequence[LatLon],
    avoid_points: Sequence[LatLon],
    min_sep_m: float,
    min_avoid_m: float,
) -> Optional[LatLon]:
    if not terrain_sites:
        return None
    for idx, point in enumerate(list(terrain_sites)):
        if existing_pts and not _far_enough(existing_pts, point, min_sep_m):
            continue
        if avoid_points and not _far_enough(avoid_points, point, min_avoid_m):
            continue
        terrain_sites.pop(idx)
        return point
    return None


def _build_target_path(base: LatLon, target_type: int, rng: random.Random) -> List[Dict[str, float]]:
    if target_type not in config.MOVING_TARGET_TYPES:
        return [
            {
                "latitude": base.latitude,
                "longitude": base.longitude,
                "altitude": _target_altitude(base),
            }
        ]
    min_n, max_n = config.TARGET_PATH_POINTS_RANGE
    count = max(1, rng.randint(min_n, max_n))
    points = [base]
    for _ in range(count - 1):
        points.append(_random_point_within_radius(base, rng, config.TARGET_PATH_RADIUS_M))
    return [{"latitude": p.latitude, "longitude": p.longitude, "altitude": _target_altitude(p)} for p in points]


def _anchor_from_flight_reference(ref_payload: Dict) -> Tuple[Optional[LatLon], Optional[float]]:
    """
    Pick a takeOver point (anchor) and derive a heading hint from its handOver pair.
    Prefers the first UAV entry. Falls back to None on missing data.
    """
    takeovers = ref_payload.get("takeOverInfoList") or []
    if not takeovers:
        return None, None

    handover_map = {
        entry.get("aircraftID"): entry.get("coordinate")
        for entry in ref_payload.get("handOverInfoList") or []
        if isinstance(entry, dict)
    }

    anchor_entry = takeovers[0]
    coord = anchor_entry.get("coordinate") or {}
    anchor = LatLon(coord.get("latitude", 0.0), coord.get("longitude", 0.0))

    heading = None
    handover_coord = handover_map.get(anchor_entry.get("aircraftID"))
    if handover_coord and all(k in handover_coord for k in ("latitude", "longitude")):
        heading = _bearing_deg(
            coord.get("latitude", 0.0),
            coord.get("longitude", 0.0),
            handover_coord["latitude"],
            handover_coord["longitude"],
        )
    return anchor, heading


def _collect_line_coords(mission: Dict) -> List[LatLon]:
    detail = mission.get("missionDetail") or {}
    lines = detail.get("lineList") or []
    if not lines:
        return []
    coords = lines[0].get("coordinateList") or []
    out: List[LatLon] = []
    for c in coords:
        try:
            out.append(LatLon(float(c["latitude"]), float(c["longitude"])))
        except Exception:
            continue
    return out


def _collect_area_coords(mission: Dict) -> List[LatLon]:
    detail = mission.get("missionDetail") or {}
    areas = detail.get("areaList") or []
    if not areas:
        return []
    coords = areas[0].get("coordinateList") or []
    out: List[LatLon] = []
    for c in coords:
        try:
            out.append(LatLon(float(c["latitude"]), float(c["longitude"])))
        except Exception:
            continue
    return out


def _point_in_poly(pt: LatLon, poly: Sequence[LatLon]) -> bool:
    """Ray casting; works for simple polygons (including rectangles)."""
    x, y = pt.longitude, pt.latitude
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i - 1].longitude, poly[i - 1].latitude
        x2, y2 = poly[i].longitude, poly[i].latitude
        if ((y1 > y) != (y2 > y)) and (x < (x2 - x1) * (y - y1) / (y2 - y1 + 1e-12) + x1):
            inside = not inside
    return inside


def _terrain_candidate_matches_mission(kind: str, point: LatLon, coords: Sequence[LatLon]) -> bool:
    if kind == "area":
        return _point_in_poly(point, coords)
    return True


def _pick_point_on_line(coords: Sequence[LatLon], rng: random.Random) -> Optional[LatLon]:
    if len(coords) < 2:
        return None
    idx = rng.randint(0, len(coords) - 2)
    a, b = coords[idx], coords[idx + 1]
    t = rng.uniform(0.2, 0.8)
    lat = a.latitude + (b.latitude - a.latitude) * t
    lon = a.longitude + (b.longitude - a.longitude) * t
    # 측면으로 약간 이동시켜 겹치지 않게 (좌/우 랜덤)
    bearing = _bearing_deg(a.latitude, a.longitude, b.latitude, b.longitude)
    lateral = rng.uniform(-config.LINE_LATERAL_OFFSET_M, config.LINE_LATERAL_OFFSET_M)
    if abs(lateral) > 1e-6:
        side_hdg = (bearing + (90.0 if lateral >= 0 else -90.0)) % 360.0
        lat, lon = offset_lat_lon(lat, lon, lateral, 0.0)
    return LatLon(lat, lon)


def _pick_point_in_area(coords: Sequence[LatLon], rng: random.Random) -> Optional[LatLon]:
    if not coords:
        return None
    lats = [c.latitude for c in coords]
    lons = [c.longitude for c in coords]
    lat_min, lat_max = min(lats), max(lats)
    lon_min, lon_max = min(lons), max(lons)
    for _ in range(80):
        lat = rng.uniform(lat_min, lat_max)
        lon = rng.uniform(lon_min, lon_max)
        candidate = LatLon(lat, lon)
        if _point_in_poly(candidate, coords):
            return candidate
    # Fallback to centroid if random search fails (should not happen for rectangles)
    return LatLon(sum(lats) / len(lats), sum(lons) / len(lons))

def _pick_point_from_coords(coords: Sequence[LatLon], rng: random.Random) -> Optional[LatLon]:
    if not coords:
        return None
    return rng.choice(list(coords))


def _collect_takeovers(ref_payload: Dict) -> List[LatLon]:
    out: List[LatLon] = []
    for entry in ref_payload.get("takeOverInfoList") or []:
        coord = entry.get("coordinate") if isinstance(entry, dict) else None
        if not coord:
            continue
        try:
            out.append(LatLon(float(coord["latitude"]), float(coord["longitude"])))
        except Exception:
            continue
    return out


def _collect_point_coords(mission: Dict) -> List[LatLon]:
    detail = mission.get("missionDetail") or {}
    coords = detail.get("coordinateList") or []
    out: List[LatLon] = []
    for c in coords:
        try:
            out.append(LatLon(float(c["latitude"]), float(c["longitude"])))
        except Exception:
            continue
    return out

def _pick_spawn_point(
    coords: Sequence[LatLon],
    rng: random.Random,
    pick_func,
    existing_pts: Sequence[LatLon],
    avoid_points: Sequence[LatLon],
    min_sep_m: float,
    min_avoid_m: float,
) -> Optional[LatLon]:
    for _ in range(config.TARGET_PLACEMENT_ATTEMPTS):
        pt = pick_func(coords, rng)
        if not pt:
            continue
        if existing_pts and not _far_enough(existing_pts, pt, min_sep_m):
            continue
        if avoid_points and not _far_enough(avoid_points, pt, min_avoid_m):
            continue
        return pt
    return None


def _append_targets(
    targets: List[Dict],
    existing_pts: List[LatLon],
    mission_id: int,
    coords: Sequence[LatLon],
    pick_func,
    target_types: Sequence[int],
    rng: random.Random,
    avoid_points: Sequence[LatLon],
    min_sep_m: float,
    min_avoid_m: float,
    terrain_sites: Optional[List[LatLon]] = None,
) -> None:
    for target_type in target_types:
        pt = _take_terrain_site(terrain_sites, existing_pts, avoid_points, min_sep_m, min_avoid_m)
        if pt is None:
            pt = _pick_spawn_point(
                coords,
                rng,
                pick_func,
                existing_pts,
                avoid_points,
                min_sep_m,
                min_avoid_m,
            )
        if not pt:
            continue
        altitude = _target_altitude(pt)
        tgt = {
            "targetID": len(targets) + 1,
            "targetType": int(target_type),
            "inputMissionID": mission_id,
            "location": {"latitude": pt.latitude, "longitude": pt.longitude, "altitude": altitude},
            "path": _build_target_path(pt, int(target_type), rng),
        }
        targets.append(tgt)
        existing_pts.append(pt)

def _target_total_from_missions(mission_count: int, rng: random.Random) -> int:
    min_ratio, max_ratio = config.TARGET_COUNT_RATIO_RANGE
    min_total = int(math.floor(mission_count * min_ratio))
    max_total = int(math.ceil(mission_count * max_ratio))
    min_total = max(1, min_total)
    max_total = max(min_total, max_total)
    return rng.randint(min_total, max_total)


def _build_targets(
    cmpk_payload: Dict,
    rng: random.Random,
    allowed_types: Sequence[int],
    avoid_points: Sequence[LatLon] | None = None,
    min_avoid_dist_m: float = config.TAKEOVER_CLEARANCE_M,
    use_terrain_targeting: bool = True,
) -> List[Dict]:
    targets: List[Dict] = []
    existing_pts: List[LatLon] = []
    type_pool = [int(t) for t in allowed_types] or [1, 2]
    avoid_points = list(avoid_points) if avoid_points else []

    missions = cmpk_payload.get("inputMissionList") or []
    package_type = int(cmpk_payload.get("inputMissionPackageType", 0) or 0)

    line_missions: List[Tuple[int, List[LatLon]]] = []
    area_missions: List[Tuple[int, List[LatLon]]] = []
    point_missions: List[Tuple[int, List[LatLon]]] = []
    for mission in missions:
        try:
            mission_id = int(mission.get("inputMissionID", 0) or 0)
        except Exception:
            mission_id = 0
        line_coords = _collect_line_coords(mission)
        if line_coords:
            line_missions.append((mission_id, line_coords))
            continue
        area_coords = _collect_area_coords(mission)
        if area_coords:
            area_missions.append((mission_id, area_coords))
            continue
        point_coords = _collect_point_coords(mission)
        if point_coords:
            point_missions.append((mission_id, point_coords))

    static_missions: List[Tuple[str, int, Sequence[LatLon]]] = []
    static_missions += [("area", mid, coords) for mid, coords in area_missions]
    terrain_sites_by_mission: Dict[int, List[LatLon]] = {}
    if use_terrain_targeting:
        terrain_sites_by_mission = _build_terrain_candidate_map(
            static_missions,
            rng,
            avoid_points,
            min_avoid_dist_m,
        )

    if package_type == 1 and area_missions:
        # 대기갑 항공타격 작전: 라인 미션에는 타깃을 두지 않고 목표지역(면)에만 배치한다.
        mission_id, coords = area_missions[0]
        tank_count = rng.randint(config.ANTI_ARMOR_AREA_TANK_RANGE[0], config.ANTI_ARMOR_AREA_TANK_RANGE[1])
        mlrs_count = rng.randint(config.ANTI_ARMOR_AREA_MLRS_RANGE[0], config.ANTI_ARMOR_AREA_MLRS_RANGE[1])
        aaa_count = rng.randint(config.ANTI_ARMOR_AREA_AAA_RANGE[0], config.ANTI_ARMOR_AREA_AAA_RANGE[1])
        target_types = [1] * tank_count + [3] * mlrs_count + [5] * aaa_count
        rng.shuffle(target_types)
        _append_targets(
            targets,
            existing_pts,
            mission_id,
            coords,
            _pick_point_in_area,
            target_types,
            rng,
            avoid_points,
            config.TARGET_MIN_SEP_M,
            min_avoid_dist_m,
            terrain_sites=terrain_sites_by_mission.get(mission_id),
        )
        if targets:
            return targets

    mission_count = len(area_missions)
    if mission_count <= 0:
        return targets

    total_targets = _target_total_from_missions(mission_count, rng)
    mission_pool: List[Tuple[str, int, List[LatLon]]] = []
    mission_pool += [("area", mid, coords) for mid, coords in area_missions]

    if not mission_pool:
        return targets

    for _ in range(total_targets):
        placed = False
        for _ in range(config.TARGET_PLACEMENT_ATTEMPTS):
            kind, mission_id, coords = rng.choice(mission_pool)
            terrain_sites = terrain_sites_by_mission.get(mission_id)
            target_type = rng.choice(type_pool)
            pick_func = _pick_point_in_area

            pt = _take_terrain_site(
                terrain_sites,
                existing_pts,
                avoid_points,
                config.TARGET_MIN_SEP_M,
                min_avoid_dist_m,
            )
            if pt is None:
                pt = _pick_spawn_point(
                    coords,
                    rng,
                    pick_func,
                    existing_pts,
                    avoid_points,
                    config.TARGET_MIN_SEP_M,
                    min_avoid_dist_m,
                )
            if not pt:
                continue
            altitude = _target_altitude(pt)
            tgt = {
                "targetID": len(targets) + 1,
                "targetType": int(target_type),
                "inputMissionID": mission_id,
                "location": {"latitude": pt.latitude, "longitude": pt.longitude, "altitude": altitude},
                "path": _build_target_path(pt, int(target_type), rng),
            }
            targets.append(tgt)
            existing_pts.append(pt)
            placed = True
            break
        if not placed:
            break

    eligible_mission_ids = {mid for mid, _coords in area_missions}
    if not eligible_mission_ids:
        return []
    filtered = [
        tgt
        for tgt in targets
        if int(tgt.get("inputMissionID", 0) or 0) in eligible_mission_ids
    ]
    for idx, tgt in enumerate(filtered, start=1):
        tgt["targetID"] = idx
    return filtered


def save_targets(payload: Dict, file_seq: Optional[int] = None) -> Path:
    """
    Persist target payload under database/TargetInfo.
    """
    target_dir = _target_dir()
    target_dir.mkdir(parents=True, exist_ok=True)
    meta = payload.get("_meta", {})
    seq = file_seq or meta.get("fileSeq") or payload.get("inputMissionPackageID", 1)
    path = target_dir / f"{int(seq):04d}.json"
    payload = dict(payload)
    payload.pop("_meta", None)
    with path.open("w", encoding="utf-8") as fh:
        import json

        json.dump(payload, fh, ensure_ascii=False, indent=2)
    return path


def generate_sequence(
    seed: Optional[int] = None,
    *,
    max_start_offset_m: float = 2500.0,
    target_types: Sequence[int] = (1, 2),
    composite_route: bool = False,
    terrain_targeting: bool = True,
    save: bool = True,
    status_cb: Optional[Callable[[str], None]] = None,
) -> Dict:
    """
    Run flight reference + mission plan + target generation in one call.

    Args:
        seed: Master seed for reproducibility.
        max_start_offset_m: Clamp how far the first line start can drift from the chosen takeOver anchor.
        target_types: Allowed targetType pool (defaults to {1, 2}).
        terrain_targeting: When True, try SAM/RADAR terrain-aware target placement before random fallback.
        save: When True, persist all payloads to disk.
    """
    master_rng = random.Random(seed)
    ref_seed = master_rng.randint(0, 1_000_000_000)
    cmpk_seed = master_rng.randint(0, 1_000_000_000)
    target_seed = master_rng.randint(0, 1_000_000_000)
    target_rng = random.Random(target_seed)

    if status_cb:
        status_cb("0203 FlightReference 생성 중...")
    if composite_route:
        base_candidates = _composite_start_reference_candidates()
        base_rng = random.Random(ref_seed)
        base_point = base_rng.choice(base_candidates)
        mid_lon = (AUTO_MISSION_AREA.southwest.longitude + AUTO_MISSION_AREA.northeast.longitude) / 2.0
        ref_payload = flight_ref.generate(
            seed=ref_seed,
            base_point=base_point,
            handover_left=base_point.longitude <= mid_lon,
        )
    else:
        ref_payload = flight_ref.generate(seed=ref_seed)
    anchor, heading = _anchor_from_flight_reference(ref_payload)
    takeovers = _collect_takeovers(ref_payload)
    if status_cb:
        status_cb("0201 InputMissionPlan 생성 중...")
    if composite_route:
        cmpk_payload = mission_plan.generate_composite_route(
            seed=cmpk_seed,
            anchor=anchor,
            takeovers=takeovers,
            max_start_offset_m=max_start_offset_m,
        )
    else:
        retry_max = max(0, int(getattr(config, "GENERATION_RETRY_MAX", 0)))
        retry_count = 0
        while True:
            try:
                cmpk_payload = mission_plan.generate(
                    seed=cmpk_seed,
                    anchor=anchor,
                    heading_hint=heading,
                    max_start_offset_m=max_start_offset_m,
                    takeovers=takeovers,
                )
                break
            except RuntimeError as exc:
                if "Failed to generate missions" not in str(exc) or retry_count >= retry_max:
                    raise
                retry_count += 1
                cmpk_seed = master_rng.randint(0, 1_000_000_000)
                if status_cb:
                    status_cb(f"0201 InputMissionPlan 재시도 {retry_count}/{retry_max}...")

    package_id = cmpk_payload.get("inputMissionPackageID")
    file_seq = cmpk_payload.get("_meta", {}).get("fileSeq") or package_id
    if package_id:
        ref_payload["missionReferencePackageID"] = package_id
    if file_seq:
        ref_meta = ref_payload.get("_meta", {})
        ref_meta["fileSeq"] = file_seq
        ref_payload["_meta"] = ref_meta

    avoid = _collect_takeovers(ref_payload)
    if status_cb:
        status_cb("Target 생성 중...")
    targets = _build_targets(
        cmpk_payload,
        target_rng,
        allowed_types=target_types,
        avoid_points=avoid,
        min_avoid_dist_m=config.TAKEOVER_CLEARANCE_M,
        use_terrain_targeting=terrain_targeting,
    )
    target_payload = {
        "timestamp": now_ms_2000(),
        "inputMissionPackageID": cmpk_payload.get("inputMissionPackageID"),
        "missionReferencePackageID": ref_payload.get("missionReferencePackageID"),
        "targetList": targets,
        "_meta": {
            "fileSeq": cmpk_payload.get("_meta", {}).get("fileSeq") or cmpk_payload.get("inputMissionPackageID"),
            "seed": seed,
            "seeds": {"flight_reference": ref_seed, "mission_plan": cmpk_seed, "targets": target_seed},
            "anchor": anchor.as_tuple() if anchor else None,
            "heading_hint": heading,
            "composite_route": bool(composite_route),
            "terrain_targeting": bool(terrain_targeting),
        },
    }

    paths = {"flight_reference": None, "input_mission_plan": None, "targets": None}
    if save:
        paths["flight_reference"] = str(flight_ref.save(ref_payload))
        paths["input_mission_plan"] = str(mission_plan.save(cmpk_payload))
        paths["targets"] = str(save_targets(target_payload))

    return {
        "flight_reference": ref_payload,
        "input_mission_plan": cmpk_payload,
        "targets": target_payload,
        "paths": paths,
    }


if __name__ == "__main__":
    result = generate_sequence()
    print("flight_reference:", result["paths"]["flight_reference"])
    print("input_mission_plan:", result["paths"]["input_mission_plan"])
    print("targets:", result["paths"]["targets"])
