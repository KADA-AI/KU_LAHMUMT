from __future__ import annotations

"""Production glue for the Type 4 donut patrol planner.

Bridges the ported annulus planner (:mod:`.logic`) into the initial-plan
pipeline:

- ``build_donut_band_pieces``  - split stage: one band piece per UAV (각자도생
  ownership: each band is a single UAV's own monitoring area; enemies found in
  it stay that UAV's responsibility and post-attack resume returns to it).
- ``build_donut_wplist``       - 0303 stage: ring-lane route waypoints + radial
  lineSearch rows for one aircraft, adapted to production waypoint rows.

The 0302 piece carries a ``_donutPatrol`` marker (full donut geometry + band
order) so d0303 can rebuild the plans deterministically without re-reading the
0201.
"""

import json
import math
from collections import OrderedDict
from typing import Any, Callable, Dict, List, Optional, Tuple

from .logic import (
    DonutMission,
    PatrolConfig,
    _band_fractions,
    _cast_ray,
    _critical_angles,
    build_donut_mission_from_0201,
    build_patrol_plans,
)

PACKAGE_TYPE_FACILITY_PROTECTION = 4
DEFAULT_STRATEGY = "band"

# Filming operation modes (mirror d0303 / SIM contract):
#   OPMODE_POINT - coordinate-fixed orientation; the sensor stares at a fixed
#     ground target while the aircraft flies by (the Area first-WP convention).
#   OPMODE_LINE  - line/area sweep; the sensor films the coordinateList *while
#     traveling to* the waypoint that carries it (arrival packing).
OPMODE_POINT = 1
OPMODE_LINE = 2

_PLAN_CACHE: Dict[str, Any] = {}
_PLAN_CACHE_MAX = 8

try:
    from modules.mission_planning.MissionPlanner.data_def.mission_helpers import terrain_elev as _terrain_elev
except Exception:  # pragma: no cover - defensive optional import
    _terrain_elev = None  # type: ignore

try:
    from modules.mission_planning.MissionPlanner.data_def.mission_helpers import (
        terrain_elev_many as _terrain_elev_many,
    )
except Exception:  # pragma: no cover - defensive optional import
    _terrain_elev_many = None  # type: ignore

_DEM_GROUND_CACHE: Dict[Tuple[float, float], int] = {}
_DEM_GROUND_CACHE_MAX = 400_000


def _dem_ground_alt(lat: float, lon: float) -> Optional[int]:
    """DEM ground elevation at a camera-target coordinate (cached, rounded key).

    lineSearch coordinates are ground points the sensor films, so their altitude
    must be the terrain elevation there - NOT the aircraft's flight altitude.
    """
    if _terrain_elev is None:
        return None
    key = (round(float(lat), 6), round(float(lon), 6))
    cached = _DEM_GROUND_CACHE.get(key)
    if cached is not None:
        return int(cached)
    try:
        value = int(round(float(_terrain_elev(float(lat), float(lon)))))
    except Exception:
        return None
    if len(_DEM_GROUND_CACHE) >= _DEM_GROUND_CACHE_MAX:
        _DEM_GROUND_CACHE.clear()
    _DEM_GROUND_CACHE[key] = int(value)
    return int(value)


def _resolve_ground_alt_batch(
    points: List[Tuple[float, float]],
    many_fn: Optional[Callable[[List[Tuple[float, float]]], List[Any]]] = None,
) -> Dict[Tuple[float, float], int]:
    """Resolve DEM ground elevations for many points in one batch pass.

    Per-point DEM lookups pay tile-resolve/cache-signature overhead on every
    call (~2 ms each), and a donut lineSearch carries hundreds of camera
    targets per aircraft - batching them is ~100x faster. Returns a map of
    rounded ``(lat, lon)`` keys (same rounding as ``_dem_ground_alt``) to
    elevations; points it cannot resolve are simply absent, so callers fall
    back to the per-point path with unchanged semantics.

    ``many_fn`` is the caller's batch resolver (d0303 passes its cached
    ``_dem_alt_many``); without one, the shared ``terrain_elev_many`` is used
    and successful values also warm ``_DEM_GROUND_CACHE``.
    """
    keys = list(dict.fromkeys((round(float(la), 6), round(float(lo), 6)) for la, lo in points))
    if not keys:
        return {}
    out: Dict[Tuple[float, float], int] = {}
    pending = keys
    if many_fn is not None:
        try:
            values = many_fn([(la, lo) for la, lo in pending])
        except Exception:
            values = None
        if values is not None and len(values) == len(pending):
            for key, val in zip(pending, values):
                try:
                    out[key] = int(round(float(val)))
                except Exception:
                    continue
            pending = [k for k in pending if k not in out]
    if pending:
        still: List[Tuple[float, float]] = []
        for key in pending:
            cached = _DEM_GROUND_CACHE.get(key)
            if cached is not None:
                out[key] = int(cached)
            else:
                still.append(key)
        pending = still
    if pending and _terrain_elev_many is not None:
        try:
            values = _terrain_elev_many([(la, lo) for la, lo in pending])
        except Exception:
            values = None
        if values is not None and len(values) == len(pending):
            for key, val in zip(pending, values):
                try:
                    ival = int(round(float(val)))
                except Exception:
                    continue
                out[key] = ival
                if len(_DEM_GROUND_CACHE) >= _DEM_GROUND_CACHE_MAX:
                    _DEM_GROUND_CACHE.clear()
                _DEM_GROUND_CACHE[key] = ival
    return out


def _to_int(value: Any) -> Optional[int]:
    try:
        return int(value)
    except Exception:
        return None


def is_donut_boundary_mission(mission: Any) -> bool:
    """True for a 협업경계(mtype=3) mission whose areaList has outer + isHole."""
    if not isinstance(mission, dict):
        return False
    if (_to_int(mission.get("inputMissionType")) or 0) != 3:
        return False
    detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else {}
    areas = detail.get("areaList") if isinstance(detail.get("areaList"), list) else []
    has_outer = False
    has_hole = False
    for area in areas:
        if not isinstance(area, dict):
            continue
        coords = area.get("coordinateList")
        if not isinstance(coords, list) or len(coords) < 3:
            continue
        if bool(area.get("isHole")):
            has_hole = True
        else:
            has_outer = True
    return has_outer and has_hole


def donut_mission_from_input(mission: Dict[str, Any]) -> DonutMission:
    payload = {"inputMissionList": [mission]}
    return build_donut_mission_from_0201(payload, input_mission_id=_to_int(mission.get("inputMissionID")))


def _ray_length_m(donut: DonutMission) -> float:
    min_x, min_y, max_x, max_y = donut.polygon_xy.bounds
    return max(1000.0, math.hypot(float(max_x) - float(min_x), float(max_y) - float(min_y)) * 2.0)


def _hole_center_xy(donut: DonutMission) -> Tuple[float, float]:
    if donut.hole_polygons_xy:
        center = donut.hole_polygons_xy[0].centroid
    else:
        center = donut.polygon_xy.centroid
    return (float(center.x), float(center.y))


def band_rings_latlon(
    donut: DonutMission,
    inner_frac: float,
    outer_frac: float,
    *,
    angle_step_deg: float = 6.0,
    altitude: int = 0,
) -> Tuple[List[Dict[str, Any]], List[Dict[str, Any]]]:
    """Ray-sampled ``(outer_edge, inner_edge)`` rings of an annular band (lat/lon).

    An annular band is a ring, not a simple polygon; representing it as a single
    ``outer + inner[::-1]`` loop is self-intersecting (a bowtie) and renders as a
    half-filled shape under the even-odd/nonzero fill rules used by the SIM and
    monitoring. Returning the two edges lets the caller emit a proper
    outer(isHole=False) + inner(isHole=True) areaList instead.
    """
    center = _hole_center_xy(donut)
    ray_len = _ray_length_m(donut)
    step = math.radians(max(1.0, float(angle_step_deg)))
    angles = set(_critical_angles(donut, center))
    theta = 0.0
    two_pi = 2.0 * math.pi
    while theta < two_pi:
        angles.add(theta % two_pi)
        theta += step
    inner_ring: List[Tuple[float, float]] = []
    outer_ring: List[Tuple[float, float]] = []
    for theta in sorted(angles):
        ray = _cast_ray(donut, center, theta, ray_len)
        if ray is None:
            continue
        ix, iy = ray.inner_xy
        ox, oy = ray.outer_xy
        inner_ring.append((ix + (ox - ix) * float(inner_frac), iy + (oy - iy) * float(inner_frac)))
        outer_ring.append((ox - (ox - ix) * (1.0 - float(outer_frac)), oy - (oy - iy) * (1.0 - float(outer_frac))))
    if len(outer_ring) < 3:
        return [], []
    outer_latlon = [donut.frame.to_latlon(x, y, altitude=int(altitude)) for (x, y) in outer_ring]
    inner_latlon = [donut.frame.to_latlon(x, y, altitude=int(altitude)) for (x, y) in inner_ring]
    return outer_latlon, inner_latlon


def _ownership_by_takeover(
    donut: DonutMission,
    fractions: List[Tuple[float, float]],
    uav_ids: List[int],
    takeover_map: Optional[Dict[int, Dict[str, float]]],
) -> List[int]:
    """Order aircraft so band k goes to the geometrically nearest free UAV.

    Bands are ordered inner->outer; the outermost band's representative point is
    farthest from the hole, so match by distance from each UAV's takeover
    position to the band's mid-fraction ring point nearest that UAV.
    """
    ids = [int(a) for a in uav_ids if _to_int(a) is not None]
    if not ids:
        return []
    if not takeover_map:
        return ids[: len(fractions)] + ids[: max(0, len(fractions) - len(ids))]

    center = _hole_center_xy(donut)
    ray_len = _ray_length_m(donut)

    def _band_rep_points(frac_mid: float) -> List[Tuple[float, float]]:
        pts: List[Tuple[float, float]] = []
        for k in range(8):
            ray = _cast_ray(donut, center, k * math.pi / 4.0, ray_len)
            if ray is None:
                continue
            ix, iy = ray.inner_xy
            ox, oy = ray.outer_xy
            pts.append((ix + (ox - ix) * frac_mid, iy + (oy - iy) * frac_mid))
        return pts

    def _uav_xy(aid: int) -> Optional[Tuple[float, float]]:
        row = (takeover_map or {}).get(int(aid))
        if not isinstance(row, dict):
            return None
        try:
            return donut.frame.to_xy(row)
        except Exception:
            return None

    order: List[int] = []
    remaining = list(ids)
    for f0, f1 in fractions:
        reps = _band_rep_points((float(f0) + float(f1)) * 0.5)
        best_aid = None
        best_d = None
        for aid in remaining or ids:
            pos = _uav_xy(aid)
            if pos is None or not reps:
                d = float("inf")
            else:
                d = min(math.hypot(pos[0] - rx, pos[1] - ry) for rx, ry in reps)
            if best_d is None or d < best_d:
                best_d = d
                best_aid = aid
        if best_aid is None:
            best_aid = (remaining or ids)[0]
        order.append(int(best_aid))
        if best_aid in remaining:
            remaining.remove(best_aid)
    return order


def build_donut_band_pieces(
    mission: Dict[str, Any],
    uav_ids: List[int],
    takeover_map: Optional[Dict[int, Dict[str, float]]] = None,
    *,
    strategy: str = DEFAULT_STRATEGY,
    ownership_override: Optional[Dict[int, List[int]]] = None,
) -> Tuple[List[Dict[str, Any]], Dict[int, List[int]]]:
    """Split a donut boundary mission into per-UAV band pieces.

    Returns ``(pieces, ownership)`` where each piece dict is split-pipeline
    piece data (Geometry Area + band polygon + ``branchIndex`` + ``_donutPatrol``
    marker) and ownership maps ``band_index -> [aircraftID]`` (각자도생 sticky).
    ``ownership_override`` (a persisted band->owner map from the sticky store)
    pins the aircraft order on replans so bands never migrate between UAVs.
    """
    donut = donut_mission_from_input(mission)
    ids = [int(a) for a in uav_ids if _to_int(a) is not None] or [4]
    band_count = min(len(ids), 3) if strategy == "band" else len(ids)
    band_count = max(1, band_count)
    fractions = _band_fractions(strategy, band_count, 0.01)
    override_order: List[int] = []
    if ownership_override:
        for band_idx in range(band_count):
            owners = ownership_override.get(int(band_idx)) or []
            if owners:
                override_order.append(int(owners[0]))
    if len(override_order) == band_count:
        aircraft_order = override_order
    else:
        aircraft_order = _ownership_by_takeover(donut, fractions, ids, takeover_map)

    marker_base = {
        "outerCoordinateList": [dict(row) for row in donut.outer_latlon],
        "holeCoordinateLists": [[dict(row) for row in hole] for hole in donut.holes_latlon],
        "strategy": str(strategy),
        "aircraftOrder": [int(a) for a in aircraft_order],
        "inputMissionID": _to_int(mission.get("inputMissionID")) or 0,
    }

    mtype = _to_int(mission.get("inputMissionType")) or 3
    mission_id = mission.get("inputMissionID")
    pieces: List[Dict[str, Any]] = []
    ownership: Dict[int, List[int]] = {}
    for band_idx, (f0, f1) in enumerate(fractions):
        outer_edge, inner_edge = band_rings_latlon(donut, float(f0), float(f1))
        if len(outer_edge) < 3:
            outer_edge = [dict(row) for row in donut.outer_latlon]
            inner_edge = []
        owner = aircraft_order[band_idx] if band_idx < len(aircraft_order) else aircraft_order[band_idx % len(aircraft_order)]
        ownership[band_idx] = [int(owner)]
        marker = dict(marker_base)
        marker["bandIndex"] = int(band_idx)
        marker["bandFraction"] = [float(f0), float(f1)]
        # coordinateList = outer band edge (valid simple polygon for split/type);
        # the inner edge rides along so the 0302 emits a proper annulus (ring).
        pieces.append(
            {
                "Geometry": "Area",
                "coordinateList": outer_edge,
                "rawCoordinateList": outer_edge,
                "_donutBandInner": inner_edge,
                "inputMissionType": int(mtype),
                "MissionID": mission_id,
                "branchIndex": int(band_idx),
                "branchOwnerCount": 1,
                "_donutPatrol": marker,
            }
        )
    return pieces, ownership


def _marker_cache_key(marker: Dict[str, Any]) -> str:
    try:
        return json.dumps(
            {
                "outer": marker.get("outerCoordinateList"),
                "holes": marker.get("holeCoordinateLists"),
                "strategy": marker.get("strategy"),
                "order": marker.get("aircraftOrder"),
            },
            sort_keys=True,
        )
    except Exception:
        return str(id(marker))


def _plans_for_marker(marker: Dict[str, Any]):
    key = _marker_cache_key(marker)
    cached = _PLAN_CACHE.get(key)
    if cached is not None:
        return cached
    mission = {
        "inputMissionID": _to_int(marker.get("inputMissionID")) or 1,
        "inputMissionType": 3,
        "regionType": 7,
        "missionDetail": {
            "coordinateList": [],
            "lineList": [],
            "areaList": (
                [{"isHole": False, "coordinateList": marker.get("outerCoordinateList") or []}]
                + [
                    {"isHole": True, "coordinateList": hole}
                    for hole in (marker.get("holeCoordinateLists") or [])
                ]
            ),
        },
    }
    donut = donut_mission_from_input(mission)
    aircraft_order = [int(a) for a in (marker.get("aircraftOrder") or []) if _to_int(a) is not None]
    if not aircraft_order:
        aircraft_order = [4, 5, 6]
    config = PatrolConfig(
        strategy=str(marker.get("strategy") or DEFAULT_STRATEGY),
        aircraft_count=len(aircraft_order),
        aircraft_ids=tuple(aircraft_order),
        laps=1,
    )
    plans = build_patrol_plans(donut, config)
    if len(_PLAN_CACHE) >= _PLAN_CACHE_MAX:
        _PLAN_CACHE.clear()
    _PLAN_CACHE[key] = plans
    return plans


def _camera_coordinate_list(line_search, cam_alt_fn) -> List[Dict[str, Any]]:
    """DEM-grounded copy of a lineSearch coordinateList.

    Each lineSearch coordinate is a ground point the sensor films, so its
    altitude is set to the terrain elevation there (``cam_alt_fn``) rather than
    the aircraft's flight altitude.
    """
    camera_coords: List[Dict[str, Any]] = []
    for coord in line_search.coordinate_list:
        row_coord = dict(coord)
        try:
            ground = cam_alt_fn(float(row_coord["latitude"]), float(row_coord["longitude"]))
        except Exception:
            ground = None
        if ground is not None:
            row_coord["altitude"] = int(ground)
        camera_coords.append(row_coord)
    return camera_coords


def _line_filming_property(line_search, fov_deg, sensor_type, cam_alt_fn) -> OrderedDict:
    """OPMODE_LINE filmingProperty for sweeps filmed on the way to a waypoint."""
    return OrderedDict(
        [
            ("fieldOfView", round(float(fov_deg), 3)),
            ("sensorType", int(sensor_type)),
            ("operationMode", OPMODE_LINE),
            (
                "lineSearch",
                OrderedDict(
                    [
                        ("coordinateList", _camera_coordinate_list(line_search, cam_alt_fn)),
                        ("searchSpeed", round(float(line_search.search_speed_mps), 3)),
                        ("interpolationPoints", int(line_search.interpolation_points)),
                    ]
                ),
            ),
        ]
    )


def _orient_filming_property(line_search, fov_deg, sensor_type, cam_alt_fn) -> Optional[OrderedDict]:
    """OPMODE_POINT filmingProperty: stare at the first sweep target.

    The Area convention makes the first waypoint a coordinate-fixed orient WP:
    the sensor looks at where filming *will begin* (the first coordinate of the
    upcoming sweep) instead of filming from afar during the long transit in.
    """
    coords = _camera_coordinate_list(line_search, cam_alt_fn)
    if not coords:
        return None
    return OrderedDict(
        [
            ("fieldOfView", round(float(fov_deg), 3)),
            ("sensorType", int(sensor_type)),
            ("operationMode", OPMODE_POINT),
            ("coordinateOrientation", OrderedDict([("coordinate", coords[0])])),
        ]
    )


def build_donut_wplist(
    marker: Dict[str, Any],
    aircraft_id: int,
    *,
    altitude_fn: Optional[Callable[[float, float], int]] = None,
    camera_altitude_fn: Optional[Callable[[float, float], int]] = None,
    camera_altitude_many_fn: Optional[Callable[[List[Tuple[float, float]]], List[Any]]] = None,
) -> List[OrderedDict]:
    """Build production 0303 waypoint rows for one aircraft's donut patrol.

    Waypoint IDs stay 0 (the d0303 post-pass allocates them); eta is seconds and
    is recomputed downstream like other UAV rows. The waypoint coordinate uses
    the flight altitude (``altitude_fn``: DEM ground + layer offset), while each
    lineSearch coordinate is a ground point the sensor films, so its altitude is
    set to the terrain elevation there via ``camera_altitude_fn`` (defaulting to
    the shared DEM lookup) instead of the flight altitude.

    Filming is *arrival-packed* to match the Area waypoint convention: the SIM
    films a waypoint's lineSearch while traveling *to* that waypoint, so the
    sweeps physically flown on the leg into WP[i] are the departure sweeps of
    WP[i-1]. Waypoint 0 therefore carries no sweep - it becomes a
    coordinate-fixed orient waypoint (OPMODE_POINT) that stares at the first
    sweep target, so the sensor does not film from far away during the long
    transit onto the ring. The planner's terminal waypoint never owns a sweep
    (it is the FLYOVER return node), so nothing is dropped by the shift.
    """
    cam_alt_fn = camera_altitude_fn or _dem_ground_alt
    plans = _plans_for_marker(marker)
    plan = next((p for p in plans if int(p.aircraft_id) == int(aircraft_id)), None)
    if plan is None:
        return []

    waypoints = list(plan.waypoints)

    # One batched DEM resolve for every lookup this wplist needs: all camera
    # targets (hundreds per aircraft) plus the WP coordinates (also warms the
    # caller's DEM cache for altitude_fn's per-WP ground lookups). Anything the
    # batch misses falls back to the per-point fn - semantics unchanged.
    batch_points: List[Tuple[float, float]] = []
    for wp in waypoints:
        batch_points.append((float(wp.latitude), float(wp.longitude)))
        if wp.line_search is not None:
            for coord in wp.line_search.coordinate_list:
                try:
                    batch_points.append((float(coord["latitude"]), float(coord["longitude"])))
                except Exception:
                    continue
    resolved = _resolve_ground_alt_batch(batch_points, camera_altitude_many_fn)
    if resolved:
        point_fn = cam_alt_fn

        def cam_alt_fn(lat: float, lon: float):  # type: ignore[no-redef]
            value = resolved.get((round(float(lat), 6), round(float(lon), 6)))
            return int(value) if value is not None else point_fn(lat, lon)

    rows: List[OrderedDict] = []
    for i, wp in enumerate(waypoints):
        if altitude_fn is not None:
            try:
                altitude = int(altitude_fn(float(wp.latitude), float(wp.longitude)))
            except Exception:
                altitude = int(wp.altitude)
        else:
            # wp.altitude is the aircraft layer offset (1000/1010/1020); make the
            # flight altitude DEM-relative (ground + layer) like the initial plan
            # so replans fly the same AGL, not a fixed absolute layer.
            ground = _dem_ground_alt(float(wp.latitude), float(wp.longitude))
            altitude = int(wp.altitude) + (int(ground) if ground is not None else 0)
        row = OrderedDict(
            [
                ("waypointID", 0),
                (
                    "coordinate",
                    {
                        "latitude": round(float(wp.latitude), 8),
                        "longitude": round(float(wp.longitude), 8),
                        "altitude": int(altitude),
                    },
                ),
                ("speed", round(float(wp.speed_mps), 3)),
                ("eta", round(float(wp.eta_s), 3)),
                ("ecf", 0.0),
                ("nextWaypointID", 0),
                ("waypointPassType", int(wp.waypoint_pass_type)),
            ]
        )
        if i == 0:
            # First WP: stare at where filming begins instead of sweeping.
            if wp.line_search is not None:
                orient = _orient_filming_property(
                    wp.line_search, wp.fov_deg, wp.sensor_type, cam_alt_fn
                )
                if orient is not None:
                    row["filmingProperty"] = orient
        else:
            # Arrival packing: film the previous WP's departure sweeps on the way
            # in, so the coordinateList is not executed before the aircraft has
            # reached this leg.
            prev = waypoints[i - 1]
            if prev.line_search is not None:
                row["filmingProperty"] = _line_filming_property(
                    prev.line_search, prev.fov_deg, prev.sensor_type, cam_alt_fn
                )
        rows.append(row)
    return rows
