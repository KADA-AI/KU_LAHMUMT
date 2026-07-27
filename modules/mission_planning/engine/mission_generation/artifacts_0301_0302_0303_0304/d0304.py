from __future__ import annotations
import os
import math
import sys
import threading
import time
from collections import OrderedDict
from copy import deepcopy
from pathlib import Path
from typing import Callable, List, Tuple

_CANONICAL_NAME = "modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304.d0304"
_ALIASES = (
    "modules.mission_planning.MissionPlanner.data_def.d0304",
    "data_def.d0304",
)
if __name__ == _CANONICAL_NAME:
    for _ALIAS in _ALIASES:
        sys.modules.setdefault(_ALIAS, sys.modules[__name__])
elif __name__ in _ALIASES:
    sys.modules.setdefault(_CANONICAL_NAME, sys.modules[__name__])

try:
    from modules.mission_planning.MissionPlanner.data_def import route_planner_algorithms as route_algos
except Exception:
    import route_planner_algorithms as route_algos  # type: ignore
try:
    from modules.mission_planning.MissionPlanner.dynamics.lah_op_envlp import DEFAULT_ENVELOPE
except Exception:
    try:
        from dynamics.lah_op_envlp import DEFAULT_ENVELOPE  # type: ignore
    except Exception:
        DEFAULT_ENVELOPE = None  # type: ignore
from modules.mission_planning.MissionPlanner.data_def.mission_helpers import (
    now_ms_since_2000,
    terrain_elev,
    terrain_elev_many,
)
from modules.mission_planning.MissionPlanner.data_def.lah_terrain_path import (
    LAH_LOW_LEVEL_CLEARANCE_M,
    LAH_LOW_TERRAIN_CORRIDOR_M,
    LAH_VERTICAL_RATE_USE_RATIO,
    build_lah_terrain_following_path,
)
try:
    from modules.mission_planning.MissionPlanner.data_def.lah_terminal_cover import (
        select_lah_terminal_cover_point,
    )
except Exception:
    select_lah_terminal_cover_point = None  # type: ignore[assignment]
from modules.mission_planning.engine.mission_generation.id_allocation.allocator import (
    reserve_waypoint_block as _reserve_waypoint_block,
)
from modules.common.regional_dem import regional_dem_path_for_coordinate
try:
    from modules.mission_planning.MissionPlanner.runtime_settings import (
        get_runtime_altitude_layers_m as _get_runtime_altitude_layers_m,
        get_runtime_value as _get_runtime_value,
    )
except Exception:
    try:
        from modules.mission_planning.MissionPlanner.runtime_settings import (  # type: ignore
            get_runtime_altitude_layers_m as _get_runtime_altitude_layers_m,
            get_runtime_value as _get_runtime_value,
        )
    except Exception:
        _get_runtime_altitude_layers_m = None
        _get_runtime_value = None

_PROJECT_ROOT = Path(__file__).resolve().parents[5]
_MISSION_PLANNER_DIR = _PROJECT_ROOT / "modules" / "mission_planning" / "MissionPlanner"

def _sw_code(default: str = "MMR") -> str:
    """Resolve module code from KU_ROLE."""
    role = (os.environ.get("KU_ROLE") or "").lower()
    return {
        "mission": "MMR",
        "monitoring": "MSM",
        "decision": "MOB",
    }.get(role, default)

WP_INTERVAL_M = 3000.0
HOVER_HOLD_SEC = 10
HOVER_LAST_SEC = 300
ALTITUDE_LAYERS_M = (1000.0, 1010.0, 1020.0)
CAPSTONE_AREA_AGL_M = 200.0
CAPSTONE_BATTLE_HOLD_SEC = 3600
LAH_DEFAULT_CRUISE_SPEED_MPS = 40.0
LAH_UAV_ETA_PAIR_MAP = {1: 4, 2: 5, 3: 6}
# LAH1 is the command aircraft and carries the team datalink relay, so its
# altitude has to clear terrain to every UAV, not just its ETA-paired one.
LAH_COMMAND_RELAY_AIRCRAFT_ID = 1
LAH_TAKEOVER_HOLD_SOUTH_OFFSET_M = 0.0
# Package types whose initial plan starts the LAH route at the 0203 TakeOver
# point instead of the first mission's own anchor.  Type 2/3 branch packages
# use the same "TO -> first mission point -> hover" opening as Type 1.
LAH_INITIAL_TO_START_PACKAGE_TYPES_DEFAULT = (1, 2, 3)
LAH_UAV_FOLLOW_TARGET_GAP_M = 1000.0
LAH_UAV_FOLLOW_MIN_GAP_M = 500.0
LAH_UAV_FOLLOW_MAX_GAP_M = 2000.0
LAH_UAV_FOLLOW_MIN_SPEED_MPS = 8.0
LAH_UAV_FOLLOW_MIN_CRUISE_SCALE = 0.6
LAH_UAV_FOLLOW_MAX_CRUISE_SCALE = 2.5
LAH_UAV_FOLLOW_SEARCH_STEPS = 25
LAH_UAV_FOLLOW_TIME_PENALTY = 0.5
LAH_UAV_TERMINAL_MAX_DISTANCE_M = 20_000.0
LAH_UAV_TERMINAL_TARGET_DISTANCE_M = 18_000.0
LAH_UAV_TERMINAL_SOLVER_ITERATIONS = 32
LAH_UAV_CONTINUITY_MATCH_TOLERANCE_M = 25.0
LAH_UAV_LOS_CLEARANCE_M = 10.0
LAH_UAV_LOS_SAMPLE_STEP_M = 30.0
LAH_UAV_LOS_MAX_SAMPLES_PER_WAYPOINT = 768
LAH_UAV_LOS_MAX_TOTAL_SAMPLES = 32_768
LAH_LINE_CENTERLINE_KEEP_RATIO = 0.30  # 70% shorter than the original LINE centerline.
LAH_REEXECUTE_LINE_HOLD_SECONDS = 300
LAH_NON_ATTACK_CLEARANCE_M = LAH_LOW_LEVEL_CLEARANCE_M
LAH_LOW_TERRAIN_BOUNDARY_KEEP_RATIO = 0.80
LAH_LOW_TERRAIN_AREA_MARGIN_RATIO = 0.10
LAH_LOW_TERRAIN_AREA_MARGIN_MAX_M = 100.0
UINT32_MAX = (1 << 32) - 1
_LAH_MISSION_PLAN_TIMINGS_LOCAL = threading.local()
_DEM_ALT_CACHE_MAX = 200_000
_DEM_ALT_CACHE_LOCK = threading.Lock()
_DEM_ALT_CACHE: dict[tuple[float, float], float] = {}


def _runtime_value(name: str, default):
    env_name = "MISSION_PLAN_" + str(name).upper()
    raw = os.environ.get(env_name)
    if raw is not None:
        return raw
    if callable(_get_runtime_value):
        try:
            return _get_runtime_value(name, default)
        except Exception:
            return default
    return default


def _dem_alt_cache_round_decimals() -> int:
    try:
        raw = int(float(_runtime_value("dem_alt_cache_round_decimals", 7)))
    except Exception:
        raw = 7
    return max(0, min(7, int(raw)))


def _dem_alt_cache_key(lat: float, lon: float) -> tuple[float, float]:
    digits = _dem_alt_cache_round_decimals()
    return (round(float(lat), digits), round(float(lon), digits))


def _terrain_elev_cached(lat: float, lon: float) -> float:
    key = _dem_alt_cache_key(lat, lon)
    with _DEM_ALT_CACHE_LOCK:
        cached = _DEM_ALT_CACHE.get(key)
    if cached is not None:
        return float(cached)
    value = float(terrain_elev(float(lat), float(lon)))
    with _DEM_ALT_CACHE_LOCK:
        if len(_DEM_ALT_CACHE) >= _DEM_ALT_CACHE_MAX:
            _DEM_ALT_CACHE.clear()
        _DEM_ALT_CACHE[key] = float(value)
    return float(value)


_ORIGINAL_TERRAIN_ELEV_CACHED = _terrain_elev_cached


def _terrain_profile_many(coords) -> list[float]:
    # Keep unit-test/diagnostic monkeypatches of the established scalar helper
    # effective, while using the fast batch DEM path during normal operation.
    if _terrain_elev_cached is not _ORIGINAL_TERRAIN_ELEV_CACHED:
        return [
            float(_terrain_elev_cached(float(latitude), float(longitude)))
            for latitude, longitude in coords
        ]
    return [float(value) for value in terrain_elev_many(coords)]


def reset_lah_mission_plan_timings() -> None:
    _LAH_MISSION_PLAN_TIMINGS_LOCAL.rows = []


def get_last_lah_mission_plan_timings(*, reset: bool = False) -> list[dict]:
    rows = getattr(_LAH_MISSION_PLAN_TIMINGS_LOCAL, "rows", None)
    if not isinstance(rows, list):
        rows = []
    out = [dict(row) for row in rows if isinstance(row, dict)]
    if reset:
        reset_lah_mission_plan_timings()
    return out


def _lah_mission_kind(info: dict) -> str:
    if not isinstance(info, dict):
        return "unknown"
    if info.get("areaList"):
        return "area"
    if info.get("lineList"):
        return "line"
    if info.get("coordinateList"):
        return "coordinate"
    return "unknown"


def _is_lah_attack_mission(mission: dict, info: dict) -> bool:
    """Return True only for LAH missions whose own path is an attack path."""

    for source in (info, mission):
        if not isinstance(source, dict):
            continue
        try:
            if int(source.get("individualMissionType", 0) or 0) == 2:
                return True
        except Exception:
            pass
    return False


def _lah_base_input_id(mission: dict, info: dict) -> int:
    related = mission.get("relatedMission") if isinstance(mission.get("relatedMission"), dict) else {}
    for value in (
        related.get("inputMissionID"),
        related.get("missionID"),
        mission.get("inputMissionID"),
        info.get("inputMissionID") if isinstance(info, dict) else None,
    ):
        try:
            parsed = int(value)
        except Exception:
            continue
        if parsed > 0:
            return parsed
    return 0


def _record_lah_mission_plan_timing(
    mission: dict,
    info: dict,
    started_at: float,
    *,
    phase: str,
    path_id: int | None = None,
    waypoint_count: int = 0,
    skipped: bool = False,
) -> None:
    rows = getattr(_LAH_MISSION_PLAN_TIMINGS_LOCAL, "rows", None)
    if not isinstance(rows, list):
        reset_lah_mission_plan_timings()
        rows = getattr(_LAH_MISSION_PLAN_TIMINGS_LOCAL, "rows", [])
    try:
        elapsed_ms = (time.perf_counter() - started_at) * 1000.0
    except Exception:
        elapsed_ms = 0.0

    def _to_int(value: object) -> int:
        try:
            return int(value)
        except Exception:
            return 0

    rows.append(
        {
            "artifact": "0304",
            "phase": str(phase or ""),
            "aircraftID": _to_int(mission.get("aircraftID")),
            "individualMissionID": _to_int(mission.get("individualMissionID")),
            "individualMissionType": _to_int(info.get("individualMissionType") if isinstance(info, dict) else 0),
            "inputMissionID": _to_int(mission.get("inputMissionID")),
            "baseInputMissionID": _lah_base_input_id(mission, info),
            "pathID": _to_int(path_id if path_id is not None else mission.get("pathID")),
            "missionKind": _lah_mission_kind(info),
            "waypointCount": max(0, int(waypoint_count or 0)),
            "elapsedMs": round(float(elapsed_ms), 3),
            "skipped": bool(skipped),
        }
    )

def _lah_alt_agl(lat: float, lon: float, offset_m: float | int | None = None) -> int:
    try:
        ground = float(_terrain_elev_cached(lat, lon))
    except Exception:
        ground = 0.0
    try:
        if offset_m is None and _get_runtime_altitude_layers_m is not None:
            layers = tuple(float(value) for value in _get_runtime_altitude_layers_m() or ())
            offset = float(layers[0]) if layers else float(ALTITUDE_LAYERS_M[0])
        else:
            offset = float(ALTITUDE_LAYERS_M[0] if offset_m is None else offset_m)
    except Exception:
        offset = float(ALTITUDE_LAYERS_M[0])
    return int(round(ground + offset))


def _lah_non_attack_altitude_m(lat: float, lon: float) -> int:
    """Terrain-following altitude for every non-attack LAH waypoint."""

    return _lah_alt_agl(lat, lon, LAH_NON_ATTACK_CLEARANCE_M)


def _terrain_follow_non_attack_waypoints(
    waypoints: list[dict],
    *,
    cruise_speed: float,
    fallback_altitude_provider: Callable[[float, float], int] | None = None,
    route_coordinates: list[object] | None = None,
    low_terrain_segment_allowed: Callable[
        [tuple[float, float], tuple[float, float]], bool
    ]
    | None = None,
    low_terrain_constrained_leg_start_index: int = 0,
    low_terrain_corridor_m: float = LAH_LOW_TERRAIN_CORRIDOR_M,
    prefer_low_terrain: bool = True,
) -> list[OrderedDict]:
    """Replace a sparse horizontal route with a collision-safe DEM profile."""

    source_coordinates = list(route_coordinates or [])
    if not source_coordinates:
        source_coordinates = [
            waypoint.get("coordinate")
            for waypoint in waypoints or []
            if isinstance(waypoint, dict) and isinstance(waypoint.get("coordinate"), dict)
        ]
    profile = build_lah_terrain_following_path(
        source_coordinates,
        clearance_m=LAH_NON_ATTACK_CLEARANCE_M,
        terrain_provider=_terrain_profile_many,
        prefer_low_terrain=bool(prefer_low_terrain),
        low_terrain_corridor_m=low_terrain_corridor_m,
        low_terrain_segment_allowed=low_terrain_segment_allowed,
        low_terrain_constrained_leg_start_index=low_terrain_constrained_leg_start_index,
    )
    if not profile:
        fallback = [OrderedDict(waypoint) for waypoint in waypoints or []]
        if callable(fallback_altitude_provider):
            for waypoint in fallback:
                coordinate = waypoint.get("coordinate")
                if not isinstance(coordinate, dict):
                    continue
                try:
                    coordinate["altitude"] = int(
                        fallback_altitude_provider(
                            float(coordinate["latitude"]),
                            float(coordinate["longitude"]),
                        )
                    )
                except Exception:
                    continue
        _recompute_lah_eta_inplace({"lahWaypointList": fallback})
        return fallback

    total_length_m = max(float(profile[-1].get("cum_m", 0.0) or 0.0), 1.0)
    speed_mps = max(1e-6, float(cruise_speed))
    result: list[OrderedDict] = []
    for index, sample in enumerate(profile):
        cumulative_m = float(sample.get("cum_m", 0.0) or 0.0)
        result.append(
            OrderedDict(
                [
                    ("waypointID", 0),
                    ("isDone", False),
                    (
                        "coordinate",
                        {
                            "latitude": round(float(sample["latitude"]), 6),
                            "longitude": round(float(sample["longitude"]), 6),
                            "altitude": int(sample["altitude"]),
                        },
                    ),
                    ("speed", float(cruise_speed)),
                    ("eta", _clamp_eta_seconds(cumulative_m / speed_mps)),
                    (
                        "ecf",
                        1.0
                        if index == len(profile) - 1
                        else round(cumulative_m / total_length_m, 2),
                    ),
                    ("nextWaypointID", 0),
                ]
            )
        )
    _recompute_lah_eta_inplace({"lahWaypointList": result})
    return result


def _terrain_refine_existing_lah_waypoints(
    waypoints: list[dict],
    *,
    cruise_speed: float,
) -> list[OrderedDict]:
    """Insert terrain-driven transit points without replacing mission anchors.

    This variant is used for existing attack-capable paths.  Original
    waypoints (and therefore their target/action fields) remain in place; only
    horizontal transit points are inserted.  A preplanned higher altitude is
    retained, while DEM clearance can raise an unsafe chord.
    """

    source = [deepcopy(waypoint) for waypoint in waypoints or [] if isinstance(waypoint, dict)]
    if len(source) < 2:
        return [OrderedDict(waypoint) for waypoint in source]

    def _transit_waypoint(sample: dict, altitude_m: float) -> OrderedDict:
        return OrderedDict(
            [
                ("waypointID", 0),
                ("isDone", False),
                (
                    "coordinate",
                    {
                        "latitude": round(float(sample["latitude"]), 6),
                        "longitude": round(float(sample["longitude"]), 6),
                        "altitude": int(math.ceil(float(altitude_m))),
                    },
                ),
                ("speed", float(cruise_speed)),
                ("eta", 0),
                ("ecf", 0.0),
                ("nextWaypointID", 0),
            ]
        )

    result: list[OrderedDict] = [OrderedDict(source[0])]
    for left_source, right_source in zip(source, source[1:]):
        left_coord = _coord_from_wp(left_source)
        right_coord = _coord_from_wp(right_source)
        if left_coord is None or right_coord is None:
            result.append(OrderedDict(right_source))
            continue
        horizontal_m = _coord_dist_m(left_coord, right_coord)
        if horizontal_m <= 0.01:
            result.append(OrderedDict(right_source))
            continue
        try:
            profile = build_lah_terrain_following_path(
                [left_coord, right_coord],
                clearance_m=LAH_LOW_LEVEL_CLEARANCE_M,
                terrain_provider=_terrain_profile_many,
                prefer_low_terrain=False,
            )
        except Exception:
            profile = []
        if len(profile) < 2:
            result.append(OrderedDict(right_source))
            continue

        profile_length_m = max(float(profile[-1].get("cum_m", 0.0) or 0.0), 1e-6)
        left_altitude_m = float(left_coord.get("altitude", 0.0) or 0.0)
        right_altitude_m = float(right_coord.get("altitude", 0.0) or 0.0)
        left_has_action = _waypoint_has_meaningful_action(left_source)
        right_has_action = _waypoint_has_meaningful_action(right_source)
        planned_left_altitude_m = (
            float(profile[0].get("altitude", 0.0) or 0.0)
            if left_has_action
            else left_altitude_m
        )
        planned_right_altitude_m = (
            float(profile[-1].get("altitude", 0.0) or 0.0)
            if right_has_action
            else right_altitude_m
        )

        first_safe_altitude = (
            planned_left_altitude_m
            if left_has_action
            else max(left_altitude_m, planned_left_altitude_m)
        )
        if (
            left_has_action
            and abs(first_safe_altitude - left_altitude_m) >= 1.0
        ):
            result.append(_transit_waypoint(profile[0], first_safe_altitude))
        else:
            first_result_coord = result[-1].get("coordinate")
            if isinstance(first_result_coord, dict):
                first_result_coord["altitude"] = int(math.ceil(first_safe_altitude))

        for sample in profile[1:-1]:
            ratio = min(
                max(float(sample.get("cum_m", 0.0) or 0.0) / profile_length_m, 0.0),
                1.0,
            )
            planned_altitude_m = planned_left_altitude_m + (
                planned_right_altitude_m - planned_left_altitude_m
            ) * ratio
            sample_altitude_m = max(
                planned_altitude_m,
                float(sample.get("altitude", 0.0) or 0.0),
            )
            result.append(_transit_waypoint(sample, sample_altitude_m))

        right_waypoint = OrderedDict(right_source)
        right_result_coord = right_waypoint.get("coordinate")
        right_safe_altitude = (
            planned_right_altitude_m
            if right_has_action
            else max(right_altitude_m, planned_right_altitude_m)
        )
        if (
            right_has_action
            and abs(right_safe_altitude - right_altitude_m) >= 1.0
        ):
            result.append(_transit_waypoint(profile[-1], right_safe_altitude))
        elif isinstance(right_result_coord, dict):
            right_result_coord["altitude"] = int(math.ceil(right_safe_altitude))
        result.append(right_waypoint)

    packet = {"lahWaypointList": result}
    _recompute_lah_eta_inplace(packet)
    _recompute_lah_ecf_inplace(packet)
    return result


def _aircraft_alt_offset_m(aid: int) -> float:
    if _get_runtime_altitude_layers_m is not None:
        try:
            layers = tuple(float(value) for value in _get_runtime_altitude_layers_m() or ())
            if layers:
                idx = (int(aid) - 1) % len(layers)
                return float(layers[idx])
        except Exception:
            pass
    try:
        idx = (int(aid) - 1) % len(ALTITUDE_LAYERS_M)
    except Exception:
        idx = 0
    return float(ALTITUDE_LAYERS_M[idx])


def _median_ground_m(points: list[tuple[float, float]]) -> float | None:
    if not points:
        return None
    samples: list[float] = []
    for lat, lon in points:
        try:
            samples.append(float(_terrain_elev_cached(lat, lon)))
        except Exception:
            continue
    if not samples:
        return None
    samples.sort()
    n = len(samples)
    mid = n // 2
    if n % 2:
        return samples[mid]
    return (samples[mid - 1] + samples[mid]) / 2.0


def _capstone_area_center(area_list: list[dict]) -> tuple[float, float] | None:
    if not isinstance(area_list, list) or not area_list:
        return None
    area0 = area_list[0] if isinstance(area_list[0], dict) else {}
    coords_raw = area0.get("coordinateList") if isinstance(area0, dict) else None
    if not isinstance(coords_raw, list) or len(coords_raw) < 3:
        return None
    pts: list[tuple[float, float]] = []
    for row in coords_raw:
        if not isinstance(row, dict):
            continue
        try:
            pts.append((float(row["latitude"]), float(row["longitude"])))
        except Exception:
            continue
    if len(pts) < 3:
        return None
    if pts[0] != pts[-1]:
        pts = pts + [pts[0]]
    k = 111_132.92
    lat0 = sum(p[0] for p in pts[:-1]) / max(1, len(pts) - 1)
    lon0 = sum(p[1] for p in pts[:-1]) / max(1, len(pts) - 1)
    cos0 = max(1e-6, math.cos(math.radians(lat0)))
    xy = [((lon - lon0) * k * cos0, (lat - lat0) * k) for lat, lon in pts]
    area2 = 0.0
    cx = 0.0
    cy = 0.0
    for (x0, y0), (x1, y1) in zip(xy, xy[1:]):
        cross = x0 * y1 - x1 * y0
        area2 += cross
        cx += (x0 + x1) * cross
        cy += (y0 + y1) * cross
    if abs(area2) < 1e-6:
        lats = [p[0] for p in pts[:-1]]
        lons = [p[1] for p in pts[:-1]]
        return (sum(lats) / len(lats), sum(lons) / len(lons))
    cx /= (3.0 * area2)
    cy /= (3.0 * area2)
    return (lat0 + (cy / k), lon0 + (cx / (k * cos0)))

_DEFAULT_WP_EXT = OrderedDict([
    ("hovering", OrderedDict([("time", 0)])),
    ("loiter",   OrderedDict([
        ("radius",    0),
        ("direction", 0),
        ("time",      0),
        ("speed",     0),
    ])),
    ("attack",   OrderedDict([
        ("targetID",   0),
        ("weaponType", 0),
    ])),
])


def _strip_wp_extras(wp: dict) -> None:
    for key in ("hovering", "loiter", "attack"):
        if key in wp:
            del wp[key]


def _coerce_bounded_int(value: object, *, default: int = 0, minimum: int = 0, maximum: int = 0xFFFFFFFF) -> int:
    try:
        number = int(value)
    except Exception:
        return int(default)
    if number < int(minimum) or number > int(maximum):
        return int(default)
    return int(number)


def _ensure_lah_attack_default_inplace(wp: dict) -> None:
    if not isinstance(wp, dict):
        return
    raw_attack = wp.get("attack")
    attack = raw_attack if isinstance(raw_attack, dict) else {}
    wp["attack"] = OrderedDict([
        ("targetID", _coerce_bounded_int(attack.get("targetID"), default=0)),
        ("weaponType", _coerce_bounded_int(attack.get("weaponType"), default=0, minimum=0, maximum=3)),
    ])


class _WPAllocator:
    def __init__(self, start: int | None = None, end: int | None = None):
        self._local_next = start
        self._local_end = end
        self._use_global = start is None
    def alloc(self) -> int:
        if self._use_global:
            return int(_reserve_waypoint_block(1))
        if self._local_next is None:
            raise RuntimeError("Waypoint allocator misconfigured (local start unset)")
        if self._local_next <= 0:
            raise RuntimeError("WaypointID pool exhausted")
        wid = self._local_next
        if self._local_end is not None and wid > int(self._local_end):
            raise RuntimeError("WaypointID reserved block exhausted")
        self._local_next += 1
        return wid

def _offset_coord(lat: float, lon: float,
                  north_m: float = 0.0,
                  east_m: float  = 0.0) -> Tuple[float, float]:

    k = 111_132.92                         # m/deg (위도)
    dlat = north_m / k
    dlon = east_m  / (k * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon

def _interior_sample_distances_with_merged_tail(
    total_len_m: float,
    step_m: float,
) -> List[float]:
    total_len_m = max(float(total_len_m), 0.0)
    spacing = max(float(step_m), 1.0)
    if total_len_m <= spacing + 1e-6:
        return []
    n_full = int(total_len_m // spacing)
    targets = [float(spacing * idx) for idx in range(1, n_full + 1)]
    remainder_m = total_len_m - float(n_full) * spacing
    if remainder_m > 1e-6 and targets:
        targets.pop()
    return targets

def _split_line(p0: Tuple[float, float],
                p1: Tuple[float, float],
                step_m: float = WP_INTERVAL_M) -> List[Tuple[float, float]]:
    lat1, lon1 = p0; lat2, lon2 = p1
    k = 111_132.92
    cos = math.cos(math.radians((lat1 + lat2) / 2))
    dx = (lon2 - lon1) * k * cos; dy = (lat2 - lat1) * k
    dist = math.hypot(dx, dy)
    if dist < 1e-6:
        return []
    targets = _interior_sample_distances_with_merged_tail(dist, step_m)
    return [
        (
            lat1 + (lat2 - lat1) * (target_m / dist),
            lon1 + (lon2 - lon1) * (target_m / dist),
        )
        for target_m in targets
    ] + [p1]


def _dist_ll_m(p0: Tuple[float, float], p1: Tuple[float, float]) -> float:
    lat1, lon1 = p0
    lat2, lon2 = p1
    k = 111_132.92
    cos_mid = math.cos(math.radians((lat1 + lat2) / 2.0))
    dx = (lon2 - lon1) * k * cos_mid
    dy = (lat2 - lat1) * k
    return math.hypot(dx, dy)


def _polyline_length_m(coords: List[Tuple[float, float]]) -> float:
    if len(coords) < 2:
        return 0.0
    return sum(_dist_ll_m(p, q) for p, q in zip(coords, coords[1:]))


def _interp_ll(
    p0: Tuple[float, float],
    p1: Tuple[float, float],
    ratio: float,
) -> Tuple[float, float]:
    ratio = min(max(float(ratio), 0.0), 1.0)
    return (
        float(p0[0]) + (float(p1[0]) - float(p0[0])) * ratio,
        float(p0[1]) + (float(p1[1]) - float(p0[1])) * ratio,
    )


def _point_at_polyline_distance(
    coords: List[Tuple[float, float]],
    distance_m: float,
) -> Tuple[float, float]:
    if not coords:
        raise ValueError("empty polyline")
    if len(coords) == 1:
        return coords[0]
    target_m = max(float(distance_m), 0.0)
    traversed_m = 0.0
    for start, end in zip(coords, coords[1:]):
        seg_len_m = _dist_ll_m(start, end)
        if seg_len_m < 1e-6:
            continue
        next_m = traversed_m + seg_len_m
        if target_m <= next_m + 1e-6:
            return _interp_ll(start, end, (target_m - traversed_m) / seg_len_m)
        traversed_m = next_m
    return coords[-1]


def _append_unique_ll(
    points: List[Tuple[float, float]],
    point: Tuple[float, float],
) -> None:
    if not points or _dist_ll_m(points[-1], point) > 0.01:
        points.append(point)


def _slice_polyline_by_distance(
    coords: List[Tuple[float, float]],
    start_m: float,
    end_m: float,
) -> List[Tuple[float, float]]:
    if len(coords) < 2:
        return list(coords)
    total_len_m = _polyline_length_m(coords)
    if total_len_m < 1e-6:
        return list(coords)
    start_m = min(max(float(start_m), 0.0), total_len_m)
    end_m = min(max(float(end_m), start_m), total_len_m)
    if end_m - start_m < 1e-6:
        mid_pt = _point_at_polyline_distance(coords, (start_m + end_m) / 2.0)
        return [mid_pt]

    out: List[Tuple[float, float]] = []
    _append_unique_ll(out, _point_at_polyline_distance(coords, start_m))

    traversed_m = 0.0
    for start, end in zip(coords, coords[1:]):
        seg_len_m = _dist_ll_m(start, end)
        if seg_len_m < 1e-6:
            continue
        next_m = traversed_m + seg_len_m
        if start_m + 1e-6 < next_m < end_m - 1e-6:
            _append_unique_ll(out, end)
        traversed_m = next_m

    _append_unique_ll(out, _point_at_polyline_distance(coords, end_m))
    return out or list(coords)


def _trim_lah_line_centerline(
    coords: List[Tuple[float, float]],
    *,
    keep_ratio: float = LAH_LINE_CENTERLINE_KEEP_RATIO,
) -> List[Tuple[float, float]]:
    if len(coords) < 2:
        return list(coords)
    total_len_m = _polyline_length_m(coords)
    if total_len_m < 1e-6:
        return list(coords)
    keep_ratio = min(max(float(keep_ratio), 0.0), 1.0)
    if keep_ratio >= 1.0:
        return list(coords)
    keep_len_m = total_len_m * keep_ratio
    start_m = (total_len_m - keep_len_m) / 2.0
    return _slice_polyline_by_distance(coords, start_m, start_m + keep_len_m)


def _coord_rows_to_latlon_list(coord_rows: object) -> List[Tuple[float, float]]:
    out: List[Tuple[float, float]] = []
    if not isinstance(coord_rows, list):
        return out
    for row in coord_rows:
        if not isinstance(row, dict):
            continue
        try:
            out.append((float(row["latitude"]), float(row["longitude"])))
        except Exception:
            continue
    return out


def _coord_rows_to_lah_coordinate_list(coord_rows: object) -> List[dict]:
    """Normalize route anchors without discarding a reported LAH altitude."""

    out: List[dict] = []
    rows = coord_rows if isinstance(coord_rows, list) else [coord_rows]
    for raw_row in rows:
        if not isinstance(raw_row, dict):
            continue
        row = raw_row.get("coordinate") if isinstance(raw_row.get("coordinate"), dict) else raw_row
        try:
            latitude = float(row["latitude"])
            longitude = float(row["longitude"])
        except Exception:
            continue
        if not math.isfinite(latitude) or not math.isfinite(longitude):
            continue
        coordinate = {"latitude": latitude, "longitude": longitude}
        try:
            altitude = float(row.get("altitude"))
        except Exception:
            altitude = float("nan")
        if math.isfinite(altitude):
            coordinate["altitude"] = int(round(altitude))
        out.append(coordinate)
    return out


def lah_initial_to_start_package_types() -> set[int]:
    """Package types that open the LAH route at the 0203 TakeOver point.

    Kept runtime-tunable because the TO opening is only correct for a genuine
    initial plan; an operator can narrow it back to Type 1 without a code
    change if a package family turns out to need its old first-mission anchor.
    """

    raw = None
    if callable(_get_runtime_value):
        try:
            raw = _get_runtime_value(
                "lah_initial_to_start_package_types",
                None,
            )
        except Exception:
            raw = None
    if raw is None or (isinstance(raw, str) and not raw.strip()):
        return set(LAH_INITIAL_TO_START_PACKAGE_TYPES_DEFAULT)
    if isinstance(raw, (int, float)) and not isinstance(raw, bool):
        return {int(raw)}
    items = raw if isinstance(raw, (list, tuple, set)) else str(raw).split(",")
    parsed: set[int] = set()
    for item in items:
        try:
            parsed.add(int(str(item).strip()))
        except Exception:
            continue
    return parsed or set(LAH_INITIAL_TO_START_PACKAGE_TYPES_DEFAULT)


def lah_initial_hold_by_aircraft_from_mrpk(
    mrpk: object,
    *,
    paired_takeover_south_offset_m: float = LAH_TAKEOVER_HOLD_SOUTH_OFFSET_M,
) -> dict[int, dict[str, float]]:
    """Build LAH initial route-start anchors from paired UAV takeover points.

    A direct LAH takeover entry wins when one is supplied.  The normal 0203
    payload contains UAV4/5/6 entries only, so LAH1/2/3 use the established
    1<->4, 2<->5, 3<->6 pairing and wait at the paired UAV takeover point.
    Invalid or missing entries are ignored so legacy plans keep their current
    first-mission anchor.
    """

    payload: object = mrpk
    if isinstance(payload, list):
        payload = next(
            (row for row in reversed(payload) if isinstance(row, dict)),
            None,
        )
    if not isinstance(payload, dict):
        return {}

    raw_takeovers = payload.get("takeOverInfoList")
    if not isinstance(raw_takeovers, list):
        return {}

    coordinates_by_aircraft: dict[int, tuple[float, float]] = {}
    for row in raw_takeovers:
        if not isinstance(row, dict):
            continue
        coordinate = row.get("coordinate")
        if not isinstance(coordinate, dict):
            continue
        try:
            aircraft_id = int(row.get("aircraftID", 0) or 0)
            latitude = float(coordinate["latitude"])
            longitude = float(coordinate["longitude"])
        except Exception:
            continue
        if aircraft_id <= 0 or not math.isfinite(latitude) or not math.isfinite(longitude):
            continue
        coordinates_by_aircraft[aircraft_id] = (latitude, longitude)

    holds: dict[int, dict[str, float]] = {}
    south_offset_m = max(0.0, float(paired_takeover_south_offset_m))
    for lah_aircraft_id, paired_uav_id in LAH_UAV_ETA_PAIR_MAP.items():
        source = coordinates_by_aircraft.get(int(lah_aircraft_id))
        paired_fallback = source is None
        if paired_fallback:
            source = coordinates_by_aircraft.get(int(paired_uav_id))
        if source is None:
            continue
        latitude, longitude = source
        if paired_fallback and south_offset_m > 0.0:
            latitude, longitude = _offset_coord(
                latitude,
                longitude,
                north_m=-south_offset_m,
            )
        holds[int(lah_aircraft_id)] = {
            "latitude": float(latitude),
            "longitude": float(longitude),
        }
    return holds


def _extract_triplet_corridor_centerline(
    coords: List[Tuple[float, float]],
) -> List[Tuple[float, float]]:
    count = len(coords)
    if count < 6 or (count % 3) != 0:
        return []
    chunk = count // 3
    if chunk < 2:
        return []

    left_chain = coords[:chunk]
    center_chain = list(reversed(coords[chunk : chunk * 2]))
    right_chain = coords[chunk * 2 :]
    if len(left_chain) != len(center_chain) or len(center_chain) != len(right_chain):
        return []

    # Split line missions for LAH often arrive as [left boundary][centerline(reversed)][right boundary].
    # In that case the middle third is the actual route we want, not the outer polygon boundary.
    midpoint_ratio_sum = 0.0
    valid_pairs = 0
    for left_pt, center_pt, right_pt in zip(left_chain, center_chain, right_chain):
        span_m = _dist_ll_m(left_pt, right_pt)
        if span_m < 1.0:
            continue
        midpoint = (
            (float(left_pt[0]) + float(right_pt[0])) / 2.0,
            (float(left_pt[1]) + float(right_pt[1])) / 2.0,
        )
        midpoint_ratio_sum += _dist_ll_m(center_pt, midpoint) / span_m
        valid_pairs += 1
    if valid_pairs != len(center_chain):
        return []
    if (midpoint_ratio_sum / max(valid_pairs, 1)) > 0.2:
        return []
    return center_chain


def _resolve_lah_route_coords(info: dict) -> List[Tuple[float, float]]:
    coord_candidates: List[object] = []
    coord_candidates.append(info.get("coordinateList"))
    line_list = info.get("lineList") if isinstance(info.get("lineList"), list) else []
    if line_list and isinstance(line_list[0], dict):
        coord_candidates.append(line_list[0].get("coordinateList"))

    for raw_coords in coord_candidates:
        coords = _coord_rows_to_latlon_list(raw_coords)
        if not coords:
            continue
        centerline = _extract_triplet_corridor_centerline(coords)
        if centerline:
            return centerline
        return coords
    return []


def _has_lah_line_route_info(info: dict) -> bool:
    if not isinstance(info, dict):
        return False
    area_list = info.get("areaList") if isinstance(info.get("areaList"), list) else []
    line_list = info.get("lineList") if isinstance(info.get("lineList"), list) else []
    for line in line_list:
        if not isinstance(line, dict):
            continue
        if len(_coord_rows_to_latlon_list(line.get("coordinateList"))) >= 2:
            return True
    try:
        mission_type = int(info.get("individualMissionType", 0) or 0)
    except Exception:
        mission_type = 0
    if area_list:
        return False
    if mission_type in (6, 7) and len(_coord_rows_to_latlon_list(info.get("coordinateList"))) >= 2:
        return True
    return mission_type == 6


def _mission_low_terrain_segment_checker(
    info: dict,
) -> Callable[[tuple[float, float], tuple[float, float]], bool] | None:
    """Return a metric LINE/AREA containment check for low-terrain detours.

    LINE geometry keeps a 20% lateral reserve inside ``width / 2``. AREA
    geometry is inset by up to 100 m, including clearance from holes. If
    mission geometry exists but cannot be built, the returned checker rejects
    detours so the original route is retained instead of silently leaving the
    mission area.
    """

    if not isinstance(info, dict):
        return None
    line_list = info.get("lineList") if isinstance(info.get("lineList"), list) else []
    if not line_list and isinstance(info.get("_lahConstraintLineList"), list):
        line_list = info.get("_lahConstraintLineList")
    area_list = info.get("areaList") if isinstance(info.get("areaList"), list) else []
    if not area_list and isinstance(info.get("_lahConstraintAreaList"), list):
        area_list = info.get("_lahConstraintAreaList")
    if not line_list and not area_list:
        return None

    def _deny_detour(
        _start: tuple[float, float],
        _end: tuple[float, float],
    ) -> bool:
        return False

    geometry_coordinates: list[tuple[float, float]] = []
    for row in list(line_list) + list(area_list):
        if not isinstance(row, dict):
            continue
        geometry_coordinates.extend(_coord_rows_to_latlon_list(row.get("coordinateList")))
    if not geometry_coordinates:
        return _deny_detour

    try:
        from shapely.geometry import LineString, Point, Polygon
        from shapely.ops import unary_union
    except Exception:
        return _deny_detour

    origin_lat = sum(point[0] for point in geometry_coordinates) / len(geometry_coordinates)
    origin_lon = sum(point[1] for point in geometry_coordinates) / len(geometry_coordinates)
    metres_per_lat = 111_132.92
    metres_per_lon = max(1.0, metres_per_lat * math.cos(math.radians(origin_lat)))

    def _xy(point: tuple[float, float]) -> tuple[float, float]:
        return (
            (float(point[1]) - origin_lon) * metres_per_lon,
            (float(point[0]) - origin_lat) * metres_per_lat,
        )

    line_geometries: list[object] = []
    outer_areas: list[object] = []
    hole_areas: list[object] = []
    try:
        for row in line_list:
            if not isinstance(row, dict):
                continue
            centerline = _coord_rows_to_latlon_list(row.get("coordinateList"))
            extracted = _extract_triplet_corridor_centerline(centerline)
            if extracted:
                centerline = extracted
            try:
                width_m = float(row.get("width", row.get("widthM", 0.0)) or 0.0)
            except Exception:
                width_m = 0.0
            if len(centerline) < 2 or not math.isfinite(width_m) or width_m <= 0.0:
                continue
            usable_half_width_m = max(
                1.0,
                width_m * 0.5 * float(LAH_LOW_TERRAIN_BOUNDARY_KEEP_RATIO),
            )
            line_geometries.append(
                LineString([_xy(point) for point in centerline]).buffer(
                    usable_half_width_m,
                    cap_style=2,
                    join_style=1,
                )
            )

        for row in area_list:
            if not isinstance(row, dict):
                continue
            points = _coord_rows_to_latlon_list(row.get("coordinateList"))
            if len(points) < 3:
                continue
            polygon = Polygon([_xy(point) for point in points]).buffer(0)
            if polygon.is_empty:
                continue
            if _truthy_lah_flag(row.get("isHole")):
                hole_areas.append(polygon)
            else:
                outer_areas.append(polygon)

        # AREA is authoritative when present; do not let an incidental LINE
        # row open a corridor through an AREA boundary or excluded hole.
        allowed_parts: list[object] = []
        if outer_areas:
            area_geometry = unary_union(outer_areas).buffer(0)
            if hole_areas:
                area_geometry = area_geometry.difference(unary_union(hole_areas)).buffer(0)
            if not area_geometry.is_empty:
                min_x, min_y, max_x, max_y = area_geometry.bounds
                short_span_m = max(0.0, min(max_x - min_x, max_y - min_y))
                margin_m = min(
                    float(LAH_LOW_TERRAIN_AREA_MARGIN_MAX_M),
                    short_span_m * float(LAH_LOW_TERRAIN_AREA_MARGIN_RATIO),
                )
                if margin_m > 0.0:
                    area_geometry = area_geometry.buffer(-margin_m, join_style=1).buffer(0)
            if not area_geometry.is_empty:
                allowed_parts.append(area_geometry)
        elif not area_list:
            allowed_parts.extend(line_geometries)
        if not allowed_parts:
            return _deny_detour
        allowed_geometry = unary_union(allowed_parts).buffer(0)
        if allowed_geometry.is_empty:
            return _deny_detour
    except Exception:
        return _deny_detour

    def _segment_allowed(
        start: tuple[float, float],
        end: tuple[float, float],
    ) -> bool:
        try:
            if _dist_ll_m(start, end) <= 0.01:
                return bool(allowed_geometry.covers(Point(_xy(start))))
            return bool(
                allowed_geometry.covers(
                    LineString([_xy(start), _xy(end)])
                )
            )
        except Exception:
            return False

    return _segment_allowed


def _lah_terminal_cover_area_rows(info: dict | None) -> list[dict]:
    """Return the private AREA/Hole constraint carried by a LAH hold mission."""

    if not isinstance(info, dict):
        return []
    rows = info.get("_lahConstraintAreaList")
    if not isinstance(rows, list):
        rows = info.get("areaList")
    return [row for row in (rows or []) if isinstance(row, dict)]


def _lah_terminal_cover_info_by_path(missions: object | None) -> dict[int, dict]:
    """Index only terminal-cover mission metadata; never copy it to 0304 output."""

    rows = missions if isinstance(missions, list) else []
    out: dict[int, dict] = {}
    for mission in rows:
        if not isinstance(mission, dict):
            continue
        info = mission.get("individualMissionInfo")
        if not isinstance(info, dict) or not _truthy_lah_flag(info.get("_lahTerminalCoverEnabled")):
            continue
        if not _lah_terminal_cover_area_rows(info):
            continue
        try:
            path_id = int(mission.get("pathID", 0) or 0)
        except Exception:
            path_id = 0
        if path_id > 0:
            out[path_id] = info
    return out


def _constrain_lah_terminal_support_coordinate(
    info: dict,
    desired_coord: dict,
    fallback_coord: dict,
) -> dict:
    """Keep a terminal relocation and its ingress segment inside AREA/Holes.

    The AREA constraint wins over the 20 km proximity objective.  If a desired
    support point is outside (or crosses a Hole), the already-safe cover point
    is retained; the caller can still command maximum speed without failing the
    mission plan.
    """

    if not isinstance(desired_coord, dict):
        return dict(fallback_coord)
    if not _lah_terminal_cover_area_rows(info):
        return dict(desired_coord)
    checker = _mission_low_terrain_segment_checker(info)
    if not callable(checker):
        return dict(fallback_coord)
    try:
        desired = (float(desired_coord["latitude"]), float(desired_coord["longitude"]))
        fallback = (float(fallback_coord["latitude"]), float(fallback_coord["longitude"]))
        if bool(checker(desired, desired)) and bool(checker(fallback, desired)):
            return dict(desired_coord)
    except Exception:
        pass
    return dict(fallback_coord)


def _mission_low_terrain_corridor_m(info: dict) -> float:
    """Keep LINE candidates distributed inside the narrowest declared width."""

    if not isinstance(info, dict):
        return float(LAH_LOW_TERRAIN_CORRIDOR_M)
    area_list = info.get("areaList") if isinstance(info.get("areaList"), list) else []
    if not area_list and isinstance(info.get("_lahConstraintAreaList"), list):
        area_list = info.get("_lahConstraintAreaList")
    if area_list:
        return float(LAH_LOW_TERRAIN_CORRIDOR_M)
    line_list = info.get("lineList") if isinstance(info.get("lineList"), list) else []
    if not line_list and isinstance(info.get("_lahConstraintLineList"), list):
        line_list = info.get("_lahConstraintLineList")
    half_widths: list[float] = []
    for row in line_list:
        if not isinstance(row, dict):
            continue
        try:
            width_m = float(row.get("width", row.get("widthM", 0.0)) or 0.0)
        except Exception:
            continue
        if math.isfinite(width_m) and width_m > 0.0:
            half_widths.append(width_m * 0.5)
    if not half_widths:
        return float(LAH_LOW_TERRAIN_CORRIDOR_M)
    half_width_m = min(half_widths)
    return max(
        1.0,
        min(
            float(LAH_LOW_TERRAIN_CORRIDOR_M),
            half_width_m * float(LAH_LOW_TERRAIN_BOUNDARY_KEEP_RATIO),
        ),
    )


def _route_is_inside_mission_geometry(
    route: list[tuple[float, float]],
    segment_allowed: Callable[[tuple[float, float], tuple[float, float]], bool] | None,
) -> bool:
    if not callable(segment_allowed) or not route:
        return True
    if len(route) == 1:
        return bool(segment_allowed(route[0], route[0]))
    return all(
        bool(segment_allowed(start, end))
        for start, end in zip(route, route[1:])
    )


def _truthy_lah_flag(value: object) -> bool:
    if isinstance(value, bool):
        return bool(value)
    if isinstance(value, (int, float)):
        return float(value) != 0.0
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on", "y"}
    return False


def _lah_line_hold_seconds(mission: dict, info: dict) -> int | None:
    for source in (mission, info):
        if not isinstance(source, dict):
            continue
        for key in (
            "_lahLineHoldSeconds",
            "_lahHoldSeconds",
            "lahLineHoldSeconds",
            "lahHoldSeconds",
            "replanHoldSeconds",
        ):
            if key not in source:
                continue
            try:
                value = int(round(float(source.get(key))))
            except Exception:
                continue
            if value > 0:
                return int(value)
        for key in (
            "_lahHoldAtLineEnd",
            "_lahReexecuteLineHold",
            "lahHoldAtLineEnd",
            "lahReexecuteLineHold",
        ):
            if _truthy_lah_flag(source.get(key)):
                return int(LAH_REEXECUTE_LINE_HOLD_SECONDS)
    return None


def _preserve_lah_line_endpoints(mission: dict, info: dict) -> bool:
    for source in (mission, info):
        if not isinstance(source, dict):
            continue
        for key in (
            "_lahPreserveLineEndpoints",
            "lahPreserveLineEndpoints",
            "preserveLineEndpoints",
        ):
            if _truthy_lah_flag(source.get(key)):
                return True
    return False


def _forced_lah_altitude_m(info: dict) -> int | None:
    if not isinstance(info, dict):
        return None
    for key in ("forceAltitudeM", "forcedAltitudeM", "forceAltitude", "forcedAltitude"):
        if key not in info:
            continue
        try:
            value = float(info.get(key))
        except Exception:
            continue
        if value > 0.0:
            return int(round(value))
    return None


def _resample_route_samples(
    samples: List[dict],
    *,
    step_m: float = WP_INTERVAL_M,
) -> List[dict]:
    points: List[Tuple[float, float]] = []
    for sample in samples or []:
        if not isinstance(sample, dict):
            continue
        try:
            points.append((float(sample["lat"]), float(sample["lon"])))
        except Exception:
            continue
    if not points:
        return []
    if len(points) == 1:
        return [{"lat": points[0][0], "lon": points[0][1], "cum_m": 0.0}]

    spacing = max(float(step_m), 1.0)
    out: List[dict] = [{"lat": points[0][0], "lon": points[0][1], "cum_m": 0.0}]
    total_len_m = 0.0
    for start, end in zip(points, points[1:]):
        total_len_m += _dist_ll_m(start, end)
    target_distances = _interior_sample_distances_with_merged_tail(total_len_m, spacing)
    target_index = 0
    traversed_m = 0.0

    for start, end in zip(points, points[1:]):
        seg_len_m = _dist_ll_m(start, end)
        if seg_len_m < 1e-6:
            continue
        seg_start_m = traversed_m
        seg_end_m = traversed_m + seg_len_m
        while target_index < len(target_distances) and target_distances[target_index] < seg_end_m - 1e-6:
            target_m = float(target_distances[target_index])
            ratio = (target_m - seg_start_m) / seg_len_m
            lat = float(start[0]) + (float(end[0]) - float(start[0])) * ratio
            lon = float(start[1]) + (float(end[1]) - float(start[1])) * ratio
            last = out[-1]
            if _dist_ll_m((float(last["lat"]), float(last["lon"])), (lat, lon)) >= 1.0:
                out.append({"lat": lat, "lon": lon, "cum_m": float(target_m)})
            target_index += 1
        traversed_m = seg_end_m
        last = out[-1]
        if _dist_ll_m((float(last["lat"]), float(last["lon"])), end) >= 1.0:
            out.append({"lat": float(end[0]), "lon": float(end[1]), "cum_m": float(traversed_m)})
        else:
            out[-1]["lat"] = float(end[0])
            out[-1]["lon"] = float(end[1])
            out[-1]["cum_m"] = float(traversed_m)
    return out


def _coord_from_wp(wp: dict) -> dict | None:
    coord = wp.get("coordinate") if isinstance(wp, dict) else None
    if not isinstance(coord, dict):
        return None
    try:
        return {
            "latitude": float(coord["latitude"]),
            "longitude": float(coord["longitude"]),
            "altitude": int(round(float(coord.get("altitude", 0.0) or 0.0))),
        }
    except Exception:
        return None


def _coord_between(c0: dict, c1: dict, ratio: float) -> dict:
    alpha = min(max(float(ratio), 0.0), 1.0)
    return {
        "latitude": float(c0["latitude"]) + (float(c1["latitude"]) - float(c0["latitude"])) * alpha,
        "longitude": float(c0["longitude"]) + (float(c1["longitude"]) - float(c0["longitude"])) * alpha,
        "altitude": int(round(
            float(c0.get("altitude", 0.0) or 0.0)
            + (float(c1.get("altitude", 0.0) or 0.0) - float(c0.get("altitude", 0.0) or 0.0)) * alpha
        )),
    }


def _normalize_altitude_value(value: object) -> int:
    try:
        return int(round(float(value or 0.0)))
    except Exception:
        return 0


def _clamp_eta_seconds(value: object) -> int:
    """Return an ICD-compatible uint32 ETA expressed in whole seconds."""
    try:
        seconds = float(value or 0.0)
    except Exception:
        seconds = 0.0
    if not math.isfinite(seconds) or seconds <= 0.0:
        return 0
    return min(UINT32_MAX, int(round(seconds)))


def normalize_lah_eta_seconds_inplace(waypoints: list[dict]) -> None:
    """Normalize one LAH flight-plan timeline to cumulative uint32 seconds.

    ETA is relative to the containing flight plan: the first waypoint is
    always zero.  A trimmed suffix therefore has its former first ETA removed
    from every following waypoint.  Existing cumulative travel times are kept
    monotonic; operation duration remains in hovering/loiter fields.
    """
    if not isinstance(waypoints, list) or not waypoints:
        return

    raw_eta: list[int] = []
    for waypoint in waypoints:
        try:
            value = float(waypoint.get("eta", 0)) if isinstance(waypoint, dict) else 0.0
        except Exception:
            value = 0.0
        raw_eta.append(int(round(value)) if math.isfinite(value) and value > 0.0 else 0)
    baseline = raw_eta[0]
    previous = 0
    for index, waypoint in enumerate(waypoints):
        if not isinstance(waypoint, dict):
            continue
        relative = 0 if index == 0 else max(0, raw_eta[index] - baseline)
        current = min(UINT32_MAX, max(previous, relative))
        waypoint["eta"] = int(current)
        previous = int(current)


def _normalize_altitudes_recursive_inplace(node: object) -> None:
    if isinstance(node, dict):
        if "altitude" in node:
            node["altitude"] = _normalize_altitude_value(node.get("altitude", 0))
        for value in node.values():
            _normalize_altitudes_recursive_inplace(value)
        return
    if isinstance(node, list):
        for item in node:
            _normalize_altitudes_recursive_inplace(item)


def _ensure_minimum_lah_waypoints_inplace(waypoints: list[dict]) -> None:
    if not isinstance(waypoints, list) or len(waypoints) != 1:
        return
    original = waypoints[0]
    if not isinstance(original, dict):
        return
    # ETA means arrival time within this flight plan.  Even a single hold
    # waypoint therefore starts at zero; its duration stays in hovering/loiter.
    original["eta"] = 0
    if original.pop("_allowSingleLahWaypoint", False):
        original["nextWaypointID"] = 0
        original["ecf"] = 1.0
        return
    try:
        assigned_waypoint_id = int(original.get("waypointID", 0) or 0)
    except Exception:
        assigned_waypoint_id = 0
    if assigned_waypoint_id > 0:
        # This can run again after waypoint IDs have already been assigned.
        # Duplicating then would copy the same waypointID into the terminal WP.
        original["nextWaypointID"] = 0
        original["ecf"] = 1.0
        return

    terminal = deepcopy(original)

    if "hovering" in original:
        terminal["hovering"] = deepcopy(original.get("hovering"))
        original.pop("hovering", None)

    terminal.pop("attack", None)
    terminal.pop("filmingProperty", None)

    original["eta"] = 0
    terminal["eta"] = 0
    original["ecf"] = 0.0
    terminal["ecf"] = 1.0
    original["nextWaypointID"] = 0
    terminal["nextWaypointID"] = 0

    waypoints.append(terminal)


def _normalize_lah_waypoint_list_inplace(waypoints: list[dict]) -> None:
    if not isinstance(waypoints, list):
        return
    for wp in waypoints:
        _normalize_altitudes_recursive_inplace(wp)
    _ensure_minimum_lah_waypoints_inplace(waypoints)
    for wp in waypoints:
        _normalize_altitudes_recursive_inplace(wp)
        _ensure_lah_attack_default_inplace(wp)
    normalize_lah_eta_seconds_inplace(waypoints)


def _coord_vector_m(c0: dict, c1: dict) -> tuple[float, float]:
    lat1 = float(c0["latitude"])
    lon1 = float(c0["longitude"])
    lat2 = float(c1["latitude"])
    lon2 = float(c1["longitude"])
    k = 111_132.92
    cos_mid = math.cos(math.radians((lat1 + lat2) / 2.0))
    dx = (lon2 - lon1) * k * cos_mid
    dy = (lat2 - lat1) * k
    return dx, dy


def _coord_dist_m(c0: dict, c1: dict) -> float:
    dx, dy = _coord_vector_m(c0, c1)
    return math.hypot(dx, dy)


def _lah_vertical_rate_mps(altitude_delta_m: float) -> float:
    """Return a conservative planning rate for one LAH vertical leg."""

    rate_attr = "climb_rate_mps" if float(altitude_delta_m) >= 0.0 else "descent_rate_mps"
    fallback_rate_mps = 8.9 if float(altitude_delta_m) >= 0.0 else 7.0
    try:
        rate_mps = float(getattr(DEFAULT_ENVELOPE, rate_attr, fallback_rate_mps))
    except Exception:
        rate_mps = float(fallback_rate_mps)
    if not math.isfinite(rate_mps) or rate_mps <= 0.0:
        rate_mps = float(fallback_rate_mps)
    return max(0.1, rate_mps * float(LAH_VERTICAL_RATE_USE_RATIO))


def _minimum_lah_leg_time_s(start_coord: dict, end_coord: dict, speed_mps: float) -> float:
    """Return a feasible LAH leg time including simultaneous vertical motion.

    Horizontal travel and climb/descent occur together, so the binding travel
    time is the larger of the two.  Each leg is rounded upward because 0304 ETA
    is serialized as whole seconds; rounding to nearest could exceed the
    planned vertical rate by almost half a second per leg.
    """

    try:
        horizontal_m = max(0.0, float(_coord_dist_m(start_coord, end_coord)))
    except Exception:
        horizontal_m = 0.0
    try:
        horizontal_speed_mps = float(speed_mps)
    except Exception:
        horizontal_speed_mps = 0.0
    if not math.isfinite(horizontal_speed_mps) or horizontal_speed_mps <= 0.0:
        horizontal_speed_mps = 1.0
    horizontal_time_s = horizontal_m / horizontal_speed_mps

    try:
        altitude_delta_m = float(end_coord.get("altitude", 0.0) or 0.0) - float(
            start_coord.get("altitude", 0.0) or 0.0
        )
    except Exception:
        altitude_delta_m = 0.0
    vertical_time_s = abs(altitude_delta_m) / _lah_vertical_rate_mps(altitude_delta_m)
    return float(math.ceil(max(0.0, horizontal_time_s, vertical_time_s) - 1e-9))


def _segment_unit_xy(c0: dict, c1: dict) -> tuple[float, float]:
    dx, dy = _coord_vector_m(c0, c1)
    norm = math.hypot(dx, dy)
    if norm < 1e-6:
        return (1.0, 0.0)
    return (dx / norm, dy / norm)


def _uav_timeline_from_packet(packet: dict) -> list[dict]:
    waypoints = _packet_waypoints(packet)
    timeline: list[dict] = []
    for wp in waypoints:
        coord = _coord_from_wp(wp)
        if coord is None:
            continue
        try:
            eta_s = _eta_to_seconds(wp.get("eta", 0), assume_ms=_packet_eta_is_milliseconds(packet))
        except Exception:
            eta_s = 0.0
        if timeline and eta_s < float(timeline[-1]["eta_s"]):
            eta_s = float(timeline[-1]["eta_s"])
        timeline.append({"eta_s": float(eta_s), "coord": coord})
    return timeline


def _packet_waypoints(packet: dict) -> list[dict]:
    for key in ("uavWaypointList", "waypointList", "lahWaypointList"):
        candidate = packet.get(key) if isinstance(packet, dict) else None
        if isinstance(candidate, list):
            return candidate
    return []


def _packet_eta_is_milliseconds(packet: dict) -> bool:
    # Kept as a compatibility hook for callers, but both 0303 and 0304 now
    # publish ETA using the ICD unit: uint32 seconds.
    return False


def _eta_to_seconds(raw_eta: object, *, assume_ms: bool) -> float:
    try:
        eta = float(raw_eta or 0.0)
    except Exception:
        eta = 0.0
    if assume_ms:
        return eta / 1000.0
    return eta


def _packet_terminal_coords(packet: dict) -> tuple[dict | None, dict | None]:
    waypoints = _packet_waypoints(packet)
    if not waypoints:
        return None, None
    start = _coord_from_wp(waypoints[0])
    end = _coord_from_wp(waypoints[-1])
    return start, end


def _pair_uav_packet_for_lah(lah_packet: dict, uav_packets: list[dict]) -> dict | None:
    if not uav_packets:
        return None
    lah_start, lah_end = _packet_terminal_coords(lah_packet)
    if lah_start is None:
        return uav_packets[0]
    best_packet = None
    best_score = None
    for pkt in uav_packets:
        uav_start, uav_end = _packet_terminal_coords(pkt)
        if uav_start is None:
            continue
        score = _coord_dist_m(lah_start, uav_start)
        if lah_end is not None and uav_end is not None:
            score += _coord_dist_m(lah_end, uav_end)
        if best_score is None or score < best_score:
            best_score = score
            best_packet = pkt
    return best_packet or uav_packets[0]


def _pair_lah_packet_by_geometry(target_packet: dict, candidate_packets: list[dict]) -> dict | None:
    if not candidate_packets:
        return None
    return _pair_uav_packet_for_lah(target_packet, candidate_packets)


def _leader_speed_profile_from_packet(packet: dict) -> list[tuple[float, float]]:
    profile: list[tuple[float, float]] = []
    wplist = packet.get("lahWaypointList") or []
    for idx, wp in enumerate(wplist):
        try:
            speed = float(wp.get("speed", 0.0) or 0.0)
        except Exception:
            continue
        if speed <= 0.0:
            continue
        try:
            progress = float(wp.get("ecf", 0.0) or 0.0)
        except Exception:
            progress = 0.0
        if idx == len(wplist) - 1:
            progress = 1.0
        progress = min(max(progress, 0.0), 1.0)
        profile.append((progress, speed))
    if not profile:
        return []
    profile.sort(key=lambda item: (item[0], item[1]))
    return profile


def _speed_from_progress(profile: list[tuple[float, float]], progress: float) -> float | None:
    if not profile:
        return None
    x = min(max(float(progress), 0.0), 1.0)
    if x <= profile[0][0]:
        return float(profile[0][1])
    for left, right in zip(profile, profile[1:]):
        p0, s0 = left
        p1, s1 = right
        if x <= p1:
            if p1 <= p0 + 1e-9:
                return float(s1)
            ratio = (x - p0) / max(p1 - p0, 1e-9)
            return float(s0 + (s1 - s0) * ratio)
    return float(profile[-1][1])


def _recompute_lah_eta_inplace(packet: dict) -> None:
    wplist = packet.get("lahWaypointList") or []
    if not isinstance(wplist, list) or not wplist:
        return
    prev_coord = _coord_from_wp(wplist[0])
    cum_eta_s = 0.0
    wplist[0]["eta"] = 0
    for idx in range(1, len(wplist)):
        curr_coord = _coord_from_wp(wplist[idx])
        if prev_coord is None or curr_coord is None:
            wplist[idx]["eta"] = _clamp_eta_seconds(cum_eta_s)
            prev_coord = curr_coord
            continue
        try:
            speed_mps = float(wplist[idx].get("speed", 0.0) or 0.0)
        except Exception:
            speed_mps = 0.0
        if speed_mps <= 0.0:
            speed_mps = 1.0
        cum_eta_s += _minimum_lah_leg_time_s(prev_coord, curr_coord, speed_mps)
        wplist[idx]["eta"] = _clamp_eta_seconds(cum_eta_s)
        prev_coord = curr_coord
    normalize_lah_eta_seconds_inplace(wplist)


def _sync_lah_packet_speeds_to_leader(
    lah_packets: List[dict],
    *,
    preferred_leader_ids: tuple[int, ...] = (1, 2, 3),
    immutable_path_ids: set[int] | None = None,
) -> None:
    immutable_ids = {int(value) for value in (immutable_path_ids or set())}
    packets_by_aircraft: dict[int, list[dict]] = {}
    for pkt in lah_packets or []:
        try:
            aid = int(pkt.get("aircraftID", 0))
        except Exception:
            continue
        if aid in (1, 2, 3):
            packets_by_aircraft.setdefault(aid, []).append(pkt)

    leader_id = None
    for candidate in preferred_leader_ids:
        if packets_by_aircraft.get(candidate):
            leader_id = int(candidate)
            break
    if leader_id is None:
        return

    leader_packets = packets_by_aircraft.get(leader_id) or []
    for aid, packets in packets_by_aircraft.items():
        if aid == leader_id:
            continue
        for pkt in packets:
            try:
                if int(pkt.get("pathID", 0) or 0) in immutable_ids:
                    continue
            except Exception:
                pass
            leader_pkt = _pair_lah_packet_by_geometry(pkt, leader_packets)
            if not isinstance(leader_pkt, dict):
                continue
            profile = _leader_speed_profile_from_packet(leader_pkt)
            wplist = pkt.get("lahWaypointList") or []
            if not profile or not isinstance(wplist, list) or not wplist:
                continue
            for idx, wp in enumerate(wplist):
                try:
                    progress = float(wp.get("ecf", 0.0) or 0.0)
                except Exception:
                    progress = 0.0
                if idx == len(wplist) - 1:
                    progress = 1.0
                ref_speed = _speed_from_progress(profile, progress)
                if ref_speed is None:
                    continue
                wp["speed"] = round(float(ref_speed), 2)
            _recompute_lah_eta_inplace(pkt)


def _uav_state_at_eta_s(timeline: list[dict], eta_s: float) -> tuple[dict, tuple[float, float]] | None:
    if not timeline:
        return None
    if len(timeline) == 1:
        return dict(timeline[0]["coord"]), (1.0, 0.0)
    eta_s = float(eta_s)
    if eta_s <= float(timeline[0]["eta_s"]):
        start = timeline[0]
        end = timeline[1]
        return dict(start["coord"]), _segment_unit_xy(start["coord"], end["coord"])
    for idx in range(1, len(timeline)):
        prev = timeline[idx - 1]
        curr = timeline[idx]
        t0 = float(prev["eta_s"])
        t1 = float(curr["eta_s"])
        if eta_s <= t1:
            if t1 <= t0 + 1e-6:
                return dict(curr["coord"]), _segment_unit_xy(prev["coord"], curr["coord"])
            ratio = (eta_s - t0) / max(t1 - t0, 1e-6)
            coord = _coord_between(prev["coord"], curr["coord"], ratio)
            return coord, _segment_unit_xy(prev["coord"], curr["coord"])
    prev = timeline[-2]
    curr = timeline[-1]
    return dict(curr["coord"]), _segment_unit_xy(prev["coord"], curr["coord"])


def _lah_max_altitude_m() -> float:
    try:
        value = float(getattr(DEFAULT_ENVELOPE, "max_altitude_m", 2000.0))
    except Exception:
        value = 2000.0
    return value if math.isfinite(value) and value > 0.0 else 2000.0


def _smooth_lah_vertical_altitudes_inplace(waypoints: list[dict]) -> set[int]:
    """Lift adjacent waypoints just enough to respect LAH vertical rates."""

    if not isinstance(waypoints, list) or len(waypoints) < 2:
        return set()
    climb_rate_mps = _lah_vertical_rate_mps(1.0)
    descent_rate_mps = _lah_vertical_rate_mps(-1.0)

    def _altitude(index: int) -> float:
        try:
            value = float((waypoints[index].get("coordinate") or {}).get("altitude", 0.0) or 0.0)
        except Exception:
            value = 0.0
        return value if math.isfinite(value) else 0.0

    def _eta(index: int) -> float:
        try:
            return max(0.0, _eta_to_seconds(waypoints[index].get("eta", 0), assume_ms=False))
        except Exception:
            return 0.0

    changed: set[int] = set()

    # Lift earlier waypoints so every subsequent LOS altitude is climbable.
    for index in range(len(waypoints) - 2, -1, -1):
        delta_t_s = max(0.0, _eta(index + 1) - _eta(index))
        required_m = _altitude(index + 1) - climb_rate_mps * delta_t_s
        if required_m <= _altitude(index) + 1e-6:
            continue
        coordinate = waypoints[index].get("coordinate")
        if isinstance(coordinate, dict):
            coordinate["altitude"] = int(math.ceil(required_m))
            changed.add(id(waypoints[index]))

    # Lift later waypoints when the requested descent would be too steep.
    for index in range(len(waypoints) - 1):
        delta_t_s = max(0.0, _eta(index + 1) - _eta(index))
        required_m = _altitude(index) - descent_rate_mps * delta_t_s
        if required_m <= _altitude(index + 1) + 1e-6:
            continue
        coordinate = waypoints[index + 1].get("coordinate")
        if isinstance(coordinate, dict):
            coordinate["altitude"] = int(math.ceil(required_m))
            changed.add(id(waypoints[index + 1]))
    return changed


def _apply_lah_uav_los_altitudes_inplace(packet: dict, uav_timeline: list[dict]) -> dict:
    """Raise LAH waypoints to the minimum terrain-safe UAV LOS altitude.

    UAV position and altitude are interpolated at each LAH waypoint ETA.  All
    terrain profiles for the packet are loaded in one DEM batch and capped, so
    the LOS pass remains inexpensive even for a long flight path.  Existing
    mission altitudes are never lowered and DEM failures leave the established
    terrain-safe altitude untouched.
    """

    waypoints = packet.get("lahWaypointList") or []
    if not isinstance(waypoints, list) or not waypoints or not uav_timeline:
        return {"applied": 0, "sampleCount": 0, "reason": "missing_path_or_uav_timeline"}

    valid_rows: list[dict] = []
    for waypoint in waypoints:
        coord = _coord_from_wp(waypoint)
        if coord is None:
            continue
        try:
            eta_s = max(0.0, _eta_to_seconds(waypoint.get("eta", 0), assume_ms=False))
        except Exception:
            eta_s = 0.0
        state = _uav_state_at_eta_s(uav_timeline, eta_s)
        if state is None:
            continue
        uav_coord = state[0]
        distance_m = _coord_dist_m(coord, uav_coord)
        valid_rows.append(
            {
                "waypoint": waypoint,
                "lah_coord": coord,
                "uav_coord": uav_coord,
                "distance_m": float(distance_m),
            }
        )
    if not valid_rows:
        return {"applied": 0, "sampleCount": 0, "reason": "no_valid_waypoints"}

    per_waypoint_cap = max(
        1,
        min(
            int(LAH_UAV_LOS_MAX_SAMPLES_PER_WAYPOINT),
            int(LAH_UAV_LOS_MAX_TOTAL_SAMPLES) // max(1, len(valid_rows)),
        ),
    )
    flat_pairs: list[tuple[float, float]] = []
    for row in valid_rows:
        distance_m = float(row["distance_m"])
        if distance_m <= 1.0:
            sample_count = 1
        else:
            sample_count = max(
                min(3, per_waypoint_cap),
                min(
                    per_waypoint_cap,
                    int(math.ceil(distance_m / max(25.0, LAH_UAV_LOS_SAMPLE_STEP_M))) + 1,
                ),
            )
        row["sample_start"] = len(flat_pairs)
        row["sample_count"] = int(sample_count)
        lah_coord = row["lah_coord"]
        uav_coord = row["uav_coord"]
        if sample_count == 1:
            flat_pairs.append((float(lah_coord["latitude"]), float(lah_coord["longitude"])))
        else:
            for index in range(sample_count):
                fraction = float(index) / float(sample_count - 1)
                flat_pairs.append(
                    (
                        float(lah_coord["latitude"])
                        + (float(uav_coord["latitude"]) - float(lah_coord["latitude"])) * fraction,
                        float(lah_coord["longitude"])
                        + (float(uav_coord["longitude"]) - float(lah_coord["longitude"])) * fraction,
                    )
                )
        row["sample_end"] = len(flat_pairs)

    try:
        terrain = [float(value) for value in _terrain_profile_many(flat_pairs)]
    except Exception:
        terrain = []
    if len(terrain) != len(flat_pairs):
        return {"applied": 0, "sampleCount": len(flat_pairs), "reason": "terrain_lookup_failed"}

    changed_waypoint_ids: set[int] = set()
    max_altitude_m = _lah_max_altitude_m()
    for row in valid_rows:
        profile = terrain[int(row["sample_start"]) : int(row["sample_end"])]
        if not profile or any(not math.isfinite(value) for value in profile):
            continue
        waypoint = row["waypoint"]
        lah_coord = row["lah_coord"]
        uav_coord = row["uav_coord"]
        distance_m = float(row["distance_m"])
        try:
            current_altitude_m = float((waypoint.get("coordinate") or {}).get("altitude", 0.0) or 0.0)
        except Exception:
            current_altitude_m = 0.0
        terrain_floor_m = float(profile[0]) + float(LAH_NON_ATTACK_CLEARANCE_M)
        required_altitude_m = max(current_altitude_m, terrain_floor_m)

        if len(profile) >= 3 and distance_m > 1.0:
            try:
                uav_altitude_m = float(uav_coord.get("altitude", 0.0) or 0.0)
            except Exception:
                uav_altitude_m = 0.0
            # A malformed UAV altitude must not place the LOS endpoint below
            # the DEM.  Normal planned UAV altitudes pass through unchanged.
            uav_altitude_m = max(
                uav_altitude_m if math.isfinite(uav_altitude_m) else 0.0,
                float(profile[-1]) + float(LAH_UAV_LOS_CLEARANCE_M),
            )
            denominator = float(len(profile) - 1)
            for index in range(0, len(profile) - 1):
                fraction = float(index) / denominator
                remaining = 1.0 - fraction
                if remaining <= 1e-6:
                    continue
                along_m = fraction * distance_m
                earth_bulge_m = along_m * (distance_m - along_m) / (2.0 * 6_371_000.0)
                terrain_requirement_m = (
                    float(profile[index])
                    + float(LAH_UAV_LOS_CLEARANCE_M)
                    + earth_bulge_m
                )
                origin_requirement_m = (
                    terrain_requirement_m - fraction * uav_altitude_m
                ) / remaining
                required_altitude_m = max(required_altitude_m, origin_requirement_m)

        capped_los_altitude_m = min(required_altitude_m, max_altitude_m)
        # Collision safety wins if local terrain itself is above the nominal
        # operational ceiling; never cap below the established AGL floor.
        final_altitude_m = max(current_altitude_m, terrain_floor_m, capped_los_altitude_m)
        coordinate = waypoint.get("coordinate")
        if isinstance(coordinate, dict):
            normalized_altitude = int(math.ceil(final_altitude_m))
            if normalized_altitude != int(round(current_altitude_m)):
                coordinate["altitude"] = normalized_altitude
                changed_waypoint_ids.add(id(waypoint))

    changed_waypoint_ids.update(_smooth_lah_vertical_altitudes_inplace(waypoints))

    return {
        "applied": int(len(changed_waypoint_ids)),
        "sampleCount": int(len(flat_pairs)),
        "reason": "ok",
    }


def _apply_lah_uav_los_altitude_plan(
    lah_packets: List[dict],
    paired_uav_packet_by_lah: dict[int, dict],
    *,
    immutable_path_ids: set[int],
    relay_uav_packets_by_lah: dict[int, list[dict]] | None = None,
) -> None:
    """Raise LAH altitudes for UAV line of sight.

    A wingman only has to see the UAV it is paired with.  The command aircraft
    relays for the whole team, so when ``relay_uav_packets_by_lah`` names its
    UAVs the pass runs once per UAV.  The routine never lowers an altitude, so
    running it repeatedly accumulates the maximum requirement across them.
    """

    for packet in lah_packets or []:
        try:
            if int(packet.get("pathID", 0) or 0) in immutable_path_ids:
                continue
        except Exception:
            pass
        relay_packets = (relay_uav_packets_by_lah or {}).get(id(packet)) or []
        if relay_packets:
            for uav_packet in relay_packets:
                if isinstance(uav_packet, dict):
                    _apply_lah_uav_los_altitudes_inplace(
                        packet,
                        _uav_timeline_from_packet(uav_packet),
                    )
            continue
        uav_packet = paired_uav_packet_by_lah.get(id(packet))
        if not isinstance(uav_packet, dict):
            continue
        _apply_lah_uav_los_altitudes_inplace(
            packet,
            _uav_timeline_from_packet(uav_packet),
        )


def _positive_waypoint_speed_mps(waypoint: dict, default: float = LAH_DEFAULT_CRUISE_SPEED_MPS) -> float:
    try:
        speed_mps = float(waypoint.get("speed", default) or default)
    except Exception:
        speed_mps = float(default)
    if not math.isfinite(speed_mps) or speed_mps <= 0.0:
        speed_mps = float(default)
    return max(1.0, speed_mps)


def _lah_max_speed_mps() -> float:
    max_speed_mps = 75.0
    try:
        if DEFAULT_ENVELOPE is not None:
            max_speed_mps = float(getattr(DEFAULT_ENVELOPE, "max_speed_kmh", 270.0)) / 3.6
    except Exception:
        max_speed_mps = 75.0
    if not math.isfinite(max_speed_mps) or max_speed_mps <= 0.0:
        max_speed_mps = 75.0
    return max(1.0, float(max_speed_mps))


def _terminal_support_coordinate_for_uav_timeline(
    terminal_waypoint: dict,
    uav_timeline: list[dict],
    *,
    support_departure_coord: dict | None = None,
    support_departure_eta_s: float | None = None,
    fixed_arrival_eta_s: float | None = None,
    support_speed_mps: float | None = None,
) -> tuple[dict | None, float, float]:
    """Return a support coordinate near the UAV at the LAH's predicted arrival.

    The arrival-time solve includes the changed terminal leg instead of using
    the old endpoint ETA.  A 2 km margin keeps integer ETA/coordinate rounding
    away from the 20 km operational limit.
    """

    terminal_coord = _coord_from_wp(terminal_waypoint)
    if terminal_coord is None or not uav_timeline:
        return None, 0.0, 0.0
    try:
        base_eta_s = max(0.0, _eta_to_seconds(terminal_waypoint.get("eta", 0), assume_ms=False))
    except Exception:
        base_eta_s = 0.0
    initial_state = _uav_state_at_eta_s(uav_timeline, base_eta_s)
    if initial_state is None:
        return None, 0.0, 0.0
    initial_gap_m = _coord_dist_m(terminal_coord, initial_state[0])
    max_distance_m = max(1.0, float(LAH_UAV_TERMINAL_MAX_DISTANCE_M))
    if initial_gap_m <= max_distance_m:
        return None, float(initial_gap_m), float(initial_gap_m)

    target_distance_m = min(
        max_distance_m - 1.0,
        max(0.0, float(LAH_UAV_TERMINAL_TARGET_DISTANCE_M)),
    )
    speed_mps = (
        _positive_waypoint_speed_mps(terminal_waypoint)
        if support_speed_mps is None
        else max(1.0, float(support_speed_mps))
    )
    departure_coord = support_departure_coord or terminal_coord
    departure_eta_s = (
        float(base_eta_s)
        if support_departure_eta_s is None
        else max(0.0, float(support_departure_eta_s))
    )
    def _projected_support_at(eta_s: float) -> dict:
        state = _uav_state_at_eta_s(uav_timeline, eta_s)
        if state is None:
            return dict(terminal_coord)
        uav_coord = state[0]
        gap_m = _coord_dist_m(uav_coord, terminal_coord)
        if gap_m <= target_distance_m or gap_m <= 1e-6:
            return dict(terminal_coord)
        return _coord_between(
            uav_coord,
            terminal_coord,
            target_distance_m / gap_m,
        )

    if fixed_arrival_eta_s is not None:
        arrival_eta_s = max(0.0, float(fixed_arrival_eta_s))
    else:
        def _arrival_residual(eta_s: float) -> float:
            candidate = _projected_support_at(eta_s)
            physical_arrival_s = departure_eta_s + _coord_dist_m(departure_coord, candidate) / speed_mps
            return physical_arrival_s - float(eta_s)

        lower_eta_s = float(departure_eta_s)
        upper_eta_s = max(
            lower_eta_s + 60.0,
            float(base_eta_s) + 1.0,
            float(uav_timeline[-1].get("eta_s", 0.0) or 0.0) + 1.0,
        )
        residual_lower = _arrival_residual(lower_eta_s)
        residual_upper = _arrival_residual(upper_eta_s)
        expansion_s = max(60.0, upper_eta_s - lower_eta_s)
        for _ in range(32):
            if residual_upper <= 0.0:
                break
            upper_eta_s = min(float(UINT32_MAX), upper_eta_s + expansion_s)
            expansion_s *= 2.0
            residual_upper = _arrival_residual(upper_eta_s)
        if residual_lower <= 0.0:
            arrival_eta_s = lower_eta_s
        elif residual_upper > 0.0:
            arrival_eta_s = upper_eta_s
        else:
            for _ in range(max(8, int(LAH_UAV_TERMINAL_SOLVER_ITERATIONS))):
                middle_eta_s = (lower_eta_s + upper_eta_s) / 2.0
                if _arrival_residual(middle_eta_s) > 0.0:
                    lower_eta_s = middle_eta_s
                else:
                    upper_eta_s = middle_eta_s
            arrival_eta_s = (lower_eta_s + upper_eta_s) / 2.0

    support_coord = _projected_support_at(arrival_eta_s)

    # One final projection at the rounded arrival instant absorbs ETA integer
    # rounding and a moving UAV without putting the output on the 20 km edge.
    final_state = _uav_state_at_eta_s(uav_timeline, _clamp_eta_seconds(arrival_eta_s))
    if final_state is not None:
        final_uav_coord = final_state[0]
        final_origin_gap_m = _coord_dist_m(final_uav_coord, terminal_coord)
        if final_origin_gap_m <= target_distance_m or final_origin_gap_m <= 1e-6:
            support_coord = dict(terminal_coord)
        else:
            support_coord = _coord_between(
                final_uav_coord,
                terminal_coord,
                target_distance_m / final_origin_gap_m,
            )
        final_gap_m = _coord_dist_m(support_coord, final_uav_coord)
    else:
        final_gap_m = initial_gap_m

    support_coord["latitude"] = round(float(support_coord["latitude"]), 6)
    support_coord["longitude"] = round(float(support_coord["longitude"]), 6)
    support_coord["altitude"] = int(terminal_coord.get("altitude", 0) or 0)
    return support_coord, float(initial_gap_m), float(final_gap_m)


def _terrain_profile_between_lah_coords(start_coord: dict, end_coord: dict) -> list[dict]:
    try:
        profile = build_lah_terrain_following_path(
            [start_coord, end_coord],
            clearance_m=LAH_NON_ATTACK_CLEARANCE_M,
            terrain_provider=_terrain_profile_many,
            # UAV-support/continuity legs have no LINE/AREA geometry.  Keep
            # their horizontal course instead of making an unbounded detour.
            prefer_low_terrain=False,
        )
    except Exception:
        profile = []
    if len(profile) >= 2:
        profile[0]["latitude"] = float(start_coord["latitude"])
        profile[0]["longitude"] = float(start_coord["longitude"])
        profile[-1]["latitude"] = float(end_coord["latitude"])
        profile[-1]["longitude"] = float(end_coord["longitude"])
        return [dict(sample) for sample in profile]

    try:
        end_altitude = _lah_non_attack_altitude_m(
            float(end_coord["latitude"]),
            float(end_coord["longitude"]),
        )
    except Exception:
        end_altitude = int(end_coord.get("altitude", start_coord.get("altitude", 0)) or 0)
    return [
        {
            "latitude": float(start_coord["latitude"]),
            "longitude": float(start_coord["longitude"]),
            "altitude": int(start_coord.get("altitude", 0) or 0),
            "cum_m": 0.0,
        },
        {
            "latitude": float(end_coord["latitude"]),
            "longitude": float(end_coord["longitude"]),
            "altitude": int(end_altitude),
            "cum_m": _coord_dist_m(start_coord, end_coord),
        },
    ]


def _reserve_lah_waypoint_ids(count: int) -> list[int]:
    requested = max(0, int(count))
    if requested <= 0:
        return []
    first_id = int(_reserve_waypoint_block(requested))
    return [first_id + offset for offset in range(requested)]


def _make_lah_transit_waypoint(sample: dict, *, speed_mps: float, waypoint_id: int) -> OrderedDict:
    return OrderedDict(
        [
            ("waypointID", int(waypoint_id)),
            ("isDone", False),
            (
                "coordinate",
                {
                    "latitude": round(float(sample["latitude"]), 6),
                    "longitude": round(float(sample["longitude"]), 6),
                    "altitude": int(round(float(sample.get("altitude", 0) or 0))),
                },
            ),
            ("speed", round(float(speed_mps), 2)),
            ("eta", 0),
            ("ecf", 0.0),
            ("nextWaypointID", 0),
        ]
    )


def _repair_lah_waypoint_chain_inplace(packet: dict) -> None:
    waypoints = packet.get("lahWaypointList") or []
    for index, waypoint in enumerate(waypoints):
        waypoint["nextWaypointID"] = (
            int(waypoints[index + 1].get("waypointID", 0) or 0)
            if index + 1 < len(waypoints)
            else 0
        )


def _recompute_lah_ecf_inplace(packet: dict) -> None:
    waypoints = packet.get("lahWaypointList") or []
    if not isinstance(waypoints, list) or not waypoints:
        return
    leg_lengths = [0.0]
    for left, right in zip(waypoints, waypoints[1:]):
        left_coord = _coord_from_wp(left)
        right_coord = _coord_from_wp(right)
        leg_lengths.append(
            _coord_dist_m(left_coord, right_coord)
            if left_coord is not None and right_coord is not None
            else 0.0
        )
    # ICD ecf is litres burned on the leg into each waypoint, not progress.
    from modules.common.ecf import apply_leg_fuel_inplace

    apply_leg_fuel_inplace(waypoints)


def _finalize_modified_lah_packet_inplace(packet: dict) -> None:
    _recompute_lah_eta_inplace(packet)
    _recompute_lah_ecf_inplace(packet)
    _normalize_lah_waypoint_list_inplace(packet.get("lahWaypointList") or [])
    _repair_lah_waypoint_chain_inplace(packet)


def _prepend_reported_lah_start_altitude_inplace(
    waypoints: list[dict],
    reported_start: dict | None,
) -> bool:
    """Keep the live altitude as ETA=0 and schedule any vertical transition."""

    if not isinstance(waypoints, list) or not waypoints or not isinstance(reported_start, dict):
        return False
    if "altitude" not in reported_start:
        return False
    first_coord = _coord_from_wp(waypoints[0])
    try:
        live_coord = {
            "latitude": float(reported_start["latitude"]),
            "longitude": float(reported_start["longitude"]),
            "altitude": int(round(float(reported_start["altitude"]))),
        }
    except Exception:
        return False
    if first_coord is None or _coord_dist_m(first_coord, live_coord) > 1.0:
        return False
    if int(first_coord.get("altitude", 0)) == int(live_coord["altitude"]):
        return False

    first_speed_mps = _positive_waypoint_speed_mps(waypoints[0])
    live_waypoint = _make_lah_transit_waypoint(
        live_coord,
        speed_mps=first_speed_mps,
        waypoint_id=0,
    )
    packet = {"lahWaypointList": [live_waypoint] + waypoints}
    _finalize_modified_lah_packet_inplace(packet)
    waypoints[:] = packet["lahWaypointList"]
    return True


def _waypoint_has_meaningful_action(waypoint: dict) -> bool:
    attack = waypoint.get("attack") if isinstance(waypoint, dict) else None
    if isinstance(attack, dict):
        try:
            if int(attack.get("targetID", 0) or 0) > 0 or int(attack.get("weaponType", 0) or 0) > 0:
                return True
        except Exception:
            return True
    if isinstance(waypoint.get("filmingProperty"), dict):
        return True
    for key in ("hovering", "loiter"):
        action = waypoint.get(key)
        if isinstance(action, dict):
            try:
                if float(action.get("time", 0) or 0) > 0.0:
                    return True
            except Exception:
                return True
    return False


def _terminal_has_nonrelocatable_action(waypoint: dict) -> bool:
    attack = waypoint.get("attack") if isinstance(waypoint, dict) else None
    if isinstance(attack, dict):
        try:
            if int(attack.get("targetID", 0) or 0) > 0 or int(attack.get("weaponType", 0) or 0) > 0:
                return True
        except Exception:
            return True
    return isinstance(waypoint.get("filmingProperty"), dict)


def _harmonize_lah_packet_boundary_altitudes_inplace(
    packets: List[dict],
    *,
    immutable_path_ids: set[int] | None = None,
) -> int:
    """Remove zero-time altitude jumps between consecutive LAH packets.

    The first ETA of every 0304 packet remains zero by contract.  Therefore a
    vertical transition at a shared mission boundary belongs either at the end
    of the previous packet or immediately after an inserted ETA=0 anchor in the
    next packet.  Attack/filming waypoints are never relocated.
    """

    immutable_ids = {int(value) for value in (immutable_path_ids or set())}
    previous_by_aircraft: dict[int, dict] = {}
    changed_packets: dict[int, dict] = {}
    inserted_count = 0

    def _new_transit_waypoint(reference: dict, coordinate: dict) -> OrderedDict:
        try:
            reference_id = int(reference.get("waypointID", 0) or 0)
        except Exception:
            reference_id = 0
        waypoint_id = _reserve_lah_waypoint_ids(1)[0] if reference_id > 0 else 0
        return _make_lah_transit_waypoint(
            coordinate,
            speed_mps=_positive_waypoint_speed_mps(reference),
            waypoint_id=waypoint_id,
        )

    for packet in packets or []:
        if not isinstance(packet, dict):
            continue
        try:
            aircraft_id = int(packet.get("aircraftID", 0) or 0)
            path_id = int(packet.get("pathID", 0) or 0)
        except Exception:
            continue
        previous = previous_by_aircraft.get(aircraft_id)
        previous_by_aircraft[aircraft_id] = packet
        if previous is None or path_id in immutable_ids:
            continue
        try:
            previous_path_id = int(previous.get("pathID", 0) or 0)
        except Exception:
            previous_path_id = 0
        if previous_path_id in immutable_ids:
            continue
        previous_waypoints = previous.get("lahWaypointList") or []
        current_waypoints = packet.get("lahWaypointList") or []
        if not previous_waypoints or not current_waypoints:
            continue
        previous_terminal = previous_waypoints[-1]
        current_first = current_waypoints[0]
        previous_coord = _coord_from_wp(previous_terminal)
        current_coord = _coord_from_wp(current_first)
        if (
            previous_coord is None
            or current_coord is None
            or _coord_dist_m(previous_coord, current_coord) > 1.0
        ):
            continue
        shared_altitude = max(
            int(previous_coord.get("altitude", 0) or 0),
            int(current_coord.get("altitude", 0) or 0),
        )
        previous_altitude = int(previous_coord.get("altitude", 0) or 0)
        current_altitude = int(current_coord.get("altitude", 0) or 0)
        if previous_altitude == current_altitude:
            continue

        shared_coord = {
            "latitude": float(previous_coord["latitude"]),
            "longitude": float(previous_coord["longitude"]),
            "altitude": int(shared_altitude),
        }
        if previous_altitude < shared_altitude:
            if _terminal_has_nonrelocatable_action(previous_terminal):
                previous_waypoints.append(_new_transit_waypoint(previous_terminal, shared_coord))
                inserted_count += 1
            else:
                previous_terminal["coordinate"]["altitude"] = int(shared_altitude)
            changed_packets[id(previous)] = previous

        if current_altitude < shared_altitude:
            if _terminal_has_nonrelocatable_action(current_first):
                current_waypoints.insert(0, _new_transit_waypoint(current_first, shared_coord))
                inserted_count += 1
            else:
                current_first["coordinate"]["altitude"] = int(shared_altitude)
            changed_packets[id(packet)] = packet

    for packet in changed_packets.values():
        _finalize_modified_lah_packet_inplace(packet)
    return int(inserted_count)


def _lah_path_is_stationary(waypoints: list[dict]) -> bool:
    if not waypoints:
        return False
    anchor = _coord_from_wp(waypoints[0])
    if anchor is None:
        return False
    for waypoint in waypoints[1:]:
        coord = _coord_from_wp(waypoint)
        if coord is None or _coord_dist_m(anchor, coord) > 1.0:
            return False
    return True


def _replace_lah_terminal_support_route_inplace(
    packet: dict,
    *,
    support_coord: dict,
    initial_gap_m: float,
    final_gap_m: float,
    reposition_speed_mps: float,
) -> tuple[bool, dict | None, dict | None, float, float]:
    waypoints = packet.get("lahWaypointList") or []
    if not isinstance(waypoints, list) or not waypoints:
        return False, None, None, 0.0, 0.0
    terminal_waypoint = waypoints[-1]
    original_terminal_coord = _coord_from_wp(terminal_waypoint)
    if original_terminal_coord is None:
        return False, original_terminal_coord, original_terminal_coord, initial_gap_m, final_gap_m

    speed_mps = max(1.0, float(reposition_speed_mps))
    if _lah_path_is_stationary(waypoints) and len(waypoints) == 1:
        profile = _terrain_profile_between_lah_coords(original_terminal_coord, support_coord)
        support_altitude = int(round(float(profile[-1].get("altitude", 0) or 0))) if profile else int(
            support_coord.get("altitude", 0) or 0
        )
        terminal_waypoint["coordinate"] = {
            "latitude": round(float(support_coord["latitude"]), 6),
            "longitude": round(float(support_coord["longitude"]), 6),
            "altitude": int(support_altitude),
        }
        terminal_waypoint["speed"] = round(speed_mps, 2)
        _finalize_modified_lah_packet_inplace(packet)
        return True, original_terminal_coord, _coord_from_wp(waypoints[-1]), initial_gap_m, final_gap_m

    if len(waypoints) < 2:
        return False, original_terminal_coord, original_terminal_coord, initial_gap_m, final_gap_m
    anchor_waypoint = waypoints[-2]
    anchor_coord = _coord_from_wp(anchor_waypoint)
    if anchor_coord is None:
        return False, original_terminal_coord, original_terminal_coord, initial_gap_m, final_gap_m
    profile = _terrain_profile_between_lah_coords(anchor_coord, support_coord)
    if len(profile) < 2:
        return False, original_terminal_coord, original_terminal_coord, initial_gap_m, final_gap_m

    replacement_samples: list[dict] = []
    anchor_altitude = int(anchor_coord.get("altitude", 0) or 0)
    safe_departure_altitude = max(anchor_altitude, int(round(float(profile[0].get("altitude", 0) or 0))))
    if safe_departure_altitude != anchor_altitude:
        replacement_samples.append(
            {
                "latitude": float(anchor_coord["latitude"]),
                "longitude": float(anchor_coord["longitude"]),
                "altitude": int(safe_departure_altitude),
            }
        )
    replacement_samples.extend(dict(sample) for sample in profile[1:])
    if not replacement_samples:
        return False, original_terminal_coord, original_terminal_coord, initial_gap_m, final_gap_m

    try:
        reusable_terminal_id = int(terminal_waypoint.get("waypointID", 0) or 0)
    except Exception:
        reusable_terminal_id = 0
    allocated_ids = iter(
        _reserve_lah_waypoint_ids(
            len(replacement_samples) - (1 if reusable_terminal_id > 0 else 0)
        )
    )
    replacement_ids = [
        reusable_terminal_id if index == len(replacement_samples) - 1 and reusable_terminal_id > 0 else next(allocated_ids)
        for index in range(len(replacement_samples))
    ]
    replacement_waypoints = [
        _make_lah_transit_waypoint(sample, speed_mps=speed_mps, waypoint_id=waypoint_id)
        for sample, waypoint_id in zip(replacement_samples, replacement_ids)
    ]
    for action_key in ("hovering", "loiter"):
        if action_key in terminal_waypoint:
            replacement_waypoints[-1][action_key] = deepcopy(terminal_waypoint.get(action_key))
    packet["lahWaypointList"] = waypoints[:-1] + replacement_waypoints
    _finalize_modified_lah_packet_inplace(packet)
    return (
        True,
        original_terminal_coord,
        _coord_from_wp(packet["lahWaypointList"][-1]),
        initial_gap_m,
        final_gap_m,
    )


def _append_lah_terminal_support_route_inplace(
    packet: dict,
    *,
    support_coord: dict,
    initial_gap_m: float,
    final_gap_m: float,
    reposition_speed_mps: float,
) -> tuple[bool, dict | None, dict | None, float, float]:
    """Keep an actionable terminal, then leave it for the constrained support point."""

    waypoints = packet.get("lahWaypointList") or []
    if not isinstance(waypoints, list) or not waypoints:
        return False, None, None, 0.0, 0.0
    terminal_waypoint = waypoints[-1]
    original_terminal_coord = _coord_from_wp(terminal_waypoint)
    if original_terminal_coord is None:
        return False, None, None, initial_gap_m, final_gap_m
    speed_mps = max(1.0, float(reposition_speed_mps))
    profile = _terrain_profile_between_lah_coords(original_terminal_coord, support_coord)
    if len(profile) < 2:
        return False, original_terminal_coord, original_terminal_coord, initial_gap_m, final_gap_m
    samples_to_add = [dict(sample) for sample in profile[1:]]
    if not samples_to_add:
        return False, original_terminal_coord, original_terminal_coord, initial_gap_m, final_gap_m
    waypoint_ids = _reserve_lah_waypoint_ids(len(samples_to_add))
    transit_waypoints = [
        _make_lah_transit_waypoint(sample, speed_mps=speed_mps, waypoint_id=waypoint_id)
        for sample, waypoint_id in zip(samples_to_add, waypoint_ids)
    ]
    for action_key in ("hovering", "loiter"):
        if action_key in terminal_waypoint:
            transit_waypoints[-1][action_key] = deepcopy(terminal_waypoint.get(action_key))
            terminal_waypoint.pop(action_key, None)
    packet["lahWaypointList"] = waypoints + transit_waypoints
    _finalize_modified_lah_packet_inplace(packet)
    return True, original_terminal_coord, _coord_from_wp(packet["lahWaypointList"][-1]), initial_gap_m, final_gap_m


def _apply_lah_terminal_support_route_inplace(
    packet: dict,
    uav_timeline: list[dict],
    *,
    constraint_info: dict | None = None,
) -> tuple[bool, dict | None, dict | None, float, float]:
    waypoints = packet.get("lahWaypointList") or []
    if not isinstance(waypoints, list) or not waypoints:
        return False, None, None, 0.0, 0.0
    terminal_waypoint = waypoints[-1]
    original_terminal_coord = _coord_from_wp(terminal_waypoint)
    preserve_terminal_action = _terminal_has_nonrelocatable_action(terminal_waypoint)
    departure_coord = None
    departure_eta_s = None
    reposition_speed_mps = _lah_max_speed_mps()
    if not preserve_terminal_action and not _lah_path_is_stationary(waypoints) and len(waypoints) >= 2:
        departure_coord = _coord_from_wp(waypoints[-2])
        try:
            departure_eta_s = float(waypoints[-2].get("eta", 0) or 0)
        except Exception:
            departure_eta_s = 0.0
    support_coord, initial_gap_m, final_gap_m = _terminal_support_coordinate_for_uav_timeline(
        terminal_waypoint,
        uav_timeline,
        support_departure_coord=departure_coord,
        support_departure_eta_s=departure_eta_s,
        fixed_arrival_eta_s=(
            float(terminal_waypoint.get("eta", 0) or 0)
            if _lah_path_is_stationary(waypoints)
            else None
        ),
        support_speed_mps=reposition_speed_mps,
    )
    if original_terminal_coord is None or support_coord is None:
        return False, original_terminal_coord, original_terminal_coord, initial_gap_m, final_gap_m
    if isinstance(constraint_info, dict):
        support_coord = _constrain_lah_terminal_support_coordinate(
            constraint_info,
            support_coord,
            original_terminal_coord,
        )
    if _coord_dist_m(original_terminal_coord, support_coord) <= 1.0:
        return False, original_terminal_coord, original_terminal_coord, initial_gap_m, final_gap_m
    helper = (
        _append_lah_terminal_support_route_inplace
        if preserve_terminal_action
        else _replace_lah_terminal_support_route_inplace
    )
    return helper(
        packet,
        support_coord=support_coord,
        initial_gap_m=initial_gap_m,
        final_gap_m=final_gap_m,
        reposition_speed_mps=reposition_speed_mps,
    )


def _maximize_lah_packet_speed_when_initially_far(packet: dict, uav_timeline: list[dict]) -> bool:
    waypoints = packet.get("lahWaypointList") or []
    if not isinstance(waypoints, list) or not waypoints or not uav_timeline:
        return False
    first_coord = _coord_from_wp(waypoints[0])
    if first_coord is None:
        return False
    try:
        first_eta_s = float(waypoints[0].get("eta", 0) or 0)
    except Exception:
        first_eta_s = 0.0
    uav_state = _uav_state_at_eta_s(uav_timeline, first_eta_s)
    if uav_state is None or _coord_dist_m(first_coord, uav_state[0]) <= float(
        LAH_UAV_TERMINAL_MAX_DISTANCE_M
    ):
        return False
    max_speed_mps = round(_lah_max_speed_mps(), 2)
    for waypoint in waypoints:
        waypoint["speed"] = float(max_speed_mps)
    _finalize_modified_lah_packet_inplace(packet)
    return True


def _prepend_lah_continuity_route_inplace(
    packet: dict,
    *,
    previous_original_terminal: dict,
    previous_support_terminal: dict,
) -> bool:
    waypoints = packet.get("lahWaypointList") or []
    if not isinstance(waypoints, list) or not waypoints:
        return False
    first_coord = _coord_from_wp(waypoints[0])
    if first_coord is None or _coord_dist_m(first_coord, previous_original_terminal) > float(
        LAH_UAV_CONTINUITY_MATCH_TOLERANCE_M
    ):
        return False
    if _coord_dist_m(previous_support_terminal, first_coord) <= 1.0:
        return False

    if _lah_path_is_stationary(waypoints) and not any(
        _terminal_has_nonrelocatable_action(waypoint) for waypoint in waypoints
    ):
        for waypoint in waypoints:
            waypoint["coordinate"] = {
                "latitude": round(float(previous_support_terminal["latitude"]), 6),
                "longitude": round(float(previous_support_terminal["longitude"]), 6),
                "altitude": int(previous_support_terminal.get("altitude", 0) or 0),
            }
        _finalize_modified_lah_packet_inplace(packet)
        return True

    drop_original_prefix = len(waypoints) >= 2 and not _waypoint_has_meaningful_action(waypoints[0])
    join_index = 1 if drop_original_prefix else 0
    join_coord = _coord_from_wp(waypoints[join_index])
    if join_coord is None:
        return False
    profile = _terrain_profile_between_lah_coords(previous_support_terminal, join_coord)
    if len(profile) < 2:
        return False

    prefix_samples: list[dict] = [
        {
            "latitude": float(previous_support_terminal["latitude"]),
            "longitude": float(previous_support_terminal["longitude"]),
            "altitude": int(previous_support_terminal.get("altitude", 0) or 0),
        }
    ]
    safe_start_altitude = max(
        int(previous_support_terminal.get("altitude", 0) or 0),
        int(round(float(profile[0].get("altitude", 0) or 0))),
    )
    if safe_start_altitude != int(previous_support_terminal.get("altitude", 0) or 0):
        prefix_samples.append(
            {
                "latitude": float(previous_support_terminal["latitude"]),
                "longitude": float(previous_support_terminal["longitude"]),
                "altitude": int(safe_start_altitude),
            }
        )
    prefix_samples.extend(dict(sample) for sample in profile[1:-1])

    reusable_id = 0
    if drop_original_prefix:
        try:
            reusable_id = int(waypoints[0].get("waypointID", 0) or 0)
        except Exception:
            reusable_id = 0
    ids_needed = len(prefix_samples) - (1 if reusable_id > 0 else 0)
    allocated_ids = iter(_reserve_lah_waypoint_ids(ids_needed))
    prefix_ids: list[int] = []
    for index in range(len(prefix_samples)):
        if index == 0 and reusable_id > 0:
            prefix_ids.append(reusable_id)
        else:
            prefix_ids.append(next(allocated_ids))

    speed_mps = _positive_waypoint_speed_mps(waypoints[min(join_index, len(waypoints) - 1)])
    prefix_waypoints = [
        _make_lah_transit_waypoint(sample, speed_mps=speed_mps, waypoint_id=waypoint_id)
        for sample, waypoint_id in zip(prefix_samples, prefix_ids)
    ]
    packet["lahWaypointList"] = prefix_waypoints + waypoints[join_index:]
    _finalize_modified_lah_packet_inplace(packet)
    return True


def _relocate_lah_terminal_to_cover_inplace(
    packet: dict,
    info: dict,
    uav_timeline: list[dict],
) -> tuple[bool, dict | None, dict | None]:
    """Refine one preselected AREA-internal cover point with ETA UAV LOS."""

    if not callable(select_lah_terminal_cover_point):
        return False, None, None
    area_rows = _lah_terminal_cover_area_rows(info)
    waypoints = packet.get("lahWaypointList") or []
    if not area_rows or not isinstance(waypoints, list) or not waypoints:
        return False, None, None
    terminal_waypoint = waypoints[-1]
    original_terminal = _coord_from_wp(terminal_waypoint)
    if original_terminal is None:
        return False, None, None
    try:
        terminal_eta_s = max(
            0.0,
            _eta_to_seconds(terminal_waypoint.get("eta", 0), assume_ms=False),
        )
    except Exception:
        terminal_eta_s = 0.0
    uav_state = _uav_state_at_eta_s(uav_timeline, terminal_eta_s)
    uav_coord = uav_state[0] if uav_state is not None else None
    threats = info.get("_lahTerminalCoverThreatCoordinateList")
    if not isinstance(threats, list):
        threats = []
    try:
        selected, _diagnostics = select_lah_terminal_cover_point(
            area_rows,
            original_terminal,
            threat_coordinates=threats,
            uav_coordinate=uav_coord,
            terrain_elev_many_fn=_terrain_profile_many,
            hold_agl_m=float(LAH_NON_ATTACK_CLEARANCE_M),
            boundary_margin_ratio=float(LAH_LOW_TERRAIN_AREA_MARGIN_RATIO),
            boundary_margin_max_m=float(LAH_LOW_TERRAIN_AREA_MARGIN_MAX_M),
            max_candidates=25,
            max_threats=5,
            max_ray_samples=48,
            max_uav_distance_m=float(LAH_UAV_TERMINAL_MAX_DISTANCE_M),
        )
    except Exception:
        return False, original_terminal, original_terminal
    if not isinstance(selected, dict):
        return False, original_terminal, original_terminal
    selected = _constrain_lah_terminal_support_coordinate(info, selected, original_terminal)

    # The replacement helper connects from the penultimate waypoint.  Check
    # that actual leg too so a valid endpoint cannot cut across an AREA Hole.
    if len(waypoints) >= 2:
        ingress = _coord_from_wp(waypoints[-2])
        checker = _mission_low_terrain_segment_checker(info)
        if ingress is not None and callable(checker):
            try:
                if not bool(
                    checker(
                        (float(ingress["latitude"]), float(ingress["longitude"])),
                        (float(selected["latitude"]), float(selected["longitude"])),
                    )
                ):
                    selected = dict(original_terminal)
            except Exception:
                selected = dict(original_terminal)
    if _coord_dist_m(original_terminal, selected) <= 1.0:
        return False, original_terminal, original_terminal

    changed, old_terminal, new_terminal, _initial_gap, _final_gap = (
        _replace_lah_terminal_support_route_inplace(
            packet,
            support_coord=selected,
            initial_gap_m=(
                _coord_dist_m(original_terminal, uav_coord)
                if isinstance(uav_coord, dict)
                else 0.0
            ),
            final_gap_m=(
                _coord_dist_m(selected, uav_coord)
                if isinstance(uav_coord, dict)
                else 0.0
            ),
            reposition_speed_mps=_lah_max_speed_mps(),
        )
    )
    return bool(changed), old_terminal, new_terminal


def _apply_lah_terminal_cover_plan(
    lah_packets: List[dict],
    paired_uav_packet_by_lah: dict[int, dict],
    cover_info_by_path: dict[int, dict],
    *,
    immutable_path_ids: set[int],
) -> None:
    """Apply cover/LOS refinement while preserving cross-packet continuity."""

    if not cover_info_by_path:
        return
    packets_by_aircraft: dict[int, list[dict]] = {}
    for packet in lah_packets or []:
        try:
            packets_by_aircraft.setdefault(int(packet.get("aircraftID", 0) or 0), []).append(packet)
        except Exception:
            continue

    for packets in packets_by_aircraft.values():
        previous_original_terminal = None
        previous_cover_terminal = None
        for packet in packets:
            try:
                path_id = int(packet.get("pathID", 0) or 0)
            except Exception:
                path_id = 0
            if path_id in immutable_path_ids:
                previous_original_terminal = None
                previous_cover_terminal = None
                continue
            original_waypoints = packet.get("lahWaypointList") or []
            builder_terminal = (
                _coord_from_wp(original_waypoints[-1])
                if isinstance(original_waypoints, list) and original_waypoints
                else None
            )
            continuity_changed = False
            if previous_original_terminal is not None and previous_cover_terminal is not None:
                continuity_changed = _prepend_lah_continuity_route_inplace(
                    packet,
                    previous_original_terminal=previous_original_terminal,
                    previous_support_terminal=previous_cover_terminal,
                )

            info = cover_info_by_path.get(path_id)
            changed = False
            old_terminal = None
            new_terminal = None
            if isinstance(info, dict):
                uav_packet = paired_uav_packet_by_lah.get(id(packet))
                uav_timeline = (
                    _uav_timeline_from_packet(uav_packet)
                    if isinstance(uav_packet, dict)
                    else []
                )
                changed, old_terminal, new_terminal = _relocate_lah_terminal_to_cover_inplace(
                    packet,
                    info,
                    uav_timeline,
                )

            current_waypoints = packet.get("lahWaypointList") or []
            current_terminal = (
                _coord_from_wp(current_waypoints[-1])
                if isinstance(current_waypoints, list) and current_waypoints
                else None
            )
            if (changed or continuity_changed) and current_terminal is not None:
                previous_original_terminal = builder_terminal or old_terminal
                previous_cover_terminal = current_terminal
            else:
                previous_original_terminal = None
                previous_cover_terminal = None


def _apply_uav_terminal_proximity_plan(
    lah_packets: List[dict],
    paired_uav_packet_by_lah: dict[int, dict],
    *,
    immutable_path_ids: set[int],
    cover_info_by_path: dict[int, dict] | None = None,
) -> None:
    packets_by_aircraft: dict[int, list[dict]] = {}
    for packet in lah_packets or []:
        try:
            aircraft_id = int(packet.get("aircraftID", 0) or 0)
        except Exception:
            continue
        packets_by_aircraft.setdefault(aircraft_id, []).append(packet)

    for packets in packets_by_aircraft.values():
        previous_original_terminal = None
        previous_support_terminal = None
        for packet in packets:
            original_waypoints = packet.get("lahWaypointList") or []
            builder_terminal_coord = (
                _coord_from_wp(original_waypoints[-1])
                if isinstance(original_waypoints, list) and original_waypoints
                else None
            )
            try:
                is_immutable = int(packet.get("pathID", 0) or 0) in immutable_path_ids
            except Exception:
                is_immutable = False
            if is_immutable:
                previous_original_terminal = None
                previous_support_terminal = None
                continue

            if previous_original_terminal is not None and previous_support_terminal is not None:
                _prepend_lah_continuity_route_inplace(
                    packet,
                    previous_original_terminal=previous_original_terminal,
                    previous_support_terminal=previous_support_terminal,
                )

            current_waypoints = packet.get("lahWaypointList") or []
            terminal_after_continuity = (
                _coord_from_wp(current_waypoints[-1])
                if isinstance(current_waypoints, list) and current_waypoints
                else None
            )
            terminal_changed_by_continuity = (
                builder_terminal_coord is not None
                and terminal_after_continuity is not None
                and _coord_dist_m(builder_terminal_coord, terminal_after_continuity) > 1.0
            )

            uav_packet = paired_uav_packet_by_lah.get(id(packet))
            uav_timeline = _uav_timeline_from_packet(uav_packet) if isinstance(uav_packet, dict) else []
            _maximize_lah_packet_speed_when_initially_far(packet, uav_timeline)
            adjusted, original_terminal, support_terminal, _initial_gap, _final_gap = (
                _apply_lah_terminal_support_route_inplace(
                    packet,
                    uav_timeline,
                    constraint_info=(cover_info_by_path or {}).get(
                        int(packet.get("pathID", 0) or 0)
                    ),
                )
            )
            current_waypoints = packet.get("lahWaypointList") or []
            current_terminal = (
                _coord_from_wp(current_waypoints[-1])
                if isinstance(current_waypoints, list) and current_waypoints
                else None
            )
            if (adjusted or terminal_changed_by_continuity) and current_terminal is not None:
                previous_original_terminal = builder_terminal_coord or original_terminal
                previous_support_terminal = current_terminal
            else:
                previous_original_terminal = None
                previous_support_terminal = None


def _follow_speed_bounds(base_cruise_speed: float) -> tuple[float, float]:
    cruise = max(1.0, float(base_cruise_speed))
    env_max_speed = _lah_max_speed_mps()
    min_speed = max(LAH_UAV_FOLLOW_MIN_SPEED_MPS, cruise * LAH_UAV_FOLLOW_MIN_CRUISE_SCALE)
    max_speed = min(env_max_speed, max(cruise * LAH_UAV_FOLLOW_MAX_CRUISE_SCALE, cruise + 10.0))
    if max_speed < min_speed:
        max_speed = min_speed
    return min_speed, max_speed


def _select_follow_arrival_eta_s(
    *,
    prev_eta_s: float,
    base_eta_s: float,
    leg_len_m: float,
    lah_coord: dict,
    uav_timeline: list[dict],
    min_speed_mps: float,
    max_speed_mps: float,
) -> float:
    if leg_len_m < 1e-6:
        return max(prev_eta_s, base_eta_s)
    earliest_eta_s = prev_eta_s + leg_len_m / max(max_speed_mps, 1e-6)
    latest_eta_s = prev_eta_s + leg_len_m / max(min_speed_mps, 1e-6)
    if latest_eta_s < earliest_eta_s:
        latest_eta_s = earliest_eta_s
    candidates = {
        float(earliest_eta_s),
        float(latest_eta_s),
        float(min(max(base_eta_s, earliest_eta_s), latest_eta_s)),
    }
    search_steps = max(3, int(LAH_UAV_FOLLOW_SEARCH_STEPS))
    if latest_eta_s > earliest_eta_s + 1e-6:
        span = latest_eta_s - earliest_eta_s
        for idx in range(search_steps):
            ratio = float(idx) / float(search_steps - 1)
            candidates.add(float(earliest_eta_s + span * ratio))

    best_eta_s = float(min(max(base_eta_s, earliest_eta_s), latest_eta_s))
    best_score = None
    for eta_s in sorted(candidates):
        state = _uav_state_at_eta_s(uav_timeline, eta_s)
        if state is None:
            continue
        uav_coord, heading_xy = state
        gap_dx, gap_dy = _coord_vector_m(lah_coord, uav_coord)
        along_gap_m = gap_dx * heading_xy[0] + gap_dy * heading_xy[1]
        slant_gap_m = math.hypot(gap_dx, gap_dy)
        shortage_m = max(0.0, LAH_UAV_FOLLOW_MIN_GAP_M - along_gap_m)
        overshoot_m = max(0.0, along_gap_m - LAH_UAV_FOLLOW_MAX_GAP_M)
        score = (
            shortage_m * 1000.0
            + overshoot_m * 10.0
            + abs(along_gap_m - LAH_UAV_FOLLOW_TARGET_GAP_M)
            + max(0.0, slant_gap_m - LAH_UAV_FOLLOW_MAX_GAP_M) * 2.0
            + abs(eta_s - base_eta_s) * LAH_UAV_FOLLOW_TIME_PENALTY
        )
        if best_score is None or score < best_score:
            best_score = score
            best_eta_s = float(eta_s)
    return best_eta_s


def apply_uav_eta_follow_speed_plan(
    lah_packets: List[dict],
    uav_packets: List[dict],
    *,
    immutable_path_ids: set[int] | None = None,
    lah_missions: List[dict] | None = None,
) -> List[dict]:
    immutable_ids = {int(value) for value in (immutable_path_ids or set())}
    cover_info_by_path = _lah_terminal_cover_info_by_path(lah_missions)
    uav_by_aircraft: dict[int, list[dict]] = {}
    paired_uav_packet_by_lah: dict[int, dict] = {}
    lah_by_aircraft: dict[int, list[dict]] = {}
    lah_ordinal_by_packet: dict[int, int] = {}
    for pkt in uav_packets or []:
        try:
            aid = int(pkt.get("aircraftID", 0))
        except Exception:
            continue
        if aid > 0:
            uav_by_aircraft.setdefault(aid, []).append(pkt)

    for pkt in lah_packets or []:
        try:
            group = lah_by_aircraft.setdefault(int(pkt.get("aircraftID", 0) or 0), [])
            lah_ordinal_by_packet[id(pkt)] = len(group)
            group.append(pkt)
        except Exception:
            continue

    # Resolve all packet pairs first. Cover-point selection needs the paired UAV
    # ETA timeline before the ordinary per-leg speed pass starts.
    for pkt in lah_packets or []:
        try:
            if int(pkt.get("pathID", 0) or 0) in immutable_ids:
                continue
        except Exception:
            pass
        try:
            lah_aircraft_id = int(pkt.get("aircraftID", 0))
        except Exception:
            continue
        paired_uav_id = int(LAH_UAV_ETA_PAIR_MAP.get(lah_aircraft_id, lah_aircraft_id + 3))
        uav_candidates = uav_by_aircraft.get(paired_uav_id) or []
        lah_candidates = lah_by_aircraft.get(lah_aircraft_id) or []
        uav_pkt = None
        if len(uav_candidates) == len(lah_candidates) and len(uav_candidates) > 0:
            try:
                uav_pkt = uav_candidates[lah_ordinal_by_packet[id(pkt)]]
            except (KeyError, IndexError):
                uav_pkt = None
        if not isinstance(uav_pkt, dict):
            uav_pkt = _pair_uav_packet_for_lah(pkt, uav_candidates)
        if not isinstance(uav_pkt, dict):
            continue
        paired_uav_packet_by_lah[id(pkt)] = uav_pkt

    _apply_lah_terminal_cover_plan(
        lah_packets,
        paired_uav_packet_by_lah,
        cover_info_by_path,
        immutable_path_ids=immutable_ids,
    )

    for pkt in lah_packets or []:
        try:
            if int(pkt.get("pathID", 0) or 0) in immutable_ids:
                continue
        except Exception:
            pass
        uav_pkt = paired_uav_packet_by_lah.get(id(pkt))
        if not isinstance(uav_pkt, dict):
            continue
        uav_timeline = _uav_timeline_from_packet(uav_pkt)
        wplist = pkt.get("lahWaypointList") or []
        if len(uav_timeline) < 2 or not isinstance(wplist, list) or len(wplist) < 2:
            continue

        speed_hints: list[float] = []
        for wp in wplist:
            try:
                speed = float(wp.get("speed", 0.0) or 0.0)
            except Exception:
                speed = 0.0
            if speed > 0.0:
                speed_hints.append(speed)
        base_cruise_speed = sum(speed_hints) / len(speed_hints) if speed_hints else LAH_DEFAULT_CRUISE_SPEED_MPS
        min_speed_mps, max_speed_mps = _follow_speed_bounds(base_cruise_speed)

        first_coord = _coord_from_wp(wplist[0])
        if first_coord is None:
            continue
        prev_coord = first_coord
        try:
            prev_eta_s = max(0.0, _eta_to_seconds(wplist[0].get("eta", 0), assume_ms=False))
        except Exception:
            prev_eta_s = 0.0
        leg_lengths = [0.0]
        first_leg_speed = None

        for idx in range(1, len(wplist)):
            curr_wp = wplist[idx]
            curr_coord = _coord_from_wp(curr_wp)
            if curr_coord is None:
                leg_lengths.append(0.0)
                continue
            leg_len_m = _coord_dist_m(prev_coord, curr_coord)
            leg_lengths.append(leg_len_m)
            try:
                original_speed = float(curr_wp.get("speed", base_cruise_speed) or base_cruise_speed)
            except Exception:
                original_speed = float(base_cruise_speed)
            if original_speed <= 0.0:
                original_speed = float(base_cruise_speed)
            try:
                original_eta_s = _eta_to_seconds(curr_wp.get("eta", 0), assume_ms=False)
            except Exception:
                original_eta_s = prev_eta_s
            base_leg_time_s = _minimum_lah_leg_time_s(
                prev_coord,
                curr_coord,
                original_speed,
            )
            base_eta_s = max(original_eta_s, prev_eta_s + base_leg_time_s)
            desired_eta_s = _select_follow_arrival_eta_s(
                prev_eta_s=prev_eta_s,
                base_eta_s=base_eta_s,
                leg_len_m=leg_len_m,
                lah_coord=curr_coord,
                uav_timeline=uav_timeline,
                min_speed_mps=min_speed_mps,
                max_speed_mps=max_speed_mps,
            )
            min_leg_time_s = _minimum_lah_leg_time_s(
                prev_coord,
                curr_coord,
                max_speed_mps,
            )
            leg_time_s = max(desired_eta_s - prev_eta_s, min_leg_time_s)
            speed_mps = original_speed
            if leg_time_s > 1e-6 and leg_len_m > 1e-6:
                speed_mps = leg_len_m / leg_time_s
            speed_mps = min(max(speed_mps, min_speed_mps), max_speed_mps)
            physical_leg_time_s = _minimum_lah_leg_time_s(
                prev_coord,
                curr_coord,
                speed_mps,
            )
            desired_eta_s = max(
                desired_eta_s,
                prev_eta_s + physical_leg_time_s,
            )
            curr_wp["speed"] = round(float(speed_mps), 2)
            curr_wp["eta"] = _clamp_eta_seconds(desired_eta_s)
            if first_leg_speed is None:
                first_leg_speed = float(curr_wp["speed"])
            prev_eta_s = float(desired_eta_s)
            prev_coord = curr_coord

        if first_leg_speed is not None:
            wplist[0]["speed"] = round(float(first_leg_speed), 2)
        from modules.common.ecf import apply_leg_fuel_inplace

        apply_leg_fuel_inplace(wplist)
        _normalize_lah_waypoint_list_inplace(wplist)
    _sync_lah_packet_speeds_to_leader(
        lah_packets,
        preferred_leader_ids=(1, 2, 3),
        immutable_path_ids=immutable_ids,
    )
    _apply_uav_terminal_proximity_plan(
        lah_packets,
        paired_uav_packet_by_lah,
        immutable_path_ids=immutable_ids,
        cover_info_by_path=cover_info_by_path,
    )
    relay_uav_packets_by_lah: dict[int, list[dict]] = {}
    for pkt in lah_packets or []:
        try:
            if int(pkt.get("aircraftID", 0)) != int(LAH_COMMAND_RELAY_AIRCRAFT_ID):
                continue
        except Exception:
            continue
        ordinal = lah_ordinal_by_packet.get(id(pkt))
        relay_packets: list[dict] = []
        for uav_aircraft_id in sorted(uav_by_aircraft):
            candidates = uav_by_aircraft.get(uav_aircraft_id) or []
            if not candidates:
                continue
            # Prefer the UAV leg that matches this LAH leg; a package with a
            # different leg count falls back to that UAV's first leg.
            chosen = None
            if ordinal is not None and ordinal < len(candidates):
                chosen = candidates[ordinal]
            if not isinstance(chosen, dict):
                chosen = candidates[0]
            if isinstance(chosen, dict):
                relay_packets.append(chosen)
        if len(relay_packets) > 1:
            relay_uav_packets_by_lah[id(pkt)] = relay_packets

    _apply_lah_uav_los_altitude_plan(
        lah_packets,
        paired_uav_packet_by_lah,
        immutable_path_ids=immutable_ids,
        relay_uav_packets_by_lah=relay_uav_packets_by_lah,
    )
    _harmonize_lah_packet_boundary_altitudes_inplace(
        lah_packets,
        immutable_path_ids=immutable_ids,
    )
    for pkt in lah_packets or []:
        try:
            if int(pkt.get("pathID", 0) or 0) in immutable_ids:
                continue
        except Exception:
            pass
        _normalize_lah_waypoint_list_inplace(pkt.get("lahWaypointList") or [])
    return lah_packets

def _validate_lah_flight_plans(pkts: List[dict]) -> None:
    seen_path = set()
    for pidx, pkt in enumerate(pkts, 1):
        aid = pkt["aircraftID"]
        if aid not in (1, 2, 3):
            raise ValueError(f"[0304] pkt#{pidx}: aircraftID must be 1~3")
        path_id = pkt["pathID"]
        if path_id in seen_path:
            raise ValueError(f"[0304] duplicate pathID {path_id}")
        seen_path.add(path_id)
        lo = {1:100_000_001, 2:200_000_001, 3:300_000_001}[aid]
        if not (lo <= path_id < lo+100_000_000):
            raise ValueError(f"[0304] aircraft {aid}: pathID {path_id} out of range")

        previous_eta = 0
        for widx, wp in enumerate(pkt["lahWaypointList"], 1):
            eta = wp.get("eta")
            if not isinstance(eta, int) or isinstance(eta, bool):
                raise ValueError(f"[0304] pkt#{pidx}/wp#{widx}: ETA must be uint32 seconds")
            if not (0 <= eta <= UINT32_MAX):
                raise ValueError(f"[0304] pkt#{pidx}/wp#{widx}: ETA out of uint32 range")
            if widx == 1 and eta != 0:
                raise ValueError(f"[0304] pkt#{pidx}/wp#1: first ETA must be 0 seconds")
            if eta < previous_eta:
                raise ValueError(f"[0304] pkt#{pidx}/wp#{widx}: ETA must be cumulative")
            previous_eta = eta
            atk = wp.get("attack")
            if atk is None:
                continue
            tid = atk.get("targetID")
            if not isinstance(tid, int):
                raise ValueError(f"[0304] pkt#{pidx}/wp#{widx}: targetID must be int")
            if not (0 <= tid <= 0xFFFFFFFF):
                raise ValueError(f"[0304] pkt#{pidx}/wp#{widx}: targetID out of range")


def build_lah_flight_plans_from_mrpk(
    missions: List[dict],
    mrpk: dict,
    *,
    cruise_speed: float = LAH_DEFAULT_CRUISE_SPEED_MPS,
    wp_interval_m: float = WP_INTERVAL_M,
    manned_plan_mode: str = "normal",
    wp_alloc: _WPAllocator | None = None,
) -> List[dict]:
    def _offset(lat: float, lon: float, north_m: float = 0.0, east_m: float = 0.0) -> tuple[float, float]:
        k = 111_132.92
        dlat = north_m / k
        dlon = east_m  / (k * math.cos(math.radians(lat)))
        return (lat + dlat, lon + dlon)

    def _dist_s(a: dict, b: dict) -> int:
        k = 111_132.92
        lat1, lon1 = a["latitude"], a["longitude"]; lat2, lon2 = b["latitude"], b["longitude"]
        cos = math.cos(math.radians((lat1 + lat2)/2))
        dx = (lon2 - lon1) * k * cos; dy = (lat2 - lat1) * k
        m  = math.hypot(dx, dy)
        return _clamp_eta_seconds(m / max(1e-6, cruise_speed))

    def _mk_wp(lat: float, lon: float, alt: float, eta_s: int) -> OrderedDict:
        wp = OrderedDict([
            ("waypointID", 0),
            ("isDone", False),
            ("coordinate", {"latitude": round(lat,6), "longitude": round(lon,6), "altitude": int(round(alt))}),
            ("speed", cruise_speed),
            ("eta",   _clamp_eta_seconds(eta_s)),
            ("ecf",   0.0),
            ("nextWaypointID", 0),
        ])
        return wp

    wp_alloc = wp_alloc or _WPAllocator()
    now_ms   = now_ms_since_2000()

    base_packets = build_lah_flight_plans_fixed(
        missions,
        cruise_speed = cruise_speed,
        wp_interval_m = wp_interval_m,
        manned_plan_mode = manned_plan_mode,
        wp_alloc = _WPAllocator(1),
    )
    attack_path_ids = {
        int(mission.get("pathID"))
        for mission in missions or []
        if isinstance(mission, dict)
        and mission.get("pathID") is not None
        and _is_lah_attack_mission(
            mission,
            mission.get("individualMissionInfo")
            if isinstance(mission.get("individualMissionInfo"), dict)
            else {},
        )
    }

    tk_list = (mrpk or {}).get("takeOverInfoList") or []
    rtb_list= (mrpk or {}).get("rtbCoordinateList") or []

    if tk_list:
        anchor = min(
            [it.get("coordinate", {}) for it in tk_list if it.get("coordinate")],
            key=lambda c: (c.get("latitude", 90), c.get("longitude", 180))
        )
        a_lat, a_lon = float(anchor["latitude"]), float(anchor["longitude"])
    else:
        if base_packets and base_packets[0]["lahWaypointList"]:
            c0 = base_packets[0]["lahWaypointList"][0]["coordinate"]
            a_lat, a_lon = float(c0["latitude"]), float(c0["longitude"])
        else:
            return base_packets

    start_map = {
        1: _offset(a_lat, a_lon, north_m=-150.0, east_m=  0.0),
        2: _offset(a_lat, a_lon, north_m=-150.0, east_m=150.0),
        3: _offset(a_lat, a_lon, north_m=-150.0, east_m=300.0),
    }

    rtb_sorted = sorted(
        [p for p in rtb_list if "latitude" in p and "longitude" in p],
        key=lambda p: p["longitude"]
    )
    rtb_map = {i+1: rtb_sorted[i] for i in range(min(3, len(rtb_sorted)))}

    out_packets: List[dict] = []
    for pkt in base_packets:
        aid = pkt["aircraftID"]
        aircraft_alt_offset = _aircraft_alt_offset_m(aid)
        is_attack_path = int(pkt.get("pathID") or 0) in attack_path_ids
        wplist = pkt.get("lahWaypointList") or []
        if not wplist:
            continue

        # start
        st_lat, st_lon = start_map.get(aid, (wplist[0]["coordinate"]["latitude"], wplist[0]["coordinate"]["longitude"]))
        start_alt = (
            _lah_alt_agl(st_lat, st_lon, aircraft_alt_offset)
            if is_attack_path
            else _lah_non_attack_altitude_m(st_lat, st_lon)
        )
        start = {"latitude": st_lat, "longitude": st_lon, "altitude": start_alt}
        eta_s = _dist_s(start, wplist[0]["coordinate"])
        wp_start = _mk_wp(st_lat, st_lon, start["altitude"], eta_s)

        # rtb
        rtb = rtb_map.get(aid)
        if rtb:
            end   = rtb
            if is_attack_path:
                alt_e = _lah_alt_agl(
                    float(end["latitude"]),
                    float(end["longitude"]),
                    aircraft_alt_offset,
                )
            else:
                alt_e = _lah_non_attack_altitude_m(
                    float(end["latitude"]),
                    float(end["longitude"]),
                )
        else:
            end   = wplist[-1]["coordinate"]
            if is_attack_path:
                alt_e = int(end.get("altitude", _lah_alt_agl(
                    float(end["latitude"]),
                    float(end["longitude"]),
                    aircraft_alt_offset,
                )))
            else:
                alt_e = _lah_non_attack_altitude_m(
                    float(end["latitude"]),
                    float(end["longitude"]),
                )
        eta_e = _dist_s(wplist[-1]["coordinate"], end)
        wp_rtb = _mk_wp(end["latitude"], end["longitude"], alt_e, eta_e)

        new_list = [wp_start] + [dict(w) for w in wplist] + [wp_rtb]
        if not is_attack_path:
            new_list = _terrain_follow_non_attack_waypoints(
                new_list,
                cruise_speed=cruise_speed,
                prefer_low_terrain=False,
            )
        else:
            new_list = _terrain_refine_existing_lah_waypoints(
                new_list,
                cruise_speed=cruise_speed,
            )

        # WaypointID 재할당 + next + ECF
        if is_attack_path:
            from modules.common.ecf import apply_leg_fuel_inplace

            apply_leg_fuel_inplace(new_list)

        for w in new_list:
            _strip_wp_extras(w)
        if new_list:
            last_wp = new_list[-1]
            last_wp["hovering"] = {"time": HOVER_LAST_SEC}

        out_packets.append(OrderedDict([
            ("timestamp",  now_ms),
            ("Source", _sw_code()),
            ("pathID",     pkt["pathID"]),
            ("aircraftID", aid),
            ("lahWaypointList", new_list),
        ]))

    if getattr(wp_alloc, "_use_global", False):
        total_wp_count = sum(len(pkt.get("lahWaypointList") or []) for pkt in out_packets)
        if total_wp_count > 0:
            wp_alloc = _WPAllocator(start=int(_reserve_waypoint_block(total_wp_count)))

    for pkt in out_packets:
        _normalize_lah_waypoint_list_inplace(pkt.get("lahWaypointList") or [])

    for pkt in out_packets:
        new_list = pkt.get("lahWaypointList") or []
        for w in new_list:
            w["waypointID"] = wp_alloc.alloc()
        for i in range(len(new_list) - 1):
            new_list[i]["nextWaypointID"] = new_list[i + 1]["waypointID"]

    _validate_lah_flight_plans(out_packets)
    return out_packets

# ── RL 경로 생성 헬퍼 ──────────────────────────────────────────

_rl_model_cache = None
_rl_policy_cache = None  # 직접 policy forward 용
_rl_env_cache: dict = {}  # key=(dem_name, roi_rounded, altitude, hex_step) → env
_rl_route_cache: dict = {}  # key=(roi_key, start, goals_tuple) → samples

def _build_rl_route(
    coords: List[Tuple[float, float]],
    *,
    altitude_m: float = 600.0,
    hex_step: int = 50,
    area_km: float = 10.0,
    wp_interval_m: float = WP_INTERVAL_M,
    cruise_speed: float = LAH_DEFAULT_CRUISE_SPEED_MPS,
) -> List[dict]:
    """RL(PPO) 모델로 LAH 경로를 생성하여 route sample 리스트를 반환한다.
    실패 시 빈 리스트를 반환하여 fallback 으로 넘어간다."""
    global _rl_model_cache
    import sys
    import types
    import tempfile
    bundle_root = _MISSION_PLANNER_DIR / "portable_mission_bundle"
    model_path = bundle_root / "models" / "latest_model.zip"
    print(f"[RL-ROUTE] bundle_root={bundle_root}", flush=True)
    print(f"[RL-ROUTE] model_path exists={model_path.exists()}", flush=True)
    if not model_path.exists():
        print("[RL-ROUTE] FAIL: model file not found", flush=True)
        return []

    # portable_mission 패키지 우회 등록
    pm_pkg = "portable_mission"
    pm_dir = bundle_root / pm_pkg
    if str(bundle_root) not in sys.path:
        sys.path.insert(0, str(bundle_root))
    if pm_pkg not in sys.modules:
        pkg = types.ModuleType(pm_pkg)
        pkg.__path__ = [str(pm_dir)]
        pkg.__package__ = pm_pkg
        sys.modules[pm_pkg] = pkg

    try:
        from portable_mission.hex_utils import DIRECTION_SEQUENCE, offset_to_axial, step_neighbor
        from portable_mission.terrain import crop_dem_by_normalized_roi
        from portable_mission.env import PortableMissionEnv
        print("[RL-ROUTE] portable_mission imports OK", flush=True)
    except Exception as _imp_err:
        print(f"[RL-ROUTE] FAIL: import error: {_imp_err}", flush=True)
        return []

    # 중심 좌표 계산
    center_lat = sum(c[0] for c in coords) / len(coords)
    center_lon = sum(c[1] for c in coords) / len(coords)
    print(f"[RL-ROUTE] center=({center_lat:.4f}, {center_lon:.4f})", flush=True)

    # DEM 자동 선택
    resource_dir = _PROJECT_ROOT / "resource"
    dem_path = regional_dem_path_for_coordinate(resource_dir, center_lat, center_lon)
    if dem_path is None:
        print(
            f"[RL-ROUTE] FAIL: coordinate outside operational DEM coverage "
            f"center=({center_lat:.6f}, {center_lon:.6f})",
            flush=True,
        )
        return []
    print(f"[RL-ROUTE] DEM={dem_path.name}", flush=True)

    # ROI 계산
    try:
        import rasterio
        from rasterio.warp import transform_bounds
        half_lat = (area_km / 2.0) / 111.0
        import math as _m
        cos_lat = max(0.01, _m.cos(_m.radians(center_lat)))
        half_lon = (area_km / 2.0) / (111.0 * cos_lat)
        with rasterio.open(str(dem_path)) as ds:
            bounds = ds.bounds
            native_roi = transform_bounds(
                "EPSG:4326",
                ds.crs,
                center_lon - half_lon,
                center_lat - half_lat,
                center_lon + half_lon,
                center_lat + half_lat,
                densify_pts=21,
            )
        dem_w = bounds.right - bounds.left
        dem_h = bounds.top - bounds.bottom
        roi_left, roi_bottom, roi_right, roi_top = native_roi
        x0 = max(0.0, min(1.0, (roi_left - bounds.left) / dem_w))
        x1 = max(0.0, min(1.0, (roi_right - bounds.left) / dem_w))
        y0 = max(0.0, min(1.0, 1.0 - (roi_top - bounds.bottom) / dem_h))
        y1 = max(0.0, min(1.0, 1.0 - (roi_bottom - bounds.bottom) / dem_h))
        if x0 > x1: x0, x1 = x1, x0
        if y0 > y1: y0, y1 = y1, y0
        roi = {"x0": x0, "y0": y0, "x1": x1, "y1": y1}
        print(f"[RL-ROUTE] ROI={roi}", flush=True)
    except Exception as _roi_err:
        print(f"[RL-ROUTE] FAIL: ROI calc: {_roi_err}", flush=True)
        return []

    # DEM 크롭 → 환경 생성 (캐시 적용)
    global _rl_env_cache
    _roi_key = (
        dem_path.name,
        round(x0, 4), round(y0, 4), round(x1, 4), round(y1, 4),
        int(altitude_m), int(hex_step),
    )
    cached = _rl_env_cache.get(_roi_key)
    if cached is not None:
        env = cached
        print(f"[RL-ROUTE] env CACHED: grid={env.grid_shape}", flush=True)
    else:
        fd, tmp = tempfile.mkstemp(suffix=".tif")
        os.close(fd)
        try:
            crop_dem_by_normalized_roi(Path(dem_path), roi, Path(tmp))
            env = PortableMissionEnv(
                dem_path=tmp, altitude_m=altitude_m, hex_step=hex_step,
                max_steps=3000, max_goals=20,
            )
            print(f"[RL-ROUTE] env created: grid={env.grid_shape}, safe={env.safe_count}, blocked={env.blocked_count}", flush=True)
            _rl_env_cache[_roi_key] = env
            # 캐시가 너무 커지지 않도록 제한
            if len(_rl_env_cache) > 20:
                oldest = next(iter(_rl_env_cache))
                _rl_env_cache.pop(oldest, None)
        except Exception as _env_err:
            print(f"[RL-ROUTE] FAIL: env creation: {_env_err}", flush=True)
            try: os.unlink(tmp)
            except Exception: pass
            return []

    # 좌표 → hex cell 변환
    def _latlon_to_cell(lat, lon):
        nrows, ncols = env.grid_shape
        b = env.bounds
        ny = 1.0 - (lat - b.bottom) / (b.top - b.bottom)
        nx = (lon - b.left) / (b.right - b.left)
        r = max(0, min(nrows - 1, int(ny * nrows)))
        c = max(0, min(ncols - 1, int(nx * ncols)))
        # 가장 가까운 안전 셀 찾기
        if not env.occupancy[r, c]:
            return (r, c)
        best, best_d = (r, c), 999999
        for sr in range(max(0, r - 5), min(nrows, r + 6)):
            for sc in range(max(0, c - 5), min(ncols, c + 6)):
                if not env.occupancy[sr, sc]:
                    d = abs(sr - r) + abs(sc - c)
                    if d < best_d:
                        best_d = d
                        best = (sr, sc)
        return best

    start_cell = _latlon_to_cell(coords[0][0], coords[0][1])
    goal_cells = [_latlon_to_cell(lat, lon) for lat, lon in coords[1:]]
    goal_cells = [g for g in goal_cells if g != start_cell]

    print(f"[RL-ROUTE] start_cell={start_cell}, goal_cells={goal_cells}", flush=True)
    print(f"[RL-ROUTE] coords: {['({:.4f},{:.4f})'.format(c[0],c[1]) for c in coords]}", flush=True)

    if not goal_cells:
        print("[RL-ROUTE] FAIL: no valid goal cells (all same as start)", flush=True)
        try: os.unlink(tmp)
        except Exception: pass
        return []

    # 시작/목표 간 hex 거리 확인
    from portable_mission.hex_utils import offset_to_axial as _o2a

    def _hex_dist(a, b):
        q1, r1 = _o2a(a[0], a[1])
        q2, r2 = _o2a(b[0], b[1])
        dq, dr = q2 - q1, r2 - r1
        return max(abs(dq), abs(dr), abs(-dq - dr))

    for i, gc in enumerate(goal_cells):
        q1, r1 = _o2a(start_cell[0], start_cell[1])
        q2, r2 = _o2a(gc[0], gc[1])
        dq, dr = q2 - q1, r2 - r1
        hdist = max(abs(dq), abs(dr), abs(-dq - dr))
        print(f"[RL-ROUTE] goal[{i}]={gc} hex_dist={hdist}", flush=True)

    # 경로 캐시 확인 (같은 시작/목표는 재계산 불필요)
    global _rl_model_cache, _rl_policy_cache, _rl_route_cache
    _route_key = (_roi_key, start_cell, tuple(goal_cells))
    cached_route = _rl_route_cache.get(_route_key)
    if cached_route is not None:
        print(f"[RL-ROUTE] route CACHED: {len(cached_route)} samples", flush=True)
        return cached_route

    # 모델 로드 및 추론
    try:
        import torch
        if _rl_model_cache is None:
            from stable_baselines3 import PPO
            _rl_model_cache = PPO.load(str(model_path), device="cpu")
            _rl_policy_cache = _rl_model_cache.policy
            _rl_policy_cache.set_training_mode(False)

        obs, _ = env.configure_manual_episode(start_cell, goal_cells)
        path_cells = [start_cell]
        steps = 0
        no_progress_count = 0
        last_goal_idx = 0
        total_hex_dist = sum(
            _hex_dist(start_cell if i == 0 else goal_cells[i - 1], g)
            for i, g in enumerate(goal_cells)
        )
        max_reasonable_steps = max(200, total_hex_dist * 6)

        # 직접 policy forward (SB3 predict 오버헤드 제거)
        obs_tensor = torch.as_tensor(obs, dtype=torch.float32).unsqueeze(0)
        while True:
            with torch.no_grad():
                action_dist = _rl_policy_cache.get_distribution(obs_tensor)
                action = int(action_dist.mode().item())

            obs, reward, terminated, truncated, info = env.step(action)
            steps += 1
            cell = getattr(env, "current_cell", None)
            if cell is not None:
                path_cells.append(tuple(cell))

            if terminated or truncated:
                break

            # 조기 종료: 목표 진전 없이 너무 오래 헤매면 중단
            cur_goal_idx = getattr(env, "current_goal_idx", 0)
            if cur_goal_idx > last_goal_idx:
                last_goal_idx = cur_goal_idx
                no_progress_count = 0
            else:
                no_progress_count += 1
            if no_progress_count > max_reasonable_steps:
                print(f"[RL-ROUTE] early stop: no progress for {no_progress_count} steps", flush=True)
                break

            obs_tensor = torch.as_tensor(obs, dtype=torch.float32).unsqueeze(0)

        reason = info.get("termination_reason", "unknown") if terminated or truncated else "early_stop"
        goals_done = getattr(env, "current_goal_idx", 0)
        print(f"[RL-ROUTE] finished: steps={steps}, reason={reason}, goals={goals_done}/{len(goal_cells)}, path_len={len(path_cells)}", flush=True)
    except Exception as _run_err:
        print(f"[RL-ROUTE] FAIL: RL run error: {_run_err}", flush=True)
        import traceback; traceback.print_exc()
        return []

    # hex cell → 위경도 변환
    world_points = []
    b = env.bounds
    nrows, ncols = env.grid_shape
    for r, c in path_cells:
        if (r, c) in env.centers:
            cx, cy = env.centers[r, c]
        else:
            cx = b.left + (c + 0.5) / ncols * (b.right - b.left)
            cy = b.top - (r + 0.5) / nrows * (b.top - b.bottom)
        # centers는 (x, y) = (lon-like, lat-like)
        world_points.append((cy, cx))

    try: os.unlink(tmp)
    except Exception: pass

    if len(world_points) < 2:
        return []

    # wp_interval_m 간격으로 리샘플링
    resampled = [world_points[0]]
    cum = 0.0
    for prev, cur in zip(world_points, world_points[1:]):
        d = _dist_ll_m(prev, cur)
        cum += d
        if cum >= wp_interval_m:
            resampled.append(cur)
            cum = 0.0
    if resampled[-1] != world_points[-1]:
        resampled.append(world_points[-1])

    # route sample 형식으로 반환
    samples = []
    cum_m = 0.0
    for i, (lat, lon) in enumerate(resampled):
        if i > 0:
            cum_m += _dist_ll_m(resampled[i - 1], (lat, lon))
        samples.append({"lat": lat, "lon": lon, "cum_m": cum_m})

    # 경로 캐시 저장
    _rl_route_cache[_route_key] = samples
    if len(_rl_route_cache) > 50:
        oldest = next(iter(_rl_route_cache))
        _rl_route_cache.pop(oldest, None)

    return samples


def build_lah_flight_plans_fixed(
    missions: List[dict],
    *,
    cruise_speed: float = LAH_DEFAULT_CRUISE_SPEED_MPS,
    wp_interval_m: float = WP_INTERVAL_M,
    manned_plan_mode: str = "normal",
    lah_path_mode: str = "linear",
    lah_rl_hex_step: int = 50,
    lah_rl_area_km: float = 10.0,
    route_start_by_aircraft: dict[int, object] | None = None,
    initial_hold_by_aircraft: dict[int, object] | None = None,
    wp_alloc: _WPAllocator | None = None,
) -> List[dict]:

    reset_lah_mission_plan_timings()
    wp_alloc = wp_alloc or _WPAllocator()
    now_ms   = now_ms_since_2000()
    packets: List[dict] = []
    capstone_mode = str(manned_plan_mode or "normal").strip().lower() in {"capstone", "capstone_mode"}
    area_count_by_aircraft: dict[int, int] = {}
    if capstone_mode:
        for miss in missions:
            try:
                aid0 = int(miss.get("aircraftID", 0))
            except Exception:
                continue
            if aid0 not in (1, 2, 3):
                continue
            info0 = miss.get("individualMissionInfo", {}) or {}
            area_list0 = info0.get("areaList") if isinstance(info0.get("areaList"), list) else []
            if area_list0:
                area_count_by_aircraft[aid0] = int(area_count_by_aircraft.get(aid0, 0)) + 1
    area_seq_by_aircraft: dict[int, int] = {}
    capstone_battle_center_by_aircraft: dict[int, tuple[float, float]] = {}
    last_route_endpoint_by_aircraft: dict[int, tuple[float, float]] = {}
    aircraft_alt_offset_by_id: dict[int, float] = {}
    first_mission_seen_by_aircraft: set[int] = set()

    for miss in missions:
        mission_build_started = time.perf_counter()
        aid = miss["aircraftID"]
        if aid not in (1, 2, 3):
            continue

        path_id = miss.get("pathID")
        if path_id is None:
            raise ValueError(f"[0304] aircraft {aid}: mission missing pathID")

        info   = miss.get("individualMissionInfo", {})
        is_attack_mission = _is_lah_attack_mission(miss, info)
        is_first_aircraft_mission = int(aid) not in first_mission_seen_by_aircraft
        first_mission_seen_by_aircraft.add(int(aid))
        area_list = info.get("areaList") if isinstance(info.get("areaList"), list) else []
        coords = _resolve_lah_route_coords(info)
        is_line_route = _has_lah_line_route_info(info)
        mission_segment_allowed = (
            _mission_low_terrain_segment_checker(info)
            if not is_attack_mission
            else None
        )
        low_terrain_corridor_m = _mission_low_terrain_corridor_m(info)
        preserve_line_endpoints = _preserve_lah_line_endpoints(miss, info) if is_line_route else False
        line_hold_seconds = _lah_line_hold_seconds(miss, info) if is_line_route else None

        is_area_capstone = (
            capstone_mode
            and bool(area_list)
            and int(area_count_by_aircraft.get(aid, 0)) >= 3
        )
        if is_area_capstone:
            area_seq = int(area_seq_by_aircraft.get(aid, 0)) + 1
            area_seq_by_aircraft[aid] = area_seq
            center = _capstone_area_center(area_list)
            if center is not None and area_seq == 2:
                capstone_battle_center_by_aircraft[aid] = center
            if area_seq >= 3:
                center = capstone_battle_center_by_aircraft.get(aid, center)
            if center is not None:
                coords = [center]

        if is_line_route and not is_area_capstone and not preserve_line_endpoints:
            coords = _trim_lah_line_centerline(coords)

        # A Type-1 initial plan starts at the takeover formation, but the first
        # mission still owns its original point hold: TO -> first mission point
        # -> hover.  Later missions continue through the normal endpoint chain.
        takeover_route_prefix: list[tuple[float, float]] = []
        if (
            is_first_aircraft_mission
            and not is_attack_mission
            and len(coords) == 1
            and int(info.get("individualMissionType", 0) or 0) == 9
            and int(info.get("patternType", 0) or 0) == 12
            and isinstance(initial_hold_by_aircraft, dict)
        ):
            raw_initial_hold = initial_hold_by_aircraft.get(
                int(aid),
                initial_hold_by_aircraft.get(str(aid)),
            )
            hold_rows = raw_initial_hold if isinstance(raw_initial_hold, list) else [raw_initial_hold]
            hold_coordinates = _coord_rows_to_latlon_list(hold_rows)
            if hold_coordinates:
                takeover_route_prefix = [hold_coordinates[0]]

        if not coords:
            _record_lah_mission_plan_timing(
                miss,
                info,
                mission_build_started,
                phase="0304_skipped_no_coords",
                path_id=path_id,
                waypoint_count=0,
                skipped=True,
            )
            continue

        unshifted_coords = list(coords)
        offset_north = 0.0
        if not is_area_capstone:
            if aid == 2:
                offset_north = 100.0      # +100 m north
            elif aid == 3:
                offset_north = -100.0     # -100 m north
        if offset_north:
            coords = [_offset_coord(lat, lon, north_m=offset_north)
                      for lat, lon in coords]
            if not _route_is_inside_mission_geometry(coords, mission_segment_allowed):
                # Formation spacing never overrides LINE/AREA containment.
                coords = unshifted_coords
            if takeover_route_prefix:
                takeover_route_prefix = [
                    _offset_coord(lat, lon, north_m=offset_north)
                    for lat, lon in takeover_route_prefix
                ]
        if line_hold_seconds is not None and is_line_route and not is_area_capstone and coords:
            coords = [coords[-1]]

        low_terrain_constrained_leg_start_index = 0
        reported_route_start: dict | None = None
        if not is_attack_mission and coords:
            route_start = last_route_endpoint_by_aircraft.get(int(aid))
            route_prefix: list[tuple[float, float]] = []
            if route_start is None and takeover_route_prefix:
                route_prefix = list(takeover_route_prefix)
                route_start = route_prefix[0]
            elif route_start is None and isinstance(route_start_by_aircraft, dict):
                raw_route_start = route_start_by_aircraft.get(
                    int(aid),
                    route_start_by_aircraft.get(str(aid)),
                )
                normalized_start_rows = _coord_rows_to_lah_coordinate_list(raw_route_start)
                route_prefix = [
                    (float(row["latitude"]), float(row["longitude"]))
                    for row in normalized_start_rows
                ]
                route_start = route_prefix[0] if route_prefix else None
                if normalized_start_rows and "altitude" in normalized_start_rows[0]:
                    reported_route_start = dict(normalized_start_rows[0])
            elif route_start is not None:
                route_prefix = [route_start]
            mission_endpoint = coords[-1]
            if route_prefix:
                if _dist_ll_m(route_prefix[-1], coords[0]) <= 1.0:
                    coords = route_prefix[:-1] + coords
                    low_terrain_constrained_leg_start_index = max(0, len(route_prefix) - 1)
                else:
                    coords = route_prefix + coords
                    low_terrain_constrained_leg_start_index = len(route_prefix)
            last_route_endpoint_by_aircraft[int(aid)] = mission_endpoint

        if is_area_capstone:
            aircraft_alt_offset = CAPSTONE_AREA_AGL_M
        else:
            aircraft_alt_offset = aircraft_alt_offset_by_id.get(int(aid))
            if aircraft_alt_offset is None:
                aircraft_alt_offset = _aircraft_alt_offset_m(aid)
                aircraft_alt_offset_by_id[int(aid)] = float(aircraft_alt_offset)
        mission_ground_ref = _median_ground_m(coords) if is_attack_mission else None
        forced_altitude_m = _forced_lah_altitude_m(info) if is_attack_mission else None

        def _mission_alt(lat: float, lon: float) -> int:
            if not is_attack_mission:
                return _lah_non_attack_altitude_m(lat, lon)
            if forced_altitude_m is not None:
                return int(forced_altitude_m)
            if mission_ground_ref is None:
                return _lah_alt_agl(lat, lon, aircraft_alt_offset)
            return int(round(float(mission_ground_ref) + float(aircraft_alt_offset)))

        wplist: List[OrderedDict] = []

        # ── RL 모드 분기 ──
        use_rl = str(lah_path_mode or "linear").strip().lower() == "rl"
        rl_samples: list = []
        if use_rl and len(coords) >= 2:
            print(f"[0304-RL] aid={aid}, coords={len(coords)}, hex_step={lah_rl_hex_step}, area_km={lah_rl_area_km}, alt={aircraft_alt_offset}", flush=True)
            try:
                rl_samples = _build_rl_route(
                    coords,
                    altitude_m=float(aircraft_alt_offset),
                    hex_step=int(lah_rl_hex_step),
                    area_km=float(lah_rl_area_km),
                    wp_interval_m=wp_interval_m,
                    cruise_speed=cruise_speed,
                )
                print(f"[0304-RL] result: {len(rl_samples)} samples", flush=True)
                # 직선 대비 5배 이상이면 RL이 헤맨 것 → 실패 처리
                straight_dist = sum(_dist_ll_m(coords[i], coords[i + 1]) for i in range(len(coords) - 1))
                max_reasonable_wp = max(10, int(straight_dist / max(wp_interval_m, 100.0)) * 5)
                if len(rl_samples) > max_reasonable_wp:
                    print(f"[0304-RL] REJECTED: {len(rl_samples)} > {max_reasonable_wp} (직선대비 과다) → fallback", flush=True)
                    rl_samples = []
            except Exception as _rl_exc:
                import traceback
                print(f"[0304-RL] EXCEPTION: {_rl_exc}", flush=True)
                traceback.print_exc()
                rl_samples = []
        elif use_rl:
            print(f"[0304-RL] skipped: coords={len(coords)} (need >= 2)", flush=True)

        if rl_samples:
            total_len_m = max(float((rl_samples[-1] or {}).get("cum_m", 0.0) or 0.0), 1.0)
            for idx, sample in enumerate(rl_samples):
                cum_len_m = float(sample.get("cum_m", 0.0) or 0.0)
                eta_s = _clamp_eta_seconds(cum_len_m / max(1e-6, float(cruise_speed)))
                ecf = 1.0 if idx == len(rl_samples) - 1 else round(cum_len_m / total_len_m, 2)
                alt = (
                    _mission_alt(float(sample.get("lat", 0.0)), float(sample.get("lon", 0.0)))
                    if is_attack_mission
                    else 0
                )
                wp = OrderedDict([
                    ("waypointID", 0),
                    ("isDone", False),
                    ("coordinate", {
                        "latitude": round(float(sample.get("lat", 0.0)), 6),
                        "longitude": round(float(sample.get("lon", 0.0)), 6),
                        "altitude": alt,
                    }),
                    ("speed", cruise_speed),
                    ("eta", eta_s),
                    ("ecf", ecf),
                    ("nextWaypointID", 0),
                ])
                wplist.append(wp)

        # ── 기존 직선 경로 (RL 미사용이거나 RL 실패 시) ──
        if not wplist and len(coords) >= 2:
            try:
                samples = route_algos.plan_route_linear(
                    coords,
                    cruise_speed=cruise_speed,
                )
            except Exception:
                samples = []
            if samples:
                route_samples = _resample_route_samples(samples, step_m=wp_interval_m)
                total_len_m = max(float((route_samples[-1] or {}).get("cum_m", 0.0) or 0.0), 1.0)
                for idx, sample in enumerate(route_samples):
                    cum_len_m = float(sample.get("cum_m", 0.0) or 0.0)
                    eta_s = _clamp_eta_seconds(cum_len_m / max(1e-6, float(cruise_speed)))
                    ecf = 1.0 if idx == len(route_samples) - 1 else round(cum_len_m / total_len_m, 2)
                    alt = (
                        _mission_alt(
                            float(sample.get("lat", 0.0)),
                            float(sample.get("lon", 0.0)),
                        )
                        if is_attack_mission
                        else 0
                    )
                    wp = OrderedDict([
                        ("waypointID", 0),
                        ("isDone", False),
                        ("coordinate", {
                            "latitude":  round(float(sample.get("lat", 0.0)), 6),
                            "longitude": round(float(sample.get("lon", 0.0)), 6),
                            "altitude":  alt,
                        }),
                        ("speed", cruise_speed),
                        ("eta",   eta_s),
                        ("ecf",   ecf),
                        ("nextWaypointID", 0),
                    ])
                    wplist.append(wp)

        # --- fallback: 기존 직선 분할 ---
        if not wplist:
            path: List[Tuple[float, float]] = [coords[0]]
            if len(coords) > 1:
                for p, q in zip(coords, coords[1:]):
                    path.extend(_split_line(p, q, step_m=wp_interval_m))

            leg_lengths = [0.0]
            for prev_pt, curr_pt in zip(path, path[1:]):
                leg_lengths.append(_dist_ll_m(prev_pt, curr_pt))
            total_len = max(sum(float(v) for v in leg_lengths[1:]), 1.0)
            cum_len   = 0.0
            for idx, (lat, lon) in enumerate(path):
                if idx:
                    cum_len += float(leg_lengths[idx])
                eta_s = _clamp_eta_seconds(cum_len / cruise_speed) if idx else 0
                ecf    = 1.0 if len(path) == 1 else round(cum_len / total_len, 2)

                alt = _mission_alt(lat, lon) if is_attack_mission else 0
                wp = OrderedDict([
                    ("waypointID", 0),
                    ("isDone", False),
                    ("coordinate", {
                        "latitude":  round(lat, 6),
                        "longitude": round(lon, 6),
                        "altitude":  alt,
                    }),
                    ("speed", cruise_speed),
                    ("eta",   eta_s),
                    ("ecf",   ecf),
                    ("nextWaypointID", 0),
                ])
                wplist.append(wp)

        if wplist and not is_attack_mission:
            wplist = _terrain_follow_non_attack_waypoints(
                wplist,
                cruise_speed=cruise_speed,
                fallback_altitude_provider=_lah_non_attack_altitude_m,
                route_coordinates=(coords if not rl_samples else None),
                low_terrain_segment_allowed=mission_segment_allowed,
                low_terrain_constrained_leg_start_index=(
                    low_terrain_constrained_leg_start_index if not rl_samples else 0
                ),
                low_terrain_corridor_m=low_terrain_corridor_m,
                # Horizontal valley detours are safe only when an explicit
                # LINE/AREA containment checker is available. Geometry-less
                # coordinate/hold missions keep their original course.
                prefer_low_terrain=callable(mission_segment_allowed),
            )
            _prepend_reported_lah_start_altitude_inplace(
                wplist,
                reported_route_start,
            )
        elif wplist:
            wplist = _terrain_refine_existing_lah_waypoints(
                wplist,
                cruise_speed=cruise_speed,
            )

        if wplist:
            for w in wplist:
                w.setdefault("isDone", False)
            for w in wplist:
                _strip_wp_extras(w)
            last_wp = wplist[-1]
            hover_time = HOVER_LAST_SEC
            if is_area_capstone:
                area_seq = int(area_seq_by_aircraft.get(aid, 0))
                hover_time = CAPSTONE_BATTLE_HOLD_SEC if area_seq == 2 else HOVER_LAST_SEC
            elif line_hold_seconds is not None:
                hover_time = int(line_hold_seconds)
                last_wp["_allowSingleLahWaypoint"] = True
            last_wp["hovering"] = {"time": int(hover_time)}
            wplist[-1]["ecf"] = 1.0

        packets.append(OrderedDict([
            ("timestamp",   now_ms),
            ("Source", _sw_code()),
            ("pathID",      path_id),
            ("aircraftID",  aid),
            ("lahWaypointList", wplist),
        ]))
        _record_lah_mission_plan_timing(
            miss,
            info,
            mission_build_started,
            phase="0304_packet_build",
            path_id=path_id,
            waypoint_count=len(wplist),
        )

    _harmonize_lah_packet_boundary_altitudes_inplace(packets)

    if getattr(wp_alloc, "_use_global", False):
        total_wp_count = sum(len(pkt.get("lahWaypointList") or []) for pkt in packets)
        if total_wp_count > 0:
            wp_alloc = _WPAllocator(start=int(_reserve_waypoint_block(total_wp_count)))

    for pkt in packets:
        _normalize_lah_waypoint_list_inplace(pkt.get("lahWaypointList") or [])

    for pkt in packets:
        wplist = pkt.get("lahWaypointList") or []
        for w in wplist:
            w["waypointID"] = wp_alloc.alloc()
        for i in range(len(wplist) - 1):
            wplist[i]["nextWaypointID"] = wplist[i + 1]["waypointID"]

    _validate_lah_flight_plans(packets)
    return packets
