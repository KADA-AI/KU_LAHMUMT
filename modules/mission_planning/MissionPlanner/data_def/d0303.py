from __future__ import annotations
import os
import math
from collections import OrderedDict
from copy import deepcopy
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
from .mission_helpers import now_ms_since_2000, terrain_elev
from .id_allocator import next_waypoint_id as _next_waypoint_id
from .search_speed import spacing_based_search_speed

try:
    from ..config import DEFAULT_SWEEP_SEPARATION_M
except ImportError:
    try:
        from config import DEFAULT_SWEEP_SEPARATION_M  # type: ignore
    except ImportError:
        from modules.mission_planning.MissionPlanner.config import DEFAULT_SWEEP_SEPARATION_M  # type: ignore
from UAV_missionPlanning import UAVMissionPlanner
from . import route_planner_algorithms as route_algos
from .coord_transform import llh_to_xy, xy_to_llh
try:
    from ....common.eta import _order_by_next_chain, _time_from_prev_to_curr_s
except ImportError:
    from modules.common.eta import _order_by_next_chain, _time_from_prev_to_curr_s


def _sw_code(default: str = "MMR") -> str:
    role = (os.environ.get("KU_ROLE") or "").lower()
    return {
        "mission": "MMR",
        "monitoring": "MSM",
        "decision": "MOB",
    }.get(role, default)


# ── 타입 alias (가독용) ─────────────────────────────
Point = Tuple[float, float]
Line  = Tuple[Point, Point]

# ── 고정 상수 ───────────────────────────────────────────
FOV_DEG         = 10
SWEEP_ENTRY_OFFSET_M = 1500.0
SWEEP_MERGE_HEADING_DEG = 5
SWEEP_LINE_INTERP_POINTS = 3  # >=2; controls how many sample points are emitted per sweep line
Altitude = 610
DEFAULT_SEARCH_SPEED_MULTIPLIER = 16.0
POINT_FOV_DEG = 66.638654
MIN_SWEEP_LEN_M = 3.0
MIN_ROUTE_SPACING_M = 200.0
SWEEP_MERGE_MODE = "heading"
ENTRY_HOLD_FOV_DEG = 10.0
ENTRY_HOLD_GIMBAL_PITCH = -90.0
ENTRY_HOLD_GIMBAL_YAW = 0.0
LOITER_RADIUS_M = 800
LOITER_DIRECTION = 1
LOITER_TIME_S = 30
LOITER_SPEED_MPS = 30
ROUTE_PLANNER_NAME = "dtatrim"


def _normalize_altitude(value: Optional[float], default: int = Altitude) -> int:
    """고도를 정수(m)로 정규화."""
    if value is None:
        return int(default)
    try:
        alt = float(value)
    except (TypeError, ValueError):
        alt = float(default)
    return int(round(alt))


@dataclass(frozen=True)
class SweepConfig:
    separation_m: float
    fov_deg: float

SWEEP_GEOMETRY = SweepConfig(
    separation_m=DEFAULT_SWEEP_SEPARATION_M,
    fov_deg=FOV_DEG,
)

def _active_sweep_geometry() -> SweepConfig:
    return SWEEP_GEOMETRY


def set_route_planner(name: str) -> None:
    """Select route planner for type-7 missions."""
    global ROUTE_PLANNER_NAME, SWEEP_MERGE_MODE
    planner = str(name or "").strip().lower() or "dtatrim"
    ROUTE_PLANNER_NAME = planner
    if planner in ("linear", "algo2"):
        SWEEP_MERGE_MODE = "all"
    elif planner in ("algo3",):
        SWEEP_MERGE_MODE = "curve"
    else:
        SWEEP_MERGE_MODE = "heading"


def _plan_route_points(
    base: list[tuple[float, float]],
    *,
    cruise_speed: float,
    heading_tol_deg: float,
) -> list[dict]:
    planner = (ROUTE_PLANNER_NAME or "dtatrim").strip().lower()
    if planner in ("linear", "algo2"):
        return route_algos.plan_route_linear(base, cruise_speed=cruise_speed)
    return route_algos.plan_route_dtatrim(
        base,
        cruise_speed=cruise_speed,
        heading_tol_deg=heading_tol_deg,
    )


def _sweep_spacing_m(*, separation_m: float, fov_deg: float) -> float:
    """Return physical spacing between sweep strips in meters."""
    base = 2.0 * max(separation_m, 1.0) * math.tan(max(math.radians(fov_deg) / 2.0, 1e-6))
    return max(base, 1.0)


def _debug_sweep(label: str, *, separation: float, fov: float, spacing: float) -> None:
    """Lightweight debug printer for sweep spacing."""
    try:
        approx = max(int(round(1000.0 / max(spacing, 1e-6))), 1)
        print(
            f"[SWEEP][{label}] separation={separation:.2f}m, fov={fov:.2f}°, "
            f"spacing={spacing:.2f}m (~{approx} strips/1km)"
        )
    except Exception:
        pass


def _coord_to_xy(coord: dict, ref: dict) -> Tuple[float, float]:
    lat0 = float(ref.get("latitude", 0.0))
    lon0 = float(ref.get("longitude", 0.0))
    lat = float(coord.get("latitude", lat0))
    lon = float(coord.get("longitude", lon0))
    return llh_to_xy(lat, lon, lat0, lon0)


def _xy_to_coord(x: float, y: float, ref: dict, *, altitude: Optional[float] = None) -> Dict[str, float]:
    lat0 = float(ref.get("latitude", 0.0))
    lon0 = float(ref.get("longitude", 0.0))
    lat, lon = xy_to_llh(x, y, lat0, lon0)
    alt_source = altitude if altitude is not None else ref.get("altitude", Altitude)
    alt = _normalize_altitude(alt_source)
    return {
        "latitude": round(lat, 6),
        "longitude": round(lon, 6),
        "altitude": alt,
    }


def _unit_vec(vec: Tuple[float, float]) -> Tuple[float, float]:
    mag = math.hypot(vec[0], vec[1])
    if mag <= 1e-6:
        return (0.0, 0.0)
    return (vec[0] / mag, vec[1] / mag)


def _heading_deg(vec: Tuple[float, float]) -> float:
    return (math.degrees(math.atan2(vec[1], vec[0])) + 360.0) % 360.0


def _wrap_delta(deg: float) -> float:
    return ((deg + 180.0) % 360.0) - 180.0


SENSOR_NONE     = 0
SENSOR_EO_IR    = 1        # 예) EO/IR 센서

# WaypointPassType
PASS_NONE = 0
PASS_FLYBY = 1
PASS_LOITER = 2

# OperationMode
OPMODE_NONE   = 0
OPMODE_POINT  = 1
OPMODE_LINE   = 2
OPMODE_TRACK  = 3
OPMODE_HOLD   = 4
OPMODE_SWEEP  = 5

MISSION_DISPATCH = {
    0: "없음", 
    1: "표적추적", # ID 필요 -> 패턴은 1
    2: "표적공격", # 헬기만 -> ID 필요, 패턴은 2번
    3: "영역수색", # 정해진 횟수만큼 영역에 대한 촬영 계획 세움 -> 패턴은 3,4,5,6,7,8,9
    4: "영역경계", # 정해진 시간동안 영역에 대한 촬영 계획 세움-> 패턴은 3,4,5,6,7,8,9
    5: "좌표점정찰", # 점만 보는 형태 -> 패턴은 1이 될 듯
    6: "통로정찰", # 중심선 기준 좌우 스윕 -> 패턴은 4번
    7: "이동", # 이동 계획 세움 -> 카메라들은 직하방으로 고정
    8: "엄호", # TBD
    9: "은엄폐", # 헬기의 유지 포지션 설정해줌
}

PATTERN_DISPATCH = {
    0:  "없음",
    1:  "표적중심선회·추적", # 0303의 Operation Mode를 3번으로, WaypointPassType가 3 -> 비행 계획은 LoiterProperty 가 생성되어야 함, 선회반경은 1000으로 고정하고 Direction도 0 : None, 1 : CW, 2 : CCW -> Time은 5분 고정 -> Speed는 40고정
    2:  "은엄폐후공격", #유인기에서 할것 TBD
    3:  "직하방-BF촬영", # 직하방 촬영으로 고정하고 비행계획 알고리즘을 CPP 알고리즘으로 대체해서 촬영 중심점 = 비행 경로인 상황임, 한 라인찍고 다음라인을 찍기위해 도는 선회 부분이 중요함
    4:  "이격-BF촬영", # 지금과 동일한 상태
    5:  "구간왕복-BF촬영", #TBD
    6:  "선형반복주사-BF촬영", #TBD
    7:  "직상공순회촬영", #TBD
    8:  "구간중심종단-선형반복주사촬영", #TBD
    9:  "구간중심종단-자동반복주사촬영", #TBD
    10: "목적지이동", #카메라를 직하방으로 고정함 -> 지금과 동일하게 이동 경로 계획함
    11: "대상엄호", #TBD
    12: "은엄폐", #해당 점에서 고도를 최대한(안전고도까지) 낮춰서 숨어있음(유인기만 사용)
}

def _dem_alt(lat: float, lon: float) -> int:
    """DEM 기반 고도를 정수(m)로 반환."""
    return int(round(terrain_elev(lat, lon)))

def _poly_sweeps_general(
    poly_llh: list[tuple[float,float]],
    anchor_llh: tuple[float,float],
    bearing_deg: float,
    fov_deg: float,
    separation_m: float,
) -> list[dict]:
    """
    ▸ Convex polygon LLH → bearing 과 평행한 띠 스윕 → lineSearch.coordinateList
    ▸ 리턴: [{"latitude":…, "longitude":…, "altitude":5}, …]  (짝수/홀수=한 라인)
    """
    lat0, lon0 = poly_llh[0]
    poly_xy = [llh_to_xy(lat, lon, lat0, lon0) for lat, lon in poly_llh]
    anchor_xy = llh_to_xy(anchor_llh[0], anchor_llh[1], lat0, lon0)

    th = math.radians(bearing_deg)
    nx, ny =  math.sin(th), -math.cos(th)            # bearing 과 직각인 노멀
    proj = [nx*x + ny*y for x, y in poly_xy]
    d_min, d_max = min(proj), max(proj)

    spacing_m = _sweep_spacing_m(separation_m=separation_m, fov_deg=fov_deg)
    d_values: list[float] = []
    current = d_min
    # ensure we always include the last strip by overshooting slightly
    while current <= d_max + spacing_m * 0.25:
        d_values.append(current)
        current += spacing_m
    if not d_values:
        d_values = [d_min, d_max]
    elif len(d_values) == 1 and d_max > d_min:
        d_values.append(d_max)
    else:
        last = d_values[-1]
        if d_max - last > spacing_m * 0.25:
            d_values.append(d_max)

    # 다각형 edge 리스트
    edges = [(poly_xy[i], poly_xy[(i+1)%len(poly_xy)]) for i in range(len(poly_xy))]
    coord_list: list[dict] = []

    for d in d_values:
        # 직선: n·x = d  ↔  (−ny, nx) 방향 unit 벡터
        hits = []
        for a,b in edges:
            f1, f2 = nx*a[0]+ny*a[1]-d, nx*b[0]+ny*b[1]-d
            if f1*f2 <= 0 and f1 != f2:
                t = f1/(f1-f2)
                x = a[0] + t*(b[0]-a[0]);   y = a[1] + t*(b[1]-a[1])
                hits.append((x,y))
        if len(hits) >= 2:
            # 스윕 선은 교차점 두 개를 연결
            p1, p2 = hits[0], hits[1]
            for x,y in (p1,p2):
                lat, lon = xy_to_llh(x,y,lat0,lon0)
                coord_list.append({
                    "latitude": lat,
                    "longitude": lon,
                    "altitude": _dem_alt(lat, lon),
                })
    return coord_list

def _mk_filming(operation_mode: int = OPMODE_NONE,
                fov: float = FOV_DEG,
                sensor: int = SENSOR_NONE,
                line_search: OrderedDict | None = None,
                gimbal_pitch: float | None = None,
                gimbal_yaw: float | None = None) -> OrderedDict:
    """
    filmingProperty 블록 생성 헬퍼

    • OPMODE_LINE  → lineSearch 필드 삽입  
    • OPMODE_HOLD → aircraftFixed  블록 삽입
                  ↳ gimbalPitch / gimbalYaw 포함
    """
    fp = OrderedDict([
        ("fieldOfView",   fov),
        ("sensorType",    sensor),
        ("operationMode", operation_mode),
    ])

    # ① 선형 탐색(lineSearch)
    if operation_mode == OPMODE_LINE and line_search is not None:
        fp["lineSearch"] = line_search

    # ② 고정 촬영(aircraftFixed + 짐벌 각도)
    if operation_mode == OPMODE_HOLD:
        # 기본값:  직하방 -90°, Yaw 0°
        gimbal_pitch = -90.0 if gimbal_pitch is None else gimbal_pitch
        gimbal_yaw   =   0.0 if gimbal_yaw   is None else gimbal_yaw
        fp["aircraftFixed"] = OrderedDict([
            ("gimbalPitch", gimbal_pitch),
            ("gimbalYaw",   gimbal_yaw),
        ])
    return fp


def _has_line_search(wp: dict) -> bool:
    fp = wp.get("filmingProperty") or {}
    if int(fp.get("operationMode", OPMODE_NONE)) != OPMODE_LINE:
        return False
    line_search = fp.get("lineSearch") or {}
    coords = line_search.get("coordinateList") or []
    return bool(coords)


def _angle_between(v1: tuple[float, float], v2: tuple[float, float]) -> float:
    x1, y1 = v1
    x2, y2 = v2
    n1 = math.hypot(x1, y1)
    n2 = math.hypot(x2, y2)
    if n1 <= 1e-6 or n2 <= 1e-6:
        return 0.0
    cos_th = max(-1.0, min(1.0, (x1 * x2 + y1 * y2) / (n1 * n2)))
    return math.degrees(math.acos(cos_th))


def _interp_points_hint(value: object) -> int | None:
    """Convert a stored interpolation hint into an int ≥ 2."""
    try:
        points = int(value)
    except (TypeError, ValueError):
        return None
    return points if points >= 2 else None


def _collect_sweep_spans(coords: list[dict], points_per_line: int | None) -> list[float]:
    """Return individual sweep spans extracted from the coordinate list."""
    if not coords or len(coords) < 2:
        return []
    chunk = max(points_per_line or 2, 2)
    spans: list[float] = []
    idx = 0
    while idx < len(coords) - 1:
        start = coords[idx]
        end_idx = min(idx + chunk - 1, len(coords) - 1)
        end = coords[end_idx]
        lat0 = float(start.get("latitude", 0.0))
        lon0 = float(start.get("longitude", 0.0))
        lat1 = float(end.get("latitude", lat0))
        lon1 = float(end.get("longitude", lon0))
        dx, dy = llh_to_xy(lat1, lon1, lat0, lon0)
        dist = math.hypot(dx, dy)
        if dist > 0.0:
            spans.append(dist)
        idx = end_idx + 1

    return spans


def _avg_sweep_width_m(coords: list[dict], *, points_per_line: int | None = None) -> float | None:
    """
    Estimate the average span of each sweep line in meters.

    When coordinates include interpolated mid-points, `points_per_line` tells the helper
    how many samples belong to a single sweep (>=2). The default pairs entries (0-1, 2-3, ...).
    """
    spans = _collect_sweep_spans(coords, points_per_line)
    if not spans:
        return None
    return round(sum(spans) / len(spans), 2)


def _subdivide_segment(start: dict, end: dict, points: int) -> list[dict]:
    """Generate evenly spaced coordinates along start-end inclusive."""
    if points <= 2:
        return [deepcopy(start), deepcopy(end)]
    lat0 = float(start.get("latitude", 0.0))
    lon0 = float(start.get("longitude", 0.0))
    alt0 = float(start.get("altitude", Altitude))
    lat1 = float(end.get("latitude", lat0))
    lon1 = float(end.get("longitude", lon0))
    alt1 = float(end.get("altitude", alt0))
    dx, dy = llh_to_xy(lat1, lon1, lat0, lon0)
    coords: list[dict] = []
    for idx in range(points):
        if idx == 0:
            coords.append(deepcopy(start))
            continue
        if idx == points - 1:
            coords.append(deepcopy(end))
            continue
        t = idx / (points - 1)
        xi = dx * t
        yi = dy * t
        lat_i, lon_i = xy_to_llh(xi, yi, lat0, lon0)
        alt_i = alt0 + (alt1 - alt0) * t
        coords.append(OrderedDict([
            ("latitude", round(lat_i, 6)),
            ("longitude", round(lon_i, 6)),
            ("altitude", int(round(alt_i))),
        ]))
    return coords


def _interpolate_line_coords(coords: list[dict], points: int) -> list[dict]:
    """Split sweep line coordinate pairs into interpolated segments."""
    if points <= 2 or not coords:
        return coords
    result: list[dict] = []
    for idx in range(0, len(coords), 2):
        start = coords[idx]
        end = coords[idx + 1] if idx + 1 < len(coords) else start
        subdivided = _subdivide_segment(start, end, points)
        if result and subdivided and result[-1] == subdivided[0]:
            subdivided = subdivided[1:]
        result.extend(deepcopy(coord) for coord in subdivided)
    return result


class _WPAllocator:
    def __init__(self, start: int | None = None) -> None:
        self._local_next = start
        self._use_global = start is None

    def alloc(self) -> int:
        if self._use_global:
            return int(_next_waypoint_id())
        if self._local_next is None:
            raise RuntimeError("Waypoint allocator misconfigured (local start unset)")
        if self._local_next > 65_535:
            raise RuntimeError("WaypointID pool exhausted")
        wid = self._local_next
        self._local_next += 1
        return wid

def _index_refpoints(ref0203: dict | None):
    """
    0203에서 기체ID→좌표 맵 구성.
    return: (to_map, ho_map)  각 값은 {aid: {"latitude":..,"longitude":..,"altitude":..}}
    """
    if not ref0203:
        return {}, {}
    to_map = {}
    for it in ref0203.get("takeOverInfoList", []) or []:
        aid = it.get("aircraftID")
        c   = it.get("coordinate") or {}
        if isinstance(aid, int) and 1 <= aid <= 6 and "latitude" in c and "longitude" in c:
            to_map[aid] = {
                "latitude": float(c.get("latitude", 0.0)),
                "longitude": float(c.get("longitude", 0.0)),
                "altitude": _normalize_altitude(c.get("altitude", Altitude)),
            }
    ho_map = {}
    for it in ref0203.get("handOverInfoList", []) or []:
        aid = it.get("aircraftID")
        c   = it.get("coordinate") or {}
        if isinstance(aid, int) and 1 <= aid <= 6 and "latitude" in c and "longitude" in c:
            ho_map[aid] = {
                "latitude": float(c.get("latitude", 0.0)),
                "longitude": float(c.get("longitude", 0.0)),
                "altitude": _normalize_altitude(c.get("altitude", Altitude)),
            }
    return to_map, ho_map


def _eta_ms_llh(c1: dict, c2: dict, speed_mps: float) -> int:
    """
    간단한 구면 근사로 거리→ETA(s) 계산. speed_mps<=0이면 0.
    c* = {"latitude": float, "longitude": float}
    """
    try:
        lat1, lon1 = float(c1["latitude"]), float(c1["longitude"])
        lat2, lon2 = float(c2["latitude"]), float(c2["longitude"])
    except Exception:
        return 0
    DEG_M = 111_132.0
    dx = (lon2 - lon1) * DEG_M * math.cos(math.radians((lat1 + lat2) / 2.0))
    dy = (lat2 - lat1) * DEG_M
    dist_m = math.hypot(dx, dy)
    if speed_mps and speed_mps > 0.0:
        return int(round(dist_m / speed_mps))
    return 0


def _annotate_eta_ms_inplace(waypoints: list[OrderedDict], default_speed_mps: float) -> None:
    # NOTE: historical function name kept for compatibility; ETA is emitted in seconds.
    if not waypoints:
        return

    ordered = _order_by_next_chain(waypoints)
    if not ordered:
        return

    # ETA is stored as seconds-per-leg for downstream consumers (spec unit: seconds)
    ordered[0]["eta"] = 0
    acc_s = 0.0
    prev_cum_s = 0
    for i in range(1, len(ordered)):
        dt_s = _time_from_prev_to_curr_s(ordered[i - 1], ordered[i], default_speed_mps=default_speed_mps)
        acc_s += dt_s
        cum_s = int(round(acc_s))
        delta_s = max(0, cum_s - prev_cum_s)
        ordered[i]["eta"] = delta_s
        prev_cum_s = cum_s

def build_flight_plans(
    missions: list[dict],
    wp_alloc: _WPAllocator | None = None,
    cruise_speed: float = 30.0,
    turn_step_deg: float = 45.0,
) -> list[dict]:
    wp_alloc = wp_alloc or _WPAllocator()
    now_ms = now_ms_since_2000()

    # ── 상수 ───────────────────────────────────────────────
    SENSOR, OPMODE = 1, 2
    DEFAULT_SEARCH_SPEED = round(cruise_speed * DEFAULT_SEARCH_SPEED_MULTIPLIER, 2)
    geom = _active_sweep_geometry()
    ALT_M = geom.separation_m
    geom_fov_deg = geom.fov_deg
    sweep_spacing_m = _sweep_spacing_m(separation_m=ALT_M, fov_deg=geom_fov_deg)
    add_end_loiter = SWEEP_MERGE_MODE in ("heading", "curve")
    use_agl = ROUTE_PLANNER_NAME in ("linear", "algo2")
    def _wp_alt(lat: float, lon: float, fallback: float | int | None = None) -> int:
        if use_agl:
            return int(round(_dem_alt(lat, lon) + float(Altitude)))
        base = fallback if fallback is not None else Altitude
        return _normalize_altitude(base)
    DEG_M = 111_132
    # ── 마지막점용 POINT 촬영 블록 생성기 ─────────────────
    def _mk_point_filming_for_coord(coord: dict) -> OrderedDict:
        lat = float(coord.get("latitude", 0.0))
        lon = float(coord.get("longitude", 0.0))
        return OrderedDict([
            ("fieldOfView", POINT_FOV_DEG),
            ("sensorType", SENSOR_EO_IR),
            ("operationMode", OPMODE_POINT),
            ("coordinateOrientation", OrderedDict([
                ("coordinate", OrderedDict([
                    ("latitude",  lat),
                    ("longitude", lon),
                    ("altitude",  0),
                ]))
            ])),
        ])

    packets: list[dict] = []

    # ────────────────────────── 1) 미션 → 패킷 ──────────────────────────
    for miss in missions:
        aid = miss["aircraftID"]
        if aid not in (4, 5, 6):
            continue

        info = miss["individualMissionInfo"]
        mtype = info.get("individualMissionType")
        wplist: list[OrderedDict] = []
        full_sweep_coords: list[dict] | None = None
        full_sweep_speed: float | None = None
        full_sweep_interp_points: int | None = None

        # 1-A. 통로정찰 / 영역수색 (type 3·4·6)
        if mtype in (3, 4, 6):
            base, width = None, 100.0
            spacing_line = sweep_spacing_m

            # (i) lineList → corridor
            if info.get("lineList"):
                line = info["lineList"][0]
                width = line["width"]
                base = [(c["latitude"], c["longitude"]) for c in line["coordinateList"]]
                spacing_line = _sweep_spacing_m(separation_m=ALT_M, fov_deg=geom_fov_deg)
                _debug_sweep("LINE", separation=ALT_M, fov=geom_fov_deg, spacing=spacing_line)

            # (ii) areaList
            elif info.get("areaList"):
                pts = [(p["latitude"], p["longitude"]) for p in info["areaList"][0]["coordinateList"]]

                bearing = miss.get("bearing_deg", 90.0)
                th = math.radians(bearing)
                ux_b, uy_b = math.sin(th), math.cos(th)

                prev_pt = miss.get("prevPoint", pts[0])

                strip_spacing = ALT_M
                _debug_sweep("AREA", separation=ALT_M, fov=geom_fov_deg, spacing=spacing_line)
                coord_list = _poly_sweeps_general(
                    poly_llh=pts,
                    anchor_llh=prev_pt,
                    bearing_deg=bearing,
                    fov_deg=geom_fov_deg,
                    separation_m=ALT_M,
                )
                if not coord_list:
                    continue

                lines = [coord_list[i:i+2] for i in range(0, len(coord_list), 2)]
                if not lines:
                    continue

                # 짧은 스윕 필터
                MIN_SWEEP_LEN = MIN_SWEEP_LEN_M
                filtered = []
                lat0, lon0 = lines[0][0]["latitude"], lines[0][0]["longitude"]
                for ln in lines:
                    s_lat, s_lon = ln[0]["latitude"], ln[0]["longitude"]
                    e_lat, e_lon = ln[1]["latitude"], ln[1]["longitude"]
                    dx = (e_lon - s_lon) * 111_132 * math.cos(math.radians((s_lat + e_lat) / 2))
                    dy = (e_lat - s_lat) * 111_132
                    if math.hypot(dx, dy) >= MIN_SWEEP_LEN:
                        filtered.append(ln)
                lines = filtered
                if not lines:
                    continue

                lat0, lon0 = lines[0][0]["latitude"], lines[0][0]["longitude"]

                last_off_xy: tuple[float, float] | None = None
                for idx, ln in enumerate(lines):
                    s, e = ln
                    s_xy = llh_to_xy(s['latitude'], s['longitude'], lat0, lon0)
                    e_xy = llh_to_xy(e['latitude'], e['longitude'], lat0, lon0)

                    mid_xy = ((s_xy[0] + e_xy[0]) / 2, (s_xy[1] + e_xy[1]) / 2)
                    off_xy = (mid_xy[0] - ux_b * strip_spacing, mid_xy[1] + uy_b * strip_spacing)
                    last_off_xy = off_xy
                    off_lat, off_lon = xy_to_llh(*off_xy, lat0, lon0)

                    sweep = [e, s] if idx % 2 else [s, e]
                    sweep_width = _avg_sweep_width_m(sweep)
                    sweep_speed = spacing_based_search_speed(
                        sweep_len_m=sweep_width,
                        spacing_m=spacing_line,
                        cruise_speed_mps=cruise_speed,
                    )
                    if sweep_speed is None:
                        sweep_speed = DEFAULT_SEARCH_SPEED

                    wplist.append(OrderedDict([
                        ("waypointID", 0),
                        ("coordinate", {"latitude": off_lat, "longitude": off_lon, "altitude": _wp_alt(off_lat, off_lon, Altitude)}),
                        ("speed", cruise_speed),
                        ("eta", 2500),
                        ("ecf", 0.0),
                        ("nextWaypointID", 0),
                        ("waypointPassType", PASS_FLYBY),
                        ("filmingProperty", _mk_filming(
                            operation_mode=OPMODE_LINE,
                            sensor=SENSOR_EO_IR,
                            line_search=OrderedDict([
                                ("coordinateList", sweep),
                                ("searchSpeed", sweep_speed),
                            ]),
                        )),
                    ]))

                # ➌ 종료 WP (Loiter + POINT 촬영)
                if add_end_loiter and len(wplist) >= 2:
                    last_wp = wplist[-1]["coordinate"]
                    prev_wp = wplist[-2]["coordinate"]
                    last_xy = llh_to_xy(last_wp["latitude"], last_wp["longitude"], lat0, lon0)
                    prev_xy = llh_to_xy(prev_wp["latitude"], prev_wp["longitude"], lat0, lon0)
                    vx, vy = last_xy[0] - prev_xy[0], last_xy[1] - prev_xy[1]
                    vlen = math.hypot(vx, vy) or 1.0
                    ux_end, uy_end = vx / vlen, vy / vlen
                    end_xy = (last_xy[0] + ux_end * 200.0, last_xy[1] + uy_end * 200.0)
                    end_lat, end_lon = xy_to_llh(*end_xy, lat0, lon0)
                    end_coord = {"latitude": end_lat, "longitude": end_lon, "altitude": _wp_alt(end_lat, end_lon, Altitude)}

                    wplist.append(OrderedDict([
                        ("waypointID", 0),
                        ("coordinate", end_coord),
                        ("speed", cruise_speed),
                        ("eta", 0),
                        ("ecf", 0.0),
                        ("nextWaypointID", 0),
                        ("waypointPassType", PASS_LOITER),
                        ("filmingProperty", _mk_point_filming_for_coord(end_coord)),
                    ]))
                elif add_end_loiter and len(wplist) == 1 and last_off_xy is not None:
                    end_xy = (last_off_xy[0] + ux_b * strip_spacing, last_off_xy[1] + uy_b * strip_spacing)
                    end_lat, end_lon = xy_to_llh(*end_xy, lat0, lon0)
                    end_coord = {"latitude": end_lat, "longitude": end_lon, "altitude": _wp_alt(end_lat, end_lon, Altitude)}

                    wplist.append(OrderedDict([
                        ("waypointID", 0),
                        ("coordinate", end_coord),
                        ("speed", cruise_speed),
                        ("eta", 0),
                        ("ecf", 0.0),
                        ("nextWaypointID", 0),
                        ("waypointPassType", PASS_LOITER),
                        ("filmingProperty", _mk_point_filming_for_coord(end_coord)),
                    ]))

            # (iii) Corridor-planner
            if base and len(base) >= 2:
                planner = UAVMissionPlanner(
                    base, corridor_width=width, separation=ALT_M,
                    fov_deg=geom_fov_deg, cruise_speed=cruise_speed, crs="lla",
                )
                use_centerline = ROUTE_PLANNER_NAME in ("linear", "algo2")
                if use_centerline:
                    base_xy: list[tuple[float, float]] = []
                    proj_fwd = getattr(planner, "_proj_fwd", None)
                    if proj_fwd is not None:
                        for lat, lon in base:
                            x, y = proj_fwd(lon, lat)
                            base_xy.append((x, y))
                    else:
                        lat0, lon0 = base[0]
                        base_xy = [llh_to_xy(lat, lon, lat0, lon0) for lat, lon in base]

                    sweep_mid_xy: list[tuple[float, float]] = []
                    for sw in planner.sweeps:
                        sweep_mid_xy.append(((sw[0][0] + sw[1][0]) / 2, (sw[0][1] + sw[1][1]) / 2))

                    def _select_sweep_indices(
                        points_xy: list[tuple[float, float]],
                        sweep_midpoints: list[tuple[float, float]],
                    ) -> list[int]:
                        if not points_xy or not sweep_midpoints:
                            return []
                        total_pts = len(points_xy)
                        total_sweeps = len(sweep_midpoints)
                        if total_sweeps <= total_pts:
                            return list(range(total_sweeps))
                        selected: list[int] = []
                        start_idx = 0
                        for pos, pt in enumerate(points_xy):
                            remaining = total_pts - pos - 1
                            max_idx = total_sweeps - 1 - remaining
                            if max_idx < start_idx:
                                max_idx = start_idx
                            best_idx = start_idx
                            best_dist = None
                            for idx in range(start_idx, max_idx + 1):
                                dx = sweep_midpoints[idx][0] - pt[0]
                                dy = sweep_midpoints[idx][1] - pt[1]
                                dist = dx * dx + dy * dy
                                if best_dist is None or dist < best_dist:
                                    best_dist = dist
                                    best_idx = idx
                            selected.append(best_idx)
                            start_idx = best_idx + 1
                        return selected

                    sweep_indices = _select_sweep_indices(base_xy, sweep_mid_xy)
                    if not sweep_indices:
                        sweep_indices = list(range(len(planner.sweeps)))
                    anchor_list = planner.offset_wps

                    merged_coords: list[dict] = []
                    for sw_idx, sw in enumerate(planner.sweeps):
                        s_xy, e_xy = sw
                        if sw_idx % 2:
                            s_xy, e_xy = e_xy, s_xy
                        s_lat, s_lon = planner._proj_back(s_xy[0], s_xy[1])[::-1]
                        e_lat, e_lon = planner._proj_back(e_xy[0], e_xy[1])[::-1]
                        merged_coords.append({
                            "latitude": s_lat,
                            "longitude": s_lon,
                            "altitude": _dem_alt(s_lat, s_lon),
                        })
                        merged_coords.append({
                            "latitude": e_lat,
                            "longitude": e_lon,
                            "altitude": _dem_alt(e_lat, e_lon),
                        })
                    if merged_coords:
                        full_sweep_coords = merged_coords
                        full_sweep_interp_points = SWEEP_LINE_INTERP_POINTS
                        merged_width = _avg_sweep_width_m(
                            merged_coords,
                            points_per_line=2,
                        )
                        full_sweep_speed = spacing_based_search_speed(
                            sweep_len_m=merged_width,
                            spacing_m=spacing_line,
                            cruise_speed_mps=cruise_speed,
                        )
                        if full_sweep_speed is None:
                            full_sweep_speed = DEFAULT_SEARCH_SPEED
                else:
                    sweep_indices = list(range(len(planner.sweeps)))
                    anchor_list = planner.orange_pts

                last_anchor_xy: tuple[float, float] | None = None
                last_first_xy: tuple[float, float] | None = None
                for idx in sweep_indices:
                    if idx >= len(planner.sweeps) or idx >= len(anchor_list):
                        continue
                    anchor_xy = anchor_list[idx]
                    sw = planner.sweeps[idx]
                    w_lat, w_lon = planner._proj_back(anchor_xy[0], anchor_xy[1])[::-1]

                    s_xy, e_xy = sw
                    if idx % 2:
                        s_xy, e_xy = e_xy, s_xy

                    s_lat, s_lon = planner._proj_back(s_xy[0], s_xy[1])[::-1]
                    e_lat, e_lon = planner._proj_back(e_xy[0], e_xy[1])[::-1]
                    coord_list = [
                        {"latitude": s_lat, "longitude": s_lon, "altitude": _dem_alt(s_lat, s_lon)},
                        {"latitude": e_lat, "longitude": e_lon, "altitude": _dem_alt(e_lat, e_lon)},
                    ]

                    coord_width = _avg_sweep_width_m(coord_list)
                    coord_speed = spacing_based_search_speed(
                        sweep_len_m=coord_width,
                        spacing_m=spacing_line,
                        cruise_speed_mps=cruise_speed,
                    )
                    if coord_speed is None:
                        coord_speed = DEFAULT_SEARCH_SPEED

                    wplist.append(OrderedDict([
                        ("waypointID", 0),
                        ("coordinate", {"latitude": w_lat, "longitude": w_lon, "altitude": _wp_alt(w_lat, w_lon, Altitude)}),
                        ("speed", cruise_speed),
                        ("eta", 2500),
                        ("ecf", 0.0),
                        ("nextWaypointID", 0),
                        ("waypointPassType", PASS_FLYBY),
                        ("filmingProperty", _mk_filming(
                            operation_mode=OPMODE_LINE,
                            sensor=SENSOR_EO_IR,
                            line_search=OrderedDict([
                                ("coordinateList", coord_list),
                                ("searchSpeed", coord_speed),
                            ]),
                        )),
                    ]))

                    last_anchor_xy = anchor_xy
                    last_first_xy = s_xy

                if add_end_loiter and last_anchor_xy and last_first_xy:
                    vx, vy = last_first_xy[0] - last_anchor_xy[0], last_first_xy[1] - last_anchor_xy[1]
                    vlen = math.hypot(vx, vy) or 1.0
                    ux_c, uy_c = vx / vlen, vy / vlen
                    end_xy = (last_anchor_xy[0] + ux_c * 300.0, last_anchor_xy[1] + uy_c * 300.0)
                    end_lat, end_lon = planner._proj_back(end_xy[0], end_xy[1])[::-1]
                    end_coord = {"latitude": end_lat, "longitude": end_lon, "altitude": _wp_alt(end_lat, end_lon, Altitude)}

                    wplist.append(OrderedDict([
                        ("waypointID", 0),
                        ("coordinate", end_coord),
                        ("speed", cruise_speed),
                        ("eta", 0),
                        ("ecf", 0.0),
                        ("nextWaypointID", 0),
                        ("waypointPassType", PASS_LOITER),
                        ("filmingProperty", _mk_point_filming_for_coord(end_coord)),
                    ]))


        # 1-B. 이동 미션 (type 7)
        elif mtype == 7:
            base: list[tuple[float, float]] = []
            if info.get("coordinateList"):
                base = [(c["latitude"], c["longitude"]) for c in info["coordinateList"]]
            elif info.get("lineList"):
                base = [(c["latitude"], c["longitude"]) for c in info["lineList"][0]["coordinateList"]]

            if len(base) == 1:
                lat, lon = base[0]
                wplist.append(OrderedDict([
                    ("waypointID", 0),
                    ("coordinate", {"latitude": lat, "longitude": lon, "altitude": _wp_alt(lat, lon, Altitude)}),
                    ("speed", cruise_speed), ("eta", 0), ("ecf", 1.0),
                    ("nextWaypointID", 0), ("waypointPassType", 1),
                    ("filmingProperty", {}),
                ]))
            elif len(base) >= 2:
                raw_pts = _plan_route_points(
                    base,
                    cruise_speed=cruise_speed,
                    heading_tol_deg=turn_step_deg,
                )
                MIN_SPACING_M = MIN_ROUTE_SPACING_M
                simp: list[dict] = [raw_pts[0]]
                for p in raw_pts[1:]:
                    d = math.hypot(
                        (p["lon"] - simp[-1]["lon"]) * DEG_M * math.cos(
                            math.radians((p["lat"] + simp[-1]["lat"]) / 2)),
                        (p["lat"] - simp[-1]["lat"]) * DEG_M
                    )
                    if d >= MIN_SPACING_M or p is raw_pts[-1]:
                        simp.append(p)

                for p in simp:
                    wplist.append(OrderedDict([
                        ("waypointID", 0),
                        ("coordinate", {"latitude": p["lat"], "longitude": p["lon"], "altitude": _wp_alt(p["lat"], p["lon"], Altitude)}),
                        ("speed", cruise_speed),
                        ("eta", p["eta_ms"]),
                        ("ecf", 0.0),
                        ("nextWaypointID", 0),
                        ("waypointPassType", 1),
                        ("filmingProperty", _mk_filming(
                            operation_mode=OPMODE_HOLD,
                            sensor=SENSOR_EO_IR
                        )),
                    ]))

        # 1-C. 패킷 저장
        if wplist:
            packet = {
                "pathID": miss["pathID"],
                "aircraftID": aid,
                "wplist": wplist,
            }
            if full_sweep_coords:
                packet["fullSweepCoords"] = full_sweep_coords
                packet["fullSweepSearchSpeed"] = full_sweep_speed
                packet["fullSweepInterpPoints"] = full_sweep_interp_points
            packets.append(packet)

    # ────────────────────────── 3) WP ID · 링크 · ECF ────────────────
    prev_tail_by_aircraft: Dict[int, dict] = {}
    for pkt in packets:
        wps = pkt["wplist"]
        aid = int(pkt.get("aircraftID", 0))
        prev_tail_coord = prev_tail_by_aircraft.get(aid)
        if not wps:
            continue

        sweep_indices = [idx for idx, wp in enumerate(wps) if _has_line_search(wp)]
        entry_wp: OrderedDict | None = None
        entry_offset_m = SWEEP_ENTRY_OFFSET_M
        if len(sweep_indices) >= 2:
            first_idx = sweep_indices[0]
            second_idx = sweep_indices[1]
            first_coord = wps[first_idx].get("coordinate") or {}
            second_coord = wps[second_idx].get("coordinate") or {}

            lat0 = float(first_coord.get("latitude", 0.0))
            lon0 = float(first_coord.get("longitude", 0.0))
            lat1 = float(second_coord.get("latitude", lat0))
            lon1 = float(second_coord.get("longitude", lon0))
            vec_x, vec_y = llh_to_xy(lat1, lon1, lat0, lon0)
            norm = math.hypot(vec_x, vec_y)
            if norm >= 1.0:
                ux, uy = vec_x / norm, vec_y / norm
                entry_xy = (-ux * entry_offset_m, -uy * entry_offset_m)
                entry_lat, entry_lon = xy_to_llh(entry_xy[0], entry_xy[1], lat0, lon0)
                entry_coord = OrderedDict([
                    ("latitude", round(entry_lat, 6)),
                    ("longitude", round(entry_lon, 6)),
                    ("altitude", _wp_alt(entry_lat, entry_lon, first_coord.get("altitude", Altitude))),
                ])
                entry_wp = OrderedDict([
                    ("waypointID", 0),
                    ("coordinate", entry_coord),
                    ("speed", cruise_speed),
                    ("eta", 0),
                    ("ecf", 0.0),
                    ("nextWaypointID", 0),
                    ("waypointPassType", PASS_FLYBY),
                    ("filmingProperty", _mk_filming(
                        operation_mode=OPMODE_HOLD,
                        fov=ENTRY_HOLD_FOV_DEG,
                        sensor=SENSOR_EO_IR,
                        gimbal_pitch=ENTRY_HOLD_GIMBAL_PITCH,
                        gimbal_yaw=ENTRY_HOLD_GIMBAL_YAW,
                    )),
                ])

            first_wp = wps[first_idx]
            first_fp = first_wp.get("filmingProperty") or OrderedDict()
            first_line_search = deepcopy(first_fp.get("lineSearch") or {})
            first_coords = deepcopy(first_line_search.get("coordinateList") or [])
            first_search_speed = first_line_search.get("searchSpeed")
            first_interp_points = _interp_points_hint(first_line_search.get("interpolationPoints"))
            if first_search_speed is None:
                first_width = _avg_sweep_width_m(first_coords, points_per_line=first_interp_points)
                first_search_speed = spacing_based_search_speed(
                    sweep_len_m=first_width,
                    spacing_m=sweep_spacing_m,
                    cruise_speed_mps=cruise_speed,
                )
            if first_search_speed is None:
                first_search_speed = DEFAULT_SEARCH_SPEED
            records: list[dict] = []
            for idx in sweep_indices[1:]:
                wp = wps[idx]
                fp = wp.get("filmingProperty") or OrderedDict()
                ls = fp.get("lineSearch") or {}
                coords = deepcopy(ls.get("coordinateList") or [])
                if not coords:
                    continue
                search_speed = ls.get("searchSpeed")
                interp_points = _interp_points_hint(ls.get("interpolationPoints"))
                if search_speed is None:
                    width_m = _avg_sweep_width_m(coords, points_per_line=interp_points)
                    search_speed = spacing_based_search_speed(
                        sweep_len_m=width_m,
                        spacing_m=sweep_spacing_m,
                        cruise_speed_mps=cruise_speed,
                    )
                records.append({
                    "idx": idx,
                    "wp": wp,
                    "fp": fp,
                    "coord": wp.get("coordinate") or {},
                    "coords": coords,
                    "search_speed": search_speed,
                    "fov": fp.get("fieldOfView", FOV_DEG),
                    "interp_points": interp_points,
                })

            if first_coords and records:
                start_target = deepcopy(first_coords[0])
                first_fov = first_fp.get("fieldOfView", FOV_DEG)
                first_wp["filmingProperty"] = OrderedDict([
                    ("fieldOfView", first_fov),
                    ("sensorType", SENSOR_EO_IR),
                    ("operationMode", OPMODE_POINT),
                    ("coordinateOrientation", OrderedDict([
                        ("coordinate", OrderedDict([
                            ("latitude", float(start_target.get("latitude", 0.0))),
                            ("longitude", float(start_target.get("longitude", 0.0))),
                            ("altitude", 0),
                        ]))
                    ])),
                ])

                def _vector_between(coord_from: dict, coord_to: dict) -> tuple[float, float]:
                    lat_a = float(coord_from.get("latitude", 0.0))
                    lon_a = float(coord_from.get("longitude", 0.0))
                    lat_b = float(coord_to.get("latitude", lat_a))
                    lon_b = float(coord_to.get("longitude", lon_a))
                    return llh_to_xy(lat_b, lon_b, lat_a, lon_a)

                if SWEEP_MERGE_MODE == "all":
                    full_coords = pkt.get("fullSweepCoords") or []
                    full_speed = pkt.get("fullSweepSearchSpeed")
                    interp_points = pkt.get("fullSweepInterpPoints") or SWEEP_LINE_INTERP_POINTS
                    width_points = 2
                    if not full_coords:
                        full_coords = deepcopy(first_coords)
                        for record in records:
                            full_coords.extend(deepcopy(record["coords"]))
                    if full_speed is None:
                        full_width = _avg_sweep_width_m(full_coords, points_per_line=width_points)
                        full_speed = spacing_based_search_speed(
                            sweep_len_m=full_width,
                            spacing_m=sweep_spacing_m,
                            cruise_speed_mps=cruise_speed,
                        )
                    if full_speed is None:
                        full_speed = first_search_speed
                    if full_speed is None:
                        full_speed = DEFAULT_SEARCH_SPEED

                    rep = records[0]
                    rep_fp = rep["fp"]
                    rep_fp["fieldOfView"] = rep["fov"]
                    rep_fp["sensorType"] = SENSOR_EO_IR
                    rep_fp["operationMode"] = OPMODE_LINE
                    interpolated_coords = _interpolate_line_coords(
                        full_coords,
                        interp_points,
                    )
                    rep_fp["lineSearch"] = OrderedDict([
                        ("coordinateList", interpolated_coords),
                        ("searchSpeed", full_speed),
                        ("interpolationPoints", interp_points),
                    ])
                    for record in records[1:]:
                        record["wp"]["filmingProperty"] = {}
                else:
                    groups: list[list[int]] = []
                    if SWEEP_MERGE_MODE == "heading":
                        current_group: list[int] = [0]
                        for pos in range(1, len(records)):
                            prev_vec = _vector_between(records[pos - 2]["coord"], records[pos - 1]["coord"]) if pos >= 2 else None
                            curr_vec = _vector_between(records[pos - 1]["coord"], records[pos]["coord"])
                            angle = 0.0 if prev_vec is None else _angle_between(prev_vec, curr_vec)
                            if angle <= SWEEP_MERGE_HEADING_DEG:
                                current_group.append(pos)
                            else:
                                groups.append(current_group)
                                current_group = [pos]
                        groups.append(current_group)
                    elif SWEEP_MERGE_MODE == "curve":
                        coords_chain = [first_coord] + [record["coord"] for record in records]
                        signs = [0] * len(coords_chain)
                        for idx in range(1, len(coords_chain) - 1):
                            v1 = _vector_between(coords_chain[idx - 1], coords_chain[idx])
                            v2 = _vector_between(coords_chain[idx], coords_chain[idx + 1])
                            angle = _angle_between(v1, v2)
                            if angle <= SWEEP_MERGE_HEADING_DEG:
                                signs[idx] = 0
                            else:
                                cross = (v1[0] * v2[1]) - (v1[1] * v2[0])
                                signs[idx] = 1 if cross >= 0 else -1
                        for idx in range(1, len(signs) - 1):
                            if signs[idx] == 0 and signs[idx - 1] == signs[idx + 1] != 0:
                                signs[idx] = signs[idx - 1]
                        current_group = [0]
                        current_sign = signs[1] if len(signs) > 1 else 0
                        for pos in range(1, len(records)):
                            sign = signs[pos + 1]
                            if sign == current_sign:
                                current_group.append(pos)
                            else:
                                groups.append(current_group)
                                current_group = [pos]
                                current_sign = sign
                        groups.append(current_group)
                    else:
                        groups = [[idx] for idx in range(len(records))]

                    to_remove: list[int] = []
                    for g_idx, group in enumerate(groups):
                        rep_pos = group[-1]
                        rep = records[rep_pos]
                        rep_fp = rep["fp"]
                        merged_coords: list[dict] = []
                        merged_spans: list[float] = []
                        if g_idx == 0:
                            merged_coords.extend(deepcopy(first_coords))
                            merged_spans.extend(_collect_sweep_spans(first_coords, first_interp_points))
                        for pos in group:
                            coords_copy = deepcopy(records[pos]["coords"])
                            merged_coords.extend(coords_copy)
                            merged_spans.extend(_collect_sweep_spans(
                                records[pos]["coords"],
                                records[pos].get("interp_points"),
                            ))
                        merged_width = None
                        if merged_spans:
                            merged_width = round(sum(merged_spans) / len(merged_spans), 2)
                        rep_speed = spacing_based_search_speed(
                            sweep_len_m=merged_width,
                            spacing_m=sweep_spacing_m,
                            cruise_speed_mps=cruise_speed,
                        )
                        if rep_speed is None:
                            rep_speed = rep["search_speed"]
                        if rep_speed is None:
                            rep_speed = first_search_speed
                        rep_fp["fieldOfView"] = rep["fov"]
                        rep_fp["sensorType"] = SENSOR_EO_IR
                        rep_fp["operationMode"] = OPMODE_LINE
                        interpolated_coords = _interpolate_line_coords(
                            merged_coords,
                            SWEEP_LINE_INTERP_POINTS,
                        )
                        rep_fp["lineSearch"] = OrderedDict([
                            ("coordinateList", interpolated_coords),
                            ("searchSpeed", rep_speed),
                            ("interpolationPoints", SWEEP_LINE_INTERP_POINTS),
                        ])
                        for pos in group:
                            if pos != rep_pos:
                                to_remove.append(records[pos]["idx"])

                    for idx in sorted(to_remove, reverse=True):
                        del wps[idx]

            if entry_wp is not None:
                wps.insert(0, entry_wp)

        for wp in wps:
            wp["waypointID"] = wp_alloc.alloc()

        for idx in range(len(wps) - 1):
            wps[idx]["nextWaypointID"] = wps[idx + 1]["waypointID"]
        wps[-1]["nextWaypointID"] = 0

        for wp in wps:
            if wp.get("waypointPassType") == PASS_LOITER:
                if not wp.get("filmingProperty"):
                    wp["filmingProperty"] = _mk_point_filming_for_coord(wp.get("coordinate") or {})
                wp["loiterProperty"] = OrderedDict([
                    ("radius", LOITER_RADIUS_M),
                    ("direction", LOITER_DIRECTION),
                    ("time", LOITER_TIME_S),
                    ("speed", LOITER_SPEED_MPS),
                ])

        _annotate_eta_ms_inplace(wps, default_speed_mps=cruise_speed)

        total_eta = sum(max(0, int(w.get("eta", 0))) for w in wps) or 1
        cum = 0
        for wp in wps:
            step_eta = max(0, int(wp.get("eta", 0)))
            cum += step_eta
            wp["ecf"] = round(min(cum / total_eta, 1.0), 2)

        wps[-1]["ecf"] = 1.0

        last_coord = wps[-1].get("coordinate") or {}
        if last_coord:
            prev_tail_by_aircraft[aid] = dict(last_coord)

    # ────────────────────────── 4) 최종 조립 ─────────────────────────
    result = []
    for pkt in packets:
        result.append(OrderedDict([
            ("timestamp", now_ms),
            ("Source", _sw_code()),
            ("pathID", pkt["pathID"]),
            ("aircraftID", pkt["aircraftID"]),
            ("isFormationFlight", False),
            ("waypointList", pkt["wplist"]),
        ]))
    return result





