from __future__ import annotations
import os
import math
from collections import OrderedDict
from copy import deepcopy
from typing import List, Tuple                       # ??異붽?
from .mission_helpers import now_ms_since_2000, terrain_elev
from .id_allocator import next_waypoint_id as _next_waypoint_id

try:
    from ..config import DEFAULT_SWEEP_SEPARATION_M
except ImportError:
    try:
        from config import DEFAULT_SWEEP_SEPARATION_M  # type: ignore
    except ImportError:
        from modules.mission_planning.MissionPlanner.config import DEFAULT_SWEEP_SEPARATION_M  # type: ignore
from UAV_missionPlanning import UAVMissionPlanner
from Aisle_Sweep_CPP_shoot_plan import RectanglePath
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


# ?? ???alias (媛?낆슜) ?????????????????????????????
Point = Tuple[float, float]
Line  = Tuple[Point, Point]

# ?? 怨좎젙 ?곸닔 ???????????????????????????????????????????
FOV_DEG         = 15
SWEEP_ENTRY_OFFSET_M = 1500.0
SWEEP_MERGE_HEADING_DEG = 5
Altitude = 700

SENSOR_NONE     = 0
SENSOR_EO_IR    = 1        # ?? EO/IR ?쇱꽌

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
    0: "?놁쓬", 
    1: "?쒖쟻異붿쟻", # ID ?꾩슂 -> ?⑦꽩? 1
    2: "?쒖쟻怨듦꺽", # ?ш린留?-> ID ?꾩슂, ?⑦꽩? 2踰?    3: "?곸뿭?섏깋", # ?뺥빐吏??잛닔留뚰겮 ?곸뿭?????珥ъ쁺 怨꾪쉷 ?몄? -> ?⑦꽩? 3,4,5,6,7,8,9
    4: "?곸뿭寃쎄퀎", # ?뺥빐吏??쒓컙?숈븞 ?곸뿭?????珥ъ쁺 怨꾪쉷 ?몄?-> ?⑦꽩? 3,4,5,6,7,8,9
    5: "醫뚰몴?먯젙李?, # ?먮쭔 蹂대뒗 ?뺥깭 -> ?⑦꽩? 1??????    6: "?듬줈?뺤같", # 以묒떖??湲곗? 醫뚯슦 ?ㅼ쐲 -> ?⑦꽩? 4踰?    7: "?대룞", # ?대룞 怨꾪쉷 ?몄? -> 移대찓?쇰뱾? 吏곹븯諛⑹쑝濡?怨좎젙
    8: "?꾪샇", # TBD
    9: "??꾪룓", # ?ш린???좎? ?ъ????ㅼ젙?댁쨲
}

PATTERN_DISPATCH = {
    0:  "?놁쓬",
    1:  "?쒖쟻以묒떖?좏쉶쨌異붿쟻", # 0303??Operation Mode瑜?3踰덉쑝濡? WaypointPassType媛 3 -> 鍮꾪뻾 怨꾪쉷? LoiterProperty 媛 ?앹꽦?섏뼱???? ?좏쉶諛섍꼍? 1000?쇰줈 怨좎젙?섍퀬 Direction??0 : None, 1 : CW, 2 : CCW -> Time? 5遺?怨좎젙 -> Speed??40怨좎젙
    2:  "??꾪룓?꾧났寃?, #?좎씤湲곗뿉???좉쾬 TBD
    3:  "吏곹븯諛?BF珥ъ쁺", # 吏곹븯諛?珥ъ쁺?쇰줈 怨좎젙?섍퀬 鍮꾪뻾怨꾪쉷 ?뚭퀬由ъ쬁??CPP ?뚭퀬由ъ쬁?쇰줈 ?泥댄빐??珥ъ쁺 以묒떖??= 鍮꾪뻾 寃쎈줈???곹솴?? ???쇱씤李띻퀬 ?ㅼ쓬?쇱씤??李띻린?꾪빐 ?꾨뒗 ?좏쉶 遺遺꾩씠 以묒슂??    4:  "?닿꺽-BF珥ъ쁺", # 吏湲덇낵 ?숈씪???곹깭
    5:  "援ш컙?뺣났-BF珥ъ쁺", #TBD
    6:  "?좏삎諛섎났二쇱궗-BF珥ъ쁺", #TBD
    7:  "吏곸긽怨듭닚?뚯눋??, #TBD
    8:  "援ш컙以묒떖醫낅떒-?좏삎諛섎났二쇱궗珥ъ쁺", #TBD
    9:  "援ш컙以묒떖醫낅떒-?먮룞諛섎났二쇱궗珥ъ쁺", #TBD
    10: "紐⑹쟻吏?대룞", #移대찓?쇰? 吏곹븯諛⑹쑝濡?怨좎젙??-> 吏湲덇낵 ?숈씪?섍쾶 ?대룞 寃쎈줈 怨꾪쉷??    11: "??곸뾼??, #TBD
    12: "??꾪룓", #?대떦 ?먯뿉??怨좊룄瑜?理쒕????덉쟾怨좊룄源뚯?) ??떠???⑥뼱?덉쓬(?좎씤湲곕쭔 ?ъ슜)
}

def _dem_alt(lat: float, lon: float) -> int:
    """DEM 湲곕컲 怨좊룄瑜??뺤닔(m)濡?諛섑솚."""
    return int(round(terrain_elev(lat, lon)))

def _poly_sweeps_general(
    poly_llh: list[tuple[float,float]],
    anchor_llh: tuple[float,float],
    bearing_deg: float,
    fov_deg: float,
    separation_m: float,
) -> list[dict]:
    """
    ??Convex polygon LLH ??bearing 怨??됲뻾?????ㅼ쐲 ??lineSearch.coordinateList
    ??由ы꽩: [{"latitude":?? "longitude":?? "altitude":5}, ??  (吏앹닔/??????쇱씤)
    """
    lat0, lon0 = poly_llh[0]
    poly_xy = [llh_to_xy(lat, lon, lat0, lon0) for lat, lon in poly_llh]
    anchor_xy = llh_to_xy(anchor_llh[0], anchor_llh[1], lat0, lon0)

    th = math.radians(bearing_deg)
    nx, ny =  math.sin(th), -math.cos(th)            # bearing 怨?吏곴컖???몃?
    proj = [nx*x + ny*y for x, y in poly_xy]
    d_min, d_max = min(proj), max(proj)

    spacing_m = 2*separation_m*math.tan(math.radians(fov_deg)/2)
    n_strip   = max(int(math.ceil((d_max-d_min)/spacing_m))+1, 3)

    # ?ㅺ컖??edge 由ъ뒪??    edges = [(poly_xy[i], poly_xy[(i+1)%len(poly_xy)]) for i in range(len(poly_xy))]
    coord_list: list[dict] = []

    for k in range(n_strip):
        d = d_min + (d_max-d_min)*k/(n_strip-1)
        # 吏곸꽑: n쨌x = d  ?? (?뭤y, nx) 諛⑺뼢 unit 踰≫꽣
        hits = []
        for a,b in edges:
            f1, f2 = nx*a[0]+ny*a[1]-d, nx*b[0]+ny*b[1]-d
            if f1*f2 <= 0 and f1 != f2:
                t = f1/(f1-f2)
                x = a[0] + t*(b[0]-a[0]);   y = a[1] + t*(b[1]-a[1])
                hits.append((x,y))
        if len(hits) >= 2:
            # ?ㅼ쐲 ?좎? 援먯감????媛쒕? ?곌껐
            p1, p2 = hits[0], hits[1]
            for x,y in (p1,p2):
                lat, lon = xy_to_llh(x,y,lat0,lon0)
                coord_list.append({
                    "latitude": lat,
                    "longitude": lon,
                    "altitude": _dem_alt(lat, lon),
                })
    return coord_list

def _cpp_line_search(
        rect_llh: list[tuple[float,float]],
        anchor_llh: tuple[float,float],
        separation_m: float,
        uav_height_m: float = 610.0,
        fov_deg: float = FOV_DEG,
) -> list[dict]:
    """
    ??4-??吏곸궗媛곹삎 LLH ??RectanglePath CPP ?ㅼ쐲 ??lineSearch.coordinateList ?앹꽦
    ??return: [{"latitude":?? "longitude":?? "altitude":5}, ??  (吏앹닔쨌??????쇱씤)
    """
    lat0, lon0 = rect_llh[0]               # XY 蹂??湲곗?
    rect_xy = [llh_to_xy(lat, lon, lat0, lon0) for lat, lon in rect_llh]
    anchor_xy = llh_to_xy(anchor_llh[0], anchor_llh[1], lat0, lon0)

    rp = RectanglePath(point=anchor_xy,
                       rectangle_vertices=rect_xy,
                       separation_dist=separation_m,
                       UAV_height=uav_height_m)

    coord_list: list[dict] = []
    # rp.path ??[p0, p1, p2, p3, ...] ??2?먯뵫 ???ㅼ쐲
    for i in range(0, len(rp.path) - 1, 2):
        s_xy, e_xy = rp.path[i], rp.path[i + 1]
        s_lat, s_lon = xy_to_llh(*s_xy, lat0, lon0)
        e_lat, e_lon = xy_to_llh(*e_xy, lat0, lon0)
        coord_list.extend([
            {"latitude": s_lat, "longitude": s_lon, "altitude": _dem_alt(s_lat, s_lon)},
            {"latitude": e_lat, "longitude": e_lon, "altitude": _dem_alt(e_lat, e_lon)},
        ])
    return coord_list

def _mk_filming(operation_mode: int = OPMODE_NONE,
                fov: float = FOV_DEG,
                sensor: int = SENSOR_NONE,
                line_search: OrderedDict | None = None,
                gimbal_pitch: float | None = None,
                gimbal_yaw: float | None = None) -> OrderedDict:
    """
    filmingProperty 釉붾줉 ?앹꽦 ?ы띁

    ??OPMODE_LINE  ??lineSearch ?꾨뱶 ?쎌엯  
    ??OPMODE_HOLD ??aircraftFixed  釉붾줉 ?쎌엯
                  ??gimbalPitch / gimbalYaw ?ы븿
    """
    fp = OrderedDict([
        ("fieldOfView",   fov),
        ("sensorType",    sensor),
        ("operationMode", operation_mode),
    ])

    # ???좏삎 ?먯깋(lineSearch)
    if operation_mode == OPMODE_LINE and line_search is not None:
        fp["lineSearch"] = line_search

    # ??怨좎젙 珥ъ쁺(aircraftFixed + 吏먮쾶 媛곷룄)
    if operation_mode == OPMODE_HOLD:
        # 湲곕낯媛?  吏곹븯諛?-90째, Yaw 0째
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


def _avg_sweep_width_m(coords: list[dict]) -> float | None:
    """Pair coordinates (0-1, 2-3, ...) to estimate the average sweep span in meters."""
    if not coords or len(coords) < 2:
        return None

    spans: list[float] = []
    for idx in range(0, len(coords) - 1, 2):
        start = coords[idx]
        end = coords[idx + 1]
        lat0 = float(start.get("latitude", 0.0))
        lon0 = float(start.get("longitude", 0.0))
        lat1 = float(end.get("latitude", lat0))
        lon1 = float(end.get("longitude", lon0))
        dx, dy = llh_to_xy(lat1, lon1, lat0, lon0)
        dist = math.hypot(dx, dy)
        if dist > 0.0:
            spans.append(dist)

    if not spans:
        return None
    return round(sum(spans) / len(spans), 2)


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
    0203?먯꽌 湲곗껜ID?믪쥖??留?援ъ꽦.
    return: (to_map, ho_map)  媛?媛믪? {aid: {"latitude":..,"longitude":..,"altitude":..}}
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
                "altitude": int(float(c.get("altitude", Altitude))),
            }
    ho_map = {}
    for it in ref0203.get("handOverInfoList", []) or []:
        aid = it.get("aircraftID")
        c   = it.get("coordinate") or {}
        if isinstance(aid, int) and 1 <= aid <= 6 and "latitude" in c and "longitude" in c:
            ho_map[aid] = {
                "latitude": float(c.get("latitude", 0.0)),
                "longitude": float(c.get("longitude", 0.0)),
                "altitude": int(float(c.get("altitude", Altitude))),
            }
    return to_map, ho_map


def _eta_ms_llh(c1: dict, c2: dict, speed_mps: float) -> int:
    """
    媛꾨떒??援щ㈃ 洹쇱궗濡?嫄곕━?묮TA(s) 怨꾩궛. speed_mps<=0?대㈃ 0.
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

    # ?? ?곸닔 ???????????????????????????????????????????????
    SENSOR, OPMODE = 1, 2
    DEFAULT_SEARCH_SPEED = round(cruise_speed * 5, 2)
    ALT_M = DEFAULT_SWEEP_SEPARATION_M
    DEG_M = 111_132
    # ?? 留덉?留됱젏??POINT 珥ъ쁺 釉붾줉 ?앹꽦湲??????????????????
    def _mk_point_filming_for_coord(coord: dict) -> OrderedDict:
        lat = float(coord.get("latitude", 0.0))
        lon = float(coord.get("longitude", 0.0))
        return OrderedDict([
            ("fieldOfView", 66.638654),
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

    # ?????????????????????????? 1) 誘몄뀡 ???⑦궥 ??????????????????????????
    for miss in missions:
        aid = miss["aircraftID"]
        if aid not in (4, 5, 6):
            continue

        info = miss["individualMissionInfo"]
        mtype = info.get("individualMissionType")
        wplist: list[OrderedDict] = []

        # 1-A. ?듬줈?뺤같 / ?곸뿭?섏깋 (type 3쨌4쨌6)
        if mtype in (3, 4, 6):
            base, width = None, 100.0

            # (i) lineList ??corridor
            if info.get("lineList"):
                line = info["lineList"][0]
                width = line["width"]
                base = [(c["latitude"], c["longitude"]) for c in line["coordinateList"]]

            # (ii) areaList
            elif info.get("areaList"):
                pts = [(p["latitude"], p["longitude"]) for p in info["areaList"][0]["coordinateList"]]

                bearing = miss.get("bearing_deg", 90.0)
                th = math.radians(bearing)
                ux_b, uy_b = math.sin(th), math.cos(th)

                prev_pt = miss.get("prevPoint", pts[0])

                if len(pts) == 4:
                    coord_list = _cpp_line_search(
                        rect_llh=pts, anchor_llh=prev_pt, separation_m=ALT_M,
                        uav_height_m=610.0, fov_deg=FOV_DEG,
                    )
                else:
                    coord_list = _poly_sweeps_general(
                        poly_llh=pts, anchor_llh=prev_pt, bearing_deg=bearing,
                        fov_deg=FOV_DEG, separation_m=ALT_M,
                    )
                if not coord_list:
                    continue

                lines = [coord_list[i:i+2] for i in range(0, len(coord_list), 2)]
                if not lines:
                    continue

                # 吏㏃? ?ㅼ쐲 ?꾪꽣
                MIN_SWEEP_LEN = 3.0
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
                    off_xy = (mid_xy[0] - ux_b * ALT_M, mid_xy[1] + uy_b * ALT_M)
                    last_off_xy = off_xy
                    off_lat, off_lon = xy_to_llh(*off_xy, lat0, lon0)

                    sweep = [e, s] if idx % 2 else [s, e]
                    sweep_speed = (_avg_sweep_width_m(sweep) or DEFAULT_SEARCH_SPEED)*2

                    wplist.append(OrderedDict([
                        ("waypointID", 0),
                        ("coordinate", {"latitude": off_lat, "longitude": off_lon, "altitude": Altitude}),
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

                # ??醫낅즺 WP (Loiter + POINT 珥ъ쁺)
                if len(wplist) >= 2:
                    last_wp = wplist[-1]["coordinate"]
                    prev_wp = wplist[-2]["coordinate"]
                    last_xy = llh_to_xy(last_wp["latitude"], last_wp["longitude"], lat0, lon0)
                    prev_xy = llh_to_xy(prev_wp["latitude"], prev_wp["longitude"], lat0, lon0)
                    vx, vy = last_xy[0] - prev_xy[0], last_xy[1] - prev_xy[1]
                    vlen = math.hypot(vx, vy) or 1.0
                    ux_end, uy_end = vx / vlen, vy / vlen
                    end_xy = (last_xy[0] + ux_end * 200.0, last_xy[1] + uy_end * 200.0)
                    end_lat, end_lon = xy_to_llh(*end_xy, lat0, lon0)
                    end_coord = {"latitude": end_lat, "longitude": end_lon, "altitude": Altitude}

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
                elif len(wplist) == 1 and last_off_xy is not None:
                    end_xy = (last_off_xy[0] + ux_b * 200.0, last_off_xy[1] + uy_b * 200.0)
                    end_lat, end_lon = xy_to_llh(*end_xy, lat0, lon0)
                    end_coord = {"latitude": end_lat, "longitude": end_lon, "altitude": Altitude}

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
                    fov_deg=FOV_DEG, cruise_speed=cruise_speed, crs="lla",
                )

                last_anchor_xy: tuple[float, float] | None = None
                last_first_xy: tuple[float, float] | None = None
                for idx, (anchor_xy, sw) in enumerate(zip(planner.orange_pts, planner.sweeps)):
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

                    coord_speed = _avg_sweep_width_m(coord_list) or DEFAULT_SEARCH_SPEED

                    wplist.append(OrderedDict([
                        ("waypointID", 0),
                        ("coordinate", {"latitude": w_lat, "longitude": w_lon, "altitude": Altitude}),
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

                if last_anchor_xy and last_first_xy:
                    vx, vy = last_first_xy[0] - last_anchor_xy[0], last_first_xy[1] - last_anchor_xy[1]
                    vlen = math.hypot(vx, vy) or 1.0
                    ux_c, uy_c = vx / vlen, vy / vlen
                    end_xy = (last_anchor_xy[0] + ux_c * 300.0, last_anchor_xy[1] + uy_c * 300.0)
                    end_lat, end_lon = planner._proj_back(end_xy[0], end_xy[1])[::-1]
                    end_coord = {"latitude": end_lat, "longitude": end_lon, "altitude": Altitude}

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

        # 1-B. ?대룞 誘몄뀡 (type 7)
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
                    ("coordinate", {"latitude": lat, "longitude": lon, "altitude": Altitude}),
                    ("speed", cruise_speed), ("eta", 0), ("ecf", 1.0),
                    ("nextWaypointID", 0), ("waypointPassType", 1),
                    ("filmingProperty", {}),
                ]))
            elif len(base) >= 2:
                raw_pts = UAVMissionPlanner.plan_route_only(
                    base, cruise_speed, heading_tol_deg=turn_step_deg, store=False
                )
                MIN_SPACING_M = 200.0
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
                        ("coordinate", {"latitude": p["lat"], "longitude": p["lon"], "altitude": Altitude}),
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

        # 1-C. ?⑦궥 ???        if wplist:
            packets.append({
                "pathID": miss["pathID"],
                "aircraftID": aid,
                "wplist": wplist,
            })

    # ?????????????????????????? 3) WP ID 쨌 留곹겕 쨌 ECF ????????????????
    for pkt in packets:
        wps = pkt["wplist"]
        if not wps:
            continue

        sweep_indices = [idx for idx, wp in enumerate(wps) if _has_line_search(wp)]
        entry_wp: OrderedDict | None = None
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
                entry_xy = (-ux * SWEEP_ENTRY_OFFSET_M, -uy * SWEEP_ENTRY_OFFSET_M)
                entry_lat, entry_lon = xy_to_llh(entry_xy[0], entry_xy[1], lat0, lon0)
                entry_coord = OrderedDict([
                    ("latitude", round(entry_lat, 6)),
                    ("longitude", round(entry_lon, 6)),
                    ("altitude", first_coord.get("altitude", Altitude)),
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
                        fov=10.0,
                        sensor=SENSOR_EO_IR,
                        gimbal_pitch=-90.0,
                        gimbal_yaw=0.0,
                    )),
                ])

            first_wp = wps[first_idx]
            first_fp = first_wp.get("filmingProperty") or OrderedDict()
            first_line_search = deepcopy(first_fp.get("lineSearch") or {})
            first_coords = deepcopy(first_line_search.get("coordinateList") or [])
            first_search_speed = first_line_search.get("searchSpeed")
            if first_search_speed is None:
                first_search_speed = _avg_sweep_width_m(first_coords)
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
                if search_speed is None:
                    search_speed = _avg_sweep_width_m(coords)
                records.append({
                    "idx": idx,
                    "wp": wp,
                    "fp": fp,
                    "coord": wp.get("coordinate") or {},
                    "coords": coords,
                    "search_speed": search_speed,
                    "fov": fp.get("fieldOfView", FOV_DEG),
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

                groups: list[list[int]] = []
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

                to_remove: list[int] = []
                for g_idx, group in enumerate(groups):
                    rep_pos = group[-1]
                    rep = records[rep_pos]
                    rep_fp = rep["fp"]
                    merged_coords: list[dict] = []
                    if g_idx ==0:
                        merged_coords.extend(deepcopy(first_coords))
                    for pos in group:
                        merged_coords.extend(deepcopy(records[pos]["coords"]))
                    rep_speed = _avg_sweep_width_m(merged_coords)
                    if rep_speed is None:
                        rep_speed = rep["search_speed"]
                    if rep_speed is None:
                        rep_speed = first_search_speed
                    rep_fp["fieldOfView"] = rep["fov"]
                    rep_fp["sensorType"] = SENSOR_EO_IR
                    rep_fp["operationMode"] = OPMODE_LINE
                    rep_fp["lineSearch"] = OrderedDict([
                        ("coordinateList", merged_coords),
                        ("searchSpeed", rep_speed),
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
                    ("radius", 800),
                    ("direction", 1),
                    ("time", 30),
                    ("speed", 30),
                ])

        _annotate_eta_ms_inplace(wps, default_speed_mps=cruise_speed)

        total_eta = sum(max(0, int(w.get("eta", 0))) for w in wps) or 1
        cum = 0
        for wp in wps:
            step_eta = max(0, int(wp.get("eta", 0)))
            cum += step_eta
            wp["ecf"] = round(min(cum / total_eta, 1.0), 2)

        wps[-1]["ecf"] = 1.0

    # ?????????????????????????? 4) 理쒖쥌 議곕┰ ?????????????????????????
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






