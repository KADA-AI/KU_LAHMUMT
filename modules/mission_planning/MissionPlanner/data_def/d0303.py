from __future__ import annotations
import os
import math
from collections import OrderedDict
from typing import List, Tuple                       # ★ 추가
from .mission_helpers import now_ms_since_2000
from UAV_missionPlanning import UAVMissionPlanner
from Aisle_Sweep_CPP_shoot_plan import RectanglePath
from .coord_transform import llh_to_xy, xy_to_llh



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
FOV_DEG         = 5.5

SENSOR_NONE     = 0
SENSOR_EO_IR    = 1        # 예) EO/IR 센서

# WaypointPassType
PASS_NONE = 0
PASS_FLYBY = 1
PASS_FLYOVER = 2

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

    spacing_m = 2*separation_m*math.tan(math.radians(fov_deg)/2)
    n_strip   = max(int(math.ceil((d_max-d_min)/spacing_m))+1, 3)

    # 다각형 edge 리스트
    edges = [(poly_xy[i], poly_xy[(i+1)%len(poly_xy)]) for i in range(len(poly_xy))]
    coord_list: list[dict] = []

    for k in range(n_strip):
        d = d_min + (d_max-d_min)*k/(n_strip-1)
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
                coord_list.append({"latitude":lat,"longitude":lon,"altitude":5})
    return coord_list

def _cpp_line_search(
        rect_llh: list[tuple[float,float]],
        anchor_llh: tuple[float,float],
        separation_m: float,
        uav_height_m: float = 610.0,
        fov_deg: float = 5.5,
) -> list[dict]:
    """
    ▸ 4-점 직사각형 LLH → RectanglePath CPP 스윕 → lineSearch.coordinateList 생성
    ▸ return: [{"latitude":…, "longitude":…, "altitude":5}, …]  (짝수·홀수=한 라인)
    """
    lat0, lon0 = rect_llh[0]               # XY 변환 기준
    rect_xy = [llh_to_xy(lat, lon, lat0, lon0) for lat, lon in rect_llh]
    anchor_xy = llh_to_xy(anchor_llh[0], anchor_llh[1], lat0, lon0)

    rp = RectanglePath(point=anchor_xy,
                       rectangle_vertices=rect_xy,
                       separation_dist=separation_m,
                       UAV_height=uav_height_m)

    coord_list: list[dict] = []
    # rp.path 는 [p0, p1, p2, p3, ...] — 2점씩 한 스윕
    for i in range(0, len(rp.path) - 1, 2):
        s_xy, e_xy = rp.path[i], rp.path[i + 1]
        s_lat, s_lon = xy_to_llh(*s_xy, lat0, lon0)
        e_lat, e_lon = xy_to_llh(*e_xy, lat0, lon0)
        coord_list.extend([
            {"latitude": s_lat, "longitude": s_lon, "altitude": 5},
            {"latitude": e_lat, "longitude": e_lon, "altitude": 5},
        ])
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
                  ↳ GimbalPitch / GimbalYaw 포함
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
            ("GimbalPitch", gimbal_pitch),
            ("GimbalYaw",   gimbal_yaw),
        ])

    return fp

class _WPAllocator:
    def __init__(self, start: int = 1) -> None:
        self._next = start

    def alloc(self) -> int:
        if self._next > 65535:
            raise RuntimeError("WaypointID pool exhausted")
        wid = self._next
        self._next += 1
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
                "altitude": int(float(c.get("altitude", 1200))),
            }
    ho_map = {}
    for it in ref0203.get("handOverInfoList", []) or []:
        aid = it.get("aircraftID")
        c   = it.get("coordinate") or {}
        if isinstance(aid, int) and 1 <= aid <= 6 and "latitude" in c and "longitude" in c:
            ho_map[aid] = {
                "latitude": float(c.get("latitude", 0.0)),
                "longitude": float(c.get("longitude", 0.0)),
                "altitude": int(float(c.get("altitude", 1200))),
            }
    return to_map, ho_map


def _eta_ms_llh(c1: dict, c2: dict, speed_mps: float) -> int:
    """
    간단한 구면 근사로 거리→ETA(ms) 계산. speed_mps<=0이면 0.
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
        return int(round(dist_m / speed_mps * 1000.0))
    return 0

def build_flight_plans(
    missions: list[dict],
    wp_alloc: _WPAllocator | None = None,
    cruise_speed: float = 40.0,
    turn_step_deg: float = 45.0,
) -> list[dict]:
    wp_alloc = wp_alloc or _WPAllocator()
    now_ms = now_ms_since_2000()

    # ── 상수 ───────────────────────────────────────────────
    FOV_DEG, SENSOR, OPMODE = 5.5, 1, 2
    SEARCH_SPEED = round(cruise_speed * 2.8955, 2)
    ALT_M = 850.0
    DEG_M = 111_132
    GROUP_K = 5

    # ── 마지막점용 POINT 촬영 블록 생성기 ─────────────────
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

    # ────────────────────────── 1) 미션 → 패킷 ──────────────────────────
    for miss in missions:
        aid = miss["aircraftID"]
        if aid not in (4, 5, 6):
            continue

        info = miss["individualMissionInfo"]
        mtype = info.get("individualMissionType")
        wplist: list[OrderedDict] = []

        # 1-A. 통로정찰 / 영역수색 (type 3·4·6)
        if mtype in (3, 4, 6):
            base, width = None, 100.0

            # (i) lineList → corridor
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

                # 짧은 스윕 필터
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

                # ➊ 그룹화
                groups: list[dict] = []
                for idx, ln in enumerate(lines):
                    gidx = idx // GROUP_K
                    if len(groups) <= gidx:
                        groups.append({"segments": []})
                    groups[gidx]["segments"].append(ln)

                # ➋ 그룹별 앵커 + lineSearch
                last_off_xy: tuple[float, float] | None = None
                for g in groups:
                    segs = g["segments"]

                    s_lat, s_lon = segs[0][0]["latitude"], segs[0][0]["longitude"]
                    e_lat, e_lon = segs[0][1]["latitude"], segs[0][1]["longitude"]
                    s_xy = llh_to_xy(s_lat, s_lon, lat0, lon0)
                    e_xy = llh_to_xy(e_lat, e_lon, lat0, lon0)
                    mid_xy = ((s_xy[0] + e_xy[0]) / 2, (s_xy[1] + e_xy[1]) / 2)
                    off_xy = (mid_xy[0] - ux_b * ALT_M, mid_xy[1] + uy_b * ALT_M)
                    last_off_xy = off_xy
                    off_lat, off_lon = xy_to_llh(*off_xy, lat0, lon0)

                    coord_list = []
                    for order, sw in enumerate(segs, 1):
                        s, e = (sw[1], sw[0]) if order % 2 == 0 else sw
                        coord_list.extend([s, e])

                    wplist.append(OrderedDict([
                        ("waypointID", 0),
                        ("coordinate", {"latitude": off_lat, "longitude": off_lon, "altitude": 1200}),
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
                                ("searchSpeed", SEARCH_SPEED),
                            ]),
                        )),
                    ]))

                # ➌ 종료 WP (Loiter + POINT 촬영)
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
                    end_coord = {"latitude": end_lat, "longitude": end_lon, "altitude": 1200}

                    wplist.append(OrderedDict([
                        ("waypointID", 0),
                        ("coordinate", end_coord),
                        ("speed", cruise_speed),
                        ("eta", 0),
                        ("ecf", 0.0),
                        ("nextWaypointID", 0),
                        ("waypointPassType", PASS_FLYOVER),
                        ("filmingProperty", _mk_point_filming_for_coord(end_coord)),
                    ]))
                elif len(wplist) == 1 and last_off_xy is not None:
                    end_xy = (last_off_xy[0] + ux_b * 200.0, last_off_xy[1] + uy_b * 200.0)
                    end_lat, end_lon = xy_to_llh(*end_xy, lat0, lon0)
                    end_coord = {"latitude": end_lat, "longitude": end_lon, "altitude": 1200}

                    wplist.append(OrderedDict([
                        ("waypointID", 0),
                        ("coordinate", end_coord),
                        ("speed", cruise_speed),
                        ("eta", 0),
                        ("ecf", 0.0),
                        ("nextWaypointID", 0),
                        ("waypointPassType", PASS_FLYOVER),
                        ("filmingProperty", _mk_point_filming_for_coord(end_coord)),
                    ]))

            # (iii) Corridor-planner
            if base and len(base) >= 2:
                planner = UAVMissionPlanner(
                    base, corridor_width=width, separation=ALT_M,
                    fov_deg=10, cruise_speed=cruise_speed, crs="lla",
                )

                groups: list[dict] = []
                for idx, (pt, sw) in enumerate(zip(planner.orange_pts, planner.sweeps)):
                    gidx = idx // GROUP_K
                    if len(groups) <= gidx:
                        groups.append({"anchor": pt, "segments": []})
                    groups[gidx]["segments"].append(sw)

                last_anchor_xy: tuple[float, float] | None = None
                first_line_xy: tuple[float, float] | None = None
                for g in groups:
                    anchor_xy = g["anchor"]
                    w_lat, w_lon = planner._proj_back(anchor_xy[0], anchor_xy[1])[::-1]

                    coord_list = []
                    for order, sw in enumerate(g["segments"], 1):
                        s_xy, e_xy = (sw[1], sw[0]) if order % 2 == 0 else sw
                        s_lat, s_lon = planner._proj_back(s_xy[0], s_xy[1])[::-1]
                        e_lat, e_lon = planner._proj_back(e_xy[0], e_xy[1])[::-1]
                        coord_list.extend([
                            {"latitude": s_lat, "longitude": s_lon, "altitude": 5},
                            {"latitude": e_lat, "longitude": e_lon, "altitude": 5},
                        ])
                        if first_line_xy is None:
                            first_line_xy = s_xy

                    wplist.append(OrderedDict([
                        ("waypointID", 0),
                        ("coordinate", {"latitude": w_lat, "longitude": w_lon, "altitude": 1200}),
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
                                ("searchSpeed", SEARCH_SPEED),
                            ]),
                        )),
                    ]))

                    last_anchor_xy = anchor_xy
                    first_line_xy = None

                if last_anchor_xy:
                    s0_xy = groups[-1]["segments"][0][0]
                    vx, vy = s0_xy[0] - last_anchor_xy[0], s0_xy[1] - last_anchor_xy[1]
                    vlen = math.hypot(vx, vy) or 1.0
                    ux_c, uy_c = vx / vlen, vy / vlen
                    end_xy = (last_anchor_xy[0] + ux_c * 300.0, last_anchor_xy[1] + uy_c * 300.0)
                    end_lat, end_lon = planner._proj_back(end_xy[0], end_xy[1])[::-1]
                    end_coord = {"latitude": end_lat, "longitude": end_lon, "altitude": 1200}

                    wplist.append(OrderedDict([
                        ("waypointID", 0),
                        ("coordinate", end_coord),
                        ("speed", cruise_speed),
                        ("eta", 0),
                        ("ecf", 0.0),
                        ("nextWaypointID", 0),
                        ("waypointPassType", PASS_FLYOVER),
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
                    ("coordinate", {"latitude": lat, "longitude": lon, "altitude": 1200}),
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
                        ("coordinate", {"latitude": p["lat"], "longitude": p["lon"], "altitude": 1200}),
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
            packets.append({
                "pathID": miss["pathID"],
                "aircraftID": aid,
                "wplist": wplist,
            })

    # ────────────────────────── 3) WP ID · 링크 · ECF ────────────────
    for pkt in packets:
        for wp in pkt["wplist"]:
            wp["waypointID"] = wp_alloc.alloc()

        total_eta = sum(w["eta"] for w in pkt["wplist"]) or 1
        cum = 0
        for idx, wp in enumerate(pkt["wplist"]):
            cum += wp["eta"]
            wp["ecf"] = round(cum / total_eta, 2)
            wp["nextWaypointID"] = (pkt["wplist"][idx + 1]["waypointID"]
                                     if idx + 1 < len(pkt["wplist"]) else 0)

            # PASS_FLYOVER(=2) 지점: Loiter + (filmingProperty 누락시) POINT 촬영 보정
            if wp.get("waypointPassType") == PASS_FLYOVER:
                if not wp.get("filmingProperty"):
                    wp["filmingProperty"] = _mk_point_filming_for_coord(wp.get("coordinate") or {})
                wp["loiterProperty"] = OrderedDict([
                    ("radius", 800),
                    ("direction", 1),
                    ("time", 30),
                    ("speed", 40),
                ])

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





