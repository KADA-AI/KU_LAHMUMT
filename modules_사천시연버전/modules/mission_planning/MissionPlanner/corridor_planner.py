"""
corridor_planner.py
────────────────────────────────────────────────────────
Mode-2(Corridor) 전용 : UAV 카메라 스윕·오프셋 비행 경로 생성기
(위·경도 입력 지원)

예시
────
    from corridor_planner import CorridorPlanner

    # 위도, 경도 (deg)
    WAYPOINTS = [(35.8900, 128.6100),
                 (35.9100, 128.6400),
                 (35.9300, 128.6700),
                 (35.9450, 128.7000)]

    planner = CorridorPlanner(
        waypoints       = WAYPOINTS,   # deg
        separation_dist = 850.0,       # m
        corridor_width  = 1000.0,      # m
        uav_alt         = 610.0        # m
    )

    cam_sets   = planner.camera_paths_deg        # [ [ (lat,lon), … ], … ]
    flight_sets= planner.offset_flight_paths_deg # [ [ (lat,lon), … ], … ]
"""

from typing import List, Tuple
import math
import numpy as np

from dividing_Aisle_class import PolygonProcessor
from Aisle_Sweep_CPP_shoot_plan import RectanglePath
import json, pprint
from data_def.mission_helpers import now_ms_since_2000
from data_def.search_speed import spacing_based_search_speed
import config as mp_config
try:
    from Search_Speed_2 import SearchSpeedCalculator as _SearchSpeedCalculator
except ImportError:
    try:
        from .Search_Speed_2 import SearchSpeedCalculator as _SearchSpeedCalculator  # type: ignore
    except ImportError:
        _SearchSpeedCalculator = None
try:
    from DB import select_best_config as _select_best_config
except ImportError:
    try:
        from .DB import select_best_config as _select_best_config  # type: ignore
    except ImportError:
        _select_best_config = None

Point = Tuple[float, float]        # generic (x,y) or (lat,lon)
LatLon = Tuple[float, float]


def _apply_db_fov_weight(fov_deg: float) -> float:
    try:
        weight = float(getattr(mp_config, "DB_FOV_WEIGHT", 1.0))
    except Exception:
        weight = 1.0
    if weight <= 0.0:
        weight = 1.0
    try:
        fov = float(fov_deg)
    except Exception:
        fov = 0.0
    return max(fov * weight, 0.0)


# ─────────────────────────────────────────────
# 좌표계 변환 (로컬 ENU)
# ─────────────────────────────────────────────
_R_EARTH = 6378137.0                 # m, WGS-84 평균 반경


def _latlon_to_xy(lat0: float, lon0: float, lat: float, lon: float) -> Point:
    """위도·경도 → 로컬 ENU [m] (lat0,lon0 기준 평면)"""
    lat0_rad = math.radians(lat0)
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    x = dlon * _R_EARTH * math.cos(lat0_rad)     # East
    y = dlat * _R_EARTH                          # North
    return (x, y)


def _xy_to_latlon(lat0: float, lon0: float, x: float, y: float) -> LatLon:
    """ENU [m] → 위도·경도 (deg)"""
    lat0_rad = math.radians(lat0)
    lat = lat0 + math.degrees(y / _R_EARTH)
    lon = lon0 + math.degrees(x / (_R_EARTH * math.cos(lat0_rad)))
    return (lat, lon)


# ─────────────────────────────────────────────
class CorridorPlanner:
    """Mode-2 Corridor 경로계획 (위·경도 입력)"""

    _V_TABLE = np.array([30.0, 40.0, 50.0])   # m/s
    _R_TABLE = np.array([340.0, 450.0, 560.0])  # m

    # ─────────────────────────────────────────
    def __init__(
        self,
        waypoints: List[LatLon],          # (lat, lon) deg
        separation_dist: float = 850.0,   # m
        corridor_width: float = 1000.0,   # m
        uav_alt: float = 610.0,           # m
        use_db: bool | None = None,
    ):
        if len(waypoints) < 2:
            raise ValueError("waypoints 는 최소 2개 이상 필요합니다.")

        self.lat0, self.lon0 = waypoints[0]          # 로컬 기준점
        self.waypoints_deg   = waypoints
        self.waypoints_xy    = [
            _latlon_to_xy(self.lat0, self.lon0, lat, lon) for lat, lon in waypoints
        ]

        self.sep = separation_dist
        self.width = corridor_width
        self.uav_alt = uav_alt
        self.fov_deg = 5.5
        self.selected_vel = None
        self.selected_dps = None
        self.selected_width = None

        use_db_flag = mp_config.USE_DB_FOR_CORRIDOR if use_db is None else bool(use_db)
        self._use_db = use_db_flag
        if use_db_flag and _select_best_config is not None:
            try:
                span = float(corridor_width)
                cfg = _select_best_config(span)
                self.sep = float(cfg["sep"])
                self.fov_deg = _apply_db_fov_weight(float(cfg["fov"]))
                self.selected_vel = float(cfg.get("vel", 0.0))
                self.selected_dps = float(cfg.get("dps", 0.0))
                self.selected_width = float(cfg.get("width", 0.0))
            except Exception:
                pass

        # 결과 컨테이너
        self.camera_paths_xy:   List[List[Point]] = []
        self.camera_paths_deg:  List[List[LatLon]] = []
        self.camera_points_flat: List[Point] = []
        self.camera_segments:   List[dict] = []
        self.offset_flight_paths_xy:  List[List[Point]] = []
        self.offset_flight_paths_deg: List[List[LatLon]] = []

        self._build_camera_paths()
        self._build_offset_paths()

    # ─────────────────────────────────────────
    @staticmethod
    def minimum_turn_radius(v_ms: float) -> float:
        return float(np.interp(v_ms,
                               CorridorPlanner._V_TABLE,
                               CorridorPlanner._R_TABLE))

    def _resolve_cruise_speed(self, cruise_speed: float) -> float:
        if self._use_db and self.selected_vel is not None and self.selected_vel > 0:
            return float(self.selected_vel)
        return float(cruise_speed)

    # ─────────────────────────────────────────
    # ▶  MSG-0303 형식(wing 단독) 본문 생성 ◀
    # ─────────────────────────────────────────
    def make_msg0303_body(self, cruise_speed: float = 40.0) -> dict:
        cruise_speed = self._resolve_cruise_speed(cruise_speed)
        # ── 헬퍼 ─────────────────────────────
        def _coord(lat, lon, alt=self.uav_alt):
            return {"latitude": round(lat, 6),
                    "longitude": round(lon, 6),
                    "altitude": round(alt, 1)}

        def _filming(ls0, ls1, search_spd):
            return {
                "fieldOfView": float(self.fov_deg),
                "sensorType":  1,
                "operationMode": 2,
                "lineSearch": {
                    "coordinateList": [_coord(*ls0), _coord(*ls1)],
                    "searchSpeed":    round(search_spd, 2)
                }
            }

        strip_spacing = max(float(self.sep), 1e-3)

        # ── 세트별 스윕라인 ↔ 비행 WP 매핑 ───────────────────
        sequence: list[tuple[LatLon, LatLon, LatLon]] = []
        #           (flightWP, sweepStart, sweepEnd)

        for s_idx, (cam_deg, flt_deg) in enumerate(
                zip(self.camera_paths_deg, self.offset_flight_paths_deg)):

            if not flt_deg:
                continue

            wp_s = np.array(self.waypoints_xy[s_idx])
            wp_e = np.array(self.waypoints_xy[s_idx + 1])
            along = (wp_e - wp_s) / np.linalg.norm(wp_e - wp_s)

            pairs = []
            for i in range(0, len(cam_deg) - 1, 2):
                ls0 = cam_deg[i]
                ls1 = cam_deg[i + 1]
                mid_xy = (np.array(self.camera_paths_xy[s_idx][i]) +
                          np.array(self.camera_paths_xy[s_idx][i + 1])) / 2
                d_along = np.dot(mid_xy - wp_s, along)
                pairs.append((d_along, ls0, ls1))

            pairs.sort(key=lambda t: t[0])          # 비행 WP 순서와 맞춤
            if len(pairs) != len(flt_deg):
                raise ValueError("flight-WP 수와 스윕라인 수가 다릅니다.")

            for flt_ll, (_, ls0_ll, ls1_ll) in zip(flt_deg, pairs):
                sequence.append((flt_ll, ls0_ll, ls1_ll))

        # ── 거리 함수 (평면 근사, m) ─────────────────────────
        def _dist(p, q):
            lat1, lon1 = p; lat2, lon2 = q
            dx = (lon2 - lon1) * 111_000 * math.cos(math.radians((lat1 + lat2) / 2))
            dy = (lat2 - lat1) * 111_000
            return math.hypot(dx, dy)

        def _to_xy(ll: LatLon) -> Point:
            return _latlon_to_xy(self.lat0, self.lon0, ll[0], ll[1])

        def _calc_search_speed(idx: int, flt_ll: LatLon, ls0_ll: LatLon, ls1_ll: LatLon) -> float:
            if idx >= len(sequence) - 1:
                return 0.0
            next_flt_ll = sequence[idx + 1][0]
            try:
                if _SearchSpeedCalculator is None:
                    raise RuntimeError("SearchSpeedCalculator unavailable")
                u0 = _to_xy(flt_ll)
                u1 = _to_xy(next_flt_ll)
                s0 = _to_xy(ls0_ll)
                s1 = _to_xy(ls1_ll)
                uav_path = [(u0[0], u0[1], 0.0), (u1[0], u1[1], 0.0)]
                sweep_path = [(s0[0], s0[1], 0.0), (s1[0], s1[1], 0.0)]
                calc = _SearchSpeedCalculator(uav_path, sweep_path, float(cruise_speed))
                return float(calc.compute_search_speed())
            except Exception:
                sweep_len = _dist(ls0_ll, ls1_ll)
                fallback = spacing_based_search_speed(
                    sweep_len_m=sweep_len,
                    spacing_m=strip_spacing,
                    cruise_speed_mps=cruise_speed,
                )
                return float(fallback) if fallback is not None else 0.0

        total_d = sum(_dist(sequence[i][0], sequence[i+1][0])
                      for i in range(len(sequence)-1))

        # ── Waypoint 리스트 ────────────────────────────────
        waypoint_list = []
        cum_d  = 0.0
        eta_ms = 0
        for idx, (flt_ll, ls0_ll, ls1_ll) in enumerate(sequence):
            sweep_len = _dist(ls0_ll, ls1_ll)     # ← **가장 먼저 계산**

            # 구간 거리 / 시간 (다음 WP까지)
            if idx < len(sequence) - 1:
                search_spd = _calc_search_speed(idx, flt_ll, ls0_ll, ls1_ll)
            else:
                search_spd = 0             # 마지막 WP는 고정

            # ETA·ECF
            if idx > 0:
                prev_d = _dist(sequence[idx-1][0], flt_ll)
                cum_d += prev_d
                eta_ms += int(prev_d / cruise_speed * 1000)
            ecf = round(cum_d / total_d, 2) if total_d else 0.0

            waypoint_id = 50 + idx
            waypoint_list.append({
                "waypointID":       waypoint_id,
                "coordinate":       _coord(*flt_ll),
                "speed":            cruise_speed,
                "eta":              eta_ms,
                "ecf":              ecf,
                "nextWaypointID":   waypoint_id + 1 if idx < len(sequence)-1 else 0,
                "waypointPassType": 1,
                "filmingProperty":  _filming(ls0_ll, ls1_ll, search_spd)
            })

        # ── 메시지 조립 ─────────────────────────
        now_ms = now_ms_since_2000()
        return {
            "timestamp":         now_ms,
            "pathID":            f"PATH-{now_ms}",
            "aircraftID":        1,
            "isFormationFlight": False,
            "formation": {
                "leaderAircraftID": {"aircraftID": 0},
                "formationDistanceList": []
            },
            "waypointList":      waypoint_list
        }

    # ─────────────────────────────────────────
    def _build_camera_paths(self) -> None:
        """RectanglePath + PolygonProcessor (XY기준)"""
        pp = PolygonProcessor(
            n=len(self.waypoints_xy),
            width=self.width,
            points=self.waypoints_xy,
            separation_dist=self.sep,
        )
        pp.calculate_polygons()

        for poly, line in zip(pp.get_sub_polygons(), pp.get_separated_lines()):
            rect = RectanglePath(
                line[0], poly,
                separation_dist=self.sep,
                UAV_height=self.uav_alt,
            )
            self.camera_paths_xy.append(rect.path)

        # 평탄화
        self.camera_points_flat = [
            pt for path in self.camera_paths_xy for pt in path
        ]

        # 세그먼트 정보
        self.camera_segments = [
            {
                "start": self.camera_points_flat[i],
                "end":   self.camera_points_flat[i + 1],
                "direction": "LR" if i % 2 == 0 else "RL",
            }
            for i in range(len(self.camera_points_flat) - 1)
        ]

        # lat/lon 변환
        self.camera_paths_deg = [
            [_xy_to_latlon(self.lat0, self.lon0, x, y) for x, y in path]
            for path in self.camera_paths_xy
        ]

    # ─────────────────────────────────────────
    def _build_offset_paths(self) -> None:
        """세트 시작 WP 기준 + 첫 스윕라인 수직(CW) offset (XY)"""
        def unit(px: Point) -> Point:
            n = math.hypot(*px)
            return (px[0] / n, px[1] / n) if n else (0.0, 0.0)

        for idx, cam in enumerate(self.camera_paths_xy):
            if idx >= len(self.waypoints_xy) - 1 or len(cam) < 2:
                self.offset_flight_paths_xy.append([])
                self.offset_flight_paths_deg.append([])
                continue

            wp_s = self.waypoints_xy[idx]
            wp_e = self.waypoints_xy[idx + 1]
            along = unit((wp_e[0] - wp_s[0], wp_e[1] - wp_s[1]))

            sv = unit((cam[1][0] - cam[0][0], cam[1][1] - cam[0][1]))
            perp = (sv[1] * self.sep, -sv[0] * self.sep)  # CW 90°

            mids = [((cam[i][0] + cam[i + 1][0]) / 2,
                     (cam[i][1] + cam[i + 1][1]) / 2)
                    for i in range(0, len(cam) - 1, 2)]
            dists = [((m[0] - wp_s[0]) * along[0] + (m[1] - wp_s[1]) * along[1], m)
                     for m in mids]

            flight_xy = [
                (wp_s[0] + along[0] * d + perp[0],
                 wp_s[1] + along[1] * d + perp[1])
                for d, _ in sorted(dists, key=lambda x: x[0])
            ]
            self.offset_flight_paths_xy.append(flight_xy)
            self.offset_flight_paths_deg.append([
                _xy_to_latlon(self.lat0, self.lon0, x, y) for x, y in flight_xy
            ])

    # ─────────────────────────────────────────
    # convenience getters
    # (camera_paths_deg / offset_flight_paths_deg 이미 public 속성으로 사용 가능)
    # ─────────────────────────────────────────


# ─────────────────────────────────────────
# 기능 확인용 데모
# ─────────────────────────────────────────
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from matplotlib.cm import get_cmap

    # 위도·경도 WAYPOINTS (deg)
    WAYPOINTS = [(35.8900, 128.6100),
                 (35.9100, 128.6400)]

    planner = CorridorPlanner(
        waypoints       = WAYPOINTS,
        separation_dist = 850.0,
        corridor_width  = 1000.0,
        uav_alt         = 610.0,
    )
    msg0303  = planner.make_msg0303_body(cruise_speed=40)
    print(json.dumps(msg0303, ensure_ascii=False, indent=2), end="")

    cam_sets = planner.camera_paths_deg
    flt_sets = planner.offset_flight_paths_deg

    fig, ax = plt.subplots(figsize=(8, 8))
    cmap = get_cmap("tab10", len(cam_sets))

    for idx, (cam, flt) in enumerate(zip(cam_sets, flt_sets)):
        cx, cy = zip(*[(lon, lat) for lat, lon in cam])   # lon→x, lat→y
        ax.plot(cx, cy, color=cmap(idx), lw=1.5, label=f"Set {idx+1}")
        ax.scatter(cx, cy, s=20, color=cmap(idx), zorder=3)

        if flt:
            fx, fy = zip(*[(lon, lat) for lat, lon in flt])
            ax.plot(fx, fy, linestyle="--", lw=1.2, color=cmap(idx))
            ax.scatter(fx, fy, s=30, color="red", zorder=5)

    ax.set_xlabel("Longitude (deg)")
    ax.set_ylabel("Latitude (deg)")
    ax.set_title("CorridorPlanner Demo (Lat/Lon)")
    ax.legend()
    ax.axis("equal")
    plt.show()
