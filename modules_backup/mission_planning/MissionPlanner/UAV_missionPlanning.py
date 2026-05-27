"""
UAV_missionPlanning.py
────────────────────────────────────────────────────────────
1) 입력:   · corridor WAYPOINTS  (global 경로점)
          · corridor WIDTH [m]
          · line SEPARATION [m]
          · camera FOV_DEG [deg]

2) 산출:
   • self.patches        : corridor sub-polygons
   • self.sweeps         : 모든 스윕 Line 집합
   • self.offset_wps     : 각 스윕에 1:1 対응하는 퍼프셋 Waypoints
   • self.orange_pts     : 시뮬레이션 궤적을 offset_wps 개수만큼 균등 분할한 점
                            (index i  ↔  sweep i  매칭)
   • self.traj           : 기체 궤적 (xs, ys)

3) public 메서드
   build_corridor()      : 패치 & 스윕 & offset_wps 갱신
   simulate()            : VerySimpleAutopilot 로 궤적 생성
   plot(show_sweeps=True): 결과 시각화
"""

from __future__ import annotations
import math, json
from functools import lru_cache
from typing import List, Tuple

try:
    from ..config import DEFAULT_SWEEP_SEPARATION_M
except ImportError:
    try:
        from config import DEFAULT_SWEEP_SEPARATION_M  # type: ignore
    except ImportError:
        from modules.mission_planning.MissionPlanner.config import DEFAULT_SWEEP_SEPARATION_M  # type: ignore

import numpy as np
import matplotlib.pyplot as plt
try:
    from .dividing_Aisle_class import PolygonProcessor
except Exception:
    from dividing_Aisle_class import PolygonProcessor  # type: ignore
try:
    from .simple_dynamics import SimpleUAV, VerySimpleAutopilot
except Exception:
    from simple_dynamics import SimpleUAV, VerySimpleAutopilot  # type: ignore

Point = Tuple[float, float]
Line  = Tuple[Point, Point]


@lru_cache(maxsize=32)
def _utm_transformers(zone: int, south: bool):
    import pyproj

    utm_code = f"+proj=utm +zone={zone} {'+south' if south else ''} +ellps=WGS84 +units=m +no_defs"
    fwd = pyproj.Transformer.from_crs("EPSG:4326", utm_code, always_xy=True).transform
    back = pyproj.Transformer.from_crs(utm_code, "EPSG:4326", always_xy=True).transform
    return utm_code, fwd, back


# ─────────────────────────────────────────────────────────
class UAVMissionPlanner:
    # ────────── 초기화 ───────────────────────────────────
    def __init__(self,
                 waypoints: List[Point],
                 corridor_width: float = 1000.0,
                 separation: float = DEFAULT_SWEEP_SEPARATION_M,
                 fov_deg: float = 10.0,
                 sweep_spacing_m: float | None = None,
                 cruise_speed: float = 40.0,
                 arrival_tol: float = 100.0,
                 simulate_path: bool = True,
                 crs: str = "xy"):              # ★ 'xy' or 'lla'
        """
        Parameters
        ----------
        waypoints : [(x,y)] 또는 [(lat,lon)]
            입력 좌표
        crs : str
            'xy'  → m 단위 ENU 좌표(기존 방식, default)  
            'lla' → (lat[deg], lon[deg])  위·경도 입력
        """
        if crs not in ("xy", "lla"):
            raise ValueError("crs must be 'xy' or 'lla'")

        # ── 위·경도 → UTM 변환 ────────────────────────
        if crs == "lla":
            self._lla_mode = True
            lat0, lon0 = waypoints[0]          # ? ? ?? zone ??
            zone = int((lon0 + 180) // 6) + 1
            south = lat0 < 0
            _utm_code, self._proj_fwd, self._proj_back = _utm_transformers(zone, south)
            waypoints_xy = [self._proj_fwd(lon, lat) for lat, lon in waypoints]
        else:
            self._lla_mode = False
            self._proj_back = None
            waypoints_xy = waypoints

        # ── 파라미터 보존 ────────────────────────────
        self.waypoints = waypoints_xy          # ← 이후 로직은 m 단위만 사용
        self.width       = corridor_width
        self.sep         = separation
        self.fov_deg     = fov_deg
        self.sweep_spacing_m = sweep_spacing_m
        self.v_cruise    = cruise_speed
        self.arrival_tol = arrival_tol

        # 산출 컨테이너
        self.patches:     List[List[Point]] = []
        self.sweeps:      List[Line]        = []
        self.offset_wps:  List[Point]       = []
        self.traj:        Tuple[List[float], List[float]] = ([], [])
        self.orange_pts:  List[Point]       = []

        # 단계별 수행
        if len(self.waypoints) >= 2:       # ★ ① 최소 2점일 때만
            self.build_corridor()
            if simulate_path:
                self.simulate()

    # ────────── Corridor & Sweep 구축 ───────────────────
    def build_corridor(self):
        # PolygonProcessor → corridor sub-patches
        pp = PolygonProcessor(n=len(self.waypoints),
                              width=self.width,
                              points=self.waypoints,
                              separation_dist=self.sep)
        pp.calculate_polygons()
        self.patches = pp.get_sub_polygons()
        self.sweeps.clear()
        self.offset_wps.clear()

        for idx, poly in enumerate(self.patches):
            if len(poly) != 4 or idx >= len(self.waypoints) - 1:
                continue

            wp_s, wp_e = self.waypoints[idx], self.waypoints[idx + 1]
            edges = [(poly[i], poly[(i + 1) % 4]) for i in range(4)]
            mids  = [self._edge_mid(*e) for e in edges]
            start_edge = edges[int(np.argmin([math.hypot(mp[0]-wp_s[0],
                                                         mp[1]-wp_s[1]) for mp in mids]))]
            end_edge   = edges[int(np.argmin([math.hypot(mp[0]-wp_e[0],
                                                         mp[1]-wp_e[1]) for mp in mids]))]

            sweeps = self._sweep_lines(poly, start_edge, end_edge,
                                       self.fov_deg, self.sep)
            self.sweeps.extend(sweeps)

            for ln in sweeps:
                self.offset_wps.append(self._perp_offset(*ln, self.sep))

    # ────────── lat/lon 변환 헬퍼 ────────────────────
    def to_latlon(self, points_xy: List[Point]) -> List[Point]:
        """
        내부 ENU(m) 좌표 리스트 → (lat, lon) deg 로 변환.
        LLA 모드 입력이었을 때만 사용 가능.
        """
        if not self._lla_mode:
            raise RuntimeError("Planner was created with crs='xy'; no lat/lon available.")
        return [self._proj_back(x, y)[::-1] for x, y in points_xy]   # (lat,lon)

    # ────────── 동역학 시뮬레이션 & 주황점 산출 ──────────
    def simulate(self):
        if len(self.offset_wps) < 2:
            raise RuntimeError("offset_wps 가 2개 미만 – build_corridor() 실패?")

        # Waypoints 중복 제거
        wps = [self.offset_wps[0]]
        for p in self.offset_wps[1:]:
            if math.hypot(p[0]-wps[-1][0], p[1]-wps[-1][1]) > 1e-3:
                wps.append(p)

        # 초기 상태
        x0, y0 = wps[0]; x1, y1 = wps[1]
        hdg0 = math.degrees(math.atan2(-(y1 - y0), x1 - x0))
        uav  = SimpleUAV(x0, y0, heading_deg=hdg0, v0=self.v_cruise)
        ap   = VerySimpleAutopilot(wps, v_cruise=self.v_cruise,
                                   arrival_tol=self.arrival_tol)

        xs, ys = [uav.x], [uav.y]
        DT = 0.1
        for _ in range(10000):          # 충분히 큰 step cap
            phi_cmd, v_cmd = ap.control(uav)
            if ap.done:
                break
            uav.step(DT, phi_cmd, v_cmd)
            xs.append(uav.x); ys.append(uav.y)

        self.traj = (xs, ys)
        self.orange_pts = self._sample_path(xs, ys, len(self.offset_wps))

    # ────────── 시각화 ─────────────────────────────────
    def plot(self, show_sweeps: bool = True,
             show_traj: bool = True,
             figsize=(8,8)):
        fig, ax = plt.subplots(figsize=figsize)

        # corridor patch
        for poly in self.patches:
            px, py = zip(*(poly + [poly[0]]))
            ax.fill(px, py, color="lightgray", alpha=0.25, zorder=0)

        # sweeps
        if show_sweeps:
            for ln in self.sweeps:
                ax.plot(*zip(*ln), color="blue", lw=1)

        # orange pts
        if self.orange_pts:
            ox, oy = zip(*self.orange_pts)
            ax.scatter(ox, oy, c="orange", s=40, zorder=4, label="orange samples")

        # trajectory
        if show_traj and self.traj[0]:
            ax.plot(self.traj[0], self.traj[1],
                    color="magenta", lw=2, label="UAV trajectory")

        # start/end edges (green/red)
        for idx, poly in enumerate(self.patches):
            if len(poly) != 4 or idx >= len(self.waypoints) - 1:
                continue
            wp_s, wp_e = self.waypoints[idx], self.waypoints[idx + 1]
            edges = [(poly[i], poly[(i + 1) % 4]) for i in range(4)]
            mids  = [self._edge_mid(*e) for e in edges]
            start_edge = edges[int(np.argmin([math.hypot(mp[0]-wp_s[0],
                                                         mp[1]-wp_s[1]) for mp in mids]))]
            end_edge   = edges[int(np.argmin([math.hypot(mp[0]-wp_e[0],
                                                         mp[1]-wp_e[1]) for mp in mids]))]
            ax.plot(*zip(*start_edge), color="green", lw=2)
            ax.plot(*zip(*end_edge),   color="red",   lw=2)

        ax.set_aspect('equal', 'box')
        ax.set_title("Mission planning : corridor / sweeps / UAV")
        ax.legend(); ax.grid(True)
        plt.show()

    # ────────── 내부 유틸 (static) ───────────────────────
    @staticmethod
    def _edge_mid(p: Point, q: Point) -> Point:
        return ((p[0]+q[0])/2, (p[1]+q[1])/2)

    @staticmethod
    def _unit(v: Point) -> Point:
        l = math.hypot(*v)
        return (v[0]/l, v[1]/l) if l else (0.0, 0.0)

    @classmethod
    def _perp_offset(cls, p1: Point, p2: Point, dist: float) -> Point:
        mid = cls._edge_mid(p1, p2)
        vx, vy = p2[0]-p1[0], p2[1]-p1[1]
        ux, uy = cls._unit((vy, -vx))          # 시계 90°
        return (mid[0]+ux*dist, mid[1]+uy*dist)

    @staticmethod
    def _intersect_ray_seg(o: Point, d: Point,
                           a: Point, b: Point, eps=1e-9):
        x1, y1 = o; dx1, dy1 = d
        x2, y2 = a; dx2, dy2 = b[0]-x2, b[1]-y2
        det = dx1*dy2 - dy1*dx2
        if abs(det) < eps:
            return None
        s = ((x2-x1)*dy2 - (y2-y1)*dx2) / det
        t = ((x2-x1)*dy1 - (y2-y1)*dx1) / det
        if 0-eps <= t <= 1+eps:
            return (x1+s*dx1, y1+s*dy1), s
        return None

    def _sweep_lines(self, patch: List[Point],
                     start_edge: Line, end_edge: Line,
                     fov_deg: float, separation: float) -> List[Line]:
        a0, a1 = self._edge_mid(*start_edge), self._edge_mid(*end_edge)
        mov    = (a1[0]-a0[0], a1[1]-a0[1])
        v0     = self._unit((start_edge[1][0]-start_edge[0][0],
                             start_edge[1][1]-start_edge[0][1]))
        v1     = self._unit((end_edge[1][0]-end_edge[0][0],
                             end_edge[1][1]-end_edge[0][1]))
        if v0[0]*v1[0] + v0[1]*v1[1] < 0:
            v0 = (-v0[0], -v0[1])

        try:
            spacing = float(self.sweep_spacing_m)
        except Exception:
            spacing = 0.0
        if spacing <= 0.0:
            spacing = 2 * separation * math.tan(math.radians(fov_deg) / 2)
        n = max(int(math.ceil(math.hypot(*mov)/spacing))+1, 3)

        segs = [(patch[i], patch[(i+1)%4]) for i in range(4)]
        lines: List[Line] = []

        for i in range(n):
            t  = i/(n-1) if n>1 else 0.0
            ax = a0[0]+mov[0]*t
            ay = a0[1]+mov[1]*t
            vx = v0[0]*(1-t)+v1[0]*t
            vy = v0[1]*(1-t)+v1[1]*t
            vx, vy = self._unit((vx, vy))

            hits=[]
            for a,b in segs:
                hit = self._intersect_ray_seg((ax,ay),(vx,vy),a,b)
                if hit: hits.append(hit)
            if len(hits)>=2:
                hits.sort(key=lambda x: x[1])
                lines.append((hits[0][0], hits[-1][0]))
        return lines

    # ────────── 균등 거리 샘플링 (안정화 버전) ──────────────
    @staticmethod
    def _sample_path(xs: List[float], ys: List[float], n: int) -> List[Point]:
        """
        궤적(xs,ys)을 누적거리 기준으로 n개(시작·끝 포함) 균등 분할.
        ──────────────────────────────────────────────────────
        • 궤적 점이 1개뿐이거나, n ≤ 1  → 동일점 반복 반환
        • 마지막 target == total 일 때 IndexError 나오지 않도록
          안전하게 처리
        """
        if n <= 1:
            return [(xs[0], ys[0])]

        if len(xs) < 2:
            return [(xs[0], ys[0])] * n

        # 세그먼트 길이 & 누적 거리
        seg_len = [math.hypot(xs[i+1]-xs[i], ys[i+1]-ys[i]) for i in range(len(xs)-1)]
        cum = [0.0]
        for l in seg_len:
            cum.append(cum[-1] + l)
        total = cum[-1]

        if total < 1e-6:               # 거의 이동 안 했을 때
            return [(xs[0], ys[0])] * n

        targets = [i * total / (n-1) for i in range(n)]
        samples = []
        j = 0
        for t in targets:
            # 마지막 포인트 보호
            if t >= total:
                samples.append((xs[-1], ys[-1]))
                continue

            # 적절한 세그먼트 찾기
            while j < len(cum)-2 and cum[j+1] < t:
                j += 1
            # 선형 보간
            ratio = (t - cum[j]) / max(cum[j+1]-cum[j], 1e-9)
            x = xs[j] + (xs[j+1]-xs[j]) * ratio
            y = ys[j] + (ys[j+1]-ys[j]) * ratio
            samples.append((x, y))
        return samples

    # ────────── ⑥  경로-전용 플래너  ──────────────────────────    
    @staticmethod
    def plan_route_only(
        input_lla: List[Tuple[float, float]],
        cruise_speed: float = 40.0,
        heading_tol_deg: float = 5.0,
        store: bool = False,
        # ▼ DTA 파라미터 (필요 시 조정)
        dta_bank_deg: float = 30.0,      # 설계 뱅크각 φ [deg]
        dta_v_extra: float = 3.0,        # +3·V 항
        dta_limit_m: float = 800.0       # DTA 상한
    ) -> List[dict]:
        """
        • 위·경도 경유지를 받아 동역학 시뮬 → 직선/선회 구간 WP 단순화  
        • 선회 구간은 Fly-by DTA를 반영해 ‘미리 끊은’ 가상 WP(선회 진입/탈출)를 삽입
        """
        import math
        if len(input_lla) < 2:
            raise ValueError("input_lla must contain at least 2 points")

        # ── ① 위·경도 ↔ UTM 변환 ──────────────────────────
        lat0, lon0 = input_lla[0]
        zone   = int((lon0 + 180) // 6) + 1
        south  = lat0 < 0
        _utm, fwd, back = _utm_transformers(zone, south)
        pts_xy = [fwd(lon, lat) for lat, lon in input_lla]

        # ── ② ★ DTA 선회 진입/탈출 WP 삽입 ─────────────────
        if len(pts_xy) >= 3:
            g = 9.80665
            v = cruise_speed
            R_turn = v * v / (g * math.tan(math.radians(dta_bank_deg)))

            new_xy: list[Tuple[float, float]] = [pts_xy[0]]
            for i in range(1, len(pts_xy) - 1):
                p_prev, p_cur, p_next = pts_xy[i-1], pts_xy[i], pts_xy[i+1]
                vin  = (p_cur[0] - p_prev[0], p_cur[1] - p_prev[1])
                vout = (p_next[0] - p_cur[0], p_next[1] - p_cur[1])
                lin, lout = math.hypot(*vin), math.hypot(*vout)
                if lin < 1e-6 or lout < 1e-6:
                    new_xy.append(p_cur); continue

                # unit vectors
                u_in  = (vin[0]/lin,  vin[1]/lin)
                u_out = (vout[0]/lout, vout[1]/lout)

                # leg 변환각 φ (0 ~ 180)
                dot = max(-1.0, min(1.0, -(u_in[0]*u_out[0] + u_in[1]*u_out[1])))
                phi = math.degrees(math.acos(dot))
                if phi < 1e-3:          # 사실상 직선
                    new_xy.append(p_cur); continue
                phi = min(phi, 90.0)    # 사양상 90 deg 이내로 제한

                # DTA[m] = R·tan(½φ) + 3·V   (문서 기준)
                dta = R_turn * math.tan(math.radians(phi) / 2.0) + dta_v_extra * v
                dta = min(dta, dta_limit_m)

                # in/out leg 길이가 짧으면 삽입 생략
                if dta*1.05 > lin or dta*1.05 > lout:
                    new_xy.append(p_cur); continue

                # 진입·탈출 가상 WP 계산
                wp_in  = (p_cur[0] - u_in[0]*dta,  p_cur[1] - u_in[1]*dta)
                wp_out = (p_cur[0] + u_out[0]*dta, p_cur[1] + u_out[1]*dta)
                new_xy.extend([wp_in, wp_out])
            new_xy.append(pts_xy[-1])
            pts_xy = new_xy

        # ── ③ VerySimpleAutopilot 시뮬레이션(기존과 동일) ─────────

        if len(pts_xy) == 2:
            simp_xy = [pts_xy[0], pts_xy[1]]
            traj_xy = simp_xy[:] if store else None
        else:
            x0, y0 = pts_xy[0]; x1, y1 = pts_xy[1]
            hdg0 = math.degrees(math.atan2(-(y1 - y0), x1 - x0))
            uav  = SimpleUAV(x0, y0, heading_deg=hdg0, v0=cruise_speed)
            ap   = VerySimpleAutopilot(pts_xy, v_cruise=cruise_speed, arrival_tol=100)

            DT = 0.2
            simp_xy: list[tuple[float, float]] = [(uav.x, uav.y)]
            traj_xy: list[tuple[float, float]] | None = [(uav.x, uav.y)] if store else None
            tol = math.radians(heading_tol_deg)
            have_ref = False
            ref = 0.0
            atan2 = math.atan2
            wrap_pi = math.pi
            for _ in range(20000):
                phi_cmd, v_cmd = ap.control(uav)
                if ap.done: break
                prev_x, prev_y = uav.x, uav.y
                uav.step(DT, phi_cmd, v_cmd)
                curr_x, curr_y = uav.x, uav.y
                if traj_xy is not None:
                    traj_xy.append((curr_x, curr_y))
                seg_hdg = atan2(curr_y - prev_y, curr_x - prev_x)
                if not have_ref:
                    ref = seg_hdg
                    have_ref = True
                else:
                    dh = abs((seg_hdg - ref + wrap_pi) % (2 * wrap_pi) - wrap_pi)
                    if dh > tol:
                        simp_xy.append((prev_x, prev_y))
                        ref = seg_hdg
            last_xy = (uav.x, uav.y)
            if simp_xy[-1] != last_xy:
                simp_xy.append(last_xy)

        wp_out: list[dict] = []
        for idx, (x_i, y_i) in enumerate(simp_xy):
            lat, lon = back(x_i, y_i)[::-1]
            if idx == len(simp_xy)-1:
                seg_ms = 0
            else:
                x_j, y_j = simp_xy[idx+1]
                seg_ms = math.hypot(x_j - x_i, y_j - y_i) / cruise_speed * 1000
            wp_out.append({"lat": round(lat,6), "lon": round(lon,6),
                        "eta_ms": int(round(seg_ms))})

        if store:
            self.route_raw  = input_lla
            self.route_traj = [back(x,y)[::-1] for x,y in (traj_xy or simp_xy)]
            self.route_wp   = [(w["lat"], w["lon"]) for w in wp_out]

        return wp_out



    # ────────── ⑦  경로-전용 시각화 ─────────────────────────

    def plot_route_only(self, figsize=(6,6)):
        """
        plan_route_only(store=True) 로 저장된 경로를 matplotlib 로 시각화.
        • 회색 점선 : 사용자가 입력한 원시 경유지
        • 파란 실선 : 동역학 시뮬레이션 궤적
        • 빨간 점   : 단순화된 Waypoints(출·도착/선회 전후)
        """
        if not hasattr(self, "route_wp"):
            raise RuntimeError("No route data – 먼저 plan_route_only(..., store=True) 호출")

        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=figsize)

        # 원시 입력
        if self.route_raw:
            lat_raw, lon_raw = zip(*self.route_raw)
            ax.plot(lon_raw, lat_raw, c="grey", ls="--", lw=1, label="input pts")

        # 동역학 궤적
        if self.route_traj:
            lat_t, lon_t = zip(*self.route_traj)
            ax.plot(lon_t, lat_t, c="blue", lw=1.5, label="sim trajectory")

        # 단순화 WP
        lat_wp, lon_wp = zip(*self.route_wp)
        ax.scatter(lon_wp, lat_wp, c="red", s=30, zorder=5, label="simplified WP")

        ax.set_aspect('equal', 'box')
        ax.set_xlabel("Longitude (deg)")
        ax.set_ylabel("Latitude (deg)")
        ax.set_title("Route planning (straight-/turn-split)")
        ax.legend(); ax.grid(True); plt.show()


    # ────────── WP ↔ Sweep 매칭 출력 (lat/lon 지원) ───────────
    def report_segments(self, zigzag: bool = True):
        """
        콘솔에 ① WP(ID) ② 스윕 시작·끝 좌표를 출력.
        • LLA 모드(crs='lla') → 위·경도(º)로 자동 변환해 표기
        • ENU 모드(crs='xy') → 그대로 (x,y) [m] 표기
        • zigzag=True  → 홀·짝 스윕 방향 뒤집어 lawn-mower 순서 확인
        """
        if len(self.sweeps) != len(self.offset_wps):
            raise RuntimeError("sweeps ↔ offset_wps 개수가 불일치!")

        # 내부 변환 헬퍼
        def _fmt(pt):
            if self._lla_mode:                       # lat/lon 출력
                lat, lon = self._proj_back(pt[0], pt[1])[::-1]
                return f"({lat:9.6f}°, {lon:9.6f}°)"
            else:                                    # ENU(m) 출력
                return f"({pt[0]:9.3f}, {pt[1]:9.3f})"

        for idx, (ln, wp) in enumerate(zip(self.sweeps, self.offset_wps), 1):
            start, end = ln
            if zigzag and idx % 2 == 0:              # 짝수 스윕은 방향 반전
                start, end = end, start
            print(f"WP{idx:02d} {_fmt(wp)}  ▶  {_fmt(start)}  →  {_fmt(end)}")
            
# # ────────── Quick self-test & demo ───────────────────────────
if __name__ == "__main__":
    # ── ① ENU(m) 좌표 예시 ────────────────────────────────
    WPS_xy = [(-2400, -1300), (-1610, 1700), (1020, 1803), (3050, 326)]

    planner_xy = UAVMissionPlanner(WPS_xy,
                                   corridor_width=1000,
                                   separation=850,
                                   fov_deg=10,
                                   cruise_speed=45,      # 순항 속도 [m/s]
                                   crs="xy")             # ← ENU(m) 모드
    print("\n--- ENU(m) demo ---")
    
    planner_xy.report_segments()
    planner_xy.plot(figsize=(7,7))

    # ── ② 위·경도(lat,lon) 예시 ───────────────────────────
    #     (임의 값, 실제 위·경도 필요 시 수정)
    WPS_lla = [
        (38.2250, 127.3000),
        (38.2600, 127.4100),
        (38.2500, 127.5000),
        (38.2300, 127.5800),
    ]

    planner_lla = UAVMissionPlanner(WPS_lla,
                                    corridor_width=1000,
                                    separation=850,
                                    fov_deg=10,
                                    cruise_speed=45,
                                    crs="lla")           # ← lat/lon 모드
    print("\n--- Lat/Lon demo ---")
    planner_lla.report_segments()
    planner_lla.plot(figsize=(7,7))

    # 주황점(lat,lon) 변환 예시
    orange_ll = planner_lla.to_latlon(planner_lla.orange_pts)
    print("\nFirst 3 orange points (lat,lon):", orange_ll[:3])
