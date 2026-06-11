# -*- coding: utf-8 -*-
"""
랜덤 LLA 도형(사각형/볼록다각형/원형)을 Folium 지도에 시각화하는 순수 데모 스크립트.
- 의존성: folium (pip install folium)
- 실행:   python random_shapes_demo.py
- 결과:   ./map_random_shapes.html 생성 및 기본 브라우저로 자동 오픈
"""

import math, random, webbrowser, os
from typing import List, Dict, Tuple
import folium

# ─────────────────────────────────────────────
# ENU 근사 좌표변환 (기존 coord_transform 과 같은 수식)
# ─────────────────────────────────────────────
_EARTH_R = 6378137.0  # WGS-84

def llh_to_xy(lat: float, lon: float, lat0: float, lon0: float) -> Tuple[float, float]:
    """(lat,lon) → 기준(lat0,lon0) 원점의 평면 ENU (x=East[m], y=North[m])"""
    d_lat = math.radians(lat - lat0)
    d_lon = math.radians(lon - lon0)
    avg_lat = math.radians((lat + lat0) / 2.0)
    x = _EARTH_R * d_lon * math.cos(avg_lat)
    y = _EARTH_R * d_lat
    return (x, y)

def xy_to_llh(x: float, y: float, lat0: float, lon0: float) -> Tuple[float, float]:
    """기준(lat0,lon0) ENU (x,y) → (lat,lon)"""
    d_lat = y / _EARTH_R
    d_lon = x / (_EARTH_R * math.cos(math.radians(lat0)))
    lat = math.degrees(d_lat) + lat0
    lon = math.degrees(d_lon) + lon0
    return (lat, lon)

# ─────────────────────────────────────────────
# 랜덤 도형 생성기
# ─────────────────────────────────────────────
def _rot(p: Tuple[float, float], deg: float) -> Tuple[float, float]:
    th = math.radians(deg)
    c, s = math.cos(th), math.sin(th)
    return (p[0]*c - p[1]*s, p[0]*s + p[1]*c)

class RandomLLAShapes:
    def __init__(self,
                 center_lla: Tuple[float, float] = (37.5, 127.0),
                 span_km_east: float = 15.0,
                 span_km_north: float = 15.0,
                 seed: int | None = 42):
        """
        center_lla     : 지도 중심 (lat, lon)
        span_km_east   : 중심 기준 동서로 ±span 범위 (km)
        span_km_north  : 중심 기준 남북으로 ±span 범위 (km)
        """
        self.lat0, self.lon0 = center_lla
        self.span_e = span_km_east * 1000.0
        self.span_n = span_km_north * 1000.0
        if seed is not None:
            random.seed(seed)

    # ── 랜덤 중심점 (LLA) 하나 ───────────────────────────
    def _rand_center(self) -> Tuple[float, float]:
        dx = random.uniform(-self.span_e, self.span_e)
        dy = random.uniform(-self.span_n, self.span_n)
        lat, lon = xy_to_llh(dx, dy, self.lat0, self.lon0)
        return (lat, lon)

    # ── 사각형 (heading 포함) ─────────────────────────────
    def _rect_at(self, lat: float, lon: float,
                 w_m: float, h_m: float, heading_deg: float) -> List[Dict]:
        """
        중심 (lat,lon) 기준 폭 w, 높이 h 의 직사각형 (시계/반시계 무관).
        heading_deg: 동쪽(0°)→북쪽(90°) 기준 회전.
        """
        # ENU 평면에서 모서리 4점 만들고 회전
        half = [(+w_m/2, +h_m/2), (+w_m/2, -h_m/2), (-w_m/2, -h_m/2), (-w_m/2, +h_m/2)]
        pts_xy = [_rot(p, heading_deg) for p in half]

        # ENU → LLA (각 점은 중심 상대 오프셋이므로 바로 변환)
        out = []
        for (x, y) in pts_xy:
            # 중심을 원점으로 가정해 변환하려면: 중심을 ENU 원점으로 두고 다시 합성
            # 1) 중심을 기준으로 (0,0) ENU
            c_x, c_y = (0.0, 0.0)
            # 2) 꼭짓점 (x,y) 를 LLA로
            vx, vy = (c_x + x, c_y + y)
            plat, plon = xy_to_llh(vx, vy, lat, lon)
            out.append({"latitude": plat, "longitude": plon, "altitude": 100})
        return out

    # ── 볼록 다각형 ───────────────────────────────────────
    def _convex_poly_at(self, lat: float, lon: float,
                        n_vert: int, radius_m: float) -> List[Dict]:
        """중심 (lat,lon)을 기준으로 반지름 ~radius 의 볼록다각형 생성."""
        # 무작위 각도/반경 생성 후 각도로 정렬
        angs = sorted([random.uniform(0, 360) for _ in range(n_vert)])
        # 반경은 0.5~1.0 사이 가중
        rads = [radius_m * random.uniform(0.5, 1.0) for _ in range(n_vert)]
        pts_xy = [ (rads[i]*math.cos(math.radians(angs[i])),
                    rads[i]*math.sin(math.radians(angs[i]))) for i in range(n_vert) ]
        # 무작위 전체 회전 한 번 더
        rot = random.uniform(0, 180)
        pts_xy = [_rot(p, rot) for p in pts_xy]

        out = []
        for (x, y) in pts_xy:
            plat, plon = xy_to_llh(x, y, lat, lon)
            out.append({"latitude": plat, "longitude": plon, "altitude": 100})
        return out

    # ── 원형(다각근사) ───────────────────────────────────
    def _circle_at(self, lat: float, lon: float,
                   radius_m: float, k: int = 36) -> List[Dict]:
        pts = []
        for i in range(k):
            th = 360.0 * i / k
            x = radius_m * math.cos(math.radians(th))
            y = radius_m * math.sin(math.radians(th))
            plat, plon = xy_to_llh(x, y, lat, lon)
            pts.append({"latitude": plat, "longitude": plon, "altitude": 100})
        return pts

    # ── 여러 도형 생성 ───────────────────────────────────
    def generate(self, n_shapes: int = 18,
                 kinds: Tuple[str, ...] = ("rect", "poly", "circle")) -> List[Dict]:
        """
        반환 형식:
          [ { "name": "RECT-01", "coordinateList": [ {lat,lon,alt}, ... ] }, ... ]
        """
        out: List[Dict] = []
        for i in range(n_shapes):
            lat, lon = self._rand_center()
            typ = random.choice(kinds)

            if typ == "rect":
                w = random.uniform(200.0, 1500.0)
                h = random.uniform(200.0, 1500.0)
                hdg = random.uniform(0.0, 180.0)
                coords = self._rect_at(lat, lon, w, h, hdg)
                name = f"RECT-{i+1:02d}"

            elif typ == "poly":
                nv = random.randint(3, 7)
                r  = random.uniform(250.0, 1200.0)
                coords = self._convex_poly_at(lat, lon, nv, r)
                name = f"POLY-{i+1:02d}"

            else:  # "circle"
                r  = random.uniform(250.0, 1200.0)
                coords = self._circle_at(lat, lon, r, k=36)
                name = f"CIRC-{i+1:02d}"

            out.append({"name": name, "coordinateList": coords})
        return out

    # ── Folium 지도 저장 ──────────────────────────────────
    def make_map(self, shapes: List[Dict], save_as: str = "map_random_shapes.html",
                 zoom_start: int = 12) -> str:
        fmap = folium.Map(location=[self.lat0, self.lon0], zoom_start=zoom_start)
        palette = [
            "#e6194b", "#3cb44b", "#0082c8", "#f58231", "#911eb4", "#46f0f0",
            "#f032e6", "#d2f53c", "#fabebe", "#008080", "#e6beff", "#aa6e28",
            "#fffac8", "#800000", "#aaffc3", "#808000", "#ffd8b1", "#000080",
        ]
        for idx, shp in enumerate(shapes):
            coords = [(p["latitude"], p["longitude"]) for p in shp["coordinateList"]]
            col = palette[idx % len(palette)]
            folium.Polygon(
                locations=coords,
                color=col, weight=2,
                fill=True, fill_opacity=0.25,
                tooltip=shp["name"],
            ).add_to(fmap)

        fmap.save(save_as)
        return os.path.abspath(save_as)

    # ── 원클릭 데모 ───────────────────────────────────────
    def demo(self) -> str:
        shapes = self.generate(n_shapes=18, kinds=("rect", "poly", "circle"))
        path = self.make_map(shapes)
        print(f"[OK] saved → {path}")
        try:
            webbrowser.open(f"file://{path}")
        except Exception:
            pass
        return path


if __name__ == "__main__":
    RandomLLAShapes(center_lla=(37.5, 127.0), span_km_east=15, span_km_north=15, seed=42).demo()
