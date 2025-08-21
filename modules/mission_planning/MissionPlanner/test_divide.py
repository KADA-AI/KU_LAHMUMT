# mission_pipeline.py  (★새 함수)
#  ──────────────────────────────────────────────────────────────
from typing import List, Dict, Tuple
import math

_R = 6_378_137.0  # WGS-84 평균반경

def _llh2xy(lat, lon, lat0, lon0):
    lat0_r = math.radians(lat0)
    x = math.radians(lon-lon0) * _R * math.cos(lat0_r)
    y = math.radians(lat-lat0) * _R
    return x, y

def _xy2llh(x, y, lat0, lon0):
    lat0_r = math.radians(lat0)
    lat = lat0 + math.degrees(y/_R)
    lon = lon0 + math.degrees(x/(_R*math.cos(lat0_r)))
    return lat, lon

# ── Half-plane(직선) clipper ──────────────────────────────────
def _clip_poly(poly: List[Tuple[float,float]],
               A: float, B: float, C: float,      # Ax+By+C ≤ 0  keep-inside
) -> List[Tuple[float,float]]:
    out: List[Tuple[float,float]] = []
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i+1)%n]
        f1 = A*x1 + B*y1 + C
        f2 = A*x2 + B*y2 + C
        if f1 <= 0:               # 현재 점이 안쪽이면 일단 보관
            out.append((x1, y1))
        if f1*f2 < 0:             # 간선이 경계와 교차 → 교점 추가
            t = f1 / (f1 - f2)
            out.append((x1 + t*(x2-x1),  y1 + t*(y2-y1)))
    return out

def divide_search_area_clip(
    area_poly: List[Dict], uav_cnt: int, bearing_deg: float
) -> List[Dict]:
    lat0, lon0 = area_poly[0]["latitude"], area_poly[0]["longitude"]
    alt0 = area_poly[0].get("altitude", 0)

    # 1) LLH → ENU
    poly_xy = [_llh2xy(p["latitude"], p["longitude"], lat0, lon0)
               for p in area_poly]

    # 2) 노멀 n̂ (bearing 과 직각)
    th = math.radians(bearing_deg)
    nx, ny =  math.sin(th), -math.cos(th)       # (E,N) 좌표

    # 3) 투영 값 범위
    projs = [nx*x + ny*y for x, y in poly_xy]
    d_min, d_max = min(projs), max(projs)

    subareas: List[Dict] = []
    for i in range(uav_cnt):
        d1 = d_min + (d_max-d_min)*i/uav_cnt
        d2 = d_min + (d_max-d_min)*(i+1)/uav_cnt

        # **부호 교정** ---------------------------
        #   n·x ≥ d1  →  -n·x + d1 ≤ 0   (A,B,C = -nx,-ny,d1)
        #   n·x ≤ d2  →   n·x - d2 ≤ 0   (A,B,C =  nx, ny,-d2)
        poly_clip = _clip_poly(poly_xy,  -nx, -ny,  d1)
        poly_clip = _clip_poly(poly_clip, nx,  ny, -d2)

        if len(poly_clip) < 3:    # 안전가드
            raise ValueError(
                f"UAV strip #{i+1}: clipping failed (vertex count={len(poly_clip)})"
            )

        # ENU → LLH
        coord_llh = [{
            "latitude":  _xy2llh(x, y, lat0, lon0)[0],
            "longitude": _xy2llh(x, y, lat0, lon0)[1],
            "altitude":  alt0
        } for x, y in poly_clip]

        subareas.append({
            "Geometry":       "Area",
            "coordinateList": coord_llh,
        })

    return subareas
# ──────────────────────────────────────────────────────────────

# 임의의 6-각형 (약간 뒤틀린 면적)
hex_poly = [
    {"latitude":37.0000,"longitude":127.0000,"altitude":100},
    {"latitude":37.0020,"longitude":127.0060,"altitude":100},
    {"latitude":37.0010,"longitude":127.0120,"altitude":100},
    {"latitude":36.9980,"longitude":127.0120,"altitude":100},
    # {"latitude":36.9960,"longitude":127.0060,"altitude":100},
    # {"latitude":36.9970,"longitude":127.0000,"altitude":100},
]

subs = divide_search_area_clip(hex_poly, uav_cnt=3, bearing_deg=50)

# 시각 확인 (matplotlib)
import matplotlib.pyplot as plt
fig, ax = plt.subplots()
for idx, s in enumerate(subs, 1):
    ll = s["coordinateList"]
    xs = [p["longitude"] for p in ll] + [ll[0]["longitude"]]
    ys = [p["latitude"]  for p in ll] + [ll[0]["latitude"]]
    ax.plot(xs, ys, label=f'UAV{idx}')
ax.set_aspect('equal'); ax.legend(); plt.show()