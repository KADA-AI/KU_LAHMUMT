# mission_pipeline.py

from __future__ import annotations
import concurrent.futures
import random
import string
import sys
import os, json, math, shutil, time
from typing import Any, Callable, List, Optional, Set
import cv2
from datetime import datetime, timezone, timedelta
import numpy as np
from scipy.spatial import ConvexHull
from stable_baselines3 import PPO
from pulp import (
    LpProblem, LpMinimize, LpVariable, lpSum,
    LpStatus, PULP_CBC_CMD, value
)

_CANONICAL_NAME = "modules.mission_planning.MissionPlanner.AnS.mission_pipeline"
if __name__ == "AnS.mission_pipeline":
    sys.modules[_CANONICAL_NAME] = sys.modules[__name__]
elif __name__ == _CANONICAL_NAME:
    sys.modules["AnS.mission_pipeline"] = sys.modules[__name__]

from data_def.id_allocator import (
    reserve_imp_ids,
    reserve_individual_mission_ids,
    reserve_mission_plan_ids,
    reserve_path_ids,
)

from typing import List, Dict, Tuple
import math
from shapely.geometry import LineString, Polygon, box
from shapely.ops import linemerge
from shapely.affinity import translate

from modules.common import db_paths

# Last run timing metrics for divide_and_pattern (seconds).
_LAST_DIVIDE_AND_PATTERN_METRICS: dict[str, float] = {}


def get_last_divide_and_pattern_metrics() -> dict[str, float]:
    return dict(_LAST_DIVIDE_AND_PATTERN_METRICS)

# ==== 외부 의존 모듈 경로 맞춰 주세요 ====
from .env_patternselection import UnifiedMissionEnvironment
from .task_patterns_ver2 import mission_patterns
from .mission_effectiveness_ver2 import calculate_flight_distance
from .coord_transform import llh_to_xy, xy_to_llh, EARTH_RADIUS
from data_def.mission_helpers import now_ms_since_2000

# DEM.jpg 는 KU/AnS 폴더 내부에 둔다고 가정
DEM_PATH       = os.path.join(os.path.dirname(__file__), "DEM.jpg")
DEM_IMG        = cv2.imread(DEM_PATH, cv2.IMREAD_GRAYSCALE) if os.path.exists(DEM_PATH) else None
DEM_RESOLUTION = 100.0   # 1 pixel = 100 m (GUI 와 동일)
_ID_COUNTER_FILE = os.path.join(os.path.dirname(__file__), "_id_counters.json")

# ──────────────────────────────────────────────────────────────
# 0.  공통 유틸, 상수
# ──────────────────────────────────────────────────────────────

UAV_VELOCITY = 40.0     # UAV 순항속도: 패턴결정 후 스케줄링 시 필요한 임무예상시간 예측 시 필요

_R = 6_378_137.0  # WGS-84 평균반경
_DEFAULT_AREA_BEARING_DEG = 90.0

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


def _extract_aircraft_id(entry) -> Optional[int]:
    """Return aircraft ID as int from dict/int/str representations."""
    if entry is None:
        return None
    if isinstance(entry, dict):
        entry = entry.get("aircraftID")
    if isinstance(entry, int):
        return entry
    if isinstance(entry, str):
        token = entry.strip().upper()
        if token.startswith(("UAV", "LAH")):
            token = "".join(ch for ch in token if ch.isdigit())
        try:
            return int(token)
        except ValueError:
            return None
    try:
        return int(entry)
    except (TypeError, ValueError):
        return None


def _load_vehicle_status_available() -> Optional[Set[int]]:
    """Read Logs/.../VehicleStatus/status.json if present and return available IDs."""
    try:
        status_path = db_paths.get_db_subpath("VehicleStatus", "status.json")
    except Exception:
        return None
    if not status_path.exists():
        return None
    try:
        with status_path.open("r", encoding="utf-8") as fh:
            payload = json.load(fh)
    except Exception:
        return None
    raw = payload.get("available")
    if not isinstance(raw, list):
        return None
    available: Set[int] = set()
    for item in raw:
        try:
            available.add(int(item))
        except (TypeError, ValueError):
            continue
    return available


def _apply_vehicle_status_filter(
    cmpk: dict,
    log: Callable[[str], None],
    *,
    trust_input_aircraft: bool = False,
) -> None:
    """Mutate CMPK's availableAircraftList to respect VehicleStatus snapshot."""
    if trust_input_aircraft:
        log("    ▸ VehicleStatus 적용 생략: 초기 임무계획은 0201 availableAircraftList 사용")
        return
    status_available = _load_vehicle_status_available()
    if status_available is None:
        return

    original = cmpk.get("availableAircraftList") or []
    filtered = []
    removed: Set[int] = set()
    kept_ids: Set[int] = set()
    kept_unknown = False
    for entry in original:
        aid = _extract_aircraft_id(entry)
        if aid is None:
            kept_unknown = True
            filtered.append(entry)
            continue
        if aid in status_available:
            filtered.append(entry)
            kept_ids.add(aid)
        else:
            removed.add(aid)

    cmpk["availableAircraftList"] = filtered
    if removed:
        log(f"    ▸ VehicleStatus 적용: 불가 기체 제외 {sorted(removed)}")
    elif kept_ids:
        log(f"    ▸ VehicleStatus 적용: 사용 가능한 기체 {sorted(kept_ids)}")
    elif len(status_available) == 0:
        log("    ▸ VehicleStatus 적용: status.json 상 사용 가능 기체가 없습니다.")
    if kept_unknown:
        log("    ▸ VehicleStatus 적용: ID 해석 불가 항목은 그대로 유지했습니다.")
    if not filtered:
        log("    ▸ VehicleStatus 적용 결과 사용 가능한 기체가 없습니다.")

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

from shapely.geometry import LineString, Polygon, box, Point
import numpy as np
from shapely.strtree import STRtree
from shapely.ops import linemerge, unary_union, split
from shapely.affinity import translate


def divide_corridor_polyline(line_seg: dict, uav_cnt: int) -> list[dict]:
    # ─────────────────────────────────────────────────────────
    # ★ 교차점(비인접 세그먼트 간) 탐지
    # ─────────────────────────────────────────────────────────
    def _non_adjacent_cross_points(xy_np: np.ndarray):
        segs = [LineString([xy_np[i], xy_np[i+1]]) for i in range(len(xy_np)-1)]
        pts = []
        if not segs:
            return pts

        # 빠른 경로: STRtree + query_bulk (Shapely 2.x 권장)
        try:
            tree = STRtree(segs)
            # query_bulk는 (hit_idx, tree_idx) 2xN 배열을 돌려줌
            import numpy as _np
            pairs = _np.array(tree.query_bulk(segs)).T
            seen = set()
            for i, j in pairs:
                if i == j:       # 자기자신
                    continue
                if abs(i - j) <= 1:  # 인접 세그먼트는 허용 → 스킵
                    continue
                if i > j:
                    i, j = j, i
                if (i, j) in seen:
                    continue
                seen.add((i, j))
                inter = segs[i].intersection(segs[j])
                if inter.is_empty:
                    continue
                if inter.geom_type == "Point":
                    pts.append(inter)
                elif inter.geom_type == "MultiPoint":
                    pts.extend(list(inter.geoms))
            return pts
        except Exception:
            # 느린 경로: O(n²) 전수 검사 (환경/버전 이슈 시 안전)
            for i in range(len(segs)):
                # i와 인접한 j(i-1, i, i+1)는 스킵 → i+2부터
                for j in range(i+2, len(segs)):
                    inter = segs[i].intersection(segs[j])
                    if inter.is_empty:
                        continue
                    if inter.geom_type == "Point":
                        pts.append(inter)
                    elif inter.geom_type == "MultiPoint":
                        pts.extend(list(inter.geoms))
            return pts

    # ─────────────────────────────────────────────────────────
    # ★ (교체) 교차점 주변 컷아웃(원형) 작성
    # ─────────────────────────────────────────────────────────
    def _build_cross_cutmask(xy_np: np.ndarray, cut_radius: float):
        pts = _non_adjacent_cross_points(xy_np)
        if not pts:
            return None
        disks = [p.buffer(cut_radius, cap_style=1, join_style=1) for p in pts]
        return unary_union(disks) if len(disks) > 1 else disks[0]

    # ─────────────────────────────────────────────────────────
    # ★ 교차(자가교차/비인접 세그먼트)로 생기는 '렌즈' 영역 마스크
    #   - 각 세그먼트의 buffer(r) 쌍 교집합을 모아 union
    #   - 약간 키워서(ε) 끈끈이 연결까지 차단
    # ─────────────────────────────────────────────────────────
    def _spurious_overlap_mask(center_xy: np.ndarray, width: float):
        if center_xy is None or len(center_xy) < 3:
            return None

        r = width * 0.5
        # 세그먼트 라인 & 버퍼 준비
        segs  = [LineString([center_xy[i], center_xy[i+1]]) for i in range(len(center_xy)-1)]
        buffs = [s.buffer(r, cap_style=CAP_SQUARE, join_style=JOIN_MITRE, mitre_limit=MITRE_LIMIT)
                 for s in segs]

        # 후보쌍 추출 (빠른 경로: query_bulk)
        try:
            import numpy as _np
            tree  = STRtree(buffs)
            pairs = _np.array(tree.query_bulk(buffs)).T
        except Exception:
            pairs = [(i, j) for i in range(len(buffs)) for j in range(i+1, len(buffs))]

        cuts, seen = [], set()
        for i, j in pairs:
            if i == j or abs(i - j) <= 1:   # 자기/인접 세그먼트는 허용 (연결 유지)
                continue
            if i > j:
                i, j = j, i
            if (i, j) in seen:
                continue
            seen.add((i, j))

            inter = buffs[i].intersection(buffs[j])
            if inter.is_empty:
                continue
            # 너무 작은 찌꺼기 제외 (폭^2 대비 비율)
            if inter.area < (width * width * 0.02):
                continue
            cuts.append(inter)

        if not cuts:
            return None

        # 렌즈 영역을 조금 키워 연결고리까지 잘라낸다 (폭의 6% 정도)
        mask = unary_union(cuts).buffer(width * 0.06, cap_style=1, join_style=1)
        return mask if (mask and not mask.is_empty) else None

    # ── 이하 기존 로직 (전개 생략 없이 핵심만 표시) ──────────
    if uav_cnt < 1:
        raise ValueError("uav_cnt must be ≥ 1")

    coords_llh = line_seg["coordinateList"]
    if len(coords_llh) < 2:
        raise ValueError("coordinateList must contain ≥ 2 points")

    lat0, lon0 = coords_llh[0]["latitude"], coords_llh[0]["longitude"]
    xy = np.array([llh_to_xy(p["latitude"], p["longitude"], lat0, lon0)
                   for p in coords_llh], dtype=float)
    line_world = LineString(xy)

    total_w = float(line_seg["width"])
    indiv_w = total_w / uav_cnt
    half_w  = total_w * 0.5

    JOIN_MITRE  = 2
    CAP_SQUARE  = 2
    MITRE_LIMIT = 10.0

    def _largest_polygon(g):
        if isinstance(g, Polygon):
            return g
        if g.geom_type == "MultiPolygon":
            return max(g.geoms, key=lambda a: a.area) if g.geoms else Polygon()
        if g.geom_type == "GeometryCollection":
            polys = [h for h in g.geoms if isinstance(h, Polygon)]
            return max(polys, key=lambda a: a.area) if polys else Polygon()
        return Polygon()

    def _offset_line(ls: LineString, dist: float):
        side = "left" if dist >= 0 else "right"
        return ls.parallel_offset(abs(dist), side,
                                  join_style=JOIN_MITRE, mitre_limit=MITRE_LIMIT)

    corridor_poly = line_world.buffer(
        half_w, cap_style=CAP_SQUARE, join_style=JOIN_MITRE, mitre_limit=MITRE_LIMIT
    )
    if not corridor_poly.is_valid:
        corridor_poly = corridor_poly.buffer(0)

    # ★ 교차부 컷아웃 마스크(폴리라인 전체 기준 1회만 계산)
    cross_cutmask = _build_cross_cutmask(xy, cut_radius=indiv_w * 0.51)

    strips: list[dict] = []
    for i in range(uav_cnt):
        d_outer  =  half_w - i * indiv_w
        d_inner  =  d_outer - indiv_w
        d_center =  d_outer - indiv_w * 0.5

        try:
            center_ls = _offset_line(line_world, d_center)

            center_geom = center_ls
            if center_geom.geom_type == "MultiLineString":
                center_geom = linemerge(center_geom)
            if center_geom.geom_type == "GeometryCollection":
                lines = [g for g in center_geom.geoms if isinstance(g, LineString)]
                center_geom = max(lines, key=lambda g: g.length) if lines else LineString()

            if isinstance(center_geom, LineString) and not center_geom.is_empty:
                center_xy = np.array(center_geom.coords, dtype=float)
            else:
                cs_xy = np.array(center_ls.coords[0], dtype=float)
                ce_xy = np.array(center_ls.coords[-1], dtype=float)
                center_xy = np.vstack([cs_xy, ce_xy])

            if len(center_xy) >= 2:
                dist_start = np.linalg.norm(center_xy[0] - xy[0])
                dist_end   = np.linalg.norm(center_xy[-1] - xy[0])
                if dist_start > dist_end:
                    center_xy = center_xy[::-1]

            strip_poly = LineString(center_xy).buffer(
                indiv_w * 0.5, cap_style=CAP_SQUARE, join_style=JOIN_MITRE, mitre_limit=MITRE_LIMIT
            )
            if not strip_poly.is_valid:
                strip_poly = strip_poly.buffer(0)

            poly = strip_poly.intersection(corridor_poly)
            if not poly.is_valid:
                poly = poly.buffer(0)

        except Exception:
            buf_poly = line_world.buffer(indiv_w * 0.5,
                                         cap_style=CAP_SQUARE, join_style=JOIN_MITRE, mitre_limit=MITRE_LIMIT)

            v = xy[1] - xy[0];  v /= np.linalg.norm(v)
            w = np.array([-v[1], v[0]])
            offset_dist = -half_w + (i + 0.5) * indiv_w
            strip_poly = translate(buf_poly, xoff=w[0]*offset_dist, yoff=w[1]*offset_dist)
            poly = strip_poly.intersection(corridor_poly)
            if not poly.is_valid:
                poly = poly.buffer(0)
            center_xy = np.array([pt + w * offset_dist for pt in xy], dtype=float)

        if poly.is_empty or (not isinstance(poly, Polygon)) or len(poly.exterior.coords) < 4:
            raise ValueError(f"strip #{i+1} polygon empty after generation")

        # ─────────────────────────────────────────────
        # ★ 최종: 비인접 세그먼트 교차부 '컷아웃' 적용
        # ─────────────────────────────────────────────
        if (cross_cutmask is not None) and (not cross_cutmask.is_empty):
            cut_result = poly.difference(cross_cutmask)
            if not cut_result.is_empty:
                poly = cut_result

        # ─────────────────────────────────────────────
        # ★ 교차부 렌즈 마스크 차감 + 조각 분해 처리
        # ─────────────────────────────────────────────
        spmask = _spurious_overlap_mask(center_xy, indiv_w)
        if (spmask is not None) and (not spmask.is_empty):
            # 차감
            cut_geom = poly.difference(spmask)
            if not cut_geom.is_empty:
                poly = cut_geom

        # 결과가 MultiPolygon/GeometryCollection이면 조각별로 처리
        pieces = []
        if isinstance(poly, Polygon):
            pieces = [poly]
        elif poly.geom_type in ("MultiPolygon", "GeometryCollection"):
            pieces = [g for g in poly.geoms if isinstance(g, Polygon)]

        if not pieces:
            raise ValueError(f"strip #{i+1} polygon empty after cutting")

        # ── XY → LLH & strips에 조각별로 push ───────────────
        for piece in pieces:
            if piece.is_empty or len(piece.exterior.coords) < 4:
                continue

            coord_llh = [{
                "latitude":  _xy2llh(float(x), float(y), lat0, lon0)[0],
                "longitude": _xy2llh(float(x), float(y), lat0, lon0)[1],
                "altitude":  coords_llh[0].get("altitude", 0)
            } for (x, y) in list(piece.exterior.coords)[:-1]]

            center_llh = [{
                "latitude":  _xy2llh(float(px), float(py), lat0, lon0)[0],
                "longitude": _xy2llh(float(px), float(py), lat0, lon0)[1],
                "altitude":  coords_llh[0].get("altitude", 0),
            } for (px, py) in center_xy] or [
                {"latitude": _xy2llh(*xy[0], lat0, lon0)[0],
                 "longitude": _xy2llh(*xy[0], lat0, lon0)[1],
                 "altitude": coords_llh[0].get("altitude", 0)},
                {"latitude": _xy2llh(*xy[-1], lat0, lon0)[0],
                 "longitude": _xy2llh(*xy[-1], lat0, lon0)[1],
                 "altitude": coords_llh[-1].get("altitude", 0)},
            ]

            mean_alt, var_alt = altitude_stats_llh(coord_llh)

            strips.append({
                "Geometry":         "Area",
                "width":            indiv_w,
                "Centerline":       center_llh,
                "coordinateList":   coord_llh,
                "meanAltitude":     mean_alt,
                "altitudeVariance": var_alt
            })

    return strips

# def divide_corridor_polyline(line_seg: dict, uav_cnt: int) -> list[dict]:
#     """
#     꺾인 Center-line을 폭(width) 기준으로 uav_cnt개 Strip으로 분할.
#     급격한 굴절에서 '이중 꺾임'이 생기지 않도록,
#     ★ parallel_offset -> (센터라인 offset) + buffer + 전체코리도어 clip 방식으로 생성.
#     """
#     if uav_cnt < 1:
#         raise ValueError("uav_cnt must be ≥ 1")

#     coords_llh = line_seg["coordinateList"]
#     if len(coords_llh) < 2:
#         raise ValueError("coordinateList must contain ≥ 2 points")

#     lat0, lon0 = coords_llh[0]["latitude"], coords_llh[0]["longitude"]

#     # ── 1) LLH → ENU ──────────────────────────────────────────
#     xy = np.array([llh_to_xy(p["latitude"], p["longitude"], lat0, lon0)
#                    for p in coords_llh])
#     line_world = LineString(xy)

#     total_w = float(line_seg["width"])
#     indiv_w = total_w / uav_cnt
#     half_w  = total_w * 0.5

#     # ★ 상수: miter를 항상 1점으로 유지하도록 크게 설정
#     JOIN_MITRE   = 2
#     CAP_SQUARE   = 2
#     MITRE_LIMIT  = 10.0  # 90°(≈1.414)~급각에서도 bevel로 바뀌지 않게 충분히 크게

#     # ★ 부동소수 오차로 생기는 미세한 찌꺼기 제거용
#     def _largest_polygon(g):
#         if isinstance(g, Polygon):
#             return g
#         if g.geom_type == "MultiPolygon":
#             return max(g.geoms, key=lambda a: a.area) if g.geoms else Polygon()
#         if g.geom_type == "GeometryCollection":
#             polys = [h for h in g.geoms if isinstance(h, Polygon)]
#             return max(polys, key=lambda a: a.area) if polys else Polygon()
#         return Polygon()

#     # ★ offset 거리에 따라 side 자동
#     def _offset_line(ls: LineString, dist: float):
#         side = "left" if dist >= 0 else "right"
#         return ls.parallel_offset(abs(dist), side,
#                                   join_style=JOIN_MITRE, mitre_limit=MITRE_LIMIT)

#     # ★ 전체 코리도어(= 총 폭) 폴리곤: 모든 스트립을 여기 안에서만 허용
#     corridor_poly = line_world.buffer(
#         half_w, cap_style=CAP_SQUARE, join_style=JOIN_MITRE, mitre_limit=MITRE_LIMIT
#     )
#     if not corridor_poly.is_valid:
#         corridor_poly = corridor_poly.buffer(0)

#     strips: list[dict] = []
#     for i in range(uav_cnt):
#         d_outer  =  half_w - i * indiv_w          # 왼쪽(+)
#         d_inner  =  d_outer - indiv_w
#         d_center =  d_outer - indiv_w * 0.5

#         try:
#             # ── 2) ★ 센터라인 offset → buffer 로 스트립 생성 ─────────
#             center_ls = _offset_line(line_world, d_center)

#             center_geom = center_ls
#             if center_geom.geom_type == "MultiLineString":
#                 center_geom = linemerge(center_geom)
#             if center_geom.geom_type == "GeometryCollection":
#                 lines = [g for g in center_geom.geoms if isinstance(g, LineString)]
#                 center_geom = max(lines, key=lambda g: g.length) if lines else LineString()

#             if isinstance(center_geom, LineString) and not center_geom.is_empty:
#                 center_xy = np.array(center_geom.coords, dtype=float)
#             else:
#                 # fallback: endpoints만
#                 cs_xy = np.array(center_ls.coords[0], dtype=float)
#                 ce_xy = np.array(center_ls.coords[-1], dtype=float)
#                 center_xy = np.vstack([cs_xy, ce_xy])

#             # 원래 라인의 시작점 방향과 정렬
#             if len(center_xy) >= 2:
#                 dist_start = np.linalg.norm(center_xy[0] - xy[0])
#                 dist_end   = np.linalg.norm(center_xy[-1] - xy[0])
#                 if dist_start > dist_end:
#                     center_xy = center_xy[::-1]

#             # ★ 핵심: buffer로 폴리곤 생성(코너당 꼭짓점 1개, miter 유지)
#             strip_poly = LineString(center_xy).buffer(
#                 indiv_w * 0.5, cap_style=CAP_SQUARE, join_style=JOIN_MITRE, mitre_limit=MITRE_LIMIT
#             )
#             if not strip_poly.is_valid:
#                 strip_poly = strip_poly.buffer(0)

#             # ★ 전체 코리도어로 clip하여 넘침 방지
#             poly = _largest_polygon(strip_poly.intersection(corridor_poly))
#             if not poly.is_valid:
#                 poly = poly.buffer(0)

#         except Exception:
#             # ── 2') 기존 직사각형 fallback ─────────────────────────
#             buf_poly = line_world.buffer(indiv_w * 0.5,
#                                          cap_style=CAP_SQUARE, join_style=JOIN_MITRE, mitre_limit=MITRE_LIMIT)

#             v = xy[1] - xy[0];  v /= np.linalg.norm(v)
#             w = np.array([-v[1], v[0]])
#             offset_dist = -half_w + (i + 0.5) * indiv_w

#             strip_poly = translate(buf_poly, xoff=w[0]*offset_dist, yoff=w[1]*offset_dist)
#             poly = _largest_polygon(strip_poly.intersection(corridor_poly))
#             center_xy = np.array([pt + w * offset_dist for pt in xy], dtype=float)

#         if poly.is_empty or (not isinstance(poly, Polygon)) or len(poly.exterior.coords) < 4:
#             raise ValueError(f"strip #{i+1} polygon empty after generation")

#         # ── 3) XY → LLH 변환 ─────────────────────────────────
#         coord_llh = [{
#             "latitude":  _xy2llh(float(x), float(y), lat0, lon0)[0],
#             "longitude": _xy2llh(float(x), float(y), lat0, lon0)[1],
#             "altitude":  coords_llh[0].get("altitude", 0)
#         } for (x, y) in list(poly.exterior.coords)[:-1]]

#         center_llh = [{
#             "latitude":  _xy2llh(float(px), float(py), lat0, lon0)[0],
#             "longitude": _xy2llh(float(px), float(py), lat0, lon0)[1],
#             "altitude":  coords_llh[0].get("altitude", 0),
#         } for (px, py) in center_xy]
#         if not center_llh:
#             cs_xy = xy[0]; ce_xy = xy[-1]
#             center_llh = [
#                 {"latitude": _xy2llh(*map(float, cs_xy), lat0, lon0)[0],
#                  "longitude": _xy2llh(*map(float, cs_xy), lat0, lon0)[1],
#                  "altitude": coords_llh[0].get("altitude", 0)},
#                 {"latitude": _xy2llh(*map(float, ce_xy), lat0, lon0)[0],
#                  "longitude": _xy2llh(*map(float, ce_xy), lat0, lon0)[1],
#                  "altitude": coords_llh[-1].get("altitude", 0)},
#             ]

#         # ── 4) DEM 기반 고도 통계 ───────────────────────────
#         mean_alt, var_alt = altitude_stats_llh(coord_llh)

#         strips.append({
#             "Geometry":         "Area",
#             "width":            indiv_w,
#             "Centerline":       center_llh,
#             "coordinateList":   coord_llh,
#             "meanAltitude":     mean_alt,
#             "altitudeVariance": var_alt
#         })

#     return strips




def divide_search_area_clip(
    area_poly: List[Dict], uav_cnt: int, bearing_deg: float
) -> List[Dict]:
    lat0, lon0 = area_poly[0]["latitude"], area_poly[0]["longitude"]
    alt0 = area_poly[0].get("altitude", 0)
    try:
        print(f"[AREA_DIVIDE] bearing_deg={bearing_deg:.2f}deg, uav_cnt={uav_cnt}")
    except Exception:
        pass

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

        # ── DEM 기반 고도 통계 ──
        mean_alt, var_alt = altitude_stats_llh(coord_llh)

        subareas.append({
            "Geometry":       "Area",
            "coordinateList": coord_llh,
            "meanAltitude":     mean_alt,
            "altitudeVariance": var_alt
        })

    return subareas

def _next_counter(key: str, start: int) -> int:
    """
    지정한 key(예: 'missionPlanID') 에 대해
    start 값부터 1씩 증가하며 uint32 범위 내에서 유일 ID를 반환.
    파일에 직전 값을 저장해 두므로 재-실행/재계획 시에도 안 겹침.
    """
    try:
        with open(_ID_COUNTER_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception:
        data = {}

    cur = data.get(key, start - 1)
    nxt = cur + 1
    if nxt > 0xFFFFFFFF:
        raise ValueError(f"{key} counter overflow (> 2³² – 1)")

    data[key] = nxt
    with open(_ID_COUNTER_FILE, "w", encoding="utf-8") as f:
        json.dump(data, f)
    return nxt

_id_seq: dict[str, int] = {}          # 카운터 저장소

def _next_counter(key: str, base: int) -> int:
    """
    base   : 시작값(첫 ID)
    return : 겹치지 않는 오름차순 ID
    """
    _id_seq[key] = _id_seq.get(key, 0) + 1
    return base + _id_seq[key] - 1



def gen_id(prefix: str, length: int = 8) -> str:
    """8‑byte 고유 ID 생성 (prefix 포함)"""
    return (prefix + datetime.utcnow().strftime("%y%m%d%H%M%S%f"))[:length]

used_ids = set()

def generate_unique_id(length=4):
    while True:
        new_id = ''.join(random.choices(string.ascii_uppercase + string.digits, k=length))
        if new_id not in used_ids:
            used_ids.add(new_id)
            return new_id
        
def ms_since_2000():
    """2000-01-01 00:00:00.000 ~ 현재까지 ms 반환 (Ulong 8byte)"""
    base = datetime(2000, 1, 1)
    now = datetime.utcnow()
    return int((now - base).total_seconds() * 1000)

def timestamp_since_2000_kst(at_time=None):
    """
    2000-01-01 00:00:00 KST 기준 ms 경과치의 하위 8자리(Ulong 8) 반환
    at_time: datetime 객체를 넘기면 해당 시점 기준, None이면 지금(now)
    """
    base = datetime(2000, 1, 1, 0, 0, 0, tzinfo=timezone(timedelta(hours=9)))
    if at_time is None:
        now = datetime.now(timezone(timedelta(hours=9)))
    else:
        if at_time.tzinfo is None:
            # naive datetime이면 KST로 간주
            now = at_time.replace(tzinfo=timezone(timedelta(hours=9)))
        else:
            now = at_time.astimezone(timezone(timedelta(hours=9)))
    elapsed = int((now - base).total_seconds() * 1000)
    return elapsed % 10**8

def current_timestamp_since_2000_kst():
    """2000.1.1 00:00:00(KST) 기준 ms 경과치의 하위 8자리 문자열 반환"""
    base = datetime(2000, 1, 1, 0, 0, 0, tzinfo=timezone(timedelta(hours=9)))
    now  = datetime.now(timezone(timedelta(hours=9)))
    elapsed = int((now - base).total_seconds() * 1000)
    return f"{elapsed % 10**8:08d}"


def write_json(path: str, obj):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(obj, f, indent=4, ensure_ascii=False)

def altitude_stats_llh(llh_list: list[dict]) -> tuple[float, float]:
    """
    4코너 LLH 좌표 리스트 → DEM 픽셀 샘플 → (mean, var) 고도 반환.
    • llh_list 는 {"latitude":…, "longitude":…} 사전 4개로 구성
    """
    if DEM_IMG is None:
        return 0.0, 0.0
    h, w = DEM_IMG.shape
    lat0, lon0 = llh_list[0]["latitude"], llh_list[0]["longitude"]
    vals = []
    for p in llh_list:
        x, y = llh_to_xy(p["latitude"], p["longitude"], lat0, lon0)
        px = int(round(x / DEM_RESOLUTION))
        py = int(round(y / DEM_RESOLUTION))
        if 0 <= px < w and 0 <= py < h:
            vals.append(DEM_IMG[py, px])
    return (float(np.mean(vals)), float(np.var(vals))) if vals else (0.0, 0.0)


# ──────────────────────────────────────────────────────────────
# 0-A.  분할 함수 (Movement / Search-Attack)
# ──────────────────────────────────────────────────────────────
def divide_movement_area(line_seg: dict, uav_cnt: int) -> list[dict]:
    """
    협업기동임무를 UAV 개수(uav_cnt)만큼 폭 방향으로 분할합니다.
    - 입력 line_seg: {"Width": 전체폭, "coordinateList": [시작점 LLH, 끝점 LLH]}
    - 반환: 각 UAV에게 할당될 사각형 임무 (폭 중심선 기반)
    """

    p0_llh, p1_llh = line_seg["coordinateList"]
    lat0, lon0 = p0_llh["latitude"], p0_llh["longitude"]
    total_width = line_seg["width"]

    # 1. ENU 좌표로 변환
    p0_xy = llh_to_xy(p0_llh["latitude"], p0_llh["longitude"], lat0, lon0)
    p1_xy = llh_to_xy(p1_llh["latitude"], p1_llh["longitude"], lat0, lon0)

    # 2. 진행 방향 unit vector
    vx, vy = p1_xy[0] - p0_xy[0], p1_xy[1] - p0_xy[1]
    length = math.hypot(vx, vy)
    unit_v = (vx / length, vy / length)

    # 3. 폭 방향 unit vector (진행 방향의 수직 벡터)
    wx, wy = -unit_v[1], unit_v[0]

    # 4. 각 UAV 폭 관련 정보
    indiv_width = total_width / uav_cnt
    half_width = indiv_width / 2

    # 5. 폭 중심선 offset 리스트 (-폭/2 + 간격 * (i+0.5))
    center_offsets = [(-total_width / 2) + indiv_width * (i + 0.5) for i in range(uav_cnt)]

    rects = []
    for i, offset in enumerate(center_offsets):
        # 중심선의 시작점과 끝점 좌표
        cs_xy = (p0_xy[0] + wx * offset, p0_xy[1] + wy * offset)
        ce_xy = (p1_xy[0] + wx * offset, p1_xy[1] + wy * offset)

        # 폭 방향으로 반폭 만큼 이동한 사각형의 네 꼭짓점
        def pt(base_xy, sign):
            return (base_xy[0] + wx * half_width * sign,
                    base_xy[1] + wy * half_width * sign)

        corners_xy = [
            pt(p0_xy, sign=1) if i == 0 else
            pt(p0_xy, sign=-1) if i == 1 else
            pt(p1_xy, sign=-1) if i == 2 else
            pt(p1_xy, sign=1) for i in range(4)
        ]

        corners_llh = [{
            "latitude": xy_to_llh(x, y, lat0, lon0)[0],
            "longitude": xy_to_llh(x, y, lat0, lon0)[1],
            "altitude": p0_llh.get("altitude", 0)
        } for x, y in corners_xy]

        # 중심선 LLH 변환
        cs_llh = {
            "latitude": xy_to_llh(cs_xy[0], cs_xy[1], lat0, lon0)[0],
            "longitude": xy_to_llh(cs_xy[0], cs_xy[1], lat0, lon0)[1],
            "altitude": p0_llh.get("altitude", 0)
        }
        ce_llh = {
            "latitude": xy_to_llh(ce_xy[0], ce_xy[1], lat0, lon0)[0],
            "longitude": xy_to_llh(ce_xy[0], ce_xy[1], lat0, lon0)[1],
            "altitude": p1_llh.get("altitude", 0)
        }

        mean_alt, var_alt = altitude_stats_llh(corners_llh)

        rects.append({
            "Geometry":        "Line",
            "width":           indiv_width,
            "Centerline":      [cs_llh, ce_llh],
            "coordinateList":  corners_llh,
            "meanAltitude":    mean_alt,
            "altitudeVariance":var_alt
        })

    return rects

def _centroid_llh(ll: list[dict]) -> dict:
    lat = sum(p["latitude"]  for p in ll) / len(ll)
    lon = sum(p["longitude"] for p in ll) / len(ll)
    return {"latitude": lat, "longitude": lon}

def _resolve_area_bearing(prev_pt: dict | None, poly_llh: list[dict]) -> tuple[dict, float]:
    """
    Always prefer prev_pt→centroid bearing when prev_pt exists; otherwise use default.
    """
    center = _centroid_llh(poly_llh)
    source = "default"
    dist_m: float | None = None
    if prev_pt is not None:
        dx, dy = _llh2xy(center["latitude"], center["longitude"],
                         prev_pt["latitude"], prev_pt["longitude"])
        dist_m = math.hypot(dx, dy)
        bearing = _bearing_deg(prev_pt, center)
        source = "prev"
    else:
        bearing = _DEFAULT_AREA_BEARING_DEG
    debug_parts = [
        f"center=({center['latitude']:.6f},{center['longitude']:.6f})",
        f"moveBearing={bearing:.2f}°",
        f"source={source}",
    ]
    if dist_m is not None and prev_pt is not None:
        debug_parts.append(f"prev=({prev_pt['latitude']:.6f},{prev_pt['longitude']:.6f})")
        debug_parts.append(f"dist={dist_m:.1f}m")
    try:
        print("[AREA_BEARING] " + " ".join(debug_parts))
    except Exception:
        pass
    return center, bearing

def _bearing_deg(p0: dict, p1: dict) -> float:
    """LLH 두 점 → 방위각(0°=북, 시계방향)"""
    import math
    lat0, lon0 = math.radians(p0["latitude"]),  math.radians(p0["longitude"])
    lat1, lon1 = math.radians(p1["latitude"]),  math.radians(p1["longitude"])
    d_lon = lon1 - lon0
    x = math.sin(d_lon) * math.cos(lat1)
    y = math.cos(lat0)*math.sin(lat1) - math.sin(lat0)*math.cos(lat1)*math.cos(d_lon)
    return (math.degrees(math.atan2(x, y)) + 360.0) % 360.0


def divide_search_area(area_poly: list[dict], uav_cnt: int) -> list[dict]:
    """
    area_poly : [ {latitude,longitude,altitude}, … ] (단일 다각형 가정)
    반환      : uav_cnt 개 사각형, 각 coordinateList에 altitude 포함
    """
    # 기준점 위경도 및 고도
    lat0, lon0 = area_poly[0]["latitude"], area_poly[0]["longitude"]
    alt0 = area_poly[0].get("altitude", 0)

    # XY로 변환
    poly_xy = [llh_to_xy(p["latitude"], p["longitude"], lat0, lon0) for p in area_poly]

    # TL, TR, BL, BR 분류
    poly_xy_sorted = sorted(poly_xy, key=lambda p: (p[1], p[0]))
    tl, tr = sorted(poly_xy_sorted[:2], key=lambda p: p[0])
    bl, br = sorted(poly_xy_sorted[2:], key=lambda p: p[0])

    subareas = []
    for i in range(uav_cnt):
        r1, r2 = i / uav_cnt, (i + 1) / uav_cnt
        def interp(a, b, r): return (a[0] + (b[0]-a[0])*r, a[1] + (b[1]-a[1])*r)

        # 네 코너 XY 계산
        r_xy = [
            interp(tl, tr, r1),
            interp(tl, tr, r2),
            interp(bl, br, r2),
            interp(bl, br, r1),
        ]

        # LLH로 변환 + altitude 채우기
        r_llh = [{
            "latitude":  xy_to_llh(x, y, lat0, lon0)[0],
            "longitude": xy_to_llh(x, y, lat0, lon0)[1],
            "altitude":  alt0
        } for (x, y) in r_xy]

        mean_alt, var_alt = altitude_stats_llh(r_llh)
        subareas.append({
            "Geometry":         "Area",
            "coordinateList":   r_llh,
            "meanAltitude":     mean_alt,
            "altitudeVariance": var_alt
        })

    return subareas

def split_mission_into_subareas(
        input_m: dict,
        uav_cnt: int,
        prev_pt: dict | None   # ❰❰ 추가
) -> list[dict]:

    mtype       = input_m["inputMissionType"]
    mission_id  = input_m["inputMissionID"]
    md          = input_m["missionDetail"]
    subs: list[dict] = []
    # ── corridor형 (1·7) ─────────────────────────────────
    if mtype in (1, 7):
        for seg in md["lineList"]:
            if mtype == 7:
                try:
                    width_val = float(seg.get("width", 0))
                except Exception:
                    width_val = 0.0
                if width_val <= 0:
                    seg["width"] = 1.0
            if prev_pt is not None:           # bearing 기록만 유지
                start_pt = seg["coordinateList"][0]
                seg["bearingFromPrev"] = _bearing_deg(prev_pt, start_pt)

            # ★ 새 함수 사용
            for r in divide_corridor_polyline(seg, uav_cnt):
                r.update({"inputMissionType": mtype, "MissionID": mission_id})
                subs.append(r)

    # ── area형 (2·3·4·5·6) ───────────────────────────────
    elif mtype in (2, 3, 4, 5, 6):
        poly = md["areaList"][0]["coordinateList"]
        center, bearing_move = _resolve_area_bearing(prev_pt, poly)
        bearing_split = (bearing_move + 90.0) % 360.0  # split lines parallel to move axis
        for r in divide_search_area_clip(poly, uav_cnt, bearing_split):
            r.update({"inputMissionType": mtype, "MissionID": mission_id})
            r["bearing_deg"] = bearing_move     # 기록: 실제 스윕 방향
            r["splitBearing_deg"] = bearing_split
            if isinstance(prev_pt, dict):
                r["prevPoint"] = {
                    "latitude": float(prev_pt.get("latitude", center["latitude"])),
                    "longitude": float(prev_pt.get("longitude", center["longitude"])),
                    "altitude": int(round(float(prev_pt.get("altitude", 0) or 0))),
                }
            else:
                r["prevPoint"] = {
                    "latitude": float(center["latitude"]),
                    "longitude": float(center["longitude"]),
                    "altitude": 0,
                }
            subs.append(r)
    else:
        raise ValueError(f"Unknown inputMissionType {mtype}")
    return subs

def save_lah_tasks(cmpk: dict, out_dir: str, log):
    # ── LAH 기체 추출 ─────────────────────────────────────────
    lah_ids: list[int] = []
    for ac in cmpk.get("availableAircraftList", []):
        n = _extract_aircraft_id(ac)
        if n is not None and 1 <= n <= 3:
            lah_ids.append(n)
    if not lah_ids:
        log("[save_lah_tasks] LAH가 없습니다.");  return []

    # ── 패키지 skeleton ──────────────────────────────────────
    ts = now_ms_since_2000()
    data_map: dict[int, dict] = {
        lah: {
            "timestamp": now_ms_since_2000(),
            "individualMissionPackageID": int(reserve_imp_ids(1)[0]),
            "aircraftID": lah,
            "individualMissionList": []
        } for lah in lah_ids
    }

    # ── CMPK InputMission → 개별 임무 변환 ────────────────
    for im in cmpk.get("inputMissionList", []):
        mtype        = im["inputMissionType"]
        md           = im["missionDetail"]
        input_id_u32 = _as_uint32(im["inputMissionID"])

        # corridor형 ------------------------------------------------
        if mtype in (1, 7) and "lineList" in md:
            for lah in lah_ids:
                for seg in md["lineList"]:
                    mission = {
                        "individualMissionID": int(reserve_individual_mission_ids(1)[0]),
                        "isDone": False,
                        "relatedMission": {
                            "relatedMissionType": 1,
                            "inputMissionID": input_id_u32,
                            "priorMissionID": 0
                        },
                        "individualMissionInfo": {
                            "individualMissionType": 7 if mtype in (1, 7) else 9,
                            "patternType": 10 if mtype in (1, 7) else 11,
                            "autoZoomIn": False,
                            "coordinateList": [
                                {
                                    "latitude":  p["latitude"],
                                    "longitude": p["longitude"],
                                    "altitude":  p.get("altitude", 0)
                                } for p in seg["coordinateList"]
                            ]
                        },
                        "pathID": int(reserve_path_ids(lah, 1)[0])
                    }
                    data_map[lah]["individualMissionList"].append(mission)

        # area형 ----------------------------------------------------
        elif mtype in (2, 3, 4, 5, 6) and "areaList" in md:
            p0 = md["areaList"][0]["coordinateList"][0]
            for lah in lah_ids:
                mission = {
                    "individualMissionID": int(reserve_individual_mission_ids(1)[0]),
                    "isDone": False,
                    "relatedMission": {
                        "relatedMissionType": 1,
                        "inputMissionID": input_id_u32,
                        "priorMissionID": 0
                    },
                    "individualMissionInfo": {
                        "individualMissionType": 9,
                        "patternType": 12,
                        "autoZoomIn": False,
                        "coordinateList": [{
                            "latitude":  p0["latitude"],
                            "longitude": p0["longitude"],
                            "altitude":  p0.get("altitude", 0)
                        }]
                    },
                    "pathID": int(reserve_path_ids(lah, 1)[0])              # ★ 바깥에 위치
                }
                data_map[lah]["individualMissionList"].append(mission)

        # 기타 ------------------------------------------------------
        else:
            for lah in lah_ids:
                log(f"[LAH{lah}] 임무조건불만족(type={mtype})")

    # ── 파일 저장 ─────────────────────────────────────────
    paths = []
    for lah, pkg in data_map.items():
        path = os.path.join(out_dir, f"IndividualMissionPlan_LAH{lah:03d}.json")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(pkg, f, indent=4, ensure_ascii=False)
        log(f"[LAH{lah}] LAH 임무파일 저장 → {path}")
        paths.append(path)

    return paths



 # ──────────────────────────────────────────────────────────────
# 0-C. 무인기 스케줄링 함수
# ──────────────────────────────────────────────────────────────
def run_pulp_scheduling(
    imp_path_in: str,
    imp_path_out: Optional[str] = None,
    uav_id_list: Optional[List[str]] = None,
    log: Callable[[str], None] = print,
) -> bool:
    t_start = time.perf_counter()
    t_load = time.perf_counter()
    log(f"[PuLP] 입력 IMP 로드 → {os.path.basename(imp_path_in)}")
    # ── 1) 파일 로드 ─────────────────────────────────────────
    if not os.path.exists(imp_path_in):
        log(f"[PuLP] 입력 IMP가 없습니다: {imp_path_in}")
        return False

    with open(imp_path_in, "r", encoding="utf-8") as f:
        imp = json.load(f)

    missions = imp.get("individualMissionList", [])
    if not missions:
        log("[PuLP] individualMissionList가 비어 있습니다.")
        return False

    # ── 2) UAV 목록 확보 ─────────────────────────────────────
    if uav_id_list is None:
        uav_id_list = [imp.get("aircraftID")] if imp.get("aircraftID") else []
    num_uavs = len(uav_id_list)
    if num_uavs == 0:
        log("[PuLP] UAV가 0대입니다.")
        return False

    # ── 3) (mid, est_time) 리스트 구성 ─────────────────────────
    mission_info: List[tuple[int, float]] = []
    for m in missions:
        mid = m.get("individualMissionID")
        est = 0.0
        try:
            est = m["individualMissionInfo"]["missionDetail"]["areaList"]\
                   .get("EstimatedMissionTime", 0.0)
        except Exception:
            pass
        mission_info.append((mid, est))

    n_m = len(mission_info)
    if n_m == 0:
        log("[PuLP] 스케줄 대상 임무가 없습니다.")
        return False

    # ── 4) 그룹핑: UAV 개수만큼씩 묶음 ─────────────────────────
    gsize = num_uavs
    groups = [mission_info[i:i+gsize] for i in range(0, n_m, gsize)]
    log(f"[PuLP] 임무 {n_m}개 → {len(groups)}개 그룹 (그룹당 ≤{gsize}개)")

    # ── 5) MILP 모델 구축 ─────────────────────────────────────
    prob = LpProblem("UAV_Schedule", LpMinimize)

    # x[u,g,i]: 그룹 g 의 태스크 i 를 UAV u 가 수행
    x: dict[tuple[int,int,int], LpVariable] = {}
    for u in range(num_uavs):
        for g, grp in enumerate(groups):
            for i in range(len(grp)):
                x[u,g,i] = LpVariable(f"x_{u}_{g}_{i}", cat="Binary")

    # 각 UAV의 총 작업시간 T[u], 최대/최소 Tmax/Tmin
    T   = [LpVariable(f"T_{u}", lowBound=0) for u in range(num_uavs)]
    Tmax = LpVariable("Tmax", lowBound=0)
    Tmin = LpVariable("Tmin", lowBound=0)

    # (1) 각 태스크는 정확히 1대 UAV에
    for g, grp in enumerate(groups):
        for i in range(len(grp)):
            prob += lpSum(x[u,g,i] for u in range(num_uavs)) == 1

    # (2) 같은 그룹에서 UAV당 최대 1개 태스크
    for g, grp in enumerate(groups):
        for u in range(num_uavs):
            prob += lpSum(x[u,g,i] for i in range(len(grp))) <= 1

    # (3) T[u] = 할당된 태스크들의 시간 합
    for u in range(num_uavs):
        prob += T[u] == lpSum(
            x[u,g,i] * groups[g][i][1]
            for g in range(len(groups))
            for i in range(len(groups[g]))
        )

    # (4) Tmax/Tmin 제약
    for u in range(num_uavs):
        prob += Tmax >= T[u]
        prob += Tmin <= T[u]

    # Objective: minimize (Tmax - Tmin)
    prob += Tmax - Tmin

    # ── 6) solve ──────────────────────────────────────────────
    status = prob.solve(PULP_CBC_CMD(msg=0))
    log(f"[PuLP] Status = {LpStatus[status]}")

    if LpStatus[status] != "Optimal":
        log("[PuLP] 최적해를 찾지 못했습니다. 원본 IMP 저장.")
        outp = imp_path_out or imp_path_in
        with open(outp, "w", encoding="utf-8") as f:
            json.dump(imp, f, indent=4, ensure_ascii=False)
        return False

    diff = value(Tmax) - value(Tmin)
    log(f"[PuLP] 최적 편차 = {diff:.2f} s")

    # ── 7) 결과 반영: 각 미션에 할당된 UAV 갱신 ─────────────────
    assign: dict[int, str] = {}
    for g, grp in enumerate(groups):
        for i, (mid, _) in enumerate(grp):
            for u in range(num_uavs):
                if value(x[u,g,i]) > 0.5:
                    assign[mid] = uav_id_list[u]
    for m in missions:
        mid = m.get("individualMissionID")
        if mid in assign:
            m["aircraftID"] = assign[mid]

    # ── 8) 파일 쓰기 ─────────────────────────────────────────
    outp = imp_path_out or imp_path_in
    with open(outp, "w", encoding="utf-8") as f:
        json.dump(imp, f, indent=4, ensure_ascii=False)
    log(f"[PuLP] 저장 완료 → {outp}")

    return True

# ──────────────────────────────────────────────────────────────
# 1.  CMPK → 개별 지역(Area) 분할  +  RL 패턴 결정
# ──────────────────────────────────────────────────────────────
def run_divide_and_pattern(
        cmpk_path: str,
        ref_path: str,
        out_dir: str,
        log: Callable[[str], None] = print,
        option_code: int | None = None,
) -> List[str]:
    t_start = time.perf_counter()
    t_load = time.perf_counter()
    log("[1] CMPK 로드")
    with open(cmpk_path, "r", encoding="utf-8") as f:
        cmpk = json.load(f)

    _apply_vehicle_status_filter(cmpk, log)

    log("    MRPK 로드")
    with open(ref_path, "r", encoding="utf-8") as f:
        mrpk = json.load(f)
    load_s = time.perf_counter() - t_load
    log(f"    ▸ 로드 완료  (소요 {load_s:.2f}s)")

    # ──────────────────────────────────────────────────────────
    # 0-A. 집결점(centroid) 계산  ▶ bearing 산출에 사용
    # ──────────────────────────────────────────────────────────
    def _centroid_llh(ll: list[dict]) -> dict:
        return {
            "latitude":  sum(p["latitude"]  for p in ll) / len(ll),
            "longitude": sum(p["longitude"] for p in ll) / len(ll),
        }

    take_over_cent = _centroid_llh(
        [t["coordinate"] for t in mrpk.get("takeOverInfoList", [])])
    hand_over_cent = _centroid_llh(
        [h["coordinate"] for h in mrpk.get("handOverInfoList", [])])

    # ──────────────────────────────────────────────────────────
    # 1. LAH 전용 IMP 저장 (변경 없음)
    # ──────────────────────────────────────────────────────────
    log("[2] LAH 임무 저장 시작")
    lah_t0 = time.perf_counter()
    lah_paths = save_lah_tasks(cmpk, out_dir, log) or []   # ← 항상 list
    lah_save_s = time.perf_counter() - lah_t0
    log(f"[TIME] LAH save: {lah_save_s:.2f}s")
    log(f"    ▸ LAH {len(lah_paths)}개 파일 저장")

    log("[3] InputMission 분할")
    total_im = len(cmpk.get("inputMissionList", []))
    prev_pt  = take_over_cent
    areas: list[dict] = []

    # ──────────────────────────────────────────────────────────
    # 2. UAV / LAH 목록 정규화  (dict 까지 처리)
    # ──────────────────────────────────────────────────────────
    def _norm(ac) -> str:
        if isinstance(ac, dict):
            ac = ac.get("aircraftID")
        if isinstance(ac, int):
            return f"LAH{ac:03d}" if ac < 4 else f"UAV{ac:03d}"
        if isinstance(ac, str):
            s = ac.upper()
            if s.startswith(("UAV", "LAH")):
                return s
            if s.isdigit():
                n = int(s)
                return f"LAH{n:03d}" if n < 4 else f"UAV{n:03d}"
        raise ValueError(f"unsupported aircraft entry: {ac!r}")

    aircrafts = [_norm(a) for a in cmpk.get("availableAircraftList", [])]
    uavs = [a for a in aircrafts if a.startswith("UAV")]
    if not uavs:
        raise RuntimeError("UAV 없음 → IMP 생성 불가")

    # ──────────────────────────────────────────────────────────
    # 3. InputMission → sub-area 분할 (+ bearing 적용)
    # ──────────────────────────────────────────────────────────
    log("[3] InputMission 분할")
    prev_pt = take_over_cent                     # 첫 bearing 기준점
    areas: list[dict] = []

    split_t0 = time.perf_counter()
    for im_idx, im in enumerate(cmpk.get("inputMissionList", []), 1):
        log(f"    ▸ ({im_idx}/{total_im}) "
            f"inputMissionID={im['inputMissionID']} type={im['inputMissionType']}")
        new_subs = split_mission_into_subareas(im, len(uavs), prev_pt)
        areas.extend(new_subs)
        log(f"       └─ sub-area {len(new_subs)}개 추가 (누적 {len(areas)})")

        # 다음 임무를 위한 prev_pt 갱신
        mtype = im.get("inputMissionType")
        md = im.get("missionDetail") or {}
        line_list = md.get("lineList") or []
        area_list = md.get("areaList") or []
        if mtype in (1, 7) and line_list:
            coords = (line_list[-1] or {}).get("coordinateList") or []
            if coords:
                prev_pt = coords[-1]
        elif area_list:
            poly = (area_list[0] or {}).get("coordinateList") or []
            if poly:
                prev_pt = _centroid_llh(poly)
        else:
            log(
                f"       [WARN] inputMissionID={im.get('inputMissionID')} "
                "has no usable geometry; keeping previous reference point."
            )

    log(f"    ▸ 분할 완료: {len(areas)}개 sub-area")

    # ──────────────────────────────────────────────────────────
    # 4. RL 패턴 선택 + 예상시간 계산  ―  GUI 실시간 로그 지원
    # ──────────────────────────────────────────────────────────
    split_s = time.perf_counter() - split_t0
    log(f"[TIME] InputMission split: {split_s:.2f}s")
    log("[4] RL 예상시간 계산")
    env = UnifiedMissionEnvironment(areas)
    env.flight_weight = env.imaging_weight = 0.5
    total_sub = len(areas)

    # ── (1) 더미 정책 ─────────────────────────────────────────
    class _Dummy:
        def predict(self, *_):
            k = random.choice(list(mission_patterns))
            return k, None          # (action, state)

    model, done, obs = _Dummy(), False, env.reset()

    # ── (2) 로그 주기 계산 ───────────────────────────────────
    def _log_every(n: int) -> int:
        if n <= 20:   return 1
        if n <= 50:   return 2
        if n <= 200:  return 5
        return 10
    LOG_EVERY = _log_every(total_sub)                ### NEW

    # ── (3) 구간별 소요시간 기록용 배열 ───────────────────────
    t_predict, t_step, t_dist = [], [], []           ### NEW
    t_start_all = time.perf_counter()                ### NEW

    while not done:
        t_step0 = time.perf_counter()

        # A) 정책 추론 -------------------------------------------------
        pat_key, _ = model.predict(obs)              # pattern key
        pat_idx    = list(mission_patterns).index(pat_key)
        t_pred1 = time.perf_counter()

        # B) 환경 진행 -------------------------------------------------
        obs, _, done, _ = env.step(pat_idx)
        t_env2 = time.perf_counter()

        # C) 비행거리 → 예상시간 --------------------------------------
        cur_idx  = env.current_mission_index         # 1-based
        cur_area = env.processed_missions[cur_idx-1]
        pat_info = mission_patterns[pat_key]

        dist = calculate_flight_distance(
            cur_area["coordinateList"],
            pat_info["비행 패턴"], pat_info["촬영 패턴"]
        )
        cur_area.update({
            "EstimatedMissionTime": dist / UAV_VELOCITY,
            "MissionPattern":       pat_info,
            "patternType":          pat_key,
        })
        t_dist3 = time.perf_counter()

        # ── 진행 로그 ──────────────────────────────────────────
        if cur_idx == 1 or cur_idx % LOG_EVERY == 0 or done:   ### CHG
            elapsed = t_dist3 - t_start_all
            log(f"       RL 진행 {cur_idx:>4}/{total_sub}  (소요 {elapsed:4.1f}s)")
            # Qt GUI : 즉시 화면 반영
            try:
                from PyQt5.QtWidgets import QApplication
                QApplication.processEvents()
            except Exception:
                pass

        # ── 소요시간 누적 ─────────────────────────────────────
        t_predict.append(t_pred1 - t_step0)
        t_step.append   (t_env2 - t_pred1)
        t_dist.append   (t_dist3 - t_env2)

    # ── (4) 요약 통계 출력 ───────────────────────────────────
    def _ms(arr): return [x*1000 for x in arr]
    import numpy as np
    log("       ┌─ RL 세부 소요(ms) ─────────────")
    log(f"       │ predict : {np.mean(_ms(t_predict)) :6.1f}  / p95 {np.percentile(_ms(t_predict),95):6.1f}")
    log(f"       │ env.step: {np.mean(_ms(t_step))    :6.1f}  / p95 {np.percentile(_ms(t_step),95)   :6.1f}")
    log(f"       │ distance: {np.mean(_ms(t_dist))    :6.1f}  / p95 {np.percentile(_ms(t_dist),95)   :6.1f}")
    log("       └───────────────────────────────")

    # 정찰특화(option=4)는 별도 runtime override를 허용하지만,
    # pattern 강제 자체는 하지 않고 기본 계획 로직 위에서 동작한다.
    if option_code == 4:
        log("[OPTION] 정찰특화: 전용 runtime override 허용, pattern 강제 없음")

    # ──────────────────────────────────────────────────────────
    # 5. 예상시간 기반 PuLP 스케줄링 (변경 없음)
    # ──────────────────────────────────────────────────────────
    rl_total = time.perf_counter() - t_start_all
    rl_s = rl_total
    log(f"[TIME] RL total: {rl_total:.2f}s")
    log("[5] PuLP 균등 스케줄링")
    flat_in  = os.path.join(out_dir, "_flat.json")
    flat_out = os.path.join(out_dir, "_flat_sched.json")
    write_json(flat_in, {
        "individualMissionList": [
            {"individualMissionID": i + 1,
             "individualMissionInfo": {"missionDetail": {
                 "areaList": {"EstimatedMissionTime": a["EstimatedMissionTime"]}}}}
            for i, a in enumerate(env.processed_missions)
        ]
    })
    pulp_t0 = time.perf_counter()
    run_pulp_scheduling(flat_in, flat_out, uavs, log)
    pulp_s = time.perf_counter() - pulp_t0
    log(f"[TIME] PuLP scheduling: {pulp_s:.2f}s")

    with open(flat_out, "r", encoding="utf-8") as f:
        sched = json.load(f)["individualMissionList"]

    # ──────────────────────────────────────────────────────────
    # 6. UAV IMP(0302) 생성 및 저장  (변경 없음)
    # ──────────────────────────────────────────────────────────
    uav_imp_t0 = time.perf_counter()
    log("[6] UAV IMP 생성‧저장")
    imp_paths, imp_objs = [], []
    uav_map: dict[str, dict] = {}

    # Formation leader selection (UAV 4 -> 5 -> 6)
    formation_candidates = []
    for uav in uavs:
        try:
            uav_num = int(uav[3:])
        except Exception:
            continue
        if uav_num in (4, 5, 6):
            formation_candidates.append(uav_num)
    formation_leader_id = min(formation_candidates) if formation_candidates else None
    if formation_leader_id is not None:
        followers = [aid for aid in sorted(formation_candidates) if aid != formation_leader_id]
        log(f"[FORMATION] leader UAV{formation_leader_id}, followers={followers}")

    for uav in uavs:
        num = int(uav[3:])
        pkg = {"timestamp": now_ms_since_2000(),
               "individualMissionPackageID": int(reserve_imp_ids(1)[0]),
               "aircraftID": num,
               "individualMissionList": []}
        uav_map[uav] = pkg
        imp_objs.append(pkg)
        imp_paths.append(os.path.join(out_dir, f"IndividualMissionPlan_{uav}.json"))

    for entry in sched:
        a_id = entry.pop("aircraftID")
        area = env.processed_missions[entry["individualMissionID"] - 1]
        tgt  = uav_map[a_id]

        orig = area["inputMissionType"]
        if orig in (1, 4, 5):                       # corridor
            im_type = 6
            detail = {"lineList": [{
                        "width": area["width"],
                        "coordinateList": area["Centerline"]}],
                      "targetID": None}
            pattern_type = area["patternType"]
        elif orig == 7:
            im_type = 7
            # Formation marker: width 0 = leader, width 1 = follower (UAV 4/5/6)
            width_tag = area.get("width", 1)
            try:
                aircraft_id = int(a_id[3:])
            except Exception:
                aircraft_id = None
            if formation_leader_id is not None and aircraft_id in (4, 5, 6):
                width_tag = 0 if aircraft_id == formation_leader_id else 1
                role = "leader" if width_tag == 0 else "follower"
                log(
                    f"[FORMATION] inputMissionID={area.get('MissionID')} "
                    f"aircraft=UAV{aircraft_id} role={role} width={width_tag}"
                )
            # type 7 is validated as a coordinate mission (0302),
            # but downstream (0303) can also consume lineList.
            # Provide both to avoid validation failures when line-only data arrives.
            detail = {
                "coordinateList": area["Centerline"],
                "lineList": [{
                    "width": width_tag,
                    "coordinateList": area["Centerline"],
                }],
                "targetID": None,
            }
            pattern_type = 10
        else:                                       # area
            im_type = 3 if orig in (2, 6) else 4
            detail = {"areaList": [{
                        "isHole": False,
                        "coordinateList": area["coordinateList"]}],
                      "targetID": None}
            pattern_type = area["patternType"]

        info = {"individualMissionType": im_type,
                "patternType": pattern_type,
                "autoZoomIn": True}
        info.update(detail)

        tgt["individualMissionList"].append({
            "individualMissionID": int(reserve_individual_mission_ids(1)[0]),
            "isDone": False,
            "relatedMission": {"relatedMissionType": 1,
                               "inputMissionID": _as_uint32(area["MissionID"]),
                               "priorMissionID": 0},
            "individualMissionInfo": info,
            "pathID": int(reserve_path_ids(int(a_id[3:]), 1)[0]),
            "bearing_deg": area.get("bearing_deg"),
            "prevPoint": area.get("prevPoint"),
        })

    for path, obj in zip(imp_paths, imp_objs):
        write_json(path, obj)
        log(f"    ▸ IMP 저장: {path}")

    # 임시 파일 정리
    for fp in (flat_in, flat_out):
        try:
            os.remove(fp)
        except Exception:
            pass

    total_s = time.perf_counter() - t_start
    log(f"[✔] divide_and_pattern 끝 (총 {total_s:.1f}s 경과)")
    uav_imp_s = time.perf_counter() - uav_imp_t0
    log(f"[TIME] UAV IMP build: {uav_imp_s:.2f}s, files={len(imp_paths)}")
    global _LAST_DIVIDE_AND_PATTERN_METRICS
    _LAST_DIVIDE_AND_PATTERN_METRICS = {
        "load_s": load_s,
        "lah_save_s": lah_save_s,
        "split_s": split_s,
        "rl_s": rl_s,
        "pulp_s": pulp_s,
        "uav_imp_s": uav_imp_s,
        "total_s": total_s,
    }
    return lah_paths + imp_paths




from datetime import datetime, timezone
import random

used_ids: set[int] = set()

def generate_unique_id() -> int:
    """충돌 없는 32-bit unsigned ID"""
    while True:
        new_id = random.randint(1, 0xFFFFFFFF)
        if new_id not in used_ids:
            used_ids.add(new_id)
            return new_id


def _as_uint32(val) -> int:
    """문자·정수 어떤 입력이든 uint32 로 반환  (파싱 실패 시 새 ID)"""
    try:
        return int(val) & 0xFFFFFFFF
    except Exception:
        return generate_unique_id()

# ──────────────────────────────────────────────────────────────
# 3.  MissionPlan(0301) 생성
# ──────────────────────────────────────────────────────────────
def build_mission_plan_0301(cmpk_path, mrpk_path, imp_paths, mp_out_path, mission_plan_id=None):
    start_time = time.time()
    # CMPK 로드
    with open(cmpk_path, "r", encoding="utf-8") as f:
        cmpk = json.load(f)
    # MRPK 로드
    with open(mrpk_path, "r", encoding="utf-8") as f:
        mrpk = json.load(f)
    # 실제 MRPK ID 사용
    mrpk_id = mrpk.get("missionReferencePackageID") or mrpk.get("inputMissionPackageID") or "MRPK0000"

    # ── aircraftList 구성 ───────────────────────────────
    def _load_imp_aircraft(imp_path):
        with open(imp_path, "r", encoding="utf-8") as f:
            imp = json.load(f)
        return {
            "aircraftID": imp["aircraftID"],
            "individualMissionPackageID": imp["individualMissionPackageID"],
        }

    if len(imp_paths) <= 1:
        aircraft_list = [_load_imp_aircraft(path) for path in imp_paths]
    else:
        aircraft_list = [None] * len(imp_paths)
        with concurrent.futures.ThreadPoolExecutor(max_workers=min(6, len(imp_paths)), thread_name_prefix="MP0301") as executor:
            futures = {
                executor.submit(_load_imp_aircraft, path): idx
                for idx, path in enumerate(imp_paths)
            }
            for future in concurrent.futures.as_completed(futures):
                aircraft_list[futures[future]] = future.result()
        aircraft_list = [row for row in aircraft_list if row is not None]

    # ── MissionPlan(0301) 빌드 ───────────────────────────
    plan_id = _as_uint32(mission_plan_id) if mission_plan_id is not None else int(reserve_mission_plan_ids(1)[0])
    mission_plan = {
        # ✅ 700 000 001 부터 1씩 증가
        "missionPlanID":            plan_id,
        "timestamp":                now_ms_since_2000(),
        "missionPlanTimestamp":     now_ms_since_2000(),
        "planningTime":             (time.time() - start_time) * 1000,
        "plannerID":                1,
        "inputMissionPackageID":    _as_uint32(cmpk.get("inputMissionPackageID") or 0),
        "missionReferencePackageID":_as_uint32(
                                      mrpk.get("missionReferencePackageID")
                                      or mrpk.get("inputMissionPackageID") or 0),
        "aircraftList":             aircraft_list,
    }

    write_json(mp_out_path, mission_plan)
    return mission_plan


# --- Enhanced planning pipeline override -------------------------------------
try:
    from ..planning_enhanced import run_enhanced_divide_and_pattern as _run_enhanced_divide_and_pattern
except Exception:
    try:
        from planning_enhanced import run_enhanced_divide_and_pattern as _run_enhanced_divide_and_pattern  # type: ignore
    except Exception:
        _run_enhanced_divide_and_pattern = None


def run_divide_and_pattern(
    cmpk_path: str,
    ref_path: str,
    out_dir: str,
    log: Callable[[str], None] = print,
    option_code: int | None = None,
    trust_input_aircraft: bool = False,
    shared_split_state: dict[str, Any] | None = None,
) -> List[str]:
    """Production entrypoint kept stable; implementation is delegated to the enhanced pipeline."""
    global _LAST_DIVIDE_AND_PATTERN_METRICS
    t0 = time.perf_counter()
    if _run_enhanced_divide_and_pattern is None:
        raise RuntimeError('enhanced mission planning pipeline import failed')
    paths = _run_enhanced_divide_and_pattern(
        cmpk_path=cmpk_path,
        ref_path=ref_path,
        out_dir=out_dir,
        log=log,
        option_code=option_code,
        trust_input_aircraft=trust_input_aircraft,
        shared_split_state=shared_split_state,
    )
    total_s = time.perf_counter() - t0
    _LAST_DIVIDE_AND_PATTERN_METRICS = {
        'load_s': 0.0,
        'lah_save_s': 0.0,
        'split_s': 0.0,
        'rl_s': 0.0,
        'pulp_s': 0.0,
        'uav_imp_s': 0.0,
        'total_s': float(total_s),
    }
    return paths
