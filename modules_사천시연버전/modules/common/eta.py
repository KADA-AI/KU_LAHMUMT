# /modules/common/eta.py
# ETA 주입 유틸 (0303 FlightPath 전용)
from __future__ import annotations
import math
from typing import Any, Dict, Iterable, List, Optional, Tuple

Coord = Dict[str, float]
Waypoint = Dict[str, Any]
FlightPlan = Dict[str, Any]

EARTH_R = 6371000.0  # meters

# ---------------- geometry ----------------
def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """수평거리(m). 위경도(deg) → m (고도는 무시: 항공 경로에서 수평 주행시간 지배적)"""
    to_rad = math.radians
    dlat = to_rad(lat2 - lat1)
    dlon = to_rad(lon2 - lon1)
    a = math.sin(dlat/2)**2 + math.cos(to_rad(lat1)) * math.cos(to_rad(lat2)) * math.sin(dlon/2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return EARTH_R * c

def _dist_m(a: Coord, b: Coord) -> float:
    return _haversine_m(a["latitude"], a["longitude"], b["latitude"], b["longitude"])

def _polyline_len_m(points: Iterable[Coord]) -> float:
    pts = list(points)
    if len(pts) < 2:
        return 0.0
    s = 0.0
    for i in range(1, len(pts)):
        s += _dist_m(pts[i-1], pts[i])
    return s

# ---------------- helpers ----------------
def _to_mps(v: Optional[float], assume_kmh_over: float = 70.0) -> Optional[float]:
    """속도 단위 추정: 70 초과면 km/h로 보고 m/s 변환, 그 이하면 m/s로 간주."""
    if v is None:
        return None
    try:
        x = float(v)
    except Exception:
        return None
    if x <= 0:
        return None
    return (x * 1000.0 / 3600.0) if (x > assume_kmh_over) else x

def _get_coord(wp: Waypoint) -> Optional[Coord]:
    c = wp.get("coordinate")
    if not isinstance(c, dict):
        return None
    if "latitude" in c and "longitude" in c:
        return {"latitude": float(c["latitude"]), "longitude": float(c["longitude"]), "altitude": float(c.get("altitude", 0.0))}
    return None

def _order_by_next_chain(waypoints: List[Waypoint]) -> List[Waypoint]:
    """
    nextWaypointID 체인으로 경로 순서를 복원. 실패 시 원래 순서 사용.
    - 시작점: 어떤 다른 wp의 nextWaypointID에도 등장하지 않는 waypointID
    - 종료: nextWaypointID == 0
    """
    id_map = {wp.get("waypointID"): wp for wp in waypoints if isinstance(wp.get("waypointID"), (int, float))}
    incoming = set()
    for wp in waypoints:
        nid = wp.get("nextWaypointID")
        if isinstance(nid, (int, float)) and nid > 0:
            incoming.add(int(nid))
    starts = [int(k) for k in id_map.keys() if int(k) not in incoming]
    if not starts:
        return list(waypoints)

    path: List[Waypoint] = []
    cur = id_map.get(starts[0])
    guard = 0
    while cur is not None and guard < 100000:
        path.append(cur)
        nxt = cur.get("nextWaypointID")
        cur = id_map.get(int(nxt)) if isinstance(nxt, (int, float)) and int(nxt) > 0 else None
        guard += 1
    return path if len(path) >= 1 else list(waypoints)

# ---------------- segment-time model ----------------
def _time_from_prev_to_curr_s(prev: Waypoint,
                              curr: Waypoint,
                              default_speed_mps: float) -> float:
    """
    prev → curr 구간 소요시간(초).
    - 기본: 수평거리 / 속도
    - prev.filmingProperty.lineSearch.coordinateList 가 있으면: [prev.coord] + polyline + [curr.coord]
      를 따라 이동 (searchSpeed 우선 적용)
    - prev.loiterProperty.time 있으면: 출발 전 체공 시간 추가
      (time 없고 radius/speed만 있으면 1회전(2πR)/speed 가정)
    """
    start = _get_coord(prev)
    goal  = _get_coord(curr)
    if not (start and goal):
        return 0.0

    # 1) 내부 작업(선회/체공) 시간
    loiter = prev.get("loiterProperty") or {}
    extra_hold_s = 0.0
    if isinstance(loiter, dict):
        t = loiter.get("time")
        if isinstance(t, (int, float)) and t > 0:
            extra_hold_s += float(t)
        else:
            R = float(loiter.get("radius", 0.0) or 0.0)
            v = _to_mps(loiter.get("speed")) or _to_mps(prev.get("speed")) or default_speed_mps
            if R > 0.0 and v > 0.0:
                # time for 1 turn if time not specified
                extra_hold_s += (2.0 * math.pi * R) / v

    # 2) 이동 경로 시간
    filming = prev.get("filmingProperty") or {}
    line = filming.get("lineSearch") or {}
    coords = line.get("coordinateList") if isinstance(line, dict) else None

    # 속도 결정: lineSearch.searchSpeed > prev.speed > default
    # lineSearch.searchSpeed is produced by MissionPlanner in m/s.
    # Use it as-is (do not apply km/h auto-conversion heuristics).
    v_mps = None
    if isinstance(line, dict):
        try:
            raw_search_speed = line.get("searchSpeed")
            if raw_search_speed is not None:
                parsed_speed = float(raw_search_speed)
                if parsed_speed > 0:
                    v_mps = parsed_speed
        except Exception:
            v_mps = None
    if not v_mps:
        v_mps = _to_mps(prev.get("speed")) or default_speed_mps

    if isinstance(coords, list) and len(coords) >= 1:
        # [prev] + polyline + [curr]
        poly_pts: List[Coord] = [start] + [
            {"latitude": float(c["latitude"]), "longitude": float(c["longitude"]), "altitude": float(c.get("altitude", 0.0))}
            for c in coords if isinstance(c, dict) and "latitude" in c and "longitude" in c
        ] + [goal]
        dist_m = _polyline_len_m(poly_pts)
    else:
        # 단순 직선 prev → curr
        dist_m = _dist_m(start, goal)

    move_s = (dist_m / v_mps) if v_mps and v_mps > 0 else 0.0
    return extra_hold_s + move_s

# ---------------- public API ----------------
def annotate_eta_flight_plan(fp: FlightPlan,
                             *,
                             default_speed_mps: float = 40.0,
                             waypoint_list_keys: Tuple[str, ...] = ("waypointList", "waypoints")) -> None:
    """
    fp["waypointList"](or "waypoints") 내부의 각 waypoint에 "eta"(초)를 주입.
    규칙:
      - 첫 번째 waypoint의 eta = 0
      - 그 뒤 i의 eta = (i-1)에서 i까지 이동시간 + (i-1)에서의 선회/체공 시간까지 누적
      - lineSearch가 있으면 prev→curr 구간에서 polyline 경로/속도 적용
    """
    wps: Optional[List[Waypoint]] = None
    for k in waypoint_list_keys:
        if isinstance(fp.get(k), list):
            wps = fp[k]
            break
    if not wps:
        return

    ordered = _order_by_next_chain(wps)
    if not ordered:
        return

    # 누적
    acc_s = 0.0
    # 첫 WP: ETA=0
    ordered[0]["eta"] = 0

    for i in range(1, len(ordered)):
        dt = _time_from_prev_to_curr_s(ordered[i-1], ordered[i], default_speed_mps=default_speed_mps)
        acc_s += dt
        ordered[i]["eta"] = int(round(acc_s))

def annotate_eta_done_flight_plan(fp: FlightPlan,
                                  *,
                                  default_speed_mps: float = 40.0,
                                  waypoint_list_keys: Tuple[str, ...] = ("waypointList", "waypoints"),
                                  min_terminal_eta_s: int = 1) -> None:
    """
    done/hold 위주의 짧은 경로가 all-zero ETA로 남지 않도록 보정.
    - 기본 ETA 주입은 annotate_eta_flight_plan을 그대로 사용
    - 최종 ETA가 0 이하이면 마지막 waypoint ETA를 최소값 이상으로 강제
    - 1점 경로는 그 waypoint ETA 자체를 최소값 이상으로 강제
    - ecf도 최종 ETA 기준으로 다시 맞춤
    """
    annotate_eta_flight_plan(
        fp,
        default_speed_mps=default_speed_mps,
        waypoint_list_keys=waypoint_list_keys,
    )

    wps: Optional[List[Waypoint]] = None
    for k in waypoint_list_keys:
        if isinstance(fp.get(k), list):
            wps = fp[k]
            break
    if not wps:
        return

    ordered = _order_by_next_chain(wps)
    if not ordered:
        return

    final_eta = 0.0
    try:
        final_eta = float(ordered[-1].get("eta", 0) or 0)
    except Exception:
        final_eta = 0.0

    if final_eta <= 0.0:
        if len(ordered) >= 2:
            enforced = _time_from_prev_to_curr_s(
                ordered[-2],
                ordered[-1],
                default_speed_mps=default_speed_mps,
            )
            final_eta = float(max(int(round(enforced)), int(min_terminal_eta_s)))
            ordered[-1]["eta"] = int(round(final_eta))
        else:
            final_eta = float(max(int(min_terminal_eta_s), 1))
            ordered[0]["eta"] = int(round(final_eta))

    if final_eta <= 0.0:
        final_eta = float(max(int(min_terminal_eta_s), 1))

    for idx, wp in enumerate(ordered):
        try:
            eta_val = float(wp.get("eta", 0) or 0)
        except Exception:
            eta_val = 0.0
        eta_val = min(max(eta_val, 0.0), final_eta)
        wp["ecf"] = 1.0 if idx == len(ordered) - 1 else round(eta_val / final_eta, 4)

def annotate_eta_list(fps: Iterable[FlightPlan], *, default_speed_mps: float = 40.0) -> None:
    for fp in fps or []:
        try:
            annotate_eta_flight_plan(fp, default_speed_mps=default_speed_mps)
        except Exception:
            # 개별 플랜 실패해도 전체는 진행
            pass
