# ──────────────────────────────────────────────────────────────────
# d0304.py  ― LAH FlightPlan (aircraft 1·2·3) 고정 100 m 버전
#           · 2번: +100 m north, 3번: –100 m north offset
# ──────────────────────────────────────────────────────────────────
from __future__ import annotations
import os
import math
from collections import OrderedDict
from typing import List, Tuple

from UAV_missionPlanning import UAVMissionPlanner
from .mission_helpers import now_ms_since_2000

def _sw_code(default: str = "MMR") -> str:
    """Resolve module code from KU_ROLE."""
    role = (os.environ.get("KU_ROLE") or "").lower()
    return {
        "mission": "MMR",
        "monitoring": "MSM",
        "decision": "MOB",
    }.get(role, default)

WP_INTERVAL_M = 500.0        # Waypoint 간격(m) ─ 기본값

# ── 고정 WP 확장 블록 ─────────────────────────────────────────────
_DEFAULT_WP_EXT = OrderedDict([
    ("hovering", OrderedDict([("time", 0)])),
    ("loiter",   OrderedDict([
        ("radius",    0),
        ("direction", 0),
        ("time",      0),
        ("speed",     0),
    ])),
    ("attack",   OrderedDict([
        ("targetID",   0),      # ← 반드시 0
        ("weaponType", 0),
    ])),
])

# ── ID 할당기 ────────────────────────────────────────────────────
class _WPAllocator:
    def __init__(self, start: int = 1):
        self._next = start
    def alloc(self) -> int:
        if self._next > 65_535:
            raise RuntimeError("WaypointID pool exhausted")
        wid = self._next
        self._next += 1
        return wid

# ── 좌표 오프셋(N/E[m]) → lat/lon 변환 ───────────────────────────
def _offset_coord(lat: float, lon: float,
                  north_m: float = 0.0,
                  east_m: float  = 0.0) -> Tuple[float, float]:
    """
    위도·경도 점을 북쪽/동쪽 거리(m)만큼 평면 이동시킨 좌표 반환
    (+north = 북, +east = 동). 100 m 급 이동에 충분한 근사치.
    """
    k = 111_132.92                         # m/deg (위도)
    dlat = north_m / k
    dlon = east_m  / (k * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon

# ── 두 점 사이 고정 간격 분할 ────────────────────────────────────
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
    n_seg = int(dist // step_m)
    return [
        (
            lat1 + (lat2 - lat1) * (i * step_m / dist),
            lon1 + (lon2 - lon1) * (i * step_m / dist),
        )
        for i in range(1, n_seg + 1)
    ] + [p1]

# ── 패킷 검증 ────────────────────────────────────────────────────
def _validate_lah_flight_plans(pkts: List[dict]) -> None:
    seen_path = set()
    for pidx, pkt in enumerate(pkts, 1):
        aid = pkt["aircraftID"]
        if aid not in (1, 2, 3):
            raise ValueError(f"[0304] pkt#{pidx}: aircraftID must be 1–3")
        path_id = pkt["pathID"]
        if path_id in seen_path:
            raise ValueError(f"[0304] duplicate pathID {path_id}")
        seen_path.add(path_id)
        lo = {1:100_000_001, 2:200_000_001, 3:300_000_001}[aid]
        if not (lo <= path_id < lo+100_000_000):
            raise ValueError(f"[0304] aircraft {aid}: pathID {path_id} out of range")

        for widx, wp in enumerate(pkt["lahWaypointList"], 1):
            atk = wp.get("attack")
            if atk is None:
                raise ValueError(f"[0304] pkt#{pidx}/wp#{widx}: 'attack' 필드 없음")
            tid = atk.get("targetID")
            if not isinstance(tid, int):
                raise ValueError(f"[0304] pkt#{pidx}/wp#{widx}: targetID must be int")
            if not (0 <= tid <= 0xFFFFFFFF):
                raise ValueError(f"[0304] pkt#{pidx}/wp#{widx}: targetID out of range")


def build_lah_flight_plans_from_mrpk(
    missions: List[dict],
    mrpk: dict,
    *,
    cruise_speed: float = 40.0,
    wp_interval_m: float = WP_INTERVAL_M,
    wp_alloc: _WPAllocator | None = None,
) -> List[dict]:
    """
    0302 → 0304(유인기) + 0203 고려
    • takeOverInfoList: LAH 시작 배치(최남/최서단 앵커 → S(-150m), E(+150m) 간격)
    • rtbCoordinateList: 각 LAH 의 RTB 최종 WP
    • 기존 0304 경로 앞뒤에 start/RTB를 붙인 뒤 WaypointID·ECF·next 링크 재계산
    """
    def _offset(lat: float, lon: float, north_m: float = 0.0, east_m: float = 0.0) -> tuple[float, float]:
        k = 111_132.92
        dlat = north_m / k
        dlon = east_m  / (k * math.cos(math.radians(lat)))
        return (lat + dlat, lon + dlon)

    def _dist_ms(a: dict, b: dict) -> int:
        k = 111_132.92
        lat1, lon1 = a["latitude"], a["longitude"]; lat2, lon2 = b["latitude"], b["longitude"]
        cos = math.cos(math.radians((lat1 + lat2)/2))
        dx = (lon2 - lon1) * k * cos; dy = (lat2 - lat1) * k
        m  = math.hypot(dx, dy)
        return int(round(1000 * m / max(1e-6, cruise_speed)))

    def _mk_wp(lat: float, lon: float, alt: float, eta_ms: int) -> OrderedDict:
        wp = OrderedDict([
            ("waypointID", 0),
            ("coordinate", {"latitude": round(lat,6), "longitude": round(lon,6), "altitude": int(round(alt))}),
            ("speed", cruise_speed),
            ("eta",   int(eta_ms)),
            ("ecf",   0.0),
            ("nextWaypointID", 0),
        ])
        wp.update(_DEFAULT_WP_EXT)
        return wp

    wp_alloc = wp_alloc or _WPAllocator()
    now_ms   = now_ms_since_2000()

    # 0) 기본 0304 (경로 본체) 먼저 생성
    base_packets = build_lah_flight_plans_fixed(
        missions,
        cruise_speed = cruise_speed,
        wp_interval_m = wp_interval_m,
        wp_alloc = _WPAllocator(1),    # 임시 할당 → 아래에서 다시 붙임
    )

    # 1) 0203: 앵커/RTB 계산
    tk_list = (mrpk or {}).get("takeOverInfoList") or []
    rtb_list= (mrpk or {}).get("rtbCoordinateList") or []

    # 앵커: 최남/최서단 (lat 최소, tie시 lon 최소)
    if tk_list:
        anchor = min(
            [it.get("coordinate", {}) for it in tk_list if it.get("coordinate")],
            key=lambda c: (c.get("latitude", 90), c.get("longitude", 180))
        )
        a_lat, a_lon = float(anchor["latitude"]), float(anchor["longitude"])
    else:
        # fallback: 첫 패킷 첫 WP 기준
        if base_packets and base_packets[0]["lahWaypointList"]:
            c0 = base_packets[0]["lahWaypointList"][0]["coordinate"]
            a_lat, a_lon = float(c0["latitude"]), float(c0["longitude"])
        else:
            return base_packets  # 아무것도 할 수 없음

    start_map = {
        1: _offset(a_lat, a_lon, north_m=-150.0, east_m=  0.0),
        2: _offset(a_lat, a_lon, north_m=-150.0, east_m=150.0),
        3: _offset(a_lat, a_lon, north_m=-150.0, east_m=300.0),
    }

    # RTB: 경도(서→동) 정렬 기준 LAH1/2/3 매핑 (부족하면 끝점 유지)
    rtb_sorted = sorted(
        [p for p in rtb_list if "latitude" in p and "longitude" in p],
        key=lambda p: p["longitude"]
    )
    rtb_map = {i+1: rtb_sorted[i] for i in range(min(3, len(rtb_sorted)))}

    # 2) 패킷별로 start/RTB 삽입 후 재계산
    out_packets: List[dict] = []
    for pkt in base_packets:
        aid = pkt["aircraftID"]
        wplist = pkt.get("lahWaypointList") or []
        if not wplist:
            continue

        # start
        st_lat, st_lon = start_map.get(aid, (wplist[0]["coordinate"]["latitude"], wplist[0]["coordinate"]["longitude"]))
        start = {"latitude": st_lat, "longitude": st_lon, "altitude": 300}
        eta_s = _dist_ms(start, wplist[0]["coordinate"])
        wp_start = _mk_wp(st_lat, st_lon, start["altitude"], eta_s)

        # rtb
        rtb = rtb_map.get(aid)
        if rtb:
            end   = rtb
            alt_e = int(rtb.get("altitude", 300))
        else:
            end   = wplist[-1]["coordinate"]
            alt_e = int(end.get("altitude", 300))
        eta_e = _dist_ms(wplist[-1]["coordinate"], end)
        wp_rtb = _mk_wp(end["latitude"], end["longitude"], alt_e, eta_e)

        new_list = [wp_start] + [dict(w) for w in wplist] + [wp_rtb]

        # WaypointID 재할당 + next + ECF
        for w in new_list:
            w["waypointID"] = wp_alloc.alloc()
        for i in range(len(new_list) - 1):
            new_list[i]["nextWaypointID"] = new_list[i+1]["waypointID"]

        tot = sum(max(0, int(w.get("eta", 0))) for w in new_list) or 1
        acc = 0
        for w in new_list:
            acc += int(w.get("eta", 0))
            w["ecf"] = round(acc / tot, 2)

        out_packets.append(OrderedDict([
            ("timestamp",  now_ms),
            ("pathID",     pkt["pathID"]),
            ("aircraftID", aid),
            ("lahWaypointList", new_list),
        ]))

    _validate_lah_flight_plans(out_packets)
    return out_packets
    
# ── 메인 빌더 ────────────────────────────────────────────────────
def build_lah_flight_plans_fixed(
    missions: List[dict],
    *,
    cruise_speed: float = 40.0,
    wp_interval_m: float = WP_INTERVAL_M,
    wp_alloc: _WPAllocator | None = None,
) -> List[dict]:
    """
    0302 -> 0304 FlightPlan 변환
    · LAH-1 은 기본 좌표 그대로 사용
    · LAH-2 는 경로 전체에 +100 m 북쪽(offset = +100 m north)
    · LAH-3 는 경로 전체에 -100 m 북쪽(offset = -100 m north)
    """
    wp_alloc = wp_alloc or _WPAllocator()
    now_ms   = now_ms_since_2000()
    packets: List[dict] = []

    for miss in missions:
        aid = miss["aircraftID"]
        if aid not in (1, 2, 3):
            continue  # LAH 기체가 아니면 skip

        path_id = miss.get("pathID")
        if path_id is None:
            raise ValueError(f"[0304] aircraft {aid}: mission missing pathID")

        info   = miss.get("individualMissionInfo", {})
        coords = [(c["latitude"], c["longitude"])
                  for c in info.get("coordinateList", [])]
        if not coords:
            continue

        # --- 100 m offset 적용 (LAH-2/3) ---
        offset_north = 0.0
        if aid == 2:
            offset_north = 100.0      # +100 m north
        elif aid == 3:
            offset_north = -100.0     # -100 m north
        if offset_north:
            coords = [_offset_coord(lat, lon, north_m=offset_north)
                      for lat, lon in coords]

        wplist: List[OrderedDict] = []

        # --- 동역학 기반 궤적 샘플링 ---
        if len(coords) >= 2:
            try:
                samples = UAVMissionPlanner.plan_route_only(
                    coords,
                    cruise_speed=cruise_speed,
                    heading_tol_deg=15.0,
                )
            except Exception:
                samples = []
            if samples:
                total_ms = sum(max(0, int(s.get("eta_ms", 0))) for s in samples) or 1
                cum_ms = 0
                for idx, sample in enumerate(samples):
                    if idx > 0:
                        prev = max(0, int(samples[idx - 1].get("eta_ms", 0)))
                        cum_ms += prev
                    eta_ms = int(cum_ms)
                    ecf = 1.0 if idx == len(samples) - 1 else round(cum_ms / total_ms, 2)
                    wp = OrderedDict([
                        ("waypointID", wp_alloc.alloc()),
                        ("coordinate", {
                            "latitude":  round(float(sample.get("lat", 0.0)), 6),
                            "longitude": round(float(sample.get("lon", 0.0)), 6),
                            "altitude":  300,
                        }),
                        ("speed", cruise_speed),
                        ("eta",   eta_ms),
                        ("ecf",   ecf),
                        ("nextWaypointID", 0),
                    ])
                    wp.update(_DEFAULT_WP_EXT)
                    wplist.append(wp)

        # --- fallback: 기존 직선 분할 ---
        if not wplist:
            path: List[Tuple[float, float]] = [coords[0]]
            if len(coords) > 1:
                for p, q in zip(coords, coords[1:]):
                    path.extend(_split_line(p, q, step_m=wp_interval_m))

            total_len = max((len(path) - 1) * wp_interval_m, 1.0)
            cum_len   = 0.0
            for idx, (lat, lon) in enumerate(path):
                if idx:
                    cum_len += wp_interval_m
                eta_ms = int(cum_len / cruise_speed * 1000) if idx else 0
                ecf    = 1.0 if len(path) == 1 else round(cum_len / total_len, 2)

                wp = OrderedDict([
                    ("waypointID", wp_alloc.alloc()),
                    ("coordinate", {
                        "latitude":  round(lat, 6),
                        "longitude": round(lon, 6),
                        "altitude":  300,
                    }),
                    ("speed", cruise_speed),
                    ("eta",   eta_ms),
                    ("ecf",   ecf),
                    ("nextWaypointID", 0),
                ])
                wp.update(_DEFAULT_WP_EXT)
                wplist.append(wp)

        for i in range(len(wplist) - 1):
            wplist[i]["nextWaypointID"] = wplist[i + 1]["waypointID"]
        if wplist:
            wplist[-1]["ecf"] = 1.0

        packets.append(OrderedDict([
            ("timestamp",   now_ms),
            ("Source", _sw_code()),
            ("pathID",      path_id),
            ("aircraftID",  aid),
            ("lahWaypointList", wplist),
        ]))

    _validate_lah_flight_plans(packets)
    return packets

