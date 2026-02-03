from __future__ import annotations
import os
import math
from collections import OrderedDict
from typing import List, Tuple

try:
    from ..UAV_missionPlanning import UAVMissionPlanner
except Exception:
    from UAV_missionPlanning import UAVMissionPlanner
from .mission_helpers import now_ms_since_2000, terrain_elev
from .id_allocator import next_waypoint_id as _next_waypoint_id

def _sw_code(default: str = "MMR") -> str:
    """Resolve module code from KU_ROLE."""
    role = (os.environ.get("KU_ROLE") or "").lower()
    return {
        "mission": "MMR",
        "monitoring": "MSM",
        "decision": "MOB",
    }.get(role, default)

WP_INTERVAL_M = 500.0        
HOVER_HOLD_SEC = 10
Altitude_LAH = 300

def _lah_alt_agl(lat: float, lon: float, offset_m: float | int | None = None) -> int:
    try:
        ground = float(terrain_elev(lat, lon))
    except Exception:
        ground = 0.0
    try:
        offset = float(Altitude_LAH if offset_m is None else offset_m)
    except Exception:
        offset = float(Altitude_LAH)
    return int(round(ground + offset))
         
_DEFAULT_WP_EXT = OrderedDict([
    ("hovering", OrderedDict([("time", 0)])),
    ("loiter",   OrderedDict([
        ("radius",    0),
        ("direction", 0),
        ("time",      0),
        ("speed",     0),
    ])),
    ("attack",   OrderedDict([
        ("targetID",   0),     
        ("weaponType", 0),
    ])),
])

class _WPAllocator:
    def __init__(self, start: int | None = None):
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

def _offset_coord(lat: float, lon: float,
                  north_m: float = 0.0,
                  east_m: float  = 0.0) -> Tuple[float, float]:

    k = 111_132.92                         # m/deg (위도)
    dlat = north_m / k
    dlon = east_m  / (k * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon

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

def _validate_lah_flight_plans(pkts: List[dict]) -> None:
    seen_path = set()
    for pidx, pkt in enumerate(pkts, 1):
        aid = pkt["aircraftID"]
        if aid not in (1, 2, 3):
            raise ValueError(f"[0304] pkt#{pidx}: aircraftID must be 1~3")
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

    base_packets = build_lah_flight_plans_fixed(
        missions,
        cruise_speed = cruise_speed,
        wp_interval_m = wp_interval_m,
        wp_alloc = _WPAllocator(1),   
    )

    tk_list = (mrpk or {}).get("takeOverInfoList") or []
    rtb_list= (mrpk or {}).get("rtbCoordinateList") or []

    if tk_list:
        anchor = min(
            [it.get("coordinate", {}) for it in tk_list if it.get("coordinate")],
            key=lambda c: (c.get("latitude", 90), c.get("longitude", 180))
        )
        a_lat, a_lon = float(anchor["latitude"]), float(anchor["longitude"])
    else:
        if base_packets and base_packets[0]["lahWaypointList"]:
            c0 = base_packets[0]["lahWaypointList"][0]["coordinate"]
            a_lat, a_lon = float(c0["latitude"]), float(c0["longitude"])
        else:
            return base_packets 

    start_map = {
        1: _offset(a_lat, a_lon, north_m=-150.0, east_m=  0.0),
        2: _offset(a_lat, a_lon, north_m=-150.0, east_m=150.0),
        3: _offset(a_lat, a_lon, north_m=-150.0, east_m=300.0),
    }

    rtb_sorted = sorted(
        [p for p in rtb_list if "latitude" in p and "longitude" in p],
        key=lambda p: p["longitude"]
    )
    rtb_map = {i+1: rtb_sorted[i] for i in range(min(3, len(rtb_sorted)))}

    out_packets: List[dict] = []
    for pkt in base_packets:
        aid = pkt["aircraftID"]
        wplist = pkt.get("lahWaypointList") or []
        if not wplist:
            continue

        # start
        st_lat, st_lon = start_map.get(aid, (wplist[0]["coordinate"]["latitude"], wplist[0]["coordinate"]["longitude"]))
        start_alt = _lah_alt_agl(st_lat, st_lon, Altitude_LAH)
        start = {"latitude": st_lat, "longitude": st_lon, "altitude": start_alt}
        eta_s = _dist_ms(start, wplist[0]["coordinate"])
        wp_start = _mk_wp(st_lat, st_lon, start["altitude"], eta_s)

        # rtb
        rtb = rtb_map.get(aid)
        if rtb:
            end   = rtb
            alt_e = _lah_alt_agl(
                float(end["latitude"]),
                float(end["longitude"]),
                end.get("altitude", Altitude_LAH),
            )
        else:
            end   = wplist[-1]["coordinate"]
            alt_e = int(end.get("altitude", _lah_alt_agl(
                float(end["latitude"]),
                float(end["longitude"]),
                Altitude_LAH,
            )))
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
    
def build_lah_flight_plans_fixed(
    missions: List[dict],
    *,
    cruise_speed: float = 40.0,
    wp_interval_m: float = WP_INTERVAL_M,
    wp_alloc: _WPAllocator | None = None,
) -> List[dict]:

    wp_alloc = wp_alloc or _WPAllocator()
    now_ms   = now_ms_since_2000()
    packets: List[dict] = []

    for miss in missions:
        aid = miss["aircraftID"]
        if aid not in (1, 2, 3):
            continue  

        path_id = miss.get("pathID")
        if path_id is None:
            raise ValueError(f"[0304] aircraft {aid}: mission missing pathID")

        info   = miss.get("individualMissionInfo", {})
        coords = [(c["latitude"], c["longitude"])
                  for c in info.get("coordinateList", [])]
        if not coords:
            continue

        offset_north = 0.0
        if aid == 2:
            offset_north = 100.0      # +100 m north
        elif aid == 3:
            offset_north = -100.0     # -100 m north
        if offset_north:
            coords = [_offset_coord(lat, lon, north_m=offset_north)
                      for lat, lon in coords]

        wplist: List[OrderedDict] = []

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
                    alt = _lah_alt_agl(
                        float(sample.get("lat", 0.0)),
                        float(sample.get("lon", 0.0)),
                        Altitude_LAH,
                    )
                    wp = OrderedDict([
                        ("waypointID", wp_alloc.alloc()),
                        ("coordinate", {
                            "latitude":  round(float(sample.get("lat", 0.0)), 6),
                            "longitude": round(float(sample.get("lon", 0.0)), 6),
                            "altitude":  alt,
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

                alt = _lah_alt_agl(lat, lon, Altitude_LAH)
                wp = OrderedDict([
                    ("waypointID", wp_alloc.alloc()),
                    ("coordinate", {
                        "latitude":  round(lat, 6),
                        "longitude": round(lon, 6),
                        "altitude":  alt,
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
            first_wp = wplist[0]
            first_wp["speed"] = 10
            hover = first_wp.get("hovering")
            if isinstance(hover, dict):
                hover["time"] = HOVER_HOLD_SEC
            else:
                first_wp["hovering"] = {"time": HOVER_HOLD_SEC}

            last_wp = wplist[-1]
            if last_wp is not first_wp:
                last_wp["speed"] = 10
                hover_last = last_wp.get("hovering")
                if isinstance(hover_last, dict):
                    hover_last["time"] = HOVER_HOLD_SEC
                else:
                    last_wp["hovering"] = {"time": HOVER_HOLD_SEC}

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


