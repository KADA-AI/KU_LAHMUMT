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
from .id_allocator import (
    next_waypoint_id as _next_waypoint_id,
    reserve_waypoint_block as _reserve_waypoint_block,
)

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
HOVER_LAST_SEC = 30
ALTITUDE_LAYERS_M = (610.0, 620.0, 630.0)

def _lah_alt_agl(lat: float, lon: float, offset_m: float | int | None = None) -> int:
    try:
        ground = float(terrain_elev(lat, lon))
    except Exception:
        ground = 0.0
    try:
        offset = float(ALTITUDE_LAYERS_M[0] if offset_m is None else offset_m)
    except Exception:
        offset = float(ALTITUDE_LAYERS_M[0])
    return int(round(ground + offset))


def _aircraft_alt_offset_m(aid: int) -> float:
    try:
        idx = (int(aid) - 1) % len(ALTITUDE_LAYERS_M)
    except Exception:
        idx = 0
    return float(ALTITUDE_LAYERS_M[idx])


def _median_ground_m(points: list[tuple[float, float]]) -> float | None:
    if not points:
        return None
    samples: list[float] = []
    for lat, lon in points:
        try:
            samples.append(float(terrain_elev(lat, lon)))
        except Exception:
            continue
    if not samples:
        return None
    samples.sort()
    n = len(samples)
    mid = n // 2
    if n % 2:
        return samples[mid]
    return (samples[mid - 1] + samples[mid]) / 2.0
         
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


def _strip_wp_extras(wp: dict) -> None:
    for key in ("hovering", "loiter", "attack"):
        if key in wp:
            del wp[key]

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
                continue
            tid = atk.get("targetID")
            if not isinstance(tid, int):
                raise ValueError(f"[0304] pkt#{pidx}/wp#{widx}: targetID must be int")
            if not (0 <= tid <= 0xFFFFFFFF):
                raise ValueError(f"[0304] pkt#{pidx}/wp#{widx}: targetID out of range")


def build_lah_flight_plans_from_mrpk(
    missions: List[dict],
    mrpk: dict,
    *,
    cruise_speed: float = 15.0,
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
            ("isDone", False),
            ("coordinate", {"latitude": round(lat,6), "longitude": round(lon,6), "altitude": int(round(alt))}),
            ("speed", cruise_speed),
            ("eta",   int(eta_ms)),
            ("ecf",   0.0),
            ("nextWaypointID", 0),
        ])
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
        aircraft_alt_offset = _aircraft_alt_offset_m(aid)
        wplist = pkt.get("lahWaypointList") or []
        if not wplist:
            continue

        # start
        st_lat, st_lon = start_map.get(aid, (wplist[0]["coordinate"]["latitude"], wplist[0]["coordinate"]["longitude"]))
        start_alt = _lah_alt_agl(st_lat, st_lon, aircraft_alt_offset)
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
                aircraft_alt_offset,
            )
        else:
            end   = wplist[-1]["coordinate"]
            alt_e = int(end.get("altitude", _lah_alt_agl(
                float(end["latitude"]),
                float(end["longitude"]),
                aircraft_alt_offset,
            )))
        eta_e = _dist_ms(wplist[-1]["coordinate"], end)
        wp_rtb = _mk_wp(end["latitude"], end["longitude"], alt_e, eta_e)

        new_list = [wp_start] + [dict(w) for w in wplist] + [wp_rtb]

        # WaypointID 재할당 + next + ECF
        tot = sum(max(0, int(w.get("eta", 0))) for w in new_list) or 1
        acc = 0
        for w in new_list:
            acc += int(w.get("eta", 0))
            w["ecf"] = round(acc / tot, 2)

        for w in new_list:
            _strip_wp_extras(w)
        if new_list:
            last_wp = new_list[-1]
            last_wp["hovering"] = {"time": HOVER_LAST_SEC}

        out_packets.append(OrderedDict([
            ("timestamp",  now_ms),
            ("Source", _sw_code()),
            ("pathID",     pkt["pathID"]),
            ("aircraftID", aid),
            ("lahWaypointList", new_list),
        ]))

    if getattr(wp_alloc, "_use_global", False):
        total_wp_count = sum(len(pkt.get("lahWaypointList") or []) for pkt in out_packets)
        if total_wp_count > 0:
            wp_alloc = _WPAllocator(start=int(_reserve_waypoint_block(total_wp_count)))

    for pkt in out_packets:
        new_list = pkt.get("lahWaypointList") or []
        for w in new_list:
            w["waypointID"] = wp_alloc.alloc()
        for i in range(len(new_list) - 1):
            new_list[i]["nextWaypointID"] = new_list[i + 1]["waypointID"]

    _validate_lah_flight_plans(out_packets)
    return out_packets
    
def build_lah_flight_plans_fixed(
    missions: List[dict],
    *,
    cruise_speed: float = 15.0,
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

        aircraft_alt_offset = _aircraft_alt_offset_m(aid)
        mission_ground_ref = _median_ground_m(coords)

        def _mission_alt(lat: float, lon: float) -> int:
            if mission_ground_ref is None:
                return _lah_alt_agl(lat, lon, aircraft_alt_offset)
            return int(round(float(mission_ground_ref) + float(aircraft_alt_offset)))

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
                    alt = _mission_alt(
                        float(sample.get("lat", 0.0)),
                        float(sample.get("lon", 0.0)),
                    )
                    wp = OrderedDict([
                        ("waypointID", 0),
                        ("isDone", False),
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

                alt = _mission_alt(lat, lon)
                wp = OrderedDict([
                    ("waypointID", 0),
                    ("isDone", False),
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
                wplist.append(wp)

        if wplist:
            for w in wplist:
                w.setdefault("isDone", False)
            for w in wplist:
                _strip_wp_extras(w)
            last_wp = wplist[-1]
            last_wp["hovering"] = {"time": HOVER_LAST_SEC}
            wplist[-1]["ecf"] = 1.0

        packets.append(OrderedDict([
            ("timestamp",   now_ms),
            ("Source", _sw_code()),
            ("pathID",      path_id),
            ("aircraftID",  aid),
            ("lahWaypointList", wplist),
        ]))

    if getattr(wp_alloc, "_use_global", False):
        total_wp_count = sum(len(pkt.get("lahWaypointList") or []) for pkt in packets)
        if total_wp_count > 0:
            wp_alloc = _WPAllocator(start=int(_reserve_waypoint_block(total_wp_count)))

    for pkt in packets:
        wplist = pkt.get("lahWaypointList") or []
        for w in wplist:
            w["waypointID"] = wp_alloc.alloc()
        for i in range(len(wplist) - 1):
            wplist[i]["nextWaypointID"] = wplist[i + 1]["waypointID"]

    _validate_lah_flight_plans(packets)
    return packets


