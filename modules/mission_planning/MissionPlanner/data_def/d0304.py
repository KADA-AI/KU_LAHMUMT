from __future__ import annotations
import os
import math
from collections import OrderedDict
from typing import List, Tuple

try:
    from . import route_planner_algorithms as route_algos
except Exception:
    import route_planner_algorithms as route_algos  # type: ignore
try:
    from ..dynamics.lah_op_envlp import DEFAULT_ENVELOPE
except Exception:
    try:
        from dynamics.lah_op_envlp import DEFAULT_ENVELOPE  # type: ignore
    except Exception:
        DEFAULT_ENVELOPE = None  # type: ignore
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

WP_INTERVAL_M = 3000.0
HOVER_HOLD_SEC = 10
HOVER_LAST_SEC = 30
ALTITUDE_LAYERS_M = (610.0, 620.0, 630.0)
CAPSTONE_AREA_AGL_M = 200.0
CAPSTONE_BATTLE_HOLD_SEC = 3600
LAH_DEFAULT_CRUISE_SPEED_MPS = 30.0
LAH_UAV_ETA_PAIR_MAP = {1: 4, 2: 5, 3: 6}
LAH_UAV_FOLLOW_TARGET_GAP_M = 1000.0
LAH_UAV_FOLLOW_MIN_GAP_M = 500.0
LAH_UAV_FOLLOW_MAX_GAP_M = 2000.0
LAH_UAV_FOLLOW_MIN_SPEED_MPS = 8.0
LAH_UAV_FOLLOW_MIN_CRUISE_SCALE = 0.6
LAH_UAV_FOLLOW_MAX_CRUISE_SCALE = 2.5
LAH_UAV_FOLLOW_SEARCH_STEPS = 25
LAH_UAV_FOLLOW_TIME_PENALTY = 0.5

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


def _capstone_area_center(area_list: list[dict]) -> tuple[float, float] | None:
    if not isinstance(area_list, list) or not area_list:
        return None
    area0 = area_list[0] if isinstance(area_list[0], dict) else {}
    coords_raw = area0.get("coordinateList") if isinstance(area0, dict) else None
    if not isinstance(coords_raw, list) or len(coords_raw) < 3:
        return None
    pts: list[tuple[float, float]] = []
    for row in coords_raw:
        if not isinstance(row, dict):
            continue
        try:
            pts.append((float(row["latitude"]), float(row["longitude"])))
        except Exception:
            continue
    if len(pts) < 3:
        return None
    if pts[0] != pts[-1]:
        pts = pts + [pts[0]]
    k = 111_132.92
    lat0 = sum(p[0] for p in pts[:-1]) / max(1, len(pts) - 1)
    lon0 = sum(p[1] for p in pts[:-1]) / max(1, len(pts) - 1)
    cos0 = max(1e-6, math.cos(math.radians(lat0)))
    xy = [((lon - lon0) * k * cos0, (lat - lat0) * k) for lat, lon in pts]
    area2 = 0.0
    cx = 0.0
    cy = 0.0
    for (x0, y0), (x1, y1) in zip(xy, xy[1:]):
        cross = x0 * y1 - x1 * y0
        area2 += cross
        cx += (x0 + x1) * cross
        cy += (y0 + y1) * cross
    if abs(area2) < 1e-6:
        lats = [p[0] for p in pts[:-1]]
        lons = [p[1] for p in pts[:-1]]
        return (sum(lats) / len(lats), sum(lons) / len(lons))
    cx /= (3.0 * area2)
    cy /= (3.0 * area2)
    return (lat0 + (cy / k), lon0 + (cx / (k * cos0)))
         
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
        if self._local_next <= 0:
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

def _interior_sample_distances_with_merged_tail(
    total_len_m: float,
    step_m: float,
) -> List[float]:
    total_len_m = max(float(total_len_m), 0.0)
    spacing = max(float(step_m), 1.0)
    if total_len_m <= spacing + 1e-6:
        return []
    n_full = int(total_len_m // spacing)
    targets = [float(spacing * idx) for idx in range(1, n_full + 1)]
    remainder_m = total_len_m - float(n_full) * spacing
    if remainder_m > 1e-6 and targets:
        targets.pop()
    return targets

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
    targets = _interior_sample_distances_with_merged_tail(dist, step_m)
    return [
        (
            lat1 + (lat2 - lat1) * (target_m / dist),
            lon1 + (lon2 - lon1) * (target_m / dist),
        )
        for target_m in targets
    ] + [p1]


def _dist_ll_m(p0: Tuple[float, float], p1: Tuple[float, float]) -> float:
    lat1, lon1 = p0
    lat2, lon2 = p1
    k = 111_132.92
    cos_mid = math.cos(math.radians((lat1 + lat2) / 2.0))
    dx = (lon2 - lon1) * k * cos_mid
    dy = (lat2 - lat1) * k
    return math.hypot(dx, dy)


def _resample_route_samples(
    samples: List[dict],
    *,
    step_m: float = WP_INTERVAL_M,
) -> List[dict]:
    points: List[Tuple[float, float]] = []
    for sample in samples or []:
        if not isinstance(sample, dict):
            continue
        try:
            points.append((float(sample["lat"]), float(sample["lon"])))
        except Exception:
            continue
    if not points:
        return []
    if len(points) == 1:
        return [{"lat": points[0][0], "lon": points[0][1], "cum_m": 0.0}]

    spacing = max(float(step_m), 1.0)
    out: List[dict] = [{"lat": points[0][0], "lon": points[0][1], "cum_m": 0.0}]
    total_len_m = 0.0
    for start, end in zip(points, points[1:]):
        total_len_m += _dist_ll_m(start, end)
    target_distances = _interior_sample_distances_with_merged_tail(total_len_m, spacing)
    target_index = 0
    traversed_m = 0.0

    for start, end in zip(points, points[1:]):
        seg_len_m = _dist_ll_m(start, end)
        if seg_len_m < 1e-6:
            continue
        seg_start_m = traversed_m
        seg_end_m = traversed_m + seg_len_m
        while target_index < len(target_distances) and target_distances[target_index] < seg_end_m - 1e-6:
            target_m = float(target_distances[target_index])
            ratio = (target_m - seg_start_m) / seg_len_m
            lat = float(start[0]) + (float(end[0]) - float(start[0])) * ratio
            lon = float(start[1]) + (float(end[1]) - float(start[1])) * ratio
            last = out[-1]
            if _dist_ll_m((float(last["lat"]), float(last["lon"])), (lat, lon)) >= 1.0:
                out.append({"lat": lat, "lon": lon, "cum_m": float(target_m)})
            target_index += 1
        traversed_m = seg_end_m
        last = out[-1]
        if _dist_ll_m((float(last["lat"]), float(last["lon"])), end) >= 1.0:
            out.append({"lat": float(end[0]), "lon": float(end[1]), "cum_m": float(traversed_m)})
        else:
            out[-1]["lat"] = float(end[0])
            out[-1]["lon"] = float(end[1])
            out[-1]["cum_m"] = float(traversed_m)
    return out


def _coord_from_wp(wp: dict) -> dict | None:
    coord = wp.get("coordinate") if isinstance(wp, dict) else None
    if not isinstance(coord, dict):
        return None
    try:
        return {
            "latitude": float(coord["latitude"]),
            "longitude": float(coord["longitude"]),
            "altitude": float(coord.get("altitude", 0.0) or 0.0),
        }
    except Exception:
        return None


def _coord_between(c0: dict, c1: dict, ratio: float) -> dict:
    alpha = min(max(float(ratio), 0.0), 1.0)
    return {
        "latitude": float(c0["latitude"]) + (float(c1["latitude"]) - float(c0["latitude"])) * alpha,
        "longitude": float(c0["longitude"]) + (float(c1["longitude"]) - float(c0["longitude"])) * alpha,
        "altitude": float(c0.get("altitude", 0.0) or 0.0)
        + (float(c1.get("altitude", 0.0) or 0.0) - float(c0.get("altitude", 0.0) or 0.0)) * alpha,
    }


def _coord_vector_m(c0: dict, c1: dict) -> tuple[float, float]:
    lat1 = float(c0["latitude"])
    lon1 = float(c0["longitude"])
    lat2 = float(c1["latitude"])
    lon2 = float(c1["longitude"])
    k = 111_132.92
    cos_mid = math.cos(math.radians((lat1 + lat2) / 2.0))
    dx = (lon2 - lon1) * k * cos_mid
    dy = (lat2 - lat1) * k
    return dx, dy


def _coord_dist_m(c0: dict, c1: dict) -> float:
    dx, dy = _coord_vector_m(c0, c1)
    return math.hypot(dx, dy)


def _segment_unit_xy(c0: dict, c1: dict) -> tuple[float, float]:
    dx, dy = _coord_vector_m(c0, c1)
    norm = math.hypot(dx, dy)
    if norm < 1e-6:
        return (1.0, 0.0)
    return (dx / norm, dy / norm)


def _uav_timeline_from_packet(packet: dict) -> list[dict]:
    waypoints = _packet_waypoints(packet)
    timeline: list[dict] = []
    for wp in waypoints:
        coord = _coord_from_wp(wp)
        if coord is None:
            continue
        try:
            eta_s = _eta_to_seconds(wp.get("eta", 0), assume_ms=_packet_eta_is_milliseconds(packet))
        except Exception:
            eta_s = 0.0
        if timeline and eta_s < float(timeline[-1]["eta_s"]):
            eta_s = float(timeline[-1]["eta_s"])
        timeline.append({"eta_s": float(eta_s), "coord": coord})
    return timeline


def _packet_waypoints(packet: dict) -> list[dict]:
    for key in ("uavWaypointList", "waypointList", "lahWaypointList"):
        candidate = packet.get(key) if isinstance(packet, dict) else None
        if isinstance(candidate, list):
            return candidate
    return []


def _packet_eta_is_milliseconds(packet: dict) -> bool:
    if not isinstance(packet, dict):
        return False
    if isinstance(packet.get("lahWaypointList"), list) and not (
        isinstance(packet.get("waypointList"), list) or isinstance(packet.get("uavWaypointList"), list)
    ):
        return True
    return False


def _eta_to_seconds(raw_eta: object, *, assume_ms: bool) -> float:
    try:
        eta = float(raw_eta or 0.0)
    except Exception:
        eta = 0.0
    if assume_ms:
        return eta / 1000.0
    return eta


def _packet_terminal_coords(packet: dict) -> tuple[dict | None, dict | None]:
    waypoints = _packet_waypoints(packet)
    if not waypoints:
        return None, None
    start = _coord_from_wp(waypoints[0])
    end = _coord_from_wp(waypoints[-1])
    return start, end


def _pair_uav_packet_for_lah(lah_packet: dict, uav_packets: list[dict]) -> dict | None:
    if not uav_packets:
        return None
    lah_start, lah_end = _packet_terminal_coords(lah_packet)
    if lah_start is None:
        return uav_packets[0]
    best_packet = None
    best_score = None
    for pkt in uav_packets:
        uav_start, uav_end = _packet_terminal_coords(pkt)
        if uav_start is None:
            continue
        score = _coord_dist_m(lah_start, uav_start)
        if lah_end is not None and uav_end is not None:
            score += _coord_dist_m(lah_end, uav_end)
        if best_score is None or score < best_score:
            best_score = score
            best_packet = pkt
    return best_packet or uav_packets[0]


def _pair_lah_packet_by_geometry(target_packet: dict, candidate_packets: list[dict]) -> dict | None:
    if not candidate_packets:
        return None
    return _pair_uav_packet_for_lah(target_packet, candidate_packets)


def _leader_speed_profile_from_packet(packet: dict) -> list[tuple[float, float]]:
    profile: list[tuple[float, float]] = []
    wplist = packet.get("lahWaypointList") or []
    for idx, wp in enumerate(wplist):
        try:
            speed = float(wp.get("speed", 0.0) or 0.0)
        except Exception:
            continue
        if speed <= 0.0:
            continue
        try:
            progress = float(wp.get("ecf", 0.0) or 0.0)
        except Exception:
            progress = 0.0
        if idx == len(wplist) - 1:
            progress = 1.0
        progress = min(max(progress, 0.0), 1.0)
        profile.append((progress, speed))
    if not profile:
        return []
    profile.sort(key=lambda item: (item[0], item[1]))
    return profile


def _speed_from_progress(profile: list[tuple[float, float]], progress: float) -> float | None:
    if not profile:
        return None
    x = min(max(float(progress), 0.0), 1.0)
    if x <= profile[0][0]:
        return float(profile[0][1])
    for left, right in zip(profile, profile[1:]):
        p0, s0 = left
        p1, s1 = right
        if x <= p1:
            if p1 <= p0 + 1e-9:
                return float(s1)
            ratio = (x - p0) / max(p1 - p0, 1e-9)
            return float(s0 + (s1 - s0) * ratio)
    return float(profile[-1][1])


def _recompute_lah_eta_inplace(packet: dict) -> None:
    wplist = packet.get("lahWaypointList") or []
    if not isinstance(wplist, list) or not wplist:
        return
    prev_coord = _coord_from_wp(wplist[0])
    cum_eta_ms = 0.0
    wplist[0]["eta"] = 0
    for idx in range(1, len(wplist)):
        curr_coord = _coord_from_wp(wplist[idx])
        if prev_coord is None or curr_coord is None:
            wplist[idx]["eta"] = int(round(cum_eta_ms))
            prev_coord = curr_coord
            continue
        leg_len_m = _coord_dist_m(prev_coord, curr_coord)
        try:
            speed_mps = float(wplist[idx].get("speed", 0.0) or 0.0)
        except Exception:
            speed_mps = 0.0
        if speed_mps <= 0.0:
            speed_mps = 1.0
        cum_eta_ms += (leg_len_m / max(speed_mps, 1e-6)) * 1000.0
        wplist[idx]["eta"] = int(round(cum_eta_ms))
        prev_coord = curr_coord


def _sync_lah_packet_speeds_to_leader(
    lah_packets: List[dict],
    *,
    preferred_leader_ids: tuple[int, ...] = (1, 2, 3),
) -> None:
    packets_by_aircraft: dict[int, list[dict]] = {}
    for pkt in lah_packets or []:
        try:
            aid = int(pkt.get("aircraftID", 0))
        except Exception:
            continue
        if aid in (1, 2, 3):
            packets_by_aircraft.setdefault(aid, []).append(pkt)

    leader_id = None
    for candidate in preferred_leader_ids:
        if packets_by_aircraft.get(candidate):
            leader_id = int(candidate)
            break
    if leader_id is None:
        return

    leader_packets = packets_by_aircraft.get(leader_id) or []
    for aid, packets in packets_by_aircraft.items():
        if aid == leader_id:
            continue
        for pkt in packets:
            leader_pkt = _pair_lah_packet_by_geometry(pkt, leader_packets)
            if not isinstance(leader_pkt, dict):
                continue
            profile = _leader_speed_profile_from_packet(leader_pkt)
            wplist = pkt.get("lahWaypointList") or []
            if not profile or not isinstance(wplist, list) or not wplist:
                continue
            for idx, wp in enumerate(wplist):
                try:
                    progress = float(wp.get("ecf", 0.0) or 0.0)
                except Exception:
                    progress = 0.0
                if idx == len(wplist) - 1:
                    progress = 1.0
                ref_speed = _speed_from_progress(profile, progress)
                if ref_speed is None:
                    continue
                wp["speed"] = round(float(ref_speed), 2)
            _recompute_lah_eta_inplace(pkt)


def _uav_state_at_eta_s(timeline: list[dict], eta_s: float) -> tuple[dict, tuple[float, float]] | None:
    if not timeline:
        return None
    if len(timeline) == 1:
        return dict(timeline[0]["coord"]), (1.0, 0.0)
    eta_s = float(eta_s)
    if eta_s <= float(timeline[0]["eta_s"]):
        start = timeline[0]
        end = timeline[1]
        return dict(start["coord"]), _segment_unit_xy(start["coord"], end["coord"])
    for idx in range(1, len(timeline)):
        prev = timeline[idx - 1]
        curr = timeline[idx]
        t0 = float(prev["eta_s"])
        t1 = float(curr["eta_s"])
        if eta_s <= t1:
            if t1 <= t0 + 1e-6:
                return dict(curr["coord"]), _segment_unit_xy(prev["coord"], curr["coord"])
            ratio = (eta_s - t0) / max(t1 - t0, 1e-6)
            coord = _coord_between(prev["coord"], curr["coord"], ratio)
            return coord, _segment_unit_xy(prev["coord"], curr["coord"])
    prev = timeline[-2]
    curr = timeline[-1]
    return dict(curr["coord"]), _segment_unit_xy(prev["coord"], curr["coord"])


def _follow_speed_bounds(base_cruise_speed: float) -> tuple[float, float]:
    cruise = max(1.0, float(base_cruise_speed))
    env_max_speed = 75.0
    try:
        if DEFAULT_ENVELOPE is not None:
            env_max_speed = float(getattr(DEFAULT_ENVELOPE, "max_speed_kmh", 265.0)) / 3.6
    except Exception:
        env_max_speed = 75.0
    min_speed = max(LAH_UAV_FOLLOW_MIN_SPEED_MPS, cruise * LAH_UAV_FOLLOW_MIN_CRUISE_SCALE)
    max_speed = min(env_max_speed, max(cruise * LAH_UAV_FOLLOW_MAX_CRUISE_SCALE, cruise + 10.0))
    if max_speed < min_speed:
        max_speed = min_speed
    return min_speed, max_speed


def _select_follow_arrival_eta_s(
    *,
    prev_eta_s: float,
    base_eta_s: float,
    leg_len_m: float,
    lah_coord: dict,
    uav_timeline: list[dict],
    min_speed_mps: float,
    max_speed_mps: float,
) -> float:
    if leg_len_m < 1e-6:
        return max(prev_eta_s, base_eta_s)
    earliest_eta_s = prev_eta_s + leg_len_m / max(max_speed_mps, 1e-6)
    latest_eta_s = prev_eta_s + leg_len_m / max(min_speed_mps, 1e-6)
    if latest_eta_s < earliest_eta_s:
        latest_eta_s = earliest_eta_s
    candidates = {
        float(earliest_eta_s),
        float(latest_eta_s),
        float(min(max(base_eta_s, earliest_eta_s), latest_eta_s)),
    }
    search_steps = max(3, int(LAH_UAV_FOLLOW_SEARCH_STEPS))
    if latest_eta_s > earliest_eta_s + 1e-6:
        span = latest_eta_s - earliest_eta_s
        for idx in range(search_steps):
            ratio = float(idx) / float(search_steps - 1)
            candidates.add(float(earliest_eta_s + span * ratio))

    best_eta_s = float(min(max(base_eta_s, earliest_eta_s), latest_eta_s))
    best_score = None
    for eta_s in sorted(candidates):
        state = _uav_state_at_eta_s(uav_timeline, eta_s)
        if state is None:
            continue
        uav_coord, heading_xy = state
        gap_dx, gap_dy = _coord_vector_m(lah_coord, uav_coord)
        along_gap_m = gap_dx * heading_xy[0] + gap_dy * heading_xy[1]
        slant_gap_m = math.hypot(gap_dx, gap_dy)
        shortage_m = max(0.0, LAH_UAV_FOLLOW_MIN_GAP_M - along_gap_m)
        overshoot_m = max(0.0, along_gap_m - LAH_UAV_FOLLOW_MAX_GAP_M)
        score = (
            shortage_m * 1000.0
            + overshoot_m * 10.0
            + abs(along_gap_m - LAH_UAV_FOLLOW_TARGET_GAP_M)
            + max(0.0, slant_gap_m - LAH_UAV_FOLLOW_MAX_GAP_M) * 2.0
            + abs(eta_s - base_eta_s) * LAH_UAV_FOLLOW_TIME_PENALTY
        )
        if best_score is None or score < best_score:
            best_score = score
            best_eta_s = float(eta_s)
    return best_eta_s


def apply_uav_eta_follow_speed_plan(
    lah_packets: List[dict],
    uav_packets: List[dict],
) -> List[dict]:
    uav_by_aircraft: dict[int, list[dict]] = {}
    for pkt in uav_packets or []:
        try:
            aid = int(pkt.get("aircraftID", 0))
        except Exception:
            continue
        if aid > 0:
            uav_by_aircraft.setdefault(aid, []).append(pkt)

    for pkt in lah_packets or []:
        try:
            lah_aircraft_id = int(pkt.get("aircraftID", 0))
        except Exception:
            continue
        paired_uav_id = int(LAH_UAV_ETA_PAIR_MAP.get(lah_aircraft_id, lah_aircraft_id + 3))
        uav_pkt = _pair_uav_packet_for_lah(pkt, uav_by_aircraft.get(paired_uav_id) or [])
        if not isinstance(uav_pkt, dict):
            continue
        uav_timeline = _uav_timeline_from_packet(uav_pkt)
        wplist = pkt.get("lahWaypointList") or []
        if len(uav_timeline) < 2 or not isinstance(wplist, list) or len(wplist) < 2:
            continue

        speed_hints: list[float] = []
        for wp in wplist:
            try:
                speed = float(wp.get("speed", 0.0) or 0.0)
            except Exception:
                speed = 0.0
            if speed > 0.0:
                speed_hints.append(speed)
        base_cruise_speed = sum(speed_hints) / len(speed_hints) if speed_hints else LAH_DEFAULT_CRUISE_SPEED_MPS
        min_speed_mps, max_speed_mps = _follow_speed_bounds(base_cruise_speed)

        first_coord = _coord_from_wp(wplist[0])
        if first_coord is None:
            continue
        prev_coord = first_coord
        try:
            prev_eta_s = max(0.0, _eta_to_seconds(wplist[0].get("eta", 0), assume_ms=True))
        except Exception:
            prev_eta_s = 0.0
        leg_lengths = [0.0]
        first_leg_speed = None

        for idx in range(1, len(wplist)):
            curr_wp = wplist[idx]
            curr_coord = _coord_from_wp(curr_wp)
            if curr_coord is None:
                leg_lengths.append(0.0)
                continue
            leg_len_m = _coord_dist_m(prev_coord, curr_coord)
            leg_lengths.append(leg_len_m)
            try:
                original_speed = float(curr_wp.get("speed", base_cruise_speed) or base_cruise_speed)
            except Exception:
                original_speed = float(base_cruise_speed)
            if original_speed <= 0.0:
                original_speed = float(base_cruise_speed)
            try:
                original_eta_s = _eta_to_seconds(curr_wp.get("eta", 0), assume_ms=True)
            except Exception:
                original_eta_s = prev_eta_s
            base_leg_time_s = leg_len_m / max(original_speed, 1e-6) if leg_len_m > 1e-6 else 0.0
            base_eta_s = max(original_eta_s, prev_eta_s + base_leg_time_s)
            desired_eta_s = _select_follow_arrival_eta_s(
                prev_eta_s=prev_eta_s,
                base_eta_s=base_eta_s,
                leg_len_m=leg_len_m,
                lah_coord=curr_coord,
                uav_timeline=uav_timeline,
                min_speed_mps=min_speed_mps,
                max_speed_mps=max_speed_mps,
            )
            min_leg_time_s = leg_len_m / max(max_speed_mps, 1e-6) if leg_len_m > 1e-6 else 0.0
            leg_time_s = max(desired_eta_s - prev_eta_s, min_leg_time_s)
            speed_mps = original_speed
            if leg_time_s > 1e-6 and leg_len_m > 1e-6:
                speed_mps = leg_len_m / leg_time_s
            speed_mps = min(max(speed_mps, min_speed_mps), max_speed_mps)
            if leg_len_m > 1e-6:
                desired_eta_s = prev_eta_s + leg_len_m / max(speed_mps, 1e-6)
            curr_wp["speed"] = round(float(speed_mps), 2)
            curr_wp["eta"] = int(round(float(desired_eta_s) * 1000.0))
            if first_leg_speed is None:
                first_leg_speed = float(curr_wp["speed"])
            prev_eta_s = float(desired_eta_s)
            prev_coord = curr_coord

        if first_leg_speed is not None:
            wplist[0]["speed"] = round(float(first_leg_speed), 2)
        total_len_m = sum(float(v) for v in leg_lengths[1:])
        cum_len_m = 0.0
        for idx, wp in enumerate(wplist):
            if idx > 0:
                cum_len_m += float(leg_lengths[idx])
            if idx == len(wplist) - 1:
                wp["ecf"] = 1.0
            elif total_len_m > 1e-6:
                wp["ecf"] = round(cum_len_m / total_len_m, 2)
            else:
                wp["ecf"] = 0.0
    _sync_lah_packet_speeds_to_leader(lah_packets, preferred_leader_ids=(1, 2, 3))
    return lah_packets

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
    cruise_speed: float = LAH_DEFAULT_CRUISE_SPEED_MPS,
    wp_interval_m: float = WP_INTERVAL_M,
    manned_plan_mode: str = "normal",
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
        manned_plan_mode = manned_plan_mode,
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
    cruise_speed: float = LAH_DEFAULT_CRUISE_SPEED_MPS,
    wp_interval_m: float = WP_INTERVAL_M,
    manned_plan_mode: str = "normal",
    wp_alloc: _WPAllocator | None = None,
) -> List[dict]:

    wp_alloc = wp_alloc or _WPAllocator()
    now_ms   = now_ms_since_2000()
    packets: List[dict] = []
    capstone_mode = str(manned_plan_mode or "normal").strip().lower() in {"capstone", "capstone_mode"}
    area_count_by_aircraft: dict[int, int] = {}
    if capstone_mode:
        for miss in missions:
            try:
                aid0 = int(miss.get("aircraftID", 0))
            except Exception:
                continue
            if aid0 not in (1, 2, 3):
                continue
            info0 = miss.get("individualMissionInfo", {}) or {}
            area_list0 = info0.get("areaList") if isinstance(info0.get("areaList"), list) else []
            if area_list0:
                area_count_by_aircraft[aid0] = int(area_count_by_aircraft.get(aid0, 0)) + 1
    area_seq_by_aircraft: dict[int, int] = {}
    capstone_battle_center_by_aircraft: dict[int, tuple[float, float]] = {}

    for miss in missions:
        aid = miss["aircraftID"]
        if aid not in (1, 2, 3):
            continue  

        path_id = miss.get("pathID")
        if path_id is None:
            raise ValueError(f"[0304] aircraft {aid}: mission missing pathID")

        info   = miss.get("individualMissionInfo", {})
        area_list = info.get("areaList") if isinstance(info.get("areaList"), list) else []
        coords = [
            (c["latitude"], c["longitude"])
            for c in info.get("coordinateList", [])
            if isinstance(c, dict) and "latitude" in c and "longitude" in c
        ]
        if not coords:
            line_list = info.get("lineList") if isinstance(info.get("lineList"), list) else []
            if line_list and isinstance(line_list[0], dict):
                coords = [
                    (c["latitude"], c["longitude"])
                    for c in (line_list[0].get("coordinateList") or [])
                    if isinstance(c, dict) and "latitude" in c and "longitude" in c
                ]

        is_area_capstone = (
            capstone_mode
            and bool(area_list)
            and int(area_count_by_aircraft.get(aid, 0)) >= 3
        )
        if is_area_capstone:
            area_seq = int(area_seq_by_aircraft.get(aid, 0)) + 1
            area_seq_by_aircraft[aid] = area_seq
            center = _capstone_area_center(area_list)
            if center is not None and area_seq == 2:
                capstone_battle_center_by_aircraft[aid] = center
            if area_seq >= 3:
                center = capstone_battle_center_by_aircraft.get(aid, center)
            if center is not None:
                coords = [center]

        if not coords:
            continue

        offset_north = 0.0
        if not is_area_capstone:
            if aid == 2:
                offset_north = 100.0      # +100 m north
            elif aid == 3:
                offset_north = -100.0     # -100 m north
        if offset_north:
            coords = [_offset_coord(lat, lon, north_m=offset_north)
                      for lat, lon in coords]

        aircraft_alt_offset = CAPSTONE_AREA_AGL_M if is_area_capstone else _aircraft_alt_offset_m(aid)
        mission_ground_ref = _median_ground_m(coords)

        def _mission_alt(lat: float, lon: float) -> int:
            if mission_ground_ref is None:
                return _lah_alt_agl(lat, lon, aircraft_alt_offset)
            return int(round(float(mission_ground_ref) + float(aircraft_alt_offset)))

        wplist: List[OrderedDict] = []

        if len(coords) >= 2:
            try:
                samples = route_algos.plan_route_linear(
                    coords,
                    cruise_speed=cruise_speed,
                )
            except Exception:
                samples = []
            if samples:
                route_samples = _resample_route_samples(samples, step_m=wp_interval_m)
                total_len_m = max(float((route_samples[-1] or {}).get("cum_m", 0.0) or 0.0), 1.0)
                for idx, sample in enumerate(route_samples):
                    cum_len_m = float(sample.get("cum_m", 0.0) or 0.0)
                    eta_ms = int(round(cum_len_m / max(1e-6, float(cruise_speed)) * 1000.0))
                    ecf = 1.0 if idx == len(route_samples) - 1 else round(cum_len_m / total_len_m, 2)
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

            leg_lengths = [0.0]
            for prev_pt, curr_pt in zip(path, path[1:]):
                leg_lengths.append(_dist_ll_m(prev_pt, curr_pt))
            total_len = max(sum(float(v) for v in leg_lengths[1:]), 1.0)
            cum_len   = 0.0
            for idx, (lat, lon) in enumerate(path):
                if idx:
                    cum_len += float(leg_lengths[idx])
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
            hover_time = HOVER_LAST_SEC
            if is_area_capstone:
                area_seq = int(area_seq_by_aircraft.get(aid, 0))
                hover_time = CAPSTONE_BATTLE_HOLD_SEC if area_seq == 2 else HOVER_LAST_SEC
            last_wp["hovering"] = {"time": int(hover_time)}
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


