from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Tuple

try:
    import pulp
except Exception:  # pragma: no cover - runtime fallback
    pulp = None  # type: ignore

from ..models import SplitPiece, SplitRunResult
from .schedule_models import Coord, ScheduleCandidate, ScheduleSlot


_R = 6_378_137.0
_AREA_TYPES = {2, 3, 6}
_LINE_TYPES = {1, 4, 5, 7}
_MOVE_SPEED_CANDS_MPS = [30.0, 35.0, 40.0, 45.0, 50.0, 55.0, 60.0]


@dataclass
class _SolveResult:
    solver: str
    status: str
    objective: Optional[float]
    gap: Optional[float]
    solve_ms: float
    selected: Dict[Tuple[int, int], Tuple[int, float]]  # (slot_idx, uav_id) -> (candidate_idx, vel_kmh)
    move_speed_mps: Dict[Tuple[int, int], float]  # (slot_idx, uav_id) -> move speed m/s


def _to_float(v: Any, default: float = 0.0) -> float:
    try:
        return float(v)
    except Exception:
        return default


def _to_int(v: Any, default: int = 0) -> int:
    try:
        return int(v)
    except Exception:
        return default


def _norm_coord(c: Any) -> Optional[Coord]:
    if not isinstance(c, dict):
        return None
    if "latitude" not in c or "longitude" not in c:
        return None
    return {
        "latitude": _to_float(c.get("latitude")),
        "longitude": _to_float(c.get("longitude")),
        "altitude": _to_float(c.get("altitude"), 0.0),
    }


def _llh_to_xy(lat: float, lon: float, lat0: float, lon0: float) -> Tuple[float, float]:
    lat0_r = math.radians(lat0)
    x = math.radians(lon - lon0) * _R * math.cos(lat0_r)
    y = math.radians(lat - lat0) * _R
    return x, y


def _distance_m(c0: Optional[Coord], c1: Optional[Coord]) -> float:
    if c0 is None or c1 is None:
        return 0.0
    lat0 = _to_float(c0.get("latitude"))
    lon0 = _to_float(c0.get("longitude"))
    x, y = _llh_to_xy(_to_float(c1.get("latitude")), _to_float(c1.get("longitude")), lat0, lon0)
    return float(math.hypot(x, y))


def _bearing_deg(c0: Optional[Coord], c1: Optional[Coord]) -> float:
    if not isinstance(c0, dict) or not isinstance(c1, dict):
        return 0.0
    lat0 = math.radians(_to_float(c0.get("latitude"), 0.0))
    lon0 = math.radians(_to_float(c0.get("longitude"), 0.0))
    lat1 = math.radians(_to_float(c1.get("latitude"), 0.0))
    lon1 = math.radians(_to_float(c1.get("longitude"), 0.0))
    dlon = lon1 - lon0
    x = math.sin(dlon) * math.cos(lat1)
    y = math.cos(lat0) * math.sin(lat1) - math.sin(lat0) * math.cos(lat1) * math.cos(dlon)
    return float((math.degrees(math.atan2(x, y)) + 360.0) % 360.0)


def _polyline_len_m(coords: Any) -> float:
    if not isinstance(coords, list) or len(coords) < 2:
        return 0.0
    c0 = _norm_coord(coords[0])
    if c0 is None:
        return 0.0
    lat0 = _to_float(c0.get("latitude"))
    lon0 = _to_float(c0.get("longitude"))
    pts: List[Tuple[float, float]] = []
    for c in coords:
        cc = _norm_coord(c)
        if cc is None:
            continue
        pts.append(_llh_to_xy(_to_float(cc["latitude"]), _to_float(cc["longitude"]), lat0, lon0))
    if len(pts) < 2:
        return 0.0
    total = 0.0
    for i in range(len(pts) - 1):
        total += math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
    return float(total)


def _centroid(coords: Iterable[Coord]) -> Optional[Coord]:
    vals = list(coords)
    if not vals:
        return None
    lat = sum(_to_float(c.get("latitude")) for c in vals) / float(len(vals))
    lon = sum(_to_float(c.get("longitude")) for c in vals) / float(len(vals))
    alt = sum(_to_float(c.get("altitude"), 0.0) for c in vals) / float(len(vals))
    return {"latitude": lat, "longitude": lon, "altitude": alt}


def _coord_copy(c: Optional[Coord]) -> Optional[Coord]:
    if not isinstance(c, dict):
        return None
    if "latitude" not in c or "longitude" not in c:
        return None
    return {
        "latitude": _to_float(c.get("latitude"), 0.0),
        "longitude": _to_float(c.get("longitude"), 0.0),
        "altitude": _to_float(c.get("altitude"), 0.0),
    }


def _piece_is_area(piece: SplitPiece) -> bool:
    return int(piece.mission_type) in _AREA_TYPES


def _piece_is_line(piece: SplitPiece) -> bool:
    return int(piece.mission_type) in _LINE_TYPES


def _piece_source_area_index(piece: SplitPiece) -> int:
    data = piece.data if isinstance(piece.data, dict) else {}
    if not _piece_is_area(piece):
        return 0
    v = _to_int(data.get("sourceAreaIndex"), 0)
    return v if v > 0 else 1


def _piece_stage(piece: SplitPiece) -> int:
    data = piece.data if isinstance(piece.data, dict) else {}
    v = _to_int(data.get("splitStage"), 0)
    if _piece_is_area(piece):
        return v if v > 0 else 1
    return 0


def _piece_base_index(piece: SplitPiece) -> int:
    data = piece.data if isinstance(piece.data, dict) else {}
    if _piece_is_area(piece):
        review = data.get("reviewArea")
        if isinstance(review, dict) and bool(review.get("subdivided", False)):
            fidx = _to_int(review.get("fromPieceIndex"), 0)
            if fidx > 0:
                return fidx
    return int(piece.piece_index)


def _piece_move_bearing(piece: SplitPiece) -> float:
    data = piece.data if isinstance(piece.data, dict) else {}
    if "phaseMoveBearing_deg" in data:
        return _to_float(data.get("phaseMoveBearing_deg"), 0.0)
    if "bearing_deg" in data:
        return _to_float(data.get("bearing_deg"), 0.0)
    return 0.0


def _bearing_unit_xy(bearing_deg: float) -> Tuple[float, float]:
    th = math.radians(float(bearing_deg))
    return math.sin(th), math.cos(th)


def _span_endpoints_by_bearing(coords: List[Coord], bearing_deg: float) -> Tuple[Optional[Coord], Optional[Coord]]:
    if len(coords) < 2:
        c = _centroid(coords)
        return c, c
    lat0 = _to_float(coords[0].get("latitude"))
    lon0 = _to_float(coords[0].get("longitude"))
    ux, uy = _bearing_unit_xy(bearing_deg)
    min_idx = 0
    max_idx = 0
    min_v = 1e30
    max_v = -1e30
    for i, c in enumerate(coords):
        x, y = _llh_to_xy(_to_float(c.get("latitude")), _to_float(c.get("longitude")), lat0, lon0)
        v = x * ux + y * uy
        if v < min_v:
            min_v = v
            min_idx = i
        if v > max_v:
            max_v = v
            max_idx = i
    return coords[min_idx], coords[max_idx]


def _piece_anchor_line(piece: SplitPiece) -> Tuple[Optional[Coord], Optional[Coord]]:
    data = piece.data if isinstance(piece.data, dict) else {}
    center = data.get("Centerline")
    if isinstance(center, list) and len(center) >= 2:
        s = _norm_coord(center[0])
        e = _norm_coord(center[-1])
        return s, e
    coords = data.get("coordinateList")
    if isinstance(coords, list) and len(coords) >= 2:
        s = _norm_coord(coords[0])
        e = _norm_coord(coords[-1])
        return s, e
    c = _centroid([_norm_coord(c) for c in (coords if isinstance(coords, list) else []) if _norm_coord(c) is not None])
    return c, c


def _piece_anchor_area(piece: SplitPiece) -> Tuple[Optional[Coord], Optional[Coord]]:
    data = piece.data if isinstance(piece.data, dict) else {}
    coords_any = data.get("coordinateList")
    coords = [_norm_coord(c) for c in coords_any] if isinstance(coords_any, list) else []
    coords = [c for c in coords if c is not None]
    if not coords:
        return None, None
    b = _piece_move_bearing(piece)
    return _span_endpoints_by_bearing(coords, b)


def _group_anchor(pieces: List[SplitPiece]) -> Tuple[Optional[Coord], Optional[Coord]]:
    if not pieces:
        return None, None
    if len(pieces) == 1:
        p = pieces[0]
        if _piece_is_line(p):
            return _piece_anchor_line(p)
        return _piece_anchor_area(p)

    first = pieces[0]
    bearing = _piece_move_bearing(first)
    all_coords: List[Coord] = []
    for p in pieces:
        data = p.data if isinstance(p.data, dict) else {}
        coords_any = data.get("coordinateList")
        if not isinstance(coords_any, list):
            continue
        for c in coords_any:
            cc = _norm_coord(c)
            if cc is not None:
                all_coords.append(cc)
    if not all_coords:
        if _piece_is_line(first):
            return _piece_anchor_line(first)
        return _piece_anchor_area(first)
    return _span_endpoints_by_bearing(all_coords, bearing)


def _piece_exp_vel(piece: SplitPiece) -> Dict[str, Any]:
    data = piece.data if isinstance(piece.data, dict) else {}
    exp = data.get("expVel")
    return exp if isinstance(exp, dict) else {}


def _vel_candidates_for_pieces(pieces: List[SplitPiece]) -> List[float]:
    per_piece: List[List[float]] = []
    for p in pieces:
        exp = _piece_exp_vel(p)
        vals = exp.get("velCandidatesKmh")
        cands: List[float] = []
        if isinstance(vals, list):
            cands = sorted({round(_to_float(v), 3) for v in vals if _to_float(v) > 0.0})
        if not cands:
            vmin = _to_float(exp.get("velMinKmh"), 0.0)
            vmax = _to_float(exp.get("velMaxKmh"), 0.0)
            if vmin > 0.0 and vmax > 0.0:
                cands = sorted({round(vmin, 3), round(vmax, 3)})
        if not cands:
            cands = [100.0]
        per_piece.append(cands)

    if not per_piece:
        return [100.0]

    common = set(per_piece[0])
    for c in per_piece[1:]:
        common &= set(c)
    if common:
        return sorted(common)

    lower = max(min(c) for c in per_piece)
    upper = min(max(c) for c in per_piece)
    if lower <= upper:
        if abs(upper - lower) <= 1e-6:
            return [round(lower, 3)]
        return sorted({round(lower, 3), round(upper, 3)})

    fallback = min(max(c) for c in per_piece)
    if fallback > 0.0:
        return [round(fallback, 3)]
    return [100.0]


def _distance_for_pieces(pieces: List[SplitPiece]) -> float:
    total = 0.0
    for p in pieces:
        exp = _piece_exp_vel(p)
        d = _to_float(exp.get("distanceRefM"), 0.0)
        if d <= 0.0:
            data = p.data if isinstance(p.data, dict) else {}
            if _piece_is_line(p):
                center = data.get("Centerline")
                d = _polyline_len_m(center)
                if d <= 0.0:
                    d = _polyline_len_m(data.get("coordinateList"))
            else:
                d = max(1.0, _to_float(exp.get("widthRefM"), 1.0))
        total += max(0.0, d)
    return float(total)


def _width_for_pieces(pieces: List[SplitPiece]) -> float:
    vals: List[float] = []
    for p in pieces:
        exp = _piece_exp_vel(p)
        w = _to_float(exp.get("widthRefM"), 0.0)
        if w > 0.0:
            vals.append(w)
    return float(max(vals)) if vals else 0.0


def _piece_refs_sorted(pieces: List[SplitPiece]) -> List[Tuple[int, int]]:
    refs = [(int(p.parent_order), int(p.piece_index)) for p in pieces]
    refs.sort()
    return refs


def _slot_key_tuple(slot: ScheduleSlot) -> Tuple[int, int, int, int]:
    return (int(slot.parent_order), int(slot.source_area_index), int(slot.split_stage), int(slot.order))


def _takeover_centroid(mrpk: Optional[Dict[str, Any]]) -> Optional[Coord]:
    if not isinstance(mrpk, dict):
        return None
    arr = mrpk.get("takeOverInfoList")
    if not isinstance(arr, list):
        return None
    coords: List[Coord] = []
    for item in arr:
        if not isinstance(item, dict):
            continue
        cc = _norm_coord(item.get("coordinate"))
        if cc is not None:
            coords.append(cc)
    return _centroid(coords)


def _takeover_map(mrpk: Optional[Dict[str, Any]]) -> Dict[int, Coord]:
    if not isinstance(mrpk, dict):
        return {}
    arr = mrpk.get("takeOverInfoList")
    if not isinstance(arr, list):
        return {}
    out: Dict[int, Coord] = {}
    for item in arr:
        if not isinstance(item, dict):
            continue
        try:
            aid = int(item.get("aircraftID"))
        except Exception:
            continue
        cc = _norm_coord(item.get("coordinate"))
        if cc is not None:
            out[int(aid)] = cc
    return out


def _takeover_for_uav(take_over_map: Optional[Dict[int, Coord]], uav_id: int, fallback: Optional[Coord]) -> Optional[Coord]:
    if isinstance(take_over_map, dict):
        cc = take_over_map.get(int(uav_id))
        if cc is not None:
            return cc
    return fallback


def _build_slots(split_result: SplitRunResult, uav_ids: List[int]) -> List[ScheduleSlot]:
    grouped: Dict[Tuple[int, int, int, int], List[SplitPiece]] = {}
    slot_members: Dict[Tuple[int, int, int], List[Tuple[int, int, int, int]]] = {}

    for piece in sorted(split_result.pieces, key=lambda p: (int(p.parent_order), int(p.piece_index))):
        parent = int(piece.parent_order)
        src_idx = _piece_source_area_index(piece)
        stage = _piece_stage(piece)
        base_idx = _piece_base_index(piece)
        gkey = (parent, src_idx, stage, base_idx)
        skey = (parent, src_idx, stage)
        grouped.setdefault(gkey, []).append(piece)
        slot_members.setdefault(skey, [])
        if gkey not in slot_members[skey]:
            slot_members[skey].append(gkey)

    slot_rows: List[ScheduleSlot] = []
    slot_order = 1
    uav_n = max(1, len(uav_ids))

    for skey in sorted(slot_members.keys()):
        parent, src_idx, stage = skey
        gkeys = sorted(slot_members[skey], key=lambda x: int(x[3]))
        candidates_all: List[ScheduleCandidate] = []
        for gk in gkeys:
            members = grouped.get(gk, [])
            if not members:
                continue
            mission_type = int(members[0].mission_type)
            refs = _piece_refs_sorted(members)
            distance_m = _distance_for_pieces(members)
            width_ref_m = _width_for_pieces(members)
            vel_cands = _vel_candidates_for_pieces(members)
            sp, ep = _group_anchor(members)
            if mission_type in _LINE_TYPES:
                label = f"M{parent}-{gk[3]}"
            else:
                label = f"M{parent}/A{src_idx}/S{stage}/{gk[3]}"
            slot_key_str = f"M{parent}-A{src_idx}-S{stage}"
            candidates_all.append(
                ScheduleCandidate(
                    slot_key=slot_key_str,
                    candidate_id=f"{slot_key_str}-C{gk[3]}",
                    parent_order=int(parent),
                    mission_type=mission_type,
                    source_area_index=int(src_idx),
                    split_stage=int(stage),
                    base_piece_index=int(gk[3]),
                    piece_refs=refs,
                    distance_m=float(distance_m),
                    width_ref_m=float(width_ref_m),
                    vel_candidates_kmh=vel_cands,
                    start_point=sp,
                    end_point=ep,
                    label=label,
                    is_dummy=False,
                )
            )

        if not candidates_all:
            continue

        chunks: List[List[ScheduleCandidate]] = []
        for i in range(0, len(candidates_all), uav_n):
            chunks.append(candidates_all[i : i + uav_n])

        for cidx, chunk in enumerate(chunks, start=1):
            chunk_name = f"M{parent}-A{src_idx}-S{stage}"
            if len(chunks) > 1:
                chunk_name += f"-B{cidx}"

            if len(chunk) < uav_n:
                for j in range(uav_n - len(chunk)):
                    dummy_idx = 9000 + j + 1
                    chunk.append(
                        ScheduleCandidate(
                            slot_key=chunk_name,
                            candidate_id=f"{chunk_name}-D{dummy_idx}",
                            parent_order=int(parent),
                            mission_type=0,
                            source_area_index=int(src_idx),
                            split_stage=int(stage),
                            base_piece_index=int(dummy_idx),
                            piece_refs=[],
                            distance_m=0.0,
                            width_ref_m=0.0,
                            vel_candidates_kmh=[100.0],
                            start_point=None,
                            end_point=None,
                            label="IDLE",
                            is_dummy=True,
                        )
                    )

            slot_rows.append(
                ScheduleSlot(
                    order=int(slot_order),
                    slot_key=chunk_name,
                    parent_order=int(parent),
                    source_area_index=int(src_idx),
                    split_stage=int(stage),
                    candidates=chunk,
                )
            )
            slot_order += 1

    slot_rows.sort(key=_slot_key_tuple)
    for i, s in enumerate(slot_rows, start=1):
        s.order = int(i)
    return slot_rows


def _allowed_uavs_by_candidate(
    split_result: SplitRunResult,
    slots: List[ScheduleSlot],
    uav_ids: List[int],
    *,
    respect_preassigned: bool,
) -> Dict[Tuple[int, int], set[int]]:
    if not respect_preassigned:
        return {}

    piece_map: Dict[Tuple[int, int], SplitPiece] = {
        (int(p.parent_order), int(p.piece_index)): p for p in split_result.pieces
    }
    valid_uavs = {int(u) for u in uav_ids}
    out: Dict[Tuple[int, int], set[int]] = {}
    for g, slot in enumerate(slots):
        for i, cand in enumerate(slot.candidates):
            allowed = set(valid_uavs)
            if cand.piece_refs:
                assigned = {
                    int(piece_map[pref].assigned_uav)
                    for pref in cand.piece_refs
                    if pref in piece_map and int(piece_map[pref].assigned_uav or 0) > 0
                }
                assigned &= valid_uavs
                if assigned:
                    allowed = set(assigned)
            out[(int(g), int(i))] = allowed
    return out


def _exec_time_sec(distance_m: float, vel_kmh: float) -> float:
    d = max(0.0, float(distance_m))
    v = max(0.0, float(vel_kmh))
    if d <= 0.0 or v <= 0.0:
        return 0.0
    return float(d / (v * 1000.0 / 3600.0))


def _move_time_sec(distance_m: float, speed_mps: float) -> float:
    d = max(0.0, float(distance_m))
    v = max(0.0, float(speed_mps))
    if d <= 0.0 or v <= 0.0:
        return 0.0
    return float(d / v)


def _solve_with_milp_legacy(
    slots: List[ScheduleSlot],
    uav_ids: List[int],
    take_over_map: Optional[Dict[int, Coord]],
    time_limit_s: float,
    mip_gap: float,
    allowed_uavs_by_candidate: Optional[Dict[Tuple[int, int], set[int]]] = None,
) -> _SolveResult:
    if pulp is None:
        return _SolveResult(
            solver="none",
            status="fallback_no_pulp",
            objective=None,
            gap=None,
            solve_ms=0.0,
            selected={},
            move_speed_mps={},
        )

    n_slots = len(slots)
    if n_slots <= 0:
        return _SolveResult(
            solver="cbc",
            status="empty",
            objective=0.0,
            gap=0.0,
            solve_ms=0.0,
            selected={},
            move_speed_mps={},
        )

    prob = pulp.LpProblem("MUMT_Scheduling", pulp.LpMinimize)

    # Precompute tables.
    vel_table: Dict[Tuple[int, int], List[float]] = {}
    exec_table: Dict[Tuple[int, int, int], float] = {}
    init_move: Dict[Tuple[int, int, int], float] = {}
    trans_move: Dict[Tuple[int, int, int], float] = {}
    speed_penalty: Dict[Tuple[int, int, int], float] = {}
    take_over_fallback = _centroid(list((take_over_map or {}).values()))

    for g, slot in enumerate(slots):
        for i, cand in enumerate(slot.candidates):
            vels = sorted({float(v) for v in cand.vel_candidates_kmh if float(v) > 0.0})
            if not vels:
                vels = [100.0]
            vel_table[(g, i)] = vels
            vmin = min(vels)
            vmax = max(vels)
            den = max(1e-6, vmax - vmin)
            for k, vel in enumerate(vels):
                exec_table[(g, i, k)] = _exec_time_sec(cand.distance_m, vel)
                speed_penalty[(g, i, k)] = (float(vel) - vmin) / den
            for u in uav_ids:
                take_over = _takeover_for_uav(take_over_map, u, take_over_fallback)
                init_move[(g, i, u)] = _distance_m(take_over, cand.start_point) / 40.0 if g == 0 else 0.0

    for g in range(n_slots - 1):
        slot_a = slots[g]
        slot_b = slots[g + 1]
        for i, ca in enumerate(slot_a.candidates):
            for j, cb in enumerate(slot_b.candidates):
                trans_move[(g, i, j)] = _distance_m(ca.end_point, cb.start_point) / 40.0

    # Variables.
    w: Dict[Tuple[int, int, int, int], Any] = {}
    x: Dict[Tuple[int, int, int], Any] = {}
    for g, slot in enumerate(slots):
        for i, _ in enumerate(slot.candidates):
            vels = vel_table[(g, i)]
            for u in uav_ids:
                allowed = allowed_uavs_by_candidate.get((g, i)) if isinstance(allowed_uavs_by_candidate, dict) else None
                up_bound = 1 if (allowed is None or int(u) in allowed) else 0
                x[(g, i, u)] = pulp.LpVariable(f"x_{g}_{i}_{u}", lowBound=0, upBound=up_bound, cat="Binary")
                ws = []
                for k, _ in enumerate(vels):
                    key = (g, i, u, k)
                    w[key] = pulp.LpVariable(f"w_{g}_{i}_{u}_{k}", lowBound=0, upBound=up_bound, cat="Binary")
                    ws.append(w[key])
                prob += x[(g, i, u)] == pulp.lpSum(ws)

    # Assignment constraints per slot.
    for g, slot in enumerate(slots):
        n_c = len(slot.candidates)
        for i in range(n_c):
            prob += pulp.lpSum(x[(g, i, u)] for u in uav_ids) == 1
        for u in uav_ids:
            prob += pulp.lpSum(x[(g, i, u)] for i in range(n_c)) == 1

    # Transition linearization q.
    q: Dict[Tuple[int, int, int, int], Any] = {}
    for g in range(n_slots - 1):
        n_i = len(slots[g].candidates)
        n_j = len(slots[g + 1].candidates)
        for i in range(n_i):
            for j in range(n_j):
                for u in uav_ids:
                    key = (g, i, j, u)
                    q[key] = pulp.LpVariable(f"q_{g}_{i}_{j}_{u}", lowBound=0, upBound=1, cat="Binary")
                    prob += q[key] <= x[(g, i, u)]
                    prob += q[key] <= x[(g + 1, j, u)]
                    prob += q[key] >= x[(g, i, u)] + x[(g + 1, j, u)] - 1

    # Time propagation.
    S: Dict[Tuple[int, int], Any] = {}
    E: Dict[Tuple[int, int], Any] = {}
    H: Dict[int, Any] = {}
    Hmax = pulp.LpVariable("Hmax", lowBound=0)
    Hmin = pulp.LpVariable("Hmin", lowBound=0)

    for g, slot in enumerate(slots):
        for u in uav_ids:
            S[(g, u)] = pulp.LpVariable(f"S_{g}_{u}", lowBound=0)
            E[(g, u)] = pulp.LpVariable(f"E_{g}_{u}", lowBound=0)
            exec_expr = []
            for i, _ in enumerate(slot.candidates):
                vels = vel_table[(g, i)]
                for k, _ in enumerate(vels):
                    exec_expr.append(exec_table[(g, i, k)] * w[(g, i, u, k)])
            prob += E[(g, u)] == S[(g, u)] + pulp.lpSum(exec_expr)

            if g == 0:
                init_expr = [init_move[(g, i, u)] * x[(g, i, u)] for i in range(len(slot.candidates))]
                prob += S[(g, u)] >= pulp.lpSum(init_expr)
            else:
                move_expr = []
                for i in range(len(slots[g - 1].candidates)):
                    for j in range(len(slot.candidates)):
                        move_expr.append(trans_move[(g - 1, i, j)] * q[(g - 1, i, j, u)])
                prob += S[(g, u)] >= E[(g - 1, u)] + pulp.lpSum(move_expr)

    last_g = n_slots - 1
    for u in uav_ids:
        H[u] = pulp.LpVariable(f"H_{u}", lowBound=0)
        prob += H[u] == E[(last_g, u)]
        prob += Hmax >= H[u]
        prob += Hmin <= H[u]

    # Objective: time balance first, then total time / speed / move.
    A, B, C, D = 1.0, 0.05, 0.02, 0.03
    obj_balance = Hmax - Hmin
    obj_total = pulp.lpSum(H[u] for u in uav_ids)
    obj_speed = pulp.lpSum(
        speed_penalty[(g, i, k)] * w[(g, i, u, k)]
        for g, slot in enumerate(slots)
        for i, _ in enumerate(slot.candidates)
        for u in uav_ids
        for k, _ in enumerate(vel_table[(g, i)])
    )
    obj_move = pulp.lpSum(
        trans_move[(g, i, j)] * q[(g, i, j, u)]
        for g in range(n_slots - 1)
        for i in range(len(slots[g].candidates))
        for j in range(len(slots[g + 1].candidates))
        for u in uav_ids
    ) + pulp.lpSum(
        init_move[(0, i, u)] * x[(0, i, u)]
        for i in range(len(slots[0].candidates))
        for u in uav_ids
    )
    prob += (A * obj_balance) + (B * obj_total) + (C * obj_speed) + (D * obj_move)

    t0 = time.perf_counter()
    try:
        solver = pulp.PULP_CBC_CMD(
            msg=False,
            timeLimit=max(0.1, float(time_limit_s)),
            gapRel=max(0.0, float(mip_gap)),
        )
        status_code = prob.solve(solver)
    except TypeError:
        solver = pulp.PULP_CBC_CMD(
            msg=False,
            timeLimit=max(0.1, float(time_limit_s)),
        )
        status_code = prob.solve(solver)
    solve_ms = (time.perf_counter() - t0) * 1000.0

    status_map = getattr(pulp, "LpStatus", {})
    status_text = str(status_map.get(status_code, "Unknown"))
    if int(status_code) != int(getattr(pulp, "LpStatusOptimal", 1)):
        return _SolveResult(
            solver="cbc",
            status=f"fallback_{status_text.lower()}",
            objective=None,
            gap=None,
            solve_ms=solve_ms,
            selected={},
            move_speed_mps={},
        )

    selected: Dict[Tuple[int, int], Tuple[int, float]] = {}
    for g, slot in enumerate(slots):
        for u in uav_ids:
            picked_i = None
            for i, _ in enumerate(slot.candidates):
                xv = 0.0
                for k, _ in enumerate(vel_table[(g, i)]):
                    xv += _to_float(pulp.value(w[(g, i, u, k)]), 0.0)
                if xv > 0.5:
                    picked_i = i
                    break
            if picked_i is None:
                continue
            picked_vel = min(vel_table[(g, picked_i)])
            for k, vel in enumerate(vel_table[(g, picked_i)]):
                wv = _to_float(pulp.value(w[(g, picked_i, u, k)]), 0.0)
                if wv > 0.5:
                    picked_vel = float(vel)
                    break
            selected[(g, u)] = (int(picked_i), float(picked_vel))

    objective = _to_float(pulp.value(prob.objective), 0.0)
    return _SolveResult(
        solver="cbc",
        status="optimal",
        objective=objective,
        gap=None,
        solve_ms=solve_ms,
        selected=selected,
        move_speed_mps={(int(g), int(u)): 40.0 for g in range(n_slots) for u in uav_ids},
    )


def _solve_with_milp_barrier(
    slots: List[ScheduleSlot],
    uav_ids: List[int],
    take_over_map: Optional[Dict[int, Coord]],
    time_limit_s: float,
    mip_gap: float,
    allowed_uavs_by_candidate: Optional[Dict[Tuple[int, int], set[int]]] = None,
) -> _SolveResult:
    if pulp is None:
        return _SolveResult(
            solver="none",
            status="fallback_no_pulp",
            objective=None,
            gap=None,
            solve_ms=0.0,
            selected={},
            move_speed_mps={},
        )

    n_slots = len(slots)
    if n_slots <= 0:
        return _SolveResult(
            solver="cbc",
            status="empty",
            objective=0.0,
            gap=0.0,
            solve_ms=0.0,
            selected={},
            move_speed_mps={},
        )

    prob = pulp.LpProblem("MUMT_Scheduling_Barrier", pulp.LpMinimize)

    # Precompute tables.
    vel_table: Dict[Tuple[int, int], List[float]] = {}
    exec_table: Dict[Tuple[int, int, int], float] = {}
    init_dist: Dict[Tuple[int, int, int], float] = {}
    trans_dist: Dict[Tuple[int, int, int], float] = {}
    init_move_t: Dict[Tuple[int, int, int, int], float] = {}
    trans_move_t: Dict[Tuple[int, int, int, int], float] = {}
    speed_penalty: Dict[Tuple[int, int, int], float] = {}
    move_penalty: Dict[int, float] = {}
    move_speeds = list(_MOVE_SPEED_CANDS_MPS)
    vmax_move = max(move_speeds) if move_speeds else 60.0
    take_over_fallback = _centroid(list((take_over_map or {}).values()))

    for g, slot in enumerate(slots):
        for i, cand in enumerate(slot.candidates):
            vels = sorted({float(v) for v in cand.vel_candidates_kmh if float(v) > 0.0})
            if not vels:
                vels = [100.0]
            vel_table[(g, i)] = vels
            vmin = min(vels)
            vmax = max(vels)
            den = max(1e-6, vmax - vmin)
            for k, vel in enumerate(vels):
                exec_table[(g, i, k)] = _exec_time_sec(cand.distance_m, vel)
                speed_penalty[(g, i, k)] = (float(vel) - vmin) / den
            for u in uav_ids:
                take_over = _takeover_for_uav(take_over_map, u, take_over_fallback)
                d0 = _distance_m(take_over, cand.start_point) if g == 0 else 0.0
                init_dist[(g, i, u)] = float(d0)
                for m_idx, mps in enumerate(move_speeds):
                    init_move_t[(g, i, u, m_idx)] = _move_time_sec(d0, mps)

    for g in range(n_slots - 1):
        slot_a = slots[g]
        slot_b = slots[g + 1]
        for i, ca in enumerate(slot_a.candidates):
            for j, cb in enumerate(slot_b.candidates):
                d = _distance_m(ca.end_point, cb.start_point)
                trans_dist[(g, i, j)] = float(d)
                for m_idx, mps in enumerate(move_speeds):
                    trans_move_t[(g, i, j, m_idx)] = _move_time_sec(d, mps)

    vmin_move = min(move_speeds) if move_speeds else 30.0
    for m_idx, mps in enumerate(move_speeds):
        move_penalty[m_idx] = max(0.0, (float(mps) - vmin_move) / max(1e-6, (vmax_move - vmin_move)))

    # Variables.
    w: Dict[Tuple[int, int, int, int], Any] = {}
    x: Dict[Tuple[int, int, int], Any] = {}
    for g, slot in enumerate(slots):
        for i, _ in enumerate(slot.candidates):
            vels = vel_table[(g, i)]
            for u in uav_ids:
                allowed = allowed_uavs_by_candidate.get((g, i)) if isinstance(allowed_uavs_by_candidate, dict) else None
                up_bound = 1 if (allowed is None or int(u) in allowed) else 0
                x[(g, i, u)] = pulp.LpVariable(f"x_{g}_{i}_{u}", lowBound=0, upBound=up_bound, cat="Binary")
                ws = []
                for k, _ in enumerate(vels):
                    key = (g, i, u, k)
                    w[key] = pulp.LpVariable(f"w_{g}_{i}_{u}_{k}", lowBound=0, upBound=up_bound, cat="Binary")
                    ws.append(w[key])
                prob += x[(g, i, u)] == pulp.lpSum(ws)

    # Assignment constraints per slot.
    for g, slot in enumerate(slots):
        n_c = len(slot.candidates)
        for i in range(n_c):
            prob += pulp.lpSum(x[(g, i, u)] for u in uav_ids) == 1
        for u in uav_ids:
            prob += pulp.lpSum(x[(g, i, u)] for i in range(n_c)) == 1

    # Transition linearization q.
    q: Dict[Tuple[int, int, int, int], Any] = {}
    for g in range(n_slots - 1):
        n_i = len(slots[g].candidates)
        n_j = len(slots[g + 1].candidates)
        for i in range(n_i):
            for j in range(n_j):
                for u in uav_ids:
                    key = (g, i, j, u)
                    q[key] = pulp.LpVariable(f"q_{g}_{i}_{j}_{u}", lowBound=0, upBound=1, cat="Binary")
                    prob += q[key] <= x[(g, i, u)]
                    prob += q[key] <= x[(g + 1, j, u)]
                    prob += q[key] >= x[(g, i, u)] + x[(g + 1, j, u)] - 1

    # Move-speed selection variables.
    # g=0: y0 selects move speed for (candidate i, u).
    y0: Dict[Tuple[int, int, int], Any] = {}
    if n_slots > 0:
        for i, _ in enumerate(slots[0].candidates):
            for u in uav_ids:
                ys = []
                for m_idx, _ in enumerate(move_speeds):
                    key = (i, u, m_idx)
                    y0[key] = pulp.LpVariable(f"y0_{i}_{u}_{m_idx}", lowBound=0, upBound=1, cat="Binary")
                    ys.append(y0[key])
                prob += pulp.lpSum(ys) == x[(0, i, u)]

    # g>0: r selects move speed for transition (i -> j, u).
    r: Dict[Tuple[int, int, int, int, int], Any] = {}
    for g in range(n_slots - 1):
        n_i = len(slots[g].candidates)
        n_j = len(slots[g + 1].candidates)
        for i in range(n_i):
            for j in range(n_j):
                for u in uav_ids:
                    rs = []
                    for m_idx, _ in enumerate(move_speeds):
                        key = (g, i, j, u, m_idx)
                        r[key] = pulp.LpVariable(f"r_{g}_{i}_{j}_{u}_{m_idx}", lowBound=0, upBound=1, cat="Binary")
                        prob += r[key] <= q[(g, i, j, u)]
                        rs.append(r[key])
                    prob += pulp.lpSum(rs) == q[(g, i, j, u)]

    # Barrier-synchronized timing.
    B: Dict[int, Any] = {}
    F: Dict[int, Any] = {}
    L: Dict[int, Any] = {}
    Amax: Dict[int, Any] = {}
    Amin: Dict[int, Any] = {}
    E: Dict[Tuple[int, int], Any] = {}
    H: Dict[int, Any] = {}
    Hmax = pulp.LpVariable("Hmax", lowBound=0)
    Hmin = pulp.LpVariable("Hmin", lowBound=0)

    for g, slot in enumerate(slots):
        B[g] = pulp.LpVariable(f"B_{g}", lowBound=0)
        F[g] = pulp.LpVariable(f"F_{g}", lowBound=0)
        L[g] = pulp.LpVariable(f"L_{g}", lowBound=0)
        Amax[g] = pulp.LpVariable(f"Amax_{g}", lowBound=0)
        Amin[g] = pulp.LpVariable(f"Amin_{g}", lowBound=0)
        for u in uav_ids:
            E[(g, u)] = pulp.LpVariable(f"E_{g}_{u}", lowBound=0)
            exec_expr = []
            for i, _ in enumerate(slot.candidates):
                vels = vel_table[(g, i)]
                for k, _ in enumerate(vels):
                    exec_expr.append(exec_table[(g, i, k)] * w[(g, i, u, k)])
            prob += E[(g, u)] == B[g] + pulp.lpSum(exec_expr)
            prob += F[g] >= E[(g, u)]
            prob += L[g] <= E[(g, u)]

            if g == 0:
                init_expr = []
                for i in range(len(slot.candidates)):
                    for m_idx, _ in enumerate(move_speeds):
                        init_expr.append(init_move_t[(0, i, u, m_idx)] * y0[(i, u, m_idx)])
                arrival_expr = pulp.lpSum(init_expr)
                prob += B[0] >= arrival_expr
            else:
                move_expr = []
                for i in range(len(slots[g - 1].candidates)):
                    for j in range(len(slot.candidates)):
                        for m_idx, _ in enumerate(move_speeds):
                            move_expr.append(trans_move_t[(g - 1, i, j, m_idx)] * r[(g - 1, i, j, u, m_idx)])
                arrival_expr = F[g - 1] + pulp.lpSum(move_expr)
                prob += B[g] >= arrival_expr
            prob += Amax[g] >= arrival_expr
            prob += Amin[g] <= arrival_expr

    last_g = n_slots - 1
    for u in uav_ids:
        H[u] = pulp.LpVariable(f"H_{u}", lowBound=0)
        prob += H[u] == E[(last_g, u)]
        prob += Hmax >= H[u]
        prob += Hmin <= H[u]

    # Objective:
    # 1) reduce per-slot completion spread
    # 2) reduce mission completion time
    # 3) reduce travel time
    # 4) mildly prefer lower speed where timing-equivalent
    W_SPREAD_SLOT = 6.0
    W_MAKESPAN = 1.0
    W_SLOT_FINISH_SUM = 0.30
    W_ARR_SPREAD = 2.0
    W_MOVE = 0.01
    W_SPEED = 0.01
    W_MOVE_SPEED_PREF = 0.04
    W_FINAL_BALANCE = 1.0

    obj_slot_spread = pulp.lpSum(F[g] - L[g] for g in range(n_slots))
    obj_arr_spread = pulp.lpSum(Amax[g] - Amin[g] for g in range(n_slots))
    obj_makespan = F[last_g]
    obj_slot_finish_sum = pulp.lpSum(F[g] for g in range(n_slots))
    obj_final_balance = Hmax - Hmin
    obj_speed = pulp.lpSum(
        speed_penalty[(g, i, k)] * w[(g, i, u, k)]
        for g, slot in enumerate(slots)
        for i, _ in enumerate(slot.candidates)
        for u in uav_ids
        for k, _ in enumerate(vel_table[(g, i)])
    )
    obj_move = pulp.lpSum(
        trans_move_t[(g, i, j, m_idx)] * r[(g, i, j, u, m_idx)]
        for g in range(n_slots - 1)
        for i in range(len(slots[g].candidates))
        for j in range(len(slots[g + 1].candidates))
        for u in uav_ids
        for m_idx, _ in enumerate(move_speeds)
    ) + pulp.lpSum(
        init_move_t[(0, i, u, m_idx)] * y0[(i, u, m_idx)]
        for i in range(len(slots[0].candidates))
        for u in uav_ids
        for m_idx, _ in enumerate(move_speeds)
    )
    obj_move_speed_pref = pulp.lpSum(
        move_penalty[m_idx] * r[(g, i, j, u, m_idx)]
        for g in range(n_slots - 1)
        for i in range(len(slots[g].candidates))
        for j in range(len(slots[g + 1].candidates))
        for u in uav_ids
        for m_idx, _ in enumerate(move_speeds)
    ) + pulp.lpSum(
        move_penalty[m_idx] * y0[(i, u, m_idx)]
        for i in range(len(slots[0].candidates))
        for u in uav_ids
        for m_idx, _ in enumerate(move_speeds)
    )
    prob += (
        (W_SPREAD_SLOT * obj_slot_spread)
        + (W_ARR_SPREAD * obj_arr_spread)
        + (W_MAKESPAN * obj_makespan)
        + (W_SLOT_FINISH_SUM * obj_slot_finish_sum)
        + (W_FINAL_BALANCE * obj_final_balance)
        + (W_MOVE * obj_move)
        + (W_SPEED * obj_speed)
        + (W_MOVE_SPEED_PREF * obj_move_speed_pref)
    )

    t0 = time.perf_counter()
    try:
        solver = pulp.PULP_CBC_CMD(
            msg=False,
            timeLimit=max(0.1, float(time_limit_s)),
            gapRel=max(0.0, float(mip_gap)),
        )
        status_code = prob.solve(solver)
    except TypeError:
        solver = pulp.PULP_CBC_CMD(
            msg=False,
            timeLimit=max(0.1, float(time_limit_s)),
        )
        status_code = prob.solve(solver)
    solve_ms = (time.perf_counter() - t0) * 1000.0

    status_map = getattr(pulp, "LpStatus", {})
    status_text = str(status_map.get(status_code, "Unknown"))
    if int(status_code) != int(getattr(pulp, "LpStatusOptimal", 1)):
        return _SolveResult(
            solver="cbc",
            status=f"fallback_{status_text.lower()}",
            objective=None,
            gap=None,
            solve_ms=solve_ms,
            selected={},
            move_speed_mps={},
        )

    selected: Dict[Tuple[int, int], Tuple[int, float]] = {}
    for g, slot in enumerate(slots):
        for u in uav_ids:
            picked_i = None
            for i, _ in enumerate(slot.candidates):
                xv = 0.0
                for k, _ in enumerate(vel_table[(g, i)]):
                    xv += _to_float(pulp.value(w[(g, i, u, k)]), 0.0)
                if xv > 0.5:
                    picked_i = i
                    break
            if picked_i is None:
                continue
            picked_vel = min(vel_table[(g, picked_i)])
            for k, vel in enumerate(vel_table[(g, picked_i)]):
                wv = _to_float(pulp.value(w[(g, picked_i, u, k)]), 0.0)
                if wv > 0.5:
                    picked_vel = float(vel)
                    break
            selected[(g, u)] = (int(picked_i), float(picked_vel))

    move_speed_mps: Dict[Tuple[int, int], float] = {}
    for g, slot in enumerate(slots):
        for u in uav_ids:
            pick = selected.get((g, u))
            if pick is None:
                continue
            i = int(pick[0])
            chosen_mps = 40.0
            if g == 0:
                for m_idx, mps in enumerate(move_speeds):
                    vv = _to_float(pulp.value(y0[(i, u, m_idx)]), 0.0)
                    if vv > 0.5:
                        chosen_mps = float(mps)
                        break
            else:
                prev = selected.get((g - 1, u))
                if prev is not None:
                    ip = int(prev[0])
                    for m_idx, mps in enumerate(move_speeds):
                        vv = _to_float(pulp.value(r[(g - 1, ip, i, u, m_idx)]), 0.0)
                        if vv > 0.5:
                            chosen_mps = float(mps)
                            break
            move_speed_mps[(int(g), int(u))] = float(chosen_mps)

    objective = _to_float(pulp.value(prob.objective), 0.0)
    return _SolveResult(
        solver="cbc",
        status="optimal",
        objective=objective,
        gap=None,
        solve_ms=solve_ms,
        selected=selected,
        move_speed_mps=move_speed_mps,
    )


def _solve_greedy(
    slots: List[ScheduleSlot],
    uav_ids: List[int],
    take_over_map: Optional[Dict[int, Coord]],
    allowed_uavs_by_candidate: Optional[Dict[Tuple[int, int], set[int]]] = None,
) -> _SolveResult:
    t0 = time.perf_counter()
    selected: Dict[Tuple[int, int], Tuple[int, float]] = {}
    move_speed_mps: Dict[Tuple[int, int], float] = {}
    cur_time = {u: 0.0 for u in uav_ids}
    take_over_fallback = _centroid(list((take_over_map or {}).values()))
    cur_pos = {u: _takeover_for_uav(take_over_map, u, take_over_fallback) for u in uav_ids}

    for g, slot in enumerate(slots):
        remaining = list(range(len(slot.candidates)))
        def _allowed_count(aid: int) -> int:
            return sum(
                1
                for cand_idx in remaining
                if int(aid) in (
                    allowed_uavs_by_candidate.get((g, cand_idx), set(uav_ids))
                    if isinstance(allowed_uavs_by_candidate, dict)
                    else set(uav_ids)
                )
            )

        for u in sorted(uav_ids, key=lambda x: (_allowed_count(x), cur_time[x], int(x))):
            best_idx = None
            best_cost = float("inf")
            best_vel = 100.0
            for i in remaining:
                allowed = allowed_uavs_by_candidate.get((g, i)) if isinstance(allowed_uavs_by_candidate, dict) else None
                if allowed is not None and int(u) not in allowed:
                    continue
                cand = slot.candidates[i]
                v = min(cand.vel_candidates_kmh) if cand.vel_candidates_kmh else 100.0
                move_s = _distance_m(cur_pos[u], cand.start_point) / 40.0
                exec_s = _exec_time_sec(cand.distance_m, v)
                score = (cur_time[u] + move_s + exec_s) + (0.02 * v)
                if score < best_cost:
                    best_cost = score
                    best_idx = i
                    best_vel = float(v)
            if best_idx is None:
                continue
            selected[(g, u)] = (int(best_idx), float(best_vel))
            move_speed_mps[(int(g), int(u))] = 40.0
            remaining.remove(best_idx)
            cand = slot.candidates[best_idx]
            cur_time[u] += _distance_m(cur_pos[u], cand.start_point) / 40.0
            cur_time[u] += _exec_time_sec(cand.distance_m, best_vel)
            cur_pos[u] = cand.end_point if cand.end_point is not None else cur_pos[u]

    solve_ms = (time.perf_counter() - t0) * 1000.0
    return _SolveResult(
        solver="greedy",
        status="fallback",
        objective=None,
        gap=None,
        solve_ms=solve_ms,
        selected=selected,
        move_speed_mps=move_speed_mps,
    )


def _build_timeline_and_apply_legacy(
    split_result: SplitRunResult,
    slots: List[ScheduleSlot],
    uav_ids: List[int],
    take_over_map: Optional[Dict[int, Coord]],
    selected: Dict[Tuple[int, int], Tuple[int, float]],
) -> Dict[str, Any]:
    piece_map: Dict[Tuple[int, int], SplitPiece] = {
        (int(p.parent_order), int(p.piece_index)): p for p in split_result.pieces
    }

    cur_t = {u: 0.0 for u in uav_ids}
    take_over_fallback = _centroid(list((take_over_map or {}).values()))
    cur_pos = {u: _takeover_for_uav(take_over_map, u, take_over_fallback) for u in uav_ids}
    timelines: List[Dict[str, Any]] = []
    slot_assignments: List[Dict[str, Any]] = []

    segments_by_u: Dict[int, List[Dict[str, Any]]] = {u: [] for u in uav_ids}
    task_sec_by_u = {u: 0.0 for u in uav_ids}
    move_sec_by_u = {u: 0.0 for u in uav_ids}

    for g, slot in enumerate(slots):
        slot_row = {
            "slotIndex": int(g + 1),
            "slotKey": str(slot.slot_key),
            "assignments": [],
        }
        for u in uav_ids:
            pick = selected.get((g, u))
            if pick is None:
                continue
            cand_idx, vel = pick
            if cand_idx < 0 or cand_idx >= len(slot.candidates):
                continue
            cand = slot.candidates[cand_idx]

            move_s = _distance_m(cur_pos[u], cand.start_point) / 40.0
            t0 = float(cur_t[u])
            t1 = float(t0 + move_s)
            if move_s > 0.05:
                segments_by_u[u].append(
                    {
                        "kind": "move",
                        "slotIndex": int(g + 1),
                        "slotKey": str(slot.slot_key),
                        "startSec": float(t0),
                        "endSec": float(t1),
                        "durationSec": float(move_s),
                        "parentOrder": int(cand.parent_order),
                        "label": f"Move->{cand.label}",
                        "fromCoord": _coord_copy(cur_pos[u]),
                        "toCoord": _coord_copy(cand.start_point),
                    }
                )
                move_sec_by_u[u] += move_s

            exec_s = _exec_time_sec(cand.distance_m, vel)
            t2 = float(t1 + exec_s)
            if (not cand.is_dummy) and exec_s > 0.0:
                seg = {
                    "kind": "task",
                    "slotIndex": int(g + 1),
                    "slotKey": str(slot.slot_key),
                    "startSec": float(t1),
                    "endSec": float(t2),
                    "durationSec": float(exec_s),
                    "parentOrder": int(cand.parent_order),
                    "missionType": int(cand.mission_type),
                    "sourceAreaIndex": int(cand.source_area_index),
                    "splitStage": int(cand.split_stage),
                    "label": str(cand.label),
                    "candidateID": str(cand.candidate_id),
                    "speedKmh": float(vel),
                    "distanceM": float(cand.distance_m),
                    "pieceRefs": [list(r) for r in cand.piece_refs],
                    "startCoord": _coord_copy(cand.start_point),
                    "endCoord": _coord_copy(cand.end_point),
                }
                segments_by_u[u].append(seg)
                task_sec_by_u[u] += exec_s

                for pref in cand.piece_refs:
                    p = piece_map.get((int(pref[0]), int(pref[1])))
                    if p is None:
                        continue
                    p.assigned_uav = int(u)
                    if isinstance(p.data, dict):
                        p.data.setdefault("schedule", {})
                        p.data["schedule"].update(
                            {
                                "slotIndex": int(g + 1),
                                "slotKey": str(slot.slot_key),
                                "candidateID": str(cand.candidate_id),
                                "selectedVelKmh": float(vel),
                                "startSec": float(t1),
                                "endSec": float(t2),
                                "durationSec": float(exec_s),
                                "distanceM": float(cand.distance_m),
                                "uavID": int(u),
                                "pieceGroupRefs": [list(r) for r in cand.piece_refs],
                            }
                        )

            cur_t[u] = float(t2)
            if cand.end_point is not None:
                cur_pos[u] = cand.end_point

            slot_row["assignments"].append(
                {
                    "uavID": int(u),
                    "candidateID": str(cand.candidate_id),
                    "label": str(cand.label),
                    "isDummy": bool(cand.is_dummy),
                    "velKmh": float(vel),
                }
            )
        slot_assignments.append(slot_row)

    for u in uav_ids:
        total = float(cur_t[u])
        timelines.append(
            {
                "uavID": int(u),
                "totalSec": total,
                "taskSec": float(task_sec_by_u[u]),
                "moveSec": float(move_sec_by_u[u]),
                "segments": segments_by_u[u],
            }
        )

    max_total = max((float(t["totalSec"]) for t in timelines), default=0.0)
    min_total = min((float(t["totalSec"]) for t in timelines), default=0.0) if timelines else 0.0

    return {
        "uavIDs": [int(u) for u in uav_ids],
        "slotCount": int(len(slots)),
        "timelineCount": int(len(timelines)),
        "timelines": timelines,
        "slotAssignments": slot_assignments,
        "totalMaxSec": float(max_total),
        "totalMinSec": float(min_total),
        "balanceGapSec": float(max_total - min_total),
    }


def _segment_bearing_deg(start: Optional[Coord], end: Optional[Coord]) -> float:
    b = _bearing_deg(start, end)
    if math.isfinite(b):
        return float(b)
    return 0.0


def _build_timeline_and_apply_barrier(
    split_result: SplitRunResult,
    slots: List[ScheduleSlot],
    uav_ids: List[int],
    take_over_map: Optional[Dict[int, Coord]],
    selected: Dict[Tuple[int, int], Tuple[int, float]],
    move_speed_mps: Optional[Dict[Tuple[int, int], float]] = None,
) -> Dict[str, Any]:
    piece_map: Dict[Tuple[int, int], SplitPiece] = {
        (int(p.parent_order), int(p.piece_index)): p for p in split_result.pieces
    }

    move_speed_mps = move_speed_mps if isinstance(move_speed_mps, dict) else {}
    n_slots = len(slots)

    # First pass: gather per-slot/per-uav task & move geometry/time.
    per_slot_u: Dict[Tuple[int, int], Dict[str, Any]] = {}
    take_over_fallback = _centroid(list((take_over_map or {}).values()))
    cur_pos = {u: _takeover_for_uav(take_over_map, u, take_over_fallback) for u in uav_ids}
    for g, slot in enumerate(slots):
        next_pos = dict(cur_pos)
        for u in uav_ids:
            pick = selected.get((g, u))
            if pick is None:
                continue
            cand_idx, vel_kmh = pick
            if cand_idx < 0 or cand_idx >= len(slot.candidates):
                continue
            cand = slot.candidates[cand_idx]
            ms = float(move_speed_mps.get((int(g), int(u)), 40.0) or 40.0)
            ms = max(min(ms, 60.0), 30.0)
            dist_m = _distance_m(cur_pos[u], cand.start_point)
            move_t = _move_time_sec(dist_m, ms)
            exec_t = _exec_time_sec(cand.distance_m, vel_kmh)
            per_slot_u[(g, u)] = {
                "cand": cand,
                "velKmh": float(vel_kmh),
                "moveSpeedMps": float(ms),
                "moveDistM": float(dist_m),
                "moveSec": float(move_t),
                "execSec": float(exec_t),
                "fromCoord": _coord_copy(cur_pos[u]),
                "toCoord": _coord_copy(cand.start_point),
                "endCoord": _coord_copy(cand.end_point),
            }
            next_pos[u] = cand.end_point if cand.end_point is not None else cur_pos[u]
        cur_pos = next_pos

    # Second pass: barrier slot start/finish by move + task time.
    slot_start: List[float] = [0.0 for _ in range(n_slots)]
    slot_finish: List[float] = [0.0 for _ in range(n_slots)]
    prev_release = 0.0
    for g in range(n_slots):
        arrivals: List[float] = []
        for u in uav_ids:
            row = per_slot_u.get((g, u))
            if row is None:
                continue
            arrivals.append(float(prev_release + float(row.get("moveSec", 0.0))))
        s = max(arrivals) if arrivals else float(prev_release)
        slot_start[g] = float(s)
        ends: List[float] = []
        for u in uav_ids:
            row = per_slot_u.get((g, u))
            if row is None:
                continue
            ends.append(float(s + float(row.get("execSec", 0.0))))
        f = max(ends) if ends else float(s)
        slot_finish[g] = float(f)
        prev_release = float(f)

    segments_by_u: Dict[int, List[Dict[str, Any]]] = {u: [] for u in uav_ids}
    task_sec_by_u = {u: 0.0 for u in uav_ids}
    move_sec_by_u = {u: 0.0 for u in uav_ids}
    slot_assignments: List[Dict[str, Any]] = []
    inserted_rows: List[Dict[str, Any]] = []

    for g, slot in enumerate(slots):
        slot_idx = int(g + 1)
        slot_row = {
            "slotIndex": slot_idx,
            "slotKey": str(slot.slot_key),
            "slotStartSec": float(slot_start[g]),
            "slotFinishSec": float(slot_finish[g]),
            "assignments": [],
        }
        for u in uav_ids:
            row = per_slot_u.get((g, u))
            if row is None:
                continue
            cand = row["cand"]
            vel = float(row["velKmh"])
            move_speed = float(row["moveSpeedMps"])
            move_s = float(row["moveSec"])
            exec_s = float(row["execSec"])
            prev_rel = 0.0 if g == 0 else float(slot_finish[g - 1])
            t0 = float(slot_start[g])
            # Start moving right after previous slot release.
            # If feasible, tune move speed to fill [prev_release, slot_start] with no left-side gap.
            mv0 = float(prev_rel)
            move_window = max(0.0, float(t0 - prev_rel))
            dist_m = float(row.get("moveDistM", 0.0) or 0.0)
            if move_window > 1e-6 and dist_m > 1e-6:
                eff_mps = dist_m / move_window
                if 30.0 - 1e-6 <= eff_mps <= 60.0 + 1e-6:
                    move_s = float(move_window)
                    move_speed = float(eff_mps)
            mv1 = float(mv0 + move_s)
            t1 = float(t0 + exec_s)

            if move_s > 0.05:
                segments_by_u[u].append(
                    {
                        "kind": "move",
                        "slotIndex": slot_idx,
                        "slotKey": str(slot.slot_key),
                        "startSec": mv0,
                        "endSec": mv1,
                        "durationSec": move_s,
                        "parentOrder": int(cand.parent_order),
                        "label": f"Move->{cand.label}",
                        "moveSpeedMps": float(move_speed),
                        "fromCoord": _coord_copy(row.get("fromCoord")),
                        "toCoord": _coord_copy(row.get("toCoord")),
                    }
                )
                move_sec_by_u[u] += move_s

            if (not cand.is_dummy) and exec_s > 0.0:
                seg = {
                    "kind": "task",
                    "slotIndex": slot_idx,
                    "slotKey": str(slot.slot_key),
                    "startSec": t0,
                    "endSec": t1,
                    "durationSec": exec_s,
                    "parentOrder": int(cand.parent_order),
                    "missionType": int(cand.mission_type),
                    "sourceAreaIndex": int(cand.source_area_index),
                    "splitStage": int(cand.split_stage),
                    "label": str(cand.label),
                    "candidateID": str(cand.candidate_id),
                    "speedKmh": float(vel),
                    "distanceM": float(cand.distance_m),
                    "pieceRefs": [list(r) for r in cand.piece_refs],
                    "startCoord": _coord_copy(cand.start_point),
                    "endCoord": _coord_copy(cand.end_point),
                }
                segments_by_u[u].append(seg)
                task_sec_by_u[u] += exec_s

                for pref in cand.piece_refs:
                    p = piece_map.get((int(pref[0]), int(pref[1])))
                    if p is None:
                        continue
                    p.assigned_uav = int(u)
                    if isinstance(p.data, dict):
                        p.data.setdefault("schedule", {})
                        p.data["schedule"].update(
                            {
                                "slotIndex": slot_idx,
                                "slotKey": str(slot.slot_key),
                                "candidateID": str(cand.candidate_id),
                                "selectedVelKmh": float(vel),
                                "startSec": t0,
                                "endSec": t1,
                                "durationSec": exec_s,
                                "distanceM": float(cand.distance_m),
                                "uavID": int(u),
                                "pieceGroupRefs": [list(r) for r in cand.piece_refs],
                            }
                        )

            # Single hold type:
            # hold after this slot task until departure to next slot.
            # (always until this slot finish barrier)
            if g < (n_slots - 1):
                h1 = float(slot_finish[g])
            else:
                h1 = float(slot_finish[g])
            h0 = float(t1 if exec_s > 0.0 else slot_start[g])
            hold_dur = max(0.0, h1 - h0)
            if hold_dur > 0.05:
                hold_coord = _coord_copy(cand.end_point if cand.end_point is not None else row.get("toCoord"))
                hold_seg = {
                    "kind": "sync_hold",
                    "slotIndex": slot_idx,
                    "slotKey": str(slot.slot_key),
                    "startSec": h0,
                    "endSec": h1,
                    "durationSec": hold_dur,
                    "parentOrder": int(cand.parent_order),
                    "missionType": 5,
                    "individualMissionType": 5,
                    "patternType": 1,
                    "bearingDeg": _segment_bearing_deg(_coord_copy(cand.start_point), _coord_copy(cand.end_point)),
                    "label": f"T5/P1 HOLD S{slot_idx}",
                    "candidateID": f"SYNC-S{slot_idx}-U{int(u)}",
                    "coordinateList": [hold_coord] if hold_coord is not None else [],
                    "startCoord": hold_coord,
                    "endCoord": hold_coord,
                }
                segments_by_u[u].append(hold_seg)

                if g < (n_slots - 1):
                    next_pick = selected.get((g + 1, u))
                    next_cand = None
                    if next_pick is not None:
                        ni = int(next_pick[0])
                        if 0 <= ni < len(slots[g + 1].candidates):
                            next_cand = slots[g + 1].candidates[ni]
                    before_parent_order = int(next_cand.parent_order) if next_cand is not None else int(cand.parent_order)
                    next_start = _coord_copy(next_cand.start_point) if next_cand is not None else None
                    next_end = _coord_copy(next_cand.end_point) if next_cand is not None else None
                    bdeg = _segment_bearing_deg(next_start, next_end)
                    if bdeg <= 1e-6:
                        bdeg = _segment_bearing_deg(hold_coord, next_start)
                    inserted_rows.append(
                        {
                            "uavID": int(u),
                            "slotIndex": slot_idx,
                            "slotKey": str(slot.slot_key),
                            "beforeParentOrder": int(before_parent_order),
                            "individualMissionType": 5,
                            "patternType": 1,
                            "bearingDeg": float(bdeg),
                            "startSec": float(h0),
                            "endSec": float(h1),
                            "durationSec": float(hold_dur),
                            "coordinateList": [hold_coord] if hold_coord is not None else [],
                            "reason": "sync_slot_hold",
                        }
                    )

            slot_row["assignments"].append(
                {
                    "uavID": int(u),
                    "candidateID": str(cand.candidate_id),
                    "label": str(cand.label),
                    "isDummy": bool(cand.is_dummy),
                    "velKmh": float(vel),
                    "moveSpeedMps": float(move_speed),
                    "taskStartSec": float(t0),
                    "taskEndSec": float(t1),
                }
            )
        slot_assignments.append(slot_row)

    timelines: List[Dict[str, Any]] = []
    for u in uav_ids:
        segs = sorted(segments_by_u[u], key=lambda s: (float(s.get("startSec", 0.0)), float(s.get("endSec", 0.0))))
        total = max((float(s.get("endSec", 0.0) or 0.0) for s in segs), default=0.0)
        timelines.append(
            {
                "uavID": int(u),
                "totalSec": float(total),
                "taskSec": float(task_sec_by_u[u]),
                "moveSec": float(move_sec_by_u[u]),
                "segments": segs,
            }
        )

    max_total = max((float(t["totalSec"]) for t in timelines), default=0.0)
    min_total = min((float(t["totalSec"]) for t in timelines), default=0.0) if timelines else 0.0

    return {
        "uavIDs": [int(u) for u in uav_ids],
        "slotCount": int(len(slots)),
        "timelineCount": int(len(timelines)),
        "timelines": timelines,
        "slotAssignments": slot_assignments,
        "totalMaxSec": float(max_total),
        "totalMinSec": float(min_total),
        "balanceGapSec": float(max_total - min_total),
        "insertedMissions": inserted_rows,
        "insertedMissionCount": int(len(inserted_rows)),
    }


def run_milp_scheduling(
    split_result: SplitRunResult,
    mrpk: Optional[Dict[str, Any]] = None,
    uav_ids_override: Optional[List[int]] = None,
    time_limit_s: float = 1.5,
    mip_gap: float = 0.02,
    respect_piece_assignment: bool = False,
) -> Dict[str, Any]:
    if split_result is None:
        return {
            "solver": "none",
            "status": "no_split",
            "slotCount": 0,
            "timelines": [],
        }

    uav_ids = [int(x) for x in (uav_ids_override or split_result.uav_ids or []) if int(x) > 0]
    if not uav_ids:
        uav_ids = [4]
    split_result.uav_ids = list(uav_ids)
    split_result.uav_count = len(uav_ids)

    slots = _build_slots(split_result, uav_ids)
    allowed_uavs_by_candidate = _allowed_uavs_by_candidate(
        split_result,
        slots,
        uav_ids,
        respect_preassigned=bool(respect_piece_assignment),
    )
    if not slots:
        out = {
            "solver": "none",
            "status": "empty",
            "syncMode": "none",
            "slotCount": 0,
            "timelines": [],
            "slotAssignments": [],
            "respectPieceAssignment": bool(respect_piece_assignment),
        }
        setattr(split_result, "schedule_result", out)
        return out

    sync_mode = str(os.environ.get("ASSIGNMENT_SCHED_SYNC_MODE", "barrier") or "barrier").strip().lower()
    if sync_mode not in {"legacy", "barrier"}:
        sync_mode = "barrier"

    take_over_map = _takeover_map(mrpk)
    if sync_mode == "legacy":
        solved = _solve_with_milp_legacy(
            slots=slots,
            uav_ids=uav_ids,
            take_over_map=take_over_map,
            time_limit_s=time_limit_s,
            mip_gap=mip_gap,
            allowed_uavs_by_candidate=allowed_uavs_by_candidate,
        )
    else:
        solved = _solve_with_milp_barrier(
            slots=slots,
            uav_ids=uav_ids,
            take_over_map=take_over_map,
            time_limit_s=time_limit_s,
            mip_gap=mip_gap,
            allowed_uavs_by_candidate=allowed_uavs_by_candidate,
        )
    if not solved.selected:
        solved = _solve_greedy(
            slots=slots,
            uav_ids=uav_ids,
            take_over_map=take_over_map,
            allowed_uavs_by_candidate=allowed_uavs_by_candidate,
        )

    if sync_mode == "legacy":
        timeline_payload = _build_timeline_and_apply_legacy(
            split_result=split_result,
            slots=slots,
            uav_ids=uav_ids,
            take_over_map=take_over_map,
            selected=solved.selected,
        )
    else:
        timeline_payload = _build_timeline_and_apply_barrier(
            split_result=split_result,
            slots=slots,
            uav_ids=uav_ids,
            take_over_map=take_over_map,
            selected=solved.selected,
            move_speed_mps=solved.move_speed_mps,
        )
    out = {
        "solver": str(solved.solver),
        "status": str(solved.status),
        "syncMode": str(sync_mode),
        "objective": solved.objective,
        "gap": solved.gap,
        "solveMs": float(solved.solve_ms),
        "timeLimitSec": float(time_limit_s),
        "mipGap": float(mip_gap),
        "respectPieceAssignment": bool(respect_piece_assignment),
    }
    out.update(timeline_payload)
    setattr(split_result, "schedule_result", out)
    return out
