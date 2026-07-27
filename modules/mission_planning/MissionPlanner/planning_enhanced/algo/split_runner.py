from __future__ import annotations
import itertools
import math
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from . import split_algorithms as sa
from ..assignment.allocator import assign_pieces_round_robin
from ..models import DirectionDebug, SplitPiece, SplitRunResult
from ..scheduling.simple_scheduler import schedule_by_parent_order
from modules.mission_planning.planning_modes import (
    MissionPlanningMode,
    mission_mode_context,
    resolve_mission_planning_mode,
)

try:
    from modules.mission_planning.pipelines.ground_maneuver_mode import (
        detect_ground_maneuver_profile,
        mission_branch_count as _mission_branch_count,
    )
    from modules.mission_planning.runtime.state import branch_ownership as _branch_ownership
except Exception:  # pragma: no cover - defensive optional import
    detect_ground_maneuver_profile = None  # type: ignore
    _mission_branch_count = None  # type: ignore
    _branch_ownership = None  # type: ignore

try:
    from modules.mission_planning.planners.donut_patrol.production import (
        build_donut_band_pieces as _donut_band_pieces,
        is_donut_boundary_mission as _is_donut_boundary_mission,
    )
except Exception:  # pragma: no cover - defensive optional import
    _donut_band_pieces = None  # type: ignore
    _is_donut_boundary_mission = None  # type: ignore

try:
    from modules.common import replan_perf
except Exception:
    import sys as _sys

    _COMMON_DIR = next(
        (
            parent / "common"
            for parent in Path(__file__).resolve().parents
            if (parent / "common" / "replan_perf.py").exists()
        ),
        None,
    )
    if _COMMON_DIR is not None and str(_COMMON_DIR) not in _sys.path:
        _sys.path.insert(0, str(_COMMON_DIR))
    import replan_perf  # type: ignore


def _safe_perm_count(n: int, r: int) -> int:
    try:
        return int(math.perm(int(n), int(r)))
    except Exception:
        return 0


def _centroid_llh(ll: List[Dict[str, Any]]) -> Dict[str, float]:
    lat = sum(float(p["latitude"]) for p in ll) / len(ll)
    lon = sum(float(p["longitude"]) for p in ll) / len(ll)
    alt = float(ll[0].get("altitude", 0.0))
    return {"latitude": lat, "longitude": lon, "altitude": alt}


def _extract_takeover_centroid(mrpk: Dict[str, Any]) -> Optional[Dict[str, float]]:
    infos = mrpk.get("takeOverInfoList")
    if not isinstance(infos, list) or not infos:
        return None
    coords = []
    for item in infos:
        if not isinstance(item, dict):
            continue
        coord = item.get("coordinate")
        if isinstance(coord, dict) and "latitude" in coord and "longitude" in coord:
            coords.append(coord)
    if not coords:
        return None
    return _centroid_llh(coords)


def _extract_takeover_map(mrpk: Dict[str, Any]) -> Dict[int, Dict[str, float]]:
    infos = mrpk.get("takeOverInfoList")
    if not isinstance(infos, list):
        return {}
    out: Dict[int, Dict[str, float]] = {}
    for item in infos:
        if not isinstance(item, dict):
            continue
        aid = item.get("aircraftID")
        coord = item.get("coordinate")
        if not isinstance(aid, int) or not isinstance(coord, dict):
            continue
        if "latitude" not in coord or "longitude" not in coord:
            continue
        out[int(aid)] = {
            "latitude": float(coord["latitude"]),
            "longitude": float(coord["longitude"]),
            "altitude": float(coord.get("altitude", 0.0)),
        }
    return out


def _dist_m(a: Dict[str, Any] | None, b: Dict[str, Any] | None) -> float:
    if not isinstance(a, dict) or not isinstance(b, dict):
        return float("inf")
    try:
        lat1 = float(a["latitude"])
        lon1 = float(a["longitude"])
        lat2 = float(b["latitude"])
        lon2 = float(b["longitude"])
    except Exception:
        return float("inf")
    deg_m = 111_132.0
    dx = (lon2 - lon1) * deg_m * math.cos(math.radians((lat1 + lat2) / 2.0))
    dy = (lat2 - lat1) * deg_m
    return math.hypot(dx, dy)


def _piece_entry_point(piece: SplitPiece) -> Optional[Dict[str, float]]:
    data = piece.data if isinstance(piece, SplitPiece) else {}
    if not isinstance(data, dict):
        return None
    for key in ("Centerline", "coordinateList", "rawCoordinateList"):
        coords = data.get(key)
        if isinstance(coords, list) and coords:
            pt = coords[0]
            if isinstance(pt, dict) and "latitude" in pt and "longitude" in pt:
                return {
                    "latitude": float(pt["latitude"]),
                    "longitude": float(pt["longitude"]),
                    "altitude": float(pt.get("altitude", 0.0)),
                }
    return None


def _is_single_point_coordinate_only_mission(mission: Dict[str, Any]) -> bool:
    if not isinstance(mission, dict):
        return False
    detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else {}
    line_list = detail.get("lineList")
    area_list = detail.get("areaList")
    if isinstance(line_list, list) and line_list:
        return False
    if isinstance(area_list, list) and area_list:
        return False
    coord_list = detail.get("coordinateList")
    return isinstance(coord_list, list) and len(coord_list) == 1


def _assign_group_by_takeover_distance(
    pieces: List[SplitPiece],
    uav_ids: List[int],
    takeover_map: Dict[int, Dict[str, float]],
) -> Dict[int, int]:
    if not pieces or not uav_ids:
        return {}

    entry_by_piece = {idx: _piece_entry_point(piece) for idx, piece in enumerate(pieces)}
    usable_uavs = [aid for aid in uav_ids if isinstance(takeover_map.get(aid), dict)]
    if not usable_uavs:
        return {}

    if len(pieces) <= len(usable_uavs) and len(usable_uavs) <= 8:
        best_cost = float("inf")
        best_map: Dict[int, int] | None = None
        perf_enabled = replan_perf.is_enabled()
        perf_start = replan_perf.start_timer() if perf_enabled else None
        checked_candidates = 0
        try:
            for candidate_uavs in itertools.permutations(usable_uavs, len(pieces)):
                if perf_enabled:
                    checked_candidates += 1
                cost = 0.0
                mapping: Dict[int, int] = {}
                for piece_idx, aid in enumerate(candidate_uavs):
                    cost += _dist_m(takeover_map.get(aid), entry_by_piece.get(piece_idx))
                    mapping[piece_idx] = aid
                if cost < best_cost:
                    best_cost = cost
                    best_map = mapping
        finally:
            if perf_enabled:
                replan_perf.add_elapsed(
                    "mission_planning.split_runner.takeover_permutation",
                    perf_start,
                    groups=1,
                    pieces=len(pieces),
                    usable_uavs=len(usable_uavs),
                    estimated_candidates=_safe_perm_count(len(usable_uavs), len(pieces)),
                    checked_candidates=checked_candidates,
                )
        if best_map is not None:
            return best_map

    assigned: Dict[int, int] = {}
    used_uavs: set[int] = set()
    for piece_idx, piece in enumerate(pieces):
        best_aid = None
        best_cost = float("inf")
        for aid in usable_uavs:
            if len(pieces) <= len(usable_uavs) and aid in used_uavs:
                continue
            cost = _dist_m(takeover_map.get(aid), entry_by_piece.get(piece_idx))
            if cost < best_cost:
                best_cost = cost
                best_aid = aid
        if best_aid is None:
            continue
        assigned[piece_idx] = best_aid
        used_uavs.add(best_aid)
    return assigned


def _mission_entry_point(mission: Dict[str, Any]) -> Optional[Dict[str, float]]:
    if not isinstance(mission, dict):
        return None
    if _is_single_point_coordinate_only_mission(mission):
        return None
    geometry = sa.mission_geometry(mission)
    detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else {}

    if geometry == "line":
        segments = sa.valid_line_segments(detail)
        if segments:
            coords = segments[0].get("coordinateList")
            if isinstance(coords, list) and coords:
                p = coords[0]
                return {
                    "latitude": float(p["latitude"]),
                    "longitude": float(p["longitude"]),
                    "altitude": float(p.get("altitude", 0.0)),
                }
        return None

    if geometry == "coordinate":
        coord_list = detail.get("coordinateList") if isinstance(detail, dict) else None
        if isinstance(coord_list, list) and coord_list:
            p = coord_list[0]
            if isinstance(p, dict) and ("latitude" in p) and ("longitude" in p):
                return {
                    "latitude": float(p["latitude"]),
                    "longitude": float(p["longitude"]),
                    "altitude": float(p.get("altitude", 0.0)),
                }
        return None

    if geometry == "area":
        area_list = detail.get("areaList") if isinstance(detail, dict) else None
        if isinstance(area_list, list) and area_list:
            centers: List[Dict[str, Any]] = []
            for area in area_list:
                if not isinstance(area, dict):
                    continue
                coords = area.get("coordinateList")
                if isinstance(coords, list) and coords:
                    centers.append(_centroid_llh(coords))
            if centers:
                return _centroid_llh(centers)
    return None


def _find_next_entry_point(missions: List[Dict[str, Any]], from_index: int) -> Optional[Dict[str, float]]:
    for j in range(from_index + 1, len(missions)):
        nxt = missions[j]
        if not isinstance(nxt, dict):
            continue
        pt = _mission_entry_point(nxt)
        if pt is not None:
            return pt
    return None


def assign_split_result_by_takeover_distance(
    split_result: SplitRunResult,
    mrpk: Dict[str, Any],
    uav_ids: List[int],
) -> Dict[str, Any]:
    if split_result is None:
        return {"groups": 0, "pieceCount": 0, "assignedPieces": 0, "uavSummary": {}}

    takeover_map = _extract_takeover_map(mrpk)
    grouped: Dict[int, List[SplitPiece]] = {}
    for piece in split_result.pieces:
        grouped.setdefault(int(piece.parent_order), []).append(piece)

    assigned_pieces = 0
    uav_summary: Dict[int, int] = {}
    for parent_order in sorted(grouped.keys()):
        group = grouped[parent_order]
        assigned_group = _assign_group_by_takeover_distance(group, uav_ids, takeover_map)
        if assigned_group:
            for idx, piece in enumerate(group):
                aid = assigned_group.get(idx)
                if aid is None:
                    continue
                piece.assigned_uav = int(aid)
        else:
            assigned = assign_pieces_round_robin(len(group), uav_ids)
            for piece, aid in zip(group, assigned):
                piece.assigned_uav = int(aid)

        for piece in group:
            aid = int(piece.assigned_uav or 0)
            if aid <= 0:
                continue
            assigned_pieces += 1
            uav_summary[aid] = int(uav_summary.get(aid, 0)) + 1

    return {
        "groups": int(len(grouped)),
        "pieceCount": int(len(split_result.pieces)),
        "assignedPieces": int(assigned_pieces),
        "uavSummary": {int(k): int(v) for k, v in sorted(uav_summary.items())},
    }


def _branch_entry_points(profile: Dict[str, Any]) -> List[Dict[str, float]]:
    anchor_order = int(profile.get("anchorOrder") or 0)
    missions_by_order = profile.get("missionsByOrder") or {}
    entry = missions_by_order.get(anchor_order) or missions_by_order.get(str(anchor_order))
    branches = (entry or {}).get("branches") if isinstance(entry, dict) else None
    points: List[Dict[str, float]] = []
    for branch in branches or []:
        pt = branch.get("entry") if isinstance(branch, dict) else None
        if isinstance(pt, dict) and "latitude" in pt and "longitude" in pt:
            points.append(pt)
    return points


def _establish_branch_ownership(
    profile: Dict[str, Any],
    uav_ids: List[int],
    takeover_map: Dict[int, Dict[str, float]],
    fallback_pt: Optional[Dict[str, float]],
) -> Dict[int, List[int]]:
    """Anchor branch->UAV ownership by geometry (nearest takeover position).

    Optimal 1:1 for the first min(branches, UAVs); spare UAVs then join their
    nearest branch (sub-division), and any branch left without an owner takes its
    nearest UAV. Ownership order within a branch is [primary, ...extras].
    """
    entries = _branch_entry_points(profile)
    n = len(entries)
    ids = [int(a) for a in uav_ids if a is not None]
    ownership: Dict[int, List[int]] = {i: [] for i in range(n)}
    if n == 0 or not ids:
        return ownership

    def _pos(aid: int) -> Optional[Dict[str, float]]:
        pos = takeover_map.get(int(aid))
        return pos if isinstance(pos, dict) else fallback_pt

    pairs: List[Tuple[float, int, int]] = []
    for bi, entry in enumerate(entries):
        for aid in ids:
            pairs.append((_dist_m(entry, _pos(aid)), bi, aid))
    pairs.sort(key=lambda item: item[0])

    used_branch: set[int] = set()
    used_uav: set[int] = set()
    target_pairs = min(n, len(ids))
    for _cost, bi, aid in pairs:
        if bi in used_branch or aid in used_uav:
            continue
        ownership[bi].append(int(aid))
        used_branch.add(bi)
        used_uav.add(aid)
        if len(used_branch) >= target_pairs:
            break

    for aid in ids:
        if aid in used_uav:
            continue
        best_bi = min(range(n), key=lambda b: _dist_m(entries[b], _pos(aid)))
        ownership[best_bi].append(int(aid))
        used_uav.add(aid)

    for bi in range(n):
        if ownership[bi]:
            continue
        best_aid = min(ids, key=lambda a: _dist_m(entries[bi], _pos(a)))
        ownership[bi].append(int(best_aid))
    return ownership


def _resolve_branch_ownership(
    profile: Dict[str, Any],
    uav_ids: List[int],
    takeover_map: Dict[int, Dict[str, float]],
    fallback_pt: Optional[Dict[str, float]],
    *,
    persist: bool,
    prefer_fresh: bool = False,
) -> Dict[int, List[int]]:
    package_id = profile.get("packageID")
    profile_package_type = int(profile.get("packageType") or 0)
    # Stored ownership is authoritative for both initial-plan refreshes and
    # replans. Check it even on an
    # apply_assignment=True refresh so current aircraft positions can never
    # re-anchor an already-owned Type-2 branch set.
    if _branch_ownership is not None:
        existing = _branch_ownership.get_branch_ownership(package_id)
        if existing and (
            profile_package_type == 2
            or (not persist and not prefer_fresh)
        ):
            return {int(k): [int(a) for a in v] for k, v in existing.items()}
    if not persist and profile_package_type == 2:
        # A Type-2 replan without the initial authoritative map must not invent
        # owners from current positions. The caller preserves the active IMP.
        return {}

    ownership = _establish_branch_ownership(profile, uav_ids, takeover_map, fallback_pt)
    if persist and ownership and _branch_ownership is not None:
        try:
            ownership = _branch_ownership.register_branch_ownership(
                package_id=package_id,
                branch_count=int(profile.get("branchCount") or len(ownership)),
                ownership=ownership,
                branch_mission_ids=profile.get("branchInputMissionIDs"),
                anchor_input_mission_id=profile.get("anchorInputMissionID"),
                source="split_pipeline",
                immutable=profile_package_type == 2,
            )
        except Exception:
            pass
    return ownership


def _assign_branch_group(group: List[SplitPiece], ownership: Dict[int, List[int]]) -> None:
    by_branch: Dict[int, List[SplitPiece]] = {}
    for piece in group:
        data = piece.data if isinstance(piece.data, dict) else {}
        bi = data.get("branchIndex")
        try:
            bi = int(bi)
        except Exception:
            bi = 0
        by_branch.setdefault(bi, []).append(piece)
    for bi, pieces in by_branch.items():
        owners = ownership.get(int(bi)) or []
        if not owners:
            continue
        for j, piece in enumerate(pieces):
            piece.assigned_uav = int(owners[j % len(owners)])


def _validate_type2_branch_ownership(
    ownership: Dict[int, List[int]],
    *,
    branch_count: int,
    uav_ids: List[int],
) -> None:
    """Enforce one-and-only-one Type-2 branch membership per current UAV."""
    expected_branches = set(range(max(0, int(branch_count))))
    if set(int(index) for index in ownership) != expected_branches:
        raise RuntimeError("Type-2 branch ownership is incomplete or has extra branches.")
    flattened = [
        int(aircraft_id)
        for branch_index in sorted(expected_branches)
        for aircraft_id in (ownership.get(int(branch_index)) or [])
    ]
    if any(not ownership.get(int(branch_index)) for branch_index in expected_branches):
        raise RuntimeError("Type-2 branch ownership contains an unowned branch.")
    if len(flattened) != len(set(flattened)):
        raise RuntimeError("A Type-2 UAV is assigned to more than one branch.")
    current_uavs = {int(aircraft_id) for aircraft_id in uav_ids if int(aircraft_id) > 0}
    if not set(flattened) & current_uavs:
        raise RuntimeError(
            "Type-2 branch owner set does not match the current UAV set; replan deferred."
        )


def _surviving_branch_ownership(
    ownership: Dict[int, List[int]],
    uav_ids: List[int],
) -> Tuple[Dict[int, List[int]], List[int]]:
    """Drop permanently absent owners, and report the branches left unowned.

    A UAV that is gone for good takes its branch set with it: the element is
    removed from the plan rather than handed to another UAV, which would break
    the one-element-one-owner contract.  A branch that keeps at least one owner
    stays, planned by whoever is still there.  Temporary detours (attack
    tracking, prior missions) never reach here - those triggers preserve the
    whole assignment instead of re-splitting.
    """

    current = {int(aircraft_id) for aircraft_id in uav_ids if int(aircraft_id) > 0}
    surviving: Dict[int, List[int]] = {}
    dropped: List[int] = []
    for branch_index, owners in ownership.items():
        kept = [int(owner) for owner in (owners or []) if int(owner) in current]
        surviving[int(branch_index)] = kept
        if not kept:
            dropped.append(int(branch_index))
    return surviving, sorted(dropped)


def run_split_pipeline(
    cmpk: Dict[str, Any],
    mrpk: Dict[str, Any],
    uav_ids: List[int],
    apply_assignment: bool = True,
    apply_scheduling: bool = True,
    planning_mode: MissionPlanningMode | Dict[str, Any] | None = None,
) -> SplitRunResult:
    missions = cmpk.get("inputMissionList")
    if not isinstance(missions, list):
        raise ValueError("0201 payload has no valid 'inputMissionList'.")

    resolved_mode = planning_mode or resolve_mission_planning_mode(cmpk)
    mode_context = mission_mode_context(mode=resolved_mode)
    uav_count = len(uav_ids) if uav_ids else 1
    prev_pt = _extract_takeover_centroid(mrpk)
    takeover_map = _extract_takeover_map(mrpk)

    # Type 2 각자도생: give each lineList/areaList element to a single UAV owner
    # (no width split) instead of pooling all elements across every UAV. Ownership
    # is anchored once and reused for the whole branch span (steps 4-6).
    branch_profile: Optional[Dict[str, Any]] = None
    branch_ownership_map: Dict[int, List[int]] = {}
    # Ownership minus permanently absent UAVs; drives which elements survive.
    surviving_ownership_map: Dict[int, List[int]] = {}
    dropped_branches: List[int] = []
    branch_owner_counts: Optional[List[int]] = None
    branch_orders: set[int] = set()
    if detect_ground_maneuver_profile is not None:
        try:
            branch_profile = detect_ground_maneuver_profile(
                cmpk, package_type=mode_context.get("package_type")
            )
        except Exception:
            branch_profile = None
    if isinstance(branch_profile, dict):
        branch_orders = {int(o) for o in branch_profile.get("branchOrders") or []}
        anchor_order = int(branch_profile.get("anchorOrder") or 0)
        anchor_entry = (branch_profile.get("missionsByOrder") or {}).get(anchor_order) or {}
        # Type 2 is immutable. Preserve the established Type-3 behaviour in
        # which its entry LINE may re-anchor the later Type-3 branch set.
        prefer_fresh = bool(
            not apply_assignment
            and int(branch_profile.get("packageType") or 0) == 3
            and str(anchor_entry.get("kind") or "") == "line"
        )
        branch_ownership_map = _resolve_branch_ownership(
            branch_profile,
            uav_ids,
            takeover_map,
            prev_pt,
            persist=bool(apply_assignment),
            prefer_fresh=prefer_fresh,
        )
        if (
            not branch_ownership_map
            and not apply_assignment
            and int(branch_profile.get("packageType") or 0) == 2
        ):
            raise RuntimeError(
                "Type-2 immutable branch ownership is unavailable; replan deferred."
            )
        if branch_ownership_map:
            n = int(branch_profile.get("branchCount") or len(branch_ownership_map))
            if int(branch_profile.get("packageType") or 0) == 2:
                _validate_type2_branch_ownership(
                    branch_ownership_map,
                    branch_count=n,
                    uav_ids=uav_ids,
                )
            surviving_ownership_map, dropped_branches = _surviving_branch_ownership(
                branch_ownership_map, uav_ids
            )
            branch_owner_counts = [
                len(surviving_ownership_map.get(i, [])) for i in range(n)
            ]
    elif (
        not branch_orders
        and _branch_ownership is not None
        and int(mode_context.get("package_type") or 0) in (2, 3)
    ):
        # Single-mission next-collab replan: a later branch mission (e.g. 목표
        # regionType=6 type2 / 착륙 regionType=9 type3) can't be recognized from
        # its own regionType, so fall back to the persisted store that recorded
        # which mission IDs are branches and their sticky ownership.
        try:
            meta = _branch_ownership.get_branch_meta(cmpk.get("inputMissionPackageID"))
        except Exception:
            meta = {}
        stored_ids = set(int(x) for x in (meta.get("branch_mission_ids") or []))
        stored_ownership = meta.get("ownership") or {}
        if stored_ids and stored_ownership and _mission_branch_count is not None:
            for i, mission in enumerate(missions):
                if not isinstance(mission, dict):
                    continue
                mid = mission.get("inputMissionID")
                try:
                    mid_int = int(mid)
                except Exception:
                    mid_int = None
                if mid_int in stored_ids and _mission_branch_count(mission) >= 1:
                    branch_orders.add(i + 1)
            if branch_orders:
                branch_ownership_map = {
                    int(k): [int(a) for a in v] for k, v in stored_ownership.items()
                }
                n = int(meta.get("branch_count") or len(branch_ownership_map))
                if int(mode_context.get("package_type") or 0) == 2:
                    _validate_type2_branch_ownership(
                        branch_ownership_map,
                        branch_count=n,
                        uav_ids=uav_ids,
                    )
                surviving_ownership_map, dropped_branches = _surviving_branch_ownership(
                    branch_ownership_map, uav_ids
                )
                branch_owner_counts = [
                    len(surviving_ownership_map.get(i, [])) for i in range(n)
                ]

    all_pieces: List[SplitPiece] = []
    directions: List[DirectionDebug] = []

    for i, mission in enumerate(missions):
        if not isinstance(mission, dict):
            continue
        if _is_single_point_coordinate_only_mission(mission):
            continue
        geometry = sa.mission_geometry(mission)
        if geometry is None:
            # No plannable shape (e.g. zero-length line): skip instead of failing.
            continue

        idx = i + 1
        next_pt = _find_next_entry_point(missions, i)
        mission_id = mission.get("inputMissionID", idx)
        mtype = int(mission.get("inputMissionType", 0) or 0)
        detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else {}
        mission_debugs: List[DirectionDebug] = []

        if geometry in ("line", "coordinate"):
            debug = DirectionDebug(
                parent_order=idx,
                mission_id=mission_id,
                mission_type=mtype,
                source_area_index=None,
                prev_point=dict(prev_pt) if isinstance(prev_pt, dict) else None,
                next_point=dict(next_pt) if isinstance(next_pt, dict) else None,
            )
            segments = sa.valid_line_segments(detail)
            if segments:
                coords = segments[0].get("coordinateList")
                if isinstance(coords, list) and coords:
                    debug.line_start = dict(coords[0])
                    debug.line_end = dict(coords[-1])
            mission_debugs.append(debug)

        elif geometry == "area":
            # Multi-area mission: build branch bearing per area (parallel from prev -> each area).
            area_list = detail.get("areaList") if isinstance(detail, dict) else None
            if isinstance(area_list, list) and area_list:
                for area_idx, area in enumerate(area_list, start=1):
                    if not isinstance(area, dict):
                        continue
                    poly = area.get("coordinateList")
                    if not (isinstance(poly, list) and poly):
                        continue
                    center, bearing_move, bearing_in, bearing_out = sa._resolve_area_bearing(prev_pt, next_pt, poly)
                    debug = DirectionDebug(
                        parent_order=idx,
                        mission_id=mission_id,
                        mission_type=mtype,
                        source_area_index=int(area_idx),
                        prev_point=dict(prev_pt) if isinstance(prev_pt, dict) else None,
                        next_point=dict(next_pt) if isinstance(next_pt, dict) else None,
                    )
                    debug.center_point = {
                        "latitude": float(center["latitude"]),
                        "longitude": float(center["longitude"]),
                        "altitude": float(center.get("altitude", 0.0)),
                    }
                    debug.bearing_in_deg = float(bearing_in) if bearing_in is not None else None
                    debug.bearing_out_deg = float(bearing_out) if bearing_out is not None else None
                    debug.bearing_move_deg = float(bearing_move)
                    debug.bearing_split_deg = float((bearing_move + 90.0) % 360.0)
                    mission_debugs.append(debug)
            if not mission_debugs:
                mission_debugs.append(
                    DirectionDebug(
                        parent_order=idx,
                        mission_id=mission_id,
                        mission_type=mtype,
                        source_area_index=None,
                        prev_point=dict(prev_pt) if isinstance(prev_pt, dict) else None,
                        next_point=dict(next_pt) if isinstance(next_pt, dict) else None,
                    )
                )
        else:
            mission_debugs.append(
                DirectionDebug(
                    parent_order=idx,
                    mission_id=mission_id,
                    mission_type=mtype,
                    source_area_index=None,
                    prev_point=dict(prev_pt) if isinstance(prev_pt, dict) else None,
                    next_point=dict(next_pt) if isinstance(next_pt, dict) else None,
                )
            )

        directions.extend(mission_debugs)

        mission_branch_counts = branch_owner_counts if int(idx) in branch_orders else None
        donut_split = None
        if (
            _is_donut_boundary_mission is not None
            and int(mode_context.get("package_type") or 0) == 4
            and _is_donut_boundary_mission(mission)
        ):
            # Type 4 도넛 경계: annular band per UAV (각자도생 소유). Replans read
            # the sticky store so bands never migrate; a fresh initial plan
            # anchors by takeover geometry and persists.
            stored: Dict[int, List[int]] = {}
            if not apply_assignment and _branch_ownership is not None:
                try:
                    stored = _branch_ownership.get_branch_ownership(cmpk.get("inputMissionPackageID"))
                except Exception:
                    stored = {}
            try:
                donut_split = _donut_band_pieces(
                    mission,
                    uav_ids,
                    takeover_map,
                    ownership_override=stored or None,
                )
            except Exception:
                donut_split = None
        if donut_split is not None:
            subs, donut_ownership = donut_split
            branch_orders.add(int(idx))
            branch_ownership_map = {int(k): [int(a) for a in v] for k, v in donut_ownership.items()}
            if apply_assignment and _branch_ownership is not None and branch_ownership_map:
                try:
                    _branch_ownership.register_branch_ownership(
                        package_id=cmpk.get("inputMissionPackageID"),
                        branch_count=len(branch_ownership_map),
                        ownership=branch_ownership_map,
                        branch_mission_ids=[mission_id],
                        anchor_input_mission_id=mission_id,
                        source="donut_split",
                    )
                except Exception:
                    pass
        else:
            subs = sa.split_mission_into_subareas(
                mission,
                uav_count,
                prev_pt,
                next_pt,
                mission_branch_counts,
                split_branch_area_into_two=bool(
                    int(mode_context.get("package_type") or 0) in (2, 3)
                    and int(idx) in branch_orders
                    and geometry == "area"
                ),
            )
        # Type id may disagree with the shape that was actually planned; align the
        # piece's type family with its geometry so downstream family checks
        # (type decider fallback, area review, metrics) stay consistent.
        piece_mtype = mtype
        if geometry == "line" and mtype not in (1, 7):
            piece_mtype = 1
        elif geometry == "area" and mtype not in (2, 3, 4, 5, 6):
            piece_mtype = 2
        for piece_idx, sub in enumerate(subs, start=1):
            all_pieces.append(
                SplitPiece(
                    parent_order=idx,
                    mission_id=mission_id,
                    mission_type=piece_mtype,
                    piece_index=piece_idx,
                    data=sub,
                )
            )

        # Keep same prev-point update behavior as current pipeline.
        line_segments = sa.valid_line_segments(detail)
        area_list = detail.get("areaList") if isinstance(detail, dict) else []
        coord_list = detail.get("coordinateList") if isinstance(detail, dict) else []
        if geometry == "line" and line_segments:
            coords = line_segments[-1].get("coordinateList")
            if isinstance(coords, list) and coords:
                prev_pt = coords[-1]
        elif geometry in ("line", "coordinate") and isinstance(coord_list, list) and coord_list:
            last = coord_list[-1]
            if isinstance(last, dict) and ("latitude" in last) and ("longitude" in last):
                prev_pt = {
                    "latitude": float(last["latitude"]),
                    "longitude": float(last["longitude"]),
                    "altitude": float(last.get("altitude", 0.0)),
                }
        elif isinstance(area_list, list) and area_list:
            centers: List[Dict[str, Any]] = []
            for area in area_list:
                if not isinstance(area, dict):
                    continue
                coords = area.get("coordinateList")
                if isinstance(coords, list) and coords:
                    centers.append(_centroid_llh(coords))
            if centers:
                prev_pt = _centroid_llh(centers)

    if apply_assignment:
        grouped: Dict[int, List[SplitPiece]] = {}
        for piece in all_pieces:
            grouped.setdefault(int(piece.parent_order), []).append(piece)

        for parent_order in sorted(grouped.keys()):
            group = grouped[parent_order]
            if branch_ownership_map and int(parent_order) in branch_orders:
                # 각자도생: assign each branch piece to its owning UAV(s), sticky.
                _assign_branch_group(group, surviving_ownership_map or branch_ownership_map)
                continue
            assigned_group = _assign_group_by_takeover_distance(group, uav_ids, takeover_map)
            if assigned_group:
                for idx, piece in enumerate(group):
                    piece.assigned_uav = assigned_group.get(idx)
            else:
                assigned = assign_pieces_round_robin(len(group), uav_ids)
                for piece, aid in zip(group, assigned):
                    piece.assigned_uav = aid

    if apply_scheduling:
        scheduled = schedule_by_parent_order(all_pieces)
    else:
        scheduled = list(all_pieces)
    return SplitRunResult(
        uav_count=uav_count,
        uav_ids=uav_ids,
        pieces=scheduled,
        directions=directions,
        planning_mode=mode_context,
        branch_ownership={int(k): [int(a) for a in v] for k, v in branch_ownership_map.items()},
        dropped_branches=[int(index) for index in dropped_branches],
        branch_orders=sorted(int(o) for o in branch_orders),
    )
