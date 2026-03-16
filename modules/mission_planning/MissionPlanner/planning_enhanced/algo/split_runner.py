from __future__ import annotations

from typing import Any, Dict, List, Optional

from . import split_algorithms as sa
from ..assignment.allocator import assign_pieces_round_robin
from ..models import DirectionDebug, SplitPiece, SplitRunResult
from ..scheduling.simple_scheduler import schedule_by_parent_order


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


def _mission_entry_point(mission: Dict[str, Any]) -> Optional[Dict[str, float]]:
    if not isinstance(mission, dict):
        return None
    mtype = int(mission.get("inputMissionType", 0) or 0)
    detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else {}

    if mtype in (1, 4, 5, 7):
        line_list = detail.get("lineList") if isinstance(detail, dict) else None
        if isinstance(line_list, list) and line_list:
            coords = line_list[0].get("coordinateList")
            if isinstance(coords, list) and coords:
                p = coords[0]
                return {
                    "latitude": float(p["latitude"]),
                    "longitude": float(p["longitude"]),
                    "altitude": float(p.get("altitude", 0.0)),
                }
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

    if mtype in (2, 3, 6):
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


def run_split_pipeline(
    cmpk: Dict[str, Any],
    mrpk: Dict[str, Any],
    uav_ids: List[int],
    apply_assignment: bool = True,
    apply_scheduling: bool = True,
) -> SplitRunResult:
    missions = cmpk.get("inputMissionList")
    if not isinstance(missions, list):
        raise ValueError("0201 payload has no valid 'inputMissionList'.")

    uav_count = len(uav_ids) if uav_ids else 1
    prev_pt = _extract_takeover_centroid(mrpk)

    all_pieces: List[SplitPiece] = []
    directions: List[DirectionDebug] = []

    for i, mission in enumerate(missions):
        if not isinstance(mission, dict):
            continue

        idx = i + 1
        next_pt = _find_next_entry_point(missions, i)
        mission_id = mission.get("inputMissionID", idx)
        mtype = int(mission.get("inputMissionType", 0) or 0)
        detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else {}
        mission_debugs: List[DirectionDebug] = []

        if mtype in (1, 4, 5, 7):
            debug = DirectionDebug(
                parent_order=idx,
                mission_id=mission_id,
                mission_type=mtype,
                source_area_index=None,
                prev_point=dict(prev_pt) if isinstance(prev_pt, dict) else None,
                next_point=dict(next_pt) if isinstance(next_pt, dict) else None,
            )
            line_list = detail.get("lineList") if isinstance(detail, dict) else None
            if isinstance(line_list, list) and line_list:
                coords = line_list[0].get("coordinateList")
                if isinstance(coords, list) and coords:
                    debug.line_start = dict(coords[0])
                    debug.line_end = dict(coords[-1])
            mission_debugs.append(debug)

        elif mtype in (2, 3, 6):
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

        subs = sa.split_mission_into_subareas(mission, uav_count, prev_pt, next_pt)
        for piece_idx, sub in enumerate(subs, start=1):
            all_pieces.append(
                SplitPiece(
                    parent_order=idx,
                    mission_id=mission_id,
                    mission_type=mtype,
                    piece_index=piece_idx,
                    data=sub,
                )
            )

        # Keep same prev-point update behavior as current pipeline.
        line_list = detail.get("lineList") if isinstance(detail, dict) else []
        area_list = detail.get("areaList") if isinstance(detail, dict) else []
        coord_list = detail.get("coordinateList") if isinstance(detail, dict) else []
        if mtype in (1, 4, 5, 7) and isinstance(line_list, list) and line_list:
            coords = line_list[-1].get("coordinateList")
            if isinstance(coords, list) and coords:
                prev_pt = coords[-1]
        elif mtype in (1, 4, 5, 7) and isinstance(coord_list, list) and coord_list:
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
        assigned = assign_pieces_round_robin(len(all_pieces), uav_ids)
        for piece, aid in zip(all_pieces, assigned):
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
    )
