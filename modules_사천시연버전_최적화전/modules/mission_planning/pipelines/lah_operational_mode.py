from __future__ import annotations

from copy import deepcopy
from typing import Any, Dict, List, Optional, Tuple


REGION_ACP = 3
REGION_ATTACK_WAIT = 4
REGION_BATTLE_POSITION = 5
REGION_TARGET = 6

SPECIAL_HOLD_ALTITUDE_M = 1500


def _to_int(value: Any) -> Optional[int]:
    try:
        return int(value)
    except Exception:
        return None


def _to_float(value: Any) -> Optional[float]:
    try:
        return float(value)
    except Exception:
        return None


def _normalize_coordinate(payload: object | None) -> Optional[Dict[str, Any]]:
    if not isinstance(payload, dict):
        return None
    lat = _to_float(payload.get("latitude"))
    lon = _to_float(payload.get("longitude"))
    if lat is None or lon is None:
        return None
    out: Dict[str, Any] = {
        "latitude": float(lat),
        "longitude": float(lon),
    }
    alt = _to_float(payload.get("altitude"))
    if alt is not None:
        out["altitude"] = int(round(float(alt)))
    return out


def _normalize_coord_list(payload: object | None) -> List[Dict[str, Any]]:
    rows = payload if isinstance(payload, list) else []
    out: List[Dict[str, Any]] = []
    for item in rows:
        coord = _normalize_coordinate(item)
        if coord is not None:
            out.append(coord)
    return out


def _centroid_coordinate(coords: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    if not coords:
        return None
    lat_vals = [float(item["latitude"]) for item in coords if item.get("latitude") is not None]
    lon_vals = [float(item["longitude"]) for item in coords if item.get("longitude") is not None]
    if not lat_vals or not lon_vals:
        return None
    out: Dict[str, Any] = {
        "latitude": sum(lat_vals) / float(len(lat_vals)),
        "longitude": sum(lon_vals) / float(len(lon_vals)),
    }
    alt_vals = [float(item["altitude"]) for item in coords if item.get("altitude") is not None]
    if alt_vals:
        out["altitude"] = int(round(sum(alt_vals) / float(len(alt_vals))))
    return out


def _mission_list(input_plan_or_missions: object | None) -> List[Dict[str, Any]]:
    if isinstance(input_plan_or_missions, dict):
        rows = input_plan_or_missions.get("inputMissionList")
    else:
        rows = input_plan_or_missions
    if not isinstance(rows, list):
        return []
    return [row for row in rows if isinstance(row, dict)]


def _mission_id(mission: Dict[str, Any], fallback: int) -> int:
    value = _to_int(mission.get("inputMissionID"))
    return int(value) if value is not None and value > 0 else int(fallback)


def _mission_detail(mission: Dict[str, Any]) -> Dict[str, Any]:
    detail = mission.get("missionDetail")
    return detail if isinstance(detail, dict) else {}


def _first_line_coords(mission: Dict[str, Any]) -> List[Dict[str, Any]]:
    detail = _mission_detail(mission)
    line_list = detail.get("lineList") if isinstance(detail.get("lineList"), list) else []
    for line in line_list:
        if not isinstance(line, dict):
            continue
        coords = _normalize_coord_list(line.get("coordinateList"))
        if coords:
            return coords
    coords = _normalize_coord_list(detail.get("coordinateList"))
    return coords


def _first_area_coords(mission: Dict[str, Any]) -> List[Dict[str, Any]]:
    detail = _mission_detail(mission)
    area_list = detail.get("areaList") if isinstance(detail.get("areaList"), list) else []
    for area in area_list:
        if not isinstance(area, dict):
            continue
        coords = _normalize_coord_list(area.get("coordinateList"))
        if coords:
            return coords
    return []


def _point_on_segment(
    px: float,
    py: float,
    ax: float,
    ay: float,
    bx: float,
    by: float,
    *,
    eps: float = 1e-10,
) -> bool:
    cross = (px - ax) * (by - ay) - (py - ay) * (bx - ax)
    if abs(cross) > eps:
        return False
    dot = (px - ax) * (bx - ax) + (py - ay) * (by - ay)
    if dot < -eps:
        return False
    length_sq = (bx - ax) * (bx - ax) + (by - ay) * (by - ay)
    return dot <= length_sq + eps


def _point_in_polygon(coord: object | None, polygon_coords: object | None) -> bool:
    point = _normalize_coordinate(coord)
    polygon = _normalize_coord_list(polygon_coords)
    if point is None or len(polygon) < 3:
        return False

    x = float(point["longitude"])
    y = float(point["latitude"])
    inside = False
    previous = polygon[-1]
    for current in polygon:
        xi = float(current["longitude"])
        yi = float(current["latitude"])
        xj = float(previous["longitude"])
        yj = float(previous["latitude"])
        if _point_on_segment(x, y, xi, yi, xj, yj):
            return True
        if (yi > y) != (yj > y):
            x_intersect = (xj - xi) * (y - yi) / (yj - yi) + xi
            if x <= x_intersect:
                inside = not inside
        previous = current
    return bool(inside)


def _area_anchor(mission: Dict[str, Any], *, forced_altitude: Optional[int] = None) -> Optional[Dict[str, Any]]:
    anchor = _centroid_coordinate(_first_area_coords(mission))
    if anchor is None:
        return None
    if forced_altitude is not None:
        anchor["altitude"] = int(forced_altitude)
    return anchor


def _line_anchor(mission: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    coords = _first_line_coords(mission)
    return dict(coords[0]) if coords else None


def _is_area_region(mission: Dict[str, Any], region_type: int) -> bool:
    if _to_int(mission.get("regionType")) != int(region_type):
        return False
    mission_type = _to_int(mission.get("inputMissionType")) or 0
    if mission_type not in (2, 3, 6):
        return False
    return bool(_first_area_coords(mission))


def _post_target_egress_line_indices(missions: List[Dict[str, Any]], target_idx: int) -> List[int]:
    indices: List[int] = []
    for idx in range(int(target_idx) + 1, len(missions)):
        mission = missions[idx]
        if not _first_line_coords(mission):
            continue
        # Region 5 after the target is the target-to-battle return corridor.
        # LAH holds at the battle position through that phase and egresses via
        # the later passage 6/7 line missions.
        if _to_int(mission.get("regionType")) == REGION_BATTLE_POSITION:
            continue
        indices.append(int(idx))
    return indices


def detect_lah_special_operation(input_plan_or_missions: object | None) -> Optional[Dict[str, Any]]:
    missions = _mission_list(input_plan_or_missions)
    if not missions:
        return None

    attack_wait_idx: Optional[int] = None
    battle_idx: Optional[int] = None
    target_idx: Optional[int] = None
    for idx, mission in enumerate(missions):
        if attack_wait_idx is None:
            if _is_area_region(mission, REGION_ATTACK_WAIT):
                attack_wait_idx = int(idx)
            continue
        if battle_idx is None:
            if _is_area_region(mission, REGION_BATTLE_POSITION):
                battle_idx = int(idx)
            continue
        if target_idx is None and _is_area_region(mission, REGION_TARGET):
            target_idx = int(idx)
            break

    if attack_wait_idx is None or battle_idx is None or target_idx is None:
        return None
    if not (attack_wait_idx < battle_idx < target_idx):
        return None

    attack_wait_mission = missions[attack_wait_idx]
    battle_mission = missions[battle_idx]
    target_mission = missions[target_idx]
    attack_wait_anchor = _area_anchor(attack_wait_mission, forced_altitude=SPECIAL_HOLD_ALTITUDE_M)
    battle_anchor = _area_anchor(battle_mission, forced_altitude=SPECIAL_HOLD_ALTITUDE_M)
    target_area_coords = _first_area_coords(target_mission)
    target_anchor = _area_anchor(target_mission, forced_altitude=SPECIAL_HOLD_ALTITUDE_M)
    if attack_wait_anchor is None or battle_anchor is None or target_anchor is None:
        return None

    egress_line_indices = _post_target_egress_line_indices(missions, target_idx)
    egress_merge_idx = egress_line_indices[-1] if egress_line_indices else None
    battle_attack_start_idx = min(int(battle_idx) + 1, len(missions) - 1)
    battle_attack_end_idx = int(egress_merge_idx) if egress_merge_idx is not None else None

    return {
        "mode": "attack_wait_battle_target",
        "holdAltitudeM": int(SPECIAL_HOLD_ALTITUDE_M),
        "attackWaitIndex": int(attack_wait_idx),
        "battlePositionIndex": int(battle_idx),
        "targetIndex": int(target_idx),
        "egressLineIndices": [int(idx) for idx in egress_line_indices],
        "egressMergeIndex": int(egress_merge_idx) if egress_merge_idx is not None else None,
        "attackWaitInputMissionID": _mission_id(attack_wait_mission, attack_wait_idx + 1),
        "battlePositionInputMissionID": _mission_id(battle_mission, battle_idx + 1),
        "targetInputMissionID": _mission_id(target_mission, target_idx + 1),
        "egressMergeInputMissionID": (
            _mission_id(missions[int(egress_merge_idx)], int(egress_merge_idx) + 1)
            if egress_merge_idx is not None
            else None
        ),
        "egressMergeSourceInputMissionID": (
            _mission_id(missions[int(egress_line_indices[0])], int(egress_line_indices[0]) + 1)
            if egress_line_indices
            else None
        ),
        "battleAttackStartIndex": int(battle_attack_start_idx),
        "battleAttackStartInputMissionID": _mission_id(
            missions[int(battle_attack_start_idx)],
            int(battle_attack_start_idx) + 1,
        ),
        "battleAttackEndIndexExclusive": int(battle_attack_end_idx) if battle_attack_end_idx is not None else None,
        "battleAttackEndInputMissionIDExclusive": (
            _mission_id(missions[int(battle_attack_end_idx)], int(battle_attack_end_idx) + 1)
            if battle_attack_end_idx is not None
            else None
        ),
        "attackWaitCoordinate": attack_wait_anchor,
        "battlePositionCoordinate": battle_anchor,
        "targetCoordinate": target_anchor,
        "targetAreaCoordinateList": deepcopy(target_area_coords),
        "egressMergedCoordinateList": _merged_line_coords(missions, egress_line_indices),
    }


def _point_mission_info(coord: Dict[str, Any], *, force_altitude: bool) -> Dict[str, Any]:
    info: Dict[str, Any] = {
        "individualMissionType": 9,
        "patternType": 12,
        "autoZoomIn": False,
        "coordinateList": [dict(coord)],
        "targetID": None,
    }
    if force_altitude:
        info["forceAltitudeM"] = int(SPECIAL_HOLD_ALTITUDE_M)
    return info


def _line_mission_info(mission: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    coords = _first_line_coords(mission)
    if not coords:
        return None
    return {
        "individualMissionType": 7,
        "patternType": 10,
        "autoZoomIn": False,
        "coordinateList": deepcopy(coords),
        "targetID": None,
    }


def _same_coordinate(left: Dict[str, Any], right: Dict[str, Any], *, eps: float = 1e-9) -> bool:
    try:
        return (
            abs(float(left.get("latitude")) - float(right.get("latitude"))) <= eps
            and abs(float(left.get("longitude")) - float(right.get("longitude"))) <= eps
        )
    except Exception:
        return False


def _append_unique_coordinate(rows: List[Dict[str, Any]], coord: Dict[str, Any]) -> None:
    if rows and _same_coordinate(rows[-1], coord):
        return
    rows.append(dict(coord))


def _merged_line_coords(
    missions: List[Dict[str, Any]],
    mission_indices: List[int],
) -> List[Dict[str, Any]]:
    out: List[Dict[str, Any]] = []
    for idx in mission_indices:
        if idx < 0 or idx >= len(missions):
            continue
        for coord in _first_line_coords(missions[idx]):
            _append_unique_coordinate(out, coord)
    return out


def _merged_line_mission_info(
    missions: List[Dict[str, Any]],
    mission_indices: List[int],
) -> Optional[Dict[str, Any]]:
    coords = _merged_line_coords(missions, mission_indices)
    if not coords:
        return None
    return {
        "individualMissionType": 7,
        "patternType": 10,
        "autoZoomIn": False,
        "coordinateList": deepcopy(coords),
        "targetID": None,
    }


def _generic_lah_info_from_input(mission: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    mission_type = _to_int(mission.get("inputMissionType")) or 0
    if mission_type in (1, 4, 5, 7):
        return _line_mission_info(mission)
    if mission_type in (2, 3, 6):
        anchor = _area_anchor(mission)
        if anchor is None:
            return None
        return _point_mission_info(anchor, force_altitude=False)
    return None


def lah_special_info_for_input(
    input_plan_or_missions: object | None,
    input_mission_id: int,
) -> Optional[Dict[str, Any]]:
    missions = _mission_list(input_plan_or_missions)
    if not missions:
        return None
    profile = detect_lah_special_operation(missions)
    if profile is None:
        return None

    target_idx: Optional[int] = None
    for idx, mission in enumerate(missions):
        if _mission_id(mission, idx + 1) == int(input_mission_id):
            target_idx = int(idx)
            break
    if target_idx is None:
        return None

    info, _source_id, _behavior = lah_special_info_for_index(missions, profile, int(target_idx))
    return info


def lah_special_info_for_index(
    missions_or_plan: object | None,
    profile: Dict[str, Any],
    mission_index: int,
) -> Tuple[Optional[Dict[str, Any]], Optional[int], str]:
    missions = _mission_list(missions_or_plan)
    if not missions or mission_index < 0 or mission_index >= len(missions):
        return None, None, "invalid"

    attack_wait_idx = int(profile.get("attackWaitIndex", -1))
    battle_idx = int(profile.get("battlePositionIndex", -1))
    egress_merge_idx = _to_int(profile.get("egressMergeIndex"))
    attack_wait_coord = _normalize_coordinate(profile.get("attackWaitCoordinate"))
    battle_coord = _normalize_coordinate(profile.get("battlePositionCoordinate"))
    if attack_wait_coord is None or battle_coord is None:
        return None, None, "invalid"

    if mission_index == 0:
        start_coord = _line_anchor(missions[0]) or _area_anchor(missions[0])
        if start_coord is None:
            return None, None, "initial_hold_missing"
        return _point_mission_info(start_coord, force_altitude=False), _mission_id(missions[0], 1), "initial_hold"

    if mission_index <= attack_wait_idx:
        source = missions[mission_index - 1]
        return _generic_lah_info_from_input(source), _mission_id(source, mission_index), "lag_follow"

    if mission_index == attack_wait_idx + 1:
        return _point_mission_info(attack_wait_coord, force_altitude=True), int(
            profile.get("attackWaitInputMissionID") or 0
        ), "attack_wait_hold"

    if mission_index <= battle_idx:
        source = missions[mission_index - 1]
        return _generic_lah_info_from_input(source), _mission_id(source, mission_index), "battle_approach_lag_follow"

    if egress_merge_idx is not None and mission_index == int(egress_merge_idx):
        egress_indices = [
            int(idx)
            for idx in (profile.get("egressLineIndices") or [])
            if _to_int(idx) is not None
        ]
        info = _merged_line_mission_info(missions, egress_indices)
        if info is not None:
            return info, int(profile.get("egressMergeSourceInputMissionID") or 0), "egress_merged_follow"

    if egress_merge_idx is None or mission_index < int(egress_merge_idx):
        return _point_mission_info(battle_coord, force_altitude=True), int(
            profile.get("battlePositionInputMissionID") or 0
        ), "battle_position_hold"

    source = missions[mission_index - 1]
    return _generic_lah_info_from_input(source), _mission_id(source, mission_index), "post_target_lag_follow"


def build_lah_special_sequence(input_plan_or_missions: object | None) -> Optional[List[Dict[str, Any]]]:
    missions = _mission_list(input_plan_or_missions)
    profile = detect_lah_special_operation(missions)
    if profile is None:
        return None

    rows: List[Dict[str, Any]] = []
    for idx, mission in enumerate(missions):
        input_mid = _mission_id(mission, idx + 1)
        info, source_mid, behavior = lah_special_info_for_index(missions, profile, int(idx))
        if info is None:
            continue
        rows.append(
            {
                "inputMissionID": int(input_mid),
                "sourceInputMissionID": int(source_mid or input_mid),
                "behavior": str(behavior),
                "individualMissionInfo": info,
            }
        )
    return rows if rows else None


def special_attack_coordinate(profile: object | None) -> Optional[Dict[str, Any]]:
    if not isinstance(profile, dict):
        return None
    return _normalize_coordinate(profile.get("battlePositionCoordinate"))


def special_target_contains_coordinate(profile: object | None, coord: object | None) -> bool:
    if not isinstance(profile, dict):
        return False
    return _point_in_polygon(coord, profile.get("targetAreaCoordinateList"))


def special_force_battle_attack(profile: object | None, input_mission_id: object | None) -> bool:
    if not isinstance(profile, dict):
        return False
    current_id = _to_int(input_mission_id)
    start_id = _to_int(profile.get("battleAttackStartInputMissionID"))
    end_id = _to_int(profile.get("battleAttackEndInputMissionIDExclusive"))
    if current_id is None or start_id is None:
        return False
    if int(current_id) < int(start_id):
        return False
    if end_id is not None and int(current_id) >= int(end_id):
        return False
    return True


def special_resume_coordinate(profile: object | None) -> Optional[Dict[str, Any]]:
    if not isinstance(profile, dict):
        return None
    return _normalize_coordinate(profile.get("attackWaitCoordinate"))
