from __future__ import annotations

from collections import OrderedDict
from copy import deepcopy
import math
import threading
from typing import Any, Dict, List, Optional, Tuple


REGION_ACP = 3
REGION_ATTACK_WAIT = 4
REGION_BATTLE_POSITION = 5
REGION_TARGET = 6

ANTI_ARMOR_REVIEWED_PATTERN = (
    (1, REGION_ACP),
    (1, REGION_ATTACK_WAIT),
    (2, REGION_ATTACK_WAIT),
    (1, REGION_BATTLE_POSITION),
    (2, REGION_BATTLE_POSITION),
    (1, REGION_TARGET),
    (2, REGION_TARGET),
    (1, REGION_BATTLE_POSITION),
    (1, REGION_ACP),
    (1, 2),
)

ANTI_ARMOR_REFRESHED_PATTERN = (
    (1, REGION_ACP),
    (1, REGION_ATTACK_WAIT),
    (2, REGION_ATTACK_WAIT),
    (1, REGION_BATTLE_POSITION),
    (2, REGION_BATTLE_POSITION),
    (1, REGION_TARGET),
    (2, REGION_TARGET),
    (1, REGION_BATTLE_POSITION),
    (1, REGION_ATTACK_WAIT),
    (1, REGION_BATTLE_POSITION),
    (2, REGION_BATTLE_POSITION),
    (1, REGION_TARGET),
    (2, REGION_TARGET),
    (1, REGION_BATTLE_POSITION),
    (1, REGION_ACP),
    (1, 2),
)

_TERMINAL_COVER_CACHE_MAX = 64
_TERMINAL_COVER_CACHE_LOCK = threading.RLock()
_TERMINAL_COVER_CACHE: "OrderedDict[tuple, tuple[Dict[str, Any], Dict[str, Any]]]" = OrderedDict()


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


def _mission_pattern(missions: List[Dict[str, Any]]) -> Tuple[Tuple[Optional[int], Optional[int]], ...]:
    return tuple(
        (
            _to_int(mission.get("inputMissionType")),
            _to_int(mission.get("regionType")),
        )
        for mission in missions
    )


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


def _constraint_line_rows(mission: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Keep the source LINE geometry, including width, for LAH containment."""

    detail = _mission_detail(mission)
    source_rows = detail.get("lineList") if isinstance(detail.get("lineList"), list) else []
    rows: List[Dict[str, Any]] = []
    for source in source_rows:
        if not isinstance(source, dict):
            continue
        coords = _normalize_coord_list(source.get("coordinateList"))
        width_m = _to_float(source.get("width", source.get("widthM")))
        if (
            len(coords) < 2
            or width_m is None
            or not math.isfinite(width_m)
            or width_m <= 0.0
        ):
            continue
        row = deepcopy(source)
        # ICD Line.width is uint, so keep the geometry value but never emit a
        # JSON floating-point number (for example ``1000.0``).  The SIM and
        # external receivers validate the JSON scalar type, not just equality.
        row["width"] = max(0, min(50_000, int(round(float(width_m)))))
        row["coordinateList"] = coords
        rows.append(row)
    return rows


def _constraint_area_rows(mission: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Keep AREA/Hole geometry as an internal LAH detour constraint."""

    detail = _mission_detail(mission)
    source_rows = detail.get("areaList") if isinstance(detail.get("areaList"), list) else []
    rows: List[Dict[str, Any]] = []
    for source in source_rows:
        if not isinstance(source, dict):
            continue
        coords = _normalize_coord_list(source.get("coordinateList"))
        if len(coords) < 3:
            continue
        row = deepcopy(source)
        row["coordinateList"] = coords
        rows.append(row)
    return rows


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


def _area_anchor(mission: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    return _centroid_coordinate(_first_area_coords(mission))


def _detail_coordinate(
    mission: Dict[str, Any],
    keys: Tuple[str, ...],
) -> Optional[Dict[str, Any]]:
    detail = _mission_detail(mission)
    for source in (detail, mission):
        if not isinstance(source, dict):
            continue
        for key in keys:
            coord = _normalize_coordinate(source.get(key))
            if coord is None:
                continue
            return coord
    return None


def _battle_attack_anchor(mission: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    return _detail_coordinate(
        mission,
        (
            "battleAttackCoordinate",
            "recommendedAttackCoordinate",
            "attackCoordinate",
            "firePositionCoordinate",
            "battleFireCoordinate",
        ),
    )


def _line_anchor(mission: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    coords = _first_line_coords(mission)
    return dict(coords[0]) if coords else None


def _line_end_anchor(mission: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    coords = _first_line_coords(mission)
    return dict(coords[-1]) if coords else None


def _is_area_region(mission: Dict[str, Any], region_type: int) -> bool:
    if _to_int(mission.get("regionType")) != int(region_type):
        return False
    mission_type = _to_int(mission.get("inputMissionType")) or 0
    if mission_type not in (2, 3, 4, 5, 6):
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


def _discover_target_battle_cycles(missions: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Pair every battle-position AREA with its following target AREA by meaning, not index."""

    battle_indices = [
        int(index)
        for index, mission in enumerate(missions)
        if _is_area_region(mission, REGION_BATTLE_POSITION)
    ]
    target_indices = [
        int(index)
        for index, mission in enumerate(missions)
        if _is_area_region(mission, REGION_TARGET)
    ]
    cycles: List[Dict[str, Any]] = []
    used_targets: set[int] = set()
    for battle_order, battle_index in enumerate(battle_indices):
        next_battle_index = (
            battle_indices[battle_order + 1]
            if battle_order + 1 < len(battle_indices)
            else len(missions)
        )
        target_index = next(
            (
                index
                for index in target_indices
                if battle_index < index < next_battle_index and index not in used_targets
            ),
            None,
        )
        if target_index is None:
            continue
        battle_mission = missions[battle_index]
        target_mission = missions[target_index]
        battle_attack_coord = _battle_attack_anchor(
            battle_mission,
        ) or _area_anchor(battle_mission)
        target_area_coords = _first_area_coords(target_mission)
        if battle_attack_coord is None or not target_area_coords:
            continue
        used_targets.add(int(target_index))
        cycles.append(
            {
                "battlePositionIndex": int(battle_index),
                "battlePositionInputMissionID": _mission_id(battle_mission, battle_index + 1),
                "battleAttackCoordinate": deepcopy(battle_attack_coord),
                "battleAreaList": deepcopy(_constraint_area_rows(battle_mission)),
                "targetIndex": int(target_index),
                "targetInputMissionID": _mission_id(target_mission, target_index + 1),
                "targetAreaCoordinateList": deepcopy(target_area_coords),
                "attackInputMissionIDs": [
                    _mission_id(missions[index], index + 1)
                    for index in range(int(battle_index) + 1, int(target_index) + 1)
                ],
            }
        )
    return cycles


def detect_lah_special_operation(
    input_plan_or_missions: object | None,
    *,
    planning_mode: object | None = None,
) -> Optional[Dict[str, Any]]:
    _ = planning_mode
    missions = _mission_list(input_plan_or_missions)
    if not missions:
        return None

    mission_pattern = _mission_pattern(missions)
    anti_armor_reviewed = mission_pattern == ANTI_ARMOR_REVIEWED_PATTERN
    anti_armor_refreshed = mission_pattern == ANTI_ARMOR_REFRESHED_PATTERN
    attack_wait_idx: Optional[int] = None
    battle_idx: Optional[int] = None
    target_idx: Optional[int] = None
    if anti_armor_reviewed or anti_armor_refreshed:
        attack_wait_idx = 2
        battle_idx = 4
        target_idx = 6
    else:
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
    attack_wait_anchor = _area_anchor(attack_wait_mission)
    battle_anchor = _area_anchor(battle_mission)
    battle_attack_anchor = _battle_attack_anchor(battle_mission)
    target_area_coords = _first_area_coords(target_mission)
    target_anchor = _area_anchor(target_mission)
    if attack_wait_anchor is None or battle_anchor is None or target_anchor is None:
        return None
    if battle_attack_anchor is None:
        battle_attack_anchor = dict(battle_anchor)

    if anti_armor_reviewed:
        egress_line_indices = [idx for idx in (7, 8, 9) if idx < len(missions) and _first_line_coords(missions[idx])]
        egress_merge_idx = None
        battle_attack_start_idx = 5
        battle_attack_end_idx = 7
    elif anti_armor_refreshed:
        egress_line_indices = [
            idx
            for idx in (7, 8, 13, 14, 15)
            if idx < len(missions) and _first_line_coords(missions[idx])
        ]
        egress_merge_idx = None
        battle_attack_start_idx = 5
        battle_attack_end_idx = 7
    else:
        egress_line_indices = _post_target_egress_line_indices(missions, target_idx)
        egress_merge_idx = egress_line_indices[-1] if egress_line_indices else None
        battle_attack_start_idx = min(int(battle_idx) + 1, len(missions) - 1)
        battle_attack_end_idx = int(egress_merge_idx) if egress_merge_idx is not None else None

    target_battle_cycles = _discover_target_battle_cycles(missions)
    if not target_battle_cycles:
        target_battle_cycles = [
            {
                "battlePositionIndex": int(battle_idx),
                "battlePositionInputMissionID": _mission_id(battle_mission, battle_idx + 1),
                "battleAttackCoordinate": deepcopy(battle_attack_anchor),
                "battleAreaList": deepcopy(_constraint_area_rows(battle_mission)),
                "targetIndex": int(target_idx),
                "targetInputMissionID": _mission_id(target_mission, target_idx + 1),
                "targetAreaCoordinateList": deepcopy(target_area_coords),
                "attackInputMissionIDs": [
                    _mission_id(missions[idx], idx + 1)
                    for idx in range(int(battle_attack_start_idx), int(battle_attack_end_idx or len(missions)))
                ],
            }
        ]

    if anti_armor_refreshed:
        profile_mode = "anti_armor_air_strike_refresh"
    elif anti_armor_reviewed:
        profile_mode = "anti_armor_air_strike_review"
    else:
        profile_mode = "attack_wait_battle_target"

    return {
        "mode": profile_mode,
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
        # The 공격대기지역 polygon, so the manned hold inside it can be placed on
        # terrain that masks it from the 목표지역 instead of on the bare centroid.
        "attackWaitAreaList": deepcopy(_constraint_area_rows(attack_wait_mission)),
        "battlePositionCoordinate": battle_anchor,
        "battleAttackCoordinate": battle_attack_anchor,
        "battlePositionAreaList": deepcopy(_constraint_area_rows(battle_mission)),
        "targetCoordinate": target_anchor,
        "targetAreaCoordinateList": deepcopy(target_area_coords),
        "targetBattleCycles": target_battle_cycles,
        "egressMergedCoordinateList": _merged_line_coords(missions, egress_line_indices),
    }


def _point_mission_info(coord: Dict[str, Any]) -> Dict[str, Any]:
    """A single-point manned hold.

    Altitude is left to the established terrain-following and UAV-LOS
    passes; this builder never pins one.
    """

    return {
        "individualMissionType": 9,
        "patternType": 12,
        "autoZoomIn": False,
        "coordinateList": [dict(coord)],
        "targetID": None,
    }


def _deterministic_terminal_threat_coordinates(
    coords: object | None,
    *,
    maximum: int = 5,
) -> List[Dict[str, Any]]:
    """Return target centroid plus evenly distributed target vertices."""

    limit = max(0, int(maximum))
    if limit <= 0:
        return []
    vertices: List[Dict[str, Any]] = []
    for coord in _normalize_coord_list(coords):
        if any(_same_coordinate(coord, existing) for existing in vertices):
            continue
        vertices.append(dict(coord))
    if len(vertices) >= 2 and _same_coordinate(vertices[0], vertices[-1]):
        vertices.pop()
    if not vertices:
        return []

    out: List[Dict[str, Any]] = []
    centroid = _centroid_coordinate(vertices)
    if centroid is not None:
        out.append(dict(centroid))

    slots = max(0, limit - len(out))
    if slots <= 0:
        return out[:limit]
    if len(vertices) <= slots:
        selected_vertices = vertices
    elif slots == 1:
        selected_vertices = [vertices[0]]
    else:
        indices = [
            int(round(float(index) * float(len(vertices) - 1) / float(slots - 1)))
            for index in range(slots)
        ]
        selected_vertices = [vertices[index] for index in indices]
    for coord in selected_vertices:
        if any(_same_coordinate(coord, existing) for existing in out):
            continue
        out.append(dict(coord))
        if len(out) >= limit:
            break
    return out


def _cover_search_radius_m() -> float:
    """Radius the manned hold may take cover within, around its anchor.

    ``0`` keeps the legacy behaviour of staying inside the tasked AREA.  A
    positive value lets the aircraft use whatever terrain is nearby, which is
    what cover actually depends on - the area it is tasked to photograph has no
    bearing on where a ridge happens to be.
    """

    try:
        from modules.mission_planning.MissionPlanner.runtime_settings import (
            get_runtime_attack_float,
        )

        value = float(get_runtime_attack_float("lah_cover_search_radius_m", 1500.0))
    except Exception:
        value = 1500.0
    return value if math.isfinite(value) and value > 0.0 else 0.0


def _terminal_cover_cache_key(
    fallback: Dict[str, Any],
    constraints: List[Dict[str, Any]],
    threats: List[Dict[str, Any]],
) -> tuple:
    def _coord_key(value: object | None) -> tuple:
        coord = _normalize_coordinate(value)
        if coord is None:
            return ()
        return (
            round(float(coord["latitude"]), 7),
            round(float(coord["longitude"]), 7),
            int(coord.get("altitude", 0) or 0),
        )

    area_key = tuple(
        (
            bool(row.get("isHole")),
            tuple(_coord_key(coord) for coord in row.get("coordinateList") or []),
        )
        for row in constraints
    )
    return (
        _coord_key(fallback),
        area_key,
        tuple(_coord_key(coord) for coord in threats),
        round(_cover_search_radius_m(), 1),
    )


def _terminal_cover_point_mission_info(
    fallback_coordinate: Dict[str, Any],
    *,
    area_rows: object | None,
    threat_coordinates: object | None,
) -> Dict[str, Any]:
    """Select an AREA-contained DEM cover point, falling back without failure."""

    fallback = _normalize_coordinate(fallback_coordinate) or dict(fallback_coordinate)
    constraints = [deepcopy(row) for row in area_rows or [] if isinstance(row, dict)]
    threats = _deterministic_terminal_threat_coordinates(threat_coordinates, maximum=5)
    selected = dict(fallback)
    diagnostics: Dict[str, Any] = {
        "applied": False,
        "reason": "missing_area_geometry" if not constraints else "selector_unavailable",
    }

    if constraints:
        cache_key = _terminal_cover_cache_key(fallback, constraints, threats)
        with _TERMINAL_COVER_CACHE_LOCK:
            cached = _TERMINAL_COVER_CACHE.get(cache_key)
            if cached is not None:
                _TERMINAL_COVER_CACHE.move_to_end(cache_key)
                selected, diagnostics = deepcopy(cached)
            else:
                try:
                    from modules.mission_planning.MissionPlanner.data_def.lah_terminal_cover import (
                        select_lah_terminal_cover_point,
                    )

                    candidate, selector_diagnostics = select_lah_terminal_cover_point(
                        constraints,
                        dict(fallback),
                        threat_coordinates=deepcopy(threats) if threats else None,
                        max_candidates=25,
                        max_ray_samples=48,
                        search_radius_m=_cover_search_radius_m(),
                    )
                    normalized_candidate = _normalize_coordinate(candidate)
                    if normalized_candidate is not None:
                        selected = normalized_candidate
                    if isinstance(selector_diagnostics, dict):
                        diagnostics = deepcopy(selector_diagnostics)
                    else:
                        diagnostics = {"applied": normalized_candidate is not None, "reason": "ok"}
                    _TERMINAL_COVER_CACHE[cache_key] = deepcopy((selected, diagnostics))
                    _TERMINAL_COVER_CACHE.move_to_end(cache_key)
                    while len(_TERMINAL_COVER_CACHE) > int(_TERMINAL_COVER_CACHE_MAX):
                        _TERMINAL_COVER_CACHE.popitem(last=False)
                except Exception as exc:
                    diagnostics = {
                        "applied": False,
                        "reason": f"selector_error:{type(exc).__name__}",
                    }

    diagnostics = dict(diagnostics)
    diagnostics.setdefault("reason", "ok")
    diagnostics["selected"] = dict(
        _normalize_coordinate(diagnostics.get("selected")) or selected
    )
    diagnostics.setdefault("fallbackUsed", _same_coordinate(selected, fallback))
    for key in (
        "candidateCount",
        "demSampleCount",
        "coverFraction",
        "uavLosClear",
        "uavLosMarginM",
        "uavDistanceM",
        "uavDistanceFeasible",
    ):
        diagnostics.setdefault(key, None)
    info = _point_mission_info(selected)
    info["_lahTerminalCoverEnabled"] = True
    info["_lahConstraintAreaList"] = constraints
    info["_lahTerminalCoverThreatCoordinateList"] = deepcopy(threats)
    info["_lahTerminalCoverFallbackCoordinate"] = dict(fallback)
    info["_lahTerminalCoverDiagnostics"] = diagnostics
    return info


def _attack_wait_hold_mission_info(
    profile: object | None,
    fallback_coordinate: object | None = None,
) -> Optional[Dict[str, Any]]:
    """Hold inside the 공격대기지역, masked from the 목표지역 where possible.

    Mirrors the battle-position hold: the anchor is the area centroid, which is
    an arbitrary spot that can sit fully exposed to the objective.  When the
    area geometry is known the terrain selector moves it to a covered point
    inside the same area; without geometry the previous centroid is kept.
    """

    source = profile if isinstance(profile, dict) else {}
    fallback = _normalize_coordinate(fallback_coordinate) or _normalize_coordinate(
        source.get("attackWaitCoordinate")
    )
    if fallback is None:
        return None
    area_rows = source.get("attackWaitAreaList")
    if isinstance(area_rows, list) and area_rows:
        return _terminal_cover_point_mission_info(
            fallback,
            area_rows=area_rows,
            threat_coordinates=source.get("targetAreaCoordinateList"),
        )
    return _point_mission_info(fallback)


def _battle_hold_mission_info(
    context: object | None,
    *,
    fallback_coordinate: object | None = None,
) -> Optional[Dict[str, Any]]:
    source = context if isinstance(context, dict) else {}
    fallback = _normalize_coordinate(fallback_coordinate) or _normalize_coordinate(
        source.get("battleAttackCoordinate") or source.get("battlePositionCoordinate")
    )
    if fallback is None:
        return None
    area_rows = source.get("battleAreaList")
    if not isinstance(area_rows, list):
        area_rows = source.get("battlePositionAreaList")
    if isinstance(area_rows, list) and area_rows:
        return _terminal_cover_point_mission_info(
            fallback,
            area_rows=area_rows,
            threat_coordinates=source.get("targetAreaCoordinateList"),
        )
    return _point_mission_info(fallback)


def _line_mission_info(
    mission: Dict[str, Any],
    *,
    preserve_endpoints: bool = False,
) -> Optional[Dict[str, Any]]:
    coords = _first_line_coords(mission)
    if not coords:
        return None
    info = {
        "individualMissionType": 7,
        "patternType": 10,
        "autoZoomIn": False,
        "coordinateList": deepcopy(coords),
        "targetID": None,
    }
    line_rows = _constraint_line_rows(mission)
    if line_rows:
        # d0304 needs the original width; coordinateList alone is insufficient
        # and previously enabled the unconstrained 1,200 m terrain corridor.
        info["lineList"] = line_rows
    if preserve_endpoints:
        info["_lahPreserveLineEndpoints"] = True
    return info


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
    info = {
        "individualMissionType": 7,
        "patternType": 10,
        "autoZoomIn": False,
        "coordinateList": deepcopy(coords),
        "targetID": None,
    }
    line_rows: List[Dict[str, Any]] = []
    for idx in mission_indices:
        if 0 <= idx < len(missions):
            line_rows.extend(_constraint_line_rows(missions[idx]))
    if line_rows:
        info["lineList"] = line_rows
    return info


def _generic_lah_info_from_input(mission: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    mission_type = _to_int(mission.get("inputMissionType")) or 0
    if mission_type in (1, 7):
        return _line_mission_info(mission)
    if mission_type in (2, 3, 4, 5, 6):
        anchor = _area_anchor(mission)
        if anchor is None:
            return None
        area_rows = _constraint_area_rows(mission)
        region_type = _to_int(mission.get("regionType")) or 0
        if area_rows and region_type in (REGION_BATTLE_POSITION, REGION_TARGET):
            fallback = (
                _battle_attack_anchor(mission)
                if region_type == REGION_BATTLE_POSITION
                else None
            ) or anchor
            detail = _mission_detail(mission)
            explicit_threats = _normalize_coord_list(
                detail.get("targetAreaCoordinateList")
                or detail.get("threatCoordinateList")
            )
            threat_coords = (
                _first_area_coords(mission)
                if region_type == REGION_TARGET
                else explicit_threats
            )
            return _terminal_cover_point_mission_info(
                fallback,
                area_rows=area_rows,
                threat_coordinates=threat_coords,
            )

        info = _point_mission_info(anchor)
        if area_rows:
            # Keep this private so a point/hold mission does not accidentally
            # enter AREA capstone handling while d0304 can still constrain it.
            info["_lahConstraintAreaList"] = area_rows
        return info
    return None


def lah_special_info_for_input(
    input_plan_or_missions: object | None,
    input_mission_id: int,
    *,
    planning_mode: object | None = None,
) -> Optional[Dict[str, Any]]:
    missions = _mission_list(input_plan_or_missions)
    if not missions:
        return None
    profile = detect_lah_special_operation(missions, planning_mode=planning_mode)
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


def _anti_armor_lah_info_for_index(
    missions: List[Dict[str, Any]],
    profile: Dict[str, Any],
    mission_index: int,
) -> Tuple[Optional[Dict[str, Any]], Optional[int], str]:
    def _id_at(idx: int) -> int:
        return _mission_id(missions[idx], idx + 1)

    attack_wait_coord = _normalize_coordinate(profile.get("attackWaitCoordinate"))
    battle_coord = _normalize_coordinate(
        profile.get("battleAttackCoordinate") or profile.get("battlePositionCoordinate")
    )
    if attack_wait_coord is None or battle_coord is None:
        return None, None, "invalid"
    cycles = profile.get("targetBattleCycles")
    cycle_rows = [row for row in cycles if isinstance(row, dict)] if isinstance(cycles, list) else []
    battle_context = cycle_rows[0] if cycle_rows else profile

    if mission_index == 0:
        start_coord = _line_anchor(missions[0]) or _area_anchor(missions[0])
        if start_coord is None:
            return None, None, "initial_hold_missing"
        return _point_mission_info(start_coord), _id_at(0), "initial_hold"

    if mission_index in (1, 2):
        acp1_coord = _line_end_anchor(missions[0]) or _line_anchor(missions[1])
        if acp1_coord is None:
            return None, None, "acp1_hold_missing"
        return _point_mission_info(acp1_coord), _id_at(0), "acp1_hold"

    if mission_index in (3, 4):
        return _attack_wait_hold_mission_info(profile, attack_wait_coord), int(
            profile.get("attackWaitInputMissionID") or 0
        ), "attack_wait_hold"

    if mission_index in (5, 6):
        return _battle_hold_mission_info(
            battle_context,
            fallback_coordinate=battle_coord,
        ), int(
            profile.get("battlePositionInputMissionID") or 0
        ), "battle_position_hold"

    if mission_index == 7:
        # This Region-5 line describes the UAV/mission flow from the target
        # back to the battle position.  LAH is already at the battle position;
        # assigning the line makes it fly the line backwards toward the target.
        # Keep LAH at the battle position until the later egress line begins.
        return _battle_hold_mission_info(
            battle_context,
            fallback_coordinate=battle_coord,
        ), int(
            profile.get("battlePositionInputMissionID") or 0
        ), "target_to_battle_battle_hold"

    if mission_index == 8:
        info = _line_mission_info(missions[mission_index], preserve_endpoints=True)
        if info is not None:
            return info, _id_at(mission_index), "battle_to_acp_follow"

    if mission_index == 9:
        info = _line_mission_info(missions[mission_index], preserve_endpoints=True)
        if info is not None:
            return info, _id_at(mission_index), "acp_to_control_follow"

    source = missions[mission_index - 1] if mission_index > 0 else missions[mission_index]
    return _generic_lah_info_from_input(source), _mission_id(source, mission_index), "anti_armor_fallback"


def _anti_armor_refresh_lah_info_for_index(
    missions: List[Dict[str, Any]],
    profile: Dict[str, Any],
    mission_index: int,
) -> Tuple[Optional[Dict[str, Any]], Optional[int], str]:
    """LAH hold/follow sequence for the additive second Type-1 target cycle."""

    def _id_at(idx: int) -> int:
        return _mission_id(missions[idx], idx + 1)

    attack_wait_coord = _normalize_coordinate(profile.get("attackWaitCoordinate"))
    cycles = profile.get("targetBattleCycles")
    cycle_rows = [row for row in cycles if isinstance(row, dict)] if isinstance(cycles, list) else []
    old_battle_coord = _normalize_coordinate(
        cycle_rows[0].get("battleAttackCoordinate") if cycle_rows else None
    ) or _normalize_coordinate(profile.get("battleAttackCoordinate") or profile.get("battlePositionCoordinate"))
    new_battle_coord = _normalize_coordinate(
        cycle_rows[1].get("battleAttackCoordinate") if len(cycle_rows) >= 2 else None
    )
    if attack_wait_coord is None or old_battle_coord is None or new_battle_coord is None:
        return None, None, "invalid"

    if mission_index == 0:
        start_coord = _line_anchor(missions[0]) or _area_anchor(missions[0])
        if start_coord is None:
            return None, None, "initial_hold_missing"
        return _point_mission_info(start_coord), _id_at(0), "initial_hold"

    if mission_index in (1, 2):
        acp1_coord = _line_end_anchor(missions[0]) or _line_anchor(missions[1])
        if acp1_coord is None:
            return None, None, "acp1_hold_missing"
        return _point_mission_info(acp1_coord), _id_at(0), "acp1_hold"

    if mission_index in (3, 4, 9, 10):
        return _attack_wait_hold_mission_info(profile, attack_wait_coord), int(
            profile.get("attackWaitInputMissionID") or 0
        ), "attack_wait_hold"

    if mission_index in (5, 6):
        return _battle_hold_mission_info(
            cycle_rows[0],
            fallback_coordinate=old_battle_coord,
        ), int(
            cycle_rows[0].get("battlePositionInputMissionID") or 0
        ), "battle_position_hold"

    if mission_index in (11, 12):
        return _battle_hold_mission_info(
            cycle_rows[1],
            fallback_coordinate=new_battle_coord,
        ), int(
            cycle_rows[1].get("battlePositionInputMissionID") or 0
        ), "battle_position_hold"

    if mission_index in (7, 13):
        battle_hold_coord = old_battle_coord if mission_index == 7 else new_battle_coord
        cycle_index = 0 if mission_index == 7 else 1
        battle_input_id = (
            _to_int(cycle_rows[cycle_index].get("battlePositionInputMissionID"))
            if len(cycle_rows) > cycle_index
            else None
        )
        return _battle_hold_mission_info(
            cycle_rows[cycle_index] if len(cycle_rows) > cycle_index else profile,
            fallback_coordinate=battle_hold_coord,
        ), int(
            battle_input_id or profile.get("battlePositionInputMissionID") or 0
        ), "target_to_battle_battle_hold"

    follow_behaviors = {
        8: "battle_to_attack_wait_follow",
        14: "battle_to_acp_follow",
        15: "acp_to_control_follow",
    }
    behavior = follow_behaviors.get(int(mission_index))
    if behavior is not None:
        info = _line_mission_info(missions[mission_index], preserve_endpoints=True)
        if info is not None:
            return info, _id_at(mission_index), behavior

    source = missions[mission_index]
    return _generic_lah_info_from_input(source), _mission_id(source, mission_index + 1), "anti_armor_refresh_fallback"


def _cycle_battle_hold_for_index(
    missions: List[Dict[str, Any]],
    profile: Dict[str, Any],
    mission_index: int,
) -> Optional[Tuple[Dict[str, Any], int, str]]:
    cycles = profile.get("targetBattleCycles")
    cycle_rows = [row for row in cycles if isinstance(row, dict)] if isinstance(cycles, list) else []
    for cycle in cycle_rows:
        battle_index = _to_int(cycle.get("battlePositionIndex"))
        target_index = _to_int(cycle.get("targetIndex"))
        battle_coord = _normalize_coordinate(cycle.get("battleAttackCoordinate"))
        battle_input_id = _to_int(cycle.get("battlePositionInputMissionID"))
        if (
            battle_index is None
            or target_index is None
            or battle_coord is None
            or battle_input_id is None
        ):
            continue
        if int(battle_index) < int(mission_index) <= int(target_index):
            info = _battle_hold_mission_info(cycle, fallback_coordinate=battle_coord)
            if info is None:
                continue
            return (
                info,
                int(battle_input_id),
                "battle_position_hold",
            )
        if int(mission_index) <= int(target_index):
            continue
        mission = missions[int(mission_index)]
        is_target_return_line = (
            _to_int(mission.get("inputMissionType")) == 1
            and _to_int(mission.get("regionType")) == REGION_BATTLE_POSITION
        )
        if not is_target_return_line:
            continue
        # Only the contiguous Region-5 line(s) directly following this target
        # are target->battle return corridors.  A later Region-5 line after an
        # attack-wait/ACP line is an approach to another battle position.
        between = missions[int(target_index) + 1 : int(mission_index)]
        if any(
            _to_int(row.get("inputMissionType")) != 1
            or _to_int(row.get("regionType")) != REGION_BATTLE_POSITION
            for row in between
        ):
            continue
        info = _battle_hold_mission_info(cycle, fallback_coordinate=battle_coord)
        if info is None:
            continue
        return (
            info,
            int(battle_input_id),
            "target_to_battle_battle_hold",
        )
    return None


def lah_special_info_for_index(
    missions_or_plan: object | None,
    profile: Dict[str, Any],
    mission_index: int,
) -> Tuple[Optional[Dict[str, Any]], Optional[int], str]:
    missions = _mission_list(missions_or_plan)
    if not missions or mission_index < 0 or mission_index >= len(missions):
        return None, None, "invalid"
    cycle_hold = _cycle_battle_hold_for_index(missions, profile, int(mission_index))
    if cycle_hold is not None:
        return cycle_hold
    profile_mode = str(profile.get("mode") or "")
    if profile_mode == "anti_armor_air_strike_refresh":
        return _anti_armor_refresh_lah_info_for_index(missions, profile, int(mission_index))
    if profile_mode == "anti_armor_air_strike_review":
        return _anti_armor_lah_info_for_index(missions, profile, int(mission_index))

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
        return _point_mission_info(start_coord), _mission_id(missions[0], 1), "initial_hold"

    if mission_index <= attack_wait_idx:
        source = missions[mission_index - 1]
        return _generic_lah_info_from_input(source), _mission_id(source, mission_index), "lag_follow"

    if mission_index == attack_wait_idx + 1:
        return _attack_wait_hold_mission_info(profile, attack_wait_coord), int(
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
        return _battle_hold_mission_info(
            profile,
            fallback_coordinate=battle_coord,
        ), int(
            profile.get("battlePositionInputMissionID") or 0
        ), "battle_position_hold"

    source = missions[mission_index - 1]
    return _generic_lah_info_from_input(source), _mission_id(source, mission_index), "post_target_lag_follow"


def build_lah_special_sequence(
    input_plan_or_missions: object | None,
    *,
    planning_mode: object | None = None,
) -> Optional[List[Dict[str, Any]]]:
    missions = _mission_list(input_plan_or_missions)
    profile = detect_lah_special_operation(missions, planning_mode=planning_mode)
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


def _special_target_battle_cycles(profile: object | None) -> List[Dict[str, Any]]:
    if not isinstance(profile, dict):
        return []
    rows = profile.get("targetBattleCycles")
    if not isinstance(rows, list):
        return []
    return [row for row in rows if isinstance(row, dict)]


def _special_cycle_for_input(
    profile: object | None,
    input_mission_id: object | None,
) -> Optional[Dict[str, Any]]:
    current_id = _to_int(input_mission_id)
    if current_id is None:
        return None
    for cycle in _special_target_battle_cycles(profile):
        phase_ids = {
            int(value)
            for value in (cycle.get("attackInputMissionIDs") or [])
            if _to_int(value) is not None
        }
        target_id = _to_int(cycle.get("targetInputMissionID"))
        if int(current_id) in phase_ids or (
            target_id is not None and int(current_id) == int(target_id)
        ):
            return cycle
    return None


def _special_cycle_for_target(
    profile: object | None,
    target_coord: object | None,
) -> Optional[Dict[str, Any]]:
    for cycle in _special_target_battle_cycles(profile):
        if _point_in_polygon(target_coord, cycle.get("targetAreaCoordinateList")):
            return cycle
    return None


def special_attack_coordinate(
    profile: object | None,
    *,
    input_mission_id: object | None = None,
    target_coord: object | None = None,
) -> Optional[Dict[str, Any]]:
    if not isinstance(profile, dict):
        return None
    cycle = _special_cycle_for_input(profile, input_mission_id)
    if cycle is None:
        cycle = _special_cycle_for_target(profile, target_coord)
    if cycle is not None:
        coord = _normalize_coordinate(cycle.get("battleAttackCoordinate"))
        if coord is not None:
            return coord
    return _normalize_coordinate(profile.get("battleAttackCoordinate") or profile.get("battlePositionCoordinate"))


def special_target_contains_coordinate(profile: object | None, coord: object | None) -> bool:
    if not isinstance(profile, dict):
        return False
    cycles = _special_target_battle_cycles(profile)
    if cycles:
        return any(
            _point_in_polygon(coord, cycle.get("targetAreaCoordinateList"))
            for cycle in cycles
        )
    return _point_in_polygon(coord, profile.get("targetAreaCoordinateList"))


def special_force_battle_attack(profile: object | None, input_mission_id: object | None) -> bool:
    if not isinstance(profile, dict):
        return False
    cycles = _special_target_battle_cycles(profile)
    if cycles:
        return _special_cycle_for_input(profile, input_mission_id) is not None
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
