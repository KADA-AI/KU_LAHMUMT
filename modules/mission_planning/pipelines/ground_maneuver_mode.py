from __future__ import annotations

"""Collaborative branch/escort planning for Type 2 and Type 3.

Two mission packages share the same "각자도생" (self-reliance) branch structure
and are handled here together; the code distinguishes them by inputMissionPackageType
and by regionType so their type-specific pieces never get mixed up:

  * Type 2 지상작전부대 기동여건 보장 작전 - escort destination is 목표지역(regionType=6).
  * Type 3 공중강습작전부대 엄호 작전 - adds a 탑재지대(regionType=8) front stage and
    the escort destination is 착륙지대(regionType=9) instead of 목표지역; the final
    통제권변경 additionally hands UAV control to ACSGCS.

Both use the incoming 0201 as-is (no MSM review / no new package ID). Shared
branch behaviour:

  * Up to the 경계지역 phase the missions are ordinary collaborative base
    missions (all UAVs share one line / one area, width-split as usual).
  * At the first 경계지역(regionType=7) mission whose lineList/areaList holds
    N elements (Type 2: N>=1, Type 3: N>=2), the mission stops being pooled.
    Each list element becomes a *branch* owned end-to-end by a fixed UAV group.
    A group has one UAV normally and multiple UAVs only when UAVs outnumber
    branches. Type-2 ownership is fixed from the first plan; Type 3 retains its
    legacy entry-LINE re-anchor. In both cases the selected map is reused for the
    following set members (경계 areaList, then 목표/착륙 lineList).
  * After the branch span the flow re-converges: the later single-destination
    ACP / 통제권변경 missions are travelled independently by each UAV from its own
    position (no width split), not re-pooled.

This module only *describes* the structure (detection + per-branch geometry +
ownership seeding + manned sequence/attack anchors). The actual splitting/assignment
lives in the split pipeline, and the sticky map is persisted by
:mod:`modules.mission_planning.runtime.state.branch_ownership`.
"""

import math
import threading
from collections import OrderedDict
from copy import deepcopy
from typing import Any, Dict, List, Optional, Tuple


PACKAGE_TYPE_GROUND_MANEUVER = 2  # 지상작전부대 기동여건 보장 (목표지역)
PACKAGE_TYPE_AIR_ASSAULT = 3  # 공중강습작전부대 엄호 (탑재지대/착륙지대)
PACKAGE_TYPE_FACILITY_PROTECTION = 4  # 항공지원-중요시설 방호 (도넛 경계, 공격대기 hold)
PACKAGE_TYPE_URBAN = 5  # 도시지역 작전 (공격대기지역 hold, 각자도생 없음)
# 공격대기지역에서 대기하는 staged-hold 유인기 시퀀스를 공유하는 패키지들.
STAGED_ATTACK_WAIT_PACKAGE_TYPES = (PACKAGE_TYPE_FACILITY_PROTECTION, PACKAGE_TYPE_URBAN)
BRANCH_PACKAGE_TYPES = (PACKAGE_TYPE_GROUND_MANEUVER, PACKAGE_TYPE_AIR_ASSAULT)

REGION_CONTROL_HANDOVER = 2  # 통제권변경지역
REGION_ACP = 3
REGION_ATTACK_WAIT = 4  # 공격대기지역 (type 5 manned hold)
REGION_TARGET = 6  # 목표지역 (type 2 escort destination)
REGION_GUARD = 7  # 경계지역 (branch anchor)
REGION_LOADING = 8  # 탑재지대 (type 3 front stage)
REGION_LANDING = 9  # 착륙지대 (type 3 escort destination)
REGION_URBAN = 11  # 도시지역 (type 5 recon/attack region)
# 목표지역(2)/착륙지대(3): the escort destination + manned target-hold region.
DESTINATION_REGIONS = (REGION_TARGET, REGION_LANDING)

# 협업기동임무. Its line is the leg the manned aircraft trails along while the
# UAVs work the region ahead of it.
MANEUVER_MISSION_TYPE = 1

# Geometry-bearing inputMissionType groups (shared with the split pipeline).
# 엄호(4 공중부대/5 지상부대) missions carry areaList per the confirmed type→shape
# contract: line=(1,7), area=(2,3,4,5,6).
_LINE_MISSION_TYPES = (1, 7)
_AREA_MISSION_TYPES = (2, 3, 4, 5, 6)

TYPE2_SELF_RELIANCE_OUTBOUND_LINE = "outbound_line"
TYPE2_SELF_RELIANCE_GUARD_AREA = "guard_area"
TYPE2_SELF_RELIANCE_RETURN_LINE = "return_line"


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
    out: Dict[str, Any] = {"latitude": float(lat), "longitude": float(lon)}
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
    lat_vals = [float(c["latitude"]) for c in coords]
    lon_vals = [float(c["longitude"]) for c in coords]
    out: Dict[str, Any] = {
        "latitude": sum(lat_vals) / float(len(lat_vals)),
        "longitude": sum(lon_vals) / float(len(lon_vals)),
    }
    alt_vals = [float(c["altitude"]) for c in coords if c.get("altitude") is not None]
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


def _package_type(input_plan_or_missions: object | None, package_type: Any = None) -> int:
    resolved = _to_int(package_type)
    if resolved is not None and resolved > 0:
        return resolved
    if isinstance(input_plan_or_missions, dict):
        for key in ("inputMissionPackageType", "InputMissionPackageType", "packageType"):
            value = _to_int(input_plan_or_missions.get(key))
            if value is not None:
                return value
    return 0


def _package_id(input_plan_or_missions: object | None) -> Optional[int]:
    if isinstance(input_plan_or_missions, dict):
        for key in ("inputMissionPackageID", "InputMissionPackageID", "inputMissionPackageId"):
            value = _to_int(input_plan_or_missions.get(key))
            if value is not None and value > 0:
                return value
    return None


def _mission_detail(mission: Dict[str, Any]) -> Dict[str, Any]:
    detail = mission.get("missionDetail")
    return detail if isinstance(detail, dict) else {}


def _mission_geometry_kind(mission: Dict[str, Any]) -> str:
    mission_type = _to_int(mission.get("inputMissionType")) or 0
    if mission_type in _AREA_MISSION_TYPES:
        return "area"
    if mission_type in _LINE_MISSION_TYPES:
        return "line"
    return "unknown"


def mission_branch_geometries(mission: Dict[str, Any]) -> List[Dict[str, Any]]:
    """Return the per-branch geometry for a mission, one entry per list element.

    Each entry: ``{"index", "kind", "coordinateList", "entry", "width"}`` where
    ``entry`` is the branch's representative approach point (line start / area
    centroid). Holes and degenerate elements are skipped so the branch count
    reflects genuinely plannable pieces.
    """
    detail = _mission_detail(mission)
    kind = _mission_geometry_kind(mission)
    out: List[Dict[str, Any]] = []

    if kind == "line":
        line_list = detail.get("lineList") if isinstance(detail.get("lineList"), list) else []
        for line in line_list:
            if not isinstance(line, dict):
                continue
            coords = _normalize_coord_list(line.get("coordinateList"))
            if len(coords) < 2:
                continue
            out.append(
                {
                    "index": len(out),
                    "kind": "line",
                    "coordinateList": coords,
                    "entry": dict(coords[0]),
                    "exit": dict(coords[-1]),
                    "width": _to_float(line.get("width")) or 0.0,
                }
            )
    elif kind == "area":
        area_list = detail.get("areaList") if isinstance(detail.get("areaList"), list) else []
        for area in area_list:
            if not isinstance(area, dict) or bool(area.get("isHole")):
                continue
            coords = _normalize_coord_list(area.get("coordinateList"))
            if len(coords) < 3:
                continue
            centroid = _centroid_coordinate(coords)
            out.append(
                {
                    "index": len(out),
                    "kind": "area",
                    "coordinateList": coords,
                    "entry": centroid or dict(coords[0]),
                    "exit": centroid or dict(coords[0]),
                    "width": 0.0,
                }
            )
    return out


def mission_branch_count(mission: Dict[str, Any]) -> int:
    return len(mission_branch_geometries(mission))


def detect_ground_maneuver_profile(
    input_plan_or_missions: object | None,
    *,
    package_type: Any = None,
) -> Optional[Dict[str, Any]]:
    """Detect the Type-2 각자도생 branch span, or return None if not applicable.

    Returns a profile describing which missions form the branch span, the branch
    count N, and each branch mission's geometry. ``None`` when the package is not
    Type 2/3 or has no applicable branch 경계지역 phase (a plain collaborative scenario
    needs no special handling and plans like any other package).
    """
    resolved_package_type = _package_type(input_plan_or_missions, package_type)
    if resolved_package_type not in BRANCH_PACKAGE_TYPES:
        return None

    missions = _mission_list(input_plan_or_missions)
    if not missions:
        return None

    any_region_type = any((_to_int(m.get("regionType")) or 0) > 0 for m in missions)

    # Type 2 may have fewer branch areas than UAVs, including one area shared
    # by every UAV. It still needs a durable owner set across every replan.
    minimum_branch_count = 1 if resolved_package_type == PACKAGE_TYPE_GROUND_MANEUVER else 2

    def _is_anchor(mission: Dict[str, Any]) -> bool:
        if mission_branch_count(mission) < minimum_branch_count:
            return False
        region = _to_int(mission.get("regionType")) or 0
        if any_region_type:
            return region == REGION_GUARD
        # regionType absent in this feed: fall back to first multi-branch mission.
        return True

    anchor_order: Optional[int] = None
    for order, mission in enumerate(missions, start=1):
        if _is_anchor(mission):
            anchor_order = order
            break
    if anchor_order is None:
        return None

    branch_count = mission_branch_count(missions[anchor_order - 1])
    if branch_count < minimum_branch_count:
        return None

    # Region-aware packages have one explicit branch set: guard entry LINE,
    # guard AREA, then the destination return LINE. Stopping at that return LINE
    # is essential for N=1, because the following ACP/control-transfer lines also
    # contain one geometry but are not part of the self-reliance assignment.
    branch_orders: List[int] = []
    if any_region_type:
        destination_region = (
            REGION_TARGET
            if resolved_package_type == PACKAGE_TYPE_GROUND_MANEUVER
            else REGION_LANDING
        )
        for order in range(anchor_order, len(missions) + 1):
            mission = missions[order - 1]
            if mission_branch_count(mission) != branch_count:
                break
            region = _to_int(mission.get("regionType")) or 0
            if region == REGION_GUARD:
                branch_orders.append(order)
                continue
            if (
                region == destination_region
                and _mission_geometry_kind(mission) == "line"
                and branch_orders
            ):
                branch_orders.append(order)
            break
    else:
        # Legacy feeds without regionType retain count-based detection.
        for order in range(anchor_order, len(missions) + 1):
            mission = missions[order - 1]
            if mission_branch_count(mission) != branch_count:
                break
            branch_orders.append(order)

    missions_by_order: Dict[int, Dict[str, Any]] = {}
    for order in branch_orders:
        mission = missions[order - 1]
        missions_by_order[order] = {
            "inputMissionID": _to_int(mission.get("inputMissionID")) or order,
            "inputMissionType": _to_int(mission.get("inputMissionType")) or 0,
            "regionType": _to_int(mission.get("regionType")) or 0,
            "kind": _mission_geometry_kind(mission),
            "branches": mission_branch_geometries(mission),
        }

    anchor_mission = missions[anchor_order - 1]
    return {
        "mode": "ground_maneuver_support",
        "packageType": int(resolved_package_type),
        "packageID": _package_id(input_plan_or_missions),
        "branchCount": int(branch_count),
        "anchorOrder": int(anchor_order),
        "anchorInputMissionID": _to_int(anchor_mission.get("inputMissionID")) or int(anchor_order),
        "branchOrders": [int(o) for o in branch_orders],
        "branchInputMissionIDs": [
            int(missions_by_order[o]["inputMissionID"]) for o in branch_orders
        ],
        "regionAware": bool(any_region_type),
        "missionsByOrder": missions_by_order,
    }


def resolve_type2_self_reliance_phase(
    input_plan_or_missions: object | None,
    input_mission_id: Any,
) -> Optional[str]:
    """Return the exact Type-2 self-reliance phase for one input mission.

    This is intentionally stricter than :func:`detect_ground_maneuver_profile`.
    The detector also supports Type 3 and legacy feeds without ``regionType``;
    callers that alter a UAV's individual suffix must instead prove that the
    *current* Type-2 input package still has the complete region-aware sequence::

        guard LINE -> guard AREA -> target return LINE

    The three missions must be consecutive and carry the same non-zero branch
    count.  Any stale, partial, legacy, or malformed package returns ``None`` so
    ordinary collaborative LINE/AREA handling remains untouched.
    """
    mission_id = _to_int(input_mission_id)
    if mission_id is None:
        return None
    if _package_type(input_plan_or_missions) != PACKAGE_TYPE_GROUND_MANEUVER:
        return None

    profile = detect_ground_maneuver_profile(input_plan_or_missions)
    if not isinstance(profile, dict):
        return None
    if int(_to_int(profile.get("packageType")) or 0) != PACKAGE_TYPE_GROUND_MANEUVER:
        return None
    if profile.get("regionAware") is not True:
        return None

    raw_orders = profile.get("branchOrders")
    if not isinstance(raw_orders, list) or len(raw_orders) != 3:
        return None
    orders = [_to_int(order) for order in raw_orders]
    if any(order is None for order in orders):
        return None
    resolved_orders = [int(order) for order in orders if order is not None]
    if resolved_orders != list(range(resolved_orders[0], resolved_orders[0] + 3)):
        return None

    missions_by_order = profile.get("missionsByOrder")
    if not isinstance(missions_by_order, dict):
        return None

    entries: List[Dict[str, Any]] = []
    for order in resolved_orders:
        entry = missions_by_order.get(order) or missions_by_order.get(str(order))
        if not isinstance(entry, dict):
            return None
        entries.append(entry)

    expected_signature = (
        (REGION_GUARD, "line", TYPE2_SELF_RELIANCE_OUTBOUND_LINE),
        (REGION_GUARD, "area", TYPE2_SELF_RELIANCE_GUARD_AREA),
        (REGION_TARGET, "line", TYPE2_SELF_RELIANCE_RETURN_LINE),
    )
    expected_branch_count = _to_int(profile.get("branchCount")) or 0
    if expected_branch_count <= 0:
        return None

    matched_phase: Optional[str] = None
    for entry, (expected_region, expected_kind, phase) in zip(entries, expected_signature):
        if (_to_int(entry.get("regionType")) or 0) != expected_region:
            return None
        if str(entry.get("kind") or "").strip().lower() != expected_kind:
            return None
        branches = entry.get("branches")
        if not isinstance(branches, list) or len(branches) != expected_branch_count:
            return None
        if (_to_int(entry.get("inputMissionID")) or 0) == mission_id:
            matched_phase = phase

    return matched_phase


def is_branch_mission_order(profile: object | None, parent_order: Any) -> bool:
    if not isinstance(profile, dict):
        return False
    order = _to_int(parent_order)
    if order is None:
        return False
    return int(order) in {int(o) for o in profile.get("branchOrders") or []}


def profile_branch_geometries(profile: object | None, parent_order: Any) -> List[Dict[str, Any]]:
    if not isinstance(profile, dict):
        return []
    order = _to_int(parent_order)
    if order is None:
        return []
    entry = (profile.get("missionsByOrder") or {}).get(int(order)) or (
        profile.get("missionsByOrder") or {}
    ).get(str(int(order)))
    if not isinstance(entry, dict):
        return []
    branches = entry.get("branches")
    return branches if isinstance(branches, list) else []


# ---------------------------------------------------------------------------
# 유인기(LAH) sequence (Type 2 & Type 3)
#
# The manned aircraft do not run the 각자도생 branch split; they hold / escort /
# follow along the operation's shared anchors, resolved from regionType so the
# two package types stay distinct:
#
#   Type 2 지상기동 (8 missions):
#     1     통제권변경 출발점 대기   2,3   ACP#1 대기   4,5 경계 목표지역 엄호이동
#     6 목표 대기   7 ACP#2 추종   8 통제권변경 추종
#   Type 3 공중강습 (9 missions):
#     1 탑재지대 이동   2 탑재지대 대기   3,4 ACP#1 대기   5,6 경계 착륙지역 엄호이동
#     7 착륙지역 대기   8 ACP#2 추종   9 통제권변경 추종 (+ 도착 후 ACSGCS 통제권변경)
#
# Unified rule: 탑재지대(8)=move-to; up to ACP#1 reached=staging hold; pre-경계=ACP#1
# hold; 경계(7)=escort acp1->destination; 목표/착륙(6/9)=destination hold; later
# ACP(3)/통제권(2)=follow. Command (LAH 1) vs wingmen (LAH 2,3) only matters for
# the attack trigger; the base 0302 sequence is uniform hold/move points.
# ---------------------------------------------------------------------------

LAH_COMMAND_AIRCRAFT_ID = 1
LAH_WINGMAN_AIRCRAFT_IDS = (2, 3)


def _line_start(mission: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    for branch in mission_branch_geometries(mission):
        if branch.get("kind") == "line":
            return dict(branch["coordinateList"][0])
    return None


def _line_end(mission: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    for branch in mission_branch_geometries(mission):
        if branch.get("kind") == "line":
            return dict(branch["coordinateList"][-1])
    return None


def _mission_anchor(mission: Dict[str, Any]) -> Optional[Dict[str, Any]]:
    branches = mission_branch_geometries(mission)
    if not branches:
        return None
    return dict(branches[0].get("entry") or {})


def _polyline_midpoint(coords: List[Dict[str, Any]]) -> Optional[Dict[str, Any]]:
    """Midpoint measured along the polyline, not the midpoint of its endpoints."""

    points = [c for c in coords if c.get("latitude") is not None and c.get("longitude") is not None]
    if len(points) < 2:
        return dict(points[0]) if points else None
    spans: List[float] = []
    for left, right in zip(points, points[1:]):
        # Planar is fine here: a corridor is short relative to the earth.
        d_lat = float(right["latitude"]) - float(left["latitude"])
        d_lon = float(right["longitude"]) - float(left["longitude"])
        spans.append(math.hypot(d_lat, d_lon))
    total = sum(spans)
    if total <= 0.0:
        return dict(points[0])
    walked = 0.0
    for index, span in enumerate(spans):
        if walked + span >= total * 0.5:
            remain = (total * 0.5) - walked
            fraction = remain / span if span > 0.0 else 0.0
            return _interp_coord(points[index], points[index + 1], fraction)
        walked += span
    return dict(points[-1])


def _region_missions(missions: List[Dict[str, Any]], region: int) -> List[Dict[str, Any]]:
    return [m for m in missions if (_to_int(m.get("regionType")) or 0) == int(region)]


def _first_branch_of_kind(
    missions: List[Dict[str, Any]], region: int, kind: str
) -> Optional[Dict[str, Any]]:
    for mission in _region_missions(missions, region):
        for branch in mission_branch_geometries(mission):
            if branch.get("kind") == kind:
                return branch
    return None


def _interp_coord(a: Dict[str, Any], b: Dict[str, Any], t: float) -> Dict[str, Any]:
    t = min(max(float(t), 0.0), 1.0)
    out = {
        "latitude": float(a["latitude"]) + (float(b["latitude"]) - float(a["latitude"])) * t,
        "longitude": float(a["longitude"]) + (float(b["longitude"]) - float(a["longitude"])) * t,
    }
    az = _to_float(a.get("altitude"))
    bz = _to_float(b.get("altitude"))
    if az is not None and bz is not None:
        out["altitude"] = int(round(az + (bz - az) * t))
    elif az is not None:
        out["altitude"] = int(round(az))
    return out


def _previous_maneuver_midpoint(
    missions: List[Dict[str, Any]], order: int
) -> Optional[Dict[str, Any]]:
    """Midpoint of the newest 협업기동임무 line strictly before ``order``.

    The manned aircraft trails the UAVs by one leg: while they work a region it
    sits half way along the maneuver leg they have just finished, which keeps it
    behind the forward edge instead of inside the region being worked.
    """

    for index in range(int(order) - 2, -1, -1):
        mission = missions[index]
        if (_to_int(mission.get("inputMissionType")) or 0) != MANEUVER_MISSION_TYPE:
            continue
        for branch in mission_branch_geometries(mission):
            if branch.get("kind") != "line":
                continue
            midpoint = _polyline_midpoint(list(branch.get("coordinateList") or []))
            if midpoint:
                return midpoint
    return None


def _lah_point_info(coord: Dict[str, Any]) -> Dict[str, Any]:
    return {
        "individualMissionType": 9,
        "patternType": 12,
        "autoZoomIn": False,
        "coordinateList": [dict(coord)],
        "targetID": None,
    }


# ---------------------------------------------------------------------------
# 지형 차폐 대기점: the geometric ladder anchor (previous-line midpoint,
# corridor point, area centroid) stays the authority on roughly WHERE the
# manned aircraft waits; the DEM cover selector then slides that point onto
# nearby low ground that puts a ridge between the aircraft and the mission the
# UAVs are currently working.  Every failure path returns the plain anchor.
# ---------------------------------------------------------------------------

_COVER_HOLD_CACHE: "OrderedDict[tuple, Dict[str, Any]]" = OrderedDict()
_COVER_HOLD_CACHE_LOCK = threading.Lock()
_COVER_HOLD_CACHE_MAX = 64
# The cover search may slide the hold sideways - or slightly forward onto the
# back slope of a ridge - but the ladder's "one leg behind" meaning has to
# survive: never advance past this fraction of the anchor's threat distance.
_COVER_HOLD_MIN_THREAT_RATIO = 0.6
_COVER_HOLD_MAX_THREATS = 5


def _cover_hold_enabled() -> bool:
    try:
        from modules.mission_planning.MissionPlanner.runtime_settings import (
            get_runtime_attack_int,
        )

        return int(get_runtime_attack_int("lah_ladder_cover_enabled", 1)) != 0
    except Exception:
        return True


def _cover_hold_search_radius_m() -> float:
    try:
        from modules.mission_planning.MissionPlanner.runtime_settings import (
            get_runtime_attack_float,
        )

        value = float(get_runtime_attack_float("lah_cover_search_radius_m", 1500.0))
    except Exception:
        return 1500.0
    return value if math.isfinite(value) and value > 0.0 else 0.0


def _equirect_distance_m(a: Dict[str, Any], b: Dict[str, Any]) -> float:
    mid = math.radians((float(a["latitude"]) + float(b["latitude"])) * 0.5)
    dx = (float(b["longitude"]) - float(a["longitude"])) * 111_132.92 * math.cos(mid)
    dy = (float(b["latitude"]) - float(a["latitude"])) * 111_132.92
    return math.hypot(dx, dy)


def _mission_threat_coordinates(
    mission: Optional[Dict[str, Any]],
    maximum: int = _COVER_HOLD_MAX_THREATS,
) -> List[Dict[str, Any]]:
    """Deterministic sample of a mission's geometry as threat points.

    The mission the UAVs are working is what the manned aircraft hides from:
    its centroid plus an even spread of its vertices, capped so the selector's
    ray budget stays bounded.
    """

    if not isinstance(mission, dict):
        return []
    coords: List[Dict[str, Any]] = []
    branches = mission_branch_geometries(mission)
    for wanted in ("area", "line"):
        for branch in branches:
            if branch.get("kind") == wanted:
                coords = list(branch.get("coordinateList") or [])
                break
        if coords:
            break
    if not coords:
        return []
    threats: List[Dict[str, Any]] = []
    centroid = _centroid_coordinate(coords)
    if centroid:
        threats.append(dict(centroid))
    picks = min(max(0, int(maximum) - len(threats)), len(coords))
    if picks:
        last_index = len(coords) - 1
        chosen: set[int] = set()
        for pick in range(picks):
            index = (
                int(round(pick * last_index / max(1, picks - 1))) if picks > 1 else 0
            )
            if index in chosen:
                continue
            chosen.add(index)
            threats.append(dict(coords[index]))
    return threats[: max(1, int(maximum))]


def _destination_area_rows(missions: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    """Raw 목표지역 AREA rows (holes included) for containment-mode cover."""

    for mission in missions:
        if (_to_int(mission.get("regionType")) or 0) != REGION_TARGET:
            continue
        if _mission_geometry_kind(mission) != "area":
            continue
        detail = _mission_detail(mission)
        rows = detail.get("areaList") if isinstance(detail.get("areaList"), list) else []
        out: List[Dict[str, Any]] = []
        for row in rows:
            if not isinstance(row, dict):
                continue
            coords = _normalize_coord_list(row.get("coordinateList"))
            if len(coords) < 3:
                continue
            out.append(
                {"coordinateList": coords, "isHole": bool(row.get("isHole"))}
            )
        if out:
            return out
    return []


def _cover_hold_cache_key(
    anchor: Dict[str, Any],
    threats: List[Dict[str, Any]],
    constraints: List[Dict[str, Any]],
    radius_m: float,
) -> tuple:
    def _coord_key(coord: object) -> tuple:
        if not isinstance(coord, dict):
            return ()
        try:
            return (
                round(float(coord.get("latitude")), 7),
                round(float(coord.get("longitude")), 7),
            )
        except Exception:
            return ()

    return (
        _coord_key(anchor),
        tuple(_coord_key(coord) for coord in threats),
        tuple(
            (
                bool(row.get("isHole")),
                tuple(_coord_key(coord) for coord in row.get("coordinateList") or []),
            )
            for row in constraints
        ),
        round(float(radius_m), 1),
    )


def _lah_cover_hold_info(
    anchor: Optional[Dict[str, Any]],
    threat_mission: Optional[Dict[str, Any]],
    *,
    area_rows: Optional[List[Dict[str, Any]]] = None,
) -> Dict[str, Any]:
    """Hold on masking terrain near ``anchor``, backing away from the mission."""

    if not isinstance(anchor, dict):
        return _lah_point_info(anchor or {})
    base = _lah_point_info(anchor)
    if not _cover_hold_enabled():
        return base
    threats = _mission_threat_coordinates(threat_mission)
    if not threats:
        return base
    constraints = [row for row in (area_rows or []) if isinstance(row, dict)]
    radius_m = _cover_hold_search_radius_m()
    if radius_m <= 0.0:
        return base

    cache_key = _cover_hold_cache_key(anchor, threats, constraints, radius_m)
    with _COVER_HOLD_CACHE_LOCK:
        cached = _COVER_HOLD_CACHE.get(cache_key)
        if cached is not None:
            _COVER_HOLD_CACHE.move_to_end(cache_key)
            return deepcopy(cached)

    try:
        from modules.mission_planning.MissionPlanner.data_def.lah_terminal_cover import (
            select_lah_terminal_cover_point,
        )

        selected, _diagnostics = select_lah_terminal_cover_point(
            constraints,
            dict(anchor),
            threat_coordinates=[dict(threat) for threat in threats],
            max_candidates=25,
            max_ray_samples=48,
            search_radius_m=radius_m,
        )
    except Exception:
        return base
    coord = _normalize_coordinate(selected)
    if coord is None:
        return base

    # A ridge's back slope may sit slightly toward the mission; that is fine.
    # Sliding well past the anchor toward the mission is not - the aircraft
    # would no longer be "one leg behind", cover or not.
    reference = _centroid_coordinate(threats) or threats[0]
    try:
        anchor_distance_m = _equirect_distance_m(anchor, reference)
        if (
            anchor_distance_m > 1.0
            and _equirect_distance_m(coord, reference)
            < anchor_distance_m * _COVER_HOLD_MIN_THREAT_RATIO
        ):
            coord = dict(anchor)
    except Exception:
        coord = dict(anchor)

    info = _lah_point_info(coord)
    if constraints:
        # AREA-contained holds keep the terminal-cover contract keys so the
        # d0304 pass can re-refine the point against UAV ETA LOS later.
        info["_lahTerminalCoverEnabled"] = True
        info["_lahConstraintAreaList"] = deepcopy(constraints)
        info["_lahTerminalCoverThreatCoordinateList"] = [dict(t) for t in threats]
        info["_lahTerminalCoverFallbackCoordinate"] = dict(anchor)

    with _COVER_HOLD_CACHE_LOCK:
        _COVER_HOLD_CACHE[cache_key] = deepcopy(info)
        _COVER_HOLD_CACHE.move_to_end(cache_key)
        while len(_COVER_HOLD_CACHE) > int(_COVER_HOLD_CACHE_MAX):
            _COVER_HOLD_CACHE.popitem(last=False)
    return info


def _lah_terminal_transit_info(
    coordinates: List[Dict[str, Any]],
    *,
    constraint_missions: List[Dict[str, Any]],
) -> Dict[str, Any]:
    """LAH route: prior-line midpoint -> ACP -> final control endpoint."""

    route: List[Dict[str, Any]] = []
    for coordinate in coordinates:
        normalized = _normalize_coordinate(coordinate)
        if normalized is None:
            continue
        if route:
            same_lat = abs(float(route[-1]["latitude"]) - float(normalized["latitude"])) <= 1e-10
            same_lon = abs(float(route[-1]["longitude"]) - float(normalized["longitude"])) <= 1e-10
            if same_lat and same_lon:
                continue
        route.append(normalized)

    info: Dict[str, Any] = {
        "individualMissionType": 7,
        "patternType": 10,
        "autoZoomIn": False,
        "coordinateList": route,
        "targetID": None,
        "_lahPreserveLineEndpoints": True,
    }
    constraint_lines: List[Dict[str, Any]] = []
    for mission in constraint_missions:
        for branch in mission_branch_geometries(mission):
            if branch.get("kind") != "line":
                continue
            width_m = _to_float(branch.get("width")) or 0.0
            if width_m <= 0.0:
                continue
            constraint_lines.append(
                {
                    "width": max(1, min(50_000, int(round(width_m)))),
                    "coordinateList": [
                        dict(coord) for coord in (branch.get("coordinateList") or [])
                    ],
                }
            )
    if constraint_lines:
        info["_lahConstraintLineList"] = constraint_lines
    return info


def resolve_ground_maneuver_lah_anchors(
    input_plan_or_missions: object | None,
) -> Optional[Dict[str, Any]]:
    """Resolve the shared LAH hold/move anchors from mission regionTypes.

    Returns ``None`` when the flow lacks the ACP or destination(목표/착륙)
    references needed to place the manned aircraft (an incomplete scenario falls
    back to the generic per-mission LAH builder).
    """
    missions = _mission_list(input_plan_or_missions)
    if not missions:
        return None

    def _orders(*regions: int) -> List[int]:
        wanted = set(regions)
        return [o for o, m in enumerate(missions, 1) if (_to_int(m.get("regionType")) or 0) in wanted]

    acp_orders = _orders(REGION_ACP)
    target_orders = _orders(*DESTINATION_REGIONS)  # 목표(type2) / 착륙(type3)
    guard_orders = _orders(REGION_GUARD)
    control_orders = _orders(REGION_CONTROL_HANDOVER)
    loading_orders = _orders(REGION_LOADING)  # 탑재지대 (type 3 only)

    acp1_order = acp_orders[0] if acp_orders else None
    acp1 = _line_end(missions[acp1_order - 1]) if acp1_order else None
    acp2 = _line_end(missions[acp_orders[-1] - 1]) if len(acp_orders) >= 2 else None
    # 목표/착륙 anchor = convergence of destination line endpoints (user-confirmed).
    target_ends = [e for o in target_orders if (e := _line_end(missions[o - 1])) is not None]
    target = _centroid_coordinate(target_ends) if target_ends else None
    # Staging point held until ACP#1 is reached: 탑재지대(type3) else 통제권변경
    # 출발점(=mission1 line start, type2).
    loading = _line_end(missions[loading_orders[0] - 1]) if loading_orders else None
    staging = loading or _line_start(missions[0]) or _mission_anchor(missions[0])
    control_end = _line_end(missions[control_orders[-1] - 1]) if control_orders else _line_end(missions[-1])
    if acp1 is None or target is None:
        return None

    # 목표지역 corridor (통로) and the 목표지역 itself are both regionType=6, told
    # apart by geometry: the corridor is the line mission, the region is the area
    # mission.  The manned aircraft walks the corridor before the guard phase and
    # then stays inside the region for the rest of it.
    corridor = _first_branch_of_kind(missions, REGION_TARGET, "line")
    corridor_coords = list(corridor.get("coordinateList") or []) if corridor else []
    corridor_start = dict(corridor_coords[0]) if corridor_coords else None
    corridor_mid = _polyline_midpoint(corridor_coords) if corridor_coords else None
    destination_area = _first_branch_of_kind(missions, REGION_TARGET, "area")
    destination_inside = (
        dict(destination_area.get("entry") or {}) if destination_area else None
    ) or None

    return {
        "staging": staging,
        "loading": loading,
        "acp1": acp1,
        "acp2": acp2 or acp1,
        "target": target,
        "controlEnd": control_end or target,
        # 통로 시작점 / 통로 중앙점: where the manned aircraft waits while the
        # UAVs work the 목표지역, before the guard phase begins.
        "corridorStart": corridor_start,
        "corridorMid": corridor_mid,
        # A point inside the 목표지역 polygon; the guard-phase hold anchor.
        "destinationInside": destination_inside,
        "anchorGuardOrder": guard_orders[0] if guard_orders else None,
        "guardOrders": guard_orders,
        "acp1Order": acp1_order,
    }


def ground_maneuver_lah_info_for_index(
    missions: List[Dict[str, Any]],
    anchors: Dict[str, Any],
    mission_index: int,
    *,
    package_type: int = PACKAGE_TYPE_GROUND_MANEUVER,
) -> Tuple[Optional[Dict[str, Any]], str]:
    """Return ``(individualMissionInfo, behavior)`` for one LAH mission slot."""
    if mission_index < 0 or mission_index >= len(missions):
        return None, "invalid"
    order = mission_index + 1
    mission = missions[mission_index]
    region = _to_int(mission.get("regionType")) or 0
    guard_start = anchors.get("anchorGuardOrder")
    guard_orders = list(anchors.get("guardOrders") or [])
    acp1_order = anchors.get("acp1Order")
    before_guard = guard_start is None or order < int(guard_start)
    # Type 3 flies to a 착륙지대 rather than holding a 목표지역, so it keeps the
    # ACP#2 / 착륙 convergence egress below.
    trails_maneuver_legs = int(package_type) == PACKAGE_TYPE_GROUND_MANEUVER

    # Type 3 탑재지대 front: manned aircraft move to the loading zone.
    if region == REGION_LOADING:
        return _lah_point_info(anchors.get("loading") or anchors["staging"]), "loading_move"

    # Until ACP#1 is actually reached (the mission after the ACP#1 mission), the
    # manned aircraft stays at the staging point: 탑재지대(type3) / 통제권변경
    # 출발점(type2). This covers type2 mission 1 and type3 missions 1-2.
    if acp1_order is not None and order <= int(acp1_order):
        return _lah_point_info(anchors["staging"]), "staging_hold"

    if before_guard:
        # Pre-경계.  The manned aircraft trails one leg behind: while the UAVs
        # work a 목표지역 it holds half way along the 협업기동임무 leg they have
        # just flown, rather than moving up into the region being worked.
        if region in DESTINATION_REGIONS:
            if trails_maneuver_legs:
                previous_mid = _previous_maneuver_midpoint(missions, order)
                if previous_mid:
                    return (
                        _lah_cover_hold_info(previous_mid, mission),
                        "previous_maneuver_mid_hold",
                    )
            target_orders_before_guard = [
                o
                for o, m in enumerate(missions, 1)
                if (_to_int(m.get("regionType")) or 0) in DESTINATION_REGIONS
                and (guard_start is None or o < int(guard_start))
            ]
            corridor_start = anchors.get("corridorStart")
            corridor_mid = anchors.get("corridorMid")
            is_first_destination = bool(
                target_orders_before_guard and order == target_orders_before_guard[0]
            )
            if is_first_destination and corridor_start:
                return (
                    _lah_cover_hold_info(corridor_start, mission),
                    "corridor_start_hold",
                )
            if not is_first_destination and corridor_mid:
                return (
                    _lah_cover_hold_info(corridor_mid, mission),
                    "corridor_mid_hold",
                )
        # No corridor geometry (or a non-destination pre-guard mission): keep the
        # previous ACP#1 hold.
        return _lah_point_info(anchors["acp1"]), "acp1_hold"

    if region == REGION_GUARD:
        # 경계: hold inside the 목표지역 for the whole guard phase.  The earlier
        # equal-fraction ACP#1->목표 march left the manned aircraft short of the
        # region while the guard lines were being flown.
        destination_inside = anchors.get("destinationInside")
        if destination_inside:
            return (
                _lah_cover_hold_info(
                    destination_inside,
                    mission,
                    area_rows=_destination_area_rows(missions),
                ),
                "destination_area_hold",
            )
        try:
            k = guard_orders.index(order)
        except ValueError:
            k = 0
        frac = (k + 1) / float(len(guard_orders) + 1)
        point = _interp_coord(anchors["acp1"], anchors["target"], frac)
        return _lah_point_info(point), "escort_move_to_destination"

    # Post-경계.  Type 2 keeps the manned aircraft inside the 목표지역 for the
    # rest of the region work, including the ACP mission where only the UAVs
    # move on; it only leaves once the 통제권변경 leg begins.
    if trails_maneuver_legs and region in (REGION_TARGET, REGION_ACP):
        destination_inside = anchors.get("destinationInside")
        if destination_inside:
            return (
                _lah_cover_hold_info(
                    destination_inside,
                    mission,
                    area_rows=_destination_area_rows(missions),
                ),
                "destination_area_hold",
            )

    if region in DESTINATION_REGIONS:
        # 목표(type2) / 착륙(type3) hold.
        return _lah_point_info(anchors["target"]), "destination_hold"

    if region == REGION_ACP:
        # Destination hold then follow UAV rear to ACP#2.
        return _lah_point_info(anchors["acp2"]), "destination_to_acp2_follow"

    if region == REGION_CONTROL_HANDOVER:
        # During the final mission LAH catches up in order instead of cutting
        # diagonally from its destination hold to the control endpoint:
        # previous maneuver-line midpoint -> ACP#2 -> control-transfer end.
        previous_mid = _previous_maneuver_midpoint(missions, order)
        route = [
            coordinate
            for coordinate in (
                previous_mid,
                anchors.get("acp2"),
                anchors.get("controlEnd"),
            )
            if isinstance(coordinate, dict)
        ]
        if route:
            previous_mission = missions[mission_index - 1] if mission_index > 0 else mission
            return _lah_terminal_transit_info(
                route,
                constraint_missions=[previous_mission, mission],
            ), "previous_mid_to_acp2_to_control_end_follow"
        return _lah_point_info(anchors["controlEnd"]), "acp2_to_control_end_follow"

    # Any other trailing mission: sit at the last known convergence (destination).
    return _lah_point_info(anchors["target"]), "hold"


def ground_maneuver_lah_info_for_input(
    input_plan_or_missions: object | None,
    input_mission_id: Any,
    *,
    package_type: Any = None,
) -> Optional[Dict[str, Any]]:
    """The Type 2/3 manned hold for one ``inputMissionID``.

    Replans rebuild the manned row from the UAVs' replacement geometry, which
    collapses an area mission to its centroid and drops the manned aircraft into
    the middle of the region the UAVs are working.  Resolving the package ladder
    here lets a replan keep the hold the initial plan chose.

    Returns ``None`` for non-Type-2/3 packages or an unknown mission, so callers
    fall through to their existing geometry.
    """

    pkg = _package_type(input_plan_or_missions, package_type)
    if pkg not in BRANCH_PACKAGE_TYPES:
        return None
    wanted = _to_int(input_mission_id)
    if wanted is None:
        return None
    missions = _mission_list(input_plan_or_missions)
    if not missions:
        return None
    anchors = resolve_ground_maneuver_lah_anchors(missions)
    if anchors is None:
        return None
    for index, mission in enumerate(missions):
        if (_to_int(mission.get("inputMissionID")) or 0) != int(wanted):
            continue
        info, _behavior = ground_maneuver_lah_info_for_index(
            missions, anchors, index, package_type=int(pkg)
        )
        return info
    return None


def build_ground_maneuver_lah_sequence(
    input_plan_or_missions: object | None,
    *,
    planning_mode: object | None = None,
    package_type: Any = None,
) -> Optional[List[Dict[str, Any]]]:
    """Build the Type 2/3 LAH per-mission hold/move sequence (uniform for 1,2,3).

    Returns ``None`` for non-Type-2/3 packages or when anchors can't be resolved,
    so the 0302 builder falls through to its generic per-mission LAH logic.
    """
    _ = planning_mode
    pkg = _package_type(input_plan_or_missions, package_type)
    if pkg not in BRANCH_PACKAGE_TYPES:
        return None
    missions = _mission_list(input_plan_or_missions)
    if not missions:
        return None
    anchors = resolve_ground_maneuver_lah_anchors(missions)
    if anchors is None:
        return None

    rows: List[Dict[str, Any]] = []
    for idx, mission in enumerate(missions):
        info, behavior = ground_maneuver_lah_info_for_index(
            missions, anchors, idx, package_type=int(pkg)
        )
        if info is None:
            continue
        input_mid = _to_int(mission.get("inputMissionID")) or (idx + 1)
        rows.append(
            {
                "inputMissionID": int(input_mid),
                "sourceInputMissionID": int(input_mid),
                "behavior": str(behavior),
                "individualMissionInfo": info,
            }
        )
    return rows or None


# ---------------------------------------------------------------------------
# Type 5 도시지역 작전 유인기(LAH) sequence
#
# No 각자도생 branch phase - the UAV side plans as a plain collaborative package.
# Manned aircraft never enter the 도시지역(regionType=11); they stage at the
# 공격대기지역(regionType=4) and follow out. Spec (7 missions):
#   1 공격대기(4) line   유인기 통제권변경 출발점 대기
#   2 공격대기(4) area   유인기 출발점 대기 지속
#   3 도시(11) line      유인기 공격대기지역으로 이동 후 대기
#   4 도시(11) area      유인기 공격대기지역 대기 (표적 발견 시 편대기 접근공격/복귀는
#                        기존 공격 재계획 기본 동작)
#   5 공격대기(4) line   유인기 공격대기지역 대기
#   6 ACP(3) line        유인기 공격대기지역 대기 후 무인기 후미 추종 ACP#2
#   7 통제권변경(2) line 유인기 ACP#2 대기 후 추종, 도착 후 ACSGCS 통제권변경
# Rule: before the first 도시 mission = start hold; from it (until ACP/통제권) =
# 공격대기지역 hold; ACP(3) = ACP#2 follow; 통제권변경(2) = control-end follow.
# ---------------------------------------------------------------------------


def resolve_urban_operation_lah_anchors(
    input_plan_or_missions: object | None,
) -> Optional[Dict[str, Any]]:
    """Resolve the type-5 LAH anchors (start / 공격대기지역 / ACP#2 / 통제권 종점).

    ``None`` when the flow has no 도시지역(11) or 공격대기지역(4) reference, so
    the 0302 builder falls through to the generic per-mission LAH logic.
    """
    missions = _mission_list(input_plan_or_missions)
    if not missions:
        return None

    def _orders(region: int) -> List[int]:
        return [o for o, m in enumerate(missions, 1) if (_to_int(m.get("regionType")) or 0) == int(region)]

    # Trigger = first mission of the operation region: 도시(11) for type 5,
    # 경계(7) for type 4 (중요시설 방호 donut boundary).
    urban_orders = sorted(_orders(REGION_URBAN) + _orders(REGION_GUARD))
    attack_wait_orders = _orders(REGION_ATTACK_WAIT)
    acp_orders = _orders(REGION_ACP)
    control_orders = _orders(REGION_CONTROL_HANDOVER)
    if not urban_orders or not attack_wait_orders:
        return None

    start = _line_start(missions[0]) or _mission_anchor(missions[0])
    # 공격대기지역 anchor: prefer the region-4 AREA mission centroid (the actual
    # staging polygon), else the first region-4 line's arrival endpoint.
    attack_wait: Optional[Dict[str, Any]] = None
    for order in attack_wait_orders:
        mission = missions[order - 1]
        if _mission_geometry_kind(mission) == "area":
            attack_wait = _mission_anchor(mission)
            if attack_wait:
                break
    if not attack_wait:
        attack_wait = _line_end(missions[attack_wait_orders[0] - 1])
    acp2 = _line_end(missions[acp_orders[-1] - 1]) if acp_orders else None
    control_end = _line_end(missions[control_orders[-1] - 1]) if control_orders else _line_end(missions[-1])
    if start is None or attack_wait is None:
        return None
    return {
        "start": start,
        "attackWait": attack_wait,
        "acp2": acp2 or attack_wait,
        "controlEnd": control_end or (acp2 or attack_wait),
        "firstUrbanOrder": int(urban_orders[0]),
    }


def urban_operation_lah_info_for_index(
    missions: List[Dict[str, Any]],
    anchors: Dict[str, Any],
    mission_index: int,
) -> Tuple[Optional[Dict[str, Any]], str]:
    """Return ``(individualMissionInfo, behavior)`` for one type-5 LAH slot."""
    if mission_index < 0 or mission_index >= len(missions):
        return None, "invalid"
    order = mission_index + 1
    region = _to_int(missions[mission_index].get("regionType")) or 0
    first_urban = int(anchors.get("firstUrbanOrder") or 0)

    if first_urban > 0 and order < first_urban:
        return _lah_point_info(anchors["start"]), "control_start_hold"
    if region == REGION_ACP:
        return _lah_point_info(anchors["acp2"]), "attack_wait_to_acp2_follow"
    if region == REGION_CONTROL_HANDOVER:
        # 도착 후 ACSGCS 통제권변경 (message-level handover handled separately).
        return _lah_point_info(anchors["controlEnd"]), "acp2_to_control_follow"
    return _lah_point_info(anchors["attackWait"]), "attack_wait_hold"


def build_urban_operation_lah_sequence(
    input_plan_or_missions: object | None,
    *,
    planning_mode: object | None = None,
    package_type: Any = None,
) -> Optional[List[Dict[str, Any]]]:
    """Build the Type-4/5 LAH per-mission hold/follow sequence (uniform for 1,2,3).

    Type 4 (중요시설 방호) and Type 5 (도시지역) share the staged attack-wait
    manned flow: start hold -> 공격대기지역 hold from the first 경계/도시 mission
    -> ACP#2 follow -> 통제권변경 follow. ``None`` for other packages or
    unresolved anchors (generic fallback).
    """
    _ = planning_mode
    if _package_type(input_plan_or_missions, package_type) not in STAGED_ATTACK_WAIT_PACKAGE_TYPES:
        return None
    missions = _mission_list(input_plan_or_missions)
    if not missions:
        return None
    anchors = resolve_urban_operation_lah_anchors(missions)
    if anchors is None:
        return None

    rows: List[Dict[str, Any]] = []
    for idx, mission in enumerate(missions):
        info, behavior = urban_operation_lah_info_for_index(missions, anchors, idx)
        if info is None:
            continue
        input_mid = _to_int(mission.get("inputMissionID")) or (idx + 1)
        rows.append(
            {
                "inputMissionID": int(input_mid),
                "sourceInputMissionID": int(input_mid),
                "behavior": str(behavior),
                "individualMissionInfo": info,
            }
        )
    return rows or None


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
        xi, yi = float(current["longitude"]), float(current["latitude"])
        xj, yj = float(previous["longitude"]), float(previous["latitude"])
        if (yi > y) != (yj > y):
            x_intersect = (xj - xi) * (y - yi) / (yj - yi) + xi
            if x <= x_intersect:
                inside = not inside
        previous = current
    return bool(inside)


def detect_ground_maneuver_attack_profile(
    input_plan_or_missions: object | None,
    *,
    package_type: Any = None,
) -> Optional[Dict[str, Any]]:
    """Type 2/3 attack profile: destination anchor + phase gate + command/wingman roles.

    The manned attack is anchored inside the escort destination (목표지역 type2 /
    착륙지대 type3) only while the manned aircraft is holding there (the destination
    hold mission that follows the 경계 branch phase); before/after it, the
    per-position default attack applies. Returns ``None`` for non-Type-2/3 packages
    or when the destination hold mission can't be resolved.
    """
    if _package_type(input_plan_or_missions, package_type) not in BRANCH_PACKAGE_TYPES:
        return None
    missions = _mission_list(input_plan_or_missions)
    if not missions:
        return None
    anchors = resolve_ground_maneuver_lah_anchors(missions)
    if anchors is None:
        return None

    guard_orders = list(anchors.get("guardOrders") or [])
    last_guard_order = guard_orders[-1] if guard_orders else 0
    # Destination hold mission = first 목표/착륙(regionType 6/9) after the 경계 phase.
    target_hold_order: Optional[int] = None
    for order, mission in enumerate(missions, 1):
        if order <= last_guard_order:
            continue
        if (_to_int(mission.get("regionType")) or 0) in DESTINATION_REGIONS:
            target_hold_order = order
            break
    if target_hold_order is None:
        return None

    target_mission = missions[target_hold_order - 1]
    target_hold_id = _to_int(target_mission.get("inputMissionID")) or target_hold_order
    # 목표지역 polygon for containment: prefer an explicit 목표 area, else fall
    # back to the convergence point (containment then just checks proximity=None).
    target_area = _first_area_coords(target_mission)
    return {
        "mode": "ground_maneuver_support",
        "packageID": _package_id(input_plan_or_missions),
        "targetCoordinate": anchors["target"],
        "targetHoldInputMissionID": int(target_hold_id),
        "targetAreaCoordinateList": target_area,
        "commandAircraftID": LAH_COMMAND_AIRCRAFT_ID,
        "wingmanAircraftIDs": list(LAH_WINGMAN_AIRCRAFT_IDS),
    }


def _first_area_coords(mission: Dict[str, Any]) -> List[Dict[str, Any]]:
    detail = _mission_detail(mission)
    area_list = detail.get("areaList") if isinstance(detail.get("areaList"), list) else []
    for area in area_list:
        if not isinstance(area, dict) or bool(area.get("isHole")):
            continue
        coords = _normalize_coord_list(area.get("coordinateList"))
        if len(coords) >= 3:
            return coords
    return []


def ground_maneuver_target_attack_anchor(
    profile: object | None,
    input_mission_id: object | None,
    aircraft_id: object | None = None,
) -> Optional[Dict[str, Any]]:
    """Return the 목표지역 attack anchor when the manned aircraft is at 목표.

    ``None`` outside the 목표 hold window (so the caller keeps its per-position
    default). Only manned aircraft (1,2,3) are anchored.
    """
    if not isinstance(profile, dict):
        return None
    aid = _to_int(aircraft_id)
    if aid is not None and aid not in (1, 2, 3):
        return None
    current = _to_int(input_mission_id)
    hold_id = _to_int(profile.get("targetHoldInputMissionID"))
    if current is None or hold_id is None or int(current) != int(hold_id):
        return None
    return _normalize_coordinate(profile.get("targetCoordinate"))


def ground_maneuver_target_contains(profile: object | None, coord: object | None) -> bool:
    if not isinstance(profile, dict):
        return False
    return _point_in_polygon(coord, profile.get("targetAreaCoordinateList"))


def distribute_uavs_to_branches(
    branch_count: int,
    uav_ids: List[int],
) -> Dict[int, List[int]]:
    """Map ``branch_index -> [aircraftID, ...]`` for the ownership anchor.

    * UAVs == branches: clean 1:1.
    * UAVs  > branches: 1:1 first, then spare UAVs join the earliest branches so
      those branches are sub-divided (spec: extra UAVs split a side).
    * UAVs  < branches: each UAV owns one branch by order; the remaining branches
      are folded onto UAVs round-robin so every branch still has an owner.

    ``uav_ids`` is assumed already ordered by whatever geometric preference the
    caller wants (e.g. takeover proximity); this function only decides counts.
    """
    n = max(0, int(branch_count))
    ids = [int(a) for a in uav_ids if _to_int(a) is not None]
    ownership: Dict[int, List[int]] = {idx: [] for idx in range(n)}
    if n == 0 or not ids:
        return ownership

    if len(ids) >= n:
        for idx in range(n):
            ownership[idx].append(ids[idx])
        # Spare UAVs subdivide branches, filling from the first branch onward.
        for offset, aid in enumerate(ids[n:]):
            ownership[offset % n].append(aid)
    else:
        for idx, aid in enumerate(ids):
            ownership[idx].append(aid)
        for idx in range(len(ids), n):
            ownership[idx].append(ids[idx % len(ids)])
    return ownership
