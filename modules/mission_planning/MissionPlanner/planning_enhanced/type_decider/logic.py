from __future__ import annotations

from collections import Counter
from typing import Any, Dict, List, Optional, Set

from ..algo.split_algorithms import mission_geometry
from ..models import SplitPiece, SplitRunResult
from modules.mission_planning.planning_modes import (
    MissionPlanningMode,
    mission_mode_context,
    resolve_mission_planning_mode,
)


PROFILE_DEFAULT = 6
# Legacy profile names remain available for compatibility, but general mission
# planning now resolves to the single base profile only.
PROFILE_RECON = PROFILE_DEFAULT
PROFILE_MIN_TIME = PROFILE_DEFAULT
VALID_PROFILE_CODES = {PROFILE_DEFAULT}
# Type 5 도시지역 작전 also follows the base mtype rules (no special patterns);
# its 협업수색공격(2)/협업도심수색공격(6) areas resolve like other area recon.
BASE_PACKAGE_TYPES = {1, 2, 3, 5}


def _to_int(v: Any, default: int = 0) -> int:
    try:
        return int(v)
    except Exception:
        return default


def _is_line_piece(piece: SplitPiece) -> bool:
    data = piece.data if isinstance(piece.data, dict) else {}
    center = data.get("Centerline")
    if isinstance(center, list) and len(center) >= 2:
        return True
    return int(piece.mission_type) in (1, 7)


def _is_area_piece(piece: SplitPiece) -> bool:
    data = piece.data if isinstance(piece.data, dict) else {}
    coords = data.get("coordinateList")
    if isinstance(coords, list) and len(coords) >= 3 and int(piece.mission_type) in (2, 3, 4, 5, 6):
        return True
    return False


def _is_coordinate_detail(mission: Dict[str, Any]) -> bool:
    # Replanning may mirror the trimmed LINE coordinates into coordinateList
    # while keeping the authoritative lineList.  Follow the same shape
    # precedence as the splitter so that this derived field cannot turn a LINE
    # piece into a type-5 coordinate route.
    return mission_geometry(mission) == "coordinate"


def _pick_area_pattern(mission_type: int, profile_code: int, piece_index: int = 1) -> int:
    _ = mission_type
    _ = profile_code
    _ = piece_index
    return 6


def _extract_aircraft_id(entry: Any) -> Optional[int]:
    if entry is None:
        return None
    if isinstance(entry, dict):
        for key in ("aircraftID", "aircraftId", "id"):
            if key in entry:
                entry = entry.get(key)
                break
    if isinstance(entry, int):
        return entry
    if isinstance(entry, str):
        token = entry.strip().upper()
        if token.startswith(("UAV", "LAH")):
            token = "".join(ch for ch in token if ch.isdigit())
        if not token:
            return None
        try:
            return int(token)
        except ValueError:
            return None
    try:
        return int(entry)
    except Exception:
        return None


def _available_aircraft_ids(cmpk: Dict[str, Any]) -> List[int]:
    raw = cmpk.get("availableAircraftList")
    if not isinstance(raw, list):
        return []
    out: List[int] = []
    for item in raw:
        aid = _extract_aircraft_id(item)
        if aid is None:
            continue
        if 4 <= aid <= 6:
            out.append(aid)
    return sorted(set(out))


def _priority_aircraft_ids(cmpk: Dict[str, Any], split_result: SplitRunResult) -> List[int]:
    from_0201 = _available_aircraft_ids(cmpk)
    from_run: List[int] = []
    for x in split_result.uav_ids:
        try:
            xi = int(x)
        except Exception:
            continue
        if xi > 0:
            from_run.append(xi)
    from_run = sorted(set(from_run))

    if from_0201 and from_run:
        run_set: Set[int] = set(from_run)
        inter = [x for x in from_0201 if x in run_set]
        if inter:
            return inter
    if from_0201:
        return from_0201
    if from_run:
        return from_run
    return [4, 5, 6]


def _resolve_type7_leader_piece_id(pieces: List[SplitPiece], leader_aircraft_id: Optional[int]) -> int:
    if not pieces:
        return -1
    if leader_aircraft_id is not None:
        for p in pieces:
            try:
                if int(p.assigned_uav or 0) == int(leader_aircraft_id):
                    return int(p.piece_index)
            except Exception:
                continue
    first = min(pieces, key=lambda p: int(p.piece_index))
    return int(first.piece_index)


def _normalize_coord_list(coords: Any) -> List[Dict[str, float]]:
    out: List[Dict[str, float]] = []
    if not isinstance(coords, list):
        return out
    for p in coords:
        if not isinstance(p, dict):
            continue
        if "latitude" not in p or "longitude" not in p:
            continue
        out.append(
            {
                "latitude": float(p.get("latitude", 0.0)),
                "longitude": float(p.get("longitude", 0.0)),
                "altitude": float(p.get("altitude", 0.0)),
            }
        )
    return out


def _ensure_coordinate_piece_data(piece: SplitPiece, mission: Dict[str, Any]) -> None:
    data = piece.data if isinstance(piece.data, dict) else {}
    if not isinstance(data, dict):
        piece.data = {}
        data = piece.data

    coords = _normalize_coord_list(data.get("coordinateList"))
    if len(coords) < 2:
        detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else {}
        coords = _normalize_coord_list(detail.get("coordinateList"))
    if len(coords) >= 2:
        data["coordinateList"] = coords
        if "Centerline" not in data or not isinstance(data.get("Centerline"), list):
            data["Centerline"] = coords
        data["fromCoordinateList"] = True


def apply_logic_type_decider(
    split_result: SplitRunResult,
    cmpk: Dict[str, Any],
    profile_code: int = PROFILE_DEFAULT,
    planning_mode: MissionPlanningMode | Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    missions = cmpk.get("inputMissionList")
    if not isinstance(missions, list):
        raise ValueError("0201 payload has no valid 'inputMissionList'.")
    resolved_mode = planning_mode or getattr(split_result, "planning_mode", None) or resolve_mission_planning_mode(cmpk)
    mode_context = mission_mode_context(mode=resolved_mode)
    package_type = _to_int(mode_context.get("package_type"), 0)
    if package_type <= 0:
        package_type = _to_int(cmpk.get("inputMissionPackageType"), 0)
    profile_code = _to_int(profile_code, PROFILE_DEFAULT)
    if profile_code not in VALID_PROFILE_CODES:
        profile_code = PROFILE_DEFAULT
    priority_uavs = _priority_aircraft_ids(cmpk, split_result)
    leader_aircraft_id = int(priority_uavs[0]) if priority_uavs else None

    mission_by_order: Dict[int, Dict[str, Any]] = {}
    for i, m in enumerate(missions, start=1):
        if isinstance(m, dict):
            mission_by_order[i] = m

    by_parent: Dict[int, List[SplitPiece]] = {}
    for p in split_result.pieces:
        by_parent.setdefault(int(p.parent_order), []).append(p)

    changed = 0
    pattern_counter: Counter[int] = Counter()
    imtype_counter: Counter[int] = Counter()
    warnings: List[str] = []
    assignments: List[Dict[str, Any]] = []

    for parent_order, pieces in sorted(by_parent.items()):
        mission = mission_by_order.get(parent_order, {})
        input_mission_type = _to_int(mission.get("inputMissionType"), 0)
        coord_mode = _is_coordinate_detail(mission)

        # Type7 leader/follower split.
        sorted_pieces = sorted(pieces, key=lambda p: int(p.piece_index))
        leader_piece_id = _resolve_type7_leader_piece_id(sorted_pieces, leader_aircraft_id)

        for piece in sorted_pieces:
            data = piece.data if isinstance(piece.data, dict) else {}
            if not isinstance(data, dict):
                piece.data = {}
                data = piece.data

            geometry = "line" if _is_line_piece(piece) else ("area" if _is_area_piece(piece) else "unknown")
            im_type = None
            pattern_type = None
            rule = ""

            # Coordinate reconnaissance applies only when coordinateList is the
            # mission's resolved geometry (no valid lineList/areaList).
            if coord_mode:
                im_type = 5
                pattern_type = 1
                rule = "coordinateList>=2 -> type5/pattern1"
                _ensure_coordinate_piece_data(piece, mission)
            elif package_type in BASE_PACKAGE_TYPES or package_type == 4:
                if input_mission_type in (1, 2):
                    if geometry == "line":
                        im_type, pattern_type, rule = 6, 8, f"pkg{package_type} mtype1/2 line"
                    elif geometry == "area":
                        im_type = 3
                        pattern_type = _pick_area_pattern(input_mission_type, profile_code, int(piece.piece_index))
                        rule = f"pkg{package_type} mtype1/2 area"
                elif input_mission_type == 3:
                    if geometry == "area":
                        im_type = 4
                        if package_type == 4:
                            pattern_type = 6
                            rule = "pkg4 mtype3 area-fixed"
                        else:
                            pattern_type = _pick_area_pattern(input_mission_type, profile_code, int(piece.piece_index))
                            rule = f"pkg{package_type} mtype3 area"
                    elif geometry == "line":
                        im_type, pattern_type, rule = 6, 8, f"pkg{package_type} mtype3 line-fallback"
                        warnings.append(
                            f"M{parent_order}-{piece.piece_index}: inputMissionType=3 but line geometry detected."
                        )
                elif input_mission_type in (4, 5):
                    # 엄호(4=공중부대/5=지상부대) area는 경계(mtype3)와 동일한 영역경계(4) 스윕.
                    # 내부 8=엄호/pattern 11=대상엄호는 대상추종용이라 지역 스윕/커버리지가 없음.
                    if geometry == "area":
                        im_type = 4
                        pattern_type = _pick_area_pattern(input_mission_type, profile_code, int(piece.piece_index))
                        rule = f"pkg{package_type} mtype{input_mission_type} escort-area"
                    elif geometry == "line":
                        im_type, pattern_type, rule = 6, 8, f"pkg{package_type} mtype{input_mission_type} line-fallback"
                        warnings.append(
                            f"M{parent_order}-{piece.piece_index}: inputMissionType={input_mission_type} but line geometry detected."
                        )
                elif input_mission_type == 6:
                    if geometry == "line":
                        im_type, pattern_type, rule = 6, 8, f"pkg{package_type} mtype6 line"
                    elif geometry == "area":
                        im_type = 3
                        pattern_type = _pick_area_pattern(6, profile_code, int(piece.piece_index))
                        rule = f"pkg{package_type} mtype6 area"
                elif input_mission_type == 7:
                    if geometry == "line":
                        im_type = 7
                        pattern_type = 9 if int(piece.piece_index) == leader_piece_id else 10
                        rule = f"pkg{package_type} mtype7 formation"
                        # d0303 legacy formation role parser uses line width tag:
                        #   width==0 -> leader, width==1 -> follower.
                        data["width"] = 0 if int(pattern_type) == 9 else 1
                    elif geometry == "area":
                        warnings.append(f"M{parent_order}-{piece.piece_index}: inputMissionType=7 but area geometry detected.")

            # Generic fallback when unmatched
            if im_type is None or pattern_type is None:
                if geometry == "area":
                    im_type, pattern_type, rule = 3, 6, "fallback area"
                elif geometry == "line":
                    im_type, pattern_type, rule = 6, 8, "fallback line"
                else:
                    im_type, pattern_type, rule = 5, 1, "fallback coordinate"
                    _ensure_coordinate_piece_data(piece, mission)

            before_im = _to_int(data.get("individualMissionType"), 0)
            before_pt = _to_int(data.get("patternType"), 0)
            data["individualMissionType"] = int(im_type)
            data["patternType"] = int(pattern_type)
            data["typeDecider"] = {
                "mode": "logic",
                "planningMode": dict(mode_context),
                "packageType": int(package_type),
                "inputMissionType": int(input_mission_type),
                "geometry": geometry,
                "profileCode": int(profile_code),
                "leaderAircraftID": int(leader_aircraft_id) if leader_aircraft_id is not None else None,
                "rule": rule,
            }
            if before_im != int(im_type) or before_pt != int(pattern_type):
                changed += 1

            imtype_counter[int(im_type)] += 1
            pattern_counter[int(pattern_type)] += 1
            assignments.append(
                {
                    "parentOrder": int(parent_order),
                    "pieceIndex": int(piece.piece_index),
                    "individualMissionType": int(im_type),
                    "patternType": int(pattern_type),
                    "rule": rule,
                }
            )

    return {
        "mode": "logic",
        "packageType": int(package_type),
        "planningMode": dict(mode_context),
        "profileCode": int(profile_code),
        "priorityAircraftIDs": [int(x) for x in priority_uavs],
        "leaderAircraftID": int(leader_aircraft_id) if leader_aircraft_id is not None else None,
        "changedPieces": int(changed),
        "pieceCount": len(split_result.pieces),
        "imTypeCounts": dict(sorted(imtype_counter.items())),
        "patternTypeCounts": dict(sorted(pattern_counter.items())),
        "warnings": warnings,
        "assignments": assignments,
    }
