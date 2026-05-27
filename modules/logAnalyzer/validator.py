"""ICD Validation Engine for DSS Log Analyzer.

Validates parsed scenario data against ICD type definitions,
range constraints, structural rules, and cross-reference integrity.
"""
from __future__ import annotations

from typing import Any


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _coerce_int(value: Any) -> int | None:
    try:
        return int(value)
    except Exception:
        return None


def _get_nested(d: dict, *keys: str) -> Any:
    """Safely traverse nested dicts."""
    cur = d
    for k in keys:
        if not isinstance(cur, dict):
            return None
        cur = cur.get(k)
    return cur


# ---------------------------------------------------------------------------
# Issue builder
# ---------------------------------------------------------------------------

def _issue(
    severity: str,
    category: str,
    location: str,
    field: str,
    message: str,
    value: Any = None,
    expected: str = "",
) -> dict:
    return {
        "severity": severity,
        "category": category,
        "location": location,
        "field": field,
        "message": message,
        "value": value,
        "expected": expected,
    }


# ---------------------------------------------------------------------------
# Type / range checkers
# ---------------------------------------------------------------------------

def _check_int(value: Any, field: str, location: str, lo: int | None = None, hi: int | None = None) -> list[dict]:
    """Value must be an int (not float). Optionally check range [lo, hi]."""
    issues: list[dict] = []
    if value is None:
        return issues
    if isinstance(value, bool):
        issues.append(_issue("error", "type", location, field,
                             f"{field}은(는) int여야 하나 bool({value})입니다",
                             value, f"int"))
        return issues
    if isinstance(value, float):
        if value != int(value):
            # 정수가 아닌 실수 (예: 826.5) → 실제 타입 오류
            issues.append(_issue("error", "type", location, field,
                                 f"{field}은(는) int여야 하나 float({value})입니다",
                                 value, f"int" + (f" {lo}-{hi}" if lo is not None else "")))
        # 정수값 float (예: 826.0) → 실무상 무해하므로 무시
        return issues
    if not isinstance(value, int):
        issues.append(_issue("error", "type", location, field,
                             f"{field}은(는) int여야 하나 {type(value).__name__}({value})입니다",
                             value, f"int" + (f" {lo}-{hi}" if lo is not None else "")))
        return issues
    if lo is not None and value < lo:
        issues.append(_issue("error", "range", location, field,
                             f"{field} 값 {value}이(가) 최소값 {lo} 미만입니다",
                             value, f"int {lo}-{hi}"))
    if hi is not None and value > hi:
        issues.append(_issue("error", "range", location, field,
                             f"{field} 값 {value}이(가) 최대값 {hi} 초과입니다",
                             value, f"int {lo}-{hi}"))
    return issues


def _check_uint(value: Any, field: str, location: str) -> list[dict]:
    """Value must be a non-negative integer."""
    issues: list[dict] = []
    if value is None:
        return issues
    if isinstance(value, bool):
        issues.append(_issue("error", "type", location, field,
                             f"{field}은(는) uint여야 하나 bool({value})입니다",
                             value, "uint (>= 0)"))
        return issues
    if isinstance(value, float):
        if value >= 0 and value == int(value):
            issues.append(_issue("warning", "type", location, field,
                                 f"{field}은(는) uint여야 하나 float({value})입니다",
                                 value, "uint (>= 0)"))
        else:
            issues.append(_issue("error", "type", location, field,
                                 f"{field}은(는) uint여야 하나 float({value})입니다",
                                 value, "uint (>= 0)"))
        return issues
    if not isinstance(value, int):
        issues.append(_issue("error", "type", location, field,
                             f"{field}은(는) uint여야 하나 {type(value).__name__}({value})입니다",
                             value, "uint (>= 0)"))
        return issues
    if value < 0:
        issues.append(_issue("error", "range", location, field,
                             f"{field} 값 {value}이(가) 음수입니다 (uint 필요)",
                             value, "uint (>= 0)"))
    return issues


def _check_uint64(value: Any, field: str, location: str) -> list[dict]:
    """Value must be a non-negative integer (uint64)."""
    issues: list[dict] = []
    if value is None:
        return issues
    if isinstance(value, bool):
        issues.append(_issue("error", "type", location, field,
                             f"{field}은(는) uint64여야 하나 bool입니다",
                             value, "uint64 (>= 0)"))
        return issues
    if isinstance(value, float):
        if value >= 0 and value == int(value):
            issues.append(_issue("warning", "type", location, field,
                                 f"{field}은(는) uint64여야 하나 float({value})입니다",
                                 value, "uint64 (>= 0)"))
        else:
            issues.append(_issue("error", "type", location, field,
                                 f"{field}은(는) uint64여야 하나 float({value})입니다",
                                 value, "uint64 (>= 0)"))
        return issues
    if not isinstance(value, int):
        issues.append(_issue("error", "type", location, field,
                             f"{field}은(는) uint64여야 하나 {type(value).__name__}입니다",
                             value, "uint64 (>= 0)"))
        return issues
    if value < 0:
        issues.append(_issue("error", "range", location, field,
                             f"{field} 값 {value}이(가) 음수입니다",
                             value, "uint64 (>= 0)"))
    return issues


def _check_float(value: Any, field: str, location: str, lo: float | None = None, hi: float | None = None) -> list[dict]:
    """Value must be float or int (numeric). Optionally check range."""
    issues: list[dict] = []
    if value is None:
        return issues
    if isinstance(value, bool):
        issues.append(_issue("error", "type", location, field,
                             f"{field}은(는) float여야 하나 bool입니다",
                             value, f"float" + (f" {lo}-{hi}" if lo is not None else "")))
        return issues
    if not isinstance(value, (int, float)):
        issues.append(_issue("error", "type", location, field,
                             f"{field}은(는) float여야 하나 {type(value).__name__}입니다",
                             value, f"float" + (f" {lo}-{hi}" if lo is not None else "")))
        return issues
    fval = float(value)
    if lo is not None and fval < lo:
        issues.append(_issue("error", "range", location, field,
                             f"{field} 값 {fval}이(가) 최소값 {lo} 미만입니다",
                             fval, f"float {lo}-{hi}"))
    if hi is not None and fval > hi:
        issues.append(_issue("error", "range", location, field,
                             f"{field} 값 {fval}이(가) 최대값 {hi} 초과입니다",
                             fval, f"float {lo}-{hi}"))
    return issues


def _check_enum(value: Any, field: str, location: str, valid: set[int], labels: dict[int, str] | None = None) -> list[dict]:
    """Value must be an int in valid set."""
    issues: list[dict] = []
    if value is None:
        return issues
    int_val = _coerce_int(value)
    if int_val is None:
        issues.append(_issue("error", "type", location, field,
                             f"{field}은(는) enum 값이어야 하나 {type(value).__name__}({value})입니다",
                             value, f"enum {sorted(valid)}"))
        return issues
    if int_val not in valid:
        label_str = ""
        if labels:
            label_str = ", ".join(f"{k}={v}" for k, v in sorted(labels.items()))
            label_str = f" ({label_str})"
        issues.append(_issue("error", "range", location, field,
                             f"{field} 값 {int_val}이(가) 유효하지 않습니다{label_str}",
                             int_val, f"enum {sorted(valid)}"))
    return issues


# ---------------------------------------------------------------------------
# Coordinate validation
# ---------------------------------------------------------------------------

_KOREA_LAT = (33.0, 39.0)
_KOREA_LON = (124.0, 132.0)


def _validate_coordinate(coord: dict, location: str) -> list[dict]:
    """Validate a Coordinate object (latitude, longitude, altitude)."""
    issues: list[dict] = []
    if not isinstance(coord, dict):
        return issues

    lat = coord.get("latitude") if "latitude" in coord else coord.get("Latitude")
    lon = coord.get("longitude") if "longitude" in coord else coord.get("Longitude")
    alt = coord.get("altitude") if "altitude" in coord else coord.get("Altitude")

    issues.extend(_check_float(lat, "latitude", location, -90.0, 90.0))
    issues.extend(_check_float(lon, "longitude", location, -180.0, 180.0))
    issues.extend(_check_int(alt, "altitude", location, 0, 50000))

    # Korea bounds check (info level)
    if lat is not None and lon is not None:
        try:
            flat, flon = float(lat), float(lon)
            if not (_KOREA_LAT[0] <= flat <= _KOREA_LAT[1]):
                issues.append(_issue("info", "range", location, "latitude",
                                     f"위도 {flat}이(가) 한반도 범위({_KOREA_LAT[0]}~{_KOREA_LAT[1]}) 밖입니다",
                                     flat, f"float {_KOREA_LAT[0]}-{_KOREA_LAT[1]}"))
            if not (_KOREA_LON[0] <= flon <= _KOREA_LON[1]):
                issues.append(_issue("info", "range", location, "longitude",
                                     f"경도 {flon}이(가) 한반도 범위({_KOREA_LON[0]}~{_KOREA_LON[1]}) 밖입니다",
                                     flon, f"float {_KOREA_LON[0]}-{_KOREA_LON[1]}"))
        except (TypeError, ValueError):
            pass

    return issues


# ---------------------------------------------------------------------------
# Waypoint validation
# ---------------------------------------------------------------------------

_WAYPOINT_PASS_TYPE_LABELS = {0: "None", 1: "Fly-by", 2: "Loiter", 3: "Fly-Over"}
_LOITER_DIRECTION_LABELS = {0: "None", 1: "CW", 2: "CCW"}
_SENSOR_TYPE_LABELS = {0: "None", 1: "EO", 2: "IR"}
_HEALTH_LABELS = {0: "Unknown", 1: "Normal", 2: "Abnormal"}


def _validate_waypoint(wp: dict, idx: int, location_prefix: str, aircraft_id: int | None = None) -> list[dict]:
    """Validate a single waypoint dict."""
    issues: list[dict] = []
    if not isinstance(wp, dict):
        return issues

    wp_id = wp.get("waypointID") or wp.get("WaypointID")
    loc = f"{location_prefix} > WP {wp_id if wp_id is not None else idx}"

    # waypointID: uint
    issues.extend(_check_uint(wp_id, "waypointID", loc))

    # speed: float 0-1000
    speed = wp.get("speed") or wp.get("Speed")
    issues.extend(_check_float(speed, "speed", loc, 0.0, 1000.0))

    # Reasonable speed check (warning level)
    if speed is not None:
        try:
            fspeed = float(speed)
            if aircraft_id is not None:
                if 1 <= aircraft_id <= 3:  # LAH
                    if fspeed > 0 and (fspeed < 10 or fspeed > 80):
                        issues.append(_issue("warning", "range", loc, "speed",
                                             f"LAH 속도 {fspeed} m/s가 일반적 범위(10~80)를 벗어납니다",
                                             fspeed, "float 10-80 (LAH)"))
                elif 4 <= aircraft_id <= 6:  # UAV
                    if fspeed > 0 and (fspeed < 20 or fspeed > 60):
                        issues.append(_issue("warning", "range", loc, "speed",
                                             f"UAV 속도 {fspeed} m/s가 일반적 범위(20~60)를 벗어납니다",
                                             fspeed, "float 20-60 (UAV)"))
        except (TypeError, ValueError):
            pass

    # eta: uint
    eta = wp.get("eta") or wp.get("ETA")
    issues.extend(_check_uint(eta, "eta", loc))

    # waypointPassType: enum 0-3
    pass_type = wp.get("waypointPassType") or wp.get("WaypointPassType")
    issues.extend(_check_enum(pass_type, "waypointPassType", loc, {0, 1, 2, 3}, _WAYPOINT_PASS_TYPE_LABELS))

    # nextWaypointID: uint
    next_wp = wp.get("nextWaypointID") or wp.get("NextWaypointID")
    issues.extend(_check_uint(next_wp, "nextWaypointID", loc))

    # Coordinate validation
    coord = wp.get("coordinate") or wp.get("Coordinate")
    if isinstance(coord, dict):
        issues.extend(_validate_coordinate(coord, loc))

    # -- Rule: loiterProperty presence --
    loiter = (
        wp.get("loiterProperty")
        or wp.get("LoiterProperty")
        or wp.get("loiter")
        or wp.get("Loiter")
        or wp.get("loiter_prop")
    )
    pass_type_int = _coerce_int(pass_type)

    if pass_type_int == 2:  # Loiter
        if loiter is None or not isinstance(loiter, dict):
            issues.append(_issue("error", "rule", loc, "loiterProperty",
                                 "waypointPassType=2(Loiter)이나 loiterProperty가 없습니다",
                                 None, "loiterProperty 필수"))
        else:
            _validate_loiter_property(loiter, loc, issues)
    elif pass_type_int in (1, 3):  # Fly-by or Fly-Over
        if loiter is not None and isinstance(loiter, dict):
            # Check if loiter has meaningful data (not all zeros)
            radius = loiter.get("radius") or loiter.get("Radius") or 0
            time_val = loiter.get("time") or loiter.get("Time") or 0
            if (_coerce_int(radius) or 0) > 0 or (_coerce_int(time_val) or 0) > 0:
                pt_name = "Fly-by" if pass_type_int == 1 else "Fly-Over"
                issues.append(_issue("warning", "rule", loc, "loiterProperty",
                                     f"waypointPassType={pass_type_int}({pt_name})인데 loiterProperty가 존재합니다",
                                     None, "loiterProperty 불필요"))

    # -- Rule: filmingProperty sub-checks --
    filming = (
        wp.get("filmingProperty")
        or wp.get("FilmingProperty")
        or wp.get("filming")
        or wp.get("Filming")
    )
    if isinstance(filming, dict):
        _validate_filming_property(filming, loc, wp, issues)

    return issues


def _validate_loiter_property(loiter: dict, loc: str, issues: list[dict]) -> None:
    """Validate LoiterProperty fields."""
    radius = loiter.get("radius") or loiter.get("Radius")
    issues.extend(_check_int(radius, "loiterProperty.radius", loc, 0, 50000))

    # Loiter must have radius > 0
    if radius is not None:
        r_int = _coerce_int(radius)
        if r_int is not None and r_int <= 0:
            issues.append(_issue("warning", "rule", loc, "loiterProperty.radius",
                                 f"Loiter의 radius가 {r_int}입니다 (0보다 커야 함)",
                                 r_int, "int > 0"))

    direction = loiter.get("direction") or loiter.get("Direction")
    issues.extend(_check_enum(direction, "loiterProperty.direction", loc, {0, 1, 2}, _LOITER_DIRECTION_LABELS))

    time_val = loiter.get("time") or loiter.get("Time")
    issues.extend(_check_uint(time_val, "loiterProperty.time", loc))

    speed = loiter.get("speed") or loiter.get("Speed")
    issues.extend(_check_float(speed, "loiterProperty.speed", loc, 0.0, 1000.0))


def _validate_filming_property(filming: dict, loc: str, wp: dict, issues: list[dict]) -> None:
    """Validate FilmingProperty fields and dependent sub-objects."""
    fov = filming.get("fieldOfView") or filming.get("FieldOfView")
    issues.extend(_check_float(fov, "filmingProperty.fieldOfView", loc, 0.0, 180.0))

    sensor = filming.get("sensorType") or filming.get("SensorType")
    issues.extend(_check_enum(sensor, "filmingProperty.sensorType", loc, {0, 1, 2}, _SENSOR_TYPE_LABELS))

    op_mode = filming.get("operationMode") or filming.get("OperationMode")
    issues.extend(_check_enum(op_mode, "filmingProperty.operationMode", loc, {0, 1, 2, 3, 4, 5}))

    op_mode_int = _coerce_int(op_mode)

    if op_mode_int == 1:  # Coordinate mode
        coord_orient = (
            filming.get("coordinateOrientation")
            or filming.get("CoordinateOrientation")
            or wp.get("coordinateOrientation")
            or wp.get("CoordinateOrientation")
        )
        if coord_orient is None:
            issues.append(_issue("warning", "rule", loc, "coordinateOrientation",
                                 "operationMode=1(Coordinate)인데 coordinateOrientation이 없습니다",
                                 None, "coordinateOrientation 필수"))

    elif op_mode_int == 2:  # Line Search
        line_search = (
            filming.get("lineSearch")
            or filming.get("LineSearch")
            or wp.get("lineSearch")
            or wp.get("LineSearch")
        )
        if line_search is None:
            issues.append(_issue("warning", "rule", loc, "lineSearch",
                                 "operationMode=2(Line Search)인데 lineSearch가 없습니다",
                                 None, "lineSearch 필수"))

    elif op_mode_int == 3:  # Auto Track
        auto_track = (
            filming.get("autoTracking")
            or filming.get("AutoTracking")
            or wp.get("autoTracking")
            or wp.get("AutoTracking")
        )
        if auto_track is None:
            issues.append(_issue("warning", "rule", loc, "autoTracking",
                                 "operationMode=3(Auto Track)인데 autoTracking이 없습니다",
                                 None, "autoTracking 필수"))
        elif isinstance(auto_track, dict):
            target_id = auto_track.get("targetID") or auto_track.get("TargetID")
            if target_id is not None:
                tid = _coerce_int(target_id)
                if tid is not None and tid <= 0:
                    issues.append(_issue("error", "rule", loc, "autoTracking.targetID",
                                         f"Auto Track의 targetID가 {tid}입니다 (0보다 커야 함)",
                                         tid, "uint > 0"))


# ---------------------------------------------------------------------------
# Flight path validation
# ---------------------------------------------------------------------------

def _validate_flight_path(path_key: str, raw_data: dict, aircraft_id_hint: int | None = None) -> list[dict]:
    """Validate a single flight path's data."""
    issues: list[dict] = []
    loc_base = f"FlightPath/{path_key}"

    # Structure: required fields
    path_id = raw_data.get("pathID") or raw_data.get("PathID")
    ac_id = raw_data.get("aircraftID") or raw_data.get("AircraftID")
    timestamp = raw_data.get("timestamp") or raw_data.get("Timestamp")

    if path_id is None:
        issues.append(_issue("error", "structure", loc_base, "pathID",
                             "pathID 필드가 없습니다", None, "pathID 필수"))
    else:
        issues.extend(_check_uint(path_id, "pathID", loc_base))

    if ac_id is None:
        issues.append(_issue("warning", "structure", loc_base, "aircraftID",
                             "aircraftID 필드가 없습니다", None, "aircraftID 필수"))
    else:
        issues.extend(_check_enum(ac_id, "aircraftID", loc_base, {0, 1, 2, 3, 4, 5, 6}))

    if timestamp is not None:
        issues.extend(_check_uint64(timestamp, "timestamp", loc_base))

    aircraft_id = _coerce_int(ac_id) if ac_id is not None else aircraft_id_hint

    # Extract waypoints
    waypoints: list[dict] = []
    for key in ("lahWaypointList", "uavWaypointList", "waypointList"):
        lst = raw_data.get(key)
        if isinstance(lst, list):
            waypoints = lst
            break

    if not waypoints:
        issues.append(_issue("error", "structure", loc_base, "waypointList",
                             "waypointList가 비어있거나 없습니다", None, "waypointList 필수"))
        return issues

    # WP 1개짜리 경로는 추적/체공/단일이동 등 정상 케이스

    # Validate each waypoint
    for i, wp in enumerate(waypoints):
        if not isinstance(wp, dict):
            continue
        issues.extend(_validate_waypoint(wp, i, loc_base, aircraft_id))

    # Rule: nextWaypointID chain consistency
    issues.extend(_validate_waypoint_chain(waypoints, loc_base))

    return issues


def _validate_waypoint_chain(waypoints: list[dict], loc_base: str) -> list[dict]:
    """Check nextWaypointID chain for gaps and cycles."""
    issues: list[dict] = []

    by_id: dict[int, dict] = {}
    for wp in waypoints:
        if not isinstance(wp, dict):
            continue
        wid = wp.get("waypointID") or wp.get("WaypointID")
        wid_int = _coerce_int(wid)
        if wid_int is not None:
            by_id[wid_int] = wp

    if not by_id:
        return issues

    # Find start node (not pointed to by any nextWaypointID)
    pointed_to: set[int] = set()
    for wp in waypoints:
        if not isinstance(wp, dict):
            continue
        nxt = wp.get("nextWaypointID") or wp.get("NextWaypointID")
        nxt_int = _coerce_int(nxt)
        if nxt_int is not None and nxt_int > 0:
            pointed_to.add(nxt_int)

    starts = [wid for wid in by_id if wid not in pointed_to]

    if len(starts) == 0:
        issues.append(_issue("error", "rule", loc_base, "nextWaypointID",
                             "시작 웨이포인트를 찾을 수 없습니다 (순환 가능성)",
                             None, "시작 WP 필요"))
        return issues

    if len(starts) > 1:
        issues.append(_issue("warning", "rule", loc_base, "nextWaypointID",
                             f"시작 웨이포인트 후보가 {len(starts)}개입니다: {starts}",
                             starts, "시작 WP 1개"))

    # Walk the chain from first start
    start = starts[0]
    visited: set[int] = set()
    curr = start
    chain_len = 0
    while curr is not None and curr in by_id and curr not in visited:
        visited.add(curr)
        chain_len += 1
        wp = by_id[curr]
        nxt = wp.get("nextWaypointID") or wp.get("NextWaypointID")
        nxt_int = _coerce_int(nxt)
        if nxt_int is None or nxt_int == 0:
            break
        if nxt_int not in by_id:
            wp_id_str = wp.get("waypointID") or wp.get("WaypointID") or "?"
            issues.append(_issue("error", "rule", loc_base,
                                 "nextWaypointID",
                                 f"WP {wp_id_str}의 nextWaypointID={nxt_int}가 존재하지 않는 WP를 가리킵니다",
                                 nxt_int, "유효한 waypointID"))
            break
        curr = nxt_int

    # Check if all waypoints are in the chain
    unreachable = set(by_id.keys()) - visited
    if unreachable:
        issues.append(_issue("warning", "rule", loc_base, "nextWaypointID",
                             f"체인에 포함되지 않은 웨이포인트가 {len(unreachable)}개 있습니다: {sorted(unreachable)}",
                             sorted(unreachable), "모든 WP 연결"))

    return issues


# ---------------------------------------------------------------------------
# IndividualMissionPlan validation
# ---------------------------------------------------------------------------

def _validate_imp(imp_key: str, imp_data: dict, existing_path_ids: set[int]) -> list[dict]:
    """Validate an IndividualMissionPlan."""
    issues: list[dict] = []
    loc_base = f"IMP/{imp_key}"

    # Structure checks
    imp_id = (
        imp_data.get("individualMissionPackageID")
        or imp_data.get("IndividualMissionPackageID")
        or imp_data.get("individualMissionPackageId")
    )
    if imp_id is None:
        issues.append(_issue("error", "structure", loc_base, "individualMissionPackageID",
                             "individualMissionPackageID 필드가 없습니다", None, "필수 필드"))
    else:
        issues.extend(_check_uint(imp_id, "individualMissionPackageID", loc_base))

    ac_id = imp_data.get("aircraftID") or imp_data.get("AircraftID")
    if ac_id is None:
        issues.append(_issue("warning", "structure", loc_base, "aircraftID",
                             "aircraftID 필드가 없습니다", None, "필수 필드"))
    else:
        issues.extend(_check_enum(ac_id, "aircraftID", loc_base, {0, 1, 2, 3, 4, 5, 6}))

    # individualMissionList
    mission_list = imp_data.get("individualMissionList") or imp_data.get("IndividualMissionList") or []
    if not mission_list:
        issues.append(_issue("warning", "structure", loc_base, "individualMissionList",
                             "individualMissionList가 비어있습니다", None, "미션 목록 필요"))

    for i, mission in enumerate(mission_list):
        if not isinstance(mission, dict):
            continue
        m_id = mission.get("individualMissionID") or mission.get("IndividualMissionID")
        m_loc = f"{loc_base} > Mission {m_id if m_id is not None else i}"

        if m_id is not None:
            issues.extend(_check_uint(m_id, "individualMissionID", m_loc))

        m_type = mission.get("missionType") or mission.get("MissionType")
        if m_type is not None:
            issues.extend(_check_uint(m_type, "missionType", m_loc))

        # pathID reference check
        path_id = mission.get("pathID") or mission.get("PathID")
        if path_id is not None:
            issues.extend(_check_uint(path_id, "pathID", m_loc))
            pid_int = _coerce_int(path_id)
            if pid_int is not None and pid_int > 0 and pid_int not in existing_path_ids:
                issues.append(_issue("error", "rule", m_loc, "pathID",
                                     f"pathID {pid_int}에 해당하는 FlightPath가 존재하지 않습니다",
                                     pid_int, f"존재하는 pathID: {sorted(existing_path_ids)[:10]}..."))

    return issues


# ---------------------------------------------------------------------------
# MissionPlan validation
# ---------------------------------------------------------------------------

def _validate_mission_plan(
    plan_data: dict,
    existing_imp_ids: set[int],
    plan_index: int,
) -> list[dict]:
    """Validate a MissionPlan."""
    issues: list[dict] = []
    mp_id = plan_data.get("missionPlanID") or plan_data.get("MissionPlanID")
    loc_base = f"MissionPlan/{mp_id if mp_id is not None else plan_index}"

    # Structure: required fields
    if mp_id is None:
        issues.append(_issue("error", "structure", loc_base, "missionPlanID",
                             "missionPlanID 필드가 없습니다", None, "필수 필드"))
    else:
        issues.extend(_check_uint(mp_id, "missionPlanID", loc_base))

    timestamp = plan_data.get("timestamp") or plan_data.get("Timestamp")
    if timestamp is not None:
        issues.extend(_check_uint64(timestamp, "timestamp", loc_base))

    aircraft_list = plan_data.get("aircraftList") or plan_data.get("AircraftList") or []
    if not aircraft_list:
        issues.append(_issue("warning", "structure", loc_base, "aircraftList",
                             "aircraftList가 비어있습니다", None, "항공기 목록 필요"))

    for i, ac_entry in enumerate(aircraft_list):
        if not isinstance(ac_entry, dict):
            continue
        ac_id = ac_entry.get("aircraftID") or ac_entry.get("AircraftID")
        ac_loc = f"{loc_base} > Aircraft {ac_id if ac_id is not None else i}"

        if ac_id is not None:
            issues.extend(_check_enum(ac_id, "aircraftID", ac_loc, {0, 1, 2, 3, 4, 5, 6}))

        # IMP reference
        imp_id = (
            ac_entry.get("individualMissionPackageID")
            or ac_entry.get("individualMissionPackageId")
            or ac_entry.get("IndividualMissionPackageID")
        )
        if imp_id is not None:
            issues.extend(_check_uint(imp_id, "individualMissionPackageID", ac_loc))
            iid = _coerce_int(imp_id)
            if iid is not None and iid > 0 and iid not in existing_imp_ids:
                issues.append(_issue("error", "rule", ac_loc, "individualMissionPackageID",
                                     f"IMP ID {iid}에 해당하는 IndividualMissionPlan이 존재하지 않습니다",
                                     iid, f"존재하는 IMP ID: {sorted(existing_imp_ids)[:10]}..."))

        # health check
        health = ac_entry.get("health") or ac_entry.get("Health")
        if health is not None:
            issues.extend(_check_enum(health, "health", ac_loc, {0, 1, 2}, _HEALTH_LABELS))

        # LAH/UAV consistency
        ac_id_int = _coerce_int(ac_id)
        if ac_id_int is not None:
            if 1 <= ac_id_int <= 3:
                expected_type = "LAH"
            elif 4 <= ac_id_int <= 6:
                expected_type = "UAV"
            else:
                expected_type = None

            ac_type = ac_entry.get("aircraftType") or ac_entry.get("AircraftType")
            if ac_type is not None and expected_type is not None:
                ac_type_int = _coerce_int(ac_type)
                # aircraftType: 1=LAH, 2=UAV (common convention)
                if ac_type_int == 1 and expected_type == "UAV":
                    issues.append(_issue("warning", "rule", ac_loc, "aircraftType",
                                         f"aircraftID={ac_id_int}은 UAV인데 aircraftType=1(LAH)입니다",
                                         ac_type_int, f"aircraftType=2(UAV)"))
                elif ac_type_int == 2 and expected_type == "LAH":
                    issues.append(_issue("warning", "rule", ac_loc, "aircraftType",
                                         f"aircraftID={ac_id_int}은 LAH인데 aircraftType=2(UAV)입니다",
                                         ac_type_int, f"aircraftType=1(LAH)"))

    return issues


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def validate_scenario(parsed_data: dict) -> list[dict]:
    """Validate an entire parsed scenario against ICD rules.

    Parameters
    ----------
    parsed_data : dict
        The result of ``log_parser.parse_scenario()``.

    Returns
    -------
    list[dict]
        List of issue dicts, each with keys:
        severity, category, location, field, message, value, expected.
    """
    issues: list[dict] = []

    if not parsed_data or not parsed_data.get("ok"):
        issues.append(_issue("error", "structure", "Scenario", "ok",
                             "시나리오 파싱 실패", parsed_data.get("error"), "ok=True"))
        return issues

    # ------------------------------------------------------------------
    # 1. Validate all FlightPaths
    # ------------------------------------------------------------------
    flight_paths = parsed_data.get("flightPaths") or {}
    individual_plans = parsed_data.get("individualPlans") or {}

    # We need raw data — the parsed flightPaths might be simplified.
    # Reconstruct from the parsed structure.
    existing_path_ids: set[int] = set()
    for key, fp_data in flight_paths.items():
        pid = _coerce_int(key)
        if pid is not None:
            existing_path_ids.add(pid)

        # fp_data has 'coordinates' and 'waypoints' (the simplified form)
        # We need to validate the waypoints
        waypoints = fp_data.get("waypoints") or []
        if waypoints:
            loc_base = f"FlightPath/{key}"

            # 개별 WP 타입/범위/규칙 검사는 validate_scenario_raw()에서 수행.
            # parsed 검증은 교차 참조 위주로만 수행.

    # ------------------------------------------------------------------
    # 2. Validate from raw data when available (full waypoint validation)
    # ------------------------------------------------------------------
    # Try to get the raw FlightPath data from resolved plans
    mission_plans_data = parsed_data.get("missionPlans") or []
    all_raw_paths: dict[str, dict] = {}

    for rp in mission_plans_data:
        resolved = rp.get("resolved") or {}
        aircraft_map = resolved.get("aircraft") or {}
        for ac_id_str, ac_data in aircraft_map.items():
            for path_info in ac_data.get("paths") or []:
                pid = path_info.get("pathID")
                if pid is not None:
                    existing_path_ids.add(int(pid) if isinstance(pid, (int, float)) else 0)

    # ------------------------------------------------------------------
    # 3. Validate IndividualMissionPlans
    # ------------------------------------------------------------------
    existing_imp_ids: set[int] = set()
    for key, imp_data in individual_plans.items():
        iid = _coerce_int(key)
        if iid is not None:
            existing_imp_ids.add(iid)
        issues.extend(_validate_imp(key, imp_data, existing_path_ids))

    # ------------------------------------------------------------------
    # 4. Validate MissionPlans
    # ------------------------------------------------------------------
    for i, rp in enumerate(mission_plans_data):
        plan = rp.get("plan") or {}
        issues.extend(_validate_mission_plan(plan, existing_imp_ids, i))

    # ------------------------------------------------------------------
    # 5. Cross-reference integrity: Plan → IMP → FlightPath
    # ------------------------------------------------------------------
    for rp in mission_plans_data:
        mp_id = rp.get("missionPlanID")
        plan = rp.get("plan") or {}
        aircraft_list = plan.get("aircraftList") or []
        resolved = rp.get("resolved") or {}
        aircraft_map = resolved.get("aircraft") or {}

        for ac_entry in aircraft_list:
            if not isinstance(ac_entry, dict):
                continue
            ac_id = ac_entry.get("aircraftID") or ac_entry.get("AircraftID")
            imp_id = (
                ac_entry.get("individualMissionPackageID")
                or ac_entry.get("individualMissionPackageId")
                or ac_entry.get("IndividualMissionPackageID")
            )
            imp_id_int = _coerce_int(imp_id)
            ac_id_int = _coerce_int(ac_id)

            if imp_id_int is not None and ac_id_int is not None:
                # Check IMP exists and its aircraftID matches
                imp_data = individual_plans.get(str(imp_id_int))
                if imp_data:
                    imp_ac_id = imp_data.get("aircraftID") or imp_data.get("AircraftID")
                    imp_ac_int = _coerce_int(imp_ac_id)
                    if imp_ac_int is not None and imp_ac_int != ac_id_int:
                        loc = f"MissionPlan/{mp_id} > Aircraft {ac_id_int}"
                        issues.append(_issue("error", "rule", loc, "aircraftID",
                                             f"MissionPlan의 aircraftID={ac_id_int}와 "
                                             f"IMP({imp_id_int})의 aircraftID={imp_ac_int}가 불일치합니다",
                                             imp_ac_int, f"aircraftID={ac_id_int}"))

    # ------------------------------------------------------------------
    # 6. Vehicle status validation
    # ------------------------------------------------------------------
    vehicle_status = parsed_data.get("vehicleStatus") or {}
    if vehicle_status:
        for key in ("agentStateList", "AgentStateList"):
            agent_list = vehicle_status.get(key)
            if isinstance(agent_list, list):
                for agent in agent_list:
                    if not isinstance(agent, dict):
                        continue
                    ac_id = agent.get("aircraftID") or agent.get("AircraftID")
                    loc = f"VehicleStatus > Aircraft {ac_id}"
                    if ac_id is not None:
                        issues.extend(_check_enum(ac_id, "aircraftID", loc, {0, 1, 2, 3, 4, 5, 6}))

                    health = agent.get("health") or agent.get("Health")
                    if health is not None:
                        issues.extend(_check_enum(health, "health", loc, {0, 1, 2}, _HEALTH_LABELS))

                    ts = agent.get("timestamp") or agent.get("Timestamp")
                    if ts is not None:
                        issues.extend(_check_uint64(ts, "timestamp", loc))

                    coord = agent.get("coordinate") or agent.get("Coordinate")
                    if isinstance(coord, dict):
                        issues.extend(_validate_coordinate(coord, loc))
                break

    return issues


# ---------------------------------------------------------------------------
# Standalone validation from raw files (for direct file-level checking)
# ---------------------------------------------------------------------------

def validate_scenario_raw(scenario_path) -> list[dict]:
    """Validate a scenario by reading raw JSON files directly.

    This provides deeper validation because it accesses the full waypoint
    data including loiterProperty, filmingProperty, etc. that are not
    included in the simplified parsed output.

    Parameters
    ----------
    scenario_path : Path
        Path to the scenario folder (e.g. Logs/Scenario_...).

    Returns
    -------
    list[dict]
        Issues list.
    """
    import json
    from pathlib import Path

    scenario_path = Path(scenario_path)
    base = scenario_path / "SBC3"
    if not base.is_dir():
        return [_issue("error", "structure", "Scenario", "SBC3",
                        "SBC3 폴더가 없습니다", None, "SBC3 폴더 필수")]

    issues: list[dict] = []

    # --- FlightPaths ---
    existing_path_ids: set[int] = set()
    fp_dir = base / "FlightPath"
    if fp_dir.is_dir():
        for fp in sorted(fp_dir.glob("*.json")):
            try:
                data = json.loads(fp.read_text(encoding="utf-8"))
            except Exception:
                issues.append(_issue("error", "structure", f"FlightPath/{fp.stem}", "file",
                                     f"JSON 파싱 실패: {fp.name}", None, "유효한 JSON"))
                continue
            if not isinstance(data, dict):
                continue

            pid = _coerce_int(data.get("pathID") or data.get("PathID") or fp.stem)
            if pid is not None:
                existing_path_ids.add(pid)

            issues.extend(_validate_flight_path(fp.stem, data))

    # --- IndividualMissionPlans ---
    existing_imp_ids: set[int] = set()
    imp_dir = base / "IndividualMissionPlan"
    if imp_dir.is_dir():
        for fp in sorted(imp_dir.glob("*.json")):
            try:
                data = json.loads(fp.read_text(encoding="utf-8"))
            except Exception:
                issues.append(_issue("error", "structure", f"IMP/{fp.stem}", "file",
                                     f"JSON 파싱 실패: {fp.name}", None, "유효한 JSON"))
                continue
            if not isinstance(data, dict):
                continue

            iid = _coerce_int(
                data.get("individualMissionPackageID")
                or data.get("IndividualMissionPackageID")
                or fp.stem
            )
            if iid is not None:
                existing_imp_ids.add(iid)

            issues.extend(_validate_imp(fp.stem, data, existing_path_ids))

    # --- MissionPlans ---
    mp_dir = base / "MissionPlan"
    if mp_dir.is_dir():
        for i, fp in enumerate(sorted(mp_dir.glob("*.json"))):
            try:
                data = json.loads(fp.read_text(encoding="utf-8"))
            except Exception:
                issues.append(_issue("error", "structure", f"MissionPlan/{fp.stem}", "file",
                                     f"JSON 파싱 실패: {fp.name}", None, "유효한 JSON"))
                continue
            if not isinstance(data, dict):
                continue
            issues.extend(_validate_mission_plan(data, existing_imp_ids, i))

    return issues
