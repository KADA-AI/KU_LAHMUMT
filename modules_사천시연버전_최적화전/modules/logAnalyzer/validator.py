"""ICD Validation Engine for DSS Log Analyzer.

Validates parsed scenario data against ICD type definitions,
range constraints, structural rules, and cross-reference integrity.
"""
from __future__ import annotations

import math
from typing import Any


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_MISSING = object()


def _pick(d: dict, *keys: str, default: Any = None) -> Any:
    """Return the first key that is present, preserving valid falsy values."""
    if not isinstance(d, dict):
        return default
    for key in keys:
        if key in d:
            return d[key]
    return default


def _has_field(d: dict, *keys: str) -> bool:
    return isinstance(d, dict) and any(key in d for key in keys)


def _is_meaningful(value: Any) -> bool:
    if value is None:
        return False
    if isinstance(value, dict):
        return any(_is_meaningful(v) for v in value.values())
    if isinstance(value, list):
        return any(_is_meaningful(v) for v in value)
    if isinstance(value, str):
        return bool(value.strip())
    return True


def _as_list(value: Any) -> list[Any]:
    return value if isinstance(value, list) else []


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


def _derive_issue_metadata(category: str, location: str, field: str, message: str) -> dict[str, str]:
    """Attach a compact rule id/source so UI users can trace why an issue fired."""
    haystack = f"{location} {field} {message}".lower()

    if "53112" in haystack:
        return {"ruleId": "NFUSION-53112", "source": "ICD 53112 UAVMissionPlan"}
    if "53111" in haystack:
        return {"ruleId": "NFUSION-53111", "source": "ICD 53111 LAHMissionPlan"}
    if "0904" in haystack:
        return {"ruleId": "NFUSION-0904", "source": "ICD 0904 RequestBehaviorTree"}
    if "0903" in haystack:
        return {"ruleId": "PLAN-SELECTION-REF", "source": "ICD 0903 RequestRenewMission"}
    if "0702" in haystack:
        return {"ruleId": "PLAN-SELECTION-REF", "source": "ICD 0702 PilotDecision"}
    if "0701" in haystack or "missionplanoptioninfo" in haystack:
        return {"ruleId": "NFUSION-0701", "source": "ICD 0701 MissionPlanOptionInfo"}
    if "0301" in haystack:
        return {"ruleId": "NFUSION-0301", "source": "ICD 0301 MissionPlan"}
    if "0302" in haystack:
        return {"ruleId": "NFUSION-0302", "source": "ICD 0302 IndividualMissionPlan"}
    if "0303" in haystack:
        return {"ruleId": "NFUSION-0303", "source": "ICD 0303 UAVFlightPlan"}
    if "0304" in haystack:
        return {"ruleId": "NFUSION-0304", "source": "ICD 0304 LAHFlightPlan"}
    if "0201" in haystack or "inputmissionpackage" in haystack or "inputmissionplan" in haystack:
        return {"ruleId": "NFUSION-0201", "source": "ICD 0201 InputMissionPlan"}
    if "0203" in haystack or "missionreferencepackage" in haystack or "missionreferenceinfo" in haystack:
        return {"ruleId": "NFUSION-0203", "source": "ICD 0203 FlightReferenceInfo"}
    if "missionsegment" in haystack:
        return {"ruleId": "SEGMENT-GRAPH", "source": "ICD 53111/53112 MissionSegment"}
    if "autotracking" in haystack or "targetid" in haystack:
        return {"ruleId": "TARGET-REF", "source": "ICD AutoTracking/Attack + TargetInfo"}
    if "operationmode" in haystack or "linesearch" in haystack or "coordinateorientation" in haystack:
        return {"ruleId": "FILMING-MODE", "source": "ICD 0303/53112 FilmingProperty"}
    if "aircraftfixed" in haystack or "autoscan" in haystack:
        return {"ruleId": "FILMING-GIMBAL", "source": "ICD AircraftFixed/AutoScan"}
    if "waypointid" in haystack or "nextwaypointid" in haystack:
        return {"ruleId": "WP-CHAIN", "source": "mission generator waypoint chain"}
    if "individualmissioninfo" in haystack or "relatedmission" in haystack:
        return {"ruleId": "IMP-GEOMETRY", "source": "ICD 0302 IndividualMissionInfo"}
    if "pathid" in haystack:
        return {"ruleId": "PATH-REF", "source": "mission generator path/aircraft ID bands"}
    if "isdone" in haystack:
        return {"ruleId": "DONE-CONSISTENCY", "source": "mission/waypoint completion state"}
    if "coordinate" in haystack or field in {"latitude", "longitude", "altitude"}:
        return {"ruleId": "COORD-TYPE", "source": "ICD Coordinate"}
    if category == "type":
        return {"ruleId": "ICD-TYPE", "source": "nFusion ICD type definition"}
    if category == "range":
        return {"ruleId": "ICD-RANGE", "source": "nFusion ICD range constraint"}
    if category == "structure":
        return {"ruleId": "ICD-REQUIRED", "source": "nFusion ICD required structure"}
    if category == "rule":
        return {"ruleId": "LOGIC-RULE", "source": "mission planning code invariant"}
    return {}


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
    expected: Any = "",
    rule_id: str | None = None,
    source: str | None = None,
    hint: str | None = None,
) -> dict:
    issue = {
        "severity": severity,
        "category": category,
        "location": location,
        "field": field,
        "message": message,
        "value": value,
        "expected": expected,
    }
    metadata = _derive_issue_metadata(category, location, field, message)
    issue["ruleId"] = rule_id or metadata.get("ruleId", "")
    issue["source"] = source or metadata.get("source", "")
    if hint:
        issue["hint"] = hint
    return issue


# ---------------------------------------------------------------------------
# Type / range checkers
# ---------------------------------------------------------------------------

_UINT32_MAX = 2**32 - 1
_UINT64_MAX = 2**64 - 1


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

def _check_icd_int(value: Any, field: str, location: str, lo: int | None = None, hi: int | None = None) -> list[dict]:
    """Strict ICD integer check. Integral floats are still a type issue."""
    issues: list[dict] = []
    if value is None:
        return issues
    expected = "int" + (f" {lo}-{hi}" if lo is not None else "")
    if isinstance(value, bool):
        issues.append(_issue("error", "type", location, field,
                             f"{field} must be int, got bool({value})", value, expected))
        return issues
    if isinstance(value, float):
        severity = "warning" if value == int(value) else "error"
        issues.append(_issue(severity, "type", location, field,
                             f"{field} must be int, got float({value})", value, expected))
        return issues
    if not isinstance(value, int):
        issues.append(_issue("error", "type", location, field,
                             f"{field} must be int, got {type(value).__name__}({value})",
                             value, expected))
        return issues
    if lo is not None and value < lo:
        issues.append(_issue("error", "range", location, field,
                             f"{field}={value} is below minimum {lo}", value, expected))
    if hi is not None and value > hi:
        issues.append(_issue("error", "range", location, field,
                             f"{field}={value} exceeds maximum {hi}", value, expected))
    return issues


def _check_uint(value: Any, field: str, location: str) -> list[dict]:
    """Value must be a non-negative integer."""
    issues: list[dict] = []
    if value is None:
        return issues
    expected = f"uint32 0..{_UINT32_MAX}"
    if isinstance(value, bool):
        issues.append(_issue("error", "type", location, field,
                             f"{field} must be uint32, got bool({value})",
                             value, expected))
        return issues
    if isinstance(value, float):
        if value >= 0 and value == int(value):
            issues.append(_issue("warning", "type", location, field,
                                 f"{field} must be uint32, got float({value})",
                                 value, expected))
            if value > _UINT32_MAX:
                issues.append(_issue("error", "range", location, field,
                                     f"{field}={value} exceeds uint32 maximum {_UINT32_MAX}",
                                     value, expected))
        else:
            issues.append(_issue("error", "type", location, field,
                                 f"{field} must be uint32, got float({value})",
                                 value, expected))
        return issues
    if not isinstance(value, int):
        issues.append(_issue("error", "type", location, field,
                             f"{field} must be uint32, got {type(value).__name__}({value})",
                             value, expected))
        return issues
    if value < 0:
        issues.append(_issue("error", "range", location, field,
                             f"{field}={value} is below uint32 minimum 0",
                             value, expected))
    if value > _UINT32_MAX:
        issues.append(_issue("error", "range", location, field,
                             f"{field}={value} exceeds uint32 maximum {_UINT32_MAX}",
                             value, expected))
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


def _check_uint16(value: Any, field: str, location: str) -> list[dict]:
    issues = _check_uint(value, field, location)
    int_value = _coerce_int(value)
    if int_value is not None and int_value > 65535:
        issues.append(_issue("warning", "range", location, field,
                             f"{field}={int_value} exceeds uint16 maximum 65535",
                             int_value, "uint16 0..65535",
                             rule_id="ICD-CODE-MISMATCH",
                             source="ICD 0304 LAHWaypoint vs generated waypoint ID allocator"))
    return issues


def _check_uint64(value: Any, field: str, location: str) -> list[dict]:
    """Value must be a non-negative integer (uint64)."""
    issues: list[dict] = []
    if value is None:
        return issues
    expected = f"uint64 0..{_UINT64_MAX}"
    if isinstance(value, bool):
        issues.append(_issue("error", "type", location, field,
                             f"{field} must be uint64, got bool({value})",
                             value, expected))
        return issues
    if isinstance(value, float):
        if value >= 0 and value == int(value):
            issues.append(_issue("warning", "type", location, field,
                                 f"{field} must be uint64, got float({value})",
                                 value, expected))
            if value > _UINT64_MAX:
                issues.append(_issue("error", "range", location, field,
                                     f"{field}={value} exceeds uint64 maximum {_UINT64_MAX}",
                                     value, expected))
        else:
            issues.append(_issue("error", "type", location, field,
                                 f"{field} must be uint64, got float({value})",
                                 value, expected))
        return issues
    if not isinstance(value, int):
        issues.append(_issue("error", "type", location, field,
                             f"{field} must be uint64, got {type(value).__name__}({value})",
                             value, expected))
        return issues
    if value < 0:
        issues.append(_issue("error", "range", location, field,
                             f"{field}={value} is below uint64 minimum 0",
                             value, expected))
    if value > _UINT64_MAX:
        issues.append(_issue("error", "range", location, field,
                             f"{field}={value} exceeds uint64 maximum {_UINT64_MAX}",
                             value, expected))
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


def _check_byte(value: Any, field: str, location: str) -> list[dict]:
    issues = _check_uint(value, field, location)
    int_val = _coerce_int(value)
    if int_val is not None and int_val > 255:
        issues.append(_issue("error", "range", location, field,
                             f"{field}={int_val} exceeds byte maximum 255",
                             int_val, "byte 0..255"))
    return issues


def _check_timestamp_531xx(value: Any, field: str, location: str) -> list[dict]:
    """53111/53112 timestamp is byte[5] on wire, but JSON logs often store ms int."""
    issues: list[dict] = []
    if value is None:
        return issues
    if isinstance(value, list):
        if len(value) != 5:
            issues.append(_issue("error", "structure", location, field,
                                 f"{field} byte array length is {len(value)}, expected 5",
                                 len(value), "byte[5]"))
        for idx, item in enumerate(value):
            issues.extend(_check_byte(item, f"{field}[{idx}]", location))
        return issues
    issues.extend(_check_uint64(value, field, location))
    return issues


def _check_bool(value: Any, field: str, location: str) -> list[dict]:
    """Value must be a real JSON bool, not 0/1 or string."""
    issues: list[dict] = []
    if value is None:
        return issues
    if not isinstance(value, bool):
        issues.append(_issue("error", "type", location, field,
                             f"{field} must be bool, got {type(value).__name__}({value})",
                             value, "bool"))
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


def _check_list(value: Any, field: str, location: str, *, required: bool = False, min_len: int = 0) -> list[dict]:
    """Value must be a list, with optional required/min length checks."""
    issues: list[dict] = []
    if value is None:
        if required:
            issues.append(_issue("error", "structure", location, field,
                                 f"{field} is required but missing", None, f"list length >= {min_len}"))
        return issues
    if not isinstance(value, list):
        issues.append(_issue("error", "type", location, field,
                             f"{field} must be list, got {type(value).__name__}",
                             value, "list"))
        return issues
    if len(value) < min_len:
        issues.append(_issue("error", "structure", location, field,
                             f"{field} has {len(value)} items, expected at least {min_len}",
                             len(value), f"list length >= {min_len}"))
    return issues


def _require_present(
    value: Any,
    field: str,
    location: str,
    issues: list[dict],
    expected: Any,
    *,
    rule_id: str | None = None,
    source: str | None = None,
) -> None:
    if value is None:
        issues.append(_issue("error", "structure", location, field,
                             f"{field} is required but missing",
                             None, expected,
                             rule_id=rule_id,
                             source=source))


def _check_enum(value: Any, field: str, location: str, valid: set[int], labels: dict[int, str] | None = None) -> list[dict]:
    """Value must be an int in valid set."""
    issues: list[dict] = []
    if value is None:
        return issues
    expected = f"enum {sorted(valid)}"
    if labels:
        expected += " (" + ", ".join(f"{k}={v}" for k, v in sorted(labels.items())) + ")"
    if isinstance(value, bool):
        issues.append(_issue("error", "type", location, field,
                             f"{field} must be integer enum, got bool({value})",
                             value, expected))
        return issues
    if isinstance(value, float):
        if value == int(value):
            issues.append(_issue("warning", "type", location, field,
                                 f"{field} must be integer enum, got float({value})",
                                 value, expected))
            int_value = int(value)
        else:
            issues.append(_issue("error", "type", location, field,
                                 f"{field} must be integer enum, got float({value})",
                                 value, expected))
            return issues
    elif isinstance(value, int):
        int_value = value
    else:
        issues.append(_issue("error", "type", location, field,
                             f"{field} must be integer enum, got {type(value).__name__}({value})",
                             value, expected))
        return issues
    if int_value not in valid:
        issues.append(_issue("error", "range", location, field,
                             f"{field}={int_value} is not a valid enum value",
                             int_value, expected))
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
        issues.append(_issue("error", "type", location, "coordinate",
                             f"coordinate must be object, got {type(coord).__name__}",
                             coord, "Coordinate object"))
        return issues

    lat = _pick(coord, "latitude", "Latitude")
    lon = _pick(coord, "longitude", "Longitude")
    alt = _pick(coord, "altitude", "Altitude")

    for field, value, expected in (
        ("latitude", lat, "float -90..90"),
        ("longitude", lon, "float -180..180"),
        ("altitude", alt, "int 0..50000"),
    ):
        if value is None:
            issues.append(_issue("error", "structure", location, field,
                                 f"Coordinate.{field} is required but missing",
                                 None, expected))

    issues.extend(_check_float(lat, "latitude", location, -90.0, 90.0))
    issues.extend(_check_float(lon, "longitude", location, -180.0, 180.0))
    issues.extend(_check_icd_int(alt, "altitude", location, 0, 50000))

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


def _validate_waypoint(
    wp: dict,
    idx: int,
    location_prefix: str,
    aircraft_id: int | None = None,
    target_ids: set[int] | None = None,
) -> list[dict]:
    """Validate a single waypoint dict."""
    issues: list[dict] = []
    if not isinstance(wp, dict):
        return issues

    wp_id = _pick(wp, "waypointID", "WaypointID")
    loc = f"{location_prefix} > WP {wp_id if wp_id is not None else idx}"

    # waypointID: uint
    _require_present(wp_id, "waypointID", loc, issues, "uint")
    if aircraft_id is not None and 1 <= aircraft_id <= 3:
        issues.extend(_check_uint16(wp_id, "waypointID", loc))
    else:
        issues.extend(_check_uint(wp_id, "waypointID", loc))

    # speed: float 0-1000
    speed = _pick(wp, "speed", "Speed")
    _require_present(speed, "speed", loc, issues, "float 0..1000")
    issues.extend(_check_float(speed, "speed", loc, 0.0, 1000.0))
    if speed is not None:
        try:
            if float(speed) <= 0.0 and _pick(wp, "nextWaypointID", "NextWaypointID") not in (0, None):
                issues.append(_issue("warning", "rule", loc, "speed",
                                     "non-terminal waypoint speed should be positive",
                                     speed, "float > 0"))
        except Exception:
            pass

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
    eta = _pick(wp, "eta", "ETA")
    _require_present(eta, "eta", loc, issues, "uint seconds")
    issues.extend(_check_uint(eta, "eta", loc))

    ecf = _pick(wp, "ecf", "ECF")
    _require_present(ecf, "ecf", loc, issues, "float 0..1000")
    issues.extend(_check_float(ecf, "ecf", loc, 0.0, 1000.0))

    # waypointPassType: enum 0-3
    pass_type = _pick(wp, "waypointPassType", "WaypointPassType")
    if aircraft_id is not None and 4 <= aircraft_id <= 6:
        _require_present(pass_type, "waypointPassType", loc, issues, "uint enum 0..3")
    issues.extend(_check_enum(pass_type, "waypointPassType", loc, {0, 1, 2, 3}, _WAYPOINT_PASS_TYPE_LABELS))

    # nextWaypointID: uint
    next_wp = _pick(wp, "nextWaypointID", "NextWaypointID")
    _require_present(next_wp, "nextWaypointID", loc, issues, "uint")
    if aircraft_id is not None and 1 <= aircraft_id <= 3:
        issues.extend(_check_uint16(next_wp, "nextWaypointID", loc))
    else:
        issues.extend(_check_uint(next_wp, "nextWaypointID", loc))

    is_done = _pick(wp, "isDone", "IsDone")
    if is_done is not None:
        issues.extend(_check_bool(is_done, "isDone", loc))

    # Coordinate validation
    coord = _pick(wp, "coordinate", "Coordinate")
    if isinstance(coord, dict):
        issues.extend(_validate_coordinate(coord, loc))
    else:
        issues.append(_issue("error", "structure", loc, "coordinate",
                             "Waypoint.coordinate is required", coord, "Coordinate"))

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
    filming_planned = _pick(wp, "filmingPlanned", "FilmingPlanned")
    if filming_planned is not None:
        issues.extend(_check_bool(filming_planned, "filmingPlanned", loc))

    filming = _pick(wp, "filmingProperty", "FilmingProperty", "filming", "Filming")
    if filming_planned is True and not isinstance(filming, dict):
        issues.append(_issue("error", "structure", loc, "filmingProperty",
                             "filmingPlanned=true but filmingProperty is missing",
                             None, "FilmingProperty"))
    if filming_planned is False and isinstance(filming, dict) and _is_meaningful(filming):
        issues.append(_issue("warning", "rule", loc, "filmingProperty",
                             "filmingPlanned=false but filmingProperty has data",
                             None, "no filmingProperty or filmingPlanned=true"))
    if isinstance(filming, dict):
        _validate_filming_property(filming, loc, wp, issues, target_ids)

    hovering = _pick(wp, "hovering", "Hovering")
    if isinstance(hovering, dict):
        hover_time = _pick(hovering, "time", "Time")
        if hover_time is None:
            issues.append(_issue("error", "structure", loc, "hovering.time",
                                 "hovering object requires time", None, "uint64"))
        else:
            issues.extend(_check_uint64(hover_time, "hovering.time", loc))
    elif hovering is not None:
        issues.extend(_check_uint(hovering, "hovering", loc))

    attack = _pick(wp, "attack", "Attack")
    if isinstance(attack, dict):
        target_id = _pick(attack, "targetID", "TargetID", "targetId")
        weapon_type = _pick(attack, "weaponType", "WeaponType")
        tid = _coerce_int(target_id)
        weapon = _coerce_int(weapon_type)
        active_attack = (tid or 0) > 0 or (weapon or 0) > 0
        if target_id is None:
            issues.append(_issue("error", "structure", loc, "attack.targetID",
                                 "attack object requires targetID", None, "uint > 0"))
        else:
            issues.extend(_check_uint(target_id, "attack.targetID", loc))
            if active_attack and tid is not None:
                _validate_target_reference(target_id, "attack.targetID", loc, issues, target_ids, "Attack")
        if weapon_type is None:
            issues.append(_issue("error", "structure", loc, "attack.weaponType",
                                 "attack object requires weaponType", None, "uint enum 0..3"))
        else:
            issues.extend(_check_enum(weapon_type, "attack.weaponType", loc, {0, 1, 2, 3}))
        if active_attack:
            if tid is None or tid <= 0:
                issues.append(_issue("error", "rule", loc, "attack.targetID",
                                     "attack data is active but targetID is not positive",
                                     target_id, "uint > 0"))
            if weapon is None or weapon <= 0:
                issues.append(_issue("error", "rule", loc, "attack.weaponType",
                                     "attack data is active but weaponType is not positive",
                                     weapon_type, "1..3"))

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


def _validate_coordinate_list(
    coords: Any,
    field: str,
    loc: str,
    issues: list[dict],
    *,
    min_len: int = 1,
) -> None:
    issues.extend(_check_list(coords, field, loc, required=True, min_len=min_len))
    if not isinstance(coords, list):
        return
    for i, coord in enumerate(coords):
        issues.extend(_validate_coordinate(coord, f"{loc} > {field}[{i}]"))


def _validate_declared_count(
    declared: Any,
    actual: Any,
    field: str,
    loc: str,
    issues: list[dict],
    *,
    required: bool = False,
) -> None:
    if declared is None:
        if required:
            issues.append(_issue("error", "structure", loc, field,
                                 f"{field} is required but missing",
                                 None, "uint list length"))
        return
    issues.extend(_check_uint(declared, field, loc))
    declared_int = _coerce_int(declared)
    actual_len = len(actual) if isinstance(actual, list) else 0
    if declared_int is not None and declared_int != actual_len:
        issues.append(_issue("error", "rule", loc, field,
                             f"{field}={declared_int} but list length is {actual_len}",
                             declared_int, f"{actual_len}"))


def _validate_coordinate_orientation(obj: Any, loc: str, issues: list[dict]) -> None:
    if not isinstance(obj, dict):
        issues.append(_issue("error", "type", loc, "coordinateOrientation",
                             f"coordinateOrientation must be object, got {type(obj).__name__}",
                             obj, "object with coordinate"))
        return
    coord = _pick(obj, "coordinate", "Coordinate")
    if coord is None:
        issues.append(_issue("error", "structure", loc, "coordinateOrientation.coordinate",
                             "operationMode=1 requires coordinateOrientation.coordinate",
                             None, "Coordinate"))
    else:
        issues.extend(_validate_coordinate(coord, f"{loc} > coordinateOrientation"))


def _validate_line_search(obj: Any, loc: str, issues: list[dict]) -> None:
    if not isinstance(obj, dict):
        issues.append(_issue("error", "type", loc, "lineSearch",
                             f"lineSearch must be object, got {type(obj).__name__}",
                             obj, "object with coordinateList/searchSpeed"))
        return
    coords = _pick(obj, "coordinateList", "CoordinateList")
    _validate_coordinate_list(coords, "lineSearch.coordinateList", loc, issues, min_len=2)
    search_speed = _pick(obj, "searchSpeed", "SearchSpeed")
    if search_speed is None:
        issues.append(_issue("error", "structure", loc, "lineSearch.searchSpeed",
                             "operationMode=2 requires lineSearch.searchSpeed",
                             None, "float 0..1000"))
    else:
        issues.extend(_check_float(search_speed, "lineSearch.searchSpeed", loc, 0.0, 1000.0))
        try:
            if float(search_speed) <= 0.0:
                issues.append(_issue("warning", "rule", loc, "lineSearch.searchSpeed",
                                     "lineSearch.searchSpeed should be positive for active search",
                                     search_speed, "float > 0"))
        except Exception:
            pass


def _validate_auto_tracking(obj: Any, loc: str, issues: list[dict], target_ids: set[int] | None = None) -> None:
    if not isinstance(obj, dict):
        issues.append(_issue("error", "type", loc, "autoTracking",
                             f"autoTracking must be object, got {type(obj).__name__}",
                             obj, "object with targetID"))
        return
    target_id = _pick(obj, "targetID", "TargetID", "targetId")
    if target_id is None:
        issues.append(_issue("error", "structure", loc, "autoTracking.targetID",
                             "operationMode=3 requires autoTracking.targetID",
                             None, "uint > 0"))
        return
    issues.extend(_check_uint(target_id, "autoTracking.targetID", loc))
    tid = _coerce_int(target_id)
    if tid is not None:
        _validate_target_reference(target_id, "autoTracking.targetID", loc, issues, target_ids, "Auto Track")


def _validate_target_reference(
    target_id: Any,
    field: str,
    loc: str,
    issues: list[dict],
    target_ids: set[int] | None,
    context: str,
) -> None:
    tid = _coerce_int(target_id)
    if tid is None:
        return
    if tid <= 0:
        issues.append(_issue("error", "rule", loc, field,
                             f"{context} targetID must be positive, got {tid}",
                             tid, "uint > 0"))
    elif target_ids is None:
        issues.append(_issue("error", "rule", loc, field,
                             f"{context} targetID {tid} cannot be verified because TargetInfo/0402/52311 source is missing",
                             tid, "TargetInfo source containing targetID",
                             rule_id="TARGET-REF",
                             source="ICD target reference + TargetInfo"))
    elif tid not in target_ids:
        issues.append(_issue("error", "rule", loc, field,
                             f"{context} targetID {tid} is not present in TargetInfo",
                             tid, f"one of {sorted(target_ids)[:20]}"))


def _validate_aircraft_fixed(obj: Any, loc: str, issues: list[dict]) -> None:
    if not isinstance(obj, dict):
        issues.append(_issue("error", "type", loc, "aircraftFixed",
                             f"aircraftFixed must be object, got {type(obj).__name__}",
                             obj, "object with gimbalPitch/gimbalYaw"))
        return
    pitch = _pick(obj, "gimbalPitch", "GimbalPitch", "sensorPitch", "SensorPitch")
    yaw = _pick(obj, "gimbalYaw", "GimbalYaw", "sensorYaw", "SensorYaw")
    if pitch is None:
        issues.append(_issue("warning", "structure", loc, "aircraftFixed.gimbalPitch",
                             "operationMode=4 normally carries fixed gimbal pitch",
                             None, "float -90..90"))
    else:
        issues.extend(_check_float(pitch, "aircraftFixed.gimbalPitch", loc, -90.0, 90.0))
    if yaw is None:
        issues.append(_issue("warning", "structure", loc, "aircraftFixed.gimbalYaw",
                             "operationMode=4 normally carries fixed gimbal yaw",
                             None, "float -180..180"))
    else:
        issues.extend(_check_float(yaw, "aircraftFixed.gimbalYaw", loc, -180.0, 180.0))


def _validate_auto_scan(obj: Any, loc: str, issues: list[dict]) -> None:
    if not isinstance(obj, dict):
        issues.append(_issue("error", "type", loc, "autoScan",
                             f"autoScan must be object, got {type(obj).__name__}",
                             obj, "object with gimbalPitch/gimbalYawLimits/gimbalYawAngularSpeed"))
        return
    pitch = _pick(obj, "gimbalPitch", "GimbalPitch", "sensorPitch", "SensorPitch")
    if pitch is not None:
        issues.extend(_check_float(pitch, "autoScan.gimbalPitch", loc, -90.0, 90.0))
    limits = _pick(obj, "gimbalYawLimits", "GimbalYawLimits")
    if isinstance(limits, dict):
        left = _pick(limits, "leftLimit", "LeftLimit")
        right = _pick(limits, "rightLimit", "RightLimit")
        issues.extend(_check_float(left, "autoScan.gimbalYawLimits.leftLimit", loc, 0.0, 90.0))
        issues.extend(_check_float(right, "autoScan.gimbalYawLimits.rightLimit", loc, 0.0, 90.0))
    speed = _pick(
        obj,
        "gimbalYawAngularSpeed",
        "GimbalYawAngularSpeed",
        "sensorYawAngularSpeed",
        "SensorYawAngularSpeed",
        "angularRate",
        "AngularRate",
    )
    if speed is not None:
        if isinstance(speed, dict):
            _validate_sensor_yaw_angular_speed(speed, loc, issues, "autoScan.sensorYawAngularSpeed")
        else:
            issues.extend(_check_float(speed, "autoScan.gimbalYawAngularSpeed", loc, -180.0, 180.0))


def _validate_sensor_yaw_angular_speed(obj: Any, loc: str, issues: list[dict], field_prefix: str) -> None:
    if not isinstance(obj, dict):
        issues.append(_issue("error", "type", loc, field_prefix,
                             f"{field_prefix} must be object, got {type(obj).__name__}",
                             obj, "object with leftLimit/rightLimit/angularRate"))
        return
    for field, lo, hi in (
        ("leftLimit", 0.0, 90.0),
        ("rightLimit", 0.0, 90.0),
        ("angularRate", -180.0, 180.0),
    ):
        value = _pick(obj, field, field[0].upper() + field[1:])
        if value is None:
            issues.append(_issue("error", "structure", loc, f"{field_prefix}.{field}",
                                 f"{field_prefix}.{field} is required",
                                 None, f"float {lo}..{hi}"))
        else:
            issues.extend(_check_float(value, f"{field_prefix}.{field}", loc, lo, hi))


def _warn_forbidden_filming_fields(
    filming: dict,
    loc: str,
    issues: list[dict],
    allowed: set[str],
) -> None:
    field_aliases = {
        "coordinateOrientation": ("coordinateOrientation", "CoordinateOrientation"),
        "lineSearch": ("lineSearch", "LineSearch"),
        "autoTracking": ("autoTracking", "AutoTracking"),
        "aircraftFixed": ("aircraftFixed", "AircraftFixed"),
        "autoScan": ("autoScan", "AutoScan"),
        "areaSearch": ("areaSearch", "AreaSearch"),
    }
    for canonical, aliases in field_aliases.items():
        if canonical in allowed:
            continue
        for alias in aliases:
            if alias in filming and _is_meaningful(filming.get(alias)):
                severity = "warning"
                message = f"{canonical} is not used by this operationMode"
                if canonical == "areaSearch":
                    message = "areaSearch is not an ICD field for 0303 FilmingProperty"
                issues.append(_issue(severity, "rule", loc, alias,
                                     message, None, f"allowed: {sorted(allowed)}"))
                break


def _flat_scalar_has_value(value: Any) -> bool:
    if value is None:
        return False
    if isinstance(value, bool):
        return True
    if isinstance(value, (int, float)):
        return value != 0
    return _is_meaningful(value)


def _flat_coordinate_list_has_value(filming: dict) -> bool:
    count = _coerce_int(_pick(filming, "coordinateListN", "CoordinateListN"))
    if count is not None and count > 0:
        return True
    return _is_meaningful(_pick(filming, "coordinateList", "CoordinateList"))


def _validate_flat_coordinate_count(filming: dict, loc: str, issues: list[dict], *, field_prefix: str = "filmingProperty") -> None:
    coords = _pick(filming, "coordinateList", "CoordinateList", default=_MISSING)
    declared = _pick(filming, "coordinateListN", "CoordinateListN", default=_MISSING)
    if coords is _MISSING and declared is _MISSING:
        return
    if declared is _MISSING:
        issues.append(_issue("error", "structure", loc, f"{field_prefix}.coordinateListN",
                             "coordinateList is present but coordinateListN is missing",
                             None, "uint list length"))
        return
    _validate_declared_count(declared, None if coords is _MISSING else coords,
                             f"{field_prefix}.coordinateListN", loc, issues)


def _validate_flat_filming_basics(filming: dict, loc: str, issues: list[dict], op_mode: int | None) -> None:
    coordinate_count = _pick(filming, "coordinateListN", "CoordinateListN")
    if coordinate_count is not None and op_mode not in (1, 2):
        issues.extend(_check_uint(coordinate_count, "filmingProperty.coordinateListN", loc))
    search_speed = _pick(filming, "searchSpeed", "SearchSpeed")
    if search_speed is not None and op_mode != 2:
        issues.extend(_check_float(search_speed, "filmingProperty.searchSpeed", loc, 0.0, 1000.0))
    target_id = _pick(filming, "targetID", "TargetID", "targetId")
    if target_id is not None and op_mode != 3:
        issues.extend(_check_uint(target_id, "filmingProperty.targetID", loc))
    sensor_yaw = _pick(filming, "sensorYaw", "SensorYaw")
    if sensor_yaw is not None and op_mode != 4:
        issues.extend(_check_float(sensor_yaw, "filmingProperty.sensorYaw", loc, -180.0, 180.0))
    sensor_pitch = _pick(filming, "sensorPitch", "SensorPitch")
    if sensor_pitch is not None and op_mode not in (4, 5):
        issues.extend(_check_float(sensor_pitch, "filmingProperty.sensorPitch", loc, -90.0, 90.0))
    angular = _pick(filming, "sensorYawAngularSpeed", "SensorYawAngularSpeed")
    if angular is not None and op_mode != 5:
        _validate_sensor_yaw_angular_speed(angular, loc, issues, "filmingProperty.sensorYawAngularSpeed")


def _warn_forbidden_flat_filming_fields(filming: dict, loc: str, issues: list[dict], op_mode: int | None) -> None:
    if op_mode is None:
        return
    forbidden_by_mode = {
        1: ("searchSpeed", "targetID", "sensorYaw", "sensorPitch", "sensorYawAngularSpeed"),
        2: ("targetID", "sensorYaw", "sensorPitch", "sensorYawAngularSpeed"),
        3: ("coordinateList", "searchSpeed", "sensorYaw", "sensorPitch", "sensorYawAngularSpeed"),
        4: ("coordinateList", "searchSpeed", "targetID", "sensorYawAngularSpeed"),
        5: ("coordinateList", "searchSpeed", "targetID", "sensorYaw"),
    }
    for field in forbidden_by_mode.get(op_mode, ()):
        if field == "coordinateList":
            has_value = _flat_coordinate_list_has_value(filming)
            value = _pick(filming, "coordinateListN", "CoordinateListN", default=_pick(filming, "coordinateList", "CoordinateList"))
        else:
            value = _pick(filming, field, field[0].upper() + field[1:])
            has_value = _flat_scalar_has_value(value)
        if not has_value:
            continue
        issues.append(_issue(
            "warning",
            "rule",
            loc,
            f"filmingProperty.{field}",
            f"{field} carries data but operationMode={op_mode} does not use it",
            value,
            "null/0/empty for inactive filming mode fields",
            rule_id="FILMING-MODE",
            source="ICD 53112 flat FilmingProperty",
        ))


def _legacy_validate_filming_property(
    filming: dict,
    loc: str,
    wp: dict,
    issues: list[dict],
    target_ids: set[int] | None = None,
) -> None:
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


def _validate_filming_property(
    filming: dict,
    loc: str,
    wp: dict,
    issues: list[dict],
    target_ids: set[int] | None = None,
) -> None:
    """Strict ICD-aware FilmingProperty validation.

    Supports both 0303 nested CommonType fields and 53112 flattened fields.
    """
    fov = _pick(filming, "fieldOfView", "FieldOfView")
    if fov is None:
        issues.append(_issue("error", "structure", loc, "filmingProperty.fieldOfView",
                             "filmingProperty.fieldOfView is required", None, "float 0..180"))
    issues.extend(_check_float(fov, "filmingProperty.fieldOfView", loc, 0.0, 180.0))

    sensor = _pick(filming, "sensorType", "SensorType")
    if sensor is None:
        issues.append(_issue("error", "structure", loc, "filmingProperty.sensorType",
                             "filmingProperty.sensorType is required", None, "uint enum 0..2"))
    issues.extend(_check_enum(sensor, "filmingProperty.sensorType", loc, {0, 1, 2}, _SENSOR_TYPE_LABELS))

    op_mode = _pick(filming, "operationMode", "OperationMode")
    if op_mode is None:
        issues.append(_issue("error", "structure", loc, "filmingProperty.operationMode",
                             "filmingProperty.operationMode is required", None, "uint enum 0..5"))
    issues.extend(_check_enum(op_mode, "filmingProperty.operationMode", loc, {0, 1, 2, 3, 4, 5}))

    op_mode_int = _coerce_int(op_mode)
    flat_filming = any(
        key in filming
        for key in (
            "coordinateListN",
            "CoordinateListN",
            "coordinateList",
            "CoordinateList",
            "searchSpeed",
            "SearchSpeed",
            "targetID",
            "TargetID",
            "targetId",
            "sensorYaw",
            "SensorYaw",
            "sensorPitch",
            "SensorPitch",
            "sensorYawAngularSpeed",
            "SensorYawAngularSpeed",
        )
    )
    if flat_filming:
        _validate_flat_filming_basics(filming, loc, issues, op_mode_int)

    if op_mode_int == 1:
        coord_orient = _pick(filming, "coordinateOrientation", "CoordinateOrientation", default=_MISSING)
        flat_coords = _pick(filming, "coordinateList", "CoordinateList", default=_MISSING)
        if coord_orient is _MISSING:
            coords = flat_coords
            coord_orient = coords if coords is not _MISSING else _pick(wp, "coordinateOrientation", "CoordinateOrientation", default=_MISSING)
        if coord_orient is _MISSING:
            issues.append(_issue("error", "rule", loc, "coordinateOrientation",
                                 "operationMode=1 requires coordinateOrientation or flat coordinateList",
                                 None, "coordinateOrientation.coordinate or coordinateList"))
        elif isinstance(coord_orient, dict):
            _validate_coordinate_orientation(coord_orient, loc, issues)
        else:
            if flat_coords is not _MISSING:
                _validate_flat_coordinate_count(filming, loc, issues)
            _validate_coordinate_list(coord_orient, "filmingProperty.coordinateList", loc, issues, min_len=1)
        _warn_forbidden_filming_fields(filming, loc, issues, {"coordinateOrientation"})
        if flat_filming:
            _warn_forbidden_flat_filming_fields(filming, loc, issues, op_mode_int)

    elif op_mode_int == 2:
        line_search = _pick(filming, "lineSearch", "LineSearch", default=_MISSING)
        if line_search is _MISSING:
            coords = _pick(filming, "coordinateList", "CoordinateList", default=_MISSING)
            if coords is not _MISSING:
                _validate_flat_coordinate_count(filming, loc, issues)
                line_search = {
                    "coordinateList": coords,
                    "searchSpeed": _pick(filming, "searchSpeed", "SearchSpeed"),
                }
            else:
                line_search = _pick(wp, "lineSearch", "LineSearch", default=_MISSING)
        if line_search is _MISSING:
            issues.append(_issue("error", "rule", loc, "lineSearch",
                                 "operationMode=2 requires lineSearch or flat coordinateList/searchSpeed",
                                 None, "lineSearch.coordinateList + searchSpeed"))
        else:
            _validate_line_search(line_search, loc, issues)
        _warn_forbidden_filming_fields(filming, loc, issues, {"lineSearch"})
        if flat_filming:
            _warn_forbidden_flat_filming_fields(filming, loc, issues, op_mode_int)

    elif op_mode_int == 3:
        auto_track = _pick(filming, "autoTracking", "AutoTracking", default=_MISSING)
        if auto_track is _MISSING:
            target_id = _pick(filming, "targetID", "TargetID", "targetId", default=_MISSING)
            auto_track = {"targetID": target_id} if target_id is not _MISSING else _pick(wp, "autoTracking", "AutoTracking", default=_MISSING)
        if auto_track is _MISSING:
            issues.append(_issue("error", "rule", loc, "autoTracking",
                                 "operationMode=3 requires autoTracking.targetID or flat targetID",
                                 None, "targetID uint > 0"))
        else:
            _validate_auto_tracking(auto_track, loc, issues, target_ids)
        _warn_forbidden_filming_fields(filming, loc, issues, {"autoTracking"})
        if flat_filming:
            _warn_forbidden_flat_filming_fields(filming, loc, issues, op_mode_int)

    elif op_mode_int == 4:
        fixed = _pick(filming, "aircraftFixed", "AircraftFixed", default=_MISSING)
        if fixed is _MISSING:
            pitch = _pick(filming, "sensorPitch", "SensorPitch", default=_MISSING)
            yaw = _pick(filming, "sensorYaw", "SensorYaw", default=_MISSING)
            if pitch is not _MISSING or yaw is not _MISSING:
                fixed = {
                    "sensorPitch": None if pitch is _MISSING else pitch,
                    "sensorYaw": None if yaw is _MISSING else yaw,
                }
        if fixed is _MISSING:
            issues.append(_issue("warning", "rule", loc, "aircraftFixed",
                                 "operationMode=4 should carry aircraftFixed or flat sensorYaw/sensorPitch",
                                 None, "aircraftFixed or sensorYaw/sensorPitch"))
        else:
            _validate_aircraft_fixed(fixed, loc, issues)
        _warn_forbidden_filming_fields(filming, loc, issues, {"aircraftFixed"})
        if flat_filming:
            _warn_forbidden_flat_filming_fields(filming, loc, issues, op_mode_int)

    elif op_mode_int == 5:
        scan = _pick(filming, "autoScan", "AutoScan", default=_MISSING)
        if scan is _MISSING:
            angular = _pick(filming, "sensorYawAngularSpeed", "SensorYawAngularSpeed", default=_MISSING)
            pitch = _pick(filming, "sensorPitch", "SensorPitch", default=_MISSING)
            if angular is not _MISSING or pitch is not _MISSING:
                scan = {
                    "sensorPitch": None if pitch is _MISSING else pitch,
                    "gimbalYawAngularSpeed": angular,
                }
        if scan is _MISSING:
            issues.append(_issue("warning", "rule", loc, "autoScan",
                                 "operationMode=5 should carry autoScan or flat sensorYawAngularSpeed",
                                 None, "autoScan"))
        else:
            _validate_auto_scan(scan, loc, issues)
        _warn_forbidden_filming_fields(filming, loc, issues, {"autoScan"})
        if flat_filming:
            _warn_forbidden_flat_filming_fields(filming, loc, issues, op_mode_int)


# ---------------------------------------------------------------------------
# Flight path validation
# ---------------------------------------------------------------------------

def _legacy_validate_flight_path(path_key: str, raw_data: dict, aircraft_id_hint: int | None = None) -> list[dict]:
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
    issues.extend(_validate_waypoint_sequence_timing(waypoints, loc_base))

    return issues


def _validate_flight_path(
    path_key: str,
    raw_data: dict,
    aircraft_id_hint: int | None = None,
    target_ids: set[int] | None = None,
) -> list[dict]:
    """Strict ICD/code-aware FlightPath validation."""
    issues: list[dict] = []
    loc_base = f"FlightPath/{path_key}"

    path_id = _pick(raw_data, "pathID", "PathID")
    ac_id = _pick(raw_data, "aircraftID", "AircraftID")
    timestamp = _pick(raw_data, "timestamp", "Timestamp")

    if path_id is None:
        issues.append(_issue("error", "structure", loc_base, "pathID",
                             "pathID is required", None, "uint"))
    else:
        issues.extend(_check_uint(path_id, "pathID", loc_base))
        pid_int = _coerce_int(path_id)
        key_int = _coerce_int(path_key)
        if key_int is not None and pid_int is not None and key_int != pid_int:
            issues.append(_issue("error", "rule", loc_base, "pathID",
                                 f"file name pathID {key_int} does not match payload pathID {pid_int}",
                                 pid_int, f"pathID={key_int}"))

    if ac_id is None:
        issues.append(_issue("error", "structure", loc_base, "aircraftID",
                             "aircraftID is required", None, "uint enum 1..6"))
    else:
        issues.extend(_check_enum(ac_id, "aircraftID", loc_base, {1, 2, 3, 4, 5, 6}))

    if timestamp is not None:
        issues.extend(_check_uint64(timestamp, "timestamp", loc_base))

    aircraft_id = _coerce_int(ac_id) if ac_id is not None else aircraft_id_hint
    pid_int = _coerce_int(path_id)
    if aircraft_id is not None and pid_int is not None:
        band_base = aircraft_id * 100000000
        band_hi = band_base + 99999999
        if not (band_base < pid_int <= band_hi):
            issues.append(_issue("error", "rule", loc_base, "pathID",
                                 f"pathID {pid_int} is outside aircraft {aircraft_id} band",
                                 pid_int, f"{band_base + 1}..{band_hi}"))

    individual_mission_id = _pick(raw_data, "individualMissionID", "IndividualMissionID")
    if individual_mission_id is not None:
        issues.extend(_check_uint(individual_mission_id, "individualMissionID", loc_base))

    is_formation = _pick(raw_data, "isFormationFlight", "IsFormationFlight")
    if is_formation is not None:
        issues.extend(_check_bool(is_formation, "isFormationFlight", loc_base))

    waypoints: list[dict] = []
    waypoint_key = None
    for key in ("lahWaypointList", "uavWaypointList", "waypointList"):
        lst = raw_data.get(key)
        if isinstance(lst, list):
            waypoints = lst
            waypoint_key = key
            break
        if key in raw_data and raw_data.get(key) is not None:
            issues.append(_issue("error", "type", loc_base, key,
                                 f"{key} must be list", raw_data.get(key), "list"))

    if aircraft_id is not None:
        if 1 <= aircraft_id <= 3 and waypoint_key == "waypointList":
            issues.append(_issue("warning", "rule", loc_base, waypoint_key,
                                 "LAH generated paths normally use lahWaypointList",
                                 waypoint_key, "lahWaypointList"))
        if 4 <= aircraft_id <= 6 and waypoint_key == "lahWaypointList":
            issues.append(_issue("error", "rule", loc_base, waypoint_key,
                                 "UAV FlightPath must not use lahWaypointList",
                                 waypoint_key, "waypointList"))
        if 1 <= aircraft_id <= 3 and waypoint_key == "uavWaypointList":
            issues.append(_issue("error", "rule", loc_base, waypoint_key,
                                 "LAH FlightPath must not use uavWaypointList",
                                 waypoint_key, "lahWaypointList"))

    if not waypoints:
        formation_info = _pick(raw_data, "formationInfo", "FormationInfo")
        if is_formation is True and isinstance(formation_info, dict):
            _validate_formation_info(formation_info, loc_base, issues)
            return issues
        issues.append(_issue("error", "structure", loc_base, "waypointList",
                             "FlightPath waypoint list is missing or empty",
                             None, "waypointList/lahWaypointList"))
        return issues

    if len(waypoints) == 1:
        issues.append(_issue("warning", "rule", loc_base, "waypointList",
                             "generated paths are normally expanded to at least two waypoints",
                             len(waypoints), ">= 2 waypoints"))

    for i, wp in enumerate(waypoints):
        if not isinstance(wp, dict):
            issues.append(_issue("error", "type", loc_base, f"waypointList[{i}]",
                                 f"waypoint must be object, got {type(wp).__name__}",
                                 wp, "Waypoint object"))
            continue
        issues.extend(_validate_waypoint(wp, i, loc_base, aircraft_id, target_ids))

    issues.extend(_validate_waypoint_chain(waypoints, loc_base))
    issues.extend(_validate_waypoint_sequence_timing(waypoints, loc_base))
    return issues


def _legacy_validate_waypoint_chain(waypoints: list[dict], loc_base: str) -> list[dict]:
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


def _validate_waypoint_chain(waypoints: list[dict], loc_base: str) -> list[dict]:
    """Strict chain validation: duplicate IDs, ordered next pointers, cycles."""
    issues: list[dict] = []
    by_id: dict[int, dict] = {}
    ordered_ids: list[int] = []
    duplicate_ids: set[int] = set()

    for idx, wp in enumerate(waypoints):
        if not isinstance(wp, dict):
            continue
        wid = _pick(wp, "waypointID", "WaypointID")
        wid_int = _coerce_int(wid)
        if wid_int is None:
            continue
        if wid_int <= 0:
            issues.append(_issue("error", "rule", loc_base, "waypointID",
                                 f"waypointID must be positive, got {wid_int}",
                                 wid_int, "uint > 0"))
        if wid_int in by_id:
            duplicate_ids.add(wid_int)
        by_id[wid_int] = wp
        ordered_ids.append(wid_int)

    for wid in sorted(duplicate_ids):
        issues.append(_issue("error", "rule", loc_base, "waypointID",
                             f"duplicate waypointID {wid} in same FlightPath",
                             wid, "unique waypointID"))

    if not by_id:
        return issues

    terminal_count = 0
    for idx, wp in enumerate(waypoints):
        if not isinstance(wp, dict):
            continue
        wid = _coerce_int(_pick(wp, "waypointID", "WaypointID"))
        nxt = _pick(wp, "nextWaypointID", "NextWaypointID")
        nxt_int = _coerce_int(nxt)
        if nxt_int is None:
            continue
        if nxt_int == 0:
            terminal_count += 1
            if idx != len(waypoints) - 1:
                issues.append(_issue("warning", "rule", loc_base, "nextWaypointID",
                                     f"WP {wid} is terminal before end of waypoint list",
                                     nxt_int, "only last waypoint nextWaypointID=0"))
            continue
        if nxt_int not in by_id:
            issues.append(_issue("error", "rule", loc_base, "nextWaypointID",
                                 f"WP {wid} points to missing nextWaypointID {nxt_int}",
                                 nxt_int, "existing waypointID in same FlightPath"))
        if idx < len(waypoints) - 1:
            expected_next = _coerce_int(_pick(waypoints[idx + 1], "waypointID", "WaypointID")) if isinstance(waypoints[idx + 1], dict) else None
            if expected_next is not None and nxt_int != expected_next:
                issues.append(_issue("warning", "rule", loc_base, "nextWaypointID",
                                     f"WP {wid} nextWaypointID={nxt_int} does not match next list waypointID={expected_next}",
                                     nxt_int, f"{expected_next}"))

    if terminal_count == 0:
        issues.append(_issue("error", "rule", loc_base, "nextWaypointID",
                             "no terminal waypoint found (nextWaypointID=0)",
                             None, "one terminal waypoint"))
    elif terminal_count > 1:
        issues.append(_issue("warning", "rule", loc_base, "nextWaypointID",
                             f"multiple terminal waypoints found: {terminal_count}",
                             terminal_count, "one terminal waypoint"))

    pointed_to = {
        _coerce_int(_pick(wp, "nextWaypointID", "NextWaypointID"))
        for wp in waypoints
        if isinstance(wp, dict) and _coerce_int(_pick(wp, "nextWaypointID", "NextWaypointID")) not in (None, 0)
    }
    starts = [wid for wid in ordered_ids if wid not in pointed_to]
    if len(starts) == 0:
        issues.append(_issue("error", "rule", loc_base, "nextWaypointID",
                             "no chain start found; possible cycle",
                             None, "one start waypoint"))
    elif len(starts) > 1:
        issues.append(_issue("warning", "rule", loc_base, "nextWaypointID",
                             f"multiple chain starts found: {starts}",
                             starts, "one start waypoint"))

    return issues


def _ordered_waypoints_for_timing(waypoints: list[dict]) -> list[dict]:
    """Return waypoints in nextWaypointID order when a single chain is resolvable."""
    valid = [wp for wp in waypoints if isinstance(wp, dict)]
    by_id: dict[int, dict] = {}
    duplicate = False
    for wp in valid:
        wid = _coerce_int(_pick(wp, "waypointID", "WaypointID"))
        if wid is None:
            return valid
        if wid in by_id:
            duplicate = True
        by_id[wid] = wp
    if duplicate or not by_id:
        return valid

    pointed_to = {
        _coerce_int(_pick(wp, "nextWaypointID", "NextWaypointID"))
        for wp in valid
        if _coerce_int(_pick(wp, "nextWaypointID", "NextWaypointID")) not in (None, 0)
    }
    starts = [wid for wid in by_id if wid not in pointed_to]
    if len(starts) != 1:
        return valid

    ordered: list[dict] = []
    seen: set[int] = set()
    current = starts[0]
    while current in by_id and current not in seen:
        seen.add(current)
        wp = by_id[current]
        ordered.append(wp)
        next_id = _coerce_int(_pick(wp, "nextWaypointID", "NextWaypointID"))
        if next_id in (None, 0):
            break
        current = next_id

    if len(ordered) != len(valid):
        return valid
    return ordered


def _coordinate_distance_m(a: Any, b: Any) -> float | None:
    if not isinstance(a, dict) or not isinstance(b, dict):
        return None
    try:
        lat1 = float(_pick(a, "latitude", "Latitude"))
        lon1 = float(_pick(a, "longitude", "Longitude"))
        alt1 = float(_pick(a, "altitude", "Altitude", default=0) or 0)
        lat2 = float(_pick(b, "latitude", "Latitude"))
        lon2 = float(_pick(b, "longitude", "Longitude"))
        alt2 = float(_pick(b, "altitude", "Altitude", default=0) or 0)
    except Exception:
        return None

    radius_m = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    hav = (
        math.sin(dphi / 2.0) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2
    )
    ground = 2.0 * radius_m * math.asin(min(1.0, math.sqrt(hav)))
    dz = alt2 - alt1
    return math.sqrt(ground * ground + dz * dz)


def _validate_waypoint_sequence_timing(waypoints: list[dict], loc_base: str) -> list[dict]:
    """Validate cumulative ETA consistency across a waypoint chain."""
    issues: list[dict] = []
    ordered = _ordered_waypoints_for_timing(waypoints)
    previous_eta: int | None = None
    previous_wp: dict | None = None

    for idx, wp in enumerate(ordered):
        if not isinstance(wp, dict):
            continue
        wid = _pick(wp, "waypointID", "WaypointID", default=idx)
        eta = _pick(wp, "eta", "ETA")
        eta_int = _coerce_int(eta)
        loc = f"{loc_base} > WP {wid}"
        if eta_int is None:
            previous_wp = wp
            continue

        if idx == 0 and eta_int != 0:
            issues.append(_issue(
                "warning",
                "rule",
                loc,
                "eta",
                "first waypoint ETA is normally fixed to 0 by the ICD; non-zero values may indicate takeover/replan carry-over timing",
                eta,
                "0 for newly generated path start",
                rule_id="WP-TIMING",
                source="ICD 0303/0304/53111/53112 Waypoint.eta",
                hint="If this path starts from a takeover/replan continuation, confirm the ETA origin is intentionally inherited.",
            ))

        if previous_eta is not None:
            if eta_int < previous_eta:
                issues.append(_issue(
                    "error",
                    "rule",
                    loc,
                    "eta",
                    f"waypoint ETA decreases from {previous_eta} to {eta_int}",
                    eta,
                    f">= {previous_eta}",
                    rule_id="WP-TIMING",
                    source="ICD 0303/0304/53111/53112 Waypoint.eta",
                    hint="ETA is cumulative within one flight plan, so later waypoints should not arrive earlier than previous waypoints.",
                ))
            elif eta_int == previous_eta and previous_wp is not None:
                dist_m = _coordinate_distance_m(
                    _pick(previous_wp, "coordinate", "Coordinate"),
                    _pick(wp, "coordinate", "Coordinate"),
                )
                if dist_m is not None and dist_m > 1000.0:
                    issues.append(_issue(
                        "warning",
                        "rule",
                        loc,
                        "eta",
                        f"waypoint moves {dist_m:.1f} m but ETA does not increase",
                        eta,
                        f"> {previous_eta}",
                        rule_id="WP-TIMING",
                        source="ICD 0303/0304/53111/53112 Waypoint.eta",
                        hint="A long move with unchanged ETA usually means arrival timing was not recalculated after path generation or merge.",
                    ))

        previous_eta = eta_int
        previous_wp = wp

    return issues


# ---------------------------------------------------------------------------
# IndividualMissionPlan validation
# ---------------------------------------------------------------------------

def _legacy_validate_imp(imp_key: str, imp_data: dict, existing_path_ids: set[int]) -> list[dict]:
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


def _validate_related_mission(
    related: Any,
    loc: str,
    issues: list[dict],
    input_mission_ids: set[int] | None = None,
    prior_mission_ids: set[int] | None = None,
    known_individual_mission_ids: set[int] | None = None,
    previous_individual_mission_ids: set[int] | None = None,
    current_mission_id: int | None = None,
) -> None:
    if not isinstance(related, dict):
        issues.append(_issue("error", "structure", loc, "relatedMission",
                             "relatedMission object is required", related, "RelatedMission"))
        return
    rel_type = _pick(related, "relatedMissionType", "RelatedMissionType")
    input_id = _pick(related, "inputMissionID", "InputMissionID")
    prior_id = _pick(related, "priorMissionID", "PriorMissionID")
    issues.extend(_check_enum(rel_type, "relatedMission.relatedMissionType", loc, {1, 2}))
    if input_id is None:
        issues.append(_issue("error", "structure", loc, "relatedMission.inputMissionID",
                             "inputMissionID is required", None, "uint > 0"))
    else:
        issues.extend(_check_uint(input_id, "relatedMission.inputMissionID", loc))
        iid = _coerce_int(input_id)
        if iid is not None and iid <= 0:
            issues.append(_issue("error", "rule", loc, "relatedMission.inputMissionID",
                                 f"inputMissionID must be positive, got {iid}", iid, "uint > 0"))
        elif iid is not None and input_mission_ids and iid not in input_mission_ids:
            issues.append(_issue("error", "rule", loc, "relatedMission.inputMissionID",
                                 f"inputMissionID {iid} does not resolve to InputMissionPlan.inputMissionList",
                                 iid, f"one of {sorted(input_mission_ids)[:20]}",
                                 rule_id="INPUT-MISSION-REF",
                                 source="ICD 0201 InputMissionPlan + ICD 0302 RelatedMission",
                                 hint="Check that the IMP relatedMission.inputMissionID matches an inputMissionID in the referenced InputMissionPlan."))
    rel_type_int = _coerce_int(rel_type)
    prior_int = _coerce_int(prior_id)
    if rel_type_int == 1:
        if prior_int not in (None, 0):
            issues.append(_issue("warning", "rule", loc, "relatedMission.priorMissionID",
                                 "priorMissionID should be 0 for input-mission related tasks",
                                 prior_id, "0"))
    elif rel_type_int == 2:
        if prior_id is None:
            issues.append(_issue("error", "structure", loc, "relatedMission.priorMissionID",
                                 "priorMissionID is required when relatedMissionType=2",
                                 None, "uint > 0"))
        else:
            issues.extend(_check_uint(prior_id, "relatedMission.priorMissionID", loc))
            if prior_int is not None and prior_int <= 0:
                issues.append(_issue("error", "rule", loc, "relatedMission.priorMissionID",
                                     f"priorMissionID must be positive for relatedMissionType=2, got {prior_int}",
                                     prior_int, "uint > 0"))
            elif prior_int is not None:
                known_individual_mission_ids = known_individual_mission_ids or set()
                previous_individual_mission_ids = previous_individual_mission_ids or set()
                if prior_int in known_individual_mission_ids:
                    if current_mission_id is not None and prior_int == current_mission_id:
                        issues.append(_issue("error", "rule", loc, "relatedMission.priorMissionID",
                                             "priorMissionID self-references the current individualMissionID",
                                             prior_int, "a previous individualMissionID",
                                             rule_id="PRIOR-MISSION-REF",
                                             source="ICD 0302 RelatedMission"))
                    elif prior_int not in previous_individual_mission_ids:
                        issues.append(_issue("error", "rule", loc, "relatedMission.priorMissionID",
                                             "priorMissionID references an individualMissionID that is not earlier in this IMP",
                                             prior_int, "an earlier individualMissionID in the same IMP",
                                             rule_id="PRIOR-MISSION-REF",
                                             source="ICD 0302 RelatedMission + mission sequence invariant"))
                elif prior_mission_ids is not None:
                    if prior_int not in prior_mission_ids:
                        issues.append(_issue("error", "rule", loc, "relatedMission.priorMissionID",
                                             f"priorMissionID {prior_int} does not resolve to PriorMissionInfo.priorMissionList",
                                             prior_int, f"one of {sorted(prior_mission_ids)[:20]}",
                                             rule_id="PRIOR-MISSION-REF",
                                             source="ICD 0202 PriorMissionInfo + ICD 0302 RelatedMission"))
                else:
                    issues.append(_issue("warning", "rule", loc, "relatedMission.priorMissionID",
                                         "priorMissionID cannot be verified because PriorMissionInfo/0202/0902 source is missing",
                                         prior_int, "PriorMissionInfo.priorMissionList or earlier same-IMP individualMissionID",
                                         rule_id="PRIOR-MISSION-REF",
                                         source="ICD 0202 PriorMissionInfo + ICD 0302 RelatedMission"))


def _validate_area_list(areas: Any, loc: str, issues: list[dict]) -> None:
    issues.extend(_check_list(areas, "individualMissionInfo.areaList", loc, required=True, min_len=1))
    if not isinstance(areas, list):
        return
    for i, area in enumerate(areas):
        area_loc = f"{loc} > areaList[{i}]"
        if not isinstance(area, dict):
            issues.append(_issue("error", "type", area_loc, "areaList",
                                 f"area must be object, got {type(area).__name__}", area, "Area object"))
            continue
        is_hole = _pick(area, "isHole", "IsHole")
        if is_hole is not None:
            issues.extend(_check_bool(is_hole, "areaList.isHole", area_loc))
        _validate_coordinate_list(_pick(area, "coordinateList", "CoordinateList"),
                                  "areaList.coordinateList", area_loc, issues, min_len=3)


def _validate_line_list(lines: Any, loc: str, issues: list[dict]) -> None:
    issues.extend(_check_list(lines, "individualMissionInfo.lineList", loc, required=True, min_len=1))
    if not isinstance(lines, list):
        return
    for i, line in enumerate(lines):
        line_loc = f"{loc} > lineList[{i}]"
        if not isinstance(line, dict):
            issues.append(_issue("error", "type", line_loc, "lineList",
                                 f"line must be object, got {type(line).__name__}", line, "Line object"))
            continue
        width = _pick(line, "width", "Width")
        if width is not None:
            issues.extend(_check_float(width, "lineList.width", line_loc, 0.0, 50000.0))
        _validate_coordinate_list(_pick(line, "coordinateList", "CoordinateList"),
                                  "lineList.coordinateList", line_loc, issues, min_len=2)


def _validate_individual_mission_info(
    info: Any,
    loc: str,
    issues: list[dict],
    target_ids: set[int] | None = None,
) -> None:
    if not isinstance(info, dict):
        issues.append(_issue("error", "structure", loc, "individualMissionInfo",
                             "individualMissionInfo object is required",
                             info, "IndividualMissionInfo"))
        return

    mission_type = _pick(info, "individualMissionType", "IndividualMissionType", "missionType", "MissionType")
    pattern_type = _pick(info, "patternType", "PatternType")
    auto_zoom = _pick(info, "autoZoomIn", "AutoZoomIn")
    target_id = _pick(info, "targetID", "TargetID", "targetId")
    coord_list = _pick(info, "coordinateList", "CoordinateList")
    line_list = _pick(info, "lineList", "LineList")
    area_list = _pick(info, "areaList", "AreaList")

    if mission_type is None:
        issues.append(_issue("error", "structure", loc, "individualMissionInfo.individualMissionType",
                             "individualMissionType is required", None, "uint"))
    else:
        issues.extend(_check_uint(mission_type, "individualMissionInfo.individualMissionType", loc))
    if pattern_type is not None:
        issues.extend(_check_uint(pattern_type, "individualMissionInfo.patternType", loc))
    if auto_zoom is not None:
        issues.extend(_check_bool(auto_zoom, "individualMissionInfo.autoZoomIn", loc))

    mt = _coerce_int(mission_type)
    if mt in {5, 7, 9}:
        _validate_coordinate_list(coord_list, "individualMissionInfo.coordinateList", loc, issues, min_len=1)
    elif mt == 6:
        _validate_line_list(line_list, loc, issues)
    elif mt in {3, 4}:
        _validate_area_list(area_list, loc, issues)
        if isinstance(coord_list, list) and coord_list:
            issues.append(_issue("warning", "rule", loc, "individualMissionInfo.coordinateList",
                                 f"mission type {mt} is area-based but coordinateList is populated",
                                 len(coord_list), "empty coordinateList for area mission"))
    elif mt in {1, 2}:
        if target_id is None:
            issues.append(_issue("error", "structure", loc, "individualMissionInfo.targetID",
                                 f"mission type {mt} requires targetID",
                                 None, "uint > 0"))
        else:
            issues.extend(_check_uint(target_id, "individualMissionInfo.targetID", loc))
            tid = _coerce_int(target_id)
            if tid is not None:
                _validate_target_reference(target_id, "individualMissionInfo.targetID", loc, issues, target_ids, "IndividualMissionInfo")


def _validate_imp(
    imp_key: str,
    imp_data: dict,
    existing_path_ids: set[int],
    path_meta: dict[int, dict[str, Any]] | None = None,
    target_ids: set[int] | None = None,
    input_mission_ids: set[int] | None = None,
    prior_mission_ids: set[int] | None = None,
) -> list[dict]:
    """Strict ICD/code-aware IndividualMissionPlan validation."""
    issues: list[dict] = []
    loc_base = f"IMP/{imp_key}"
    path_meta = path_meta or {}

    imp_id = _pick(imp_data, "individualMissionPackageID", "IndividualMissionPackageID", "individualMissionPackageId")
    if imp_id is None:
        issues.append(_issue("error", "structure", loc_base, "individualMissionPackageID",
                             "individualMissionPackageID is required", None, "uint"))
    else:
        issues.extend(_check_uint(imp_id, "individualMissionPackageID", loc_base))
        key_int = _coerce_int(imp_key)
        iid = _coerce_int(imp_id)
        if key_int is not None and iid is not None and key_int != iid:
            issues.append(_issue("error", "rule", loc_base, "individualMissionPackageID",
                                 f"file name IMP ID {key_int} does not match payload ID {iid}",
                                 iid, f"{key_int}"))

    ac_id = _pick(imp_data, "aircraftID", "AircraftID")
    if ac_id is None:
        issues.append(_issue("error", "structure", loc_base, "aircraftID",
                             "aircraftID is required", None, "uint enum 1..6"))
    else:
        issues.extend(_check_enum(ac_id, "aircraftID", loc_base, {1, 2, 3, 4, 5, 6}))
    ac_int = _coerce_int(ac_id)

    mission_list = _pick(imp_data, "individualMissionList", "IndividualMissionList")
    issues.extend(_check_list(mission_list, "individualMissionList", loc_base, required=True, min_len=1))
    if not isinstance(mission_list, list):
        return issues

    known_mission_ids = {
        mid
        for mid in (
            _coerce_int(_pick(mission, "individualMissionID", "IndividualMissionID"))
            for mission in mission_list
            if isinstance(mission, dict)
        )
        if mid is not None
    }
    seen_missions: set[int] = set()
    previous_mission_ids: set[int] = set()
    last_mid: int | None = None
    for i, mission in enumerate(mission_list):
        if not isinstance(mission, dict):
            issues.append(_issue("error", "type", loc_base, f"individualMissionList[{i}]",
                                 f"mission must be object, got {type(mission).__name__}",
                                 mission, "IndividualMission object"))
            continue
        m_id = _pick(mission, "individualMissionID", "IndividualMissionID")
        mid = _coerce_int(m_id)
        m_loc = f"{loc_base} > Mission {m_id if m_id is not None else i}"
        if m_id is None:
            issues.append(_issue("error", "structure", m_loc, "individualMissionID",
                                 "individualMissionID is required", None, "uint >= 900000001"))
        else:
            issues.extend(_check_uint(m_id, "individualMissionID", m_loc))
            if mid is not None:
                if mid < 900000001:
                    issues.append(_issue("warning", "rule", m_loc, "individualMissionID",
                                         f"generated individualMissionID usually starts at 900000001, got {mid}",
                                         mid, ">= 900000001"))
                if mid in seen_missions:
                    issues.append(_issue("error", "rule", m_loc, "individualMissionID",
                                         f"duplicate individualMissionID {mid} in IMP",
                                         mid, "unique in IMP"))
                if last_mid is not None and mid <= last_mid:
                    issues.append(_issue("warning", "rule", m_loc, "individualMissionID",
                                         f"individualMissionID {mid} is not ascending after {last_mid}",
                                         mid, "> previous mission ID"))
                seen_missions.add(mid)
                last_mid = mid

        is_done = _pick(mission, "isDone", "IsDone")
        if is_done is None:
            issues.append(_issue("error", "structure", m_loc, "isDone",
                                 "isDone is required", None, "bool"))
        else:
            issues.extend(_check_bool(is_done, "isDone", m_loc))

        _validate_related_mission(
            _pick(mission, "relatedMission", "RelatedMission"),
            m_loc,
            issues,
            input_mission_ids,
            prior_mission_ids,
            known_mission_ids,
            previous_mission_ids,
            mid,
        )
        _validate_individual_mission_info(_pick(mission, "individualMissionInfo", "IndividualMissionInfo"),
                                          m_loc, issues, target_ids)
        if mid is not None:
            previous_mission_ids.add(mid)

        path_id = _pick(mission, "pathID", "PathID")
        if path_id is None:
            issues.append(_issue("error", "structure", m_loc, "pathID",
                                 "pathID is required", None, "uint"))
            continue
        issues.extend(_check_uint(path_id, "pathID", m_loc))
        pid_int = _coerce_int(path_id)
        if pid_int is not None:
            if pid_int not in existing_path_ids:
                issues.append(_issue("error", "rule", m_loc, "pathID",
                                     f"pathID {pid_int} does not resolve to a FlightPath",
                                     pid_int, "existing FlightPath.pathID"))
            meta = path_meta.get(pid_int, {})
            fp_ac = meta.get("aircraftID")
            if ac_int is not None and fp_ac is not None and fp_ac != ac_int:
                issues.append(_issue("error", "rule", m_loc, "pathID",
                                     f"mission aircraftID {ac_int} does not match FlightPath aircraftID {fp_ac}",
                                     fp_ac, f"aircraftID={ac_int}"))
            fp_mid = meta.get("individualMissionID")
            mid = _coerce_int(m_id)
            if mid is not None and fp_mid is not None and fp_mid != mid:
                issues.append(_issue("error", "rule", m_loc, "individualMissionID",
                                     f"mission ID {mid} does not match FlightPath individualMissionID {fp_mid}",
                                     fp_mid, f"individualMissionID={mid}"))
            wp_done = meta.get("waypointDone")
            if isinstance(wp_done, list) and wp_done:
                if is_done is True and any(done is False for done in wp_done):
                    issues.append(_issue("warning", "rule", m_loc, "isDone",
                                         "mission isDone=true but at least one waypoint is not done",
                                         wp_done, "all waypoints isDone=true"))
                if is_done is False and all(done is True for done in wp_done):
                    issues.append(_issue("warning", "rule", m_loc, "isDone",
                                         "mission isDone=false but all linked waypoints are done",
                                         wp_done, "active mission should have unfinished waypoints"))

    return issues


# ---------------------------------------------------------------------------
# MissionPlan validation
# ---------------------------------------------------------------------------

def _legacy_validate_mission_plan(
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


def _validate_mission_plan(
    plan_data: dict,
    existing_imp_ids: set[int],
    plan_index: int,
    existing_input_package_ids: set[int] | None = None,
    existing_reference_package_ids: set[int] | None = None,
) -> list[dict]:
    """Strict ICD-aware MissionPlan validation."""
    issues: list[dict] = []
    existing_input_package_ids = existing_input_package_ids or set()
    existing_reference_package_ids = existing_reference_package_ids or set()
    mp_id = _pick(plan_data, "missionPlanID", "MissionPlanID")
    loc_base = f"MissionPlan/{mp_id if mp_id is not None else plan_index}"

    if mp_id is None:
        issues.append(_issue("error", "structure", loc_base, "missionPlanID",
                             "missionPlanID is required", None, "uint"))
    else:
        issues.extend(_check_uint(mp_id, "missionPlanID", loc_base))
        mid = _coerce_int(mp_id)
        if mid is not None and mid < 700000001:
            issues.append(_issue("warning", "rule", loc_base, "missionPlanID",
                                 f"generated missionPlanID usually starts at 700000001, got {mid}",
                                 mid, ">= 700000001"))

    for field in ("timestamp", "missionPlanTimestamp"):
        value = _pick(plan_data, field, field[0].upper() + field[1:])
        if value is not None:
            issues.extend(_check_uint64(value, field, loc_base))
    planning_time = _pick(plan_data, "planningTime", "PlanningTime")
    if planning_time is not None:
        issues.extend(_check_float(planning_time, "planningTime", loc_base, 0.0, None))
    planner_id = _pick(plan_data, "plannerID", "PlannerID")
    if planner_id is not None:
        issues.extend(_check_enum(planner_id, "plannerID", loc_base, {1, 2, 3, 4, 5, 6}))
    for field in ("inputMissionPackageID", "missionReferencePackageID"):
        value = _pick(plan_data, field, field[0].upper() + field[1:])
        if value is not None:
            issues.extend(_check_uint(value, field, loc_base))
            int_value = _coerce_int(value)
            if field == "inputMissionPackageID" and int_value is not None and existing_input_package_ids and int_value not in existing_input_package_ids:
                issues.append(_issue("error", "rule", loc_base, field,
                                     f"inputMissionPackageID {int_value} does not resolve to InputMissionPlan",
                                     int_value, f"one of {sorted(existing_input_package_ids)[:20]}"))
            if field == "missionReferencePackageID" and int_value is not None and existing_reference_package_ids and int_value not in existing_reference_package_ids:
                issues.append(_issue("error", "rule", loc_base, field,
                                     f"missionReferencePackageID {int_value} does not resolve to MissionReferenceInfo",
                                     int_value, f"one of {sorted(existing_reference_package_ids)[:20]}"))

    aircraft_list = _pick(plan_data, "aircraftList", "AircraftList")
    issues.extend(_check_list(aircraft_list, "aircraftList", loc_base, required=True, min_len=1))
    if not isinstance(aircraft_list, list):
        return issues

    seen_aircraft: set[int] = set()
    for i, ac_entry in enumerate(aircraft_list):
        if not isinstance(ac_entry, dict):
            issues.append(_issue("error", "type", loc_base, f"aircraftList[{i}]",
                                 f"aircraft entry must be object, got {type(ac_entry).__name__}",
                                 ac_entry, "Aircraft object"))
            continue
        ac_id = _pick(ac_entry, "aircraftID", "AircraftID")
        ac_loc = f"{loc_base} > Aircraft {ac_id if ac_id is not None else i}"
        if ac_id is None:
            issues.append(_issue("error", "structure", ac_loc, "aircraftID",
                                 "aircraftID is required", None, "uint enum 1..6"))
        else:
            issues.extend(_check_enum(ac_id, "aircraftID", ac_loc, {1, 2, 3, 4, 5, 6}))
            ac_int = _coerce_int(ac_id)
            if ac_int is not None:
                if ac_int in seen_aircraft:
                    issues.append(_issue("error", "rule", ac_loc, "aircraftID",
                                         f"duplicate aircraftID {ac_int} in MissionPlan.aircraftList",
                                         ac_int, "unique aircraftID"))
                seen_aircraft.add(ac_int)

        imp_id = _pick(ac_entry, "individualMissionPackageID", "individualMissionPackageId", "IndividualMissionPackageID")
        if imp_id is None:
            issues.append(_issue("error", "structure", ac_loc, "individualMissionPackageID",
                                 "individualMissionPackageID is required", None, "uint"))
        else:
            issues.extend(_check_uint(imp_id, "individualMissionPackageID", ac_loc))
            iid = _coerce_int(imp_id)
            if iid is not None and iid not in existing_imp_ids:
                issues.append(_issue("error", "rule", ac_loc, "individualMissionPackageID",
                                     f"IMP ID {iid} does not resolve to an IndividualMissionPlan",
                                     iid, f"one of {sorted(existing_imp_ids)[:20]}"))

        health = _pick(ac_entry, "health", "Health")
        if health is not None:
            issues.extend(_check_enum(health, "health", ac_loc, {0, 1, 2}, _HEALTH_LABELS))

        ac_type = _pick(ac_entry, "aircraftType", "AircraftType")
        ac_int = _coerce_int(ac_id)
        if ac_type is not None and ac_int is not None:
            ac_type_int = _coerce_int(ac_type)
            if 1 <= ac_int <= 3 and ac_type_int == 2:
                issues.append(_issue("warning", "rule", ac_loc, "aircraftType",
                                     "LAH aircraftID has UAV aircraftType", ac_type_int, "aircraftType=1"))
            if 4 <= ac_int <= 6 and ac_type_int == 1:
                issues.append(_issue("warning", "rule", ac_loc, "aircraftType",
                                     "UAV aircraftID has LAH aircraftType", ac_type_int, "aircraftType=2"))

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

def _walk_json_values(obj: Any):
    if isinstance(obj, dict):
        yield obj
        for value in obj.values():
            yield from _walk_json_values(value)
    elif isinstance(obj, list):
        for item in obj:
            yield from _walk_json_values(item)


def _collect_target_ids_from_obj(obj: Any) -> set[int]:
    target_ids: set[int] = set()
    for item in _walk_json_values(obj):
        if not isinstance(item, dict):
            continue
        for key in ("targetID", "TargetID", "targetId"):
            if key not in item:
                continue
            tid = _coerce_int(item.get(key))
            if tid is not None and tid > 0:
                target_ids.add(tid)
    return target_ids


def _extract_path_meta(path_data: dict) -> dict[str, Any]:
    ac_id = _coerce_int(_pick(path_data, "aircraftID", "AircraftID"))
    mission_id = _coerce_int(_pick(path_data, "individualMissionID", "IndividualMissionID"))
    waypoints: list[dict] = []
    for key in ("lahWaypointList", "uavWaypointList", "waypointList"):
        value = path_data.get(key)
        if isinstance(value, list):
            waypoints = [wp for wp in value if isinstance(wp, dict)]
            break
    waypoint_done = [
        _pick(wp, "isDone", "IsDone")
        for wp in waypoints
        if _pick(wp, "isDone", "IsDone") is not None
    ]
    return {
        "aircraftID": ac_id,
        "individualMissionID": mission_id,
        "waypointCount": len(waypoints),
        "waypointDone": waypoint_done,
    }


def _validate_531xx_root_counts(
    data: dict,
    loc_base: str,
    expected_aircraft: set[int],
    issues: list[dict],
) -> list[dict]:
    presence = _pick(data, "presenceVector", "PresenceVector")
    if presence is None:
        issues.append(_issue("error", "structure", loc_base, "presenceVector",
                             "presenceVector is required", None, "byte 0..255"))
    else:
        issues.extend(_check_byte(presence, "presenceVector", loc_base))
    timestamp = _pick(data, "timestamp", "Timestamp")
    if timestamp is None:
        issues.append(_issue("error", "structure", loc_base, "timestamp",
                             "timestamp is required", None, "uint64 ms or byte[5]"))
    else:
        issues.extend(_check_timestamp_531xx(timestamp, "timestamp", loc_base))
    mp_id = _pick(data, "missionPlanID", "MissionPlanID")
    if mp_id is None:
        issues.append(_issue("error", "structure", loc_base, "missionPlanID",
                             "missionPlanID is required", None, "uint"))
    else:
        issues.extend(_check_uint(mp_id, "missionPlanID", loc_base))
    ac_id = _pick(data, "aircraftID", "AircraftID")
    if ac_id is None:
        issues.append(_issue("error", "structure", loc_base, "aircraftID",
                             "aircraftID is required", None, f"enum {sorted(expected_aircraft)}"))
    else:
        issues.extend(_check_enum(ac_id, "aircraftID", loc_base, expected_aircraft))
    seg_list = _pick(data, "missionSegmentList", "MissionSegmentList")
    issues.extend(_check_list(seg_list, "missionSegmentList", loc_base, required=True, min_len=1))
    _validate_declared_count(_pick(data, "missionSegmentN", "MissionSegmentN"), seg_list,
                             "missionSegmentN", loc_base, issues, required=True)
    return _as_list(seg_list)


def _validate_mission_segment_ids(segments: list[Any], loc_base: str, issues: list[dict]) -> None:
    segment_ids: list[int] = []
    seen: set[int] = set()
    for i, seg in enumerate(segments):
        if not isinstance(seg, dict):
            issues.append(_issue("error", "type", loc_base, f"missionSegmentList[{i}]",
                                 f"missionSegment must be object, got {type(seg).__name__}",
                                 seg, "MissionSegment object"))
            continue
        sid = _coerce_int(_pick(seg, "missionSegmentID", "MissionSegmentID"))
        if sid is not None:
            segment_ids.append(sid)
            if sid in seen:
                issues.append(_issue("error", "rule", loc_base, "missionSegmentID",
                                     f"duplicate missionSegmentID {sid}",
                                     sid, "unique missionSegmentID"))
            seen.add(sid)
    if segment_ids:
        sorted_ids = sorted(segment_ids)
        expected = list(range(sorted_ids[0], sorted_ids[0] + len(sorted_ids)))
        if sorted_ids != expected:
            missing = sorted(set(expected) - set(sorted_ids))
            extra = sorted(set(sorted_ids) - set(expected))
            msg = f"missionSegmentID sequence is not contiguous from {sorted_ids[0]} to {expected[-1]}"
            if missing:
                msg += f"; missing {missing}"
            if extra:
                msg += f"; out-of-range {extra}"
            issues.append(_issue("error", "rule", loc_base, "missionSegmentID",
                                 msg, segment_ids, expected))


def _validate_formation_info(formation: Any, loc: str, issues: list[dict]) -> None:
    if not isinstance(formation, dict):
        issues.append(_issue("error", "structure", loc, "formationInfo",
                             "formationInfo must be object",
                             formation, "FormationInfo"))
        return

    leader = _pick(formation, "leaderAircraftID", "LeaderAircraftID")
    if leader is None:
        issues.append(_issue("error", "structure", loc, "formationInfo.leaderAircraftID",
                             "formationInfo.leaderAircraftID is required",
                             None, "uint enum 4..6"))
    else:
        issues.extend(_check_enum(leader, "formationInfo.leaderAircraftID", loc, {4, 5, 6}))

    offset = _pick(formation, "formation", "Formation")
    if not isinstance(offset, dict):
        issues.append(_issue("error", "structure", loc, "formationInfo.formation",
                             "formationInfo.formation is required",
                             offset, "Formation dX/dY/dZ"))
        return
    for axis in ("dX", "dY", "dZ"):
        value = _pick(offset, axis, axis.lower())
        if value is None:
            issues.append(_issue("error", "structure", loc, f"formationInfo.formation.{axis}",
                                 f"formationInfo.formation.{axis} is required",
                                 None, "int 0..50000"))
        else:
            issues.extend(_check_icd_int(value, f"formationInfo.formation.{axis}", loc, 0, 50000))


def _validate_53112_uav_mission_plan(msg_key: str, data: dict, target_ids: set[int] | None = None) -> list[dict]:
    issues: list[dict] = []
    loc_base = f"53112/{msg_key}"
    segments = _validate_531xx_root_counts(data, loc_base, {4, 5, 6}, issues)
    _validate_mission_segment_ids(segments, loc_base, issues)

    seen_mission_ids: set[int] = set()
    for i, seg in enumerate(segments):
        if not isinstance(seg, dict):
            continue
        sid_value = _pick(seg, "missionSegmentID", "MissionSegmentID")
        sid = sid_value if sid_value is not None else i
        seg_loc = f"{loc_base} > Segment {sid}"
        _require_present(sid_value, "missionSegmentID", seg_loc, issues, "uint")
        issues.extend(_check_uint(sid_value, "missionSegmentID", seg_loc))
        is_done = _pick(seg, "isDone", "IsDone")
        if is_done is None:
            issues.append(_issue("error", "structure", seg_loc, "isDone",
                                 "missionSegment.isDone is required", None, "bool"))
        else:
            issues.extend(_check_bool(is_done, "isDone", seg_loc))
        segment_type = _pick(seg, "missionSegmentType", "MissionSegmentType")
        _require_present(segment_type, "missionSegmentType", seg_loc, issues, "uint")
        issues.extend(_check_uint(segment_type, "missionSegmentType", seg_loc))
        im_list = _pick(seg, "individualMissionList", "IndividualMissionList")
        issues.extend(_check_list(im_list, "individualMissionList", seg_loc, required=True, min_len=1))
        _validate_declared_count(_pick(seg, "individualMissionListN", "IndividualMissionListN"),
                                 im_list, "individualMissionListN", seg_loc, issues, required=True)

        for j, mission in enumerate(_as_list(im_list)):
            if not isinstance(mission, dict):
                issues.append(_issue("error", "type", seg_loc, f"individualMissionList[{j}]",
                                     f"individualMission must be object, got {type(mission).__name__}",
                                     mission, "IndividualMission object"))
                continue
            mid_value = _pick(mission, "individualMissionID", "IndividualMissionID")
            mid = mid_value if mid_value is not None else j
            m_loc = f"{seg_loc} > Mission {mid}"
            _require_present(mid_value, "individualMissionID", m_loc, issues, "uint")
            issues.extend(_check_uint(mid_value, "individualMissionID", m_loc))
            mid_int = _coerce_int(mid_value)
            if mid_int is not None:
                if mid_int in seen_mission_ids:
                    issues.append(_issue("error", "rule", m_loc, "individualMissionID",
                                         f"duplicate individualMissionID {mid_int} in same 53112 plan",
                                         mid_int, "unique individualMissionID",
                                         rule_id="NFUSION-53112",
                                         source="ICD 53112 IndividualMission"))
                seen_mission_ids.add(mid_int)
            mission_done = _pick(mission, "isDone", "IsDone")
            if mission_done is None:
                issues.append(_issue("error", "structure", m_loc, "isDone",
                                     "individualMission.isDone is required", None, "bool"))
            else:
                issues.extend(_check_bool(mission_done, "isDone", m_loc))

            allocated = _pick(mission, "allocatedArea", "AllocatedArea")
            if not isinstance(allocated, dict):
                issues.append(_issue("error", "structure", m_loc, "allocatedArea",
                                     "53112 IndividualMission requires allocatedArea",
                                     allocated, "AllocatedArea"))
            else:
                coords = _pick(allocated, "coordinateList", "CoordinateList")
                _validate_declared_count(_pick(allocated, "coordinateListN", "CoordinateListN"),
                                         coords, "allocatedArea.coordinateListN", m_loc, issues, required=True)
                if isinstance(coords, list):
                    if len(coords) < 3:
                        issues.append(_issue("warning", "rule", m_loc, "allocatedArea.coordinateList",
                                             f"allocatedArea has only {len(coords)} coordinate(s); area missions normally need polygon coordinates",
                                             len(coords), ">= 3 for polygon area",
                                             rule_id="NFUSION-53112",
                                             source="ICD 53112 AllocatedArea"))
                    for c_idx, coord in enumerate(coords):
                        issues.extend(_validate_coordinate(coord, f"{m_loc} > allocatedArea.coordinateList[{c_idx}]"))

            flight_type = _pick(mission, "flightType", "FlightType")
            _require_present(flight_type, "flightType", m_loc, issues, "uint enum 1..2")
            issues.extend(_check_enum(flight_type, "flightType", m_loc, {1, 2}))
            flight_type_int = _coerce_int(flight_type)
            wp_list = _pick(mission, "waypointList", "WaypointList")
            if flight_type_int == 1:
                issues.extend(_check_list(wp_list, "waypointList", m_loc, required=True, min_len=1))
                _validate_declared_count(_pick(mission, "waypointListN", "WaypointListN"),
                                         wp_list, "waypointListN", m_loc, issues, required=True)
                formation = _pick(mission, "formationInfo", "FormationInfo")
                if isinstance(formation, dict) and _is_meaningful(formation):
                    issues.append(_issue("warning", "rule", m_loc, "formationInfo",
                                         "flightType=1 waypoint mission should not carry formationInfo",
                                         formation, "omit formationInfo for flightType=1",
                                         rule_id="NFUSION-53112",
                                         source="ICD 53112 flightType switch"))
                wp_done: list[bool] = []
                for w_idx, wp in enumerate(_as_list(wp_list)):
                    if isinstance(wp, dict) and mission_done is not None:
                        wp = dict(wp)
                        wp.setdefault("isDone", mission_done)
                    if isinstance(wp, dict):
                        wp_id = _pick(wp, "waypointID", "WaypointID", default=w_idx)
                        wp_loc = f"{m_loc} > WP {wp_id}"
                        filming_planned = _pick(wp, "filmingPlanned", "FilmingPlanned")
                        _require_present(filming_planned, "filmingPlanned", wp_loc, issues, "bool",
                                         rule_id="NFUSION-53112",
                                         source="ICD 53112 Waypoint")
                        pass_type_int = _coerce_int(_pick(wp, "waypointPassType", "WaypointPassType"))
                        loiter = _pick(wp, "loiterProperty", "LoiterProperty")
                        if pass_type_int != 2 and isinstance(loiter, dict) and _is_meaningful(loiter):
                            issues.append(_issue("warning", "rule", wp_loc, "loiterProperty",
                                                 f"waypointPassType={pass_type_int} does not use loiterProperty",
                                                 loiter, "null/empty unless waypointPassType=2",
                                                 rule_id="NFUSION-53112",
                                                 source="ICD 53112 Waypoint"))
                    issues.extend(_validate_waypoint(wp, w_idx, m_loc, _coerce_int(_pick(data, "aircraftID", "AircraftID")), target_ids))
                    if isinstance(wp, dict) and _pick(wp, "isDone", "IsDone") is not None:
                        wp_done.append(_pick(wp, "isDone", "IsDone"))
                issues.extend(_validate_waypoint_chain(_as_list(wp_list), m_loc))
                issues.extend(_validate_waypoint_sequence_timing(_as_list(wp_list), m_loc))
                if mission_done is False and wp_done and all(done is True for done in wp_done):
                    issues.append(_issue("warning", "rule", m_loc, "isDone",
                                         "mission isDone=false but all nested waypoints are done",
                                         wp_done, "active mission should have unfinished waypoints"))
            elif flight_type_int == 2:
                formation = _pick(mission, "formationInfo", "FormationInfo")
                if not isinstance(formation, dict):
                    issues.append(_issue("error", "structure", m_loc, "formationInfo",
                                         "flightType=2 requires formationInfo",
                                         formation, "FormationInfo"))
                else:
                    _validate_formation_info(formation, m_loc, issues)
                if isinstance(wp_list, list) and _is_meaningful(wp_list):
                    issues.append(_issue("warning", "rule", m_loc, "waypointList",
                                         "flightType=2 formation mission should not carry waypointList",
                                         len(wp_list), "empty/omitted waypointList for flightType=2",
                                         rule_id="NFUSION-53112",
                                         source="ICD 53112 flightType switch"))
                waypoint_count = _pick(mission, "waypointListN", "WaypointListN")
                if _coerce_int(waypoint_count) not in (None, 0):
                    issues.append(_issue("warning", "rule", m_loc, "waypointListN",
                                         "flightType=2 formation mission should not carry waypointListN > 0",
                                         waypoint_count, "0 or omitted",
                                         rule_id="NFUSION-53112",
                                         source="ICD 53112 flightType switch"))

    return issues


def _validate_53111_lah_mission_plan(msg_key: str, data: dict, target_ids: set[int] | None = None) -> list[dict]:
    issues: list[dict] = []
    loc_base = f"53111/{msg_key}"
    segments = _validate_531xx_root_counts(data, loc_base, {1, 2, 3}, issues)
    _validate_mission_segment_ids(segments, loc_base, issues)

    seen_mission_ids: set[int] = set()
    for i, seg in enumerate(segments):
        if not isinstance(seg, dict):
            continue
        sid_value = _pick(seg, "missionSegmentID", "MissionSegmentID")
        sid = sid_value if sid_value is not None else i
        seg_loc = f"{loc_base} > Segment {sid}"
        _require_present(sid_value, "missionSegmentID", seg_loc, issues, "uint")
        issues.extend(_check_uint(sid_value, "missionSegmentID", seg_loc))
        is_done = _pick(seg, "isDone", "IsDone")
        if is_done is None:
            issues.append(_issue("error", "structure", seg_loc, "isDone",
                                 "missionSegment.isDone is required", None, "bool"))
        else:
            issues.extend(_check_bool(is_done, "isDone", seg_loc))
        segment_type = _pick(seg, "missionSegmentType", "MissionSegmentType")
        _require_present(segment_type, "missionSegmentType", seg_loc, issues, "uint")
        issues.extend(_check_uint(segment_type, "missionSegmentType", seg_loc))
        im_list = _pick(seg, "individualMissionList", "IndividualMissionList")
        issues.extend(_check_list(im_list, "individualMissionList", seg_loc, required=True, min_len=1))
        _validate_declared_count(_pick(seg, "individualMissionListN", "IndividualMissionListN"),
                                 im_list, "individualMissionListN", seg_loc, issues, required=True)
        for j, mission in enumerate(_as_list(im_list)):
            if not isinstance(mission, dict):
                issues.append(_issue("error", "type", seg_loc, f"individualMissionList[{j}]",
                                     f"individualMission must be object, got {type(mission).__name__}",
                                     mission, "IndividualMission object"))
                continue
            mid_value = _pick(mission, "individualMissionID", "IndividualMissionID")
            mid = mid_value if mid_value is not None else j
            m_loc = f"{seg_loc} > Mission {mid}"
            _require_present(mid_value, "individualMissionID", m_loc, issues, "uint")
            issues.extend(_check_uint(mid_value, "individualMissionID", m_loc))
            mid_int = _coerce_int(mid_value)
            if mid_int is not None:
                if mid_int in seen_mission_ids:
                    issues.append(_issue("error", "rule", m_loc, "individualMissionID",
                                         f"duplicate individualMissionID {mid_int} in same 53111 plan",
                                         mid_int, "unique individualMissionID",
                                         rule_id="NFUSION-53111",
                                         source="ICD 53111 IndividualMission"))
                seen_mission_ids.add(mid_int)
            mission_done = _pick(mission, "isDone", "IsDone")
            if mission_done is None:
                issues.append(_issue("error", "structure", m_loc, "isDone",
                                     "individualMission.isDone is required", None, "bool"))
            else:
                issues.extend(_check_bool(mission_done, "isDone", m_loc))
            wp_list = _pick(mission, "waypointList", "WaypointList")
            issues.extend(_check_list(wp_list, "waypointList", m_loc, required=True, min_len=1))
            _validate_declared_count(_pick(mission, "waypointListN", "WaypointListN"),
                                     wp_list, "waypointListN", m_loc, issues, required=True)
            for w_idx, wp in enumerate(_as_list(wp_list)):
                wp_loc = f"{m_loc} > WP {w_idx}"
                if not isinstance(wp, dict):
                    issues.append(_issue("error", "type", wp_loc, "waypoint",
                                         f"waypoint must be object, got {type(wp).__name__}",
                                         wp, "Waypoint object"))
                    continue
                issues.extend(_validate_lah_53111_waypoint(wp, w_idx, m_loc, target_ids))
            issues.extend(_validate_waypoint_chain(_as_list(wp_list), m_loc))
            issues.extend(_validate_waypoint_sequence_timing(_as_list(wp_list), m_loc))
    return issues


def _validate_lah_53111_waypoint(wp: dict, idx: int, location_prefix: str, target_ids: set[int] | None = None) -> list[dict]:
    issues: list[dict] = []
    wp_id = _pick(wp, "waypointID", "WaypointID")
    loc = f"{location_prefix} > WP {wp_id if wp_id is not None else idx}"
    _require_present(wp_id, "waypointID", loc, issues, "uint")
    issues.extend(_check_uint(wp_id, "waypointID", loc))
    coord = _pick(wp, "coordinate", "Coordinate")
    if coord is None:
        issues.append(_issue("error", "structure", loc, "coordinate",
                             "Waypoint.coordinate is required", None, "Coordinate"))
    else:
        issues.extend(_validate_coordinate(coord, loc))
    speed = _pick(wp, "speed", "Speed")
    _require_present(speed, "speed", loc, issues, "float 0..1000")
    issues.extend(_check_float(speed, "speed", loc, 0.0, 1000.0))
    eta = _pick(wp, "eta", "ETA")
    _require_present(eta, "eta", loc, issues, "uint seconds")
    issues.extend(_check_uint(eta, "eta", loc))
    next_wp = _pick(wp, "nextWaypointID", "NextWaypointID")
    _require_present(next_wp, "nextWaypointID", loc, issues, "uint")
    issues.extend(_check_uint(next_wp, "nextWaypointID", loc))
    hovering = _pick(wp, "hovering", "Hovering")
    if isinstance(hovering, dict):
        hovering = _pick(hovering, "time", "Time")
    if hovering is not None:
        issues.extend(_check_uint(hovering, "hovering", loc))
    attack = _pick(wp, "attack", "Attack")
    if isinstance(attack, dict):
        target_id = _pick(attack, "targetID", "TargetID", "targetId")
        weapon_type = _pick(attack, "weaponType", "WeaponType")
        issues.extend(_check_uint(target_id, "attack.targetID", loc))
        issues.extend(_check_enum(weapon_type, "attack.weaponType", loc, {0, 1, 2, 3}))
        tid = _coerce_int(target_id)
        weapon = _coerce_int(weapon_type)
        if (tid or 0) > 0 or (weapon or 0) > 0:
            if tid is None or tid <= 0:
                issues.append(_issue("error", "rule", loc, "attack.targetID",
                                     "attack weapon/target data is active but targetID is not positive",
                                     target_id, "uint > 0"))
            else:
                _validate_target_reference(target_id, "attack.targetID", loc, issues, target_ids, "Attack")
            if weapon is None or weapon <= 0:
                issues.append(_issue("error", "rule", loc, "attack.weaponType",
                                     "attack target data is active but weaponType is not positive",
                                     weapon_type, "1..3"))
    return issues


def _validate_030x_message_head(
    msg_id: str,
    msg_key: str,
    data: dict,
    id_field: str,
    existing_ids: set[int],
    path_meta: dict[int, dict[str, Any]] | None = None,
    expected_aircraft_ids: set[int] | None = None,
) -> list[dict]:
    issues: list[dict] = []
    loc_base = f"{msg_id}/{msg_key}"

    timestamp = _pick(data, "timestamp", "Timestamp")
    if timestamp is None:
        issues.append(_issue("error", "structure", loc_base, "timestamp",
                             "timestamp is required", None, "uint64 ms"))
    else:
        issues.extend(_check_uint64(timestamp, "timestamp", loc_base))

    source = _pick(data, "source", "Source")
    if source is not None and not isinstance(source, str):
        issues.append(_issue("warning", "type", loc_base, "source",
                             f"source should be string or null, got {type(source).__name__}",
                             source, "string|null"))

    value = _pick(data, id_field, id_field[0].upper() + id_field[1:])
    if value is None:
        issues.append(_issue("error", "structure", loc_base, id_field,
                             f"{id_field} is required", None, "uint32"))
        return issues
    issues.extend(_check_uint(value, id_field, loc_base))
    int_value = _coerce_int(value)
    if int_value is not None and existing_ids and int_value not in existing_ids:
        issues.append(_issue("error", "rule", loc_base, id_field,
                             f"{msg_id} references {id_field} {int_value}, but no matching generated payload exists",
                             int_value, f"one of {sorted(existing_ids)[:20]}"))
    if int_value is not None and path_meta is not None and expected_aircraft_ids is not None:
        meta = path_meta.get(int_value)
        if meta is not None:
            ac_id = meta.get("aircraftID")
            if ac_id is not None and ac_id not in expected_aircraft_ids:
                issues.append(_issue("error", "rule", loc_base, id_field,
                                     f"{msg_id} references pathID {int_value} for aircraftID {ac_id}",
                                     int_value, f"path aircraftID in {sorted(expected_aircraft_ids)}"))
    return issues


def _validate_lat_lon(obj: Any, location: str, field_prefix: str = "") -> list[dict]:
    issues: list[dict] = []
    if not isinstance(obj, dict):
        issues.append(_issue("error", "type", location, field_prefix or "latLon",
                             f"{field_prefix or 'latLon'} must be object, got {type(obj).__name__}",
                             obj, "object with latitude/longitude"))
        return issues
    lat = _pick(obj, "latitude", "Latitude")
    lon = _pick(obj, "longitude", "Longitude")
    if lat is None:
        issues.append(_issue("error", "structure", location, f"{field_prefix}.latitude".strip("."),
                             "latitude is required", None, "float -90..90"))
    else:
        issues.extend(_check_float(lat, f"{field_prefix}.latitude".strip("."), location, -90.0, 90.0))
    if lon is None:
        issues.append(_issue("error", "structure", location, f"{field_prefix}.longitude".strip("."),
                             "longitude is required", None, "float -180..180"))
    else:
        issues.extend(_check_float(lon, f"{field_prefix}.longitude".strip("."), location, -180.0, 180.0))
    return issues


def _validate_input_mission_package(pkg_key: str, data: dict) -> list[dict]:
    issues: list[dict] = []
    loc_base = f"InputMissionPlan/{pkg_key}"

    for field in ("timestamp", "inputMissionPackageID"):
        value = _pick(data, field, field[0].upper() + field[1:])
        if value is None:
            expected = "uint64" if field == "timestamp" else "uint32"
            issues.append(_issue("error", "structure", loc_base, field,
                                 f"{field} is required", None, expected))
        elif field == "timestamp":
            issues.extend(_check_uint64(value, field, loc_base))
        else:
            issues.extend(_check_uint(value, field, loc_base))

    pkg_id = _coerce_int(_pick(data, "inputMissionPackageID", "InputMissionPackageID", default=pkg_key))
    key_int = _coerce_int(pkg_key)
    if key_int is not None and pkg_id is not None and key_int != pkg_id:
        issues.append(_issue("error", "rule", loc_base, "inputMissionPackageID",
                             f"file name package ID {key_int} does not match payload ID {pkg_id}",
                             pkg_id, f"{key_int}"))

    pkg_type = _pick(data, "inputMissionPackageType", "InputMissionPackageType")
    if pkg_type is not None:
        issues.extend(_check_enum(pkg_type, "inputMissionPackageType", loc_base, set(range(0, 8))))
    main_sensor = _pick(data, "mainSensor", "MainSensor")
    if main_sensor is not None:
        issues.extend(_check_enum(main_sensor, "mainSensor", loc_base, {0, 1, 2}))

    aircraft_list = _pick(data, "availableAircraftList", "AvailableAircraftList")
    if aircraft_list is not None:
        issues.extend(_check_list(aircraft_list, "availableAircraftList", loc_base))
        seen_aircraft: set[int] = set()
        for idx, item in enumerate(_as_list(aircraft_list)):
            item_loc = f"{loc_base} > availableAircraftList[{idx}]"
            if not isinstance(item, dict):
                issues.append(_issue("error", "type", item_loc, "availableAircraft",
                                     f"availableAircraft must be object, got {type(item).__name__}",
                                     item, "AvailableAircraft object"))
                continue
            ac_id = _pick(item, "aircraftID", "AircraftID")
            issues.extend(_check_enum(ac_id, "aircraftID", item_loc, {0, 1, 2, 3, 4, 5, 6}))
            ac_int = _coerce_int(ac_id)
            if ac_int is not None and ac_int != 0:
                if ac_int in seen_aircraft:
                    issues.append(_issue("warning", "rule", item_loc, "aircraftID",
                                         f"duplicate available aircraftID {ac_int}",
                                         ac_int, "unique available aircraftID"))
                seen_aircraft.add(ac_int)

    mission_list = _pick(data, "inputMissionList", "InputMissionList")
    if mission_list is not None:
        issues.extend(_check_list(mission_list, "inputMissionList", loc_base))
        seen_missions: set[int] = set()
        for idx, mission in enumerate(_as_list(mission_list)):
            m_loc = f"{loc_base} > InputMission[{idx}]"
            if not isinstance(mission, dict):
                issues.append(_issue("error", "type", m_loc, "inputMission",
                                     f"inputMission must be object, got {type(mission).__name__}",
                                     mission, "InputMission object"))
                continue
            mission_id = _pick(mission, "inputMissionID", "InputMissionID")
            issues.extend(_check_uint(mission_id, "inputMissionID", m_loc))
            mid = _coerce_int(mission_id)
            if mid is not None:
                if mid in seen_missions:
                    issues.append(_issue("error", "rule", m_loc, "inputMissionID",
                                         f"duplicate inputMissionID {mid}",
                                         mid, "unique inputMissionID"))
                seen_missions.add(mid)
            mission_type = _pick(mission, "inputMissionType", "InputMissionType")
            if mission_type is not None:
                issues.extend(_check_enum(mission_type, "inputMissionType", m_loc, set(range(0, 8))))
            region_type = _pick(mission, "regionType", "RegionType")
            if region_type is not None:
                issues.extend(_check_enum(region_type, "regionType", m_loc, set(range(0, 21))))
            is_done = _pick(mission, "isDone", "IsDone")
            if is_done is not None:
                issues.extend(_check_bool(is_done, "isDone", m_loc))

            detail = _pick(mission, "missionDetail", "MissionDetail")
            if isinstance(detail, dict):
                coords = _pick(detail, "coordinateList", "CoordinateList")
                if isinstance(coords, list):
                    for c_idx, coord in enumerate(coords):
                        issues.extend(_validate_coordinate(coord, f"{m_loc} > coordinateList[{c_idx}]"))
                lines = _pick(detail, "lineList", "LineList")
                if isinstance(lines, list):
                    _validate_line_list(lines, m_loc, issues)
                areas = _pick(detail, "areaList", "AreaList")
                if isinstance(areas, list):
                    _validate_area_list(areas, m_loc, issues)

    return issues


def _collect_input_mission_ids_from_package(data: Any) -> set[int]:
    ids: set[int] = set()
    if not isinstance(data, dict):
        return ids
    mission_list = _pick(data, "inputMissionList", "InputMissionList")
    for mission in _as_list(mission_list):
        if not isinstance(mission, dict):
            continue
        mission_id = _coerce_int(_pick(mission, "inputMissionID", "InputMissionID"))
        if mission_id is not None and mission_id > 0:
            ids.add(int(mission_id))
    return ids


def _collect_individual_mission_ids_from_imp(data: Any) -> set[int]:
    ids: set[int] = set()
    if not isinstance(data, dict):
        return ids
    mission_list = _pick(data, "individualMissionList", "IndividualMissionList")
    for mission in _as_list(mission_list):
        if not isinstance(mission, dict):
            continue
        mission_id = _coerce_int(_pick(mission, "individualMissionID", "IndividualMissionID"))
        if mission_id is not None and mission_id > 0:
            ids.add(int(mission_id))
    return ids


def _collect_mission_plan_ids_from_obj(obj: Any) -> set[int]:
    ids: set[int] = set()
    for item in _walk_json_values(obj):
        if not isinstance(item, dict):
            continue
        plan_id = _coerce_int(_pick(item, "missionPlanID", "MissionPlanID"))
        if plan_id is not None and plan_id > 0:
            ids.add(int(plan_id))
    return ids


def _collect_pending_option_plan_ids_from_replan(record: Any) -> set[int]:
    ids: set[int] = set()
    if not isinstance(record, dict):
        return ids
    for option_list in (
        _pick(record, "pendingOptionList", "PendingOptionList"),
        _pick(record, "optionList", "OptionList"),
    ):
        for option in _as_list(option_list):
            if not isinstance(option, dict):
                continue
            plan_id = _coerce_int(_pick(option, "missionPlanID", "MissionPlanID"))
            if plan_id is not None and plan_id > 0:
                ids.add(int(plan_id))
    return ids


def _collect_0701_offer_record(msg_key: str, data: dict) -> dict[str, Any]:
    timestamp = _coerce_int(_pick(data, "timestamp", "Timestamp"))
    plan_ids: set[int] = set()
    option_list = _pick(data, "optionList", "OptionList")
    for option in _as_list(option_list):
        if not isinstance(option, dict):
            continue
        plan_id = _coerce_int(_pick(option, "missionPlanID", "MissionPlanID"))
        if plan_id is not None and plan_id > 0:
            plan_ids.add(int(plan_id))
    return {"key": msg_key, "timestamp": timestamp, "planIds": plan_ids}


def _option_signature_list(option_list: Any) -> list[tuple[int | None, int | None, str]]:
    signatures: list[tuple[int | None, int | None, str]] = []
    for option in _as_list(option_list):
        if not isinstance(option, dict):
            continue
        option_name = _pick(option, "optionName", "OptionName", default="")
        signatures.append((
            _coerce_int(_pick(option, "optionID", "OptionID")),
            _coerce_int(_pick(option, "missionPlanID", "MissionPlanID")),
            str(option_name if option_name is not None else ""),
        ))
    return sorted(signatures)


def _validate_timestamp_and_source(
    data: dict,
    loc_base: str,
    *,
    rule_id: str,
    source: str,
    source_required: bool = False,
) -> list[dict]:
    issues: list[dict] = []
    timestamp = _pick(data, "timestamp", "Timestamp")
    if timestamp is None:
        issues.append(_issue("error", "structure", loc_base, "timestamp",
                             "timestamp is required", None, "uint64",
                             rule_id=rule_id,
                             source=source))
    else:
        for issue in _check_uint64(timestamp, "timestamp", loc_base):
            issue.setdefault("ruleId", rule_id)
            issue.setdefault("source", source)
            issues.append(issue)

    src = _pick(data, "source", "Source", "requestModuleName", "RequestModuleName")
    if src is None:
        issues.append(_issue("error" if source_required else "warning", "structure", loc_base, "source",
                             "source is required" if source_required else "source is missing",
                             None, "string",
                             rule_id=rule_id,
                             source=source))
    elif not isinstance(src, str):
        issues.append(_issue("error", "type", loc_base, "source",
                             f"source must be string, got {type(src).__name__}",
                             src, "string",
                             rule_id=rule_id,
                             source=source))
    elif not src.strip():
        issues.append(_issue("warning", "rule", loc_base, "source",
                             "source is empty",
                             src, "non-empty module/source name",
                             rule_id=rule_id,
                             source=source))
    return issues


def _require_positive_uint(value: Any, field: str, location: str, *, rule_id: str, source: str) -> list[dict]:
    issues = _check_uint(value, field, location)
    int_value = _coerce_int(value)
    if int_value is not None and int_value <= 0:
        issues.append(_issue("error", "rule", location, field,
                             f"{field} must be positive",
                             int_value, "uint32 > 0",
                             rule_id=rule_id,
                             source=source))
    for issue in issues:
        if not issue.get("ruleId"):
            issue["ruleId"] = rule_id
        if not issue.get("source"):
            issue["source"] = source
    return issues


def _validate_0701_mission_plan_option_info(
    msg_key: str,
    data: dict,
    *,
    location_prefix: str = "0701",
    materialized_plan_ids: set[int] | None = None,
    pending_option_plan_ids: set[int] | None = None,
) -> list[dict]:
    issues: list[dict] = []
    loc_base = f"{location_prefix}/{msg_key}"
    rule_id = "NFUSION-0701"
    source = "ICD 0701 MissionPlanOptionInfo + modules.common.message0701"

    issues.extend(_validate_timestamp_and_source(data, loc_base, rule_id=rule_id, source=source))

    auto_execution = _pick(data, "autoExecution", "AutoExecution")
    if auto_execution is None:
        issues.append(_issue("error", "structure", loc_base, "autoExecution",
                             "autoExecution is required",
                             None, "bool",
                             rule_id=rule_id,
                             source=source))
    else:
        for issue in _check_bool(auto_execution, "autoExecution", loc_base):
            issue["ruleId"] = rule_id
            issue["source"] = source
            issues.append(issue)

    option_list = _pick(data, "optionList", "OptionList")
    issues.extend(_check_list(option_list, "optionList", loc_base, required=True, min_len=1))
    seen_option_ids: set[int] = set()
    seen_plan_ids: set[int] = set()
    recommend_count = 0
    for idx, option in enumerate(_as_list(option_list)):
        opt_loc = f"{loc_base} > optionList[{idx}]"
        if not isinstance(option, dict):
            issues.append(_issue("error", "type", opt_loc, "option",
                                 f"option must be object, got {type(option).__name__}",
                                 option, "Option object",
                                 rule_id=rule_id,
                                 source=source))
            continue

        option_id = _pick(option, "optionID", "OptionID")
        if option_id is None:
            issues.append(_issue("error", "structure", opt_loc, "optionID",
                                 "optionID is required", None, "uint32 > 0",
                                 rule_id=rule_id,
                                 source=source))
        else:
            issues.extend(_require_positive_uint(option_id, "optionID", opt_loc, rule_id=rule_id, source=source))
            option_int = _coerce_int(option_id)
            if option_int is not None and option_int > 0:
                if option_int in seen_option_ids:
                    issues.append(_issue("warning", "rule", opt_loc, "optionID",
                                         f"duplicate optionID {option_int}",
                                         option_int, "unique optionID",
                                         rule_id=rule_id,
                                         source=source))
                seen_option_ids.add(int(option_int))

        recommend = _pick(option, "recommend", "Recommend")
        if recommend is None:
            issues.append(_issue("error", "structure", opt_loc, "recommend",
                                 "recommend is required", None, "bool",
                                 rule_id=rule_id,
                                 source=source))
        else:
            for issue in _check_bool(recommend, "recommend", opt_loc):
                issue["ruleId"] = rule_id
                issue["source"] = source
                issues.append(issue)
            if recommend is True:
                recommend_count += 1

        option_name = _pick(option, "optionName", "OptionName")
        if option_name is None:
            issues.append(_issue("error", "structure", opt_loc, "optionName",
                                 "optionName is required", None, "uint32 enum",
                                 rule_id=rule_id,
                                 source=source))
        else:
            for issue in _check_uint(option_name, "optionName", opt_loc):
                issue["ruleId"] = rule_id
                issue["source"] = source
                issues.append(issue)
            option_code = _coerce_int(option_name)
            if option_code is not None:
                if option_code == 1:
                    issues.append(_issue("warning", "rule", opt_loc, "optionName",
                                         "optionName=1 is used by InternalMessage/msg_0701.cs as a system recommendation code, but msg_0701 ICD lists enum 2..6",
                                         option_code, "ICD enum 2..6; code extension 1",
                                         rule_id="ICD-CODE-MISMATCH",
                                         source="resource/nFusion msg_0701/Option.nftype vs InternalMessage/msg_0701.cs"))
                elif option_code not in {2, 3, 4, 5, 6}:
                    issues.append(_issue("error", "range", opt_loc, "optionName",
                                         f"optionName={option_code} is not a valid 0701 option enum",
                                         option_code, "enum 2=attack-specialized, 3=attack-exclusion, 4=recon-specialized, 5=min-time, 6=recon/time-balanced",
                                         rule_id=rule_id,
                                         source="ICD 0701 Option.optionName"))

        plan_id = _pick(option, "missionPlanID", "MissionPlanID")
        if plan_id is None:
            issues.append(_issue("error", "structure", opt_loc, "missionPlanID",
                                 "missionPlanID is required", None, "uint32 > 0",
                                 rule_id=rule_id,
                                 source=source))
        else:
            issues.extend(_require_positive_uint(plan_id, "missionPlanID", opt_loc, rule_id=rule_id, source=source))
            plan_int = _coerce_int(plan_id)
            if plan_int is not None and plan_int > 0:
                if plan_int in seen_plan_ids:
                    issues.append(_issue("warning", "rule", opt_loc, "missionPlanID",
                                         f"duplicate missionPlanID {plan_int} in optionList",
                                         plan_int, "unique missionPlanID per optionList",
                                         rule_id="PLAN-SELECTION-REF",
                                         source=source))
                seen_plan_ids.add(int(plan_int))
                known_0701_plan_sources = set(materialized_plan_ids or set()).union(pending_option_plan_ids or set())
                if known_0701_plan_sources and plan_int not in known_0701_plan_sources:
                    issues.append(_issue("warning", "rule", opt_loc, "missionPlanID",
                                         f"0701 advertises missionPlanID {plan_int}, but it is not present in MissionPlan/0301 or 0902 pending options",
                                         plan_int, f"one of {sorted(known_0701_plan_sources)[:30]}",
                                         rule_id="PLAN-SELECTION-REF",
                                         source="ICD 0701 + generated MissionPlan/0902 logs",
                                         hint="Do not count the 0701 option itself as proof that the plan exists."))

        for metric_field in (
            "survivalRate",
            "timeContraction",
            "recogEffectiveness",
        ):
            metric_value = _pick(option, metric_field, metric_field[:1].upper() + metric_field[1:])
            if metric_value is None:
                continue
            for issue in _check_icd_int(metric_value, metric_field, opt_loc):
                issue["ruleId"] = rule_id
                issue["source"] = source
                issues.append(issue)
            metric_int = _coerce_int(metric_value)
            if metric_int is not None and metric_int not in {-1, 0, 1}:
                issues.append(_issue("error", "range", opt_loc, metric_field,
                                     f"{metric_field}={metric_int} is not a valid 0701 effect enum",
                                     metric_int, "enum -1, 0, 1",
                                     rule_id=rule_id,
                                     source="ICD 0701 Option effect enum"))

        fuel_warning = _pick(option, "fuelWarning", "FuelWarning")
        if fuel_warning is not None:
            for issue in _check_icd_int(fuel_warning, "fuelWarning", opt_loc):
                issue["ruleId"] = rule_id
                issue["source"] = source
                issues.append(issue)
            fuel_int = _coerce_int(fuel_warning)
            if fuel_int is not None and fuel_int not in {-1, 0}:
                issues.append(_issue("error", "range", opt_loc, "fuelWarning",
                                     f"fuelWarning={fuel_int} is not a valid 0701 fuel warning enum",
                                     fuel_int, "enum -1=fuel-shortage, 0=fuel-good, null=not provided",
                                     rule_id=rule_id,
                                     source="ICD 0701 Option.fuelWarning enum"))

        for uint_field in ("distance", "target"):
            uint_value = _pick(option, uint_field, uint_field[:1].upper() + uint_field[1:])
            if uint_value is None:
                continue
            for issue in _check_uint(uint_value, uint_field, opt_loc):
                issue["ruleId"] = rule_id
                issue["source"] = source
                issues.append(issue)

    if auto_execution is True and recommend_count == 0:
        issues.append(_issue("warning", "rule", loc_base, "optionList",
                             "autoExecution=true but no option is marked recommend=true",
                             recommend_count, "at least one recommended option",
                             rule_id="PLAN-SELECTION-REF",
                             source=source))
    elif auto_execution is True and recommend_count > 1:
        issues.append(_issue("error", "rule", loc_base, "optionList",
                             f"autoExecution=true but {recommend_count} options are marked recommend=true",
                             recommend_count, "exactly one recommended option",
                             rule_id="PLAN-SELECTION-REF",
                             source=source))
    return issues


def _check_selected_plan_reference(
    plan_id: Any,
    field: str,
    loc_base: str,
    *,
    known_plan_ids: set[int],
    pending_option_plan_ids: set[int],
    offered_option_plan_ids: set[int] | None = None,
    transmitted_0301_plan_ids: set[int] | None = None,
    require_transmitted_0301: bool = False,
) -> list[dict]:
    issues: list[dict] = []
    rule_id = "PLAN-SELECTION-REF"
    source = "ICD 0702/0903 + MissionPlan/MissionPlanOptionInfo/0902"
    plan_int = _coerce_int(plan_id)
    if plan_int is None or plan_int <= 0:
        return issues
    candidate_ids = set(known_plan_ids or set()).union(pending_option_plan_ids or set())
    if candidate_ids and plan_int not in candidate_ids:
        issues.append(_issue("error", "rule", loc_base, field,
                             f"selected missionPlanID {plan_int} is not present in generated/announced plan candidates",
                             plan_int, f"one of {sorted(candidate_ids)[:30]}",
                             rule_id=rule_id,
                             source=source,
                             hint="Check MissionPlan, MissionPlanOptionInfo/0701, and 0902 pendingOptionList logs."))
    elif known_plan_ids and plan_int not in known_plan_ids:
        issues.append(_issue("warning", "rule", loc_base, field,
                             f"selected missionPlanID {plan_int} only appears in 0902 pending options; no generated/announced plan payload was found",
                             plan_int, "MissionPlan or MissionPlanOptionInfo/0701 payload",
                             rule_id=rule_id,
                             source=source))
    if offered_option_plan_ids and plan_int not in offered_option_plan_ids:
        issues.append(_issue("warning", "rule", loc_base, field,
                             f"selected missionPlanID {plan_int} was not present in any 0701 optionList",
                             plan_int, f"one of {sorted(offered_option_plan_ids)[:30]}",
                             rule_id=rule_id,
                             source="ICD 0701/0702 pilot decision flow"))
    if require_transmitted_0301 and transmitted_0301_plan_ids and plan_int not in transmitted_0301_plan_ids:
        issues.append(_issue("error", "rule", loc_base, field,
                             f"0903 requested missionPlanID {plan_int}, but no matching 0301 transmission exists",
                             plan_int, f"0301 missionPlanID one of {sorted(transmitted_0301_plan_ids)[:30]}",
                             rule_id=rule_id,
                             source="mission_planning_gui post-0301 apply flow"))
    return issues


def _validate_0903_request_renew_mission(
    msg_key: str,
    data: dict,
    known_plan_ids: set[int],
    pending_option_plan_ids: set[int],
    transmitted_0301_plan_ids: set[int],
) -> list[dict]:
    issues: list[dict] = []
    loc_base = f"0903/{msg_key}"
    rule_id = "NFUSION-0903"
    source = "ICD 0903 RequestRenewMission"

    issues.extend(_validate_timestamp_and_source(data, loc_base, rule_id=rule_id, source=source))
    plan_id = _pick(data, "missionPlanID", "MissionPlanID", "missionPlanId", "MissionPlanId")
    if plan_id is None:
        issues.append(_issue("error", "structure", loc_base, "missionPlanID",
                             "missionPlanID is required", None, "uint32 > 0",
                             rule_id=rule_id,
                             source=source))
        return issues
    issues.extend(_require_positive_uint(plan_id, "missionPlanID", loc_base, rule_id=rule_id, source=source))
    issues.extend(_check_selected_plan_reference(
        plan_id,
        "missionPlanID",
        loc_base,
        known_plan_ids=known_plan_ids,
        pending_option_plan_ids=pending_option_plan_ids,
        transmitted_0301_plan_ids=transmitted_0301_plan_ids,
        require_transmitted_0301=True,
    ))
    return issues


def _validate_0702_pilot_decision(
    msg_key: str,
    data: dict,
    known_plan_ids: set[int],
    pending_option_plan_ids: set[int],
    offered_option_plan_ids: set[int],
    offered_option_records: list[dict[str, Any]] | None = None,
) -> list[dict]:
    issues: list[dict] = []
    loc_base = f"0702/{msg_key}"
    rule_id = "NFUSION-0702"
    source = "ICD 0702 PilotDecision"

    issues.extend(_validate_timestamp_and_source(data, loc_base, rule_id=rule_id, source=source))
    ignore = _pick(data, "ignore", "Ignore")
    if ignore is None:
        issues.append(_issue("error", "structure", loc_base, "ignore",
                             "ignore is required", None, "enum 0=None, 1=keep existing, 2=select missionPlanID",
                             rule_id=rule_id,
                             source=source))
    else:
        for issue in _check_enum(ignore, "ignore", loc_base, {0, 1, 2}, {0: "None", 1: "existing mission", 2: "MissionPlanID selected"}):
            issue["ruleId"] = rule_id
            issue["source"] = source
            issues.append(issue)

    plan_id = _pick(data, "missionPlanID", "MissionPlanID", "missionPlanId", "MissionPlanId")
    if plan_id is None:
        issues.append(_issue("error", "structure", loc_base, "missionPlanID",
                             "missionPlanID is required by ICD 0702",
                             None, "uint32",
                             rule_id=rule_id,
                             source=source))
        return issues
    for issue in _check_uint(plan_id, "missionPlanID", loc_base):
        issue["ruleId"] = rule_id
        issue["source"] = source
        issues.append(issue)

    ignore_int = _coerce_int(ignore)
    plan_int = _coerce_int(plan_id)
    if ignore_int == 2:
        if plan_int is None or plan_int <= 0:
            issues.append(_issue("error", "rule", loc_base, "missionPlanID",
                                 "ignore=2 selects by missionPlanID, so missionPlanID must be positive",
                                 plan_id, "uint32 > 0",
                                 rule_id="PLAN-SELECTION-REF",
                                 source=source))
        else:
            issues.extend(_check_selected_plan_reference(
                plan_id,
                "missionPlanID",
                loc_base,
                known_plan_ids=known_plan_ids,
                pending_option_plan_ids=pending_option_plan_ids,
                offered_option_plan_ids=offered_option_plan_ids,
            ))
            decision_ts = _coerce_int(_pick(data, "timestamp", "Timestamp"))
            if decision_ts is not None and offered_option_records:
                preceding = [
                    record
                    for record in offered_option_records
                    if isinstance(record, dict)
                    and isinstance(record.get("planIds"), set)
                    and record.get("planIds")
                    and _coerce_int(record.get("timestamp")) is not None
                    and int(_coerce_int(record.get("timestamp")) or 0) <= int(decision_ts)
                ]
                if preceding:
                    latest = max(preceding, key=lambda item: int(_coerce_int(item.get("timestamp")) or 0))
                    latest_plan_ids = set(latest.get("planIds") or set())
                    if plan_int not in latest_plan_ids:
                        issues.append(_issue("warning", "rule", loc_base, "missionPlanID",
                                             f"0702 selected missionPlanID {plan_int}, but the nearest preceding 0701 candidate set did not include it",
                                             plan_int, f"0701/{latest.get('key')} candidates {sorted(latest_plan_ids)[:30]}",
                                             rule_id="PLAN-SELECTION-REF",
                                             source="ICD 0701/0702 pilot decision temporal flow",
                                             hint="This may indicate the pilot decision applied an older/newer option set or logs are out of order."))
                else:
                    issues.append(_issue("warning", "rule", loc_base, "missionPlanID",
                                         "0702 selects a missionPlanID, but no preceding 0701 optionList was found in the log",
                                         plan_int, "0701 optionList before 0702 timestamp",
                                         rule_id="PLAN-SELECTION-REF",
                                         source="ICD 0701/0702 pilot decision temporal flow"))
    elif ignore_int in {0, 1} and plan_int is not None and plan_int > 0:
        issues.append(_issue("warning", "rule", loc_base, "missionPlanID",
                             f"ignore={ignore_int} does not select a new plan, but missionPlanID is positive",
                             plan_int, "0 or omitted when ignore is 0/1",
                             rule_id="PLAN-SELECTION-REF",
                             source=source))
    return issues


def _validate_0904_request_behavior_tree(msg_key: str, data: dict) -> list[dict]:
    issues: list[dict] = []
    loc_base = f"0904/{msg_key}"
    rule_id = "NFUSION-0904"
    source = "ICD 0904 RequestBehaviorTree + modules.common.message0904"

    issues.extend(_validate_timestamp_and_source(data, loc_base, rule_id=rule_id, source=source))
    mode = _pick(data, "mode", "Mode")
    behavior_tree_file_id = _pick(data, "BehaviorTreeFileID", "behaviorTreeFileID", "behaviorTreeFileId")
    if mode is None and behavior_tree_file_id is None:
        issues.append(_issue("error", "structure", loc_base, "mode",
                             "0904 must include ICD mode or generated-code BehaviorTreeFileID",
                             None, "mode enum {0,2} or BehaviorTreeFileID uint32",
                             rule_id=rule_id,
                             source=source,
                             hint="ICD 0904 defines mode, while current generated push/receive code serializes BehaviorTreeFileID."))
    if mode is not None:
        for issue in _check_uint(mode, "mode", loc_base):
            issue["ruleId"] = rule_id
            issue["source"] = source
            issues.append(issue)
        mode_int = _coerce_int(mode)
        if mode_int is not None and mode_int not in {0, 1, 2}:
            issues.append(_issue("error", "range", loc_base, "mode",
                                 f"mode={mode_int} is not valid for 0904",
                                 mode_int, "0=stop, 1=start/internal-code, 2=run/ICD",
                                 rule_id=rule_id,
                                 source=source))
        elif mode_int == 1:
            issues.append(_issue("warning", "rule", loc_base, "mode",
                                 "mode=1 is used by InternalMessage/msg_0904.cs logs, but msg_0904 ICD lists enum {0,2}",
                                 mode_int, "ICD enum {0,2} or code enum {0,1}",
                                 rule_id="ICD-CODE-MISMATCH",
                                 source="resource/nFusion msg_0904.nfpsh vs InternalMessage/msg_0904.cs",
                                 hint="Confirm whether 0904 start/run should be encoded as 1 or 2, then update ICD or generated bindings."))
    if behavior_tree_file_id is not None:
        for issue in _check_uint(behavior_tree_file_id, "BehaviorTreeFileID", loc_base):
            issue["ruleId"] = rule_id
            issue["source"] = source
            issues.append(issue)
    if mode is not None and behavior_tree_file_id is not None:
        issues.append(_issue("warning", "rule", loc_base, "0904",
                             "0904 contains both ICD mode and generated-code BehaviorTreeFileID",
                             {"mode": mode, "BehaviorTreeFileID": behavior_tree_file_id},
                             "only one control field",
                             rule_id=rule_id,
                             source=source))
    return issues


def _collect_prior_mission_ids_from_obj(obj: Any) -> set[int]:
    ids: set[int] = set()
    for item in _walk_json_values(obj):
        if not isinstance(item, dict):
            continue
        mission_id = _coerce_int(_pick(item, "priorMissionID", "PriorMissionID"))
        if mission_id is not None and mission_id > 0:
            ids.add(int(mission_id))
    return ids


def _validate_prior_mission_info(msg_key: str, data: dict, target_ids: set[int] | None = None) -> list[dict]:
    issues: list[dict] = []
    loc_base = f"PriorMissionInfo/{msg_key}"

    timestamp = _pick(data, "timestamp", "Timestamp")
    if timestamp is None:
        issues.append(_issue("error", "structure", loc_base, "timestamp",
                             "timestamp is required", None, "uint64"))
    else:
        issues.extend(_check_uint64(timestamp, "timestamp", loc_base))

    source = _pick(data, "source", "Source")
    if source is None:
        issues.append(_issue("warning", "structure", loc_base, "source",
                             "source is missing", None, "string"))
    elif not isinstance(source, str):
        issues.append(_issue("error", "type", loc_base, "source",
                             f"source must be string, got {type(source).__name__}",
                             source, "string"))

    mission_list = _pick(data, "priorMissionList", "PriorMissionList")
    issues.extend(_check_list(mission_list, "priorMissionList", loc_base, required=True, min_len=1))
    seen: set[int] = set()
    for idx, mission in enumerate(_as_list(mission_list)):
        m_loc = f"{loc_base} > PriorMission[{idx}]"
        if not isinstance(mission, dict):
            issues.append(_issue("error", "type", m_loc, "priorMission",
                                 f"priorMission must be object, got {type(mission).__name__}",
                                 mission, "PriorMission object"))
            continue

        prior_id = _pick(mission, "priorMissionID", "PriorMissionID")
        issues.extend(_check_uint(prior_id, "priorMissionID", m_loc))
        prior_int = _coerce_int(prior_id)
        if prior_int is None or prior_int <= 0:
            issues.append(_issue("error", "rule", m_loc, "priorMissionID",
                                 "priorMissionID must be positive",
                                 prior_id, "uint > 0",
                                 rule_id="PRIOR-MISSION-REF",
                                 source="ICD 0202 PriorMissionInfo"))
        elif prior_int in seen:
            issues.append(_issue("error", "rule", m_loc, "priorMissionID",
                                 f"duplicate priorMissionID {prior_int}",
                                 prior_int, "unique priorMissionID",
                                 rule_id="PRIOR-MISSION-REF",
                                 source="ICD 0202 PriorMissionInfo"))
        else:
            seen.add(prior_int)

        mission_type = _pick(mission, "missionType", "MissionType")
        issues.extend(_check_enum(mission_type, "missionType", m_loc, {1, 2}))
        mt = _coerce_int(mission_type)
        coord_orientation = _pick(mission, "coordinateOrientation", "CoordinateOrientation")
        target_orientation = _pick(mission, "targetOrientation", "TargetOrientation")

        if mt == 1:
            if not isinstance(coord_orientation, dict):
                issues.append(_issue("error", "structure", m_loc, "coordinateOrientation",
                                     "missionType=1 requires coordinateOrientation",
                                     coord_orientation, "CoordinateOrientation",
                                     rule_id="PRIOR-MISSION-REF",
                                     source="ICD 0202 PriorMissionInfo"))
            else:
                coord = _pick(coord_orientation, "coordinate", "Coordinate")
                if coord is None:
                    issues.append(_issue("error", "structure", m_loc, "coordinateOrientation.coordinate",
                                         "coordinateOrientation.coordinate is required",
                                         None, "Coordinate"))
                else:
                    issues.extend(_validate_coordinate(coord, m_loc))
            if _is_meaningful(target_orientation):
                issues.append(_issue("warning", "rule", m_loc, "targetOrientation",
                                     "missionType=1 should not carry targetOrientation data",
                                     target_orientation, "empty targetOrientation",
                                     rule_id="PRIOR-MISSION-REF",
                                     source="ICD 0202 PriorMissionInfo"))
        elif mt == 2:
            if not isinstance(target_orientation, dict):
                issues.append(_issue("error", "structure", m_loc, "targetOrientation",
                                     "missionType=2 requires targetOrientation",
                                     target_orientation, "TargetOrientation",
                                     rule_id="PRIOR-MISSION-REF",
                                     source="ICD 0202 PriorMissionInfo"))
            else:
                target_id = _pick(target_orientation, "targetID", "TargetID", "targetId")
                issues.extend(_check_uint(target_id, "targetOrientation.targetID", m_loc))
                _validate_target_reference(
                    target_id,
                    "targetOrientation.targetID",
                    m_loc,
                    issues,
                    target_ids,
                    "PriorMissionInfo.targetOrientation",
                )
            if _is_meaningful(coord_orientation):
                issues.append(_issue("warning", "rule", m_loc, "coordinateOrientation",
                                     "missionType=2 should not carry coordinateOrientation data",
                                     coord_orientation, "empty coordinateOrientation",
                                     rule_id="PRIOR-MISSION-REF",
                                     source="ICD 0202 PriorMissionInfo"))

    return issues


def _extract_replan_id_value(item: Any, field: str) -> Any:
    if isinstance(item, dict):
        return _pick(item, field, field[0].upper() + field[1:])
    return item


def _validate_replan_id_list(
    entries: Any,
    list_field: str,
    id_field: str,
    loc_base: str,
    issues: list[dict],
    existing_ids: set[int],
) -> set[int]:
    seen: set[int] = set()
    if entries is None:
        return seen
    issues.extend(_check_list(entries, list_field, loc_base))
    for idx, item in enumerate(_as_list(entries)):
        item_loc = f"{loc_base} > {list_field}[{idx}]"
        if isinstance(item, dict):
            value = _pick(item, id_field, id_field[0].upper() + id_field[1:])
        else:
            value = item
        if value is None:
            issues.append(_issue("error", "structure", item_loc, id_field,
                                 f"{id_field} is required", None, "uint > 0",
                                 rule_id="REPLAN-REF",
                                 source="ICD 0902 ReplanRequest"))
            continue
        issues.extend(_check_uint(value, id_field, item_loc))
        int_value = _coerce_int(value)
        if int_value is None:
            continue
        if int_value <= 0:
            issues.append(_issue("error", "rule", item_loc, id_field,
                                 f"{id_field} must be positive",
                                 int_value, "uint > 0",
                                 rule_id="REPLAN-REF",
                                 source="ICD 0902 ReplanRequest"))
            continue
        if int_value in seen:
            issues.append(_issue("warning", "rule", item_loc, id_field,
                                 f"duplicate {id_field} {int_value} in {list_field}",
                                 int_value, f"unique {id_field}",
                                 rule_id="REPLAN-REF",
                                 source="ICD 0902 ReplanRequest"))
        seen.add(int(int_value))
        if existing_ids and int_value not in existing_ids:
            issues.append(_issue("error", "rule", item_loc, id_field,
                                 f"{id_field} {int_value} does not resolve to generated mission data",
                                 int_value, f"one of {sorted(existing_ids)[:20]}",
                                 rule_id="REPLAN-REF",
                                 source="ICD 0902 ReplanRequest + generated mission logs"))
    return seen


def _validate_replan_request_0902(
    msg_key: str,
    data: dict,
    existing_input_mission_ids: set[int],
    existing_individual_mission_ids: set[int],
    existing_mission_plan_ids: set[int],
    target_ids: set[int] | None = None,
) -> list[dict]:
    issues: list[dict] = []
    loc_base = f"0902/{msg_key}"

    timestamp = _pick(data, "timestamp", "Timestamp")
    if timestamp is None:
        issues.append(_issue("error", "structure", loc_base, "timestamp",
                             "timestamp is required", None, "uint64",
                             rule_id="NFUSION-0902",
                             source="ICD 0902 ReplanRequest"))
    else:
        issues.extend(_check_uint64(timestamp, "timestamp", loc_base))

    source = _pick(data, "source", "Source")
    if source is None:
        issues.append(_issue("warning", "structure", loc_base, "source",
                             "source is missing", None, "string",
                             rule_id="NFUSION-0902",
                             source="ICD 0902 ReplanRequest"))
    elif not isinstance(source, str):
        issues.append(_issue("error", "type", loc_base, "source",
                             f"source must be string, got {type(source).__name__}",
                             source, "string",
                             rule_id="NFUSION-0902",
                             source="ICD 0902 ReplanRequest"))

    request_time = _pick(data, "replanRequestTime", "ReplanRequestTime")
    if not isinstance(request_time, dict):
        issues.append(_issue("error", "structure", loc_base, "replanRequestTime",
                             "replanRequestTime object is required",
                             request_time, "ReplanRequestTime",
                             rule_id="NFUSION-0902",
                             source="ICD 0902 ReplanRequest"))
    else:
        req_ts = _pick(request_time, "replanRequestTimestamp", "ReplanRequestTimestamp")
        if req_ts is None:
            issues.append(_issue("error", "structure", loc_base, "replanRequestTime.replanRequestTimestamp",
                                 "replanRequestTimestamp is required",
                                 None, "uint64",
                                 rule_id="NFUSION-0902",
                                 source="ICD 0902 ReplanRequestTime"))
        else:
            issues.extend(_check_uint64(req_ts, "replanRequestTime.replanRequestTimestamp", loc_base))

    replan_level = _pick(data, "replanLevel", "ReplanLevel")
    if replan_level is None:
        issues.append(_issue("error", "structure", loc_base, "replanLevel",
                             "replanLevel is required", None, "enum 0..4",
                             rule_id="NFUSION-0902",
                             source="ICD 0902 ReplanRequest"))
    else:
        issues.extend(_check_enum(replan_level, "replanLevel", loc_base, {0, 1, 2, 3, 4}))

    input_ids = _validate_replan_id_list(
        _pick(data, "inputMissionIDList", "InputMissionIDList"),
        "inputMissionIDList",
        "inputMissionID",
        loc_base,
        issues,
        existing_input_mission_ids,
    )
    individual_ids = _validate_replan_id_list(
        _pick(data, "individualMissionIDList", "IndividualMissionIDList"),
        "individualMissionIDList",
        "individualMissionID",
        loc_base,
        issues,
        existing_individual_mission_ids,
    )

    prior_list = _pick(data, "priorMissionList", "PriorMissionList")
    if prior_list is not None:
        issues.extend(_check_list(prior_list, "priorMissionList", loc_base))
        seen_prior: set[int] = set()
        for idx, mission in enumerate(_as_list(prior_list)):
            m_loc = f"{loc_base} > priorMissionList[{idx}]"
            if not isinstance(mission, dict):
                issues.append(_issue("error", "type", m_loc, "priorMission",
                                     f"priorMission must be object, got {type(mission).__name__}",
                                     mission, "PriorMission object",
                                     rule_id="PRIOR-MISSION-REF",
                                     source="ICD 0902 ReplanRequest"))
                continue
            prior_id = _coerce_int(_pick(mission, "priorMissionID", "PriorMissionID"))
            issues.extend(_validate_prior_mission_info(
                f"{msg_key}.priorMissionList[{idx}]",
                {
                    "timestamp": timestamp if timestamp is not None else 0,
                    "source": source if isinstance(source, str) else "",
                    "priorMissionList": [mission],
                },
                target_ids,
            ))
            if prior_id is not None and prior_id > 0:
                if prior_id in seen_prior:
                    issues.append(_issue("warning", "rule", m_loc, "priorMissionID",
                                         f"duplicate priorMissionID {prior_id} in priorMissionList",
                                         prior_id, "unique priorMissionID",
                                         rule_id="PRIOR-MISSION-REF",
                                         source="ICD 0902 ReplanRequest"))
                seen_prior.add(prior_id)

    reason = _pick(data, "replanReason", "ReplanReason", "replanRequest", "ReplanRequest")
    if reason is None:
        issues.append(_issue("warning", "structure", loc_base, "replanReason",
                             "replan reason text is missing", None, "string",
                             rule_id="NFUSION-0902",
                             source="ICD 0902 ReplanRequest"))
    elif not isinstance(reason, str):
        issues.append(_issue("error", "type", loc_base, "replanReason",
                             f"replan reason must be string, got {type(reason).__name__}",
                             reason, "string",
                             rule_id="NFUSION-0902",
                             source="ICD 0902 ReplanRequest"))

    pending_option_list = _pick(data, "pendingOptionList", "PendingOptionList")
    alt_option_list = _pick(data, "optionList", "OptionList")
    if pending_option_list is not None and alt_option_list is not None:
        pending_sig = _option_signature_list(pending_option_list)
        alt_sig = _option_signature_list(alt_option_list)
        if pending_sig != alt_sig:
            issues.append(_issue("warning", "rule", loc_base, "pendingOptionList",
                                 "pendingOptionList and optionList are both present but contain different option IDs/plan IDs/names",
                                 {"pendingOptionList": pending_sig, "optionList": alt_sig},
                                 "matching option payloads or only pendingOptionList",
                                 rule_id="REPLAN-REF",
                                 source="ICD 0902 ReplanRequest + common receiver aliases",
                                 hint="The receiver may expose optionList while push code writes pendingOptionList; mismatches can make downstream plan selection ambiguous."))
    option_list = pending_option_list if pending_option_list is not None else alt_option_list
    option_count = 0
    if option_list is not None:
        issues.extend(_check_list(option_list, "pendingOptionList", loc_base))
        seen_options: set[int] = set()
        for idx, option in enumerate(_as_list(option_list)):
            option_count += 1
            o_loc = f"{loc_base} > pendingOptionList[{idx}]"
            if not isinstance(option, dict):
                issues.append(_issue("error", "type", o_loc, "pendingOption",
                                     f"pendingOption must be object, got {type(option).__name__}",
                                     option, "PendingOption object",
                                     rule_id="REPLAN-REF",
                                     source="ICD PendingOption"))
                continue
            option_id = _pick(option, "optionID", "OptionID")
            issues.extend(_check_uint(option_id, "optionID", o_loc))
            option_int = _coerce_int(option_id)
            if option_int is not None:
                if option_int <= 0:
                    issues.append(_issue("error", "rule", o_loc, "optionID",
                                         "optionID must be positive",
                                         option_int, "uint > 0",
                                         rule_id="REPLAN-REF",
                                         source="ICD PendingOption"))
                elif option_int in seen_options:
                    issues.append(_issue("warning", "rule", o_loc, "optionID",
                                         f"duplicate optionID {option_int}",
                                         option_int, "unique optionID",
                                         rule_id="REPLAN-REF",
                                         source="ICD PendingOption"))
                seen_options.add(option_int)
            option_name = _pick(option, "optionName", "OptionName")
            if option_name is not None and not isinstance(option_name, str):
                if isinstance(option_name, int) and not isinstance(option_name, bool):
                    issues.append(_issue("warning", "type", o_loc, "optionName",
                                         f"optionName should be string on the 0902 wire type, got numeric option code {option_name}",
                                         option_name, "string option label/name",
                                         rule_id="ICD-CODE-MISMATCH",
                                         source="CommonType.PendingOption.nftype vs mission planning option code",
                                         hint="Normalize the numeric option code to a string label before push if this is the transmitted 0902 payload."))
                else:
                    issues.append(_issue("error", "type", o_loc, "optionName",
                                         f"optionName must be string, got {type(option_name).__name__}",
                                         option_name, "string",
                                         rule_id="REPLAN-REF",
                                         source="ICD PendingOption"))
            plan_id = _pick(option, "missionPlanID", "MissionPlanID")
            issues.extend(_check_uint(plan_id, "missionPlanID", o_loc))
            plan_int = _coerce_int(plan_id)
            if existing_mission_plan_ids and plan_int is not None and plan_int > 0 and plan_int not in existing_mission_plan_ids:
                issues.append(_issue("warning", "rule", o_loc, "missionPlanID",
                                     f"pending option missionPlanID {plan_int} is not present in MissionPlan/MissionPlanOptionInfo logs",
                                     plan_int, f"one of {sorted(existing_mission_plan_ids)[:20]}",
                                     rule_id="REPLAN-REF",
                                     source="ICD PendingOption + generated MissionPlan logs"))

    if not input_ids and not individual_ids and not _as_list(prior_list) and option_count == 0:
        issues.append(_issue("warning", "rule", loc_base, "replanRequest",
                             "replan request contains no mission identifiers or pending options",
                             None, "inputMissionIDList, individualMissionIDList, priorMissionList, or pendingOptionList",
                             rule_id="REPLAN-REF",
                             source="ICD 0902 ReplanRequest"))

    return issues


def _replan_request_signature(record: dict) -> str:
    def _ids(list_value: Any, field: str) -> list[int]:
        values: list[int] = []
        for item in _as_list(list_value):
            value = _extract_replan_id_value(item, field)
            int_value = _coerce_int(value)
            if int_value is not None:
                values.append(int(int_value))
        return sorted(values)

    options: list[tuple[int | None, int | None, str]] = []
    for option in _as_list(_pick(record, "pendingOptionList", "PendingOptionList", "optionList", "OptionList")):
        if not isinstance(option, dict):
            continue
        options.append((
            _coerce_int(_pick(option, "optionID", "OptionID")),
            _coerce_int(_pick(option, "missionPlanID", "MissionPlanID")),
            str(_pick(option, "optionName", "OptionName", default="") or ""),
        ))
    prior_ids = [
        int(value)
        for value in (
            _coerce_int(_pick(item, "priorMissionID", "PriorMissionID"))
            for item in _as_list(_pick(record, "priorMissionList", "PriorMissionList"))
            if isinstance(item, dict)
        )
        if value is not None
    ]
    payload = {
        "timestamp": _coerce_int(_pick(record, "timestamp", "Timestamp")),
        "source": str(_pick(record, "source", "Source", default="") or ""),
        "replanLevel": _coerce_int(_pick(record, "replanLevel", "ReplanLevel")),
        "inputMissionIDList": _ids(_pick(record, "inputMissionIDList", "InputMissionIDList"), "inputMissionID"),
        "individualMissionIDList": _ids(_pick(record, "individualMissionIDList", "IndividualMissionIDList"), "individualMissionID"),
        "priorMissionList": sorted(prior_ids),
        "options": sorted(options),
        "reason": str(_pick(record, "replanReason", "ReplanReason", "replanRequest", "ReplanRequest", default="") or ""),
    }
    try:
        import json

        return json.dumps(payload, sort_keys=True, ensure_ascii=False)
    except Exception:
        return repr(sorted(payload.items()))


def _validate_mission_reference_package(pkg_key: str, data: dict) -> list[dict]:
    issues: list[dict] = []
    loc_base = f"MissionReferenceInfo/{pkg_key}"

    for field in ("timestamp", "missionReferencePackageID"):
        value = _pick(data, field, field[0].upper() + field[1:])
        if value is None:
            expected = "uint64" if field == "timestamp" else "uint32"
            issues.append(_issue("error", "structure", loc_base, field,
                                 f"{field} is required", None, expected))
        elif field == "timestamp":
            issues.extend(_check_uint64(value, field, loc_base))
        else:
            issues.extend(_check_uint(value, field, loc_base))

    ref_id = _coerce_int(_pick(data, "missionReferencePackageID", "MissionReferencePackageID", default=pkg_key))
    key_int = _coerce_int(pkg_key)
    if key_int is not None and ref_id is not None and key_int != ref_id:
        issues.append(_issue("error", "rule", loc_base, "missionReferencePackageID",
                             f"file name reference package ID {key_int} does not match payload ID {ref_id}",
                             ref_id, f"{key_int}"))

    input_ts = _pick(data, "inputTimestamp", "InputTimestamp")
    if input_ts is not None:
        issues.extend(_check_uint64(input_ts, "inputTimestamp", loc_base))

    for list_field in ("takeOverInfoList", "handOverInfoList"):
        entries = _pick(data, list_field, list_field[0].upper() + list_field[1:])
        if entries is None:
            continue
        issues.extend(_check_list(entries, list_field, loc_base))
        for idx, entry in enumerate(_as_list(entries)):
            e_loc = f"{loc_base} > {list_field}[{idx}]"
            if not isinstance(entry, dict):
                issues.append(_issue("error", "type", e_loc, list_field,
                                     f"{list_field} entry must be object, got {type(entry).__name__}",
                                     entry, "object with aircraftID/coordinate"))
                continue
            issues.extend(_check_enum(_pick(entry, "aircraftID", "AircraftID"), "aircraftID", e_loc, {0, 1, 2, 3, 4, 5, 6}))
            coord = _pick(entry, "coordinate", "Coordinate")
            if coord is not None:
                issues.extend(_validate_coordinate(coord, e_loc))

    rtb_list = _pick(data, "rtbCoordinateList", "RTBCoordinateList")
    if rtb_list is not None:
        issues.extend(_check_list(rtb_list, "rtbCoordinateList", loc_base))
        for idx, coord in enumerate(_as_list(rtb_list)):
            issues.extend(_validate_coordinate(coord, f"{loc_base} > rtbCoordinateList[{idx}]"))

    for area_field, id_field in (("flightAreaList", "flightAreaID"), ("prohibitedAreaList", "prohibitedAreaID")):
        areas = _pick(data, area_field, area_field[0].upper() + area_field[1:])
        if areas is None:
            continue
        issues.extend(_check_list(areas, area_field, loc_base))
        for idx, area in enumerate(_as_list(areas)):
            a_loc = f"{loc_base} > {area_field}[{idx}]"
            if not isinstance(area, dict):
                issues.append(_issue("error", "type", a_loc, area_field,
                                     f"{area_field} entry must be object, got {type(area).__name__}",
                                     area, "area object"))
                continue
            issues.extend(_check_uint(_pick(area, id_field, id_field[0].upper() + id_field[1:]), id_field, a_loc))
            latlons = _pick(area, "areaLatLonList", "AreaLatLonList")
            if isinstance(latlons, list):
                for c_idx, point in enumerate(latlons):
                    issues.extend(_validate_lat_lon(point, f"{a_loc} > areaLatLonList[{c_idx}]", "areaLatLon"))

    return issues


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

    target_ids: set[int] | None = None
    target_candidate_dirs = [
        base / "TargetInfo",
        base / "TargetInformation",
        base / "SituationAwarenessInfo",
        base / "0402",
        base / "52311",
    ]
    target_candidate_files = [base / "DSS_Internal" / "targetInfo.json"]
    for target_dir in target_candidate_dirs:
        if target_dir.is_dir():
            target_candidate_files.extend(sorted(target_dir.glob("*.json")))
    for target_file in target_candidate_files:
        if not target_file.is_file():
            continue
        try:
            target_data = json.loads(target_file.read_text(encoding="utf-8"))
        except Exception:
            issues.append(_issue("warning", "structure", "TargetInfo", "file",
                                 f"failed to parse target source: {target_file.name}",
                                 None, "valid JSON"))
            continue
        if target_ids is None:
            target_ids = set()
        target_ids.update(_collect_target_ids_from_obj(target_data))

    prior_mission_ids: set[int] | None = None
    replan_request_records: list[tuple[str, dict]] = []
    prior_candidate_dirs = [
        base / "PriorMissionInfo",
        base / "0202",
        base / "0902",
        base / "ReplanRequest",
        base / "DSS_Internal" / "replan_request_transport",
    ]
    for prior_dir in prior_candidate_dirs:
        if not prior_dir.is_dir():
            continue
        for fp in sorted(prior_dir.glob("*.json")):
            try:
                data = json.loads(fp.read_text(encoding="utf-8"))
            except Exception:
                issues.append(_issue("warning", "structure", f"PriorMissionInfo/{fp.stem}", "file",
                                     f"failed to parse prior mission source: {fp.name}",
                                     None, "valid JSON"))
                continue
            records = data if isinstance(data, list) else [data]
            if prior_mission_ids is None:
                prior_mission_ids = set()
            for idx, record in enumerate(records):
                if not isinstance(record, dict):
                    issues.append(_issue("error", "type", f"PriorMissionInfo/{fp.stem}[{idx}]", "message",
                                         f"prior mission source must be object, got {type(record).__name__}",
                                         record, "message object"))
                    continue
                prior_mission_ids.update(_collect_prior_mission_ids_from_obj(record))
                key = f"{prior_dir.name}/{fp.stem}[{idx}]" if isinstance(data, list) else f"{prior_dir.name}/{fp.stem}"
                if prior_dir.name in {"0902", "ReplanRequest", "replan_request_transport"}:
                    replan_request_records.append((key, record))
                if prior_dir.name in {"PriorMissionInfo", "0202"}:
                    issues.extend(_validate_prior_mission_info(key, record, target_ids))

    # --- InputMissionPlan / MissionReferenceInfo roots ---
    existing_input_package_ids: set[int] = set()
    existing_input_mission_ids: set[int] = set()
    existing_reference_package_ids: set[int] = set()

    input_dir = base / "InputMissionPlan"
    if input_dir.is_dir():
        for fp in sorted(input_dir.glob("*.json")):
            try:
                data = json.loads(fp.read_text(encoding="utf-8"))
            except Exception:
                issues.append(_issue("error", "structure", f"InputMissionPlan/{fp.stem}", "file",
                                     f"JSON parse failed: {fp.name}", None, "valid JSON"))
                continue
            if not isinstance(data, dict):
                issues.append(_issue("error", "type", f"InputMissionPlan/{fp.stem}", "message",
                                     f"InputMissionPlan must be object, got {type(data).__name__}",
                                     data, "InputMissionPlan object"))
                continue
            pkg_id = _coerce_int(_pick(data, "inputMissionPackageID", "InputMissionPackageID", default=fp.stem))
            if pkg_id is not None:
                existing_input_package_ids.add(pkg_id)
            existing_input_mission_ids.update(_collect_input_mission_ids_from_package(data))
            issues.extend(_validate_input_mission_package(fp.stem, data))

    ref_dir = base / "MissionReferenceInfo"
    if ref_dir.is_dir():
        for fp in sorted(ref_dir.glob("*.json")):
            try:
                data = json.loads(fp.read_text(encoding="utf-8"))
            except Exception:
                issues.append(_issue("error", "structure", f"MissionReferenceInfo/{fp.stem}", "file",
                                     f"JSON parse failed: {fp.name}", None, "valid JSON"))
                continue
            if not isinstance(data, dict):
                issues.append(_issue("error", "type", f"MissionReferenceInfo/{fp.stem}", "message",
                                     f"MissionReferenceInfo must be object, got {type(data).__name__}",
                                     data, "MissionReferenceInfo object"))
                continue
            ref_id = _coerce_int(_pick(data, "missionReferencePackageID", "MissionReferencePackageID", default=fp.stem))
            if ref_id is not None:
                existing_reference_package_ids.add(ref_id)
            issues.extend(_validate_mission_reference_package(fp.stem, data))

    # --- FlightPaths ---
    existing_path_ids: set[int] = set()
    path_meta: dict[int, dict[str, Any]] = {}
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

            pid = _coerce_int(_pick(data, "pathID", "PathID", default=fp.stem))
            if pid is not None:
                existing_path_ids.add(pid)
                path_meta[pid] = _extract_path_meta(data)

            issues.extend(_validate_flight_path(fp.stem, data, target_ids=target_ids))

    # --- IndividualMissionPlans ---
    existing_imp_ids: set[int] = set()
    existing_individual_mission_ids: set[int] = set()
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

            iid = _coerce_int(_pick(data, "individualMissionPackageID", "IndividualMissionPackageID", default=fp.stem))
            if iid is not None:
                existing_imp_ids.add(iid)
            existing_individual_mission_ids.update(_collect_individual_mission_ids_from_imp(data))

            issues.extend(_validate_imp(
                fp.stem,
                data,
                existing_path_ids,
                path_meta,
                target_ids,
                existing_input_mission_ids,
                prior_mission_ids,
            ))

    # --- MissionPlans ---
    existing_mission_plan_ids: set[int] = set()
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
            mid = _coerce_int(_pick(data, "missionPlanID", "MissionPlanID", default=fp.stem))
            if mid is not None:
                existing_mission_plan_ids.add(mid)
            issues.extend(_validate_mission_plan(
                data,
                existing_imp_ids,
                i,
                existing_input_package_ids,
                existing_reference_package_ids,
            ))

    materialized_mission_plan_ids: set[int] = set(existing_mission_plan_ids)

    transmitted_0301_plan_ids: set[int] = set()
    msg_0301_dir = base / "0301"
    if msg_0301_dir.is_dir():
        for fp in sorted(msg_0301_dir.glob("*.json")):
            try:
                data = json.loads(fp.read_text(encoding="utf-8"))
            except Exception:
                continue
            records = data if isinstance(data, list) else [data]
            for record in records:
                if not isinstance(record, dict):
                    continue
                plan_id = _coerce_int(_pick(record, "missionPlanID", "MissionPlanID", "missionPlanId", "MissionPlanId"))
                if plan_id is not None and plan_id > 0:
                    transmitted_0301_plan_ids.add(int(plan_id))

    pending_option_plan_ids: set[int] = set()
    for _, record in replan_request_records:
        pending_option_plan_ids.update(_collect_pending_option_plan_ids_from_replan(record))

    offered_option_plan_ids: set[int] = set()
    offered_option_records: list[dict[str, Any]] = []
    option_dir = base / "MissionPlanOptionInfo"
    if option_dir.is_dir():
        for fp in sorted(option_dir.glob("*.json")):
            try:
                data = json.loads(fp.read_text(encoding="utf-8"))
            except Exception:
                issues.append(_issue("warning", "structure", f"MissionPlanOptionInfo/{fp.stem}", "file",
                                     f"JSON parse failed: {fp.name}", None, "valid JSON"))
                continue
            records = data if isinstance(data, list) else [data]
            for idx, record in enumerate(records):
                if not isinstance(record, dict):
                    issues.append(_issue("error", "type", f"MissionPlanOptionInfo/{fp.stem}[{idx}]", "message",
                                         f"MissionPlanOptionInfo must be object, got {type(record).__name__}",
                                         record, "MissionPlanOptionInfo object",
                                         rule_id="NFUSION-0701",
                                         source="ICD 0701 MissionPlanOptionInfo"))
                    continue
                key = f"{fp.stem}[{idx}]" if isinstance(data, list) else fp.stem
                plan_ids = _collect_mission_plan_ids_from_obj(record)
                existing_mission_plan_ids.update(plan_ids)
                offered_option_plan_ids.update(plan_ids)
                offer_record = _collect_0701_offer_record(key, record)
                if offer_record.get("planIds"):
                    offered_option_records.append(offer_record)
                issues.extend(_validate_0701_mission_plan_option_info(
                    key,
                    record,
                    location_prefix="MissionPlanOptionInfo",
                    materialized_plan_ids=materialized_mission_plan_ids.union(transmitted_0301_plan_ids),
                    pending_option_plan_ids=pending_option_plan_ids,
                ))

    msg_0701_dir = base / "0701"
    if msg_0701_dir.is_dir():
        for fp in sorted(msg_0701_dir.glob("*.json")):
            try:
                data = json.loads(fp.read_text(encoding="utf-8"))
            except Exception:
                issues.append(_issue("error", "structure", f"0701/{fp.stem}", "file",
                                     f"JSON parse failed: {fp.name}", None, "valid JSON",
                                     rule_id="NFUSION-0701",
                                     source="ICD 0701 MissionPlanOptionInfo"))
                continue
            records = data if isinstance(data, list) else [data]
            for idx, record in enumerate(records):
                if not isinstance(record, dict):
                    issues.append(_issue("error", "type", f"0701/{fp.stem}[{idx}]", "message",
                                         f"0701 message must be object, got {type(record).__name__}",
                                         record, "MissionPlanOptionInfo object",
                                         rule_id="NFUSION-0701",
                                         source="ICD 0701 MissionPlanOptionInfo"))
                    continue
                key = f"{fp.stem}[{idx}]" if isinstance(data, list) else fp.stem
                plan_ids = _collect_mission_plan_ids_from_obj(record)
                existing_mission_plan_ids.update(plan_ids)
                offered_option_plan_ids.update(plan_ids)
                offer_record = _collect_0701_offer_record(key, record)
                if offer_record.get("planIds"):
                    offered_option_records.append(offer_record)
                issues.extend(_validate_0701_mission_plan_option_info(
                    key,
                    record,
                    materialized_plan_ids=materialized_mission_plan_ids.union(transmitted_0301_plan_ids),
                    pending_option_plan_ids=pending_option_plan_ids,
                ))

    seen_replan_signatures: set[str] = set()
    for key, record in replan_request_records:
        signature = _replan_request_signature(record)
        if signature in seen_replan_signatures:
            continue
        seen_replan_signatures.add(signature)
        pending_option_plan_ids.update(_collect_pending_option_plan_ids_from_replan(record))
        issues.extend(_validate_replan_request_0902(
            key,
            record,
            existing_input_mission_ids,
            existing_individual_mission_ids,
            existing_mission_plan_ids,
            target_ids,
        ))

    for msg_id, validator in (
        ("0903", lambda key, record: _validate_0903_request_renew_mission(
            key,
            record,
            existing_mission_plan_ids,
            pending_option_plan_ids,
            transmitted_0301_plan_ids,
        )),
        ("0702", lambda key, record: _validate_0702_pilot_decision(
            key,
            record,
            existing_mission_plan_ids,
            pending_option_plan_ids,
            offered_option_plan_ids,
            offered_option_records,
        )),
        ("0904", _validate_0904_request_behavior_tree),
    ):
        msg_dir = base / msg_id
        if not msg_dir.is_dir():
            continue
        for fp in sorted(msg_dir.glob("*.json")):
            try:
                data = json.loads(fp.read_text(encoding="utf-8"))
            except Exception:
                issues.append(_issue("error", "structure", f"{msg_id}/{fp.stem}", "file",
                                     f"JSON parse failed: {fp.name}", None, "valid JSON"))
                continue
            records = data if isinstance(data, list) else [data]
            for idx, record in enumerate(records):
                if not isinstance(record, dict):
                    issues.append(_issue("error", "type", f"{msg_id}/{fp.stem}[{idx}]", "message",
                                         f"{msg_id} message must be object, got {type(record).__name__}",
                                         record, "message object"))
                    continue
                key = f"{fp.stem}[{idx}]" if isinstance(data, list) else fp.stem
                issues.extend(validator(key, record))

    # --- nFusion message headers: 0201/0203/0301/0302/0303/0304 ---
    for msg_id, id_field, existing_ids in (
        ("0201", "inputMissionPackageID", existing_input_package_ids),
        ("0203", "missionReferencePackageID", existing_reference_package_ids),
        ("0301", "missionPlanID", materialized_mission_plan_ids),
        ("0302", "individualMissionPackageID", existing_imp_ids),
        ("0303", "pathID", existing_path_ids),
        ("0304", "pathID", existing_path_ids),
    ):
        msg_dir = base / msg_id
        if not msg_dir.is_dir():
            continue
        for fp in sorted(msg_dir.glob("*.json")):
            try:
                data = json.loads(fp.read_text(encoding="utf-8"))
            except Exception:
                issues.append(_issue("error", "structure", f"{msg_id}/{fp.stem}", "file",
                                     f"JSON parse failed: {fp.name}", None, "valid JSON"))
                continue
            records = data if isinstance(data, list) else [data]
            for idx, record in enumerate(records):
                if not isinstance(record, dict):
                    issues.append(_issue("error", "type", f"{msg_id}/{fp.stem}[{idx}]", "message",
                                         f"{msg_id} message must be object, got {type(record).__name__}",
                                         record, "message object"))
                    continue
                key = f"{fp.stem}[{idx}]" if isinstance(data, list) else fp.stem
                expected_aircraft_ids = None
                meta = None
                if msg_id == "0303":
                    expected_aircraft_ids = {4, 5, 6}
                    meta = path_meta
                elif msg_id == "0304":
                    expected_aircraft_ids = {1, 2, 3}
                    meta = path_meta
                issues.extend(_validate_030x_message_head(
                    msg_id,
                    key,
                    record,
                    id_field,
                    existing_ids,
                    meta,
                    expected_aircraft_ids,
                ))

    # --- nFusion message logs: 53112 UAVMissionPlan / 53111 LAHMissionPlan ---
    for msg_id, validator in (
        ("53112", _validate_53112_uav_mission_plan),
        ("53111", _validate_53111_lah_mission_plan),
    ):
        msg_dir = base / msg_id
        if not msg_dir.is_dir():
            continue
        for fp in sorted(msg_dir.glob("*.json")):
            try:
                data = json.loads(fp.read_text(encoding="utf-8"))
            except Exception:
                issues.append(_issue("error", "structure", f"{msg_id}/{fp.stem}", "file",
                                     f"JSON parse failed: {fp.name}", None, "valid JSON"))
                continue
            records = data if isinstance(data, list) else [data]
            for idx, record in enumerate(records):
                if not isinstance(record, dict):
                    issues.append(_issue("error", "type", f"{msg_id}/{fp.stem}[{idx}]", "message",
                                         f"{msg_id} message must be object, got {type(record).__name__}",
                                         record, "message object"))
                    continue
                key = f"{fp.stem}[{idx}]" if isinstance(data, list) else fp.stem
                issues.extend(validator(key, record, target_ids))

    return issues
