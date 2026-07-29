# -*- coding: utf-8 -*-
from __future__ import annotations

import copy
import json
import math
import re
import threading
import time
from pathlib import Path
from typing import Any, Iterable

from modules.common import db_paths, next_collab_replan_store
from modules.common.footprint_corners import normalize_footprint_corner_dicts
from modules.mission_planning.pipelines.ground_maneuver_mode import (
    TYPE2_SELF_RELIANCE_OUTBOUND_LINE,
    TYPE2_SELF_RELIANCE_RETURN_LINE,
    resolve_type2_self_reliance_phase,
)

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

_DB_CACHE_LOCK = threading.RLock()
_DB_JSON_CACHE_MAX = 256
_DB_JSON_CACHE: dict[tuple[str, int, int], dict[str, Any]] = {}
_DB_JSON_CACHE_ORDER: list[tuple[str, int, int]] = []
_DB_FOLDER_INDEX_CACHE: dict[
    tuple[str, str],
    tuple[tuple[tuple[str, int, int], ...], dict[int, Path]],
] = {}


def _coerce_int(value: object) -> int | None:
    try:
        return int(value)
    except Exception:
        return None


def _coerce_float(value: object) -> float | None:
    try:
        return float(value)
    except Exception:
        return None


def _first_present_value(
    candidates: Iterable[tuple[object, str]],
    *,
    default: Any = None,
) -> Any:
    """Return the first explicitly present key, preserving empty/None values."""

    for mapping, key in candidates:
        if isinstance(mapping, dict) and key in mapping:
            return mapping.get(key)
    return default


def _coerce_bool(value: object) -> bool | None:
    if isinstance(value, bool):
        return value
    if value is None:
        return None
    text = str(value).strip().lower()
    if text in {"1", "true", "yes", "y", "on"}:
        return True
    if text in {"0", "false", "no", "n", "off"}:
        return False
    return None


_POST_ATTACK_BOUNDARY_HOLD_KEYS = (
    "postAttackBoundaryHold",
    "post_attack_boundary_hold",
    "attackCompletionBoundaryHold",
    "attack_completion_boundary_hold",
)


def _has_post_attack_boundary_hold(*sources: object) -> bool:
    """Normalize the planner's legacy/current post-attack hold flags."""

    for source in sources:
        if not isinstance(source, dict):
            continue
        for key in _POST_ATTACK_BOUNDARY_HOLD_KEYS:
            if _coerce_bool(source.get(key)) is True:
                return True
    return False


def _mission_execution_blocked_until_next_collab(mission: object) -> bool:
    if not isinstance(mission, dict):
        return False
    for key in (
        "executionBlockedUntilNextCollab",
        "ExecutionBlockedUntilNextCollab",
    ):
        if key in mission:
            return _coerce_bool(mission.get(key)) is True
    return False


def _extract_line_width_m(mission_info: dict[str, Any]) -> float | None:
    widths: list[float] = []
    line_list = mission_info.get("lineList") or []
    if not isinstance(line_list, list):
        return None
    for item in line_list:
        if not isinstance(item, dict):
            continue
        width_m = _coerce_float(item.get("width") or item.get("Width"))
        if width_m is None or width_m <= 0.0:
            continue
        widths.append(float(width_m))
    if not widths:
        return None
    return max(widths)


def lookup_fov_db_max_width_m(fov_deg: object | None) -> float | None:
    fov_value = _coerce_float(fov_deg)
    if fov_value is None or fov_value <= 0.0:
        return None
    try:
        from modules.mission_planning.MissionPlanner.runtime_settings import load_fov_db_rows
    except Exception:
        return None

    rows = load_fov_db_rows()
    if not rows:
        return None

    fov_groups = sorted(
        {
            float(row.get("fov", 0.0) or 0.0)
            for row in rows
            if float(row.get("fov", 0.0) or 0.0) > 0.0
        }
    )
    if not fov_groups:
        return None

    matched_fov = min(
        fov_groups,
        key=lambda candidate: (abs(float(candidate) - float(fov_value)), -float(candidate)),
    )
    widths = [
        float(row.get("width", 0.0) or 0.0)
        for row in rows
        if abs(float(row.get("fov", 0.0) or 0.0) - float(matched_fov)) <= 1e-9
        and float(row.get("width", 0.0) or 0.0) > 0.0
    ]
    if not widths:
        return None
    return max(widths)


def compute_filming_quality_threshold_m(
    sep_m: object,
    width_m: object | None = None,
) -> float | None:
    sep_value = _coerce_float(sep_m)
    if sep_value is None or sep_value <= 0.0:
        return None
    width_value = _coerce_float(width_m)
    half_width_m = 0.0
    if width_value is not None and width_value > 0.0:
        half_width_m = float(width_value) * 0.5
    return math.hypot(float(sep_value), float(half_width_m))


def _derive_cumulative_etas(raw_etas: list[float]) -> tuple[list[float], bool]:
    """Return cumulative ETAs and whether the raw values looked cumulative."""
    if not raw_etas:
        return [], False
    values = [max(0.0, float(v)) for v in raw_etas]
    eps = 1e-6
    is_non_decreasing = all(values[idx] + eps >= values[idx - 1] for idx in range(1, len(values)))
    starts_at_zero = values[0] <= eps
    if starts_at_zero and is_non_decreasing:
        return values, True
    cumulative: list[float] = []
    total = 0.0
    for v in values:
        total += v
        cumulative.append(total)
    return cumulative, False


def _select_next_pending_id(items: Iterable[object], id_key: str) -> int | None:
    entries = [item for item in items if isinstance(item, dict)]
    for item in entries:
        if not item.get("isDone"):
            value = _coerce_int(item.get(id_key))
            if value is not None:
                return value
    for item in reversed(entries):
        value = _coerce_int(item.get(id_key))
        if value is not None:
            return value
    return None


def _waypoint_line_search_coordinates(waypoint: object) -> list[dict[str, Any]]:
    if not isinstance(waypoint, dict):
        return []
    filming = waypoint.get("filmingProperty") or waypoint.get("FilmingProperty") or {}
    if not isinstance(filming, dict):
        return []
    line_search = filming.get("lineSearch") or filming.get("LineSearch") or {}
    if not isinstance(line_search, dict):
        return []
    coords = line_search.get("coordinateList") or line_search.get("CoordinateList") or []
    return [dict(coord) for coord in coords if isinstance(coord, dict)]


def _coverage_coord_signature(coord: dict[str, Any]) -> tuple[int, int, int] | None:
    lat = _coerce_float(coord.get("latitude") if "latitude" in coord else coord.get("Latitude"))
    lon = _coerce_float(coord.get("longitude") if "longitude" in coord else coord.get("Longitude"))
    alt = _coerce_float(coord.get("altitude") if "altitude" in coord else coord.get("Altitude"))
    if lat is None or lon is None:
        return None
    return (
        int(round(float(lat) * 10_000_000.0)),
        int(round(float(lon) * 10_000_000.0)),
        int(round(float(alt or 0.0) * 10.0)),
    )


def _line_search_signature(waypoint: object) -> tuple[tuple[int, int, int], ...]:
    signature: list[tuple[int, int, int]] = []
    for coord in _waypoint_line_search_coordinates(waypoint):
        item = _coverage_coord_signature(coord)
        if item is None:
            return ()
        signature.append(item)
    return tuple(signature)


def _infer_reciprocal_area_waypoint_roles(
    waypoints: list[dict[str, Any]],
) -> dict[int, dict[str, str]]:
    """Infer a stripped reciprocal contract only from an exact 2-pass shape.

    Some 0303 schema round-trips omit planner extension keys.  We recover them
    only when there is a lineSearch prefix, exactly three non-filming turn
    waypoints, and an equal suffix whose sweeps are exact reversed copies.
    Ambiguous or ordinary single-pass paths remain untouched.
    """

    if not waypoints or any(
        isinstance(waypoint, dict)
        and (
            waypoint.get("areaCoveragePass")
            or waypoint.get("areaTurnRole")
            or waypoint.get("areaTurnPhase")
        )
        for waypoint in waypoints
    ):
        return {}
    candidates: list[dict[int, dict[str, str]]] = []
    count = len(waypoints)
    for turn_start in range(1, count - 3):
        turn_end = turn_start + 3
        turn_rows = waypoints[turn_start:turn_end]
        if any(_waypoint_line_search_coordinates(waypoint) for waypoint in turn_rows):
            continue
        if any(
            isinstance(waypoint, dict)
            and isinstance(
                waypoint.get("filmingProperty") or waypoint.get("FilmingProperty"),
                dict,
            )
            for waypoint in turn_rows
        ):
            continue
        prefix_start = turn_start
        while prefix_start > 0 and _waypoint_line_search_coordinates(waypoints[prefix_start - 1]):
            prefix_start -= 1
        suffix_end = turn_end
        while suffix_end < count and _waypoint_line_search_coordinates(waypoints[suffix_end]):
            suffix_end += 1
        prefix_indexes = list(range(prefix_start, turn_start))
        suffix_indexes = list(range(turn_end, suffix_end))
        if not prefix_indexes or len(prefix_indexes) != len(suffix_indexes):
            continue
        matched = True
        for suffix_index, prefix_index in zip(suffix_indexes, reversed(prefix_indexes)):
            prefix_signature = _line_search_signature(waypoints[prefix_index])
            suffix_signature = _line_search_signature(waypoints[suffix_index])
            if (
                len(prefix_signature) < 2
                or len(suffix_signature) != len(prefix_signature)
                or suffix_signature != tuple(reversed(prefix_signature))
            ):
                matched = False
                break
        if not matched:
            continue
        roles: dict[int, dict[str, str]] = {}
        for index in prefix_indexes:
            roles[int(index)] = {"area_coverage_pass": "forward"}
        for offset, phase in enumerate(("turn_entry", "turn_exit", "reentry")):
            roles[int(turn_start + offset)] = {
                "area_turn_role": "reciprocal_turn",
                "area_turn_phase": phase,
            }
        for index in suffix_indexes:
            roles[int(index)] = {"area_coverage_pass": "reverse"}
        candidates.append(roles)
    return candidates[0] if len(candidates) == 1 else {}


def _select_current_input_id(items: Iterable[object], id_key: str) -> int | None:
    entries = [item for item in items if isinstance(item, dict)]
    last_done_id: int | None = None
    for item in entries:
        value = _coerce_int(item.get(id_key))
        if value is None:
            continue
        if item.get("isDone"):
            last_done_id = int(value)
            continue
        if last_done_id is not None:
            return int(last_done_id)
        return int(value)
    if last_done_id is not None:
        return int(last_done_id)
    return None


def _transition_target_input_id(
    mission_plan_id: int | None,
    valid_input_ids: set[int],
    *,
    input_mission_package_id: int | None = None,
) -> int | None:
    if mission_plan_id is None:
        return None
    try:
        detail = next_collab_replan_store.load_detail(int(mission_plan_id))
    except Exception:
        detail = None
    if not isinstance(detail, dict):
        try:
            detail = next_collab_replan_store.load_latest_detail_at_or_before(
                int(mission_plan_id),
                input_mission_package_id=input_mission_package_id,
            )
        except Exception:
            detail = None
    if not isinstance(detail, dict):
        return None
    target_id = _coerce_int(detail.get("targetInputMissionID"))
    if target_id is None or int(target_id) not in valid_input_ids:
        return None
    return int(target_id)


def _mission_related_input_id(mission: dict[str, Any]) -> int | None:
    related = mission.get("relatedMission") or {}
    if not isinstance(related, dict):
        return None
    return _coerce_int(related.get("inputMissionID"))


def _select_current_mission_id(
    missions: Iterable[object],
    id_key: str,
    current_input_id: int | None,
    *,
    allow_blocked_current_input: bool = False,
) -> int | None:
    entries = [item for item in missions if isinstance(item, dict)]
    if current_input_id is not None:
        same_input = [
            item
            for item in entries
            if _mission_related_input_id(item) == int(current_input_id)
            and (
                allow_blocked_current_input
                or not _mission_execution_blocked_until_next_collab(item)
            )
        ]
        for item in same_input:
            if item.get("isDone"):
                continue
            value = _coerce_int(item.get(id_key))
            if value is not None:
                return int(value)
        for item in reversed(same_input):
            value = _coerce_int(item.get(id_key))
            if value is not None:
                return int(value)
        # The input mission is authoritative until an explicit next-collab
        # transition changes it.  Never fill a missing current branch with a
        # mission belonging to a later input: one completed Type-2 branch must
        # wait while its peers finish the same LINE.
        return None
    # Without an authoritative input, retained future missions are still
    # ineligible until their collaboration barrier is explicitly released.
    executable_entries = [
        item
        for item in entries
        if not _mission_execution_blocked_until_next_collab(item)
    ]
    if not executable_entries:
        return None
    return _select_next_pending_id(executable_entries, id_key)


def _payload_text(raw: bytes | str) -> str:
    if isinstance(raw, bytes):
        return raw.decode("utf-8", "ignore")
    return str(raw)


def parse_payload(payload: object | None) -> dict[str, Any]:
    if payload is None:
        return {}
    if isinstance(payload, dict):
        return dict(payload)
    if isinstance(payload, list) and payload:
        payload = payload[-1]
        if isinstance(payload, dict):
            return dict(payload)
    raw_bytes: bytes | None = None
    if isinstance(payload, bytes):
        raw_bytes = payload
    elif isinstance(payload, str):
        raw_bytes = payload.encode("utf-8", "ignore")
    else:
        try:
            raw_bytes = bytes(payload)  # type: ignore[arg-type]
        except Exception:
            raw_bytes = None
    if raw_bytes is None:
        return {}
    text = _payload_text(raw_bytes)
    match = re.search(r"\{.*\}", text, flags=re.S)
    json_text = match.group(0) if match else text.strip()
    if not json_text.startswith("{"):
        return {}
    try:
        return json.loads(json_text)
    except Exception:
        return {}


def extract_0903_info(payload: object | None) -> tuple[int | None, int | None, str | None, dict[str, Any]]:
    body = parse_payload(payload)
    if not body:
        return None, None, None, {}
    ts = None
    for key in ("timestamp", "Timestamp", "timeStamp", "TimeStamp"):
        if key in body:
            ts = _coerce_int(body.get(key))
            break
    mpid = None
    for key in ("missionPlanID", "MissionPlanID", "missionPlanId", "mission_plan_id"):
        if key in body:
            mpid = _coerce_int(body.get(key))
            break
    source = None
    for key in ("source", "Source", "sourceModuleName", "SourceModuleName"):
        if key in body:
            source = str(body.get(key))
            break
    return ts, mpid, source, body


def extract_0803_execute(payload: object | None) -> tuple[int | None, int | None, str | None, dict[str, Any]]:
    body = parse_payload(payload)
    if not body:
        return None, None, None, {}
    ts = None
    for key in ("timestamp", "Timestamp", "timeStamp", "TimeStamp"):
        if key in body:
            ts = _coerce_int(body.get(key))
            break
    execute = None
    for key in ("execute", "Execute"):
        if key in body:
            execute = _coerce_int(body.get(key))
            break
    source = None
    for key in ("source", "Source", "sourceModuleName", "SourceModuleName"):
        if key in body:
            source = str(body.get(key))
            break
    return ts, execute, source, body


def extract_0802_command(
    payload: object | None,
) -> tuple[int | None, int | None, int | None, str | None, dict[str, Any]]:
    """Extract timestamp/aircraftID/mandatoryType from 0802 (MandatoryCommand)."""
    body = parse_payload(payload)
    if not body:
        return None, None, None, None, {}

    ts = None
    for key in ("timestamp", "Timestamp", "timeStamp", "TimeStamp"):
        if key in body:
            ts = _coerce_int(body.get(key))
            break

    aircraft_id = None
    for key in ("aircraftID", "AircraftID", "aircraftId", "aircraft_id"):
        if key in body:
            aircraft_id = _coerce_int(body.get(key))
            break

    mandatory_type = None
    for key in ("mandatoryType", "MandatoryType", "mandatory_type"):
        if key in body:
            mandatory_type = _coerce_int(body.get(key))
            break

    source = None
    for key in ("source", "Source", "sourceModuleName", "SourceModuleName"):
        if key in body:
            source = str(body.get(key))
            break

    return ts, aircraft_id, mandatory_type, source, body


def extract_0702_decision(
    payload: object | None,
) -> tuple[int | None, int | None, int | None, str | None, dict[str, Any]]:
    """Extract timestamp/ignore/missionPlanID from 0702 (PilotDecision)."""
    body = parse_payload(payload)
    if not body:
        return None, None, None, None, {}

    ts = None
    for key in ("timestamp", "Timestamp", "timeStamp", "TimeStamp"):
        if key in body:
            ts = _coerce_int(body.get(key))
            break

    ignore_val = None
    for key in ("ignore", "Ignore"):
        if key in body:
            ignore_val = _coerce_int(body.get(key))
            break

    mpid = None
    for key in ("missionPlanID", "MissionPlanID", "missionPlanId", "mission_plan_id"):
        if key in body:
            mpid = _coerce_int(body.get(key))
            break

    source = None
    for key in ("source", "Source", "sourceModuleName", "SourceModuleName"):
        if key in body:
            source = str(body.get(key))
            break

    return ts, ignore_val, mpid, source, body


def extract_0401_agent_states(payload: object | None) -> tuple[int | None, list[dict[str, Any]]]:
    body = parse_payload(payload)
    if not body:
        return None, []
    ts = None
    for key in ("timestamp", "Timestamp", "timeStamp", "TimeStamp"):
        if key in body:
            ts = _coerce_int(body.get(key))
            break
    raw_list = body.get("agentStateList") or body.get("AgentStateList")
    if not isinstance(raw_list, list):
        raw_list = body.get("uavStates") or body.get("UavStates") or []
    states: list[dict[str, Any]] = []

    def _value_ci(container: object, *names: str) -> object | None:
        if not isinstance(container, dict):
            return None
        lowered = {str(key).lower(): value for key, value in container.items()}
        for name in names:
            if name in container:
                return container[name]
            lowered_name = name.lower()
            if lowered_name in lowered:
                return lowered[lowered_name]
        return None

    def _extract_current_waypoint(*containers: object) -> int | None:
        for container in containers:
            if not isinstance(container, dict):
                continue
            for key in ("currentWaypointID", "CurrentWaypointID", "currentWaypointId"):
                if key not in container:
                    continue
                value = container.get(key)
                if isinstance(value, dict):
                    for sub_key in ("waypointID", "WaypointID", "waypointId"):
                        if sub_key in value:
                            return _coerce_int(value.get(sub_key))
                return _coerce_int(value)
        return None

    def _extract_flying(*containers: object) -> int | None:
        for container in containers:
            if not isinstance(container, dict):
                continue
            for key in ("flying", "Flying"):
                if key in container:
                    return _coerce_int(container.get(key))
        return None

    def _extract_filming(sensor_info: dict[str, Any], *containers: object) -> int | None:
        for container in (sensor_info, *containers):
            if not isinstance(container, dict):
                continue
            nested = container.get("sensorInfo") or container.get("SensorInfo")
            if isinstance(nested, dict):
                value = _extract_filming(nested)
                if value is not None:
                    return value
            for key in ("filming", "Filming"):
                if key in container:
                    return _coerce_int(container.get(key))
        return None

    def _extract_flight_mode(*containers: object) -> int | None:
        for container in containers:
            if not isinstance(container, dict):
                continue
            for key in ("flightMode", "FlightMode"):
                if key in container:
                    value = container.get(key)
                    if isinstance(value, dict):
                        nested = _coerce_int(value.get("flightMode") or value.get("FlightMode"))
                        if nested is not None:
                            return nested
                    parsed = _coerce_int(value)
                    if parsed is not None:
                        return parsed
        return None

    def _extract_fuel(*containers: object) -> float | None:
        for container in containers:
            if not isinstance(container, dict):
                continue
            for key in ("fuel", "Fuel", "fuelLiters", "fuel_liters"):
                if key in container:
                    return _coerce_float(container.get(key))
        return None

    def _extract_fuel_warning(*containers: object) -> int | None:
        for container in containers:
            if not isinstance(container, dict):
                continue
            for key in ("fuelWarning", "FuelWarning", "fuel_warning"):
                if key in container:
                    return _coerce_int(container.get(key))
        return None

    def _extract_payload_health(*containers: object) -> int | None:
        for container in containers:
            if not isinstance(container, dict):
                continue
            for key in ("payloadHealth", "PayloadHealth", "payload_health"):
                if key in container:
                    return _coerce_int(container.get(key))
        return None

    def _extract_last_signal(item: dict[str, Any]) -> int | None:
        return _coerce_int(item.get("lastSignalTime") or item.get("LastSignalTime"))

    def _extract_leader_aircraft_id(*containers: object) -> int | None:
        for container in containers:
            if not isinstance(container, dict):
                continue
            value = (
                container.get("leaderAircraftID")
                or container.get("LeaderAircraftID")
                or container.get("leaderAircraftId")
                or container.get("leader_aircraft_id")
            )
            if isinstance(value, dict):
                nested = _coerce_int(
                    value.get("aircraftID")
                    or value.get("AircraftID")
                    or value.get("aircraftId")
                    or value.get("aircraft_id")
                )
                if nested is not None:
                    return nested
            parsed = _coerce_int(value)
            if parsed is not None:
                return parsed
        return None

    def _extract_boundary_guard_value(
        *containers: object,
        names: tuple[str, ...],
    ) -> object | None:
        for container in containers:
            value = _value_ci(container, *names)
            if value is not None:
                return value
        return None

    def _extract_coordinate(*containers: object) -> dict[str, float] | None:
        for container in containers:
            if not isinstance(container, dict):
                continue
            coordinate = container.get("coordinate") or container.get("Coordinate") or {}
            if not isinstance(coordinate, dict):
                continue
            lat = _coerce_float(coordinate.get("latitude") or coordinate.get("Latitude"))
            lon = _coerce_float(coordinate.get("longitude") or coordinate.get("Longitude"))
            alt = _coerce_float(coordinate.get("altitude") or coordinate.get("Altitude"))
            if lat is None or lon is None:
                continue
            out: dict[str, float] = {
                "latitude": float(lat),
                "longitude": float(lon),
            }
            if alt is not None:
                out["altitude"] = float(alt)
            return out
        return None

    def _extract_velocity(*containers: object) -> dict[str, float] | None:
        for container in containers:
            if not isinstance(container, dict):
                continue
            velocity = _value_ci(container, "velocity")
            if not isinstance(velocity, dict):
                velocity = container
            speed = _coerce_float(_value_ci(velocity, "speed"))
            heading = _coerce_float(_value_ci(velocity, "heading"))
            if speed is None and heading is None:
                continue
            result: dict[str, float] = {}
            if speed is not None:
                result["speed"] = float(speed)
            if heading is not None:
                result["heading"] = float(heading) % 360.0
            return result
        return None

    def _extract_attitude(*containers: object) -> dict[str, float] | None:
        for container in containers:
            if not isinstance(container, dict):
                continue
            attitude = _value_ci(container, "attitude")
            if not isinstance(attitude, dict):
                continue
            roll = _coerce_float(_value_ci(attitude, "roll"))
            pitch = _coerce_float(_value_ci(attitude, "pitch"))
            yaw = _coerce_float(_value_ci(attitude, "yaw"))
            if roll is None and pitch is None and yaw is None:
                continue
            result: dict[str, float] = {}
            if roll is not None:
                result["roll"] = max(-180.0, min(180.0, float(roll)))
            if pitch is not None:
                result["pitch"] = max(-90.0, min(90.0, float(pitch)))
            if yaw is not None:
                result["yaw"] = float(yaw) % 360.0
            return result
        return None

    def _extract_sensor_info(*containers: object) -> dict[str, Any]:
        for container in containers:
            if not isinstance(container, dict):
                continue
            sensor_info = container.get("sensorInfo") or container.get("SensorInfo")
            if isinstance(sensor_info, dict):
                return dict(sensor_info)
        return {}

    def _extract_sensor_center_coordinate(*containers: object) -> dict[str, float] | None:
        sensor_info = _extract_sensor_info(*containers)
        if not sensor_info:
            return None
        center = sensor_info.get("centerCoordinate") or sensor_info.get("CenterCoordinate")
        if not isinstance(center, dict):
            return None
        lat = _coerce_float(center.get("latitude") or center.get("Latitude"))
        lon = _coerce_float(center.get("longitude") or center.get("Longitude"))
        alt = _coerce_float(center.get("altitude") or center.get("Altitude"))
        if lat is None or lon is None:
            return None
        out: dict[str, float] = {
            "latitude": float(lat),
            "longitude": float(lon),
        }
        if alt is not None:
            out["altitude"] = float(alt)
        return out

    def _extract_sensor_fov_deg(*containers: object) -> float | None:
        sensor_info = _extract_sensor_info(*containers)
        if not sensor_info:
            return None
        return _coerce_float(
            sensor_info.get("fov")
            or sensor_info.get("Fov")
            or sensor_info.get("fieldOfView")
            or sensor_info.get("FieldOfView")
        )

    def _extract_manned_datalink(item: dict[str, Any]) -> dict[int, bool | None]:
        manned_info = item.get("mannedInfo") or item.get("MannedInfo") or {}
        if not isinstance(manned_info, dict):
            return {}
        datalink = manned_info.get("datalinkStatus") or manned_info.get("DatalinkStatus") or {}
        if not isinstance(datalink, dict):
            return {}
        return {
            4: _coerce_bool(datalink.get("isConnectedToUAV1", datalink.get("uav1"))),
            5: _coerce_bool(datalink.get("isConnectedToUAV2", datalink.get("uav2"))),
            6: _coerce_bool(datalink.get("isConnectedToUAV3", datalink.get("uav3"))),
        }

    def _extract_footprint(item: dict[str, Any], info: object, camera_mode: object) -> list[dict[str, Any]]:
        corners: list[dict[str, Any]] = []
        for container in (info, camera_mode, item):
            if not isinstance(container, dict):
                continue
            sensor_info = container.get("sensorInfo") or container.get("SensorInfo") or container
            if isinstance(sensor_info, dict):
                raw = sensor_info.get("footprintCornerList") or sensor_info.get("FootprintCornerList")
                if isinstance(raw, list):
                    for corner in raw:
                        if isinstance(corner, dict):
                            corners.append(dict(corner))
                    if corners:
                        return normalize_footprint_corner_dicts(corners)
            if isinstance(container, dict):
                legacy_keys = (
                    "cornerUpperLeft",
                    "cornerUpperRight",
                    "cornerLowerRight",
                    "cornerLowerLeft",
                )
                legacy: list[dict[str, Any]] = []
                for key in legacy_keys:
                    value = container.get(key)
                    if isinstance(value, dict):
                        legacy.append(dict(value))
                if legacy:
                    return normalize_footprint_corner_dicts(legacy)
        return normalize_footprint_corner_dicts(corners)

    datalink_votes: dict[int, list[bool]] = {4: [], 5: [], 6: []}
    datalink_connected_by_pair: dict[int, dict[int, bool]] = {4: {}, 5: {}, 6: {}}
    for item in raw_list:
        if not isinstance(item, dict):
            continue
        manned_aircraft_id = _coerce_int(item.get("aircraftID") or item.get("AircraftID"))
        if manned_aircraft_id is None or not (1 <= int(manned_aircraft_id) <= 3):
            continue
        for aircraft_id, connected in _extract_manned_datalink(item).items():
            if connected is None:
                continue
            datalink_votes.setdefault(int(aircraft_id), []).append(bool(connected))
            datalink_connected_by_pair.setdefault(int(aircraft_id), {})[int(manned_aircraft_id)] = bool(connected)

    datalink_connected_by_uav: dict[int, bool | None] = {}
    for aircraft_id, votes in datalink_votes.items():
        if not votes:
            datalink_connected_by_uav[int(aircraft_id)] = None
        else:
            datalink_connected_by_uav[int(aircraft_id)] = any(bool(v) for v in votes)

    for item in raw_list:
        if not isinstance(item, dict):
            continue
        unmanned_info = item.get("unmannedInfo") or item.get("UnmannedInfo") or {}
        flight_mode_info = item.get("flightMode") or item.get("FlightMode") or {}
        camera_mode = item.get("cameraMode") or item.get("CameraMode") or {}
        sensor_info = _extract_sensor_info(unmanned_info, camera_mode, item)
        fuel_val = _extract_fuel(item, unmanned_info)
        fuel_warning = _extract_fuel_warning(unmanned_info, flight_mode_info, item)
        aircraft_id = _coerce_int(item.get("aircraftID") or item.get("AircraftID"))
        flying = _extract_flying(unmanned_info, flight_mode_info, item)
        filming = _extract_filming(sensor_info, unmanned_info, camera_mode, item)
        states.append(
            {
                "aircraft_id": aircraft_id,
                "health": _coerce_int(item.get("health") or item.get("Health")),
                "last_signal_time": _extract_last_signal(item),
                "leader_aircraft_id": _extract_leader_aircraft_id(unmanned_info, flight_mode_info, item),
                "is_unmanned": item.get("isUnmanned")
                if "isUnmanned" in item or "IsUnmanned" in item
                else (aircraft_id or 0) >= 4,
                "current_waypoint_id": _extract_current_waypoint(unmanned_info, flight_mode_info, item),
                "flying": flying,
                "filming": filming,
                "flight_mode": _extract_flight_mode(unmanned_info, flight_mode_info, item),
                "boundary_guard_set_id": _extract_boundary_guard_value(
                    unmanned_info,
                    flight_mode_info,
                    item,
                    names=(
                        "boundaryGuardSetID",
                        "boundary_guard_set_id",
                    ),
                ),
                "boundary_guard_cycle_count": _coerce_int(
                    _extract_boundary_guard_value(
                        unmanned_info,
                        flight_mode_info,
                        item,
                        names=(
                            "boundaryGuardCycleCount",
                            "boundary_guard_cycle_count",
                        ),
                    )
                ),
                "boundary_guard_loop_active": _coerce_bool(
                    _extract_boundary_guard_value(
                        unmanned_info,
                        flight_mode_info,
                        item,
                        names=(
                            "boundaryGuardLoopActive",
                            "boundary_guard_loop_active",
                        ),
                    )
                ),
                "payload_health": _extract_payload_health(unmanned_info, flight_mode_info, item),
                "coordinate": _extract_coordinate(item, unmanned_info),
                "velocity": _extract_velocity(item, unmanned_info),
                "attitude": _extract_attitude(item, unmanned_info),
                "sensor_center_coordinate": _extract_sensor_center_coordinate(unmanned_info, camera_mode, item),
                "sensor_fov_deg": _extract_sensor_fov_deg(unmanned_info, camera_mode, item),
                "sensor_type": _coerce_int(sensor_info.get("sensorType") or sensor_info.get("SensorType")),
                "sensor_operation_mode": _coerce_int(
                    sensor_info.get("operationalMode")
                    or sensor_info.get("OperationalMode")
                    or sensor_info.get("operationMode")
                    or sensor_info.get("OperationMode")
                ),
                "fuel_liters": fuel_val,
                "fuel_warning": fuel_warning,
                "datalink_connected": datalink_connected_by_uav.get(int(aircraft_id), None)
                if aircraft_id is not None and aircraft_id >= 4
                else None,
                "datalink_connected_by_manned": (
                    dict(datalink_connected_by_pair.get(int(aircraft_id), {}))
                    if aircraft_id is not None and aircraft_id >= 4
                    else {}
                ),
                "footprint_corners": _extract_footprint(item, unmanned_info, camera_mode),
            }
        )
    if ts is None:
        # Some simulators omit top-level 0401 timestamp; fall back to the
        # latest per-agent signal time to keep progress in simulation time.
        inferred_ts: int | None = None
        for state in states:
            value = _coerce_int(state.get("last_signal_time"))
            if value is None or value <= 0:
                continue
            if inferred_ts is None or value > inferred_ts:
                inferred_ts = int(value)
        ts = inferred_ts
    return ts, states


def format_timestamp_ms(timestamp_ms: int | None) -> str:
    if timestamp_ms is None:
        return "-"
    try:
        iso = db_paths.ms_to_iso(int(timestamp_ms))
    except Exception:
        return str(timestamp_ms)
    if "T" in iso:
        date_part, time_part = iso.split("T", 1)
        if len(time_part) >= 6:
            time_part = f"{time_part[0:2]}:{time_part[2:4]}:{time_part[4:6]}"
        return f"{date_part} {time_part}"
    return iso


def mission_plan_json_path(mission_plan_id: int | None, db_root: Path | str | None = None) -> Path | None:
    if mission_plan_id is None:
        return None
    if db_root is None:
        base = db_paths.get_db_subpath("MissionPlan")
    else:
        base = Path(db_root) / "MissionPlan"
    return base / f"{int(mission_plan_id)}.json"


def _folder_id_keys(folder: str) -> tuple[str, ...]:
    mapping: dict[str, tuple[str, ...]] = {
        "MissionPlan": ("missionPlanID", "MissionPlanID", "missionPlanId"),
        "InputMissionPlan": (
            "inputMissionPackageID",
            "InputMissionPackageID",
            "inputMissionPackageId",
        ),
        "IndividualMissionPlan": (
            "individualMissionPackageID",
            "IndividualMissionPackageID",
            "individualMissionPlanPackageID",
            "IndividualMissionPlanPackageID",
        ),
        "FlightPath": ("pathID", "PathID", "pathId"),
    }
    return mapping.get(str(folder), ())


def _folder_signature(base: Path) -> tuple[tuple[str, int, int], ...]:
    rows: list[tuple[str, int, int]] = []
    try:
        paths = sorted(base.glob("*.json"))
    except Exception:
        return ()
    for path in paths:
        try:
            stat = path.stat()
        except Exception:
            continue
        rows.append((path.name, int(stat.st_mtime_ns), int(stat.st_size)))
    return tuple(rows)


def _folder_index_cache_key(folder: str, base: Path) -> tuple[str, str]:
    try:
        base_key = str(base.resolve())
    except Exception:
        base_key = str(base)
    return str(folder), base_key


def _get_folder_id_index(folder: str, base: Path, id_keys: tuple[str, ...]) -> dict[int, Path]:
    perf_start = replan_perf.start_timer()
    signature = _folder_signature(base)
    cache_key = _folder_index_cache_key(folder, base)
    with _DB_CACHE_LOCK:
        cached = _DB_FOLDER_INDEX_CACHE.get(cache_key)
        if cached is not None and cached[0] == signature:
            replan_perf.add_elapsed(
                "monitoring.db.folder_id_index",
                perf_start,
                cache_hit=1,
                files=len(signature),
            )
            return dict(cached[1])

    mapping: dict[int, Path] = {}
    scanned = 0
    loaded = 0
    read_chars = 0
    try:
        paths = sorted(base.glob("*.json"))
    except Exception:
        paths = []
    for path in paths:
        scanned += 1
        try:
            text = path.read_text(encoding="utf-8")
            read_chars += len(text)
            payload = json.loads(text)
            loaded += 1
        except Exception:
            continue
        if not isinstance(payload, dict):
            continue
        for key in id_keys:
            value = _coerce_int(payload.get(key))
            if value is None:
                continue
            mapping.setdefault(int(value), path)

    with _DB_CACHE_LOCK:
        _DB_FOLDER_INDEX_CACHE[cache_key] = (signature, dict(mapping))
    replan_perf.add_elapsed(
        "monitoring.db.folder_id_index",
        perf_start,
        cache_miss=1,
        files=len(signature),
        scanned=scanned,
        loaded=loaded,
        read_chars=read_chars,
    )
    return mapping


def _db_json_cache_key(path: Path) -> tuple[str, int, int] | None:
    try:
        resolved = str(path.resolve())
        stat = path.stat()
    except Exception:
        return None
    return resolved, int(stat.st_mtime_ns), int(stat.st_size)


def _copy_cached_payload(payload: dict[str, Any]) -> dict[str, Any]:
    copy_start = time.perf_counter() if replan_perf.is_enabled() else None
    result = copy.deepcopy(payload)
    copy_ms = 0.0
    if copy_start is not None:
        copy_ms = (time.perf_counter() - copy_start) * 1000.0
    replan_perf.add("monitoring.db.load_json.deepcopy", elapsed_ms=copy_ms)
    return result


def _load_db_json_from_path(path: Path) -> tuple[dict[str, Any], bool, int]:
    cache_key = _db_json_cache_key(path)
    if cache_key is not None:
        with _DB_CACHE_LOCK:
            cached = _DB_JSON_CACHE.get(cache_key)
            if cached is not None:
                return _copy_cached_payload(cached), True, 0

    text = path.read_text(encoding="utf-8")
    payload = json.loads(text)
    if not isinstance(payload, dict):
        return {}, False, len(text)
    if cache_key is not None:
        with _DB_CACHE_LOCK:
            if cache_key not in _DB_JSON_CACHE:
                _DB_JSON_CACHE[cache_key] = payload
                _DB_JSON_CACHE_ORDER.append(cache_key)
                while len(_DB_JSON_CACHE_ORDER) > _DB_JSON_CACHE_MAX:
                    old_key = _DB_JSON_CACHE_ORDER.pop(0)
                    _DB_JSON_CACHE.pop(old_key, None)
    return _copy_cached_payload(payload), False, len(text)


def _invalidate_db_cache_path(path: Path) -> None:
    try:
        resolved = str(path.resolve())
    except Exception:
        resolved = str(path)
    with _DB_CACHE_LOCK:
        stale = [key for key in _DB_JSON_CACHE.keys() if key[0] == resolved]
        for key in stale:
            _DB_JSON_CACHE.pop(key, None)
            try:
                _DB_JSON_CACHE_ORDER.remove(key)
            except ValueError:
                pass


def _invalidate_folder_index(folder: str, base: Path) -> None:
    with _DB_CACHE_LOCK:
        _DB_FOLDER_INDEX_CACHE.pop(_folder_index_cache_key(folder, base), None)


def _resolve_db_json_path(folder: str, file_id: int | None, db_root: Path | str | None = None) -> Path | None:
    perf_start = replan_perf.start_timer()
    scanned = 0
    loaded = 0
    read_chars = 0
    if file_id is None:
        replan_perf.add_elapsed("monitoring.db.resolve", perf_start, file_id_none=1)
        return None
    if db_root is None:
        base = db_paths.get_db_subpath(folder)
    else:
        base = Path(db_root) / folder
    direct = base / f"{int(file_id)}.json"
    if direct.exists():
        replan_perf.add_elapsed(
            "monitoring.db.resolve",
            perf_start,
            direct_hit=1,
            scanned=scanned,
            loaded=loaded,
            read_chars=read_chars,
        )
        return direct

    id_keys = _folder_id_keys(folder)
    if not id_keys or not base.exists():
        replan_perf.add_elapsed(
            "monitoring.db.resolve",
            perf_start,
            fallback_unavailable=1,
            scanned=scanned,
            loaded=loaded,
            read_chars=read_chars,
        )
        return direct

    index = _get_folder_id_index(folder, base, id_keys)
    scanned = len(index)
    path = index.get(int(file_id))
    if path is not None:
        replan_perf.add_elapsed(
            "monitoring.db.resolve",
            perf_start,
            fallback_hit=1,
            scanned=scanned,
            loaded=loaded,
            read_chars=read_chars,
        )
        return path
    replan_perf.add_elapsed(
        "monitoring.db.resolve",
        perf_start,
        fallback_miss=1,
        scanned=scanned,
        loaded=loaded,
        read_chars=read_chars,
    )
    return direct


def load_db_json(folder: str, file_id: int | None, db_root: Path | str | None = None) -> dict[str, Any]:
    perf_start = replan_perf.start_timer()
    if file_id is None:
        replan_perf.add_elapsed("monitoring.db.load_json", perf_start, file_id_none=1)
        return {}
    path = _resolve_db_json_path(folder, file_id, db_root=db_root)
    if path is None:
        replan_perf.add_elapsed("monitoring.db.load_json", perf_start, path_none=1)
        return {}
    if not path.exists():
        replan_perf.add_elapsed("monitoring.db.load_json", perf_start, missing=1)
        return {}
    try:
        payload, cache_hit, read_chars = _load_db_json_from_path(path)
        replan_perf.add_elapsed(
            "monitoring.db.load_json",
            perf_start,
            loaded=1,
            cache_hit=1 if cache_hit else 0,
            cache_miss=0 if cache_hit else 1,
            read_chars=read_chars,
        )
        return payload
    except Exception:
        replan_perf.add_elapsed("monitoring.db.load_json", perf_start, error=1)
        return {}


def save_db_json(folder: str, file_id: int | None, payload: dict[str, Any], db_root: Path | str | None = None) -> bool:
    perf_start = replan_perf.start_timer()
    if file_id is None:
        replan_perf.add_elapsed("monitoring.db.save_json", perf_start, file_id_none=1)
        return False
    if db_root is None:
        base = db_paths.get_db_subpath(folder)
    else:
        base = Path(db_root) / folder
    path = _resolve_db_json_path(folder, file_id, db_root=db_root)
    if path is None:
        path = base / f"{int(file_id)}.json"
    try:
        base.mkdir(parents=True, exist_ok=True)
        text = json.dumps(payload, ensure_ascii=False, indent=2)
        path.write_text(text, encoding="utf-8")
        _invalidate_db_cache_path(path)
        _invalidate_folder_index(folder, base)
        replan_perf.add_elapsed(
            "monitoring.db.save_json",
            perf_start,
            written=1,
            write_chars=len(text),
        )
        return True
    except Exception:
        replan_perf.add_elapsed("monitoring.db.save_json", perf_start, error=1)
        return False


def build_uav_mission_view(
    mission_plan_id: int | None,
    *,
    uav_ids: Iterable[int] = (4, 5, 6),
    db_root: Path | str | None = None,
) -> dict[str, Any]:
    plan = load_db_json("MissionPlan", mission_plan_id, db_root=db_root)
    aircraft_list = plan.get("aircraftList") or []
    input_package_id = _coerce_int(plan.get("inputMissionPackageID"))
    input_plan = load_db_json("InputMissionPlan", input_package_id, db_root=db_root)
    input_missions = input_plan.get("inputMissionList") or []
    input_id_values = {
        int(value)
        for value in (
            _coerce_int(item.get("inputMissionID"))
            for item in input_missions
            if isinstance(item, dict)
        )
        if value is not None
    }
    transition_input_id = _transition_target_input_id(
        mission_plan_id,
        input_id_values,
        input_mission_package_id=input_package_id,
    )
    current_input_mission_id = (
        int(transition_input_id)
        if transition_input_id is not None
        else _select_current_input_id(input_missions, "inputMissionID")
    )
    input_type_map: dict[int, int] = {}
    input_region_type_map: dict[int, int] = {}
    input_detail_map: dict[int, dict[str, Any]] = {}
    input_items: list[dict[str, Any]] = []
    for item in input_missions:
        if not isinstance(item, dict):
            continue
        mission_id = _coerce_int(item.get("inputMissionID"))
        if mission_id is None:
            continue
        mission_type = _coerce_int(item.get("inputMissionType"))
        if mission_type is not None:
            input_type_map[int(mission_id)] = int(mission_type)
        region_type = _coerce_int(item.get("regionType"))
        if region_type is not None:
            input_region_type_map[int(mission_id)] = int(region_type)
        mission_detail = item.get("missionDetail") or {}
        if not isinstance(mission_detail, dict):
            mission_detail = {}
        coverage_pass_order = _first_present_value(
            ((mission_detail, "coveragePassOrder"), (item, "coveragePassOrder")),
            default=[],
        )
        remaining_coverage_passes = _first_present_value(
            (
                (mission_detail, "remainingCoveragePasses"),
                (item, "remainingCoveragePasses"),
            ),
            default=[],
        )
        completed_coverage_passes = _first_present_value(
            (
                (mission_detail, "completedCoveragePasses"),
                (item, "completedCoveragePasses"),
            ),
            default=[],
        )
        coverage_pass_details = _first_present_value(
            ((mission_detail, "coveragePassDetails"), (item, "coveragePassDetails")),
            default=[],
        )
        coverage_pass_obligations = _first_present_value(
            (
                (mission_detail, "coveragePassObligations"),
                (item, "coveragePassObligations"),
            ),
            default=[],
        )
        coverage_depth_details = _first_present_value(
            ((mission_detail, "coverageDepthDetails"), (item, "coverageDepthDetails")),
            default=[],
        )
        coverage_depth_obligations = _first_present_value(
            (
                (mission_detail, "coverageDepthObligations"),
                (item, "coverageDepthObligations"),
            ),
            default=[],
        )
        coverage_observation_details = _first_present_value(
            (
                (mission_detail, "coverageObservationDetails"),
                (item, "coverageObservationDetails"),
            ),
            default=[],
        )
        input_detail_map[int(mission_id)] = {
            "coordinate_list": list(mission_detail.get("coordinateList") or [])
            if isinstance(mission_detail.get("coordinateList"), list)
            else [],
            "line_list": list(mission_detail.get("lineList") or [])
            if isinstance(mission_detail.get("lineList"), list)
            else [],
            "area_list": list(mission_detail.get("areaList") or [])
            if isinstance(mission_detail.get("areaList"), list)
            else [],
            "source_line_width_m": _coerce_float(mission_detail.get("sourceLineWidthM")),
            "area_coverage_pass_contract_version": _coerce_int(
                _first_present_value(
                    (
                        (mission_detail, "areaCoveragePassContractVersion"),
                        (item, "areaCoveragePassContractVersion"),
                    )
                )
            ),
            "coverage_pass_policy": _first_present_value(
                ((mission_detail, "coveragePassPolicy"), (item, "coveragePassPolicy"))
            ),
            "coverage_pass_order": list(coverage_pass_order or []),
            "remaining_coverage_passes": list(remaining_coverage_passes or []),
            "completed_coverage_passes": list(completed_coverage_passes or []),
            "current_coverage_pass": _first_present_value(
                ((mission_detail, "currentCoveragePass"), (item, "currentCoveragePass"))
            ),
            "active_coverage_pass": _first_present_value(
                ((mission_detail, "activeCoveragePass"), (item, "activeCoveragePass"))
            ),
            "area_coverage_phase": _first_present_value(
                ((mission_detail, "areaCoveragePhase"), (item, "areaCoveragePhase"))
            ),
            "coverage_pass_details": copy.deepcopy(coverage_pass_details or []),
            "coverage_pass_obligations": copy.deepcopy(coverage_pass_obligations or []),
            "area_coverage_depth_contract_version": _coerce_int(
                _first_present_value(
                    (
                        (mission_detail, "areaCoverageDepthContractVersion"),
                        (item, "areaCoverageDepthContractVersion"),
                    )
                )
            ),
            "coverage_depth_policy": _first_present_value(
                ((mission_detail, "coverageDepthPolicy"), (item, "coverageDepthPolicy"))
            ),
            "required_coverage_depth": _coerce_int(
                _first_present_value(
                    ((mission_detail, "requiredCoverageDepth"), (item, "requiredCoverageDepth"))
                )
            ),
            "coverage_depth_details": copy.deepcopy(coverage_depth_details or []),
            "coverage_depth_obligations": copy.deepcopy(coverage_depth_obligations or []),
            "coverage_observation_details": copy.deepcopy(coverage_observation_details or []),
        }
        input_items.append(
            {
                "input_mission_id": mission_id,
                "input_mission_type": int(mission_type) if mission_type is not None else None,
                "region_type": int(region_type) if region_type is not None else None,
                "is_done": bool(item.get("isDone")),
            }
        )

    type2_self_reliance_phase_map: dict[int, str] = {}
    for input_id in input_id_values:
        # Only the two LINE members can use line-scan progress.  Resolve them
        # against the complete input package so an ordinary collaborative LINE
        # with the same regionType never inherits Type-2 branch semantics.
        if input_type_map.get(int(input_id)) not in {1, 7}:
            continue
        phase = resolve_type2_self_reliance_phase(input_plan, int(input_id))
        if phase in {
            TYPE2_SELF_RELIANCE_OUTBOUND_LINE,
            TYPE2_SELF_RELIANCE_RETURN_LINE,
        }:
            type2_self_reliance_phase_map[int(input_id)] = str(phase)

    uav_entries: list[dict[str, Any]] = []
    for uav_id in uav_ids:
        package_id = None
        for entry in aircraft_list:
            if _coerce_int(entry.get("aircraftID")) == int(uav_id):
                package_id = _coerce_int(entry.get("individualMissionPackageID"))
                break

        individual_plan = load_db_json("IndividualMissionPlan", package_id, db_root=db_root)
        mission_list = individual_plan.get("individualMissionList") or []
        boundary_mission_id: int | None = None
        boundary_input_id: int | None = None
        for mission in mission_list:
            if not isinstance(mission, dict):
                continue
            if _mission_execution_blocked_until_next_collab(mission):
                continue
            if not _has_post_attack_boundary_hold(mission):
                continue
            boundary_mission_id = _coerce_int(mission.get("individualMissionID"))
            if boundary_mission_id is not None:
                boundary_input_id = _mission_related_input_id(mission)
                break
        if (
            boundary_mission_id is not None
            and current_input_mission_id is not None
            and boundary_input_id is not None
            and int(boundary_input_id) != int(current_input_mission_id)
        ):
            boundary_mission_id = None
        current_mission_id: int | None = boundary_mission_id or _select_current_mission_id(
            mission_list,
            "individualMissionID",
            current_input_mission_id,
            allow_blocked_current_input=transition_input_id is not None,
        )
        missions: list[dict[str, Any]] = []

        for mission in mission_list:
            if not isinstance(mission, dict):
                continue
            mission_id = _coerce_int(mission.get("individualMissionID"))
            related = mission.get("relatedMission") or {}
            input_id = _coerce_int(related.get("inputMissionID"))
            explicit_transition_target = bool(
                transition_input_id is not None
                and input_id is not None
                and int(input_id) == int(transition_input_id)
            )
            outside_current_input_before_handoff = bool(
                current_input_mission_id is not None
                and input_id is not None
                and int(input_id) != int(current_input_mission_id)
            )
            execution_blocked_until_next_collab = bool(
                (
                    _mission_execution_blocked_until_next_collab(mission)
                    and not explicit_transition_target
                )
                or outside_current_input_before_handoff
            )
            input_type = input_type_map.get(int(input_id)) if input_id is not None else None
            region_type = (
                input_region_type_map.get(int(input_id))
                if input_id is not None
                else None
            )
            input_detail = input_detail_map.get(int(input_id)) if input_id is not None else None
            type2_self_reliance_phase = (
                type2_self_reliance_phase_map.get(int(input_id))
                if input_id is not None
                else None
            )
            path_id = _coerce_int(mission.get("pathID"))
            mission_info = mission.get("individualMissionInfo") or {}
            if not isinstance(mission_info, dict):
                mission_info = {}
            line_list = mission_info.get("lineList") or []
            area_list = mission_info.get("areaList") or []
            coordinate_list = mission_info.get("coordinateList") or []
            source_coordinate_list = mission_info.get("sourceCoordinateList") or []
            mission_type = _coerce_int(mission_info.get("individualMissionType"))
            pattern_type = _coerce_int(mission_info.get("patternType"))
            sep_m = _coerce_float(
                mission_info.get("SEP")
                or mission_info.get("sep")
                or mission_info.get("sepM")
            )
            width_m = _extract_line_width_m(mission_info)
            source_line_width_m = _coerce_float((input_detail or {}).get("source_line_width_m"))
            if source_line_width_m is None or source_line_width_m <= 0.0:
                source_line_width_m = _coerce_float(mission_info.get("sourceLineWidthM"))
            quality_threshold_m = None
            waypoint_ids: list[int] = []
            is_done = bool(mission.get("isDone"))
            raw_etas: list[float] = []
            waypoint_defs: list[dict[str, Any]] = []
            waypoint_meta_defs: list[dict[str, Any]] = []
            sweep_line_coordinate_lists: list[list[dict[str, Any]]] = []
            sweep_point_count = 0
            flight_path_timestamp_ms: int | None = None
            is_formation_flight = False
            formation_leader_id: int | None = None
            post_attack_boundary_hold = _has_post_attack_boundary_hold(mission)
            boundary_guard_sources: list[dict[str, Any]] = [
                source
                for source in (mission_info, mission)
                if isinstance(source, dict)
            ]
            if path_id is not None:
                path_data = load_db_json("FlightPath", path_id, db_root=db_root)
                if isinstance(path_data, dict):
                    boundary_guard_sources.insert(0, path_data)
                flight_path_timestamp_ms = _coerce_int(
                    _first_present_value(
                        (
                            (path_data, "timestamp"),
                            (path_data, "Timestamp"),
                            (path_data, "timeStamp"),
                            (path_data, "TimeStamp"),
                        )
                    )
                )
                post_attack_boundary_hold = bool(
                    post_attack_boundary_hold
                    or _has_post_attack_boundary_hold(path_data)
                )
                is_formation_flight = bool(
                    path_data.get("isFormationFlight") or path_data.get("IsFormationFlight")
                )
                formation_info = (
                    path_data.get("formationInfo")
                    or path_data.get("FormationInfo")
                    or {}
                )
                if isinstance(formation_info, dict):
                    formation_leader_id = _coerce_int(
                        formation_info.get("leaderAircraftID")
                        or formation_info.get("leaderAircraftId")
                        or formation_info.get("LeaderAircraftID")
                    )
                path_waypoints = [
                    waypoint
                    for waypoint in (path_data.get("waypointList") or [])
                    if isinstance(waypoint, dict)
                ]
                is_area_candidate = bool(
                    area_list
                    or ((input_detail or {}).get("area_list") or [])
                )
                inferred_area_roles = (
                    _infer_reciprocal_area_waypoint_roles(path_waypoints)
                    if is_area_candidate
                    else {}
                )
                for waypoint_index, wp in enumerate(path_waypoints):
                    if isinstance(wp, dict):
                        wid = _coerce_int(wp.get("waypointID"))
                        if wid is None:
                            continue
                        line_search_coords: list[dict[str, Any]] = []
                        fp = wp.get("filmingProperty") or wp.get("FilmingProperty") or {}
                        line_search: dict[str, Any] = {}
                        if isinstance(fp, dict):
                            raw_line_search = fp.get("lineSearch") or {}
                            line_search = (
                                raw_line_search if isinstance(raw_line_search, dict) else {}
                            )
                            if isinstance(line_search, dict):
                                coords = line_search.get("coordinateList") or []
                                if isinstance(coords, list):
                                    sweep_point_count += len(coords)
                                    line_search_coords = [
                                        dict(coord)
                                        for coord in coords
                                        if isinstance(coord, dict)
                                    ]
                                    if line_search_coords:
                                        sweep_line_coordinate_lists.append(line_search_coords)
                        waypoint_ids.append(wid)
                        eta_val = wp.get("eta")
                        if eta_val is None:
                            eta_val = wp.get("ETA")
                        eta_float = _coerce_float(eta_val)
                        raw_etas.append(float(eta_float) if eta_float is not None else 0.0)
                        coordinate = wp.get("coordinate") or {}
                        if not isinstance(coordinate, dict):
                            coordinate = {}
                        inferred_role = inferred_area_roles.get(int(waypoint_index), {})
                        waypoint_meta_defs.append(
                            {
                                "sensor_type": _coerce_int(wp.get("sensorType") or wp.get("SensorType")),
                                "operation_mode": _coerce_int(
                                    wp.get("operationMode")
                                    or wp.get("OperationMode")
                                    or fp.get("operationMode")
                                    or fp.get("OperationMode")
                                ),
                                "waypoint_pass_type": _coerce_int(
                                    wp.get("waypointPassType") or wp.get("WaypointPassType")
                                ),
                                "has_filming_property": isinstance(
                                    wp.get("filmingProperty") or wp.get("FilmingProperty"),
                                    dict,
                                ),
                                # Next-collab rugged Area missions deliberately contain
                                # two complete, reciprocal coverage passes.  Preserve the
                                # planner contract here; dropping it made monitoring fold
                                # both passes into one coverage state.
                                "area_coverage_pass": (
                                    str(
                                        wp.get("areaCoveragePass")
                                        or inferred_role.get("area_coverage_pass")
                                        or ""
                                    ).strip().lower()
                                    or None
                                ),
                                "area_turn_role": (
                                    str(
                                        wp.get("areaTurnRole")
                                        or inferred_role.get("area_turn_role")
                                        or ""
                                    ).strip().lower()
                                    or None
                                ),
                                "area_turn_phase": (
                                    str(
                                        wp.get("areaTurnPhase")
                                        or inferred_role.get("area_turn_phase")
                                        or ""
                                    ).strip().lower()
                                    or None
                                ),
                                "coverage_acquisition_id": (
                                    str(
                                        wp.get("coverageAcquisitionID")
                                        or wp.get("coverageAcquisitionId")
                                        or wp.get("coverage_acquisition_id")
                                        or line_search.get("coverageAcquisitionID")
                                        or line_search.get("coverageAcquisitionId")
                                        or line_search.get("coverage_acquisition_id")
                                        or ""
                                    ).strip()
                                    or None
                                ),
                                "line_search_coordinate_list": line_search_coords,
                                "latitude": _coerce_float(coordinate.get("latitude") or coordinate.get("Latitude")),
                                "longitude": _coerce_float(coordinate.get("longitude") or coordinate.get("Longitude")),
                                "altitude": _coerce_float(coordinate.get("altitude") or coordinate.get("Altitude")),
                            }
                        )

            cumulative_etas, used_cumulative = _derive_cumulative_etas(raw_etas)
            if cumulative_etas:
                eta_total = float(cumulative_etas[-1])
            else:
                eta_total = float(sum(max(0.0, v) for v in raw_etas))

            for idx, wid in enumerate(waypoint_ids):
                raw_eta = raw_etas[idx] if idx < len(raw_etas) else 0.0
                if idx < len(cumulative_etas):
                    cum_eta = cumulative_etas[idx]
                elif cumulative_etas:
                    cum_eta = cumulative_etas[-1]
                else:
                    cum_eta = raw_eta
                meta_def = waypoint_meta_defs[idx] if idx < len(waypoint_meta_defs) else {}
                waypoint_defs.append(
                    {
                        "waypoint_id": int(wid),
                        "eta": float(raw_eta),
                        "eta_cumulative": float(cum_eta),
                        "eta_is_cumulative": bool(used_cumulative),
                        "sensor_type": meta_def.get("sensor_type"),
                        "operation_mode": meta_def.get("operation_mode"),
                        "waypoint_pass_type": meta_def.get("waypoint_pass_type"),
                        "has_filming_property": bool(meta_def.get("has_filming_property")),
                        "has_line_search": bool(meta_def.get("line_search_coordinate_list")),
                        "line_search_point_count": len(meta_def.get("line_search_coordinate_list") or []),
                        "area_coverage_pass": meta_def.get("area_coverage_pass"),
                        "area_turn_role": meta_def.get("area_turn_role"),
                        "area_turn_phase": meta_def.get("area_turn_phase"),
                        "coverage_acquisition_id": meta_def.get("coverage_acquisition_id"),
                        # 0303's fixed DLL schema strips custom acquisition
                        # fields, but preserves the FlightPath publication
                        # timestamp.  Keep it in the normalized view so the
                        # coverage ledger can still distinguish replans.
                        "coverage_generation_token": flight_path_timestamp_ms,
                        "flight_path_timestamp_ms": flight_path_timestamp_ms,
                        "latitude": meta_def.get("latitude"),
                        "longitude": meta_def.get("longitude"),
                        "altitude": meta_def.get("altitude"),
                    }
                )
            has_waypoints = bool(waypoint_ids)
            is_formation_input = input_type == 7 if input_type is not None else False
            is_formation_follower = False
            if is_formation_flight:
                if formation_leader_id is not None:
                    is_formation_follower = int(formation_leader_id) != int(uav_id)
                else:
                    is_formation_follower = not has_waypoints
            if not is_formation_follower and is_formation_input and not has_waypoints:
                is_formation_follower = True
            label = "편대 무인기" if is_formation_follower else None
            eta_seconds: int | None = int(round(eta_total))
            if is_formation_follower and not has_waypoints:
                eta_seconds = None
            missions.append(
                {
                    "individual_mission_id": mission_id,
                    "input_id": input_id,
                    "input_mission_type": input_type,
                    "region_type": region_type,
                    "type2_self_reliance_phase": type2_self_reliance_phase,
                    "independent_line_progress": bool(
                        type2_self_reliance_phase
                        in {
                            TYPE2_SELF_RELIANCE_OUTBOUND_LINE,
                            TYPE2_SELF_RELIANCE_RETURN_LINE,
                        }
                    ),
                    "path_id": path_id,
                    "coverage_generation_token": flight_path_timestamp_ms,
                    "flight_path_timestamp_ms": flight_path_timestamp_ms,
                    "waypoint_ids": waypoint_ids,
                    "is_done": is_done,
                    "eta_seconds": eta_seconds,
                    "waypoints": waypoint_defs,
                    "label": label,
                    "skip_progress": bool(
                        is_formation_follower
                        or execution_blocked_until_next_collab
                        or outside_current_input_before_handoff
                    ),
                    "skip_pending": bool(
                        is_formation_follower
                        or execution_blocked_until_next_collab
                        or outside_current_input_before_handoff
                    ),
                    "execution_blocked_until_next_collab": bool(
                        execution_blocked_until_next_collab
                    ),
                    "post_attack_boundary_hold": bool(post_attack_boundary_hold),
                    "boundary_guard_loop": bool(
                        _first_present_value(
                            tuple(
                                (source, "boundaryGuardLoop")
                                for source in boundary_guard_sources
                            ),
                            default=False,
                        )
                    ),
                    "boundary_guard_loop_version": _coerce_int(
                        _first_present_value(
                            tuple(
                                (source, "boundaryGuardLoopVersion")
                                for source in boundary_guard_sources
                            )
                        )
                    ),
                    "boundary_guard_set_id": _first_present_value(
                        tuple(
                            (source, "boundaryGuardSetID")
                            for source in boundary_guard_sources
                        )
                    ),
                    "boundary_guard_sequence": _coerce_int(
                        _first_present_value(
                            tuple(
                                (source, "boundaryGuardSequence")
                                for source in boundary_guard_sources
                            )
                        )
                    ),
                    "boundary_guard_sequence_count": _coerce_int(
                        _first_present_value(
                            tuple(
                                (source, "boundaryGuardSequenceCount")
                                for source in boundary_guard_sources
                            )
                        )
                    ),
                    "boundary_guard_duration_s": _coerce_float(
                        _first_present_value(
                            tuple(
                                (source, "boundaryGuardDurationS")
                                for source in boundary_guard_sources
                            )
                        )
                    ),
                    "boundary_guard_cycle_first_waypoint_id": _coerce_int(
                        _first_present_value(
                            tuple(
                                (source, "boundaryGuardCycleFirstWaypointID")
                                for source in boundary_guard_sources
                            )
                        )
                    ),
                    "boundary_guard_cycle_last_waypoint_id": _coerce_int(
                        _first_present_value(
                            tuple(
                                (source, "boundaryGuardCycleLastWaypointID")
                                for source in boundary_guard_sources
                            )
                        )
                    ),
                    "is_formation_flight": bool(is_formation_flight),
                    "formation_leader_id": formation_leader_id,
                    "sweep_point_count": int(sweep_point_count),
                    "individual_mission_type": mission_type,
                    "pattern_type": pattern_type,
                    "sep_m": sep_m,
                    "width_m": width_m,
                    "source_line_width_m": source_line_width_m,
                    "quality_threshold_m": quality_threshold_m,
                    "sweep_line_coordinate_lists": sweep_line_coordinate_lists,
                    "coordinate_list": list(coordinate_list) if isinstance(coordinate_list, list) else [],
                    "source_coordinate_list": (
                        list(source_coordinate_list)
                        if isinstance(source_coordinate_list, list)
                        else []
                    ),
                    "line_list": list(line_list) if isinstance(line_list, list) else [],
                    "area_list": list(area_list) if isinstance(area_list, list) else [],
                    "input_coordinate_list": list((input_detail or {}).get("coordinate_list") or []),
                    "input_line_list": list((input_detail or {}).get("line_list") or []),
                    "input_area_list": list((input_detail or {}).get("area_list") or []),
                    "area_coverage_pass_contract_version": _coerce_int(
                        _first_present_value(
                            (
                                (mission_info, "areaCoveragePassContractVersion"),
                                (mission, "areaCoveragePassContractVersion"),
                                (input_detail, "area_coverage_pass_contract_version"),
                            )
                        )
                    ),
                    "coverage_acquisition_id": _first_present_value(
                        (
                            (mission_info, "coverageAcquisitionID"),
                            (mission_info, "coverageAcquisitionId"),
                            (mission, "coverageAcquisitionID"),
                            (mission, "coverageAcquisitionId"),
                        )
                    ),
                    "area_coverage_depth_contract_version": _coerce_int(
                        _first_present_value(
                            (
                                (mission_info, "areaCoverageDepthContractVersion"),
                                (mission, "areaCoverageDepthContractVersion"),
                                (input_detail, "area_coverage_depth_contract_version"),
                            )
                        )
                    ),
                    "coverage_depth_policy": _first_present_value(
                        (
                            (mission_info, "coverageDepthPolicy"),
                            (mission, "coverageDepthPolicy"),
                            (input_detail, "coverage_depth_policy"),
                        )
                    ),
                    "required_coverage_depth": _coerce_int(
                        _first_present_value(
                            (
                                (mission_info, "requiredCoverageDepth"),
                                (mission, "requiredCoverageDepth"),
                                (input_detail, "required_coverage_depth"),
                            )
                        )
                    ),
                    "coverage_depth_details": copy.deepcopy(
                        _first_present_value(
                            (
                                (mission_info, "coverageDepthDetails"),
                                (mission, "coverageDepthDetails"),
                                (input_detail, "coverage_depth_details"),
                            ),
                            default=[],
                        )
                        or []
                    ),
                    "coverage_depth_obligations": copy.deepcopy(
                        _first_present_value(
                            (
                                (mission_info, "coverageDepthObligations"),
                                (mission, "coverageDepthObligations"),
                                (input_detail, "coverage_depth_obligations"),
                            ),
                            default=[],
                        )
                        or []
                    ),
                    "coverage_observation_details": copy.deepcopy(
                        _first_present_value(
                            (
                                (mission_info, "coverageObservationDetails"),
                                (mission, "coverageObservationDetails"),
                                (input_detail, "coverage_observation_details"),
                            ),
                            default=[],
                        )
                        or []
                    ),
                    "coverage_pass_policy": _first_present_value(
                        (
                            (mission_info, "coveragePassPolicy"),
                            (mission, "coveragePassPolicy"),
                            (input_detail, "coverage_pass_policy"),
                        )
                    ),
                    "coverage_pass_order": list(
                        _first_present_value(
                            (
                                (mission_info, "coveragePassOrder"),
                                (mission, "coveragePassOrder"),
                                (input_detail, "coverage_pass_order"),
                            ),
                            default=[],
                        )
                        or []
                    ),
                    "remaining_coverage_passes": list(
                        _first_present_value(
                            (
                                (mission_info, "remainingCoveragePasses"),
                                (mission, "remainingCoveragePasses"),
                                (input_detail, "remaining_coverage_passes"),
                            ),
                            default=[],
                        )
                        or []
                    ),
                    "completed_coverage_passes": list(
                        _first_present_value(
                            (
                                (mission_info, "completedCoveragePasses"),
                                (mission, "completedCoveragePasses"),
                                (input_detail, "completed_coverage_passes"),
                            ),
                            default=[],
                        )
                        or []
                    ),
                    "current_coverage_pass": _first_present_value(
                        (
                            (mission_info, "currentCoveragePass"),
                            (mission, "currentCoveragePass"),
                            (input_detail, "current_coverage_pass"),
                        )
                    ),
                    "active_coverage_pass": _first_present_value(
                        (
                            (mission_info, "activeCoveragePass"),
                            (mission, "activeCoveragePass"),
                            (input_detail, "active_coverage_pass"),
                        )
                    ),
                    "area_coverage_phase": _first_present_value(
                        (
                            (mission_info, "areaCoveragePhase"),
                            (mission, "areaCoveragePhase"),
                            (input_detail, "area_coverage_phase"),
                        )
                    ),
                    "coverage_pass_details": copy.deepcopy(
                        _first_present_value(
                            (
                                (mission_info, "coveragePassDetails"),
                                (mission, "coveragePassDetails"),
                                (input_detail, "coverage_pass_details"),
                            ),
                            default=[],
                        )
                        or []
                    ),
                    "coverage_pass_obligations": copy.deepcopy(
                        _first_present_value(
                            (
                                (mission_info, "coveragePassObligations"),
                                (mission, "coveragePassObligations"),
                                (input_detail, "coverage_pass_obligations"),
                            ),
                            default=[],
                        )
                        or []
                    ),
                }
            )
            if is_area_candidate:
                # AREA completion is a single spatial union.  Ignore reciprocal
                # metadata that may still be present in a carried legacy plan.
                area_mission = missions[-1]
                area_mission.update(
                    {
                        "area_coverage_pass_contract_version": None,
                        "area_coverage_depth_contract_version": None,
                        "coverage_depth_policy": None,
                        "required_coverage_depth": 1,
                        "coverage_depth_details": [],
                        "coverage_depth_obligations": [],
                        "coverage_observation_details": [],
                        "coverage_pass_policy": None,
                        "coverage_pass_order": [],
                        "remaining_coverage_passes": [],
                        "completed_coverage_passes": [],
                        "current_coverage_pass": None,
                        "active_coverage_pass": None,
                        "area_coverage_phase": None,
                        "coverage_pass_details": [],
                        "coverage_pass_obligations": [],
                    }
                )
                for waypoint_def in area_mission.get("waypoints") or []:
                    if isinstance(waypoint_def, dict):
                        waypoint_def["area_coverage_pass"] = None
                        waypoint_def["area_turn_role"] = None

        uav_entries.append(
            {
                "aircraft_id": int(uav_id),
                "individual_mission_package_id": package_id,
                "current_individual_mission_id": current_mission_id,
                "missions": missions,
            }
        )

    return {
        "mission_plan_id": mission_plan_id,
        "input_mission_package_id": input_package_id,
        "current_input_mission_id": current_input_mission_id,
        "input_missions": input_items,
        "uav_entries": uav_entries,
    }


def extract_input_mission_package_id(payload: object | None) -> int | None:
    body = parse_payload(payload)
    if not body:
        return None
    for key in ("inputMissionPackageID", "InputMissionPackageID", "inputMissionPackageId"):
        if key in body:
            return _coerce_int(body.get(key))
    return None


def collect_available_aircraft_ids(
    payload: object | None,
    *,
    db_root: Path | str | None = None,
) -> list[int]:
    body = parse_payload(payload)
    available = body.get("availableAircraftList") if body else None
    if available:
        return sorted({
            _coerce_int(item.get("aircraftID"))
            for item in available
            if isinstance(item, dict)
        } - {None})

    package_id = extract_input_mission_package_id(payload)
    if package_id is not None:
        plan = load_db_json("InputMissionPlan", package_id, db_root=db_root)
        available = plan.get("availableAircraftList") or []
        resolved = sorted({
            _coerce_int(item.get("aircraftID"))
            for item in available
            if isinstance(item, dict)
        } - {None})
        if resolved:
            return resolved

    if db_root is None:
        base = db_paths.get_db_subpath("InputMissionPlan")
    else:
        base = Path(db_root) / "InputMissionPlan"

    if not base.exists() or not base.is_dir():
        return []

    resolved_ids: set[int] = set()
    for path in sorted(base.glob("*.json")):
        try:
            plan = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            continue
        available = plan.get("availableAircraftList") or []
        for item in available:
            if not isinstance(item, dict):
                continue
            aircraft_id = _coerce_int(item.get("aircraftID"))
            if aircraft_id is not None:
                resolved_ids.add(aircraft_id)

    return sorted({
        int(aid)
        for aid in resolved_ids
    })


def mark_individual_mission_done(
    package_id: int | None,
    mission_id: int | None,
    *,
    db_root: Path | str | None = None,
) -> bool:
    if package_id is None or mission_id is None:
        return False
    payload = load_db_json("IndividualMissionPlan", package_id, db_root=db_root)
    if not payload:
        return False
    changed = False
    missions = payload.get("individualMissionList") or []
    for mission in missions:
        if not isinstance(mission, dict):
            continue
        if _coerce_int(mission.get("individualMissionID")) != int(mission_id):
            continue
        if not mission.get("isDone"):
            mission["isDone"] = True
            changed = True
    if not changed:
        return False
    return save_db_json("IndividualMissionPlan", package_id, payload, db_root=db_root)


def mark_input_mission_done(
    package_id: int | None,
    input_mission_id: int | None,
    *,
    db_root: Path | str | None = None,
) -> bool:
    if package_id is None or input_mission_id is None:
        return False
    payload = load_db_json("InputMissionPlan", package_id, db_root=db_root)
    if not payload:
        return False
    changed = False
    missions = payload.get("inputMissionList") or []
    for mission in missions:
        if not isinstance(mission, dict):
            continue
        if _coerce_int(mission.get("inputMissionID")) != int(input_mission_id):
            continue
        if not mission.get("isDone"):
            mission["isDone"] = True
            changed = True
    if not changed:
        return False
    return save_db_json("InputMissionPlan", package_id, payload, db_root=db_root)


def mark_waypoints_done(
    path_id: int | None,
    waypoint_ids: Iterable[int] | None,
    *,
    db_root: Path | str | None = None,
) -> bool:
    if path_id is None or not waypoint_ids:
        return False
    payload = load_db_json("FlightPath", path_id, db_root=db_root)
    if not payload:
        return False
    changed = False
    id_set = {int(wid) for wid in waypoint_ids if wid is not None}
    if not id_set:
        return False
    for key in ("waypointList", "lahWaypointList"):
        waypoints = payload.get(key)
        if not isinstance(waypoints, list):
            continue
        for wp in waypoints:
            if not isinstance(wp, dict):
                continue
            wid = _coerce_int(wp.get("waypointID"))
            if wid is None or wid not in id_set:
                continue
            if not wp.get("isDone"):
                wp["isDone"] = True
                changed = True
    if not changed:
        return False
    return save_db_json("FlightPath", path_id, payload, db_root=db_root)


def mark_individual_mission_undone(
    package_id: int | None,
    mission_id: int | None,
    *,
    db_root: Path | str | None = None,
) -> bool:
    if package_id is None or mission_id is None:
        return False
    payload = load_db_json("IndividualMissionPlan", package_id, db_root=db_root)
    if not payload:
        return False
    changed = False
    missions = payload.get("individualMissionList") or []
    for mission in missions:
        if not isinstance(mission, dict):
            continue
        if _coerce_int(mission.get("individualMissionID")) != int(mission_id):
            continue
        if mission.get("isDone"):
            mission["isDone"] = False
            changed = True
    if not changed:
        return False
    return save_db_json("IndividualMissionPlan", package_id, payload, db_root=db_root)


def mark_input_mission_undone(
    package_id: int | None,
    input_mission_id: int | None,
    *,
    db_root: Path | str | None = None,
) -> bool:
    if package_id is None or input_mission_id is None:
        return False
    payload = load_db_json("InputMissionPlan", package_id, db_root=db_root)
    if not payload:
        return False
    changed = False
    missions = payload.get("inputMissionList") or []
    for mission in missions:
        if not isinstance(mission, dict):
            continue
        if _coerce_int(mission.get("inputMissionID")) != int(input_mission_id):
            continue
        if mission.get("isDone"):
            mission["isDone"] = False
            changed = True
    if not changed:
        return False
    return save_db_json("InputMissionPlan", package_id, payload, db_root=db_root)


def coerce_int_list(values: Iterable[object] | None) -> list[int]:
    result: list[int] = []
    if values is None:
        return result
    for value in values:
        try:
            result.append(int(value))
        except Exception:
            continue
    return result
