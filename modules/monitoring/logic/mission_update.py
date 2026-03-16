# -*- coding: utf-8 -*-
from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any, Iterable

from modules.common import db_paths


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

    def _extract_on_mission(*containers: object) -> int | None:
        for container in containers:
            if not isinstance(container, dict):
                continue
            for key in ("onMission", "OnMission"):
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
                        return corners
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
                    return legacy
        return corners

    datalink_votes: dict[int, list[bool]] = {4: [], 5: [], 6: []}
    for item in raw_list:
        if not isinstance(item, dict):
            continue
        for aircraft_id, connected in _extract_manned_datalink(item).items():
            if connected is None:
                continue
            datalink_votes.setdefault(int(aircraft_id), []).append(bool(connected))

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
        fuel_val = _extract_fuel(item, unmanned_info)
        fuel_warning = _extract_fuel_warning(unmanned_info, flight_mode_info, item)
        aircraft_id = _coerce_int(item.get("aircraftID") or item.get("AircraftID"))
        states.append(
            {
                "aircraft_id": aircraft_id,
                "health": _coerce_int(item.get("health") or item.get("Health")),
                "last_signal_time": _extract_last_signal(item),
                "is_unmanned": item.get("isUnmanned")
                if "isUnmanned" in item or "IsUnmanned" in item
                else (aircraft_id or 0) >= 4,
                "current_waypoint_id": _extract_current_waypoint(unmanned_info, flight_mode_info, item),
                "on_mission": _extract_on_mission(unmanned_info, flight_mode_info, item),
                "flight_mode": _extract_flight_mode(unmanned_info, flight_mode_info, item),
                "payload_health": _extract_payload_health(unmanned_info, flight_mode_info, item),
                "fuel_liters": fuel_val,
                "fuel_warning": fuel_warning,
                "datalink_connected": datalink_connected_by_uav.get(int(aircraft_id), None)
                if aircraft_id is not None and aircraft_id >= 4
                else None,
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


def _resolve_db_json_path(folder: str, file_id: int | None, db_root: Path | str | None = None) -> Path | None:
    if file_id is None:
        return None
    if db_root is None:
        base = db_paths.get_db_subpath(folder)
    else:
        base = Path(db_root) / folder
    direct = base / f"{int(file_id)}.json"
    if direct.exists():
        return direct

    id_keys = _folder_id_keys(folder)
    if not id_keys or not base.exists():
        return direct

    for path in sorted(base.glob("*.json")):
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            continue
        if not isinstance(payload, dict):
            continue
        for key in id_keys:
            value = _coerce_int(payload.get(key))
            if value is not None and int(value) == int(file_id):
                return path
    return direct


def load_db_json(folder: str, file_id: int | None, db_root: Path | str | None = None) -> dict[str, Any]:
    if file_id is None:
        return {}
    path = _resolve_db_json_path(folder, file_id, db_root=db_root)
    if path is None:
        return {}
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}


def save_db_json(folder: str, file_id: int | None, payload: dict[str, Any], db_root: Path | str | None = None) -> bool:
    if file_id is None:
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
        path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        return True
    except Exception:
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
    current_input_mission_id = _select_next_pending_id(input_missions, "inputMissionID")
    input_type_map: dict[int, int] = {}
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
        input_items.append(
            {
                "input_mission_id": mission_id,
                "is_done": bool(item.get("isDone")),
            }
        )

    uav_entries: list[dict[str, Any]] = []
    for uav_id in uav_ids:
        package_id = None
        for entry in aircraft_list:
            if _coerce_int(entry.get("aircraftID")) == int(uav_id):
                package_id = _coerce_int(entry.get("individualMissionPackageID"))
                break

        individual_plan = load_db_json("IndividualMissionPlan", package_id, db_root=db_root)
        mission_list = individual_plan.get("individualMissionList") or []
        current_mission_id: int | None = _select_next_pending_id(mission_list, "individualMissionID")
        missions: list[dict[str, Any]] = []

        for mission in mission_list:
            mission_id = _coerce_int(mission.get("individualMissionID"))
            related = mission.get("relatedMission") or {}
            input_id = _coerce_int(related.get("inputMissionID"))
            input_type = input_type_map.get(int(input_id)) if input_id is not None else None
            path_id = _coerce_int(mission.get("pathID"))
            mission_info = mission.get("individualMissionInfo") or {}
            if not isinstance(mission_info, dict):
                mission_info = {}
            line_list = mission_info.get("lineList") or []
            area_list = mission_info.get("areaList") or []
            mission_type = _coerce_int(mission_info.get("individualMissionType"))
            pattern_type = _coerce_int(mission_info.get("patternType"))
            waypoint_ids: list[int] = []
            is_done = bool(mission.get("isDone"))
            raw_etas: list[float] = []
            waypoint_defs: list[dict[str, Any]] = []
            waypoint_meta_defs: list[dict[str, Any]] = []
            sweep_point_count = 0
            is_formation_flight = False
            formation_leader_id: int | None = None
            if path_id is not None:
                path_data = load_db_json("FlightPath", path_id, db_root=db_root)
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
                for wp in path_data.get("waypointList") or []:
                    if isinstance(wp, dict):
                        wid = _coerce_int(wp.get("waypointID"))
                        if wid is None:
                            continue
                        fp = wp.get("filmingProperty") or {}
                        if isinstance(fp, dict):
                            line_search = fp.get("lineSearch") or {}
                            if isinstance(line_search, dict):
                                coords = line_search.get("coordinateList") or []
                                if isinstance(coords, list):
                                    sweep_point_count += len(coords)
                        waypoint_ids.append(wid)
                        eta_val = wp.get("eta")
                        if eta_val is None:
                            eta_val = wp.get("ETA")
                        eta_float = _coerce_float(eta_val)
                        raw_etas.append(float(eta_float) if eta_float is not None else 0.0)
                        coordinate = wp.get("coordinate") or {}
                        if not isinstance(coordinate, dict):
                            coordinate = {}
                        waypoint_meta_defs.append(
                            {
                                "sensor_type": _coerce_int(wp.get("sensorType") or wp.get("SensorType")),
                                "operation_mode": _coerce_int(wp.get("operationMode") or wp.get("OperationMode")),
                                "waypoint_pass_type": _coerce_int(
                                    wp.get("waypointPassType") or wp.get("WaypointPassType")
                                ),
                                "has_filming_property": isinstance(
                                    wp.get("filmingProperty") or wp.get("FilmingProperty"),
                                    dict,
                                ),
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
                    "path_id": path_id,
                    "waypoint_ids": waypoint_ids,
                    "is_done": is_done,
                    "eta_seconds": eta_seconds,
                    "waypoints": waypoint_defs,
                    "label": label,
                    "skip_progress": bool(is_formation_follower),
                    "skip_pending": bool(is_formation_follower),
                    "is_formation_flight": bool(is_formation_flight),
                    "formation_leader_id": formation_leader_id,
                    "sweep_point_count": int(sweep_point_count),
                    "individual_mission_type": mission_type,
                    "pattern_type": pattern_type,
                    "line_list": list(line_list) if isinstance(line_list, list) else [],
                    "area_list": list(area_list) if isinstance(area_list, list) else [],
                }
            )

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
    if package_id is None:
        return []
    plan = load_db_json("InputMissionPlan", package_id, db_root=db_root)
    available = plan.get("availableAircraftList") or []
    return sorted({
        _coerce_int(item.get("aircraftID"))
        for item in available
        if isinstance(item, dict)
    } - {None})


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
