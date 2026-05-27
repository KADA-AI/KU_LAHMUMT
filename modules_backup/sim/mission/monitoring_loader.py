from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from modules.common import agent_status_snapshot, db_paths
from modules.common.footprint_corners import normalize_footprint_ring

from .mission_plan_loader import build_mission_plan_payload
from .mission_validator import validate_mission_plan_result


def _now_ms_2000() -> int:
    from datetime import datetime, timezone

    epoch = datetime(2000, 1, 1, tzinfo=timezone.utc)
    return int((datetime.now(timezone.utc) - epoch).total_seconds() * 1000)


def _to_int(value: Any) -> int | None:
    try:
        return int(value)
    except Exception:
        return None


def _to_float(value: Any) -> float | None:
    try:
        return float(value)
    except Exception:
        return None


def _pick_value(obj: dict[str, Any] | None, keys: tuple[str, ...], default: Any = None) -> Any:
    if not isinstance(obj, dict):
        return default
    for key in keys:
        if key in obj:
            return obj[key]
    return default


def _load_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}


def _normalize_payload(raw: Any) -> dict[str, Any] | None:
    if raw is None:
        return None
    if isinstance(raw, dict):
        return raw
    if isinstance(raw, list):
        for item in reversed(raw):
            parsed = _normalize_payload(item)
            if parsed:
                return parsed
        return None
    if isinstance(raw, (bytes, bytearray)):
        try:
            return _normalize_payload(bytes(raw).decode("utf-8", "ignore"))
        except Exception:
            return None
    if isinstance(raw, str):
        try:
            return json.loads(raw)
        except Exception:
            start = raw.find("{")
            end = raw.rfind("}")
            if start >= 0 and end > start:
                try:
                    return json.loads(raw[start : end + 1])
                except Exception:
                    return None
    return None


def _agent_label(aircraft_id: int | None) -> str | None:
    if aircraft_id is None:
        return None
    if 1 <= aircraft_id <= 3:
        return f"LAH{aircraft_id}"
    if 4 <= aircraft_id <= 6:
        return f"UAV{aircraft_id - 3}"
    return f"AC{aircraft_id}"


def _extract_plan_id(payload: dict[str, Any] | None) -> int | None:
    if not isinstance(payload, dict):
        return None
    return _to_int(
        _pick_value(
            payload,
            ("missionPlanID", "MissionPlanID", "missionPlanId", "mission_plan_id"),
        )
    )


def _extract_payload_timestamp(payload: dict[str, Any] | None) -> int | None:
    if not isinstance(payload, dict):
        return None
    return _to_int(_pick_value(payload, ("timestamp", "Timestamp", "timeStamp", "TimeStamp")))


def _latest_integration_payload(integration, msg_id: str) -> dict[str, Any] | None:
    if integration is None:
        return None
    try:
        result = integration.get_payload(msg_id, "rx")
    except Exception:
        return None
    if not isinstance(result, dict) or not result.get("ok"):
        return None
    return _normalize_payload(result.get("payload"))


def _resolve_option_plan(payload: dict[str, Any] | None) -> int | None:
    if not isinstance(payload, dict):
        return None
    raw_list = _pick_value(
        payload,
        ("optionList", "OptionList", "option_list", "optionInfoList"),
        [],
    )
    if not isinstance(raw_list, list):
        return None
    normalized = []
    for option in raw_list:
        if not isinstance(option, dict):
            continue
        plan_id = _extract_plan_id(option)
        if plan_id is None:
            continue
        recommend = _pick_value(option, ("recommend", "Recommend", "recommended"), False)
        normalized.append((bool(recommend), plan_id))
    if not normalized:
        return None
    recommended = next((plan_id for is_recommended, plan_id in normalized if is_recommended), None)
    return recommended if recommended is not None else normalized[0][1]


def _latest_mission_plan_id_from_db(base: Path) -> tuple[int | None, int | None]:
    folder = base / "MissionPlan"
    if not folder.exists():
        return None, None
    best_id = None
    best_ts = None
    for path in folder.glob("*.json"):
        data = _load_json(path)
        if not data:
            continue
        plan_id = _extract_plan_id(data)
        if plan_id is None:
            plan_id = _to_int(path.stem)
        ts = _to_int(
            _pick_value(
                data,
                ("missionPlanTimestamp", "MissionPlanTimestamp", "timestamp", "Timestamp"),
            )
        )
        if ts is None:
            try:
                ts = int(path.stat().st_mtime_ns)
            except Exception:
                ts = None
        if plan_id is None or ts is None:
            continue
        if best_id is None or ts >= (best_ts or -1):
            best_id = plan_id
            best_ts = ts
    return best_id, best_ts


def _resolve_current_mission_plan(integration, base: Path) -> tuple[int | None, str | None, int | None]:
    candidates: list[tuple[int, int, str]] = []

    payload_0903 = _latest_integration_payload(integration, "0903")
    plan_0903 = _extract_plan_id(payload_0903)
    ts_0903 = _extract_payload_timestamp(payload_0903)
    if plan_0903 is not None:
        candidates.append((ts_0903 or 0, plan_0903, "0903"))

    payload_0702 = _latest_integration_payload(integration, "0702")
    if isinstance(payload_0702, dict):
        ignore = _to_int(_pick_value(payload_0702, ("ignore", "Ignore")))
        plan_0702 = _extract_plan_id(payload_0702)
        ts_0702 = _extract_payload_timestamp(payload_0702)
        if ignore == 2 and plan_0702 is not None:
            candidates.append((ts_0702 or 0, plan_0702, "0702"))

    payload_0701 = _latest_integration_payload(integration, "0701")
    plan_0701 = _resolve_option_plan(payload_0701)
    ts_0701 = _extract_payload_timestamp(payload_0701)
    if plan_0701 is not None:
        candidates.append((ts_0701 or 0, plan_0701, "0701"))

    if candidates:
        candidates.sort(key=lambda item: (item[0], item[2]))
        timestamp, plan_id, source = candidates[-1]
        return plan_id, source, timestamp or None

    latest_id, latest_ts = _latest_mission_plan_id_from_db(base)
    if latest_id is not None:
        return latest_id, "db", latest_ts
    return None, None, None


def _coord_from_0401(coord: dict[str, Any] | None) -> dict[str, float] | None:
    if not isinstance(coord, dict):
        return None
    lat = _to_float(_pick_value(coord, ("latitude", "Latitude")))
    lon = _to_float(_pick_value(coord, ("longitude", "Longitude")))
    alt = _to_float(_pick_value(coord, ("altitude", "Altitude")))
    if lat is None or lon is None:
        return None
    return {
        "lat": float(lat),
        "lon": float(lon),
        "alt": float(alt) if alt is not None else 0.0,
    }


def _footprint_corners_from_0401(sensor_info: dict[str, Any] | None) -> list[list[float]] | None:
    if not isinstance(sensor_info, dict):
        return None
    corners = sensor_info.get("footprintCornerList")
    if not isinstance(corners, list) or len(corners) < 3:
        return None
    ring: list[list[float]] = []
    for corner in corners:
        coord = _coord_from_0401(corner)
        if coord is None:
            continue
        ring.append([coord["lon"], coord["lat"]])
    if len(ring) < 3:
        return None
    normalized = normalize_footprint_ring(ring, closed=True)
    return normalized or None


def _build_vehicles_from_0401(payload: dict[str, Any] | None) -> tuple[dict[str, Any], dict[str, int], int | None]:
    vehicles: dict[str, Any] = {}
    current_waypoints: dict[str, int] = {}
    timestamp = _extract_payload_timestamp(payload)
    agent_states = payload.get("agentStateList") if isinstance(payload, dict) else None
    if not isinstance(agent_states, list):
        return vehicles, current_waypoints, timestamp

    for state in agent_states:
        if not isinstance(state, dict):
            continue
        aircraft_id = _to_int(_pick_value(state, ("aircraftID", "AircraftID")))
        label = _agent_label(aircraft_id)
        coord = _coord_from_0401(state.get("coordinate"))
        velocity = state.get("velocity") if isinstance(state.get("velocity"), dict) else {}
        if label is None or coord is None:
            continue
        entry: dict[str, Any] = {
            "lat": coord["lat"],
            "lon": coord["lon"],
            "alt": coord["alt"],
            "speed": float(_to_float(_pick_value(velocity, ("speed", "Speed"))) or 0.0),
            "heading": float(_to_float(_pick_value(velocity, ("heading", "Heading"))) or 0.0),
            "alive": int(_to_int(state.get("health")) or 1) != 2,
            "health": int(_to_int(state.get("health")) or 1),
            "fuel": float(_to_float(state.get("fuel")) or 0.0),
        }
        if bool(state.get("isUnmanned")):
            unmanned = state.get("unmannedInfo") if isinstance(state.get("unmannedInfo"), dict) else {}
            current_wp = _to_int(
                _pick_value(
                    unmanned.get("currentWaypointID") if isinstance(unmanned.get("currentWaypointID"), dict) else {},
                    ("waypointID", "WaypointID"),
                )
            ) or 0
            target_following = (
                unmanned.get("targetFollowing")
                if isinstance(unmanned.get("targetFollowing"), dict)
                else {}
            )
            target_id = _to_int(_pick_value(target_following, ("targetID", "TargetID"))) or 0
            entry["flightMode"] = int(_to_int(unmanned.get("flightMode")) or 0)
            entry["flying"] = int(_to_int(_pick_value(unmanned, ("flying", "Flying"))) or 0)
            entry["currentWaypointID"] = current_wp
            entry["targetID"] = target_id
            entry["payloadHealth"] = int(_to_int(unmanned.get("payloadHealth")) or 0)
            entry["fuelWarning"] = int(_to_int(unmanned.get("fuelWarning")) or 0)
            current_waypoints[label] = current_wp

            loiter = _coord_from_0401(unmanned.get("loiterCoordinate"))
            if loiter is not None:
                entry["loiterCoordinate"] = {
                    "latitude": loiter["lat"],
                    "longitude": loiter["lon"],
                    "altitude": loiter["alt"],
                }

            sensor_info = unmanned.get("sensorInfo") if isinstance(unmanned.get("sensorInfo"), dict) else {}
            entry["filming"] = int(_to_int(_pick_value(sensor_info, ("filming", "Filming"))) or 0)
            center = _coord_from_0401(sensor_info.get("centerCoordinate"))
            if center is not None:
                entry["filmingTarget"] = center
            fov = _to_float(_pick_value(sensor_info, ("fov", "Fov")))
            if fov is not None:
                entry["filmingFov"] = float(fov)
            corners = _footprint_corners_from_0401(sensor_info)
            if corners is not None:
                entry["footprintCorners"] = corners
        else:
            manned = state.get("mannedInfo") if isinstance(state.get("mannedInfo"), dict) else {}
            weapons = manned.get("weapons") if isinstance(manned.get("weapons"), dict) else None
            if weapons:
                entry["weapons"] = dict(weapons)
        vehicles[label] = entry

    return vehicles, current_waypoints, timestamp


def build_monitoring_snapshot(integration, mission_since: str | None = None) -> dict[str, Any]:
    db_root = Path(db_paths.get_active_db_root())
    payload_0401 = _latest_integration_payload(integration, "0401")
    source = "integration:0401"
    if payload_0401 is None:
        snapshot = agent_status_snapshot.load_agent_status_snapshot() or {}
        payload_0401 = _normalize_payload(snapshot.get("raw") or snapshot.get("agent_states") or snapshot)
        source = "snapshot:0401"

    vehicles, current_waypoints, timestamp = _build_vehicles_from_0401(payload_0401)
    plan_id, plan_source, plan_timestamp = _resolve_current_mission_plan(integration, db_root)

    mission_signature = (
        f"{plan_id}:{plan_source or 'unknown'}:{plan_timestamp or 0}" if plan_id is not None else None
    )
    mission_since_text = str(mission_since or "").strip()
    mission_payload: dict[str, Any] | None = None
    if plan_id is not None:
        if mission_signature != mission_since_text:
            result = build_mission_plan_payload(plan_id, db_root=db_root)
            if result.get("ok"):
                validation = validate_mission_plan_result(result)
                mission_payload = {
                    **result,
                    "signature": mission_signature,
                    "source": plan_source,
                    "selectedTimestamp": plan_timestamp,
                    "validation": validation,
                }
            else:
                mission_payload = {
                    "ok": False,
                    "missionPlanID": plan_id,
                    "error": result.get("error"),
                    "signature": mission_signature,
                    "source": plan_source,
                    "selectedTimestamp": plan_timestamp,
                }

    current_timestamp = timestamp if timestamp is not None else _now_ms_2000()
    return {
        "ok": True,
        "mode": "monitor",
        "monitoring": True,
        "source": source,
        "timestamp": int(current_timestamp),
        "step": int(current_timestamp),
        "simTime": 0.0,
        "vehicles": vehicles,
        "targets": [],
        "projectiles": [],
        "effects": [],
        "currentWaypoints": current_waypoints,
        "missionAvailable": mission_signature is not None,
        "missionSignature": mission_signature,
        "mission": mission_payload,
        "dbRoot": str(db_root),
    }
