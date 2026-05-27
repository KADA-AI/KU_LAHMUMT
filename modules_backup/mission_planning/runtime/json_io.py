# -*- coding: utf-8 -*-
from __future__ import annotations

import json
from copy import deepcopy
from pathlib import Path
from typing import Any

try:
    import orjson as _orjson
except Exception:  # pragma: no cover - optional dependency
    _orjson = None


def _normalize_flight_path_payload(path: Path, data: Any) -> Any:
    if not _is_flight_path_target(path, data) or "Source" not in data:
        return data

    normalized: dict[str, Any] = {}
    has_lower_source = "source" in data
    for key, value in data.items():
        if key == "Source":
            if not has_lower_source:
                normalized["source"] = value
            continue
        normalized[key] = value
    return normalized


def _is_flight_path_target(path: Path, data: Any) -> bool:
    if not isinstance(path, Path) or not isinstance(data, dict):
        return False
    if not any(isinstance(data.get(key), list) for key in ("waypointList", "uavWaypointList", "lahWaypointList")):
        return False
    if path.parent.name == "FlightPath":
        return True
    return path.stem.startswith("FlightPath")


def _apply_uav_speed_weight_to_flight_path_payload(path: Path, data: Any) -> Any:
    if not _is_flight_path_target(path, data) or not isinstance(data, dict):
        return data
    try:
        aircraft_id = int(data.get("aircraftID", 0) or 0)
    except Exception:
        aircraft_id = 0
    if aircraft_id not in (4, 5, 6):
        return data

    try:
        from ..MissionPlanner.runtime_settings import apply_runtime_uav_speed_weight_mps
    except Exception:
        try:
            from modules.mission_planning.MissionPlanner.runtime_settings import apply_runtime_uav_speed_weight_mps  # type: ignore
        except Exception:
            apply_runtime_uav_speed_weight_mps = None  # type: ignore

    if apply_runtime_uav_speed_weight_mps is None:
        return data

    payload = deepcopy(data)
    changed = False
    changed_keys: list[str] = []

    def _apply_speed(value: Any) -> float | None:
        try:
            speed = float(value)
        except Exception:
            return None
        if speed <= 0.0:
            return None
        weighted = float(apply_runtime_uav_speed_weight_mps(speed))
        return round(weighted, 2)

    for key in ("waypointList", "uavWaypointList"):
        waypoints = payload.get(key)
        if not isinstance(waypoints, list):
            continue
        key_changed = False
        for waypoint in waypoints:
            if not isinstance(waypoint, dict):
                continue
            new_speed = _apply_speed(waypoint.get("speed"))
            if new_speed is not None and abs(float(waypoint.get("speed", 0.0) or 0.0) - new_speed) > 1e-6:
                waypoint["speed"] = float(new_speed)
                key_changed = True
                changed = True
            loiter = waypoint.get("loiterProperty")
            if isinstance(loiter, dict):
                new_loiter_speed = _apply_speed(loiter.get("speed"))
                if (
                    new_loiter_speed is not None
                    and abs(float(loiter.get("speed", 0.0) or 0.0) - new_loiter_speed) > 1e-6
                ):
                    loiter["speed"] = float(new_loiter_speed)
                    key_changed = True
                    changed = True
        if key_changed:
            changed_keys.append(key)

    if not changed:
        return data

    try:
        from modules.common.eta import annotate_eta_flight_plan
    except Exception:
        annotate_eta_flight_plan = None  # type: ignore

    if annotate_eta_flight_plan is not None:
        for key in changed_keys:
            try:
                annotate_eta_flight_plan(payload, waypoint_list_keys=(key,))
            except Exception:
                continue

    return payload


def prepare_json_payload(path: Path, data: Any) -> Any:
    prepared = _normalize_flight_path_payload(path, data)
    prepared = _apply_uav_speed_weight_to_flight_path_payload(path, prepared)
    return prepared


def dumps_json(
    data: Any,
    *,
    pretty: bool = True,
    ensure_ascii: bool = False,
    sort_keys: bool = False,
) -> bytes:
    if _orjson is not None:
        option = 0
        if pretty:
            option |= _orjson.OPT_INDENT_2
        if sort_keys:
            option |= _orjson.OPT_SORT_KEYS
        if ensure_ascii:
            option |= _orjson.OPT_ESCAPE_UNICODE
        return _orjson.dumps(data, option=option)

    if pretty:
        text = json.dumps(
            data,
            ensure_ascii=ensure_ascii,
            indent=2,
            sort_keys=sort_keys,
        )
    else:
        text = json.dumps(
            data,
            ensure_ascii=ensure_ascii,
            separators=(",", ":"),
            sort_keys=sort_keys,
        )
    return text.encode("utf-8")


def write_json(
    path: Path,
    data: Any,
    *,
    pretty: bool = True,
    ensure_ascii: bool = False,
    sort_keys: bool = False,
    skip_if_unchanged: bool = True,
) -> bool:
    data = prepare_json_payload(path, data)
    payload = dumps_json(
        data,
        pretty=pretty,
        ensure_ascii=ensure_ascii,
        sort_keys=sort_keys,
    )

    if skip_if_unchanged and path.exists():
        try:
            if path.stat().st_size == len(payload):
                if path.read_bytes() == payload:
                    return False
        except Exception:
            pass

    path.parent.mkdir(parents=True, exist_ok=True)
    tmp_path = path.with_suffix(path.suffix + ".tmp")
    tmp_path.write_bytes(payload)
    tmp_path.replace(path)
    return True
