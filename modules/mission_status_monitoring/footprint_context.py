from __future__ import annotations

from copy import deepcopy
from typing import Any

from modules.monitoring.logic.mission_update import extract_0401_agent_states


def build_0401_footprint_context(payload: object | None) -> dict[str, Any]:
    """Keep compact 0401 positions/footprints for timestamped event context."""

    timestamp, states = extract_0401_agent_states(payload)
    by_aircraft: dict[str, list[dict[str, Any]]] = {}
    positions: dict[str, dict[str, float]] = {}
    for state in states:
        if not isinstance(state, dict):
            continue
        try:
            aircraft_id = int(state.get("aircraft_id"))
        except Exception:
            continue
        coordinate = state.get("coordinate")
        if isinstance(coordinate, dict):
            try:
                position = {
                    "latitude": float(coordinate.get("latitude")),
                    "longitude": float(coordinate.get("longitude")),
                }
                if coordinate.get("altitude") is not None:
                    position["altitude"] = float(coordinate.get("altitude"))
                positions[str(aircraft_id)] = position
            except Exception:
                pass
        corners = state.get("footprint_corners")
        if not isinstance(corners, list) or len(corners) < 4:
            continue
        by_aircraft[str(aircraft_id)] = deepcopy(corners[:4])
    return {
        "timestamp": int(timestamp) if timestamp is not None else None,
        "byAircraft": by_aircraft,
        "positions": positions,
    }


def footprint_for_aircraft(
    context: object,
    aircraft_id: int | None,
) -> tuple[list[dict[str, Any]], int | None]:
    if aircraft_id is None or not isinstance(context, dict):
        return [], None
    values = context.get("byAircraft")
    if not isinstance(values, dict):
        return [], None
    corners = values.get(str(int(aircraft_id)))
    if not isinstance(corners, list) or len(corners) < 4:
        return [], None
    try:
        timestamp = int(context.get("timestamp"))
    except Exception:
        timestamp = None
    return deepcopy(corners[:4]), timestamp


def position_for_aircraft(
    context: object,
    aircraft_id: int | None,
) -> tuple[dict[str, float] | None, int | None]:
    if aircraft_id is None or not isinstance(context, dict):
        return None, None
    values = context.get("positions")
    if not isinstance(values, dict):
        return None, None
    position = values.get(str(int(aircraft_id)))
    if not isinstance(position, dict):
        return None, None
    try:
        latitude = float(position.get("latitude"))
        longitude = float(position.get("longitude"))
    except Exception:
        return None, None
    result = {"latitude": latitude, "longitude": longitude}
    try:
        if position.get("altitude") is not None:
            result["altitude"] = float(position.get("altitude"))
    except Exception:
        pass
    try:
        timestamp = int(context.get("timestamp"))
    except Exception:
        timestamp = None
    return result, timestamp


__all__ = [
    "build_0401_footprint_context",
    "footprint_for_aircraft",
    "position_for_aircraft",
]
