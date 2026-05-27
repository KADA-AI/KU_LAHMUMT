from __future__ import annotations

import math
from copy import deepcopy
from typing import Any, Callable, Dict, Iterable, List

from .mission_helpers import terrain_elev


DEFAULT_FILMING_TARGET_CLEARANCE_M = 30.0


def _to_float(value: Any) -> float | None:
    try:
        if value is None:
            return None
        return float(value)
    except Exception:
        return None


def _dem_altitude_for_coord(coord: Dict[str, Any]) -> int | None:
    lat = _to_float(coord.get("latitude"))
    lon = _to_float(coord.get("longitude"))
    if lat is None or lon is None:
        return None
    try:
        return int(round(float(terrain_elev(float(lat), float(lon)))))
    except Exception:
        return None


def _normalize_target_coord_altitude(coord: Dict[str, Any]) -> int:
    dem_altitude = _dem_altitude_for_coord(coord)
    if dem_altitude is None:
        return 0
    previous = _to_float(coord.get("altitude"))
    if previous is not None and int(round(float(previous))) == int(dem_altitude):
        return 0
    coord["altitude"] = int(dem_altitude)
    return 1


def _iter_line_search_coords(filming: Dict[str, Any]) -> Iterable[Dict[str, Any]]:
    line_search = filming.get("lineSearch")
    if isinstance(line_search, dict):
        for coord in line_search.get("coordinateList") or []:
            if isinstance(coord, dict):
                yield coord


def _iter_area_search_coords(filming: Dict[str, Any]) -> Iterable[Dict[str, Any]]:
    area_search = filming.get("areaSearch")
    if isinstance(area_search, dict):
        for coord in area_search.get("coordinateList") or []:
            if isinstance(coord, dict):
                yield coord


def normalize_filming_target_altitudes_in_waypoints(
    waypoints: List[Dict[str, Any]],
    *,
    clearance_m: float = DEFAULT_FILMING_TARGET_CLEARANCE_M,
) -> int:
    """Keep filming target altitudes as DEM ground and keep WP altitude above them."""
    if not isinstance(waypoints, list):
        return 0

    changed = 0
    try:
        clearance = max(float(clearance_m), 0.0)
    except Exception:
        clearance = DEFAULT_FILMING_TARGET_CLEARANCE_M

    for waypoint in waypoints:
        if not isinstance(waypoint, dict):
            continue
        filming = waypoint.get("filmingProperty")
        if not isinstance(filming, dict):
            continue

        target_coords: List[Dict[str, Any]] = []
        orientation = filming.get("coordinateOrientation")
        if isinstance(orientation, dict):
            orientation_coord = orientation.get("coordinate")
            if isinstance(orientation_coord, dict):
                target_coords.append(orientation_coord)

        target_coords.extend(_iter_line_search_coords(filming))
        target_coords.extend(_iter_area_search_coords(filming))

        max_target_altitude: float | None = None
        for coord in target_coords:
            changed += _normalize_target_coord_altitude(coord)
            altitude = _to_float(coord.get("altitude"))
            if altitude is None:
                continue
            max_target_altitude = (
                float(altitude)
                if max_target_altitude is None
                else max(float(max_target_altitude), float(altitude))
            )

        if max_target_altitude is None:
            continue

        waypoint_coord = waypoint.get("coordinate")
        if not isinstance(waypoint_coord, dict):
            continue
        waypoint_altitude = _to_float(waypoint_coord.get("altitude"))
        minimum_altitude = int(math.ceil(float(max_target_altitude) + clearance))
        if waypoint_altitude is None or int(round(float(waypoint_altitude))) < minimum_altitude:
            waypoint_coord["altitude"] = int(minimum_altitude)
            waypoint["coordinate"] = waypoint_coord
            changed += 1

    return changed


def sanitize_flight_path_payload_filming_altitudes(
    payload: Dict[str, Any],
    *,
    clearance_m: float = DEFAULT_FILMING_TARGET_CLEARANCE_M,
) -> int:
    if not isinstance(payload, dict):
        return 0
    primary = payload.get("waypointList")
    secondary = payload.get("lahWaypointList")
    if isinstance(primary, list) and primary:
        waypoints = primary
    elif isinstance(secondary, list) and secondary:
        waypoints = secondary
    elif isinstance(primary, list):
        waypoints = primary
    else:
        waypoints = secondary
    if not isinstance(waypoints, list):
        return 0

    changed = normalize_filming_target_altitudes_in_waypoints(
        waypoints,
        clearance_m=clearance_m,
    )
    if "waypointList" in payload:
        payload["waypointList"] = waypoints
    if isinstance(payload.get("lahWaypointList"), list):
        payload["lahWaypointList"] = deepcopy(waypoints)
    return int(changed)
