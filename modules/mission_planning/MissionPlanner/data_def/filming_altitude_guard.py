from __future__ import annotations

import math
import os
import threading
import time
from copy import deepcopy
from typing import Any, Callable, Dict, Iterable, List

from .mission_helpers import terrain_elev, terrain_elev_many
try:
    from ..runtime_settings import get_runtime_value as _get_runtime_value
except Exception:
    try:
        from runtime_settings import get_runtime_value as _get_runtime_value  # type: ignore
    except Exception:
        try:
            from modules.mission_planning.MissionPlanner.runtime_settings import (  # type: ignore
                get_runtime_value as _get_runtime_value,
            )
        except Exception:
            _get_runtime_value = None


DEFAULT_FILMING_TARGET_CLEARANCE_M = 30.0
DEFAULT_OUTPUT_MIN_FOV_DEG = 3.0
_DEM_ALT_COORD_TRUSTED_KEY = "_demAltitudeTrusted"
_FILMING_ALTITUDE_METRICS_LOCAL = threading.local()
_UINT32_MAX = 0xFFFFFFFF
_DEM_ALT_CACHE_MAX = 200_000
_DEM_ALT_CACHE_LOCK = threading.Lock()
_DEM_ALT_CACHE: dict[tuple[float, float], int] = {}


def reset_filming_altitude_metrics() -> None:
    _FILMING_ALTITUDE_METRICS_LOCAL.metrics = {
        "filmingTargetCoordCount": 0,
        "filmingUniqueCoordCount": 0,
        "filmingTerrainBatchMs": 0.0,
        "filmingTerrainBatchFallbackCount": 0,
        "filmingTrustedCoordReuseCount": 0,
        "filmingTerrainLookupCoordCount": 0,
    }


def get_filming_altitude_metrics(*, reset: bool = False) -> dict:
    metrics = getattr(_FILMING_ALTITUDE_METRICS_LOCAL, "metrics", None)
    if not isinstance(metrics, dict):
        reset_filming_altitude_metrics()
        metrics = getattr(_FILMING_ALTITUDE_METRICS_LOCAL, "metrics", {})
    out = dict(metrics)
    out["filmingTerrainBatchMs"] = round(float(out.get("filmingTerrainBatchMs") or 0.0), 3)
    if reset:
        reset_filming_altitude_metrics()
    return out


def _record_filming_altitude_metrics(**values: float | int) -> None:
    metrics = getattr(_FILMING_ALTITUDE_METRICS_LOCAL, "metrics", None)
    if not isinstance(metrics, dict):
        reset_filming_altitude_metrics()
        metrics = getattr(_FILMING_ALTITUDE_METRICS_LOCAL, "metrics", {})
    for key, value in values.items():
        if key == "filmingTerrainBatchMs":
            metrics[key] = float(metrics.get(key, 0.0) or 0.0) + float(value or 0.0)
        else:
            metrics[key] = int(metrics.get(key, 0) or 0) + int(value or 0)


def _batch_enabled(default: bool = True) -> bool:
    raw = os.environ.get("MISSION_PLAN_FILMING_ALTITUDE_BATCH_ENABLED")
    if raw is None and callable(_get_runtime_value):
        try:
            raw = _get_runtime_value("filming_altitude_batch_enabled", default)
        except Exception:
            raw = default
    if raw is None:
        return bool(default)
    if isinstance(raw, str):
        lowered = raw.strip().lower()
        if lowered in {"1", "true", "yes", "on"}:
            return True
        if lowered in {"0", "false", "no", "off"}:
            return False
    return bool(raw)


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


def _coord_key(coord: Dict[str, Any]) -> tuple[float, float] | None:
    lat = _to_float(coord.get("latitude"))
    lon = _to_float(coord.get("longitude"))
    if lat is None or lon is None:
        return None
    return (round(float(lat), 7), round(float(lon), 7))


def _trusted_dem_altitude(coord: Dict[str, Any]) -> int | None:
    if coord.get(_DEM_ALT_COORD_TRUSTED_KEY) is not True:
        return None
    altitude = _to_float(coord.get("altitude"))
    if altitude is None:
        return None
    return int(round(float(altitude)))


def _dem_altitudes_for_coords(
    coords: List[Dict[str, Any]],
    *,
    dem_lookup_many: Callable[[list[tuple[float, float]]], list[Any]] | None = None,
) -> dict[tuple[float, float], int]:
    dem_by_key: dict[tuple[float, float], int] = {}
    for coord in coords:
        key = _coord_key(coord)
        if key is None:
            continue
        altitude = _trusted_dem_altitude(coord)
        if altitude is not None:
            dem_by_key[key] = int(altitude)

    keys: list[tuple[float, float]] = []
    seen: set[tuple[float, float]] = set()
    for coord in coords:
        key = _coord_key(coord)
        if key is None or key in seen:
            continue
        seen.add(key)
        if key in dem_by_key:
            continue
        keys.append(key)

    if dem_by_key:
        _record_filming_altitude_metrics(
            filmingTrustedCoordReuseCount=len(dem_by_key),
        )
    _record_filming_altitude_metrics(
        filmingTerrainLookupCoordCount=len(keys),
    )

    if keys:
        uncached_keys: list[tuple[float, float]] = []
        with _DEM_ALT_CACHE_LOCK:
            for key in keys:
                cached = _DEM_ALT_CACHE.get(key)
                if cached is None:
                    uncached_keys.append(key)
                else:
                    dem_by_key[key] = int(cached)
        keys = uncached_keys

    if not keys:
        return dem_by_key

    values: list[float]
    started_at = time.perf_counter()
    try:
        if callable(dem_lookup_many):
            values = dem_lookup_many(keys)
        else:
            values = terrain_elev_many(keys)
        if len(values) != len(keys):
            raise RuntimeError("terrain_elev_many returned unexpected length")
    except Exception:
        _record_filming_altitude_metrics(
            filmingTerrainBatchMs=(time.perf_counter() - started_at) * 1000.0,
            filmingTerrainBatchFallbackCount=1,
        )
        out: dict[tuple[float, float], int] = {}
        for key in keys:
            try:
                out[key] = int(round(float(terrain_elev(float(key[0]), float(key[1])))))
            except Exception:
                pass
        with _DEM_ALT_CACHE_LOCK:
            if len(_DEM_ALT_CACHE) + len(out) > _DEM_ALT_CACHE_MAX:
                _DEM_ALT_CACHE.clear()
            _DEM_ALT_CACHE.update({key: int(value) for key, value in out.items()})
        out.update(dem_by_key)
        return out
    _record_filming_altitude_metrics(
        filmingTerrainBatchMs=(time.perf_counter() - started_at) * 1000.0,
    )
    updates = {
        key: int(round(float(value)))
        for key, value in zip(keys, values)
    }
    if updates:
        with _DEM_ALT_CACHE_LOCK:
            if len(_DEM_ALT_CACHE) + len(updates) > _DEM_ALT_CACHE_MAX:
                _DEM_ALT_CACHE.clear()
            _DEM_ALT_CACHE.update(updates)
    dem_by_key.update(updates)
    return dem_by_key


def _normalize_target_coord_altitude(coord: Dict[str, Any]) -> int:
    dem_altitude = _dem_altitude_for_coord(coord)
    if dem_altitude is None:
        return 0
    previous = _to_float(coord.get("altitude"))
    if previous is not None and int(round(float(previous))) == int(dem_altitude):
        return 0
    coord["altitude"] = int(dem_altitude)
    return 1


def _normalize_target_coord_altitude_from_dem(coord: Dict[str, Any], dem_altitude: int | None) -> int:
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
    dem_lookup_many: Callable[[list[tuple[float, float]]], list[Any]] | None = None,
) -> int:
    """Keep filming target altitudes as DEM ground and keep WP altitude above them."""
    if not isinstance(waypoints, list):
        return 0

    changed = 0
    try:
        clearance = max(float(clearance_m), 0.0)
    except Exception:
        clearance = DEFAULT_FILMING_TARGET_CLEARANCE_M

    waypoint_targets: list[tuple[Dict[str, Any], List[Dict[str, Any]]]] = []
    all_target_coords: list[Dict[str, Any]] = []
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
        waypoint_targets.append((waypoint, target_coords))
        all_target_coords.extend(target_coords)

    if not all_target_coords:
        _record_filming_altitude_metrics(
            filmingTargetCoordCount=0,
            filmingUniqueCoordCount=0,
        )
        return 0

    unique_target_keys = {
        key
        for coord in all_target_coords
        for key in [_coord_key(coord)]
        if key is not None
    }
    _record_filming_altitude_metrics(
        filmingTargetCoordCount=len(all_target_coords),
        filmingUniqueCoordCount=len(unique_target_keys),
    )

    dem_by_key: dict[tuple[float, float], int] = {}
    use_batch = _batch_enabled(True)
    if use_batch:
        dem_by_key = _dem_altitudes_for_coords(
            all_target_coords,
            dem_lookup_many=dem_lookup_many,
        )

    for waypoint, target_coords in waypoint_targets:
        max_target_altitude: float | None = None
        for coord in target_coords:
            if use_batch:
                changed += _normalize_target_coord_altitude_from_dem(
                    coord,
                    dem_by_key.get(_coord_key(coord)),
                )
            else:
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


def _to_bounded_int(value: Any, *, default: int = 0, minimum: int = 0, maximum: int = _UINT32_MAX) -> int:
    try:
        number = int(value)
    except Exception:
        return int(default)
    if number < int(minimum) or number > int(maximum):
        return int(default)
    return int(number)


def normalize_lah_attack_blocks_in_waypoints(waypoints: List[Dict[str, Any]]) -> int:
    if not isinstance(waypoints, list):
        return 0

    changed = 0
    for waypoint in waypoints:
        if not isinstance(waypoint, dict):
            continue
        attack = waypoint.get("attack")
        if not isinstance(attack, dict):
            waypoint["attack"] = {"targetID": 0, "weaponType": 0}
            changed += 1
            continue

        target_id = _to_bounded_int(attack.get("targetID"), default=0)
        weapon_type = _to_bounded_int(attack.get("weaponType"), default=0, minimum=0, maximum=3)
        if attack.get("targetID") != target_id:
            attack["targetID"] = target_id
            changed += 1
        if attack.get("weaponType") != weapon_type:
            attack["weaponType"] = weapon_type
            changed += 1
        waypoint["attack"] = attack

    return changed


def normalize_filming_fov_in_waypoints(
    waypoints: List[Dict[str, Any]],
    *,
    minimum_fov_deg: float = DEFAULT_OUTPUT_MIN_FOV_DEG,
) -> int:
    if not isinstance(waypoints, list):
        return 0

    try:
        minimum_fov = float(minimum_fov_deg)
    except Exception:
        minimum_fov = DEFAULT_OUTPUT_MIN_FOV_DEG
    if minimum_fov <= 0.0:
        minimum_fov = DEFAULT_OUTPUT_MIN_FOV_DEG

    changed = 0
    for waypoint in waypoints:
        if not isinstance(waypoint, dict):
            continue
        filming = waypoint.get("filmingProperty")
        if not isinstance(filming, dict):
            continue
        fov = _to_float(filming.get("fieldOfView"))
        if fov is None or float(fov) >= float(minimum_fov):
            continue
        filming["fieldOfView"] = float(minimum_fov)
        waypoint["filmingProperty"] = filming
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
    changed += normalize_filming_fov_in_waypoints(waypoints)
    if isinstance(payload.get("lahWaypointList"), list):
        changed += normalize_lah_attack_blocks_in_waypoints(waypoints)
    if "waypointList" in payload:
        payload["waypointList"] = waypoints
    if isinstance(payload.get("lahWaypointList"), list):
        payload["lahWaypointList"] = deepcopy(waypoints)
    return int(changed)
