"""Operational enemy-contact concealment and UAV-relay route planner.

This production, GUI-free adapter keeps the source-resolution DEM cached, uses
live 0401 MSL altitudes, and returns only plain dictionaries suitable for the
attack-planning log and 0304 builders.

The tactical guarantee is deliberately explicit: the emitted 3-D route reaches
continuous terrain masking by the hide deadline, remains masked after that
point, and reaches an endpoint with the required UAV LOS by the reconnect
deadline. Terrain and dynamics are checked on the serialized route. UAV LOS
before the endpoint is not claimed.
"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
import math
from pathlib import Path
import threading
import time
from typing import Any, Iterable

import numpy as np

from modules.common.regional_dem import regional_dem_path_for_coordinate
from modules.common.terrain_los import TERRAIN_LOS_POLICY_VERSION
from modules.mission_planning.MissionPlanner.dynamics.lah_op_envlp import (
    DEFAULT_ENVELOPE,
)


_PROJECT_ROOT = Path(__file__).resolve().parents[3]
_RESOURCE_DIR = _PROJECT_ROOT / "resource"
_DEM_CACHE_LOCK = threading.RLock()
_DEM_CACHE_MAX = 2
_DEM_CACHE: "OrderedDict[tuple[str, int, int, int], _DemBundle]" = OrderedDict()
# Native LOS refinement allocates sizeable temporary arrays.  Descriptor
# builders may run four-way parallel, but at most two tactical analyses are
# allowed to compete for memory/CPU on the deployment PC.
_PLAN_SEMAPHORE = threading.BoundedSemaphore(2)
# Numerical margin applied when re-verifying concealment along the serialized
# route.  Paired with the refinement's endpoint selection margin.
_ROUTE_CONCEALMENT_VERIFY_MARGIN_M = 0.25


@dataclass(frozen=True)
class _DemBundle:
    path: Path
    coarse: Any
    native: Any
    load_elapsed_s: float


def _finite(value: Any) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return parsed if math.isfinite(parsed) else None


def _coordinate(value: Any) -> dict[str, float] | None:
    if not isinstance(value, dict):
        return None
    source = value.get("coordinate") if isinstance(value.get("coordinate"), dict) else value
    lat = _finite(source.get("latitude", source.get("lat")))
    lon = _finite(source.get("longitude", source.get("lon")))
    alt = _finite(source.get("altitude", source.get("alt")))
    if lat is None or lon is None:
        return None
    return {"latitude": lat, "longitude": lon, "altitude": alt}


def _aircraft_id(value: Any) -> int | None:
    if not isinstance(value, dict):
        return None
    try:
        return int(value.get("aircraft_id", value.get("aircraftID")))
    except (TypeError, ValueError, OverflowError):
        return None


def _dem_cache_key(path: Path, max_dim: int) -> tuple[str, int, int, int]:
    stat = path.stat()
    return (str(path.resolve()), int(stat.st_mtime_ns), int(stat.st_size), int(max_dim))


def _load_dem_bundle(path: Path, *, max_dim: int) -> tuple[_DemBundle, bool]:
    """Single-flight load of one coarse/native DEM pair."""

    from modules.monitoring.logic.dem_cover.dem import DemGrid
    from modules.monitoring.logic.dem_cover.dem_native import NativeDemRaster

    key = _dem_cache_key(path, max_dim)
    with _DEM_CACHE_LOCK:
        cached = _DEM_CACHE.get(key)
        if cached is not None:
            _DEM_CACHE.move_to_end(key)
            return cached, True
        started = time.perf_counter()
        bundle = _DemBundle(
            path=path.resolve(),
            coarse=DemGrid(path, max_dim=int(max_dim)),
            native=NativeDemRaster(path),
            load_elapsed_s=time.perf_counter() - started,
        )
        _DEM_CACHE[key] = bundle
        while len(_DEM_CACHE) > _DEM_CACHE_MAX:
            _DEM_CACHE.popitem(last=False)
        return bundle, False


def clear_enemy_contact_dem_cache() -> None:
    """Test/maintenance hook; normal replanning reuses the process cache."""

    with _DEM_CACHE_LOCK:
        _DEM_CACHE.clear()


def _failure(
    aircraft_id: int,
    role: str,
    started: float,
    code: str,
    detail: str,
    *,
    timings: dict[str, float] | None = None,
) -> dict[str, Any]:
    return {
        "applied": False,
        "aircraftID": int(aircraft_id),
        "role": str(role),
        "status": "no_route",
        "deadlineMet": False,
        "failureCodes": [str(code)],
        "detail": str(detail),
        "routeWaypoints": [],
        "timingMs": {
            **{key: round(float(value), 3) for key, value in (timings or {}).items()},
            "total": round((time.perf_counter() - started) * 1000.0, 3),
        },
    }


def _horizontal_reach_m(
    speed_mps: float,
    deadline_s: float,
    *,
    max_speed_mps: float,
    acceleration_mps2: float,
) -> float:
    speed = min(max(0.0, float(speed_mps)), float(max_speed_mps))
    acceleration = max(0.01, float(acceleration_mps2))
    remaining = max(0.0, float(deadline_s))
    accelerate_s = max(0.0, (float(max_speed_mps) - speed) / acceleration)
    powered_s = min(remaining, accelerate_s)
    return (
        speed * powered_s
        + 0.5 * acceleration * powered_s * powered_s
        + float(max_speed_mps) * max(0.0, remaining - powered_s)
    )


def _native_uav_points(
    dem: Any,
    states: Iterable[dict[str, Any]],
    *,
    validation_dem: Any | None = None,
) -> tuple[list[Any], list[int], list[dict[str, Any]]]:
    from modules.monitoring.logic.dem_cover.hide_com import UavPoint

    points: list[tuple[int, Any]] = []
    rejected: list[dict[str, Any]] = []
    for state in states or []:
        aircraft_id = _aircraft_id(state)
        coord = _coordinate(state)
        if aircraft_id is None or coord is None or coord.get("altitude") is None:
            rejected.append(
                {
                    "aircraftID": aircraft_id,
                    "reason": "MISSING_COORDINATE_OR_ALTITUDE",
                }
            )
            continue
        x, y = dem.latlon_to_native(coord["latitude"], coord["longitude"])
        if not dem.contains_native(x, y):
            rejected.append(
                {"aircraftID": int(aircraft_id), "reason": "OUTSIDE_DEM"}
            )
            continue
        terrain_dem = validation_dem or dem
        if not terrain_dem.contains_native(x, y):
            rejected.append(
                {"aircraftID": int(aircraft_id), "reason": "OUTSIDE_NATIVE_DEM"}
            )
            continue
        ground = float(terrain_dem.sample_native(x, y))
        altitude = float(coord["altitude"])
        if not math.isfinite(ground) or not math.isfinite(altitude) or altitude <= ground:
            rejected.append(
                {
                    "aircraftID": int(aircraft_id),
                    "reason": "ALTITUDE_NOT_ABOVE_NATIVE_TERRAIN",
                    "altitudeM": round(altitude, 3),
                    "groundM": round(ground, 3) if math.isfinite(ground) else None,
                }
            )
            continue
        row, col = dem.nearest_index(x, y)
        points.append(
            (
                int(aircraft_id),
                UavPoint(
                    x=float(x),
                    y=float(y),
                    row=float(row),
                    col=float(col),
                    ground_m=ground,
                    alt_m=altitude,
                    lat=float(coord["latitude"]),
                    lon=float(coord["longitude"]),
                ),
            )
        )
    # The operational contract is UAV4/5/6 where available.  Deterministic ID
    # ordering is preferable to choosing a different relay set per LAH.
    selected = sorted(points, key=lambda item: item[0])[:3]
    return (
        [point for _aircraft_id_value, point in selected],
        [int(aircraft_id_value) for aircraft_id_value, _point in selected],
        rejected,
    )


def _native_enemy_points(
    dem: Any,
    coordinates: Iterable[dict[str, Any]],
    *,
    enemy_height_m: float,
) -> list[Any]:
    from modules.monitoring.logic.dem_cover.sampling import EnemyPoint

    points: list[Any] = []
    seen: set[tuple[float, float]] = set()
    for raw in coordinates or []:
        coord = _coordinate(raw)
        if coord is None:
            continue
        key = (round(coord["latitude"], 7), round(coord["longitude"], 7))
        if key in seen:
            continue
        seen.add(key)
        x, y = dem.latlon_to_native(coord["latitude"], coord["longitude"])
        if not dem.contains_native(x, y):
            continue
        ground = float(dem.sample_native(x, y))
        if not math.isfinite(ground):
            continue
        row, col = dem.nearest_index(x, y)
        points.append(
            EnemyPoint(
                x=float(x),
                y=float(y),
                row=float(row),
                col=float(col),
                ground_m=ground,
                alt_m=ground + max(0.0, float(enemy_height_m)),
                lat=float(coord["latitude"]),
                lon=float(coord["longitude"]),
            )
        )
    return points


def _constant_acceleration_leg_time(
    left: Any,
    right: Any,
    distance_fraction: float,
) -> float:
    """Interpolate a waypoint-leg time from its endpoint speeds.

    Route generation propagates a constant-acceleration speed envelope.  A
    linear distance-to-time interpolation is wrong while accelerating or
    decelerating and can claim concealment several seconds too early.  This
    inverse kinematic interpolation preserves the serialized endpoint ETAs.
    """

    fraction = min(1.0, max(0.0, float(distance_fraction)))
    start_s = float(left.timestamp_s)
    duration_s = max(0.0, float(right.timestamp_s) - start_s)
    if duration_s <= 1.0e-9 or fraction <= 0.0:
        return start_s
    if fraction >= 1.0:
        return float(right.timestamp_s)

    try:
        start_speed = float(getattr(left, "speed_mps"))
        end_speed = float(getattr(right, "speed_mps"))
    except (AttributeError, TypeError, ValueError):
        # Older route/test objects predate serialized endpoint speeds.  Keep
        # their established linear ETA semantics instead of failing the
        # entire tactical-cover validation path.
        return start_s + (fraction * duration_s)
    if not math.isfinite(start_speed) or not math.isfinite(end_speed):
        return start_s + (fraction * duration_s)
    start_speed = max(0.0, start_speed)
    end_speed = max(0.0, end_speed)
    acceleration = (end_speed - start_speed) / duration_s
    integrated_distance = 0.5 * (start_speed + end_speed) * duration_s
    if integrated_distance <= 1.0e-9:
        return start_s + (fraction * duration_s)
    target_distance = fraction * integrated_distance
    if abs(acceleration) <= 1.0e-9:
        elapsed_s = target_distance / max(start_speed, 1.0e-9)
    else:
        discriminant = max(
            0.0,
            (start_speed * start_speed)
            + (2.0 * acceleration * target_distance),
        )
        elapsed_s = (-start_speed + math.sqrt(discriminant)) / acceleration
    if not math.isfinite(elapsed_s):
        elapsed_s = fraction * duration_s
    return start_s + min(duration_s, max(0.0, float(elapsed_s)))


def _serialized_route_samples(
    waypoints: Iterable[Any],
    *,
    cell_m: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Densely sample the exact straight chords emitted to the flight path."""

    route = list(waypoints or [])
    if not route:
        empty = np.asarray([], dtype=np.float64)
        return empty, empty, empty, empty
    spacing_m = max(1.0, float(cell_m) * 0.5)
    xs: list[float] = []
    ys: list[float] = []
    altitudes: list[float] = []
    times: list[float] = []
    if len(route) == 1:
        waypoint = route[0]
        return (
            np.asarray([float(waypoint.x)], dtype=np.float64),
            np.asarray([float(waypoint.y)], dtype=np.float64),
            np.asarray([float(waypoint.alt_m)], dtype=np.float64),
            np.asarray([float(waypoint.timestamp_s)], dtype=np.float64),
        )
    for leg_index, (left, right) in enumerate(zip(route, route[1:])):
        x0, y0 = float(left.x), float(left.y)
        x1, y1 = float(right.x), float(right.y)
        horizontal_m = math.hypot(x1 - x0, y1 - y0)
        sample_count = max(1, int(math.ceil(horizontal_m / spacing_m)))
        fractions = np.linspace(0.0, 1.0, sample_count + 1, dtype=np.float64)
        if leg_index > 0:
            fractions = fractions[1:]
        for fraction in fractions:
            ratio = float(fraction)
            xs.append(x0 + ratio * (x1 - x0))
            ys.append(y0 + ratio * (y1 - y0))
            altitudes.append(
                float(left.alt_m) + ratio * (float(right.alt_m) - float(left.alt_m))
            )
            times.append(_constant_acceleration_leg_time(left, right, ratio))
    return tuple(
        np.asarray(values, dtype=np.float64)
        for values in (xs, ys, altitudes, times)
    )


def _route_concealment_schedule(
    dem: Any,
    config: Any,
    enemies: Iterable[Any],
    candidate: Any,
    *,
    hide_deadline_s: float,
    planning_elapsed_s: float,
) -> dict[str, Any]:
    """Prove that a serialized route hides by the deadline and stays hidden."""

    from modules.monitoring.logic.dem_cover.hide_com_refine import (
        _fractional_cell,
        _observer_requirements_chunked,
    )

    xs, ys, altitudes, route_times = _serialized_route_samples(
        candidate.waypoints,
        cell_m=float(dem.cell_m),
    )
    if not xs.size or not (
        xs.size == ys.size == altitudes.size == route_times.size
    ):
        return {
            "valid": False,
            "failureCode": "EMPTY_SERIALIZED_ROUTE_PROFILE",
            "sampleCount": int(xs.size),
        }
    if not np.all(np.isfinite(xs)) or not np.all(np.isfinite(ys)):
        return {
            "valid": False,
            "failureCode": "NONFINITE_SERIALIZED_ROUTE_PROFILE",
            "sampleCount": int(xs.size),
        }
    rows_and_cols = [_fractional_cell(dem, float(x), float(y)) for x, y in zip(xs, ys)]
    rows = np.asarray([value[0] for value in rows_and_cols], dtype=np.float64)
    cols = np.asarray([value[1] for value in rows_and_cols], dtype=np.float64)
    native_step_cap = int(
        math.ceil(
            (int(dem.height) + int(dem.width))
            * max(1.0, float(config.los_samples_per_cell))
        )
    ) + 2
    precise_config = config.with_overrides(
        los_samples_per_cell=max(1.0, float(config.los_samples_per_cell)),
        los_max_steps=max(int(config.los_max_steps), native_step_cap),
    )
    bounded_enemies = list(enemies or [])[: max(1, int(precise_config.max_analysis_enemies))]
    if not bounded_enemies:
        return {
            "valid": False,
            "failureCode": "NO_ENEMY_FOR_ROUTE_VALIDATION",
            "sampleCount": int(xs.size),
        }
    # -inf so an enemy whose ray could not be traced collapses the ceiling and
    # marks the sample exposed, instead of dropping out of the np.min below.
    thresholds = np.full(
        (len(bounded_enemies), int(xs.size)),
        -np.inf,
        dtype=np.float64,
    )
    for index, enemy in enumerate(bounded_enemies):
        thresholds[index] = _observer_requirements_chunked(
            dem,
            precise_config,
            enemy,
            rows=rows,
            cols=cols,
            xs=xs,
            ys=ys,
            max_distance_m=precise_config.enemy_observation_range_effective_m,
            chunk_size=64,
            reject_nodata=True,
            unevaluated_fill_m=-np.inf,
        )
    # The LOS threshold is open on the concealed side, so the same numerical
    # margin the native endpoint refinement applies is retained here.  The
    # refinement must keep its own selection ceiling strictly below this one
    # (see _ENEMY_CEILING_SELECTION_MARGIN_M): the two stages recompute the
    # threshold over different sample batches, so an endpoint chosen exactly at
    # the ceiling is rejected here and no concealment plan is ever issued.
    hidden_ceiling_m = (
        np.min(thresholds, axis=0)
        - float(precise_config.hide_safety_margin_m)
        - _ROUTE_CONCEALMENT_VERIFY_MARGIN_M
    )
    hidden = np.isfinite(altitudes) & (altitudes <= hidden_ceiling_m)
    continuous_hidden_suffix = np.logical_and.accumulate(hidden[::-1])[::-1]
    response_times = route_times + max(0.0, float(planning_elapsed_s))
    reachable = continuous_hidden_suffix & np.isfinite(response_times)
    eligible = np.nonzero(
        reachable & (response_times <= float(hide_deadline_s) + 1.0e-9)
    )[0]
    hide_deadline_met = bool(eligible.size > 0)
    if not hide_deadline_met:
        # Cover that takes longer than the deadline is still cover.  Rejecting
        # it outright leaves the aircraft in the open with no plan at all, which
        # is strictly worse; take the soonest concealed sample and report that
        # the deadline was missed.
        eligible = np.nonzero(reachable)[0]
    if eligible.size == 0:
        return {
            "valid": False,
            # The route genuinely never reaches sustained concealment - not a
            # timing miss.
            "failureCode": "ROUTE_NEVER_REACHES_CONCEALMENT",
            "sampleCount": int(xs.size),
            "hiddenSampleCount": int(np.count_nonzero(hidden)),
        }
    selected_index = int(eligible[0])
    return {
        "valid": True,
        "failureCode": None,
        "sampleCount": int(xs.size),
        "hiddenSampleCount": int(np.count_nonzero(hidden)),
        "hideAchievedRouteS": round(float(route_times[selected_index]), 4),
        "hideAchievedS": round(float(response_times[selected_index]), 4),
        "continuousAfterHide": True,
        "hideDeadlineMet": bool(hide_deadline_met),
    }


def _candidate_endpoint_status(candidate: Any) -> tuple[int, int, list[int], int] | None:
    validator = getattr(candidate.endpoint, "canonical_los_validator", None)
    if not callable(validator):
        return None
    try:
        status = validator(float(candidate.arrival_altitude_m))
        return int(status[0]), int(status[1]), list(status[2]), int(status[3])
    except Exception:
        return None


def _select_operational_route(
    route_result: Any,
    dem: Any,
    config: Any,
    enemies: Iterable[Any],
    *,
    required_links: int,
    hide_deadline_s: float,
    reconnect_deadline_s: float,
    planning_elapsed_s: float,
    allow_hide_then_reconnect: bool,
) -> tuple[Any | None, dict[str, Any] | None, tuple[int, int, list[int], int] | None, list[str]]:
    """Select only a route satisfying hide<=10 and reconnect<=60 semantics."""

    response_limit_s = (
        float(reconnect_deadline_s)
        if allow_hide_then_reconnect
        else float(hide_deadline_s)
    )
    route_limit_s = max(0.0, response_limit_s - max(0.0, float(planning_elapsed_s)))
    candidates = sorted(
        (
            candidate
            for candidate in (route_result.candidates or [])
            if bool(candidate.route_constraints_valid)
            and math.isfinite(float(candidate.eta_s))
            and float(candidate.eta_s) <= route_limit_s + 1.0e-9
        ),
        key=lambda candidate: (
            float(candidate.eta_s),
            float(candidate.max_altitude_m),
            float(candidate.distance_m),
            int(candidate.endpoint_index),
            str(candidate.kind),
        ),
    )
    rejection_codes: list[str] = []
    for candidate in candidates:
        endpoint_status = _candidate_endpoint_status(candidate)
        if endpoint_status is None:
            rejection_codes.append("ENDPOINT_CANONICAL_LOS_VALIDATION_ERROR")
            continue
        enemy_visible, _enemy_in_range, _connected, uav_links = endpoint_status
        if enemy_visible != 0:
            rejection_codes.append("ENDPOINT_CANONICAL_ENEMY_LOS_FAILED")
            continue
        if uav_links < int(required_links):
            rejection_codes.append("ENDPOINT_CANONICAL_UAV_LOS_FAILED")
            continue
        schedule = _route_concealment_schedule(
            dem,
            config,
            enemies,
            candidate,
            hide_deadline_s=float(hide_deadline_s),
            planning_elapsed_s=float(planning_elapsed_s),
        )
        if not schedule.get("valid"):
            rejection_codes.append(str(schedule.get("failureCode") or "ROUTE_CONCEALMENT_FAILED"))
            continue
        return candidate, schedule, endpoint_status, list(dict.fromkeys(rejection_codes))
    if not candidates:
        rejection_codes.append("NO_ROUTE_WITHIN_RESPONSE_WINDOW")
    return None, None, None, list(dict.fromkeys(rejection_codes))


def _plan_enemy_contact_response_unbounded(
    aircraft_id: int,
    own_state: dict[str, Any],
    uav_states: Iterable[dict[str, Any]],
    enemy_coordinates: Iterable[dict[str, Any]],
    *,
    role: str,
    deadline_s: float = 10.0,
    reconnect_deadline_s: float = 60.0,
    enemy_range_m: float = 5_000.0,
    # Concealment gate, deliberately separate from the friendly weapon range
    # above.  <= 0 means unlimited: every detected enemy constrains the hide
    # point no matter how far away it is.
    enemy_observation_range_m: float = 0.0,
    communication_range_m: float = 10_000.0,
    min_uav_links: int = 3,
    hide_agl_m: float = 50.0,
    max_hide_agl_m: float = 1_000.0,
    analysis_max_dim: int = 340,
    refinement_radius_m: float = 300.0,
    max_precision_candidates: int = 24,
    allow_best_effort: bool = True,
    degraded_min_uav_links: int | None = None,
    timing_guard_s: float = 1.0,
    prefer_agl_target_m: float | None = 150.0,
    vertical_rate_derate: float = 0.75,
    # Let the hide point know which enemy has to be engaged from it and how
    # high the airframe may climb. Concealment that cannot be fired from is
    # not useful cover; both default to None, which keeps the legacy ranking.
    attack_target_coordinate: dict[str, Any] | None = None,
    attack_ceiling_m: float | None = None,
    _wall_started: float | None = None,
    _queue_wait_ms: float = 0.0,
) -> dict[str, Any]:
    """Plan one LAH response from the latest live state.

    ``role`` is ``relay`` for LAH1 and ``attacker`` for LAH2/3.  A strict LOS
    endpoint is mandatory. A route whose joint endpoint is later than 10 s is
    accepted only if dense native-DEM validation proves concealment by 10 s,
    no re-exposure after concealment, and UAV reconnection by 60 s.
    """

    started = (
        float(_wall_started)
        if _wall_started is not None and math.isfinite(float(_wall_started))
        else time.perf_counter()
    )
    timings: dict[str, float] = {
        "queueWait": max(0.0, float(_queue_wait_ms)),
    }
    aid = int(aircraft_id)
    role_name = str(role or "attacker")
    uav_inputs = list(uav_states or [])
    enemy_inputs = list(enemy_coordinates or [])
    supplied_enemy_keys: set[tuple[float, float]] = set()
    invalid_enemy_input_count = 0
    for raw_enemy in enemy_inputs:
        normalized_enemy = _coordinate(raw_enemy)
        if normalized_enemy is None:
            invalid_enemy_input_count += 1
            continue
        supplied_enemy_keys.add(
            (
                round(float(normalized_enemy["latitude"]), 7),
                round(float(normalized_enemy["longitude"]), 7),
            )
        )
    own_coord = _coordinate(own_state)
    speed = _finite((own_state or {}).get("speed"))
    heading = _finite((own_state or {}).get("heading"))
    if own_coord is None or own_coord.get("altitude") is None:
        return _failure(aid, role_name, started, "MISSING_OWN_COORDINATE", "live MSL coordinate unavailable")
    if speed is None or speed < 0.0:
        return _failure(aid, role_name, started, "MISSING_OWN_SPEED", "live 0401 speed unavailable")
    if heading is None:
        return _failure(aid, role_name, started, "MISSING_OWN_HEADING", "live 0401 heading unavailable")
    if not math.isfinite(float(deadline_s)) or float(deadline_s) <= 0.0:
        return _failure(aid, role_name, started, "INVALID_DEADLINE", str(deadline_s))

    dem_path = regional_dem_path_for_coordinate(
        _RESOURCE_DIR,
        own_coord["latitude"],
        own_coord["longitude"],
    )
    if dem_path is None or not Path(dem_path).is_file():
        return _failure(aid, role_name, started, "DEM_UNAVAILABLE", "no regional DEM covers the live position")

    try:
        load_started = time.perf_counter()
        bundle, cache_hit = _load_dem_bundle(Path(dem_path), max_dim=int(analysis_max_dim))
        timings["demLoad"] = (time.perf_counter() - load_started) * 1000.0
        coarse_dem = bundle.coarse
        native_dem = bundle.native

        own_x, own_y = coarse_dem.latlon_to_native(
            own_coord["latitude"], own_coord["longitude"]
        )
        if not coarse_dem.contains_native(own_x, own_y):
            return _failure(aid, role_name, started, "OWN_OUTSIDE_DEM", str(bundle.path), timings=timings)
        own_ground = float(coarse_dem.sample_native(own_x, own_y))
        if not math.isfinite(own_ground) or float(own_coord["altitude"]) <= own_ground:
            return _failure(
                aid,
                role_name,
                started,
                "OWN_ALTITUDE_BELOW_TERRAIN",
                f"alt={own_coord['altitude']} ground={own_ground}",
                timings=timings,
            )

        requested_links = max(1, min(3, int(min_uav_links)))
        uavs, accepted_uav_ids, rejected_uavs = _native_uav_points(
            coarse_dem,
            uav_inputs,
            validation_dem=native_dem,
        )
        if not uavs:
            return _failure(
                aid,
                role_name,
                started,
                "NO_VALID_UAV_STATE",
                f"rejected={rejected_uavs}",
                timings=timings,
            )
        required_links = int(requested_links)
        degraded_links = False
        if len(uavs) < requested_links:
            degraded_floor = (
                max(1, min(int(requested_links), int(degraded_min_uav_links)))
                if degraded_min_uav_links is not None
                else int(requested_links)
            )
            if len(uavs) < degraded_floor:
                return _failure(
                    aid,
                    role_name,
                    started,
                    "INSUFFICIENT_VALID_UAV_STATES",
                    (
                        f"required={requested_links} degradedFloor={degraded_floor} "
                        f"valid={len(uavs)} accepted={accepted_uav_ids} "
                        f"rejected={rejected_uavs}"
                    ),
                    timings=timings,
                )
            required_links = len(uavs)
            degraded_links = True

        from modules.monitoring.logic.dem_cover.config import CoverConfig
        from modules.monitoring.logic.dem_cover.hide_com import CommunicationHideAnalyzer
        from modules.monitoring.logic.dem_cover.hide_com_refine import refine_communication_hide
        from modules.monitoring.logic.dem_cover.hide_com_route import (
            RouteDynamics,
            plan_hide_communication_routes,
        )

        config = CoverConfig(
            dem_path=str(bundle.path),
            analysis_max_dim=int(analysis_max_dim),
            # ``enemy_range_m`` is the friendly engagement range and stays where
            # firing feasibility is judged.  Concealment is proven against every
            # detected enemy regardless of range: capping it deletes enemies
            # from the proof and reports cover for a point in plain view.
            weapon_range_m=max(1.0, float(enemy_range_m)),
            enemy_observation_range_m=max(0.0, float(enemy_observation_range_m or 0.0)),
        )
        enemies = _native_enemy_points(
            coarse_dem,
            enemy_inputs,
            enemy_height_m=float(config.enemy_height_m),
        )
        if not enemies:
            return _failure(aid, role_name, started, "NO_VALID_ENEMY", "no supplied target lies on the selected DEM", timings=timings)
        if invalid_enemy_input_count or len(enemies) != len(supplied_enemy_keys):
            return _failure(
                aid,
                role_name,
                started,
                "ENEMY_SET_INCOMPLETE",
                (
                    f"input={len(enemy_inputs)} unique={len(supplied_enemy_keys)} "
                    f"validOnDem={len(enemies)} invalid={invalid_enemy_input_count}"
                ),
                timings=timings,
            )
        if len(enemies) > max(1, int(config.max_analysis_enemies)):
            return _failure(
                aid,
                role_name,
                started,
                "ENEMY_ANALYSIS_LIMIT_EXCEEDED",
                f"count={len(enemies)} limit={int(config.max_analysis_enemies)}",
                timings=timings,
            )

        max_speed = float(DEFAULT_ENVELOPE.max_speed_kmh) / 3.6
        # The same derate every other LAH vertical-rate consumer applies.
        vertical_derate = max(0.05, min(1.0, float(vertical_rate_derate)))
        climb_rate_mps = float(DEFAULT_ENVELOPE.climb_rate_mps) * vertical_derate
        descent_rate_mps = float(DEFAULT_ENVELOPE.descent_rate_mps) * vertical_derate
        timing_guard = max(0.0, float(timing_guard_s))
        elapsed_before_search_s = (time.perf_counter() - started) + timing_guard
        remaining_hide_s = float(deadline_s) - elapsed_before_search_s
        remaining_reconnect_s = float(reconnect_deadline_s) - elapsed_before_search_s
        if remaining_hide_s <= 0.0:
            return _failure(
                aid,
                role_name,
                started,
                "PLANNING_DEADLINE_EXHAUSTED",
                f"elapsed={elapsed_before_search_s:.3f}s deadline={deadline_s:.3f}s",
                timings=timings,
            )
        hide_reach_m = _horizontal_reach_m(
            speed,
            remaining_hide_s,
            max_speed_mps=max_speed,
            acceleration_mps2=float(DEFAULT_ENVELOPE.max_accel_mps2),
        )
        reconnect_reach_m = _horizontal_reach_m(
            speed,
            max(0.0, remaining_reconnect_s),
            max_speed_mps=max_speed,
            acceleration_mps2=float(DEFAULT_ENVELOPE.max_accel_mps2),
        )
        endpoint_reach_m = (
            reconnect_reach_m if bool(allow_best_effort) else hide_reach_m
        )
        search_radius_m = max(float(coarse_dem.cell_m), endpoint_reach_m)

        coarse_started = time.perf_counter()
        coarse_result = CommunicationHideAnalyzer(coarse_dem, config).analyze(
            own_x=float(own_x),
            own_y=float(own_y),
            own_altitude_m=float(own_coord["altitude"]),
            uavs=uavs,
            enemies=enemies,
            hide_agl_m=float(hide_agl_m),
            max_hide_agl_m=float(max_hide_agl_m),
            search_radius_m=float(search_radius_m),
            communication_range_m=float(communication_range_m),
            min_uav_links=int(required_links),
        )
        timings["coarseLos"] = (time.perf_counter() - coarse_started) * 1000.0

        refine_started = time.perf_counter()
        refined = refine_communication_hide(
            coarse_result,
            native_dem,
            config,
            uav_agl_m=None,
            hide_agl_m=float(hide_agl_m),
            max_hide_agl_m=float(max_hide_agl_m),
            search_radius_m=float(search_radius_m),
            communication_range_m=float(communication_range_m),
            min_uav_links=int(required_links),
            refinement_radius_m=min(float(refinement_radius_m), float(search_radius_m)),
            own_reachable_radius_m=float(hide_reach_m),
            chunk_size=64,
            max_precision_candidates=max(1, int(max_precision_candidates)),
            preferred_heading_deg=float(heading) % 360.0,
            current_speed_mps=float(speed),
            max_turn_rate_deg_s=float(DEFAULT_ENVELOPE.max_turn_rate_dps),
            prefer_agl_target_m=(
                None if prefer_agl_target_m is None else float(prefer_agl_target_m)
            ),
            descent_rate_mps=descent_rate_mps,
            climb_rate_mps=climb_rate_mps,
            max_speed_mps=max_speed,
            attack_target_coordinate=attack_target_coordinate,
            attack_ceiling_m=attack_ceiling_m,
        )
        timings["nativeLos"] = (time.perf_counter() - refine_started) * 1000.0
        if not refined.precision_candidates:
            return _failure(
                aid,
                role_name,
                started,
                "NO_STRICT_HIDE_COMM_ENDPOINT",
                "; ".join(refined.notes[-3:]),
                timings=timings,
            )

        dynamics = RouteDynamics(
            max_speed_mps=max_speed,
            acceleration_mps2=float(DEFAULT_ENVELOPE.max_accel_mps2),
            deceleration_mps2=float(DEFAULT_ENVELOPE.max_accel_mps2),
            climb_rate_mps=climb_rate_mps,
            descent_rate_mps=descent_rate_mps,
            max_turn_rate_deg_s=float(DEFAULT_ENVELOPE.max_turn_rate_dps),
            max_bank_deg=float(DEFAULT_ENVELOPE.pitch_roll_limit_deg),
            max_altitude_m=float(DEFAULT_ENVELOPE.max_altitude_m),
            clearance_m=float(hide_agl_m),
            max_endpoint_candidates=min(12, max(1, int(max_precision_candidates))),
        )
        route_started = time.perf_counter()
        route_response_limit_s = (
            float(reconnect_deadline_s)
            if bool(allow_best_effort)
            else float(deadline_s)
        )
        route_budget_s = max(
            0.001,
            route_response_limit_s - (time.perf_counter() - started),
        )
        route = plan_hide_communication_routes(
            native_dem,
            float(own_x),
            float(own_y),
            float(own_coord["altitude"]),
            float(speed),
            float(heading) % 360.0,
            refined.precision_candidates,
            deadline_s=float(route_budget_s),
            dynamics=dynamics,
        )
        timings["routeSearch"] = (time.perf_counter() - route_started) * 1000.0
        planning_elapsed_s = (time.perf_counter() - started) + timing_guard
        selected, concealment, endpoint_status, candidate_rejections = _select_operational_route(
            route,
            native_dem,
            config,
            refined.enemies,
            required_links=int(required_links),
            hide_deadline_s=float(deadline_s),
            reconnect_deadline_s=float(reconnect_deadline_s),
            planning_elapsed_s=float(planning_elapsed_s),
            allow_hide_then_reconnect=bool(allow_best_effort),
        )
        timings["routeConcealmentValidation"] = (
            time.perf_counter() - route_started
        ) * 1000.0 - timings["routeSearch"]
        if selected is None or concealment is None or endpoint_status is None:
            return _failure(
                aid,
                role_name,
                started,
                "NO_HIDE_THEN_RECONNECT_ROUTE",
                ",".join(
                    list(dict.fromkeys(list(route.failure_codes) + candidate_rejections))
                ),
                timings=timings,
            )

        route_waypoints = [
            {
                "latitude": round(float(waypoint.lat), 7),
                "longitude": round(float(waypoint.lon), 7),
                "altitude": round(float(waypoint.alt_m), 3),
                "etaS": round(float(waypoint.timestamp_s), 4),
                "speedMps": round(float(waypoint.speed_mps), 4),
                "groundM": round(float(waypoint.ground_m), 3),
                "distanceM": round(float(waypoint.distance_m), 3),
            }
            for waypoint in selected.waypoints
        ]
        endpoint = route_waypoints[-1]
        final_planning_elapsed_s = (time.perf_counter() - started) + timing_guard
        hide_achieved_response_s = final_planning_elapsed_s + float(
            concealment.get("hideAchievedRouteS") or 0.0
        )
        response_eta_s = final_planning_elapsed_s + float(selected.eta_s)
        hide_deadline_met = bool(concealment.get("hideDeadlineMet", True)) and (
            hide_achieved_response_s <= float(deadline_s) + 1.0e-9
        )
        reconnect_deadline_met = response_eta_s <= float(reconnect_deadline_s) + 1.0e-9
        # Missing a deadline is late cover, not no cover.  Only a route that
        # never conceals at all is a failure - discarding a slow one leaves the
        # aircraft in the open with no plan, which is strictly worse.  The
        # timings are reported so the operator can see the delay.
        endpoint_enemy_visible = int(endpoint_status[0])
        endpoint_uav_links = int(endpoint_status[3])
        joint_endpoint_deadline_met = response_eta_s <= float(deadline_s) + 1.0e-9
        base_status = "green_valid" if joint_endpoint_deadline_met else "hide_then_reconnect"
        status = f"degraded_{base_status}" if degraded_links else base_status
        return {
            "applied": True,
            "aircraftID": aid,
            "role": role_name,
            "status": status,
            "deadlineMet": bool(hide_deadline_met and reconnect_deadline_met),
            "hideDeadlineMet": bool(hide_deadline_met),
            "reconnectDeadlineMetActual": bool(reconnect_deadline_met),
            "jointEndpointDeadlineMet": bool(joint_endpoint_deadline_met),
            "reconnectDeadlineMet": True,
            "deadlineS": round(float(deadline_s), 3),
            "reconnectDeadlineS": round(float(reconnect_deadline_s), 3),
            "etaS": round(float(selected.eta_s), 4),
            "responseEtaS": round(float(response_eta_s), 4),
            "hideAchievedS": round(float(hide_achieved_response_s), 4),
            "hideAchievedRouteS": round(
                float(concealment.get("hideAchievedRouteS") or 0.0), 4
            ),
            "timingGuardS": round(float(timing_guard), 3),
            "endpoint": dict(endpoint),
            "routeWaypoints": route_waypoints,
            "enemyVisibleCount": endpoint_enemy_visible,
            "uavLinkCount": endpoint_uav_links,
            "requiredUavLinks": int(required_links),
            "requestedUavLinks": int(requested_links),
            "degradedUavCoverage": bool(degraded_links),
            "usedUavIDs": list(accepted_uav_ids),
            "rejectedUavs": list(rejected_uavs),
            "demPath": str(bundle.path),
            "losPolicyVersion": TERRAIN_LOS_POLICY_VERSION,
            "demCacheHit": bool(cache_hit),
            "demNativeCellM": round(float(native_dem.cell_m), 3),
            "horizontalReachM": round(float(hide_reach_m), 3),
            "hideHorizontalReachM": round(float(hide_reach_m), 3),
            "reconnectHorizontalReachM": round(float(reconnect_reach_m), 3),
            "failureCodes": [],
            "warningCodes": (
                ["DEGRADED_UAV_LINK_REQUIREMENT"] if degraded_links else []
            ),
            "candidateRejectionCodes": list(candidate_rejections),
            "enemyInputCount": len(enemy_inputs),
            "enemyAnalyzedCount": len(refined.enemies),
            "concealmentValidation": dict(concealment),
            "guarantees": {
                "hideByDeadline": True,
                "continuousEnemyMaskingAfterHide": bool(
                    concealment.get("continuousAfterHide")
                ),
                "endpointEnemyMasked": endpoint_enemy_visible == 0,
                "endpointUavLos": endpoint_uav_links >= int(required_links),
                "requestedUavCoverageMet": endpoint_uav_links >= int(requested_links),
                "routeTerrainSafe": bool(selected.terrain_safe),
                "routeDynamicsFeasible": bool(selected.dynamics_feasible),
                "staticUavSnapshot": True,
                "continuousIngressEnemyMaskingClaimed": False,
                "continuousIngressUavLosClaimed": False,
            },
            "timingMs": {
                **{key: round(float(value), 3) for key, value in timings.items()},
                "total": round((time.perf_counter() - started) * 1000.0, 3),
            },
        }
    except Exception as exc:
        return _failure(
            aid,
            role_name,
            started,
            "TACTICAL_PLANNER_EXCEPTION",
            f"{type(exc).__name__}: {exc}",
            timings=timings,
        )


def plan_enemy_contact_response(*args: Any, **kwargs: Any) -> dict[str, Any]:
    """Bounded-concurrency public wrapper around the native planner."""

    wall_started = time.perf_counter()
    queue_started = time.perf_counter()
    with _PLAN_SEMAPHORE:
        kwargs.setdefault("_wall_started", wall_started)
        kwargs.setdefault(
            "_queue_wait_ms",
            (time.perf_counter() - queue_started) * 1000.0,
        )
        return _plan_enemy_contact_response_unbounded(*args, **kwargs)


__all__ = [
    "clear_enemy_contact_dem_cache",
    "plan_enemy_contact_response",
]
