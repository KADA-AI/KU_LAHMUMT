from __future__ import annotations

import math
from typing import Any, Callable, Dict, Iterable, List, Sequence, Tuple

from modules.common.terrain_los import (
    EFFECTIVE_EARTH_RADIUS_M,
    ENEMY_OBSERVER_HEIGHT_M,
    LOS_CLEARANCE_M,
)


EARTH_RADIUS_M = 6_371_000.0
DEFAULT_PROFILE_STEP_M = 10.0
MAX_PROFILE_SAMPLES = 1024


def _to_float(value: Any) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _lat_lon(coord: Dict[str, Any]) -> Tuple[float, float] | None:
    if not isinstance(coord, dict):
        return None
    lat = _to_float(coord.get("latitude", coord.get("lat")))
    lon = _to_float(coord.get("longitude", coord.get("lon")))
    if lat is None or lon is None:
        return None
    return float(lat), float(lon)


def haversine_distance_m(left: Dict[str, Any], right: Dict[str, Any]) -> float | None:
    left_ll = _lat_lon(left)
    right_ll = _lat_lon(right)
    if left_ll is None or right_ll is None:
        return None
    lat1, lon1 = left_ll
    lat2, lon2 = right_ll
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lon2 - lon1)
    term = (
        math.sin(d_phi * 0.5) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda * 0.5) ** 2
    )
    return 2.0 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(max(0.0, term))))


def terrain_profile_coordinates(
    enemy_coord: Dict[str, Any],
    attack_coord: Dict[str, Any],
    *,
    sample_step_m: float = DEFAULT_PROFILE_STEP_M,
    max_samples: int = MAX_PROFILE_SAMPLES,
) -> Tuple[List[Tuple[float, float]], float]:
    enemy_ll = _lat_lon(enemy_coord)
    attack_ll = _lat_lon(attack_coord)
    distance_m = haversine_distance_m(enemy_coord, attack_coord)
    if enemy_ll is None or attack_ll is None or distance_m is None or distance_m <= 0.0:
        return [], 0.0
    step_m = _to_float(sample_step_m)
    if step_m is None or step_m <= 0.0:
        step_m = DEFAULT_PROFILE_STEP_M
    sample_cap = max(3, min(65536, int(max_samples)))
    sample_count = max(3, int(math.ceil(float(distance_m) / float(step_m))) + 1)
    sample_count = min(sample_cap, sample_count)
    enemy_lat, enemy_lon = enemy_ll
    attack_lat, attack_lon = attack_ll
    pairs = [
        (
            float(enemy_lat) + (float(attack_lat) - float(enemy_lat)) * idx / (sample_count - 1),
            float(enemy_lon) + (float(attack_lon) - float(enemy_lon)) * idx / (sample_count - 1),
        )
        for idx in range(sample_count)
    ]
    return pairs, float(distance_m)


def solve_minimum_attack_altitude(
    terrain_elevations_m: Sequence[Any],
    distance_m: float,
    *,
    target_height_m: float = ENEMY_OBSERVER_HEIGHT_M,
    clearance_m: float = LOS_CLEARANCE_M,
) -> Dict[str, Any]:
    terrain: List[float] = []
    for value in terrain_elevations_m or []:
        parsed = _to_float(value)
        if parsed is None:
            return {"verified": False, "reason": "terrain_profile_contains_nodata"}
        terrain.append(float(parsed))
    if len(terrain) < 3 or not math.isfinite(float(distance_m)) or float(distance_m) <= 0.0:
        return {"verified": False, "reason": "terrain_profile_too_short"}

    target_height = max(0.0, float(target_height_m))
    clearance = max(0.0, float(clearance_m))
    target_ground_m = float(terrain[0])
    attack_ground_m = float(terrain[-1])
    target_altitude_m = target_ground_m + target_height
    denominator = float(len(terrain) - 1)
    # Match modules.common.terrain_los exactly.  Positive clearance is a
    # conservative *masking* margin there: terrain blocks only when it rises
    # above ``ray + clearance``.  The old attack solver added clearance to the
    # terrain and also forced the aircraft to be no lower than the target.
    # Both choices inverted the shared policy and could turn a clear uphill ray
    # into a spurious multi-kilometre climb.
    required_altitude_m = float(attack_ground_m + clearance + 1.0e-3)
    controlling_index = len(terrain) - 1
    for idx in range(1, len(terrain) - 1):
        fraction = float(idx) / denominator
        along_m = fraction * float(distance_m)
        effective_terrain_m = float(terrain[idx]) - (
            along_m * along_m / (2.0 * float(EFFECTIVE_EARTH_RADIUS_M))
        )
        endpoint_requirement_m = target_altitude_m + (
            effective_terrain_m - clearance - target_altitude_m
        ) / fraction
        if endpoint_requirement_m > required_altitude_m:
            required_altitude_m = float(endpoint_requirement_m)
            controlling_index = int(idx)

    return {
        "verified": True,
        "required_altitude_m": float(required_altitude_m),
        "distance_m": float(distance_m),
        "sample_count": int(len(terrain)),
        "sample_step_m": float(distance_m) / denominator,
        "target_ground_m": target_ground_m,
        "attack_ground_m": attack_ground_m,
        "target_height_m": target_height,
        "clearance_m": clearance,
        "controlling_distance_m": float(distance_m) * controlling_index / denominator,
        "controlling_terrain_m": float(terrain[controlling_index]),
        "earth_curvature_included": True,
        "earth_curvature_model": "shared_4_3_earth_source_tangent",
    }


def profile_with_batch_dem(
    attack_coord: Dict[str, Any],
    enemy_coord: Dict[str, Any],
    terrain_lookup_many: Callable[[Iterable[Tuple[float, float]]], Sequence[Any]],
    *,
    target_height_m: float = ENEMY_OBSERVER_HEIGHT_M,
    clearance_m: float = LOS_CLEARANCE_M,
    sample_step_m: float = DEFAULT_PROFILE_STEP_M,
    max_samples: int = MAX_PROFILE_SAMPLES,
) -> Dict[str, Any]:
    pairs, distance_m = terrain_profile_coordinates(
        enemy_coord,
        attack_coord,
        sample_step_m=float(sample_step_m),
        max_samples=int(max_samples),
    )
    if not pairs:
        return {"verified": False, "reason": "invalid_attack_target_distance"}
    try:
        terrain = list(terrain_lookup_many(pairs))
    except Exception as exc:
        return {"verified": False, "reason": f"terrain_lookup_failed:{exc}"}
    if len(terrain) != len(pairs):
        return {"verified": False, "reason": "terrain_lookup_length_mismatch"}
    return solve_minimum_attack_altitude(
        terrain,
        float(distance_m),
        target_height_m=float(target_height_m),
        clearance_m=float(clearance_m),
    )
