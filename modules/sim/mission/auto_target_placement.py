from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Any, Iterable, Sequence


EARTH_RADIUS_M = 6_371_008.8
METERS_PER_DEG_LAT = 111_320.0
TARGET_REGION_TYPE = 6


@dataclass(frozen=True)
class AutoTargetPlacementConfig:
    """Sparse SIM enemy distribution policy for input-mission geometry."""

    general_zone_probability: float = 0.5
    target_region_min_count: int = 2
    target_region_max_count: int = 5
    min_separation_m: float = 220.0
    line_width_margin_ratio: float = 0.82
    general_type_pool: tuple[int, ...] = (1, 1, 2, 2, 3, 4, 5, 6)
    target_type_pool: tuple[int, ...] = (1, 1, 1, 2, 2, 2, 3, 4, 5)
    # Appended to preserve the positional order of the original public fields.
    general_zone_min_count: int = 0
    general_zone_max_count: int = 5
    first_mission_exclusion_radius_m: float = 3_000.0
    area_placement_ratio: float = 0.7


def _get_ci(mapping: Any, *keys: str) -> Any:
    if not isinstance(mapping, dict):
        return None
    for key in keys:
        if key in mapping:
            return mapping[key]
    lowered = {str(key).lower(): value for key, value in mapping.items()}
    for key in keys:
        if str(key).lower() in lowered:
            return lowered[str(key).lower()]
    return None


def _to_int(value: Any, default: int | None = None) -> int | None:
    try:
        return int(value)
    except Exception:
        return default


def _to_float(value: Any, default: float | None = None) -> float | None:
    try:
        parsed = float(value)
    except Exception:
        return default
    return parsed if math.isfinite(parsed) else default


def _to_bool(value: Any, default: bool = False) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(int(value))
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"1", "true", "yes", "y", "on"}:
            return True
        if lowered in {"0", "false", "no", "n", "off"}:
            return False
    return default


def _coordinate(value: Any) -> tuple[float, float] | None:
    if not isinstance(value, dict):
        return None
    lat = _to_float(_get_ci(value, "latitude", "lat"))
    lon = _to_float(_get_ci(value, "longitude", "lon", "lng"))
    if lat is None or lon is None:
        return None
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        return None
    return float(lat), float(lon)


def _coordinate_list(value: Any) -> list[tuple[float, float]]:
    if not isinstance(value, list):
        return []
    return [coord for coord in (_coordinate(item) for item in value) if coord is not None]


def _point_in_polygon(point: tuple[float, float], polygon: Sequence[tuple[float, float]]) -> bool:
    if len(polygon) < 3:
        return False
    lat, lon = point
    inside = False
    previous = polygon[-1]
    for current in polygon:
        y1, x1 = previous
        y2, x2 = current
        crosses = (y1 > lat) != (y2 > lat)
        if crosses:
            x_cross = ((x2 - x1) * (lat - y1) / ((y2 - y1) or 1e-15)) + x1
            if lon < x_cross:
                inside = not inside
        previous = current
    return inside


def _distance_m(left: tuple[float, float], right: tuple[float, float]) -> float:
    lat1, lon1 = left
    lat2, lon2 = right
    mean_lat = math.radians((lat1 + lat2) * 0.5)
    dx = math.radians(lon2 - lon1) * EARTH_RADIUS_M * math.cos(mean_lat)
    dy = math.radians(lat2 - lat1) * EARTH_RADIUS_M
    return math.hypot(dx, dy)


def _line_segment_lengths(coords: Sequence[tuple[float, float]]) -> list[float]:
    return [_distance_m(coords[idx - 1], coords[idx]) for idx in range(1, len(coords))]


def _weighted_index(weights: Sequence[float], rng: random.Random) -> int:
    total = sum(max(0.0, float(value)) for value in weights)
    if total <= 0.0:
        return 0
    cursor = rng.random() * total
    for idx, value in enumerate(weights):
        cursor -= max(0.0, float(value))
        if cursor <= 0.0:
            return idx
    return max(0, len(weights) - 1)


def _sample_line_zone(zone: dict[str, Any], rng: random.Random) -> tuple[float, float] | None:
    coords = zone.get("coordinates") or []
    if len(coords) < 2:
        return None
    lengths = _line_segment_lengths(coords)
    segment_idx = _weighted_index(lengths, rng)
    start = coords[segment_idx]
    end = coords[segment_idx + 1]
    lat1, lon1 = start
    lat2, lon2 = end
    mean_lat = math.radians((lat1 + lat2) * 0.5)
    east = math.radians(lon2 - lon1) * EARTH_RADIUS_M * math.cos(mean_lat)
    north = math.radians(lat2 - lat1) * EARTH_RADIUS_M
    length = math.hypot(east, north)
    if length <= 1e-6:
        return start

    along = rng.uniform(0.04, 0.96)
    center_lat = lat1 + (lat2 - lat1) * along
    center_lon = lon1 + (lon2 - lon1) * along
    width = max(0.0, float(zone.get("width_m") or 0.0))
    margin_ratio = max(0.0, min(1.0, float(zone.get("width_margin_ratio") or 0.82)))
    lateral = rng.uniform(-0.5 * width * margin_ratio, 0.5 * width * margin_ratio)
    normal_east = -north / length
    normal_north = east / length
    offset_east = normal_east * lateral
    offset_north = normal_north * lateral
    lon_scale = METERS_PER_DEG_LAT * max(1e-6, math.cos(math.radians(center_lat)))
    return (
        center_lat + (offset_north / METERS_PER_DEG_LAT),
        center_lon + (offset_east / lon_scale),
    )


def _sample_area_zone(zone: dict[str, Any], rng: random.Random) -> tuple[float, float] | None:
    outer = zone.get("coordinates") or []
    holes = zone.get("holes") or []
    if len(outer) < 3:
        return None
    lats = [coord[0] for coord in outer]
    lons = [coord[1] for coord in outer]
    for _ in range(160):
        candidate = (rng.uniform(min(lats), max(lats)), rng.uniform(min(lons), max(lons)))
        if not _point_in_polygon(candidate, outer):
            continue
        if any(_point_in_polygon(candidate, hole) for hole in holes if len(hole) >= 3):
            continue
        return candidate
    return None


def _mission_zones(
    mission: dict[str, Any],
    *,
    line_width_margin_ratio: float,
) -> list[dict[str, Any]]:
    detail = _get_ci(mission, "missionDetail", "detail")
    if not isinstance(detail, dict):
        detail = mission
    zones: list[dict[str, Any]] = []

    line_list = _get_ci(detail, "lineList")
    if isinstance(line_list, list):
        for line in line_list:
            if not isinstance(line, dict):
                continue
            coords = _coordinate_list(_get_ci(line, "coordinateList"))
            width = _to_float(_get_ci(line, "width"), 0.0) or 0.0
            if len(coords) >= 2 and width > 0.0:
                zones.append(
                    {
                        "kind": "line",
                        "coordinates": coords,
                        "width_m": float(width),
                        "width_margin_ratio": float(line_width_margin_ratio),
                    }
                )

    area_list = _get_ci(detail, "areaList")
    if isinstance(area_list, list):
        outer_polygons: list[list[tuple[float, float]]] = []
        hole_polygons: list[list[tuple[float, float]]] = []
        for area in area_list:
            if not isinstance(area, dict):
                continue
            coords = _coordinate_list(_get_ci(area, "coordinateList"))
            if len(coords) < 3:
                continue
            if _to_bool(_get_ci(area, "isHole"), False):
                hole_polygons.append(coords)
            else:
                outer_polygons.append(coords)
        for outer in outer_polygons:
            contained_holes = [
                hole for hole in hole_polygons if hole and _point_in_polygon(hole[0], outer)
            ]
            zones.append(
                {
                    "kind": "area",
                    "coordinates": outer,
                    "holes": contained_holes,
                }
            )
    return zones


def _iter_input_missions(input_mission_plans: Any) -> Iterable[dict[str, Any]]:
    plans = input_mission_plans if isinstance(input_mission_plans, list) else [input_mission_plans]
    for plan in plans:
        if not isinstance(plan, dict):
            continue
        mission_list = _get_ci(plan, "inputMissionList")
        if not isinstance(mission_list, list):
            continue
        for mission in mission_list:
            if isinstance(mission, dict):
                yield mission


def _first_mission_start_coordinate(
    input_mission_plans: Any,
) -> tuple[float, float] | None:
    """Return the first usable coordinate of the first positioned input mission."""

    for mission in _iter_input_missions(input_mission_plans):
        detail = _get_ci(mission, "missionDetail", "detail")
        if not isinstance(detail, dict):
            detail = mission

        direct_coordinates = _coordinate_list(_get_ci(detail, "coordinateList"))
        if direct_coordinates:
            return direct_coordinates[0]

        line_list = _get_ci(detail, "lineList")
        if isinstance(line_list, list):
            for line in line_list:
                coordinates = _coordinate_list(_get_ci(line, "coordinateList"))
                if coordinates:
                    return coordinates[0]

        area_list = _get_ci(detail, "areaList")
        if isinstance(area_list, list):
            # A hole is not a mission start boundary. Prefer the first outer area.
            for area in area_list:
                if _to_bool(_get_ci(area, "isHole"), False):
                    continue
                coordinates = _coordinate_list(_get_ci(area, "coordinateList"))
                if coordinates:
                    return coordinates[0]
    return None


def _sample_zone(zone: dict[str, Any], rng: random.Random) -> tuple[float, float] | None:
    if zone.get("kind") == "line":
        return _sample_line_zone(zone, rng)
    if zone.get("kind") == "area":
        return _sample_area_zone(zone, rng)
    return None


def _is_separated(
    candidate: tuple[float, float],
    occupied: Sequence[tuple[float, float]],
    min_separation_m: float,
) -> bool:
    return all(_distance_m(candidate, existing) >= min_separation_m for existing in occupied)


def generate_auto_target_placements(
    input_mission_plans: Any,
    *,
    rng: random.Random | None = None,
    config: AutoTargetPlacementConfig | None = None,
    occupied_coordinates: Sequence[tuple[float, float]] | None = None,
) -> dict[str, Any]:
    """Generate sparse enemies inside Line/Area mission geometry.

    Each general geometry component is selected with a medium probability and,
    when selected, receives 0-5 targets.  A target-region input mission
    (regionType=6) receives 2-5 targets in total across all of its geometry
    components. Candidates inside the configured radius of the first input
    mission's user-selected start coordinate are always rejected.
    """

    random_source = rng or random.Random()
    policy = config or AutoTargetPlacementConfig()
    occupied = list(occupied_coordinates or [])
    first_mission_start = _first_mission_start_coordinate(input_mission_plans)
    first_mission_exclusion_radius_m = max(
        0.0,
        float(policy.first_mission_exclusion_radius_m),
    )
    placements: list[dict[str, Any]] = []
    general_count = 0
    target_region_count = 0
    area_count = 0
    line_count = 0
    zone_count = 0
    selected_general_zone_count = 0
    first_mission_exclusion_rejection_count = 0

    def add_from_zone(
        zone: dict[str, Any],
        *,
        mission_id: int | None,
        region_type: int | None,
        target_region: bool,
    ) -> bool:
        nonlocal general_count, target_region_count
        nonlocal area_count, line_count
        nonlocal first_mission_exclusion_rejection_count
        for attempt in range(80):
            candidate = _sample_zone(zone, random_source)
            if candidate is None:
                continue
            if (
                first_mission_start is not None
                and first_mission_exclusion_radius_m > 0.0
                and _distance_m(candidate, first_mission_start)
                < first_mission_exclusion_radius_m
            ):
                first_mission_exclusion_rejection_count += 1
                continue
            separation = float(policy.min_separation_m)
            if attempt >= 50:
                separation = max(60.0, separation * 0.55)
            if not _is_separated(candidate, occupied, separation):
                continue
            type_pool = policy.target_type_pool if target_region else policy.general_type_pool
            type_id = int(random_source.choice(type_pool or (1,)))
            lat, lon = candidate
            placements.append(
                {
                    "type": type_id,
                    "lat": float(lat),
                    "lon": float(lon),
                    "alt": 0.0,
                    "inputMissionID": mission_id,
                    "regionType": region_type,
                    "geometryType": str(zone.get("kind") or "unknown"),
                    "targetRegion": bool(target_region),
                    "source": "auto_mission_geometry",
                }
            )
            occupied.append(candidate)
            if target_region:
                target_region_count += 1
            else:
                general_count += 1
            if zone.get("kind") == "area":
                area_count += 1
            elif zone.get("kind") == "line":
                line_count += 1
            return True
        return False

    placement_domains: list[dict[str, Any]] = []

    for mission in _iter_input_missions(input_mission_plans):
        if _to_bool(_get_ci(mission, "isDone"), False):
            continue
        mission_id = _to_int(_get_ci(mission, "inputMissionID"))
        region_type = _to_int(_get_ci(mission, "regionType"))
        zones = _mission_zones(
            mission,
            line_width_margin_ratio=policy.line_width_margin_ratio,
        )
        zone_count += len(zones)
        if not zones:
            continue

        is_target_region = region_type == TARGET_REGION_TYPE
        if is_target_region:
            lower = max(0, int(policy.target_region_min_count))
            upper = max(lower, int(policy.target_region_max_count))
            desired = random_source.randint(lower, upper)
            placement_domains.append(
                {
                    "zones": zones,
                    "desired": desired,
                    "mission_id": mission_id,
                    "region_type": region_type,
                    "target_region": True,
                }
            )
            continue

        probability = max(0.0, min(1.0, float(policy.general_zone_probability)))
        lower = max(0, int(policy.general_zone_min_count))
        upper = max(lower, int(policy.general_zone_max_count))
        desired_total = 0
        for zone in zones:
            if random_source.random() > probability:
                continue
            selected_general_zone_count += 1
            desired_total += random_source.randint(lower, upper)
        placement_domains.append(
            {
                # Zone selection still determines the existing random total;
                # distribution then uses every general mission geometry so the
                # requested global Area/Line ratio is not biased by loop order.
                "zones": zones,
                "desired": desired_total,
                "mission_id": mission_id,
                "region_type": region_type,
                "target_region": False,
            }
        )

    def placement_options(
        domains: Sequence[dict[str, Any]],
        kind: str,
    ) -> list[tuple[dict[str, Any], dict[str, Any]]]:
        return [
            (zone, domain)
            for domain in domains
            for zone in domain.get("zones") or []
            if zone.get("kind") == kind
        ]

    def add_from_options(
        options: Sequence[tuple[dict[str, Any], dict[str, Any]]],
    ) -> bool:
        if not options:
            return False
        start_idx = random_source.randrange(len(options))
        for offset in range(len(options)):
            zone, domain = options[(start_idx + offset) % len(options)]
            if add_from_zone(
                zone,
                mission_id=domain.get("mission_id"),
                region_type=domain.get("region_type"),
                target_region=bool(domain.get("target_region")),
            ):
                return True
        return False

    target_domains = [domain for domain in placement_domains if domain["target_region"]]
    general_domains = [domain for domain in placement_domains if not domain["target_region"]]
    domain_groups = [target_domains, general_domains]
    group_totals = [sum(int(domain["desired"]) for domain in group) for group in domain_groups]
    group_options = [
        {
            "area": placement_options(group, "area"),
            "line": placement_options(group, "line"),
        }
        for group in domain_groups
    ]
    group_area_min = [
        total if options["area"] and not options["line"] else 0
        for total, options in zip(group_totals, group_options)
    ]
    group_area_max = [
        total if options["area"] else 0
        for total, options in zip(group_totals, group_options)
    ]
    total_desired = sum(group_totals)
    area_ratio = max(0.0, min(1.0, float(policy.area_placement_ratio)))
    requested_area_total = int(math.floor((total_desired * area_ratio) + 0.5))
    feasible_area_min = sum(group_area_min)
    feasible_area_max = sum(group_area_max)
    requested_area_total = max(
        feasible_area_min,
        min(feasible_area_max, requested_area_total),
    )
    group_area_quota = [
        max(
            group_area_min[idx],
            min(
                group_area_max[idx],
                int(math.floor((group_totals[idx] * area_ratio) + 0.5)),
            ),
        )
        for idx in range(len(domain_groups))
    ]
    quota_delta = requested_area_total - sum(group_area_quota)
    while quota_delta != 0:
        changed = False
        for idx in range(len(group_area_quota)):
            if quota_delta > 0 and group_area_quota[idx] < group_area_max[idx]:
                group_area_quota[idx] += 1
                quota_delta -= 1
                changed = True
            elif quota_delta < 0 and group_area_quota[idx] > group_area_min[idx]:
                group_area_quota[idx] -= 1
                quota_delta += 1
                changed = True
            if quota_delta == 0:
                break
        if not changed:
            break

    for idx, total in enumerate(group_totals):
        area_quota = group_area_quota[idx]
        preferred_kinds = (["area"] * area_quota) + (["line"] * (total - area_quota))
        random_source.shuffle(preferred_kinds)
        options = group_options[idx]
        for preferred_kind in preferred_kinds:
            fallback_kind = "line" if preferred_kind == "area" else "area"
            if not add_from_options(options[preferred_kind]):
                add_from_options(options[fallback_kind])

    return {
        "placements": placements,
        "count": len(placements),
        "generalCount": int(general_count),
        "targetRegionCount": int(target_region_count),
        "areaCount": int(area_count),
        "lineCount": int(line_count),
        "areaPlacementRatio": float(policy.area_placement_ratio),
        "zoneCount": int(zone_count),
        "selectedGeneralZoneCount": int(selected_general_zone_count),
        "generalZoneProbability": float(policy.general_zone_probability),
        "generalZoneRange": [
            int(policy.general_zone_min_count),
            int(policy.general_zone_max_count),
        ],
        "targetRegionRange": [
            int(policy.target_region_min_count),
            int(policy.target_region_max_count),
        ],
        "firstMissionStartCoordinate": (
            {
                "latitude": float(first_mission_start[0]),
                "longitude": float(first_mission_start[1]),
            }
            if first_mission_start is not None
            else None
        ),
        "firstMissionExclusionRadiusM": float(first_mission_exclusion_radius_m),
        "firstMissionExclusionRejectedCandidateCount": int(
            first_mission_exclusion_rejection_count
        ),
    }


__all__ = [
    "AutoTargetPlacementConfig",
    "TARGET_REGION_TYPE",
    "generate_auto_target_placements",
]
