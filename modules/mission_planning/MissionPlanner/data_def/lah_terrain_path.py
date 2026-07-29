from __future__ import annotations

import math
from bisect import bisect_left
from typing import Any, Callable, Iterable, Sequence

from modules.mission_planning.MissionPlanner.data_def.mission_helpers import (
    terrain_elev,
    terrain_elev_many,
)
from modules.mission_planning.MissionPlanner.dynamics.lah_op_envlp import (
    DEFAULT_ENVELOPE,
)


# 50 m is the operational hard floor.  Planning at 75 m leaves 25 m for DEM
# sampling/rounding and normal path-following error without changing mission
# geometry or adding raster-search complexity.
LAH_LOW_LEVEL_CLEARANCE_M = 75.0
# Inje DTED/DEM cells are approximately 10 m.  A 20 m centre-line sample is
# dense enough to retain short ridges while keeping the lookup a single cheap
# vectorized batch; only the simplified points are emitted as ICD waypoints.
LAH_TERRAIN_SAMPLE_SPACING_M = 20.0
LAH_TERRAIN_MAX_WAYPOINT_SPACING_M = 750.0
LAH_TERRAIN_MAX_PROFILE_EXCESS_M = 35.0
LAH_TERRAIN_MAX_OUTPUT_WAYPOINTS = 64
LAH_VERTICAL_RATE_USE_RATIO = 0.75
LAH_LOW_TERRAIN_MIN_LEG_M = 400.0
# Wide enough to reach the low ground beside the leg: the valley worth
# following is often a ridge line away, not on the leg's own shoulder.
LAH_LOW_TERRAIN_CORRIDOR_M = 2_400.0
LAH_LOW_TERRAIN_CORRIDOR_RATIO = 0.8
# Station spacing along the leg.  This is the along-track resolution of the
# search: at 3 km a 4 km leg gets a single mid-point that can only be nudged,
# which is why the first cut barely moved the route at all.
LAH_LOW_TERRAIN_STAGE_SPACING_M = 100.0
LAH_LOW_TERRAIN_MAX_STAGES = 220
# Lateral resolution: a ~100 m lane pitch across the corridor.  A coarse pitch
# can only pick a side of the ridge, and it is the pitch - not the corridor
# width - that decides whether the route can follow the low ground.  The lane
# count scales with the corridor (2x corridor -> 2x lanes) so the pitch stays
# ~100 m and the per-station turn angle is unchanged.
LAH_LOW_TERRAIN_LANE_COUNT = 49
# One lane per station keeps the divert flyable: lane pitch is sized to the
# station spacing, so a full-rate sidestep is a ~40 degree turn.
LAH_LOW_TERRAIN_MAX_LANE_STEP = 1
# Perf backstop when strength widens the corridor: lanes are added to hold the
# ~100 m pitch, but never past this many per side.
_LOW_TERRAIN_MAX_HALF_LANES = 60
# A detour is only worth flying if the terrain saved pays for the extra track.
LAH_LOW_TERRAIN_MAX_LENGTH_RATIO = 1.9
# One interior sample plus the arrival node per edge.  Edges are ~100-140 m and
# the sampler dedups to the quantum grid, so denser edge sampling buys almost
# nothing while doubling the DEM batch.
LAH_LOW_TERRAIN_EDGE_SAMPLES = 1
LAH_LOW_TERRAIN_SIMPLIFY_M = 150.0
# Re-centre passes: the corridor spine is rebuilt around the previous winner so
# the lanes can follow a bent valley.  One pass is the measured sweet spot on
# real terrain (a second pass returned equal-or-worse routes for +60% time);
# the loop also stops early on a small gain.
LAH_LOW_TERRAIN_REFINE_PASSES = 1
# Stop re-centring when a pass improves the score by less than this.
_LOW_TERRAIN_REFINE_MIN_GAIN_M = 3.0
# Cheap pre-probe: a handful of samples either side of the leg.  If nothing
# out there is meaningfully lower than the direct line, skip the whole lattice.
_LOW_TERRAIN_PROBE_MIN_GAIN_M = 15.0
# Extra peak allowance per strength step above 1.0.  At the tuned baseline the
# route must stay under the direct line's peak, which forbids crossing a low
# saddle into a much deeper valley beyond it; a stronger search may pay a
# bounded, deliberate saddle crossing for a large mean reduction.
_LOW_TERRAIN_PEAK_RELAX_PER_STRENGTH_M = 30.0
# How many metres of lateral detour one metre of terrain elevation is worth.
# Climbing is slow and puts the aircraft on a skyline; going around is neither,
# so terrain dominates distance.  The length budget, not these weights, is what
# stops the search wandering.
_LOW_TERRAIN_MEAN_WEIGHT = 16.0
# Peak weights swept alongside the distance term.  A route can average lower
# while still topping a ridge somewhere, so the search needs the option of
# paying much more to stay out of the high ground entirely.
_LOW_TERRAIN_PEAK_WEIGHT_LADDER = (5.0, 15.0, 40.0, 100.0, 250.0, 600.0)
# DEM sampling noise between two differently-shaped routes.  Anything above
# this counts as genuinely crossing higher ground and is rejected.
_LOW_TERRAIN_PEAK_TOLERANCE_M = 5.0
# Per metre of lateral movement.  Breaks ties towards a straight track through
# equally low ground without being able to veto a genuinely lower valley.
_LOW_TERRAIN_LANE_PENALTY_PER_M = 0.12
# Multipliers applied to the distance term until the route fits the length
# budget.  Re-running the search over already-sampled terrain is arithmetic
# only, so the whole ladder costs far less than one extra DEM lookup.
_LOW_TERRAIN_LENGTH_WEIGHT_LADDER = (1.0, 2.0, 4.0, 8.0, 16.0, 32.0, 64.0, 128.0)
# Collapses the sample points that adjacent corridor edges share so one batched
# lookup covers the whole search.  Capped at the DEM cell size, and at a quarter
# of the lane pitch so a narrow corridor never quantises its lanes together.
_LOW_TERRAIN_SAMPLE_QUANTUM_MAX_M = 10.0
_LOW_TERRAIN_SAMPLE_QUANTUM_MIN_M = 0.5
_METRES_PER_DEGREE_LAT = 111_132.92
_EARTH_RADIUS_M = 6_371_000.0


def _to_lat_lon(value: Any) -> tuple[float, float] | None:
    if isinstance(value, dict):
        latitude = value.get("latitude", value.get("lat"))
        longitude = value.get("longitude", value.get("lon"))
    elif isinstance(value, (tuple, list)) and len(value) >= 2:
        latitude, longitude = value[0], value[1]
    else:
        return None
    try:
        lat = float(latitude)
        lon = float(longitude)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(lat) or not math.isfinite(lon):
        return None
    return lat, lon


def _distance_m(a: tuple[float, float], b: tuple[float, float]) -> float:
    lat1, lon1 = map(math.radians, a)
    lat2, lon2 = map(math.radians, b)
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    value = (
        math.sin(dlat / 2.0) ** 2
        + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
    )
    value = max(0.0, min(1.0, value))
    return 2.0 * _EARTH_RADIUS_M * math.asin(math.sqrt(value))


def _sample_horizontal_route(
    route: Sequence[tuple[float, float]],
    *,
    sample_spacing_m: float,
) -> tuple[list[dict[str, float]], list[int]]:
    samples: list[dict[str, float]] = [
        {"latitude": route[0][0], "longitude": route[0][1], "cum_m": 0.0}
    ]
    anchor_indices = [0]
    cumulative_m = 0.0
    spacing_m = max(5.0, float(sample_spacing_m))
    for start, end in zip(route, route[1:]):
        leg_m = _distance_m(start, end)
        if leg_m <= 0.01:
            anchor_indices.append(len(samples) - 1)
            continue
        segment_count = max(1, int(math.ceil(leg_m / spacing_m)))
        for part in range(1, segment_count + 1):
            fraction = float(part) / float(segment_count)
            latitude = start[0] + (end[0] - start[0]) * fraction
            longitude = start[1] + (end[1] - start[1]) * fraction
            samples.append(
                {
                    "latitude": latitude,
                    "longitude": longitude,
                    "cum_m": cumulative_m + leg_m * fraction,
                }
            )
        cumulative_m += leg_m
        anchor_indices.append(len(samples) - 1)
    return samples, anchor_indices


def _load_terrain_profile(
    samples: list[dict[str, float]],
    provider: Callable[[Iterable[Any]], Iterable[float]] | None,
) -> list[float]:
    pairs = [(sample["latitude"], sample["longitude"]) for sample in samples]
    terrain_provider = provider or terrain_elev_many
    try:
        values = list(terrain_provider(pairs))
        if len(values) != len(pairs):
            raise ValueError("terrain provider returned a mismatched sample count")
        return [float(value) if math.isfinite(float(value)) else 0.0 for value in values]
    except Exception:
        values: list[float] = []
        for latitude, longitude in pairs:
            try:
                value = float(terrain_elev(latitude, longitude))
                values.append(value if math.isfinite(value) else 0.0)
            except Exception:
                values.append(0.0)
        return values


def _offset_along_polyline(
    centerline: Sequence[tuple[float, float]],
    fraction: float,
    lateral_offset_m: float,
) -> tuple[float, float]:
    """Point at an arc-length fraction of a polyline, offset perpendicular.

    The station is placed by cumulative distance along the (possibly bent)
    centerline, and the lateral offset uses the local perpendicular of the
    segment that contains the station.  A straight two-point centerline
    reduces to the plain leg-offset case.
    """

    alpha = min(max(float(fraction), 0.0), 1.0)
    mid_lat = (float(centerline[0][0]) + float(centerline[-1][0])) * 0.5
    metres_per_lat = _METRES_PER_DEGREE_LAT
    metres_per_lon = max(1.0, metres_per_lat * math.cos(math.radians(mid_lat)))

    lengths = [
        _distance_m(left, right) for left, right in zip(centerline, centerline[1:])
    ]
    total_m = sum(lengths)
    if total_m <= 1e-6:
        return (float(centerline[0][0]), float(centerline[0][1]))

    target_m = total_m * alpha
    covered_m = 0.0
    segment_index = len(lengths) - 1
    for index, length in enumerate(lengths):
        if covered_m + length >= target_m or length <= 0.0:
            if length > 0.0:
                segment_index = index
                break
            continue
        covered_m += length
    segment_start = centerline[segment_index]
    segment_end = centerline[segment_index + 1]
    segment_m = lengths[segment_index]
    local_ratio = 0.0 if segment_m <= 1e-9 else (target_m - covered_m) / segment_m
    local_ratio = min(max(local_ratio, 0.0), 1.0)

    base_lat = (
        float(segment_start[0])
        + (float(segment_end[0]) - float(segment_start[0])) * local_ratio
    )
    base_lon = (
        float(segment_start[1])
        + (float(segment_end[1]) - float(segment_start[1])) * local_ratio
    )
    dx = (float(segment_end[1]) - float(segment_start[1])) * metres_per_lon
    dy = (float(segment_end[0]) - float(segment_start[0])) * metres_per_lat
    norm = math.hypot(dx, dy)
    if norm <= 1e-6:
        return (base_lat, base_lon)
    perp_x = -dy / norm
    perp_y = dx / norm
    return (
        base_lat + (perp_y * float(lateral_offset_m)) / metres_per_lat,
        base_lon + (perp_x * float(lateral_offset_m)) / metres_per_lon,
    )


def _polyline_stations(
    centerline: Sequence[tuple[float, float]],
    fractions: Sequence[float],
) -> list[tuple[float, float, float, float]]:
    """Station centre and per-metre perpendicular for arc-length fractions.

    One walk over the centerline serves every station: placing each lattice
    node through :func:`_offset_along_polyline` would recompute the polyline's
    segment lengths per node, which dominated the whole search.  Returns
    ``(base_lat, base_lon, dlat_per_m, dlon_per_m)`` per fraction, where the
    offsets are the local left-perpendicular in degrees per metre.
    """

    mid_lat = (float(centerline[0][0]) + float(centerline[-1][0])) * 0.5
    metres_per_lat = _METRES_PER_DEGREE_LAT
    metres_per_lon = max(1.0, metres_per_lat * math.cos(math.radians(mid_lat)))

    lengths = [
        _distance_m(left, right) for left, right in zip(centerline, centerline[1:])
    ]
    cumulative = [0.0]
    for length in lengths:
        cumulative.append(cumulative[-1] + length)
    total_m = cumulative[-1]

    stations: list[tuple[float, float, float, float]] = []
    if total_m <= 1e-6:
        base = (float(centerline[0][0]), float(centerline[0][1]))
        return [(base[0], base[1], 0.0, 0.0) for _ in fractions]

    segment_index = 0
    last_segment = len(lengths) - 1
    for fraction in fractions:  # fractions arrive ascending
        target_m = total_m * min(max(float(fraction), 0.0), 1.0)
        while segment_index < last_segment and (
            cumulative[segment_index + 1] < target_m
            or lengths[segment_index] <= 0.0
        ):
            segment_index += 1
        segment_start = centerline[segment_index]
        segment_end = centerline[segment_index + 1]
        segment_m = lengths[segment_index]
        local_ratio = (
            0.0
            if segment_m <= 1e-9
            else (target_m - cumulative[segment_index]) / segment_m
        )
        local_ratio = min(max(local_ratio, 0.0), 1.0)
        base_lat = (
            float(segment_start[0])
            + (float(segment_end[0]) - float(segment_start[0])) * local_ratio
        )
        base_lon = (
            float(segment_start[1])
            + (float(segment_end[1]) - float(segment_start[1])) * local_ratio
        )
        dx = (float(segment_end[1]) - float(segment_start[1])) * metres_per_lon
        dy = (float(segment_end[0]) - float(segment_start[0])) * metres_per_lat
        norm = math.hypot(dx, dy)
        if norm <= 1e-6:
            stations.append((base_lat, base_lon, 0.0, 0.0))
            continue
        stations.append(
            (
                base_lat,
                base_lon,
                (dx / norm) / metres_per_lat,
                (-dy / norm) / metres_per_lon,
            )
        )
    return stations


def _low_terrain_strength(runtime_payload: dict[str, Any] | None = None) -> float:
    """Operator dial for the low-terrain search, from runtime settings.

    ``lah_low_terrain_strength``: 0 disables the detour search entirely,
    1.0 (default) is the tuned behaviour, up to 3 searches wider/longer.
    """

    try:
        from modules.mission_planning.MissionPlanner.runtime_settings import (
            get_runtime_float,
        )

        value = float(
            get_runtime_float(
                "lah_low_terrain_strength",
                1.0,
                runtime_payload,
            )
        )
    except Exception:
        return 1.0
    if not math.isfinite(value):
        return 1.0
    return min(max(value, 0.0), 3.0)


def _corridor_incoming_edges(
    layers: Sequence[Sequence[dict[str, Any]]],
    edges: dict[tuple[int, int, int], dict[str, Any]],
) -> list[list[list[tuple[int, dict[str, Any]]]]]:
    """Group lattice edges by their arrival node.

    ``incoming[layer_index][current_index]`` lists ``(previous_index, edge)``
    pairs, so the search visits only the few lanes that can actually reach a
    node instead of scanning the whole previous layer.
    """

    incoming: list[list[list[tuple[int, dict[str, Any]]]]] = [
        [[] for _ in layer] for layer in layers
    ]
    for (layer_index, previous_index, current_index), edge in edges.items():
        incoming[layer_index][current_index].append((previous_index, edge))
    return incoming


def _low_terrain_route_for_leg(
    start: tuple[float, float],
    end: tuple[float, float],
    *,
    terrain_provider: Callable[[Iterable[Any]], Iterable[float]] | None,
    corridor_width_m: float,
    min_leg_m: float,
    stage_spacing_m: float,
    max_stages: int,
    edge_samples: int,
    segment_allowed: Callable[
        [tuple[float, float], tuple[float, float]], bool
    ]
    | None,
    strength: float | None = None,
) -> list[tuple[float, float]]:
    """Route one leg through the low ground on either side of it.

    A corridor several kilometres wide is laid out around the direct line and
    divided into stations every few hundred metres.  A shortest-path search
    over that lattice picks the lane at each station, so the route can walk
    into a neighbouring valley and follow it rather than merely shaving the
    shoulder of the ridge in front of it.  The winning route then becomes the
    centerline for the next pass, so the lanes can follow a bent valley.  All
    passes share one de-duplicated DEM batch, which keeps the search far
    cheaper than a raster search.
    """

    direct_m = _distance_m(start, end)
    if direct_m < max(100.0, float(min_leg_m)):
        return [start, end]
    if callable(segment_allowed):
        try:
            if not bool(segment_allowed(start, end)):
                # The source leg itself is outside/through a forbidden part of
                # the geometry.  Preserve it, but do not create a new detour.
                return [start, end]
        except Exception:
            return [start, end]

    if strength is None:
        strength = _low_terrain_strength()
    strength = min(max(float(strength), 0.0), 3.0)
    if strength <= 0.0:
        return [start, end]

    stage_count = max(
        1,
        min(
            max(1, int(max_stages)),
            int(round(direct_m / max(50.0, float(stage_spacing_m)))) - 1,
        ),
    )
    corridor_request_m = max(10.0, float(corridor_width_m))
    corridor_scaled_m = corridor_request_m * strength
    if corridor_request_m < float(LAH_LOW_TERRAIN_CORRIDOR_M):
        # A corridor narrower than the default is mission geometry (declared
        # LINE width via d0304), not a tuning value: strength may shrink the
        # search inside it but must never widen past the caller's promise.
        corridor_scaled_m = min(corridor_scaled_m, corridor_request_m)
    corridor_m = min(
        max(10.0, corridor_scaled_m),
        max(25.0, direct_m * LAH_LOW_TERRAIN_CORRIDOR_RATIO),
    )
    length_ratio = 1.0 + (LAH_LOW_TERRAIN_MAX_LENGTH_RATIO - 1.0) * strength
    lane_penalty_per_m = _LOW_TERRAIN_LANE_PENALTY_PER_M / max(0.25, strength)
    refine_passes = min(
        int(round(max(0, LAH_LOW_TERRAIN_REFINE_PASSES) * strength)),
        int(LAH_LOW_TERRAIN_REFINE_PASSES),
    )
    length_budget_m = direct_m * length_ratio
    peak_allow_m = (
        _LOW_TERRAIN_PEAK_TOLERANCE_M
        + max(0.0, strength - 1.0) * _LOW_TERRAIN_PEAK_RELAX_PER_STRENGTH_M
    )

    lane_count = max(3, int(LAH_LOW_TERRAIN_LANE_COUNT) | 1)
    base_half_lanes = (lane_count - 1) // 2
    # When strength widens the corridor, add lanes to hold the ~100 m pitch:
    # the pitch - not the corridor width - is what lets the route follow the
    # low ground, and it keeps the per-station sidestep angle flyable.  A
    # narrow requested corridor keeps the base lane count (finer pitch).
    target_pitch_m = float(LAH_LOW_TERRAIN_CORRIDOR_M) / float(base_half_lanes)
    half_lanes = max(
        base_half_lanes,
        min(_LOW_TERRAIN_MAX_HALF_LANES, int(round(corridor_m / target_pitch_m))),
    )
    lane_pitch_m = corridor_m / float(half_lanes)
    interior_ratios = [
        float(index) / float(max(1, int(edge_samples)) + 1)
        for index in range(1, max(1, int(edge_samples)) + 1)
    ]
    max_lane_step = max(1, int(LAH_LOW_TERRAIN_MAX_LANE_STEP))

    # One sampler is shared by every pass: a re-centred corridor overlaps most
    # of the previous one, and resolve() only fetches keys not yet looked up.
    # The probe points are requested before the direct-line stats so that one
    # provider batch serves both - the production provider pays a tile-index
    # overhead per call, not per point.
    sampler = _TerrainSampler(terrain_provider, quantum_m=lane_pitch_m / 4.0)
    probe_points = [
        _offset_along_polyline([start, end], fraction, offset * corridor_m)
        for fraction in (0.25, 0.5, 0.75)
        for offset in (-1.0, -0.5, 0.5, 1.0)
    ]
    probe_keys = sampler.request(probe_points)
    direct_mean_m, direct_peak_m = _route_ground_stats([start, end], sampler)

    # A handful of probe points either side of the leg decide whether the
    # lattice is worth building at all.  Plains and along-valley legs exit
    # here for a couple dozen DEM cells instead of a full corridor search.
    probe_values = sampler.values(probe_keys)
    probe_min_m = min(probe_values) if probe_values else 0.0
    if direct_mean_m - probe_min_m < _LOW_TERRAIN_PROBE_MIN_GAIN_M:
        return [start, end]

    def _search_around(
        centerline: Sequence[tuple[float, float]],
    ) -> tuple[list[tuple[float, float]], float, float] | None:
        """One lattice search around a centerline; (route, score, peak)."""

        layers: list[list[dict[str, Any]]] = [[{"coord": start, "lane": 0}]]
        station_bases = _polyline_stations(
            centerline,
            [
                float(stage) / float(stage_count + 1)
                for stage in range(1, stage_count + 1)
            ],
        )
        for base_lat, base_lon, dlat_per_m, dlon_per_m in station_bases:
            stations: list[dict[str, Any]] = []
            for lane in range(-half_lanes, half_lanes + 1):
                offset_m = float(lane) * lane_pitch_m
                coord = (
                    base_lat + dlat_per_m * offset_m,
                    base_lon + dlon_per_m * offset_m,
                )
                # The corridor is a promise to the caller (mission LINE width),
                # so a re-centred pass may reshape the station spine but must
                # never let cumulative drift place a node outside the corridor
                # of the original leg.  Lane 0 sits on a previously accepted
                # route, so every stage keeps at least one node.
                if _cross_track_m(coord, start, end) > corridor_m + 1.0:
                    continue
                stations.append({"coord": coord, "lane": lane})
            if not stations:
                return None
            layers.append(stations)
        layers.append([{"coord": end, "lane": 0}])

        edges: dict[tuple[int, int, int], dict[str, Any]] = {}
        for layer_index in range(1, len(layers)):
            previous_layer = layers[layer_index - 1]
            current_layer = layers[layer_index]
            # A station-to-station sidestep is bounded so the divert stays
            # flyable; index the arrival layer by lane so each node considers
            # only its few reachable neighbours instead of the whole layer.
            current_by_lane = {
                int(node["lane"]): index for index, node in enumerate(current_layer)
            }
            for previous_index, previous in enumerate(previous_layer):
                previous_lane = int(previous["lane"])
                for lane in range(
                    previous_lane - max_lane_step, previous_lane + max_lane_step + 1
                ):
                    current_index = current_by_lane.get(lane)
                    if current_index is None:
                        continue
                    current = current_layer[current_index]
                    left = previous["coord"]
                    right = current["coord"]
                    if callable(segment_allowed):
                        try:
                            if not bool(segment_allowed(left, right)):
                                continue
                        except Exception:
                            continue
                    points = [
                        (
                            float(left[0]) + (float(right[0]) - float(left[0])) * ratio,
                            float(left[1]) + (float(right[1]) - float(left[1])) * ratio,
                        )
                        for ratio in interior_ratios
                    ]
                    points.append((float(right[0]), float(right[1])))
                    edges[(layer_index, previous_index, current_index)] = {
                        "keys": sampler.request(points),
                        "length_m": _distance_m(left, right),
                        "lateral_m": abs(previous_lane - lane) * lane_pitch_m,
                    }

        if not edges:
            return None
        sampler.resolve()
        for edge in edges.values():
            values = sampler.values(edge["keys"]) or [0.0]
            edge["mean_ground"] = max(0.0, sum(values) / float(len(values)))
            edge["peak_ground"] = max(0.0, max(values))
            edge["turn_cost"] = lane_penalty_per_m * float(edge["lateral_m"])
        incoming = _corridor_incoming_edges(layers, edges)

        # Terrain is worth far more than distance here: going a few hundred
        # metres around costs seconds, climbing the same height costs far
        # longer and puts the aircraft on a skyline where it is seen.  Both
        # weights are then swept: the distance term bounds the detour honestly
        # instead of by cramping the corridor, and the peak term buys routes
        # that stay out of the high ground rather than merely averaging lower
        # across it.
        pass_best: list[tuple[float, float]] | None = None
        pass_best_score: float | None = None
        pass_best_peak = 0.0
        # Neighbouring weightings very often agree on the winning lane
        # sequence, so scoring is keyed by the route itself, not the weights.
        scored: set[tuple[int, ...]] = set()
        for peak_weight in _LOW_TERRAIN_PEAK_WEIGHT_LADDER:
            # The distance multipliers exist to squeeze the route under the
            # length budget: walk them upward and stop at the first fit.  The
            # first fitting route is the most terrain-greedy one this peak
            # weighting can afford, so the higher multipliers - which only
            # produce straighter, higher routes - are never worth running.
            for length_weight in _LOW_TERRAIN_LENGTH_WEIGHT_LADDER:
                lanes = _cheapest_corridor_lanes(
                    layers, incoming, length_weight, peak_weight
                )
                if lanes is None or len(lanes) < 2:
                    continue
                candidate = [
                    tuple(layers[layer_index][node_index]["coord"])
                    for layer_index, node_index in enumerate(lanes)
                ]
                if _route_length_m(candidate) > length_budget_m:
                    continue
                if lanes not in scored:
                    scored.add(lanes)
                    # The edge costs are a proxy sampled per edge; what
                    # actually matters is the ground the aircraft ends up
                    # over.  Score the whole candidate, and never accept one
                    # that flies lower on average by crossing much higher
                    # ground somewhere along the way.
                    mean_m, peak_m = _route_ground_stats(candidate, sampler)
                    if mean_m < direct_mean_m and peak_m <= direct_peak_m + peak_allow_m:
                        score = mean_m + peak_m
                        if pass_best_score is None or score < pass_best_score:
                            pass_best_score = score
                            pass_best = candidate
                            pass_best_peak = peak_m
                break
        if pass_best is None or pass_best_score is None:
            return None
        return pass_best, pass_best_score, pass_best_peak

    best: list[tuple[float, float]] | None = None
    best_score: float | None = None
    centerline: list[tuple[float, float]] = [start, end]
    for _ in range(1 + refine_passes):
        found = _search_around(centerline)
        if found is None:
            break
        route, score, _peak = found
        if best_score is not None and score >= best_score - _LOW_TERRAIN_REFINE_MIN_GAIN_M:
            # Converged: another full lattice is not worth a marginal gain.
            if score < best_score:
                best = route
                best_score = score
            break
        best = route
        best_score = score
        centerline = route

    if best is None:
        return [start, end]
    # Every corner survives downstream as an emitted waypoint, so collapse the
    # long straight runs the lattice reports one station at a time.  The
    # tolerance is held below a quarter of the corridor: a narrow corridor's
    # whole detour is thinner than the flat tolerance would erase.
    return _simplify_horizontal_route(
        best, min(LAH_LOW_TERRAIN_SIMPLIFY_M, corridor_m * 0.25)
    )


class _TerrainSampler:
    """Collect corridor sample points, look them up once, serve them back.

    Adjacent lattice edges share most of their ground, so quantising to the DEM
    cell size and de-duplicating turns thousands of requested points into a few
    hundred distinct lookups.
    """

    def __init__(
        self,
        provider: Callable[[Iterable[Any]], Iterable[float]] | None,
        *,
        quantum_m: float = _LOW_TERRAIN_SAMPLE_QUANTUM_MAX_M,
    ) -> None:
        self._provider = provider
        self._quantum_deg = (
            min(
                max(float(quantum_m), _LOW_TERRAIN_SAMPLE_QUANTUM_MIN_M),
                _LOW_TERRAIN_SAMPLE_QUANTUM_MAX_M,
            )
            / _METRES_PER_DEGREE_LAT
        )
        self._pending: dict[tuple[int, int], tuple[float, float]] = {}
        self._values: dict[tuple[int, int], float] = {}

    def request(self, points: Iterable[tuple[float, float]]) -> list[tuple[int, int]]:
        keys: list[tuple[int, int]] = []
        for latitude, longitude in points:
            key = (
                int(round(float(latitude) / self._quantum_deg)),
                int(round(float(longitude) / self._quantum_deg)),
            )
            if key not in self._pending:
                self._pending[key] = (float(latitude), float(longitude))
            keys.append(key)
        return keys

    def resolve(self) -> None:
        outstanding = [key for key in self._pending if key not in self._values]
        if not outstanding:
            return
        samples = [
            {
                "latitude": self._pending[key][0],
                "longitude": self._pending[key][1],
                "cum_m": 0.0,
            }
            for key in outstanding
        ]
        heights = _load_terrain_profile(samples, self._provider)
        for key, height in zip(outstanding, heights):
            self._values[key] = float(height)

    def values(self, keys: Iterable[tuple[int, int]]) -> list[float]:
        return [self._values.get(key, 0.0) for key in keys]


def _route_ground_stats(
    route: Sequence[tuple[float, float]],
    sampler: "_TerrainSampler",
    *,
    spacing_m: float = 100.0,
) -> tuple[float, float]:
    """Mean and highest terrain under a polyline, sampled at a fixed spacing."""

    points: list[tuple[float, float]] = []
    for left, right in zip(route, route[1:]):
        steps = max(2, int(math.ceil(_distance_m(left, right) / max(10.0, spacing_m))))
        for index in range(steps):
            ratio = float(index) / float(steps)
            points.append(
                (
                    float(left[0]) + (float(right[0]) - float(left[0])) * ratio,
                    float(left[1]) + (float(right[1]) - float(left[1])) * ratio,
                )
            )
    points.append((float(route[-1][0]), float(route[-1][1])))
    keys = sampler.request(points)
    sampler.resolve()
    values = sampler.values(keys) or [0.0]
    return sum(values) / float(len(values)), max(values)


def _cheapest_corridor_lanes(
    layers: Sequence[Sequence[dict[str, Any]]],
    incoming: Sequence[Sequence[Sequence[tuple[int, dict[str, Any]]]]],
    length_weight: float,
    peak_weight: float,
) -> tuple[int, ...] | None:
    """Cheapest node index per station across the lattice, for one weighting.

    ``incoming`` (from :func:`_corridor_incoming_edges`) lists the few lanes
    that can reach each node, so a wide corridor costs O(lanes x lane-steps)
    per station instead of O(lanes^2).
    """

    # Parent pointers instead of per-node history lists: copying a growing
    # history on every relaxation is quadratic in route length.
    costs: dict[int, float] = {0: 0.0}
    parents: list[dict[int, int]] = [{0: -1}]
    length_weight = float(length_weight)
    peak_weight = float(peak_weight)
    for layer_index in range(1, len(layers)):
        next_costs: dict[int, float] = {}
        layer_parents: dict[int, int] = {}
        for current_index in range(len(layers[layer_index])):
            best_cost: float | None = None
            best_parent = -1
            for previous_index, edge in incoming[layer_index][current_index]:
                previous_cost = costs.get(previous_index)
                if previous_cost is None:
                    continue
                candidate_cost = (
                    previous_cost
                    + length_weight * edge["length_m"]
                    + _LOW_TERRAIN_MEAN_WEIGHT * edge["mean_ground"]
                    + peak_weight * edge["peak_ground"]
                    + edge["turn_cost"]
                )
                if best_cost is None or candidate_cost < best_cost:
                    best_cost = candidate_cost
                    best_parent = previous_index
            if best_cost is not None:
                next_costs[current_index] = best_cost
                layer_parents[current_index] = best_parent
        if not next_costs:
            return None
        costs = next_costs
        parents.append(layer_parents)

    node = min(costs, key=costs.get)
    lanes: list[int] = []
    for layer_parents in reversed(parents):
        lanes.append(node)
        node = layer_parents[node]
    lanes.reverse()
    return tuple(lanes)


def _route_length_m(route: Sequence[tuple[float, float]]) -> float:
    return sum(_distance_m(left, right) for left, right in zip(route, route[1:]))


def _cross_track_m(
    point: tuple[float, float],
    start: tuple[float, float],
    end: tuple[float, float],
) -> float:
    """Perpendicular distance from ``point`` to the ``start``-``end`` chord."""

    metres_per_lat = 111_132.92
    metres_per_lon = max(
        1.0, metres_per_lat * math.cos(math.radians((start[0] + end[0]) * 0.5))
    )
    ax = (point[1] - start[1]) * metres_per_lon
    ay = (point[0] - start[0]) * metres_per_lat
    bx = (end[1] - start[1]) * metres_per_lon
    by = (end[0] - start[0]) * metres_per_lat
    norm = math.hypot(bx, by)
    if norm <= 1e-6:
        return math.hypot(ax, ay)
    return abs(ax * by - ay * bx) / norm


def _simplify_horizontal_route(
    route: Sequence[tuple[float, float]],
    tolerance_m: float,
) -> list[tuple[float, float]]:
    """Douglas-Peucker over a horizontal route, endpoints always preserved."""

    if len(route) <= 2 or tolerance_m <= 0.0:
        return list(route)
    keep = {0, len(route) - 1}
    pending = [(0, len(route) - 1)]
    while pending:
        left, right = pending.pop()
        if right - left <= 1:
            continue
        worst_index = -1
        worst_m = tolerance_m
        for index in range(left + 1, right):
            offset_m = _cross_track_m(route[index], route[left], route[right])
            if offset_m > worst_m:
                worst_m = offset_m
                worst_index = index
        if worst_index < 0:
            continue
        keep.add(worst_index)
        pending.append((left, worst_index))
        pending.append((worst_index, right))
    return [route[index] for index in sorted(keep)]


def _mean_route_ground_m(
    route: Sequence[tuple[float, float]],
    terrain_provider: Callable[[Iterable[Any]], Iterable[float]] | None,
    *,
    spacing_m: float = 100.0,
) -> float:
    """Mean terrain height along a polyline, sampled at a fixed spacing."""

    samples: list[dict[str, float]] = []
    for left, right in zip(route, route[1:]):
        leg_m = _distance_m(left, right)
        steps = max(2, int(math.ceil(leg_m / max(10.0, float(spacing_m)))))
        for index in range(steps + 1):
            ratio = float(index) / float(steps)
            samples.append(
                {
                    "latitude": float(left[0]) + (float(right[0]) - float(left[0])) * ratio,
                    "longitude": float(left[1]) + (float(right[1]) - float(left[1])) * ratio,
                    "cum_m": 0.0,
                }
            )
    if not samples:
        return float("inf")
    values = [float(value) for value in _load_terrain_profile(samples, terrain_provider)]
    if not values:
        return float("inf")
    return sum(values) / float(len(values))


def _prefer_low_terrain_horizontal_route(
    route: Sequence[tuple[float, float]],
    *,
    terrain_provider: Callable[[Iterable[Any]], Iterable[float]] | None,
    corridor_width_m: float,
    min_leg_m: float,
    stage_spacing_m: float,
    max_stages: int,
    edge_samples: int,
    segment_allowed: Callable[
        [tuple[float, float], tuple[float, float]], bool
    ]
    | None,
    constrained_leg_start_index: int,
    runtime_payload: dict[str, Any] | None = None,
) -> list[tuple[float, float]]:
    if len(route) < 2:
        return list(route)
    selected: list[tuple[float, float]] = [route[0]]
    first_constrained_leg = max(0, int(constrained_leg_start_index))
    # Resolved once per route: reading the dial goes through the runtime
    # settings payload, which is too expensive to repeat per leg.
    if runtime_payload is None:
        strength = _low_terrain_strength()
    else:
        try:
            strength = _low_terrain_strength(runtime_payload)
        except TypeError:
            # Preserve compatibility with diagnostic/test hooks that replaced
            # the historical zero-argument resolver.
            strength = _low_terrain_strength()
    for leg_index, (start, end) in enumerate(zip(route, route[1:])):
        if leg_index < first_constrained_leg:
            # Live-position/previous-mission ingress has no applicable mission
            # geometry.  Preserve that leg instead of making a free side detour.
            leg = [start, end]
        else:
            leg = _low_terrain_route_for_leg(
                start,
                end,
                terrain_provider=terrain_provider,
                corridor_width_m=corridor_width_m,
                min_leg_m=min_leg_m,
                stage_spacing_m=stage_spacing_m,
                max_stages=max_stages,
                edge_samples=edge_samples,
                segment_allowed=segment_allowed,
                strength=strength,
            )
        for point in leg[1:]:
            if _distance_m(selected[-1], point) > 0.01:
                selected.append(point)
    return selected


def _simplify_profile_interval(
    samples: list[dict[str, float]],
    required_altitudes: list[float],
    start_index: int,
    end_index: int,
    *,
    max_waypoint_spacing_m: float,
    max_profile_excess_m: float,
) -> list[int]:
    if end_index <= start_index:
        return [start_index]
    selected = {start_index, end_index}
    pending: list[tuple[int, int]] = [(start_index, end_index)]
    max_spacing_m = max(25.0, float(max_waypoint_spacing_m))
    max_excess_m = max(0.0, float(max_profile_excess_m))

    while pending:
        left, right = pending.pop()
        if right - left <= 1:
            continue
        start_distance = float(samples[left]["cum_m"])
        end_distance = float(samples[right]["cum_m"])
        segment_distance = end_distance - start_distance
        if segment_distance <= 1e-6:
            continue
        start_alt = float(required_altitudes[left])
        end_alt = float(required_altitudes[right])

        worst_deficit = 0.0
        worst_deficit_index: int | None = None
        worst_excess = 0.0
        worst_excess_index: int | None = None
        for index in range(left + 1, right):
            fraction = (float(samples[index]["cum_m"]) - start_distance) / segment_distance
            line_alt = start_alt + (end_alt - start_alt) * fraction
            delta = float(required_altitudes[index]) - line_alt
            if delta > worst_deficit:
                worst_deficit = delta
                worst_deficit_index = index
            if -delta > worst_excess:
                worst_excess = -delta
                worst_excess_index = index

        split_index: int | None = None
        # Any meaningful profile penetration wins over waypoint-count reduction.
        if worst_deficit_index is not None and worst_deficit > 0.01:
            split_index = worst_deficit_index
        elif worst_excess_index is not None and worst_excess > max_excess_m:
            split_index = worst_excess_index
        elif segment_distance > max_spacing_m:
            midpoint_m = start_distance + segment_distance / 2.0
            split_index = min(
                range(left + 1, right),
                key=lambda index: abs(float(samples[index]["cum_m"]) - midpoint_m),
            )

        if split_index is None or split_index <= left or split_index >= right:
            continue
        selected.add(split_index)
        pending.append((left, split_index))
        pending.append((split_index, right))

    return sorted(selected)


def _select_nominal_spacing_indices(
    samples: list[dict[str, float]],
    anchor_indices: list[int],
    *,
    waypoint_spacing_m: float,
) -> list[int]:
    """Select route vertices at approximately the requested horizontal spacing."""

    target_spacing_m = max(50.0, float(waypoint_spacing_m))
    cumulative_distances = [float(sample["cum_m"]) for sample in samples]
    selected: set[int] = set()
    if len(anchor_indices) == 1:
        return [anchor_indices[0]]
    for start_index, end_index in zip(anchor_indices, anchor_indices[1:]):
        selected.add(start_index)
        selected.add(end_index)
        start_m = float(samples[start_index]["cum_m"])
        end_m = float(samples[end_index]["cum_m"])
        leg_m = max(0.0, end_m - start_m)
        if leg_m <= 1e-6 or end_index - start_index <= 1:
            continue
        segment_count = max(1, int(round(leg_m / target_spacing_m)))
        for part in range(1, segment_count):
            target_m = start_m + leg_m * (float(part) / float(segment_count))
            insertion = bisect_left(
                cumulative_distances,
                target_m,
                start_index + 1,
                end_index,
            )
            candidates = [
                index
                for index in (insertion - 1, insertion)
                if start_index < index < end_index
            ]
            if candidates:
                selected.add(
                    min(candidates, key=lambda index: abs(cumulative_distances[index] - target_m))
                )
    return sorted(selected)


def _limit_adaptive_profile_indices(
    samples: list[dict[str, float]],
    selected_indices: list[int],
    anchor_indices: list[int],
    *,
    preferred_limit: int,
    max_waypoint_spacing_m: float,
) -> list[int]:
    """Bound ordinary-route WP count without weakening terrain clearance.

    The emitted segment altitudes are certified against every dense sample
    below, so a reduced index set remains collision-safe.  We retain every
    source-route corner and bisect the largest remaining horizontal gap until
    both the preferred budget and the hard spacing rule are satisfied.
    """

    selected = sorted(set(int(index) for index in selected_indices))
    if len(selected) <= max(2, int(preferred_limit)):
        return selected
    selected_set = set(selected)
    retained = {
        int(index) for index in anchor_indices if int(index) in selected_set
    }
    retained.update((selected[0], selected[-1]))
    budget = max(2, int(preferred_limit), len(retained))
    hard_spacing_m = max(25.0, float(max_waypoint_spacing_m))

    while True:
        ordered = sorted(retained)
        gaps = [
            (
                float(samples[right]["cum_m"]) - float(samples[left]["cum_m"]),
                left,
                right,
            )
            for left, right in zip(ordered, ordered[1:])
        ]
        largest_gap = max(gaps, default=(0.0, selected[0], selected[-1]))
        if len(retained) >= budget and float(largest_gap[0]) <= hard_spacing_m + 1e-6:
            break
        _, left, right = largest_gap
        candidates = [index for index in selected if left < index < right and index not in retained]
        if not candidates:
            # All adaptive points are already retained; a route with more than
            # the preferred budget is allowed only when source corners or the
            # hard spacing rule require it.
            break
        midpoint_m = (
            float(samples[left]["cum_m"]) + float(samples[right]["cum_m"])
        ) / 2.0
        retained.add(
            min(
                candidates,
                key=lambda index: abs(float(samples[index]["cum_m"]) - midpoint_m),
            )
        )
    return sorted(retained)


def _altitude_smoothing_config(
    runtime_payload: dict[str, Any] | None = None,
) -> tuple[bool, float, float, float]:
    """Resolve the LAH vertical-profile knobs once per generated route."""

    try:
        from modules.mission_planning.MissionPlanner.runtime_settings import (
            get_runtime_bool,
            get_runtime_float,
        )

        enabled = bool(
            get_runtime_bool(
                "lah_altitude_smoothing_enabled",
                True,
                runtime_payload,
            )
        )
        max_dip_depth_m = float(
            get_runtime_float(
                "lah_altitude_short_dip_max_depth_m",
                30.0,
                runtime_payload,
            )
        )
        max_dip_span_m = float(
            get_runtime_float(
                "lah_altitude_short_dip_max_span_m",
                1200.0,
                runtime_payload,
            )
        )
        redundant_tolerance_m = float(
            get_runtime_float(
                "lah_altitude_redundant_tolerance_m",
                3.0,
                runtime_payload,
            )
        )
    except Exception:
        return True, 30.0, 1200.0, 3.0
    if not math.isfinite(max_dip_depth_m):
        max_dip_depth_m = 30.0
    if not math.isfinite(max_dip_span_m):
        max_dip_span_m = 1200.0
    if not math.isfinite(redundant_tolerance_m):
        redundant_tolerance_m = 3.0
    return (
        bool(enabled),
        min(max(0.0, max_dip_depth_m), 500.0),
        min(max(0.0, max_dip_span_m), 20_000.0),
        min(max(0.0, redundant_tolerance_m), 100.0),
    )


def _fill_short_altitude_dips(
    samples: list[dict[str, float]],
    selected_indices: list[int],
    selected_altitudes: dict[int, float],
    *,
    max_depth_m: float,
    max_span_m: float,
) -> None:
    """Raise short valleys to the chord between their surrounding high points."""

    if len(selected_indices) < 3 or max_depth_m <= 0.0 or max_span_m <= 0.0:
        return
    ordered = list(selected_indices)
    # A second pass lets two neighbouring low points become one smooth bridge
    # after the shorter sub-valleys were lifted on the first pass.
    for _pass in range(2):
        changed = False
        for left_pos in range(len(ordered) - 2):
            left_index = ordered[left_pos]
            left_m = float(samples[left_index]["cum_m"])
            for right_pos in range(left_pos + 2, len(ordered)):
                right_index = ordered[right_pos]
                right_m = float(samples[right_index]["cum_m"])
                span_m = right_m - left_m
                if span_m > max_span_m + 1e-6:
                    break
                if span_m <= 1e-6:
                    continue
                left_alt_m = float(selected_altitudes[left_index])
                right_alt_m = float(selected_altitudes[right_index])
                lifts: list[tuple[int, float]] = []
                valid_bridge = True
                for middle_pos in range(left_pos + 1, right_pos):
                    middle_index = ordered[middle_pos]
                    fraction = (
                        float(samples[middle_index]["cum_m"]) - left_m
                    ) / span_m
                    chord_alt_m = left_alt_m + (
                        right_alt_m - left_alt_m
                    ) * fraction
                    current_alt_m = float(selected_altitudes[middle_index])
                    lift_m = chord_alt_m - current_alt_m
                    if lift_m < -1e-6 or lift_m > max_depth_m + 1e-6:
                        valid_bridge = False
                        break
                    if lift_m > 1e-6:
                        lifts.append((middle_index, chord_alt_m))
                if not valid_bridge or not lifts:
                    continue
                for middle_index, chord_alt_m in lifts:
                    selected_altitudes[middle_index] = max(
                        float(selected_altitudes[middle_index]),
                        float(chord_alt_m),
                    )
                changed = True
        if not changed:
            break


def _smooth_altitudes_for_vertical_rates(
    samples: list[dict[str, float]],
    selected_indices: list[int],
    selected_altitudes: dict[int, float],
    *,
    cruise_speed_mps: float,
) -> None:
    """Raise the profile into climb/descent-rate-feasible linear ramps."""

    if len(selected_indices) < 2:
        return
    speed_mps = max(1.0, float(cruise_speed_mps))
    climb_rate_mps = max(
        0.1,
        float(getattr(DEFAULT_ENVELOPE, "climb_rate_mps", 8.9))
        * float(LAH_VERTICAL_RATE_USE_RATIO),
    )
    descent_rate_mps = max(
        0.1,
        float(getattr(DEFAULT_ENVELOPE, "descent_rate_mps", 7.0))
        * float(LAH_VERTICAL_RATE_USE_RATIO),
    )
    climb_grade = climb_rate_mps / speed_mps
    descent_grade = descent_rate_mps / speed_mps

    # Raising one side can tighten the opposite direction on the next pass.
    # Two alternating passes converge for this one-dimensional max envelope.
    for _pass in range(2):
        for left_index, right_index in reversed(
            list(zip(selected_indices, selected_indices[1:]))
        ):
            distance_m = max(
                0.0,
                float(samples[right_index]["cum_m"])
                - float(samples[left_index]["cum_m"]),
            )
            selected_altitudes[left_index] = max(
                float(selected_altitudes[left_index]),
                float(selected_altitudes[right_index])
                - climb_grade * distance_m,
            )
        for left_index, right_index in zip(selected_indices, selected_indices[1:]):
            distance_m = max(
                0.0,
                float(samples[right_index]["cum_m"])
                - float(samples[left_index]["cum_m"]),
            )
            selected_altitudes[right_index] = max(
                float(selected_altitudes[right_index]),
                float(selected_altitudes[left_index])
                - descent_grade * distance_m,
            )


def _prune_redundant_altitude_indices(
    samples: list[dict[str, float]],
    required_altitudes: list[float],
    selected_indices: list[int],
    selected_altitudes: dict[int, float],
    protected_anchor_indices: list[int],
    *,
    max_waypoint_spacing_m: float,
    altitude_tolerance_m: float,
) -> list[int]:
    """Remove DEM-only points whose surrounding 3-D chord is already safe."""

    retained = list(selected_indices)
    protected = {int(index) for index in protected_anchor_indices}
    protected.update((retained[0], retained[-1]))
    hard_spacing_m = max(25.0, float(max_waypoint_spacing_m))
    tolerance_m = max(0.0, float(altitude_tolerance_m))

    def _cross_track_distance_m(
        left_index: int,
        middle_index: int,
        right_index: int,
    ) -> float:
        left = samples[left_index]
        middle = samples[middle_index]
        right = samples[right_index]
        mid_lat = (
            float(left["latitude"])
            + float(middle["latitude"])
            + float(right["latitude"])
        ) / 3.0
        metres_per_lon = _METRES_PER_DEGREE_LAT * math.cos(math.radians(mid_lat))
        ax = (
            float(middle["longitude"]) - float(left["longitude"])
        ) * metres_per_lon
        ay = (
            float(middle["latitude"]) - float(left["latitude"])
        ) * _METRES_PER_DEGREE_LAT
        bx = (
            float(right["longitude"]) - float(left["longitude"])
        ) * metres_per_lon
        by = (
            float(right["latitude"]) - float(left["latitude"])
        ) * _METRES_PER_DEGREE_LAT
        norm_sq = bx * bx + by * by
        if norm_sq <= 1e-9:
            return math.hypot(ax, ay)
        fraction = min(max((ax * bx + ay * by) / norm_sq, 0.0), 1.0)
        return math.hypot(ax - bx * fraction, ay - by * fraction)

    changed = True
    while changed and len(retained) >= 3:
        changed = False
        for position in range(1, len(retained) - 1):
            middle_index = retained[position]
            if middle_index in protected:
                continue
            left_index = retained[position - 1]
            right_index = retained[position + 1]
            left_m = float(samples[left_index]["cum_m"])
            right_m = float(samples[right_index]["cum_m"])
            span_m = right_m - left_m
            if span_m <= 1e-6 or span_m > hard_spacing_m + 1e-6:
                continue
            # DEM-only altitude vertices may be removed, but a genuine
            # horizontal valley/ridge detour must keep its bend.
            if (
                _cross_track_distance_m(left_index, middle_index, right_index)
                > 5.0
            ):
                continue
            left_alt_m = float(selected_altitudes[left_index])
            right_alt_m = float(selected_altitudes[right_index])
            middle_fraction = (
                float(samples[middle_index]["cum_m"]) - left_m
            ) / span_m
            middle_chord_alt_m = left_alt_m + (
                right_alt_m - left_alt_m
            ) * middle_fraction
            if (
                abs(float(selected_altitudes[middle_index]) - middle_chord_alt_m)
                > tolerance_m + 1e-6
            ):
                continue

            chord_is_safe = True
            for sample_index in range(left_index, right_index + 1):
                fraction = (
                    float(samples[sample_index]["cum_m"]) - left_m
                ) / span_m
                chord_alt_m = left_alt_m + (
                    right_alt_m - left_alt_m
                ) * fraction
                if chord_alt_m + 1e-6 < float(required_altitudes[sample_index]):
                    chord_is_safe = False
                    break
            if not chord_is_safe:
                continue
            retained.pop(position)
            changed = True
            break
    return retained


def build_lah_terrain_following_path(
    route_coordinates: Iterable[Any],
    *,
    clearance_m: float = LAH_LOW_LEVEL_CLEARANCE_M,
    sample_spacing_m: float = LAH_TERRAIN_SAMPLE_SPACING_M,
    max_waypoint_spacing_m: float = LAH_TERRAIN_MAX_WAYPOINT_SPACING_M,
    max_profile_excess_m: float = LAH_TERRAIN_MAX_PROFILE_EXCESS_M,
    terrain_provider: Callable[[Iterable[Any]], Iterable[float]] | None = None,
    prefer_low_terrain: bool = False,
    low_terrain_corridor_m: float = LAH_LOW_TERRAIN_CORRIDOR_M,
    low_terrain_min_leg_m: float = LAH_LOW_TERRAIN_MIN_LEG_M,
    low_terrain_stage_spacing_m: float = LAH_LOW_TERRAIN_STAGE_SPACING_M,
    low_terrain_max_stages: int = LAH_LOW_TERRAIN_MAX_STAGES,
    low_terrain_edge_samples: int = LAH_LOW_TERRAIN_EDGE_SAMPLES,
    low_terrain_segment_allowed: Callable[
        [tuple[float, float], tuple[float, float]], bool
    ]
    | None = None,
    low_terrain_constrained_leg_start_index: int = 0,
    cruise_speed_mps: float = 40.0,
    runtime_payload: dict[str, Any] | None = None,
) -> list[dict[str, float | int]]:
    """Build a compact DEM-following LAH route.

    DEM is loaded once in a dense batch.  Output waypoints are then selected by
    terrain shape: a point is added when the straight 3-D chord would penetrate
    the required DEM clearance, when the chord would float excessively above
    the terrain profile, or when the safety maximum spacing is exceeded.  Short
    shallow descents are bridged, unavoidable climbs/descents are lifted into
    rate-feasible ramps, and redundant DEM-only altitude vertices are removed
    only after the replacement chord is checked against every dense DEM sample.
    Input route corners and genuine horizontal detours are always kept.
    """

    route: list[tuple[float, float]] = []
    for raw in route_coordinates or []:
        point = _to_lat_lon(raw)
        if point is None:
            continue
        if route and _distance_m(route[-1], point) <= 0.01:
            continue
        route.append(point)
    if not route:
        return []

    source_route = list(route)
    if prefer_low_terrain and len(route) >= 2:
        route = _prefer_low_terrain_horizontal_route(
            route,
            terrain_provider=terrain_provider,
            corridor_width_m=low_terrain_corridor_m,
            min_leg_m=low_terrain_min_leg_m,
            stage_spacing_m=low_terrain_stage_spacing_m,
            max_stages=low_terrain_max_stages,
            edge_samples=low_terrain_edge_samples,
            segment_allowed=low_terrain_segment_allowed,
            constrained_leg_start_index=low_terrain_constrained_leg_start_index,
            runtime_payload=runtime_payload,
        )

    samples, anchor_indices = _sample_horizontal_route(
        route,
        sample_spacing_m=sample_spacing_m,
    )
    protected_anchor_indices = [
        int(anchor_indices[route_index])
        for route_index, route_point in enumerate(route)
        if route_index < len(anchor_indices)
        and any(
            _distance_m(route_point, source_point) <= 0.5
            for source_point in source_route
        )
    ]
    terrain_values = _load_terrain_profile(samples, terrain_provider)
    clearance = max(0.0, float(clearance_m))
    required_altitudes = [float(ground) + clearance for ground in terrain_values]

    selected_index_set: set[int] = set()
    if len(anchor_indices) == 1:
        selected_index_set.add(anchor_indices[0])
    else:
        for start_index, end_index in zip(anchor_indices, anchor_indices[1:]):
            selected_index_set.update(
                _simplify_profile_interval(
                    samples,
                    required_altitudes,
                    start_index,
                    end_index,
                    max_waypoint_spacing_m=max_waypoint_spacing_m,
                    max_profile_excess_m=max_profile_excess_m,
                )
            )
    selected_indices = sorted(selected_index_set)
    selected_indices = _limit_adaptive_profile_indices(
        samples,
        selected_indices,
        anchor_indices,
        preferred_limit=LAH_TERRAIN_MAX_OUTPUT_WAYPOINTS,
        max_waypoint_spacing_m=max_waypoint_spacing_m,
    )

    # A raster ridge may begin between two dense samples.  Raising both ends
    # of every emitted segment to at least that interval's sampled peak keeps
    # the whole straight 3-D chord above the DEM floor, rather than asking the
    # aircraft to reach ridge altitude only at the first high sample.
    selected_altitudes = {
        index: float(required_altitudes[index]) for index in selected_indices
    }
    for left_index, right_index in zip(selected_indices, selected_indices[1:]):
        interval_peak_m = max(required_altitudes[left_index : right_index + 1])
        selected_altitudes[left_index] = max(
            selected_altitudes[left_index],
            float(interval_peak_m),
        )
        selected_altitudes[right_index] = max(
            selected_altitudes[right_index],
            float(interval_peak_m),
        )

    (
        altitude_smoothing_enabled,
        max_dip_depth_m,
        max_dip_span_m,
        redundant_tolerance_m,
    ) = _altitude_smoothing_config(runtime_payload)
    if altitude_smoothing_enabled:
        _fill_short_altitude_dips(
            samples,
            selected_indices,
            selected_altitudes,
            max_depth_m=max_dip_depth_m,
            max_span_m=max_dip_span_m,
        )
        _smooth_altitudes_for_vertical_rates(
            samples,
            selected_indices,
            selected_altitudes,
            cruise_speed_mps=cruise_speed_mps,
        )
        selected_indices = _prune_redundant_altitude_indices(
            samples,
            required_altitudes,
            selected_indices,
            selected_altitudes,
            protected_anchor_indices,
            max_waypoint_spacing_m=max_waypoint_spacing_m,
            altitude_tolerance_m=redundant_tolerance_m,
        )

    result: list[dict[str, float | int]] = []
    for index in selected_indices:
        sample = samples[index]
        result.append(
            {
                "latitude": round(float(sample["latitude"]), 7),
                "longitude": round(float(sample["longitude"]), 7),
                # Rounding upward preserves the sampled clearance guarantee.
                "altitude": int(math.ceil(selected_altitudes[index])),
                "groundElevation": float(terrain_values[index]),
                "cum_m": float(sample["cum_m"]),
            }
        )
    return result


__all__ = [
    "LAH_LOW_LEVEL_CLEARANCE_M",
    "LAH_TERRAIN_SAMPLE_SPACING_M",
    "LAH_TERRAIN_MAX_WAYPOINT_SPACING_M",
    "LAH_TERRAIN_MAX_PROFILE_EXCESS_M",
    "LAH_TERRAIN_MAX_OUTPUT_WAYPOINTS",
    "LAH_VERTICAL_RATE_USE_RATIO",
    "LAH_LOW_TERRAIN_MIN_LEG_M",
    "LAH_LOW_TERRAIN_CORRIDOR_M",
    "LAH_LOW_TERRAIN_STAGE_SPACING_M",
    "LAH_LOW_TERRAIN_MAX_STAGES",
    "LAH_LOW_TERRAIN_MAX_LENGTH_RATIO",
    "LAH_LOW_TERRAIN_EDGE_SAMPLES",
    "LAH_LOW_TERRAIN_REFINE_PASSES",
    "build_lah_terrain_following_path",
]
