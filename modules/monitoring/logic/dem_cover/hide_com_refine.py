"""Native-resolution second pass for the joint hide/communication solver.

The full mission-area search remains on the coarse display grid.  This module
rechecks a small window around that result with the source DEM cells, while
also retaining the exact ownship click as a distance-zero candidate.  An
optional, independently bounded native-cell disk around ownship can be added
for route endpoint discovery without changing the existing recommendation.
LOS rays are sampled from the native raster and processed in bounded chunks
so a 10 m DEM does not require full-size coordinate meshes or huge temporary
arrays.
"""

from __future__ import annotations

import math
import time
from typing import Any

import numpy as np

from .config import CoverConfig
from .hide_com import (
    CommunicationHideAnalyzer,
    CommunicationHideResult,
    HideEndpointCandidate,
    UavPoint,
    _required_target_altitude_batch,
)
from .sampling import EnemyPoint


_ALTITUDE_MARGIN_M = 0.25

# The route stage re-verifies concealment over its own sample batch, and an
# observer's step count is only invariant within a single call.  The same
# (enemy, point) pair therefore yields thresholds that differ by a fraction of
# a metre between the two stages.  Because the preferred altitude is clipped to
# the ceiling, an endpoint selected exactly at it lands on the boundary and the
# verifier rejects every candidate, so no concealment plan is ever issued.
# Selecting strictly below the ceiling keeps an accepted endpoint verifiable;
# the cost is a slightly lower hide altitude, which is the conservative
# direction for concealment.  Measured cross-stage spread is ~0.5 m; 2 m was
# enough to push the arrival altitude interval out of the route's reach, so
# this keeps 2x headroom over the observed spread and no more.
_ENEMY_CEILING_SELECTION_MARGIN_M = 1.0


def _fractional_cell(dem: Any, x: float, y: float) -> tuple[float, float]:
    """Return row/col in the LOS kernel's cell-centre coordinate system."""

    row_edge, col_edge = dem.native_to_rowcol(float(x), float(y))
    return float(row_edge) - 0.5, float(col_edge) - 0.5


def _valid_native_cell(dem: Any, row: int, col: int) -> bool:
    return bool(
        0 <= int(row) < int(dem.height)
        and 0 <= int(col) < int(dem.width)
        and np.isfinite(dem.block_elev[int(row), int(col)])
        and float(dem.block_elev[int(row), int(col)]) >= -1000.0
    )


def _native_uav(
    dem: Any,
    source: UavPoint,
    *,
    agl_m: float | None,
) -> UavPoint:
    x, y = float(source.x), float(source.y)
    ground_m = float(dem.sample_native(x, y))
    nearest_row, nearest_col = dem.nearest_index(x, y)
    if not np.isfinite(ground_m) or not _valid_native_cell(
        dem, nearest_row, nearest_col
    ):
        raise ValueError("A UAV click falls on invalid terrain in the native DEM.")
    row, col = _fractional_cell(dem, x, y)
    lat, lon = dem.native_to_latlon(x, y)
    altitude_m = (
        float(source.alt_m)
        if agl_m is None
        else ground_m + max(1.0, float(agl_m))
    )
    if not np.isfinite(altitude_m) or altitude_m <= ground_m:
        raise ValueError(
            "UAV MSL altitude must be above native DEM terrain "
            f"({altitude_m:.1f} <= {ground_m:.1f} m)."
        )
    return UavPoint(
        x=x,
        y=y,
        row=float(row),
        col=float(col),
        ground_m=ground_m,
        # The interactive prototype supplies a common clicked-UAV AGL.  The
        # operational planner passes ``None`` so the live 0401 MSL altitude is
        # preserved instead of silently replacing it with DEM + 1000 m.
        alt_m=altitude_m,
        lat=float(lat),
        lon=float(lon),
    )


def _native_enemy(
    dem: Any,
    source: EnemyPoint,
    *,
    height_m: float,
) -> EnemyPoint:
    x, y = float(source.x), float(source.y)
    ground_m = float(dem.sample_native(x, y))
    nearest_row, nearest_col = dem.nearest_index(x, y)
    if not np.isfinite(ground_m) or not _valid_native_cell(
        dem, nearest_row, nearest_col
    ):
        raise ValueError("An enemy click falls on invalid terrain in the native DEM.")
    row, col = _fractional_cell(dem, x, y)
    lat, lon = dem.native_to_latlon(x, y)
    return EnemyPoint(
        x=x,
        y=y,
        row=float(row),
        col=float(col),
        ground_m=ground_m,
        alt_m=ground_m + max(0.0, float(height_m)),
        lat=float(lat),
        lon=float(lon),
    )


def _native_disk_candidates(
    dem: Any,
    *,
    center_x: float,
    center_y: float,
    radius_m: float,
    own_x: float,
    own_y: float,
    search_radius_m: float,
) -> tuple[
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
]:
    """Build one local native-cell disk and its stable flat cell IDs."""

    center_x = float(center_x)
    center_y = float(center_y)
    center_row, center_col = _fractional_cell(dem, center_x, center_y)
    radius_m = max(0.0, float(radius_m))
    row_pad = int(
        np.ceil(radius_m / max(1e-6, abs(float(dem.cell_y_m))))
    ) + 2
    col_pad = int(
        np.ceil(radius_m / max(1e-6, abs(float(dem.cell_x_m))))
    ) + 2
    row_lo = max(0, int(np.floor(center_row)) - row_pad)
    row_hi = min(int(dem.height) - 1, int(np.ceil(center_row)) + row_pad)
    col_lo = max(0, int(np.floor(center_col)) - col_pad)
    col_hi = min(int(dem.width) - 1, int(np.ceil(center_col)) + col_pad)

    grid_rows, grid_cols = np.meshgrid(
        np.arange(row_lo, row_hi + 1, dtype=np.int32),
        np.arange(col_lo, col_hi + 1, dtype=np.int32),
        indexing="ij",
    )
    rows = grid_rows.ravel()
    cols = grid_cols.ravel()
    # Affine transform evaluated directly on the local arrays.  This avoids
    # NativeDemRaster ever holding two ~184 MiB float64 coordinate meshes.
    pixel_cols = cols.astype(np.float64) + 0.5
    pixel_rows = rows.astype(np.float64) + 0.5
    transform = dem.transform
    xs = transform.a * pixel_cols + transform.b * pixel_rows + transform.c
    ys = transform.d * pixel_cols + transform.e * pixel_rows + transform.f

    own_x = float(own_x)
    own_y = float(own_y)
    from_refinement_center_m = np.hypot(xs - center_x, ys - center_y)
    from_own_m = np.hypot(xs - own_x, ys - own_y)
    grounds_m = dem.block_elev[rows, cols].astype(np.float64, copy=False)
    keep = (
        np.isfinite(grounds_m)
        & (grounds_m >= -1000.0)
        & (from_refinement_center_m <= radius_m)
        & (from_own_m <= float(search_radius_m))
    )
    flat_cell_ids = (
        rows[keep].astype(np.int64, copy=False) * int(dem.width)
        + cols[keep].astype(np.int64, copy=False)
    )
    rows = rows[keep].astype(np.float64, copy=False)
    cols = cols[keep].astype(np.float64, copy=False)
    xs = xs[keep].astype(np.float64, copy=False)
    ys = ys[keep].astype(np.float64, copy=False)
    grounds_m = grounds_m[keep].astype(np.float64, copy=False)
    distances_m = from_own_m[keep].astype(np.float64, copy=False)
    return rows, cols, xs, ys, grounds_m, distances_m, flat_cell_ids


def _native_candidates(
    dem: Any,
    coarse_result: CommunicationHideResult,
    *,
    refinement_radius_m: float,
    search_radius_m: float,
    own_reachable_radius_m: float = 0.0,
) -> tuple[
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    int,
]:
    """Build primary and optional route candidates from two small local disks."""

    own_x = float(coarse_result.own_x)
    own_y = float(coarse_result.own_y)
    refinement_radius_m = max(float(dem.cell_m), float(refinement_radius_m))
    (
        rows,
        cols,
        xs,
        ys,
        grounds_m,
        distances_m,
        primary_cell_ids,
    ) = _native_disk_candidates(
        dem,
        center_x=float(coarse_result.recommended_x),
        center_y=float(coarse_result.recommended_y),
        radius_m=refinement_radius_m,
        own_x=own_x,
        own_y=own_y,
        search_radius_m=search_radius_m,
    )

    # Always retain the operator's exact XY.  It may be outside the local
    # window when the coarse pass moved far away, yet native terrain may show
    # that an altitude-only change is already feasible.
    own_ground_m = float(dem.sample_native(own_x, own_y))
    own_nearest_row, own_nearest_col = dem.nearest_index(own_x, own_y)
    if not np.isfinite(own_ground_m) or not _valid_native_cell(
        dem, own_nearest_row, own_nearest_col
    ):
        raise ValueError("Own position is on invalid terrain in the native DEM.")
    own_row, own_col = _fractional_cell(dem, own_x, own_y)
    rows = np.concatenate((rows, np.asarray([own_row], dtype=np.float64)))
    cols = np.concatenate((cols, np.asarray([own_col], dtype=np.float64)))
    xs = np.concatenate((xs, np.asarray([own_x], dtype=np.float64)))
    ys = np.concatenate((ys, np.asarray([own_y], dtype=np.float64)))
    grounds_m = np.concatenate(
        (grounds_m, np.asarray([own_ground_m], dtype=np.float64))
    )
    distances_m = np.concatenate((distances_m, np.asarray([0.0], dtype=np.float64)))
    own_index = int(rows.size) - 1
    primary_mask = np.ones(rows.size, dtype=bool)

    # The route-only disk is generated from its own local bounding box.  Do
    # not allocate the potentially large rectangle spanning ownship and the
    # coarse recommendation. Native/native overlap is removed by stable flat
    # cell ID; the exact fractional ownship candidate remains intentionally.
    reachable_radius_m = max(0.0, float(own_reachable_radius_m))
    if reachable_radius_m > 0.0:
        (
            reachable_rows,
            reachable_cols,
            reachable_xs,
            reachable_ys,
            reachable_grounds_m,
            reachable_distances_m,
            reachable_cell_ids,
        ) = _native_disk_candidates(
            dem,
            center_x=own_x,
            center_y=own_y,
            radius_m=reachable_radius_m,
            own_x=own_x,
            own_y=own_y,
            search_radius_m=search_radius_m,
        )
        if reachable_cell_ids.size and primary_cell_ids.size:
            route_only = ~np.isin(
                reachable_cell_ids,
                primary_cell_ids,
                assume_unique=True,
            )
        else:
            route_only = np.ones(reachable_cell_ids.size, dtype=bool)
        if np.any(route_only):
            rows = np.concatenate((rows, reachable_rows[route_only]))
            cols = np.concatenate((cols, reachable_cols[route_only]))
            xs = np.concatenate((xs, reachable_xs[route_only]))
            ys = np.concatenate((ys, reachable_ys[route_only]))
            grounds_m = np.concatenate(
                (grounds_m, reachable_grounds_m[route_only])
            )
            distances_m = np.concatenate(
                (distances_m, reachable_distances_m[route_only])
            )
            primary_mask = np.concatenate(
                (
                    primary_mask,
                    np.zeros(int(np.count_nonzero(route_only)), dtype=bool),
                )
            )
    return (
        rows,
        cols,
        xs,
        ys,
        grounds_m,
        distances_m,
        primary_mask,
        own_index,
    )


def _observer_step_count(
    observer: UavPoint | EnemyPoint,
    *,
    rows: np.ndarray,
    cols: np.ndarray,
    xs: np.ndarray,
    ys: np.ndarray,
    max_distance_m: float,
    config: CoverConfig,
) -> tuple[np.ndarray, int]:
    distances_m = np.hypot(xs - float(observer.x), ys - float(observer.y))
    indices = np.nonzero(distances_m <= float(max_distance_m))[0]
    if indices.size == 0:
        return indices, 2
    spans = np.abs(rows[indices] - float(observer.row)) + np.abs(
        cols[indices] - float(observer.col)
    )
    steps = int(
        np.clip(
            int(float(np.max(spans)) * float(config.los_samples_per_cell)) + 1,
            2,
            int(config.los_max_steps),
        )
    )
    return indices, steps


def _ray_known_batch(
    block_elev: np.ndarray,
    *,
    observer_row: float,
    observer_col: float,
    target_rows: np.ndarray,
    target_cols: np.ndarray,
    steps: int,
) -> np.ndarray:
    """Whether every sampled interior cell has known terrain."""

    count = int(target_rows.size)
    if count == 0:
        return np.zeros(0, dtype=bool)
    if int(steps) <= 2:
        return np.ones(count, dtype=bool)
    t = np.linspace(0.0, 1.0, int(steps), dtype=np.float64).reshape(-1, 1)
    rr = float(observer_row) + t * (
        target_rows.astype(np.float64, copy=False) - float(observer_row)
    )
    cc = float(observer_col) + t * (
        target_cols.astype(np.float64, copy=False) - float(observer_col)
    )
    height, width = block_elev.shape
    ri = np.clip(np.rint(rr).astype(np.int32), 0, height - 1)
    ci = np.clip(np.rint(cc).astype(np.int32), 0, width - 1)
    terrain = block_elev[ri, ci]
    return np.all(np.isfinite(terrain[1:-1]) & (terrain[1:-1] >= -1000.0), axis=0)


def _observer_requirements_chunked(
    dem: Any,
    config: CoverConfig,
    observer: UavPoint | EnemyPoint,
    *,
    rows: np.ndarray,
    cols: np.ndarray,
    xs: np.ndarray,
    ys: np.ndarray,
    max_distance_m: float,
    chunk_size: int,
    reject_nodata: bool = False,
) -> np.ndarray:
    """Compute thresholds with a chunk-invariant step count per observer."""

    distances_m = np.hypot(xs - float(observer.x), ys - float(observer.y))
    required_m = np.full(rows.size, np.inf, dtype=np.float64)
    indices, steps = _observer_step_count(
        observer,
        rows=rows,
        cols=cols,
        xs=xs,
        ys=ys,
        max_distance_m=max_distance_m,
        config=config,
    )
    if indices.size == 0:
        return required_m
    batch = max(16, int(chunk_size))
    for start in range(0, int(indices.size), batch):
        selected = indices[start : start + batch]
        required_m[selected] = _required_target_altitude_batch(
            dem.block_elev,
            observer_row=float(observer.row),
            observer_col=float(observer.col),
            observer_altitude_m=float(observer.alt_m),
            target_rows=rows[selected],
            target_cols=cols[selected],
            horizontal_distances_m=distances_m[selected],
            config=config,
            steps_override=steps,
        )
        if reject_nodata:
            known = _ray_known_batch(
                dem.block_elev,
                observer_row=float(observer.row),
                observer_col=float(observer.col),
                target_rows=rows[selected],
                target_cols=cols[selected],
                steps=steps,
            )
            required_m[selected[~known]] = np.inf
    return required_m


def _uav_ray_known_at_point(
    dem: Any,
    config: CoverConfig,
    uav: UavPoint,
    *,
    row: float,
    col: float,
    x: float,
    y: float,
    communication_range_m: float,
    step_reference_rows: np.ndarray,
    step_reference_cols: np.ndarray,
    step_reference_xs: np.ndarray,
    step_reference_ys: np.ndarray,
) -> bool:
    if float(np.hypot(float(x) - uav.x, float(y) - uav.y)) > float(
        communication_range_m
    ):
        return False
    _indices, reference_steps = _observer_step_count(
        uav,
        rows=step_reference_rows,
        cols=step_reference_cols,
        xs=step_reference_xs,
        ys=step_reference_ys,
        max_distance_m=communication_range_m,
        config=config,
    )
    # A route-only ownship-disk endpoint may lie outside the primary refinement
    # window used above. Include this exact arrival ray in the resolution
    # bound, otherwise a farther ray can under-sample and skip a narrow nodata
    # cell even though its LOS result itself used the longer target span.
    target_span = abs(float(row) - float(uav.row)) + abs(
        float(col) - float(uav.col)
    )
    target_steps = int(
        np.clip(
            int(target_span * float(config.los_samples_per_cell)) + 1,
            2,
            int(config.los_max_steps),
        )
    )
    steps = max(int(reference_steps), target_steps)
    return bool(
        _ray_known_batch(
            dem.block_elev,
            observer_row=float(uav.row),
            observer_col=float(uav.col),
            target_rows=np.asarray([row], dtype=np.float64),
            target_cols=np.asarray([col], dtype=np.float64),
            steps=steps,
        )[0]
    )


def _select_fallback(
    *,
    own_altitude_m: float,
    terrain_floor_m: np.ndarray,
    altitude_ceiling_m: np.ndarray,
    candidate_distance_m: np.ndarray,
    enemy_requirements_m: np.ndarray,
    uav_requirements_m: np.ndarray,
    hide_safety_margin_m: float = 0.0,
) -> tuple[int, float]:
    """Preserve the coarse solver's degraded selection ordering exactly."""

    count = int(terrain_floor_m.size)
    event_altitudes = [
        terrain_floor_m,
        np.clip(
            np.full(count, float(own_altitude_m), dtype=np.float64),
            terrain_floor_m,
            altitude_ceiling_m,
        ),
    ]
    for uav_row in uav_requirements_m:
        event_altitudes.append(
            np.clip(
                uav_row + _ALTITUDE_MARGIN_M,
                terrain_floor_m,
                altitude_ceiling_m,
            )
        )

    best_altitude_m = terrain_floor_m.copy()
    best_keys: list[tuple[int, int, int, float] | None] = [None] * count
    for event_altitude_m in event_altitudes:
        enemy_visible, uav_links = CommunicationHideAnalyzer._counts_at_altitude(
            event_altitude_m,
            enemy_requirements_m,
            uav_requirements_m,
            float(hide_safety_margin_m),
        )
        for index in range(count):
            key = (
                0 if int(uav_links[index]) > 0 else 1,
                int(enemy_visible[index]),
                -int(uav_links[index]),
                abs(float(event_altitude_m[index]) - float(own_altitude_m)),
            )
            if best_keys[index] is None or key < best_keys[index]:
                best_keys[index] = key
                best_altitude_m[index] = float(event_altitude_m[index])

    def candidate_key(index: int) -> tuple:
        quality = best_keys[index] or (1, 999, 0, float("inf"))
        return (
            quality[0],
            quality[1],
            quality[2],
            float(candidate_distance_m[index]),
            quality[3],
            int(index),
        )

    selected = int(min(range(count), key=candidate_key))
    return selected, float(best_altitude_m[selected])


_ATTACK_TARGET_MATCH_TOLERANCE_DEG = 1e-4


def _attack_infeasibility_rank(
    precise_enemies: list,
    enemy_requirements_m: np.ndarray,
    *,
    attack_target_coordinate: dict | None,
    attack_ceiling_m: float | None,
) -> np.ndarray | None:
    """0 for candidates the target can be engaged from, 1 for the rest.

    ``enemy_requirements_m[i][c]`` is the altitude at which enemy ``i`` starts
    to see candidate ``c``.  LOS is symmetric, so that same altitude is what the
    aircraft needs to see the enemy - i.e. the firing altitude.  Returns
    ``None`` (no reordering at all) whenever the target cannot be identified or
    the caller supplied no ceiling.
    """

    if attack_target_coordinate is None or attack_ceiling_m is None:
        return None
    try:
        ceiling_m = float(attack_ceiling_m)
        target_lat = float(attack_target_coordinate["latitude"])
        target_lon = float(attack_target_coordinate["longitude"])
    except (KeyError, TypeError, ValueError):
        return None
    if not math.isfinite(ceiling_m) or ceiling_m <= 0.0:
        return None

    best_index: int | None = None
    best_delta = float("inf")
    for index, enemy in enumerate(precise_enemies):
        try:
            delta = abs(float(enemy.lat) - target_lat) + abs(float(enemy.lon) - target_lon)
        except (AttributeError, TypeError, ValueError):
            continue
        if delta < best_delta:
            best_delta = delta
            best_index = index
    if best_index is None or best_delta > _ATTACK_TARGET_MATCH_TOLERANCE_DEG:
        # The designated target is not among the analysed enemies; ranking it
        # would be a guess, so leave the legacy order untouched.
        return None
    if best_index >= int(enemy_requirements_m.shape[0]):
        return None

    firing_altitude_m = enemy_requirements_m[best_index]
    feasible = np.isfinite(firing_altitude_m) & (firing_altitude_m <= ceiling_m)
    return np.where(feasible, 0, 1).astype(np.int8)


def refine_communication_hide(
    coarse_result: CommunicationHideResult,
    precision_dem: Any,
    config: CoverConfig | None = None,
    *,
    uav_agl_m: float | None = 1_000.0,
    hide_agl_m: float = 50.0,
    max_hide_agl_m: float = 1_000.0,
    search_radius_m: float = 5_000.0,
    communication_range_m: float = 0.0,
    min_uav_links: int | None = None,
    refinement_radius_m: float = 300.0,
    own_reachable_radius_m: float = 0.0,
    chunk_size: int = 64,
    max_precision_candidates: int | None = None,
    preferred_heading_deg: float | None = None,
    current_speed_mps: float = 0.0,
    max_turn_rate_deg_s: float = 20.0,
    prefer_agl_target_m: float | None = None,
    descent_rate_mps: float | None = None,
    climb_rate_mps: float | None = None,
    max_speed_mps: float | None = None,
    attack_target_coordinate: dict | None = None,
    attack_ceiling_m: float | None = None,
) -> CommunicationHideResult:
    """Refine a coarse recommendation with native DEM cells and LOS rays.

    ``prefer_agl_target_m`` selects the altitude the endpoint aims for, as a
    height above the candidate's own terrain, instead of "hold the altitude you
    are already at".  It is a preference only: the value is clipped into the
    interval already proven feasible by the enemy ceiling and the UAV-link
    floor, so it can never remove a candidate.

    ``descent_rate_mps``/``climb_rate_mps``/``max_speed_mps`` let candidates be
    ranked by the time it actually takes to *get* to them.  Without them the
    ranking prefers whatever is closest to the current altitude, which on a
    high-flying aircraft systematically promotes endpoints it cannot reach
    inside the hide deadline.  All four arguments default to the legacy
    behaviour when omitted.

    ``attack_target_coordinate``/``attack_ceiling_m`` make the hide point aware
    of the shot that has to follow it.  Concealment quality and firing ability
    pull against each other: the deeper a candidate sits behind the ridge next
    to the enemy, the higher the aircraft has to climb to see over it, and past
    the airframe ceiling the attack cannot happen at all.  Because LOS is
    symmetric, the altitude at which that enemy starts to see a candidate is
    also the altitude at which the aircraft can see the enemy - a number this
    function already computes.  Supplying both arguments promotes candidates
    whose shot fits inside the ceiling; candidates are only reordered, never
    dropped, and omitting either argument keeps the legacy ranking exactly.
    """

    t0 = time.perf_counter()
    base_config = config or CoverConfig()
    if not coarse_result.uavs:
        raise ValueError("At least one valid UAV position is required for refinement.")
    if not coarse_result.enemies:
        raise ValueError("At least one enemy point is required for refinement.")

    # Permit enough samples for a full native-raster diagonal.  Each observer
    # still uses only the steps required by its farthest applicable candidate.
    native_step_cap = int(
        np.ceil(
            (int(precision_dem.height) + int(precision_dem.width))
            * max(1.0, float(base_config.los_samples_per_cell))
        )
    ) + 2
    precise_config = base_config.with_overrides(
        los_samples_per_cell=max(1.0, float(base_config.los_samples_per_cell)),
        los_max_steps=max(int(base_config.los_max_steps), native_step_cap),
    )

    own_x = float(coarse_result.own_x)
    own_y = float(coarse_result.own_y)
    own_altitude_m = float(coarse_result.own_alt_m)
    own_ground_m = float(precision_dem.sample_native(own_x, own_y))
    if not np.isfinite(own_ground_m):
        raise ValueError("Own position is on invalid terrain in the native DEM.")
    if not np.isfinite(own_altitude_m) or own_altitude_m <= own_ground_m:
        raise ValueError(
            "Own MSL altitude must be above native DEM terrain "
            f"({own_ground_m:.1f} m)."
        )

    precise_uavs = [
        _native_uav(precision_dem, uav, agl_m=uav_agl_m)
        for uav in coarse_result.uavs[:3]
    ]
    enemy_cap = max(1, int(precise_config.max_analysis_enemies))
    precise_enemies = [
        _native_enemy(
            precision_dem,
            enemy,
            height_m=float(precise_config.enemy_height_m),
        )
        for enemy in coarse_result.enemies[:enemy_cap]
    ]
    required_links = (
        len(precise_uavs)
        if min_uav_links is None
        else max(1, min(len(precise_uavs), int(min_uav_links)))
    )
    floor_agl_m = max(1.0, float(hide_agl_m))
    ceiling_agl_m = max(floor_agl_m, float(max_hide_agl_m))
    search_m = max(float(precision_dem.cell_m), float(search_radius_m))
    refine_m = max(float(precision_dem.cell_m), float(refinement_radius_m))
    reachable_m = max(0.0, float(own_reachable_radius_m))
    enemy_range_m = max(1.0, float(precise_config.weapon_range_m))
    communication_m = (
        float("inf")
        if float(communication_range_m) <= 0.0
        else float(communication_range_m)
    )

    (
        rows,
        cols,
        xs,
        ys,
        grounds_m,
        candidate_distance_m,
        primary_mask,
        own_index,
    ) = _native_candidates(
        precision_dem,
        coarse_result,
        refinement_radius_m=refine_m,
        search_radius_m=search_m,
        own_reachable_radius_m=reachable_m,
    )
    if rows.size == 0:
        raise ValueError("No valid native DEM cells exist in the refinement window.")
    count = int(rows.size)
    primary_indices = np.nonzero(primary_mask)[0]
    route_only_indices = np.nonzero(~primary_mask)[0]
    terrain_floor_m = grounds_m + floor_agl_m
    altitude_ceiling_m = grounds_m + ceiling_agl_m

    enemy_requirements_m = np.full(
        (len(precise_enemies), count), np.inf, dtype=np.float64
    )
    for index, enemy in enumerate(precise_enemies):
        for candidate_indices in (primary_indices, route_only_indices):
            if candidate_indices.size == 0:
                continue
            enemy_requirements_m[index, candidate_indices] = (
                _observer_requirements_chunked(
                    precision_dem,
                    precise_config,
                    enemy,
                    rows=rows[candidate_indices],
                    cols=cols[candidate_indices],
                    xs=xs[candidate_indices],
                    ys=ys[candidate_indices],
                    max_distance_m=enemy_range_m,
                    chunk_size=chunk_size,
                )
            )

    uav_requirements_m = np.full(
        (len(precise_uavs), count), np.inf, dtype=np.float64
    )
    for index, uav in enumerate(precise_uavs):
        for candidate_indices in (primary_indices, route_only_indices):
            if candidate_indices.size == 0:
                continue
            uav_requirements_m[index, candidate_indices] = (
                _observer_requirements_chunked(
                    precision_dem,
                    precise_config,
                    uav,
                    rows=rows[candidate_indices],
                    cols=cols[candidate_indices],
                    xs=xs[candidate_indices],
                    ys=ys[candidate_indices],
                    max_distance_m=communication_m,
                    chunk_size=chunk_size,
                    reject_nodata=True,
                )
            )

    enemy_hidden_ceiling_m = (
        np.min(enemy_requirements_m, axis=0)
        - _ALTITUDE_MARGIN_M
        - _ENEMY_CEILING_SELECTION_MARGIN_M
    )
    sorted_uav_requirements_m = np.sort(uav_requirements_m, axis=0)
    link_floor_m = (
        sorted_uav_requirements_m[required_links - 1] + _ALTITUDE_MARGIN_M
    )
    feasible_floor_m = np.maximum(terrain_floor_m, link_floor_m)
    feasible_ceiling_m = np.minimum(altitude_ceiling_m, enemy_hidden_ceiling_m)
    strict_candidates = np.isfinite(feasible_floor_m) & (
        feasible_floor_m <= feasible_ceiling_m
    )
    # Altitude preference.  The legacy target is "stay where you are", which on
    # an aircraft holding far above the concealment ceiling drags every
    # endpoint up with it.  A terrain-referenced band target pulls the endpoint
    # down to where concealment actually lives.  Either way the value is only a
    # clip inside an interval already proven feasible, so no candidate is lost.
    if prefer_agl_target_m is None:
        altitude_target_m = np.full(count, own_altitude_m, dtype=np.float64)
    else:
        altitude_target_m = grounds_m + max(0.0, float(prefer_agl_target_m))
    preferred_altitudes_m = np.clip(
        altitude_target_m,
        feasible_floor_m,
        feasible_ceiling_m,
    )

    # Time to reach each candidate's preferred altitude and position.  Used for
    # ranking only; an unreachable candidate still ships so the longer
    # hide-then-reconnect window can still use it.
    if descent_rate_mps is None or climb_rate_mps is None:
        vertical_time_s = None
    else:
        descent_rate = max(0.1, float(descent_rate_mps))
        climb_rate = max(0.1, float(climb_rate_mps))
        altitude_delta_m = preferred_altitudes_m - own_altitude_m
        vertical_time_s = np.where(
            altitude_delta_m < 0.0,
            -altitude_delta_m / descent_rate,
            altitude_delta_m / climb_rate,
        )
    if max_speed_mps is None:
        horizontal_time_s = None
    else:
        horizontal_time_s = candidate_distance_m / max(1.0, float(max_speed_mps))
    if vertical_time_s is None and horizontal_time_s is None:
        achievable_time_s = None
    else:
        achievable_time_s = np.maximum(
            vertical_time_s if vertical_time_s is not None else 0.0,
            horizontal_time_s if horizontal_time_s is not None else 0.0,
        )

    analyzer = CommunicationHideAnalyzer(precision_dem, precise_config)
    primary_rows = rows[primary_indices]
    primary_cols = cols[primary_indices]
    primary_xs = xs[primary_indices]
    primary_ys = ys[primary_indices]

    def point_status(candidate_index: int, altitude_m: float):
        enemy_visible, enemy_in_range, connected, _link_count = analyzer._point_status(
            row=float(rows[candidate_index]),
            col=float(cols[candidate_index]),
            x=float(xs[candidate_index]),
            y=float(ys[candidate_index]),
            altitude_m=float(altitude_m),
            enemies=precise_enemies,
            uavs=precise_uavs,
            enemy_range_m=enemy_range_m,
            communication_range_m=communication_m,
            step_reference_rows=primary_rows,
            step_reference_cols=primary_cols,
            step_reference_xs=primary_xs,
            step_reference_ys=primary_ys,
        )
        known_connected = [
            uav_index
            for uav_index in connected
            if _uav_ray_known_at_point(
                precision_dem,
                precise_config,
                precise_uavs[uav_index - 1],
                row=float(rows[candidate_index]),
                col=float(cols[candidate_index]),
                x=float(xs[candidate_index]),
                y=float(ys[candidate_index]),
                communication_range_m=communication_m,
                step_reference_rows=primary_rows,
                step_reference_cols=primary_cols,
                step_reference_xs=primary_xs,
                step_reference_ys=primary_ys,
            )
        ]
        return enemy_visible, enemy_in_range, known_connected, len(known_connected)

    # Coarse success/degraded conclusions are superseded by this pass. Keep
    # only input-shaping diagnostics that remain true at native resolution.
    notes = [
        note for note in coarse_result.notes if note.startswith("Enemy input was capped")
    ]
    selected_index: int
    selected_altitude_m: float
    selected_status: tuple[int, int, list[int], int]
    strict_feasible = False
    strict_indices = np.nonzero(strict_candidates)[0]

    # Which candidates can still take the shot?  Only computed when the caller
    # names the target and the airframe ceiling; otherwise the ranking below is
    # byte-for-byte the legacy one.
    attack_infeasible_rank = _attack_infeasibility_rank(
        precise_enemies,
        enemy_requirements_m,
        attack_target_coordinate=attack_target_coordinate,
        attack_ceiling_m=attack_ceiling_m,
    )

    def _reach_key(index: int) -> tuple:
        """Rank candidates by how soon they can actually be occupied.

        Distance alone hides the vertical cost: an endpoint 200 m below the
        aircraft is far more expensive than a slightly more distant one at the
        current altitude.  Distance stays as the immediate tiebreak so equally
        reachable points still resolve to the nearest one.

        A hide point the aircraft cannot shoot from is ranked last, because the
        attack that follows is the reason it is hiding at all.
        """

        if achievable_time_s is None:
            base = (
                float(candidate_distance_m[index]),
                abs(float(preferred_altitudes_m[index]) - own_altitude_m),
                float(preferred_altitudes_m[index]),
                int(index),
            )
        else:
            base = (
                float(achievable_time_s[index]),
                float(candidate_distance_m[index]),
                float(preferred_altitudes_m[index]),
                int(index),
            )
        if attack_infeasible_rank is None:
            return base
        return (int(attack_infeasible_rank[index]),) + base

    strict_order = sorted((int(index) for index in strict_indices), key=_reach_key)
    primary_strict_indices = np.nonzero(strict_candidates & primary_mask)[0]
    primary_strict_order = sorted(
        (int(index) for index in primary_strict_indices), key=_reach_key
    )
    canonical_cache: dict[
        tuple[int, float], tuple[int, int, list[int], int]
    ] = {}

    def route_endpoint_validator(
        candidate_index: int,
        altitude_m: float,
    ) -> tuple[int, int, list[int], int]:
        """Canonical native LOS status cached by endpoint and arrival altitude."""

        key = (int(candidate_index), float(altitude_m))
        if key not in canonical_cache:
            canonical_cache[key] = point_status(int(candidate_index), float(altitude_m))
        return canonical_cache[key]

    # Preserve every analytically strict native endpoint for the route stage.
    # Each exported candidate can canonically recheck the route's actual
    # arrival altitude, including candidates from the route-only ownship disk.
    export_order = strict_order
    heading_value: float | None = None
    try:
        parsed_heading = float(preferred_heading_deg)
        if np.isfinite(parsed_heading):
            heading_value = parsed_heading % 360.0
    except (TypeError, ValueError, OverflowError):
        heading_value = None
    try:
        moving_speed = max(0.0, float(current_speed_mps))
    except (TypeError, ValueError, OverflowError):
        moving_speed = 0.0
    try:
        turn_rate = max(0.1, float(max_turn_rate_deg_s))
    except (TypeError, ValueError, OverflowError):
        turn_rate = 20.0
    if heading_value is not None and moving_speed > 1.0 and export_order:
        # Nearest-cell ordering over-selects the zero-distance hover and a
        # small adjacent cluster, all of which can be unreachable by a moving
        # LAH. Rank by a turn+travel lower bound, then retain one point from
        # every relative-bearing sector before the route-stage cap is applied.
        def dynamic_key(index: int) -> tuple[float, float, float, int]:
            dx = float(xs[index]) - own_x
            dy = float(ys[index]) - own_y
            distance_m = float(candidate_distance_m[index])
            if distance_m <= max(1.0, float(precision_dem.cell_m) * 0.5):
                return (1.0e9, 180.0, distance_m, int(index))
            bearing = (float(np.degrees(np.arctan2(dx, dy))) + 360.0) % 360.0
            delta = abs(((bearing - heading_value + 180.0) % 360.0) - 180.0)
            lower_bound_s = delta / turn_rate + distance_m / max(moving_speed, 20.0)
            if achievable_time_s is not None:
                # Descending to the endpoint can dominate the turn-and-travel
                # cost, and an endpoint that cannot be descended to in time is
                # not a fast candidate however close it is.
                lower_bound_s = max(lower_bound_s, float(achievable_time_s[index]))
            return (lower_bound_s, delta, distance_m, int(index))

        dynamic_order = sorted((int(index) for index in export_order), key=dynamic_key)
        sector_first: list[int] = []
        seen_sectors: set[int] = set()
        for index in dynamic_order:
            dx = float(xs[index]) - own_x
            dy = float(ys[index]) - own_y
            bearing = (float(np.degrees(np.arctan2(dx, dy))) + 360.0) % 360.0
            relative_bearing = (bearing - heading_value + 360.0) % 360.0
            sector = int(relative_bearing // 45.0) % 8
            if sector not in seen_sectors:
                seen_sectors.add(sector)
                sector_first.append(index)
        export_order = list(
            dict.fromkeys(dynamic_order[:8] + sector_first + dynamic_order)
        )
    if max_precision_candidates is not None:
        export_order = export_order[: max(1, int(max_precision_candidates))]
    precision_candidates = [
        HideEndpointCandidate(
            x=float(xs[index]),
            y=float(ys[index]),
            row=float(rows[index]),
            col=float(cols[index]),
            ground_m=float(grounds_m[index]),
            min_altitude_m=float(feasible_floor_m[index]),
            max_altitude_m=float(feasible_ceiling_m[index]),
            preferred_altitude_m=float(preferred_altitudes_m[index]),
            horizontal_distance_m=float(candidate_distance_m[index]),
            canonical_los_validator=(
                lambda altitude_m, candidate_index=int(index): route_endpoint_validator(
                    candidate_index,
                    altitude_m,
                )
            ),
            required_uav_links=int(required_links),
            requires_canonical_los_validation=True,
        )
        for index in export_order
    ]
    # A canonical float32 LOS pass is the final authority.  Numerical edge
    # cases can invalidate the first analytic candidate, so try nearby strict
    # candidates before falling back.
    canonical_limit = min(len(primary_strict_order), 64)
    for candidate_index in primary_strict_order[:canonical_limit]:
        altitude_m = float(preferred_altitudes_m[candidate_index])
        status = point_status(candidate_index, altitude_m)
        if int(status[0]) == 0 and int(status[3]) >= required_links:
            selected_index = candidate_index
            selected_altitude_m = altitude_m
            selected_status = status
            strict_feasible = True
            break

    if not strict_feasible and len(primary_strict_order) > canonical_limit:
        notes.append(
            f"Canonical LOS verification was capped at the first {canonical_limit} "
            "analytic strict candidates."
        )

    if not strict_feasible:
        primary_selected_index, selected_altitude_m = _select_fallback(
            own_altitude_m=own_altitude_m,
            terrain_floor_m=terrain_floor_m[primary_indices],
            altitude_ceiling_m=altitude_ceiling_m[primary_indices],
            candidate_distance_m=candidate_distance_m[primary_indices],
            enemy_requirements_m=enemy_requirements_m[:, primary_indices],
            uav_requirements_m=uav_requirements_m[:, primary_indices],
            hide_safety_margin_m=float(
                getattr(analyzer.config, "hide_safety_margin_m", 0.0)
            ),
        )
        selected_index = int(primary_indices[primary_selected_index])
        selected_status = point_status(selected_index, selected_altitude_m)
        if primary_strict_order:
            notes.append(
                "No checked native-resolution strict candidate passed the canonical LOS "
                "recheck; showing the degraded fallback."
            )
        else:
            notes.append(
                "No native-resolution point in the refinement window satisfies complete "
                "enemy masking and the required UAV-link count; showing the degraded fallback."
            )

    selected_enemy_visible, selected_in_range, connected_uavs, selected_links = (
        selected_status
    )
    (
        current_enemy_visible,
        current_enemy_in_range,
        _current_connected,
        current_uav_links,
    ) = point_status(own_index, own_altitude_m)

    selected_x = float(xs[selected_index])
    selected_y = float(ys[selected_index])
    selected_ground_m = float(grounds_m[selected_index])
    selected_row, selected_col = precision_dem.nearest_index(selected_x, selected_y)
    lat, lon = precision_dem.native_to_latlon(selected_x, selected_y)
    refine_elapsed_s = time.perf_counter() - t0
    if strict_feasible:
        notes.append(
            f"Native {float(precision_dem.cell_m):.1f} m DEM recheck masks every "
            f"in-range enemy and maintains LOS to {selected_links}/{len(precise_uavs)} UAVs."
        )
    notes.append(
        f"Native-resolution selection is local: {refine_m:.0f} m around the coarse "
        "recommendation plus the exact ownship position."
    )
    if reachable_m > 0.0:
        notes.append(
            f"Route endpoint export also includes native cells within {reachable_m:.0f} m "
            "of the ownship position."
        )
    notes.append(
        "UAV LOS rays crossing native DEM nodata are treated as unavailable."
    )

    # The full-map overlays intentionally stay coarse.  Only the recommendation
    # star/coordinates are native-resolution; stretching a local 10 m mask over
    # the coarse extent would draw a false suitability area.
    return CommunicationHideResult(
        own_x=own_x,
        own_y=own_y,
        own_alt_m=own_altitude_m,
        uavs=precise_uavs,
        enemies=precise_enemies,
        candidate_mask=coarse_result.candidate_mask,
        strict_mask=coarse_result.strict_mask,
        enemy_visible_count=coarse_result.enemy_visible_count,
        uav_link_count=coarse_result.uav_link_count,
        # Preserve the coarse-grid row/mask indexing contract. Native indices
        # are exposed separately through precision_row/precision_col.
        recommended_row=int(coarse_result.recommended_row),
        recommended_col=int(coarse_result.recommended_col),
        recommended_x=selected_x,
        recommended_y=selected_y,
        recommended_lat=float(lat),
        recommended_lon=float(lon),
        recommended_ground_m=selected_ground_m,
        recommended_alt_m=float(selected_altitude_m),
        horizontal_distance_m=float(candidate_distance_m[selected_index]),
        strict_feasible=bool(strict_feasible),
        required_uav_links=required_links,
        connected_uav_indices=list(connected_uavs),
        visible_enemy_count=int(selected_enemy_visible),
        in_range_enemy_count=int(selected_in_range),
        current_uav_link_count=int(current_uav_links),
        current_visible_enemy_count=int(current_enemy_visible),
        current_in_range_enemy_count=int(current_enemy_in_range),
        elapsed_s=float(coarse_result.elapsed_s) + refine_elapsed_s,
        notes=notes,
        refined_native=True,
        precision_cell_m=float(precision_dem.cell_m),
        refinement_radius_m=refine_m,
        refinement_elapsed_s=refine_elapsed_s,
        precision_row=int(selected_row),
        precision_col=int(selected_col),
        overlay_grids_coarse=True,
        precision_candidates=precision_candidates,
    )


__all__ = ["refine_communication_hide"]
