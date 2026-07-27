"""Nearest joint enemy-cover and UAV-communication position search.

The solver evaluates exact operator-clicked enemy and UAV points.  For every
candidate DEM cell it derives, from the terrain ray itself, the minimum target
altitude required for LOS to each observer.  This turns the joint problem into
an altitude interval:

* stay below every in-range enemy's LOS threshold, and
* stay above the selected UAV-link thresholds and the terrain-clearance floor.

The interval calculation avoids repeatedly sweeping the same ray at many
trial altitudes, so three UAVs and multiple enemies remain interactive.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import time
from typing import Callable

import numpy as np

from .config import CoverConfig
from .dem import DemGrid
from .sampling import EnemyPoint
from .visibility import _EFFECTIVE_EARTH_RADIUS_M, _los_visible_batch


@dataclass(frozen=True)
class UavPoint:
    x: float
    y: float
    row: float
    col: float
    ground_m: float
    alt_m: float
    lat: float
    lon: float

    def as_dict(self) -> dict:
        return {
            "x": round(float(self.x), 3),
            "y": round(float(self.y), 3),
            "lat": round(float(self.lat), 7),
            "lon": round(float(self.lon), 7),
            "groundM": round(float(self.ground_m), 2),
            "altM": round(float(self.alt_m), 2),
        }


@dataclass(frozen=True)
class HideEndpointCandidate:
    """Native strict endpoint with the full altitude interval kept for routing."""

    x: float
    y: float
    row: float
    col: float
    ground_m: float
    min_altitude_m: float
    max_altitude_m: float
    preferred_altitude_m: float
    horizontal_distance_m: float
    # Runtime-only native LOS hook. Native refinement sets this so routing can
    # validate its actual arrival altitude instead of trusting only the
    # analytic interval. Excluding it from equality keeps deterministic tests.
    canonical_los_validator: Callable[[float], tuple[int, int, list[int], int]] | None = field(
        default=None,
        repr=False,
        compare=False,
    )
    required_uav_links: int = 0
    requires_canonical_los_validation: bool = False

    def as_dict(self) -> dict:
        return {
            "x": round(float(self.x), 2),
            "y": round(float(self.y), 2),
            "groundM": round(float(self.ground_m), 1),
            "minAltitudeMslM": round(float(self.min_altitude_m), 1),
            "maxAltitudeMslM": round(float(self.max_altitude_m), 1),
            "preferredAltitudeMslM": round(float(self.preferred_altitude_m), 1),
            "horizontalDistanceM": round(float(self.horizontal_distance_m), 1),
            "canonicalLosValidationRequired": bool(
                self.requires_canonical_los_validation
            ),
            "requiredUavLinks": int(self.required_uav_links),
        }


@dataclass
class CommunicationHideResult:
    own_x: float
    own_y: float
    own_alt_m: float
    uavs: list[UavPoint]
    enemies: list[EnemyPoint]

    candidate_mask: np.ndarray
    strict_mask: np.ndarray
    enemy_visible_count: np.ndarray
    uav_link_count: np.ndarray

    recommended_row: int
    recommended_col: int
    recommended_x: float
    recommended_y: float
    recommended_lat: float
    recommended_lon: float
    recommended_ground_m: float
    recommended_alt_m: float
    horizontal_distance_m: float

    strict_feasible: bool
    required_uav_links: int
    connected_uav_indices: list[int]
    visible_enemy_count: int
    in_range_enemy_count: int
    current_uav_link_count: int
    current_visible_enemy_count: int
    current_in_range_enemy_count: int
    elapsed_s: float
    notes: list[str] = field(default_factory=list)
    refined_native: bool = False
    precision_cell_m: float | None = None
    refinement_radius_m: float | None = None
    refinement_elapsed_s: float | None = None
    precision_row: int | None = None
    precision_col: int | None = None
    overlay_grids_coarse: bool = False
    precision_candidates: list[HideEndpointCandidate] = field(
        default_factory=list,
        repr=False,
    )

    @property
    def refined_10m(self) -> bool:
        """Backward-friendly flag that is true only for an actual ~10 m source DEM."""

        return bool(
            self.refined_native
            and self.precision_cell_m is not None
            and 9.0 <= float(self.precision_cell_m) <= 11.0
        )

    def summary(self) -> dict:
        return {
            "recommendedPosition": {
                "x": round(float(self.recommended_x), 2),
                "y": round(float(self.recommended_y), 2),
                "lat": round(float(self.recommended_lat), 7),
                "lon": round(float(self.recommended_lon), 7),
                "groundM": round(float(self.recommended_ground_m), 1),
                "altitudeMslM": round(float(self.recommended_alt_m), 1),
                "aglM": round(
                    float(self.recommended_alt_m - self.recommended_ground_m), 1
                ),
            },
            "horizontalDistanceM": round(float(self.horizontal_distance_m), 1),
            "strictFeasible": bool(self.strict_feasible),
            "requiredUavLinks": int(self.required_uav_links),
            "uavLinkCount": len(self.connected_uav_indices),
            "connectedUavIndices": list(self.connected_uav_indices),
            "visibleEnemyCount": int(self.visible_enemy_count),
            "inRangeEnemyCount": int(self.in_range_enemy_count),
            "currentUavLinkCount": int(self.current_uav_link_count),
            "currentVisibleEnemyCount": int(self.current_visible_enemy_count),
            "currentInRangeEnemyCount": int(self.current_in_range_enemy_count),
            "enemyCount": len(self.enemies),
            "elapsedS": round(float(self.elapsed_s), 3),
            "refinedNative": bool(self.refined_native),
            "refined10m": bool(self.refined_10m),
            "precisionCellM": (
                None
                if self.precision_cell_m is None
                else round(float(self.precision_cell_m), 2)
            ),
            "refinementRadiusM": (
                None
                if self.refinement_radius_m is None
                else round(float(self.refinement_radius_m), 1)
            ),
            "refinementElapsedS": (
                None
                if self.refinement_elapsed_s is None
                else round(float(self.refinement_elapsed_s), 3)
            ),
            "precisionGridCell": (
                None
                if self.precision_row is None or self.precision_col is None
                else {"row": int(self.precision_row), "col": int(self.precision_col)}
            ),
            "overlayGridsCoarse": bool(self.overlay_grids_coarse),
            "precisionCandidateCount": len(self.precision_candidates),
            "notes": list(self.notes),
        }


def uav_from_native(
    dem: DemGrid,
    x: float,
    y: float,
    *,
    agl_m: float = 1_000.0,
) -> UavPoint:
    """Create a clicked UAV at exactly ``DEM + agl_m``."""

    if not dem.contains_native(float(x), float(y)):
        raise ValueError("UAV position is outside the DEM.")
    ground_m = float(dem.sample_native(float(x), float(y)))
    if not np.isfinite(ground_m):
        raise ValueError("UAV position is on invalid DEM terrain.")
    row, col = dem.nearest_index(float(x), float(y))
    if not bool(dem.valid[row, col]):
        raise ValueError("UAV position is on an invalid DEM cell.")
    lat, lon = dem.native_to_latlon(float(x), float(y))
    return UavPoint(
        x=float(x),
        y=float(y),
        row=int(row),
        col=int(col),
        ground_m=ground_m,
        alt_m=ground_m + max(1.0, float(agl_m)),
        lat=float(lat),
        lon=float(lon),
    )


def _required_target_altitude_batch(
    block_elev: np.ndarray,
    *,
    observer_row: float,
    observer_col: float,
    observer_altitude_m: float,
    target_rows: np.ndarray,
    target_cols: np.ndarray,
    horizontal_distances_m: np.ndarray,
    config: CoverConfig,
    steps_override: int | None = None,
) -> np.ndarray:
    """Minimum target MSL altitude that produces clear terrain LOS.

    ``-inf`` means a ray has no interior terrain cell and is therefore clear
    at any target altitude above its own terrain floor.  This is the analytic
    counterpart of ``visibility._los_visible_batch``.
    """

    count = int(target_rows.size)
    if count == 0:
        return np.zeros(0, dtype=np.float64)
    span = np.abs(target_rows - float(observer_row)) + np.abs(
        target_cols - float(observer_col)
    )
    max_span = float(np.max(span)) if span.size else 1.0
    steps = (
        int(np.clip(int(steps_override), 2, int(config.los_max_steps)))
        if steps_override is not None
        else int(
            np.clip(
                int(max_span * float(config.los_samples_per_cell)) + 1,
                2,
                int(config.los_max_steps),
            )
        )
    )
    if steps <= 2:
        return np.full(count, -np.inf, dtype=np.float64)

    t = np.linspace(0.0, 1.0, steps, dtype=np.float64).reshape(-1, 1)
    rr = float(observer_row) + t * (
        target_rows.astype(np.float64, copy=False) - float(observer_row)
    )
    cc = float(observer_col) + t * (
        target_cols.astype(np.float64, copy=False) - float(observer_col)
    )
    height, width = block_elev.shape
    ri = np.clip(np.rint(rr).astype(np.int32), 0, height - 1)
    ci = np.clip(np.rint(cc).astype(np.int32), 0, width - 1)
    terrain_m = block_elev[ri, ci].astype(np.float64, copy=False)

    if bool(config.earth_curvature):
        along_m = t * horizontal_distances_m.astype(np.float64, copy=False)
        terrain_m = terrain_m - (
            along_m * along_m / (2.0 * float(_EFFECTIVE_EARTH_RADIUS_M))
        )

    interior_t = t[1:-1]
    interior_terrain_m = terrain_m[1:-1]
    observer_alt_m = float(observer_altitude_m)
    clearance_m = float(config.los_clearance_m)
    # Match terrain_los/visibility exactly: the configured clearance is a
    # masking tolerance, so terrain blocks only above ``ray + clearance``.
    required_m = observer_alt_m + (
        interior_terrain_m - clearance_m - observer_alt_m
    ) / interior_t
    return np.max(required_m, axis=0)


class CommunicationHideAnalyzer:
    """Jointly enforce terrain concealment and UAV communication LOS."""

    def __init__(self, dem: DemGrid, config: CoverConfig | None = None) -> None:
        self.dem = dem
        self.config = config or CoverConfig()

    def _observer_requirements(
        self,
        observer,
        *,
        rows: np.ndarray,
        cols: np.ndarray,
        xs: np.ndarray,
        ys: np.ndarray,
        max_distance_m: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        distances_m = np.hypot(xs - float(observer.x), ys - float(observer.y))
        applicable = distances_m <= max(1.0, float(max_distance_m))
        required_m = np.full(rows.size, np.inf, dtype=np.float64)
        if np.any(applicable):
            indices = np.nonzero(applicable)[0]
            required_m[indices] = _required_target_altitude_batch(
                self.dem.block_elev,
                observer_row=float(observer.row),
                observer_col=float(observer.col),
                observer_altitude_m=float(observer.alt_m),
                target_rows=rows[indices].astype(np.float64, copy=False),
                target_cols=cols[indices].astype(np.float64, copy=False),
                horizontal_distances_m=distances_m[indices],
                config=self.config,
            )
        return required_m, applicable

    @staticmethod
    def _counts_at_altitude(
        altitudes_m: np.ndarray,
        enemy_requirements_m: np.ndarray,
        uav_requirements_m: np.ndarray,
        hide_safety_margin_m: float = 0.0,
    ) -> tuple[np.ndarray, np.ndarray]:
        altitudes = np.asarray(altitudes_m, dtype=np.float64).reshape(1, -1)
        # Count an enemy as seeing us while we are still within the margin of
        # its sightline, so borderline cover is never sold as concealment.
        enemy_visible = np.sum(
            altitudes >= enemy_requirements_m - float(hide_safety_margin_m),
            axis=0,
            dtype=np.int16,
        )
        uav_links = np.sum(
            altitudes >= uav_requirements_m,
            axis=0,
            dtype=np.int16,
        )
        return enemy_visible, uav_links

    def _point_status(
        self,
        *,
        row: float,
        col: float,
        x: float,
        y: float,
        altitude_m: float,
        enemies: list[EnemyPoint],
        uavs: list[UavPoint],
        enemy_range_m: float,
        communication_range_m: float,
        step_reference_rows: np.ndarray | None = None,
        step_reference_cols: np.ndarray | None = None,
        step_reference_xs: np.ndarray | None = None,
        step_reference_ys: np.ndarray | None = None,
    ) -> tuple[int, int, list[int], int]:
        def target_arrays(observer, distance_m: float, max_distance_m: float):
            target_rows = [float(row)]
            target_cols = [float(col)]
            target_altitudes = [float(altitude_m)]
            target_distances = [float(distance_m)]
            if (
                isinstance(step_reference_rows, np.ndarray)
                and isinstance(step_reference_cols, np.ndarray)
                and isinstance(step_reference_xs, np.ndarray)
                and isinstance(step_reference_ys, np.ndarray)
                and step_reference_rows.size
                and step_reference_rows.size == step_reference_cols.size
                and step_reference_rows.size == step_reference_xs.size
                and step_reference_rows.size == step_reference_ys.size
            ):
                applicable = (
                    np.hypot(
                        step_reference_xs - float(observer.x),
                        step_reference_ys - float(observer.y),
                    )
                    <= float(max_distance_m)
                )
                applicable_indices = np.nonzero(applicable)[0]
                if applicable_indices.size == 0:
                    applicable_indices = np.arange(step_reference_rows.size)
                spans = np.abs(
                    step_reference_rows[applicable_indices] - float(observer.row)
                ) + np.abs(
                    step_reference_cols[applicable_indices] - float(observer.col)
                )
                reference_index = int(applicable_indices[int(np.argmax(spans))])
                reference_row = float(step_reference_rows[reference_index])
                reference_col = float(step_reference_cols[reference_index])
                target_rows.append(reference_row)
                target_cols.append(reference_col)
                target_altitudes.append(float(altitude_m))
                target_distances.append(
                    float(
                        np.hypot(
                            reference_row - float(observer.row),
                            reference_col - float(observer.col),
                        )
                        * float(self.dem.cell_m)
                    )
                )
            return (
                np.asarray(target_rows, dtype=np.float64),
                np.asarray(target_cols, dtype=np.float64),
                np.asarray(target_altitudes, dtype=np.float64),
                np.asarray(target_distances, dtype=np.float64),
            )

        enemy_visible = 0
        enemy_in_range = 0
        enemy_unevaluated = 0
        for enemy in enemies:
            distance_m = float(np.hypot(float(x) - enemy.x, float(y) - enemy.y))
            if distance_m > float(enemy_range_m):
                # Out of the configured observation range: no ray was traced, so
                # this enemy is *unevaluated*, not proven blocked.  Callers must
                # be able to tell those apart - silently skipping it is what let
                # a point in plain view report enemyVisibleCount=0.
                enemy_unevaluated += 1
                continue
            enemy_in_range += 1
            target_rows, target_cols, target_altitudes, target_distances = target_arrays(
                enemy, distance_m, enemy_range_m
            )
            visible = _los_visible_batch(
                self.dem.block_elev,
                float(enemy.row),
                float(enemy.col),
                float(enemy.alt_m),
                target_rows,
                target_cols,
                target_altitudes,
                target_distances,
                float(self.dem.cell_m),
                self.config,
            )
            enemy_visible += int(bool(visible[0]))
        connected: list[int] = []
        for index, uav in enumerate(uavs, start=1):
            distance_m = float(np.hypot(float(x) - uav.x, float(y) - uav.y))
            if distance_m > float(communication_range_m):
                continue
            target_rows, target_cols, target_altitudes, target_distances = target_arrays(
                uav, distance_m, communication_range_m
            )
            visible = _los_visible_batch(
                self.dem.block_elev,
                float(uav.row),
                float(uav.col),
                float(uav.alt_m),
                target_rows,
                target_cols,
                target_altitudes,
                target_distances,
                float(self.dem.cell_m),
                self.config,
            )
            if bool(visible[0]):
                connected.append(index)
        # An enemy we never traced a ray to is not cover.  Every consumer gates
        # on ``enemy_visible == 0``, so counting the unevaluated ones here makes
        # "we could not prove this one is blocked" reject the candidate instead
        # of passing it silently.
        return enemy_visible + enemy_unevaluated, enemy_in_range, connected, len(connected)

    def analyze(
        self,
        *,
        own_x: float,
        own_y: float,
        own_altitude_m: float,
        uavs: list[UavPoint],
        enemies: list[EnemyPoint],
        hide_agl_m: float = 50.0,
        max_hide_agl_m: float = 1_000.0,
        search_radius_m: float = 5_000.0,
        communication_range_m: float = 0.0,
        min_uav_links: int | None = None,
    ) -> CommunicationHideResult:
        t0 = time.perf_counter()
        dem = self.dem
        cfg = self.config
        notes: list[str] = []

        bounded_uavs = list(uavs[:3])
        if not bounded_uavs:
            raise ValueError("At least one valid UAV position is required.")
        if not enemies:
            raise ValueError("At least one enemy point is required.")
        enemy_cap = max(1, int(cfg.max_analysis_enemies))
        bounded_enemies = list(enemies[:enemy_cap])
        if len(enemies) > enemy_cap:
            notes.append(
                f"Enemy input was capped from {len(enemies)} to {enemy_cap} points."
            )
        if not dem.contains_native(float(own_x), float(own_y)):
            raise ValueError("Own position is outside the DEM.")
        own_ground_m = float(dem.sample_native(float(own_x), float(own_y)))
        own_alt_m = float(own_altitude_m)
        if not np.isfinite(own_ground_m):
            raise ValueError("Own position is on invalid DEM terrain.")
        if not np.isfinite(own_alt_m) or own_alt_m <= own_ground_m:
            raise ValueError(
                f"Own MSL altitude must be above terrain ({own_ground_m:.1f} m)."
            )

        required_links = (
            len(bounded_uavs)
            if min_uav_links is None
            else max(1, min(len(bounded_uavs), int(min_uav_links)))
        )
        floor_agl_m = max(1.0, float(hide_agl_m))
        ceiling_agl_m = max(floor_agl_m, float(max_hide_agl_m))
        search_m = max(float(dem.cell_m), float(search_radius_m))
        # Concealment is proven against every detected enemy, not just the ones
        # inside our own weapon range.
        enemy_range_m = cfg.enemy_observation_range_effective_m
        raw_communication_m = float(communication_range_m)
        communication_m = (
            float("inf") if raw_communication_m <= 0.0 else raw_communication_m
        )

        horizontal_distance_m = np.hypot(
            dem.X - float(own_x), dem.Y - float(own_y)
        )
        candidate_mask = dem.valid & (horizontal_distance_m <= search_m)
        if not np.any(candidate_mask):
            raise ValueError("No valid DEM cells exist inside the search radius.")

        grid_rows, grid_cols = np.nonzero(candidate_mask)
        grid_count = int(grid_rows.size)
        own_row_index, own_col_index = dem.nearest_index(float(own_x), float(own_y))
        own_ray_row = float(own_row_index)
        own_ray_col = float(own_col_index)
        native_to_rowcol = getattr(dem, "native_to_rowcol", None)
        if callable(native_to_rowcol):
            try:
                raw_row, raw_col = native_to_rowcol(float(own_x), float(own_y))
                own_ray_row = float(raw_row) - 0.5
                own_ray_col = float(raw_col) - 0.5
            except Exception:
                pass

        # The click itself is a virtual candidate. The display grid can be
        # ~150 m/cell, so searching cell centres alone could recommend a move
        # even when a pure altitude change at the exact current XY is feasible.
        rows = np.concatenate(
            [grid_rows.astype(np.float64), np.array([own_ray_row])]
        )
        cols = np.concatenate(
            [grid_cols.astype(np.float64), np.array([own_ray_col])]
        )
        xs = np.concatenate(
            [dem.X[candidate_mask].astype(np.float64), np.array([float(own_x)])]
        )
        ys = np.concatenate(
            [dem.Y[candidate_mask].astype(np.float64), np.array([float(own_y)])]
        )
        grounds_m = np.concatenate(
            [dem.elev[candidate_mask].astype(np.float64), np.array([own_ground_m])]
        )
        candidate_distance_m = np.concatenate(
            [horizontal_distance_m[candidate_mask], np.array([0.0])]
        )
        count = int(rows.size)
        terrain_floor_m = grounds_m + floor_agl_m
        altitude_ceiling_m = grounds_m + ceiling_agl_m

        enemy_requirements_m = np.full(
            (len(bounded_enemies), count), np.inf, dtype=np.float64
        )
        for index, enemy in enumerate(bounded_enemies):
            required_m, _applicable = self._observer_requirements(
                enemy,
                rows=rows,
                cols=cols,
                xs=xs,
                ys=ys,
                max_distance_m=enemy_range_m,
            )
            enemy_requirements_m[index] = required_m

        uav_requirements_m = np.full(
            (len(bounded_uavs), count), np.inf, dtype=np.float64
        )
        for index, uav in enumerate(bounded_uavs):
            required_m, _applicable = self._observer_requirements(
                uav,
                rows=rows,
                cols=cols,
                xs=xs,
                ys=ys,
                max_distance_m=communication_m,
            )
            uav_requirements_m[index] = required_m

        # Enemy LOS becomes clear at/above its threshold, so concealment owns
        # the open interval below it, held down by the safety margin so a
        # grazing sightline is treated as exposed rather than as cover.
        enemy_hidden_ceiling_m = np.min(enemy_requirements_m, axis=0) - float(
            self.config.hide_safety_margin_m
        )
        sorted_uav_requirements_m = np.sort(uav_requirements_m, axis=0)
        link_floor_m = sorted_uav_requirements_m[required_links - 1] + 0.25
        feasible_floor_m = np.maximum(terrain_floor_m, link_floor_m)
        feasible_ceiling_m = np.minimum(
            altitude_ceiling_m, enemy_hidden_ceiling_m
        )
        strict_candidates = (
            np.isfinite(feasible_floor_m)
            & (feasible_floor_m <= feasible_ceiling_m)
        )

        recommended_altitudes_m = np.clip(
            np.full(count, own_alt_m, dtype=np.float64),
            feasible_floor_m,
            feasible_ceiling_m,
        )
        if np.any(strict_candidates):
            strict_indices = np.nonzero(strict_candidates)[0]
            selected_index = int(
                min(
                    strict_indices,
                    key=lambda idx: (
                        float(candidate_distance_m[idx]),
                        abs(float(recommended_altitudes_m[idx]) - own_alt_m),
                        float(recommended_altitudes_m[idx]),
                        int(idx),
                    ),
                )
            )
            selected_altitude_m = float(recommended_altitudes_m[selected_index])
            strict_feasible = True
        else:
            strict_feasible = False
            notes.append(
                "No point satisfies complete enemy masking and the required UAV-link count; "
                "showing the least-exposed communication fallback."
            )
            # Evaluate only meaningful altitude events: terrain floor, current
            # altitude (clamped), and each UAV's LOS threshold. Select the best
            # event per cell, then the nearest best cell globally.
            event_altitudes = [
                terrain_floor_m,
                np.clip(
                    np.full(count, own_alt_m, dtype=np.float64),
                    terrain_floor_m,
                    altitude_ceiling_m,
                ),
            ]
            for uav_row in uav_requirements_m:
                event_altitudes.append(
                    np.clip(uav_row + 0.25, terrain_floor_m, altitude_ceiling_m)
                )

            best_event_alt_m = terrain_floor_m.copy()
            best_event_key: list[tuple[int, int, int, float] | None] = [
                None for _ in range(count)
            ]
            for event_alt_m in event_altitudes:
                enemy_visible, uav_links = self._counts_at_altitude(
                    event_alt_m,
                    enemy_requirements_m,
                    uav_requirements_m,
                    float(self.config.hide_safety_margin_m),
                )
                for idx in range(count):
                    key = (
                        0 if int(uav_links[idx]) > 0 else 1,
                        int(enemy_visible[idx]),
                        -int(uav_links[idx]),
                        abs(float(event_alt_m[idx]) - own_alt_m),
                    )
                    if best_event_key[idx] is None or key < best_event_key[idx]:
                        best_event_key[idx] = key
                        best_event_alt_m[idx] = float(event_alt_m[idx])

            def fallback_candidate_key(idx: int) -> tuple:
                quality = best_event_key[idx] or (1, 999, 0, float("inf"))
                return (
                    quality[0],
                    quality[1],
                    quality[2],
                    float(candidate_distance_m[idx]),
                    quality[3],
                    int(idx),
                )

            selected_index = int(min(range(count), key=fallback_candidate_key))
            selected_altitude_m = float(best_event_alt_m[selected_index])

        selected_x = float(xs[selected_index])
        selected_y = float(ys[selected_index])
        selected_ground_m = float(grounds_m[selected_index])
        selected_row, selected_col = dem.nearest_index(selected_x, selected_y)
        (
            selected_enemy_visible,
            selected_in_range,
            connected_uav_indices,
            selected_uav_links,
        ) = self._point_status(
            row=float(rows[selected_index]),
            col=float(cols[selected_index]),
            x=selected_x,
            y=selected_y,
            altitude_m=selected_altitude_m,
            enemies=bounded_enemies,
            uavs=bounded_uavs,
            enemy_range_m=enemy_range_m,
            communication_range_m=communication_m,
            step_reference_rows=rows,
            step_reference_cols=cols,
            step_reference_xs=xs,
            step_reference_ys=ys,
        )
        strict_feasible = bool(
            strict_feasible
            and selected_enemy_visible == 0
            and selected_uav_links >= required_links
        )
        if bool(strict_candidates[selected_index]) and not strict_feasible:
            notes.append(
                "Canonical LOS recheck disagreed with the analytic threshold; "
                "the recommendation is marked degraded."
            )

        enemy_visible_at_selected_alt, uav_links_at_selected_alt = (
            self._counts_at_altitude(
                np.full(count, selected_altitude_m, dtype=np.float64),
                enemy_requirements_m,
                uav_requirements_m,
                float(self.config.hide_safety_margin_m),
            )
        )
        strict_mask = np.zeros_like(candidate_mask, dtype=bool)
        # Show cells that have some feasible altitude interval, not merely
        # cells that work at the one altitude selected for the recommendation.
        strict_mask[grid_rows, grid_cols] = strict_candidates[:grid_count]
        enemy_visible_grid = np.zeros_like(dem.elev, dtype=np.int16)
        uav_link_grid = np.zeros_like(dem.elev, dtype=np.int16)
        enemy_visible_grid[grid_rows, grid_cols] = enemy_visible_at_selected_alt[:grid_count]
        uav_link_grid[grid_rows, grid_cols] = uav_links_at_selected_alt[:grid_count]

        (
            current_enemy_visible,
            current_enemy_in_range,
            _current_connected,
            current_uav_links,
        ) = self._point_status(
            row=float(own_ray_row),
            col=float(own_ray_col),
            x=float(own_x),
            y=float(own_y),
            altitude_m=own_alt_m,
            enemies=bounded_enemies,
            uavs=bounded_uavs,
            enemy_range_m=enemy_range_m,
            communication_range_m=communication_m,
            step_reference_rows=rows,
            step_reference_cols=cols,
            step_reference_xs=xs,
            step_reference_ys=ys,
        )

        lat, lon = dem.native_to_latlon(selected_x, selected_y)
        if strict_feasible:
            notes.append(
                f"Terrain masks every in-range enemy and maintains LOS to "
                f"{selected_uav_links}/{len(bounded_uavs)} UAVs."
            )

        return CommunicationHideResult(
            own_x=float(own_x),
            own_y=float(own_y),
            own_alt_m=own_alt_m,
            uavs=bounded_uavs,
            enemies=bounded_enemies,
            candidate_mask=candidate_mask,
            strict_mask=strict_mask,
            enemy_visible_count=enemy_visible_grid,
            uav_link_count=uav_link_grid,
            recommended_row=selected_row,
            recommended_col=selected_col,
            recommended_x=selected_x,
            recommended_y=selected_y,
            recommended_lat=float(lat),
            recommended_lon=float(lon),
            recommended_ground_m=selected_ground_m,
            recommended_alt_m=selected_altitude_m,
            horizontal_distance_m=float(candidate_distance_m[selected_index]),
            strict_feasible=strict_feasible,
            required_uav_links=required_links,
            connected_uav_indices=connected_uav_indices,
            visible_enemy_count=selected_enemy_visible,
            in_range_enemy_count=selected_in_range,
            current_uav_link_count=int(current_uav_links),
            current_visible_enemy_count=int(current_enemy_visible),
            current_in_range_enemy_count=int(current_enemy_in_range),
            elapsed_s=time.perf_counter() - t0,
            notes=notes,
        )


__all__ = [
    "CommunicationHideAnalyzer",
    "CommunicationHideResult",
    "HideEndpointCandidate",
    "UavPoint",
    "uav_from_native",
]
