"""Fast nearest-cover search for an explicitly selected ownship and enemies.

Unlike :mod:`dem_cover.analysis`, this module does not sample an enemy area or
rank a broad tactical region.  It accepts the exact enemy points clicked by an
operator and finds the horizontally nearest DEM cell where a unit at the
configured minimum AGL is outside weapon range or terrain-masked from every
clicked enemy.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import time

import numpy as np

from .config import CoverConfig
from .dem import DemGrid
from .sampling import EnemyPoint
from .visibility import _los_visible_batch


@dataclass
class HideResult:
    own_x: float
    own_y: float
    own_alt_m: float
    enemies: list[EnemyPoint]

    candidate_mask: np.ndarray
    fully_hidden_mask: np.ndarray
    visible_count: np.ndarray
    in_range_count: np.ndarray

    recommended_row: int
    recommended_col: int
    recommended_x: float
    recommended_y: float
    recommended_lat: float
    recommended_lon: float
    recommended_ground_m: float
    recommended_alt_m: float
    horizontal_distance_m: float

    fully_hidden: bool
    visible_enemy_count: int
    in_range_enemy_count: int
    current_visible_enemy_count: int
    current_in_range_enemy_count: int
    current_enemy_visible: list[bool]
    elapsed_s: float
    notes: list[str] = field(default_factory=list)

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
            "fullyHidden": bool(self.fully_hidden),
            "visibleEnemyCount": int(self.visible_enemy_count),
            "inRangeEnemyCount": int(self.in_range_enemy_count),
            "currentVisibleEnemyCount": int(self.current_visible_enemy_count),
            "currentInRangeEnemyCount": int(self.current_in_range_enemy_count),
            "enemyCount": len(self.enemies),
            "elapsedS": round(float(self.elapsed_s), 3),
            "notes": list(self.notes),
        }


def enemy_from_native(
    dem: DemGrid,
    x: float,
    y: float,
    *,
    height_m: float = 5.0,
) -> EnemyPoint:
    """Build an exact clicked enemy point on valid DEM terrain."""

    if not dem.contains_native(float(x), float(y)):
        raise ValueError("Enemy position is outside the DEM.")
    ground_m = float(dem.sample_native(float(x), float(y)))
    if not np.isfinite(ground_m):
        raise ValueError("Enemy position is on invalid DEM terrain.")
    row, col = dem.nearest_index(float(x), float(y))
    if not bool(dem.valid[row, col]):
        raise ValueError("Enemy position is on an invalid DEM cell.")
    lat, lon = dem.native_to_latlon(float(x), float(y))
    return EnemyPoint(
        x=float(x),
        y=float(y),
        row=int(row),
        col=int(col),
        ground_m=ground_m,
        alt_m=ground_m + max(0.0, float(height_m)),
        lat=float(lat),
        lon=float(lon),
    )


class NearestHideAnalyzer:
    """Find the nearest terrain-masked point with bounded vectorised LOS work."""

    def __init__(self, dem: DemGrid, config: CoverConfig | None = None) -> None:
        self.dem = dem
        self.config = config or CoverConfig()

    def _visible_from_enemy(
        self,
        enemy: EnemyPoint,
        rows: np.ndarray,
        cols: np.ndarray,
        xs: np.ndarray,
        ys: np.ndarray,
        altitudes_m: np.ndarray,
    ) -> np.ndarray:
        if rows.size == 0:
            return np.zeros(0, dtype=bool)
        distance_m = np.hypot(xs - float(enemy.x), ys - float(enemy.y))
        return _los_visible_batch(
            self.dem.block_elev,
            float(enemy.row),
            float(enemy.col),
            float(enemy.alt_m),
            rows.astype(np.float64, copy=False),
            cols.astype(np.float64, copy=False),
            altitudes_m.astype(np.float64, copy=False),
            distance_m.astype(np.float64, copy=False),
            float(self.dem.cell_m),
            self.config,
        )

    def _point_exposure(
        self,
        *,
        row: int,
        col: int,
        x: float,
        y: float,
        altitude_m: float,
        enemies: list[EnemyPoint],
    ) -> tuple[int, int, list[bool]]:
        visible_flags: list[bool] = []
        in_range_count = 0
        visible_count = 0
        weapon_range_m = max(1.0, float(self.config.weapon_range_m))
        for enemy in enemies:
            distance_m = float(np.hypot(float(x) - enemy.x, float(y) - enemy.y))
            if distance_m > weapon_range_m:
                visible_flags.append(False)
                continue
            in_range_count += 1
            visible = bool(
                self._visible_from_enemy(
                    enemy,
                    np.array([row], dtype=np.float64),
                    np.array([col], dtype=np.float64),
                    np.array([x], dtype=np.float64),
                    np.array([y], dtype=np.float64),
                    np.array([altitude_m], dtype=np.float64),
                )[0]
            )
            visible_flags.append(visible)
            visible_count += int(visible)
        return visible_count, in_range_count, visible_flags

    def _highest_hidden_altitude(
        self,
        *,
        row: int,
        col: int,
        x: float,
        y: float,
        floor_altitude_m: float,
        own_altitude_m: float,
        enemies: list[EnemyPoint],
    ) -> float:
        """Keep as much of the current altitude as terrain masking permits."""

        floor_m = float(floor_altitude_m)
        upper_m = max(floor_m, float(own_altitude_m))
        upper_visible, _in_range, _flags = self._point_exposure(
            row=row,
            col=col,
            x=x,
            y=y,
            altitude_m=upper_m,
            enemies=enemies,
        )
        if upper_visible == 0:
            return upper_m
        if upper_m <= floor_m + 0.5:
            return floor_m

        low_m = floor_m
        high_m = upper_m
        # Terrain LOS is monotonic enough for a fixed horizontal point: raising
        # the unit cannot restore masking once the blocking crest is cleared.
        for _ in range(12):
            mid_m = (low_m + high_m) * 0.5
            visible, _in_range, _flags = self._point_exposure(
                row=row,
                col=col,
                x=x,
                y=y,
                altitude_m=mid_m,
                enemies=enemies,
            )
            if visible == 0:
                low_m = mid_m
            else:
                high_m = mid_m
        return max(floor_m, low_m)

    def analyze(
        self,
        *,
        own_x: float,
        own_y: float,
        own_altitude_m: float,
        enemies: list[EnemyPoint],
        hide_agl_m: float = 50.0,
        search_radius_m: float = 5_000.0,
    ) -> HideResult:
        t0 = time.perf_counter()
        dem = self.dem
        cfg = self.config
        notes: list[str] = []

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
        if not np.isfinite(own_ground_m):
            raise ValueError("Own position is on invalid DEM terrain.")
        own_alt_m = float(own_altitude_m)
        if not np.isfinite(own_alt_m) or own_alt_m <= own_ground_m:
            raise ValueError(
                f"Own MSL altitude must be above terrain ({own_ground_m:.1f} m)."
            )

        agl_m = max(1.0, float(hide_agl_m))
        radius_m = max(float(dem.cell_m), float(search_radius_m))
        distance_from_own = np.hypot(dem.X - float(own_x), dem.Y - float(own_y))
        candidate_mask = dem.valid & (distance_from_own <= radius_m)
        if not np.any(candidate_mask):
            raise ValueError("No valid DEM cells exist inside the hide-search radius.")

        rows, cols = np.nonzero(candidate_mask)
        xs = dem.X[candidate_mask].astype(np.float64)
        ys = dem.Y[candidate_mask].astype(np.float64)
        grounds_m = dem.elev[candidate_mask].astype(np.float64)
        candidate_altitudes_m = grounds_m + agl_m
        count = rows.size
        in_range = np.zeros(count, dtype=np.int16)
        visible = np.zeros(count, dtype=np.int16)
        weapon_range_m = max(1.0, float(cfg.weapon_range_m))

        for enemy in bounded_enemies:
            distances_m = np.hypot(xs - float(enemy.x), ys - float(enemy.y))
            threatened = distances_m <= weapon_range_m
            if not np.any(threatened):
                continue
            indices = np.nonzero(threatened)[0]
            in_range[indices] += 1
            is_visible = self._visible_from_enemy(
                enemy,
                rows[indices],
                cols[indices],
                xs[indices],
                ys[indices],
                candidate_altitudes_m[indices],
            )
            visible[indices[is_visible]] += 1

        fully_hidden_candidates = visible == 0
        candidate_distances_m = distance_from_own[candidate_mask]
        if np.any(fully_hidden_candidates):
            eligible_indices = np.nonzero(fully_hidden_candidates)[0]
            selected_index = int(
                eligible_indices[
                    np.argmin(candidate_distances_m[eligible_indices])
                ]
            )
            fully_hidden = True
        else:
            # No perfect point: minimise the exposed share, then visible count,
            # then horizontal travel.  The GUI marks this as degraded, never as
            # a successful cover recommendation.
            exposure_fraction = visible.astype(np.float64) / np.maximum(
                in_range.astype(np.float64), 1.0
            )
            selected_index = int(
                min(
                    range(count),
                    key=lambda idx: (
                        float(exposure_fraction[idx]),
                        int(visible[idx]),
                        float(candidate_distances_m[idx]),
                    ),
                )
            )
            fully_hidden = False
            notes.append(
                "No point hidden from every in-range enemy was found; showing the least-exposed candidate."
            )

        selected_row = int(rows[selected_index])
        selected_col = int(cols[selected_index])
        selected_x = float(xs[selected_index])
        selected_y = float(ys[selected_index])
        selected_ground_m = float(grounds_m[selected_index])
        floor_altitude_m = selected_ground_m + agl_m
        if fully_hidden:
            recommended_altitude_m = self._highest_hidden_altitude(
                row=selected_row,
                col=selected_col,
                x=selected_x,
                y=selected_y,
                floor_altitude_m=floor_altitude_m,
                own_altitude_m=own_alt_m,
                enemies=bounded_enemies,
            )
        else:
            recommended_altitude_m = floor_altitude_m

        final_visible, final_in_range, _final_flags = self._point_exposure(
            row=selected_row,
            col=selected_col,
            x=selected_x,
            y=selected_y,
            altitude_m=recommended_altitude_m,
            enemies=bounded_enemies,
        )
        if fully_hidden and final_visible:
            # Numeric/grid edge safety: retain the DEM+AGL result used during
            # the full-grid selection if the altitude refinement disagrees.
            recommended_altitude_m = floor_altitude_m
            final_visible, final_in_range, _final_flags = self._point_exposure(
                row=selected_row,
                col=selected_col,
                x=selected_x,
                y=selected_y,
                altitude_m=recommended_altitude_m,
                enemies=bounded_enemies,
            )

        own_row, own_col = dem.nearest_index(float(own_x), float(own_y))
        current_visible, current_in_range, current_flags = self._point_exposure(
            row=int(own_row),
            col=int(own_col),
            x=float(own_x),
            y=float(own_y),
            altitude_m=own_alt_m,
            enemies=bounded_enemies,
        )

        hidden_mask = np.zeros_like(candidate_mask, dtype=bool)
        hidden_mask[rows, cols] = fully_hidden_candidates
        visible_grid = np.zeros_like(dem.elev, dtype=np.int16)
        in_range_grid = np.zeros_like(dem.elev, dtype=np.int16)
        visible_grid[rows, cols] = visible
        in_range_grid[rows, cols] = in_range
        lat, lon = dem.native_to_latlon(selected_x, selected_y)

        if final_in_range == 0:
            notes.append("Recommended point is outside every clicked enemy's weapon range.")
        elif final_visible == 0:
            notes.append("Terrain blocks LOS from every in-range clicked enemy.")

        return HideResult(
            own_x=float(own_x),
            own_y=float(own_y),
            own_alt_m=own_alt_m,
            enemies=bounded_enemies,
            candidate_mask=candidate_mask,
            fully_hidden_mask=hidden_mask,
            visible_count=visible_grid,
            in_range_count=in_range_grid,
            recommended_row=selected_row,
            recommended_col=selected_col,
            recommended_x=selected_x,
            recommended_y=selected_y,
            recommended_lat=float(lat),
            recommended_lon=float(lon),
            recommended_ground_m=selected_ground_m,
            recommended_alt_m=float(recommended_altitude_m),
            horizontal_distance_m=float(candidate_distances_m[selected_index]),
            fully_hidden=bool(fully_hidden and final_visible == 0),
            visible_enemy_count=int(final_visible),
            in_range_enemy_count=int(final_in_range),
            current_visible_enemy_count=int(current_visible),
            current_in_range_enemy_count=int(current_in_range),
            current_enemy_visible=current_flags,
            elapsed_s=time.perf_counter() - t0,
            notes=notes,
        )


__all__ = ["HideResult", "NearestHideAnalyzer", "enemy_from_native"]

