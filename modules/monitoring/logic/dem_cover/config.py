"""Configuration for the DEM cover-position analysis tool.

All tunable parameters live here as a single frozen dataclass so the logic
package can be driven head-lessly (no GUI) and reproducibly.

Coordinate convention: every ``*_m`` distance is in metres, matching the DEM's
projected CRS (EPSG:32652 / UTM 52N for the bundled rasters), so the 9 km
weapon range is used directly without any lat/lon conversion.
"""

from __future__ import annotations

from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any

from modules.common.terrain_los import (
    EARTH_CURVATURE_ENABLED,
    ENEMY_OBSERVER_HEIGHT_M,
    LOS_CLEARANCE_M,
    LOS_SAMPLES_PER_CELL,
    MAX_ENEMY_LOS_TARGETS,
)


def resolve_repo_root() -> Path:
    """Locate the project root without mistaking ``modules/resource`` for it."""
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "run.py").is_file() and (
            parent / "modules" / "common" / "db_paths.py"
        ).is_file():
            return parent
    # Fallback from modules/monitoring/logic/dem_cover to the repository root.
    return here.parents[3]


@dataclass(frozen=True)
class CoverConfig:
    # --- DEM source -------------------------------------------------------
    dem_path: str = "resource/Inje_10m.tif"
    dem_fallback_epsg: int = 32652
    # Down-sample the DEM so the longest analysis-grid side is ~this many cells.
    # ~340 gives ~150 m cells on Inje_10m and keeps the full run under a second.
    analysis_max_dim: int = 340

    # --- Tactical model ---------------------------------------------------
    weapon_range_m: float = 5000.0     # friendly engagement range (사거리)
    enemy_height_m: float = ENEMY_OBSERVER_HEIGHT_M
    friendly_height_m: float = 2.0     # exposure height of a position we occupy

    # --- Enemy sampling ---------------------------------------------------
    # The enemy count adapts to the target-area size so a small area does not
    # get an unrealistic crowd of enemies:
    #     count = round(area_km2 * enemy_density_per_km2),
    #     clamped to [enemy_min_count, max_analysis_enemies].
    # Set enemy_sample_count > 0 to force a fixed count instead of auto.
    enemy_sample_count: int = 0          # 0 = auto (area-proportional); >0 = fixed
    enemy_density_per_km2: float = 2.0   # enemies per km^2 in auto mode
    enemy_min_count: int = 6             # floor for very small areas
    # Hard cap on how many enemies the analysis uses (realistic + keeps the
    # whole run well under 5 s; runtime scales ~linearly with enemy count).
    max_analysis_enemies: int = MAX_ENEMY_LOS_TARGETS
    random_seed: int = 7

    # --- Line-of-sight / viewshed ----------------------------------------
    los_samples_per_cell: float = LOS_SAMPLES_PER_CELL
    los_max_steps: int = 192           # hard cap on samples per ray (covers a full diagonal)
    los_clearance_m: float = LOS_CLEARANCE_M
    # How far below the enemy's sightline a position must sit before it
    # counts as concealed.  A sightline that grazes the ridge by tens of
    # centimetres reads as blocked here and as open to the attack solver,
    # which produced hide points the enemy could already see.  DEM cells
    # are ~10 m, so sub-metre agreement is noise either way.
    hide_safety_margin_m: float = 5.0
    earth_curvature: bool = EARTH_CURVATURE_ENABLED

    # --- Suitability scoring ---------------------------------------------
    cover_min_frac: float = 0.60       # a candidate must be hidden from >= this share of enemies
    fire_move_radius_m: float = 600.0  # a firing spot must be reachable within this ("reverse slope")
    weight_cover: float = 1.0          # weight on how hidden the position is
    weight_fire: float = 0.6           # weight on nearby firing access
    weight_ref: float = 0.4            # weight on the Ref term (see ref_mode)
    weight_standoff: float = 0.2       # reward distance from the nearest enemy (moderate stand-off)
    # How the Ref point is used:
    #   "direction" : prefer cells on the Ref's SIDE of the enemy centroid
    #                 (which way the cover forms — up/down/left/right), distance-agnostic
    #   "proximity" : prefer cells CLOSE to the Ref (classic distance decay)
    #   "none"      : ignore the Ref in scoring
    ref_mode: str = "direction"
    ref_decay_m: float = 6000.0        # e-folding distance (proximity mode only)
    select_percentile: float = 90.0    # cells above this score percentile form the broad suitable region
    recommend_radius_m: float = 1500.0 # max radius of the single compact recommended zone
    min_area_cells: int = 6            # ignore suitable blobs smaller than this
    exclude_enemy_polygon: bool = True # never place cover inside the enemy area itself

    # --- Recommended-area simplification (usable polygon) -----------------
    simplify_max_vertices: int = 6     # simplify the chosen blob to at most this many vertices
    simplify_min_vertices: int = 4     # ... and at least this many
    simplify_area_tol: float = 0.03    # stop dropping corners once a cut exceeds this area fraction

    # ---------------------------------------------------------------------
    @property
    def dem_full_path(self) -> Path:
        raw = Path(self.dem_path)
        return raw if raw.is_absolute() else resolve_repo_root() / raw

    def with_overrides(self, **kwargs: Any) -> "CoverConfig":
        return replace(self, **kwargs)
