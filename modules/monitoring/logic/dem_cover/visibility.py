"""Vectorised line-of-sight / viewshed on the analysis grid.

Terrain LOS is symmetric: if the straight 3-D line between an enemy (at
ground + enemy_height) and a candidate position (at ground + friendly_height)
clears the intervening terrain, then the enemy can see the position *and* the
position can see the enemy. So a single sightline test answers both "am I
exposed?" and "can I engage that enemy?".

We only test enemies that are within weapon range of a candidate cell, which
bounds every ray to <= range and keeps the whole sweep well under a second.
"""

from __future__ import annotations

import numpy as np

from .config import CoverConfig
from .dem import DemGrid

# 4/3-earth-radius model folds in standard atmospheric refraction.
_EFFECTIVE_EARTH_RADIUS_M = 6_371_000.0 * 4.0 / 3.0


def _los_visible_batch(
    block_elev: np.ndarray,
    obs_row: float,
    obs_col: float,
    obs_z: float,
    tgt_rows: np.ndarray,
    tgt_cols: np.ndarray,
    tgt_z: np.ndarray,
    horiz_dist: np.ndarray,
    cell_m: float,
    config: CoverConfig,
) -> np.ndarray:
    """Boolean visibility of each target from one observer (vectorised over targets)."""
    height, width = block_elev.shape
    n = tgt_rows.size
    if n == 0:
        return np.zeros(0, dtype=bool)

    # Step count sized to the most grid cells any ray actually traverses. Using
    # the Manhattan span (|drow| + |dcol|) rather than Euclidean distance keeps
    # ~1 sample per traversed cell even for near-diagonal rays (a 45-degree ray
    # crosses up to sqrt(2)x more cells than its straight-line length).
    span = np.abs(tgt_rows - obs_row) + np.abs(tgt_cols - obs_col)
    max_span = float(np.max(span)) if span.size else 1.0
    steps = int(np.clip(int(max_span * config.los_samples_per_cell) + 1, 2, config.los_max_steps))

    t = np.linspace(0.0, 1.0, steps, dtype=np.float32).reshape(-1, 1)  # (steps, 1)
    rr = obs_row + t * (tgt_rows.astype(np.float32) - obs_row)          # (steps, n)
    cc = obs_col + t * (tgt_cols.astype(np.float32) - obs_col)
    ri = np.clip(np.rint(rr).astype(np.int32), 0, height - 1)
    ci = np.clip(np.rint(cc).astype(np.int32), 0, width - 1)

    terr = block_elev[ri, ci]                                          # (steps, n)
    zline = obs_z + t * (tgt_z.astype(np.float32) - np.float32(obs_z))  # (steps, n)

    if config.earth_curvature:
        along = t * horiz_dist.astype(np.float32)                      # horizontal dist from observer
        drop = (along * along) / np.float32(2.0 * _EFFECTIVE_EARTH_RADIUS_M)
        terr = terr - drop

    blocked = terr > (zline + np.float32(config.los_clearance_m))
    # Endpoints never block: the observer's own cell and the target's own cell.
    blocked[0, :] = False
    blocked[-1, :] = False
    return ~np.any(blocked, axis=0)


def accumulate_engagement(
    dem: DemGrid,
    enemies: list,
    cand_rows: np.ndarray,
    cand_cols: np.ndarray,
    cand_x: np.ndarray,
    cand_y: np.ndarray,
    cand_z: np.ndarray,
    config: CoverConfig,
) -> np.ndarray:
    """For each candidate cell, count in-range enemies with a clear sightline.

    Because LOS is symmetric this is simultaneously:
      * the number of enemies that can see (and hit) a unit at that cell, and
      * the number of enemies a unit at that cell could engage.
    """
    n_cand = cand_rows.size
    engage = np.zeros(n_cand, dtype=np.int32)
    if n_cand == 0:
        return engage

    block_elev = dem.block_elev
    cell_m = dem.cell_m
    rng = float(config.weapon_range_m)

    for enemy in enemies:
        d = np.hypot(cand_x - enemy.x, cand_y - enemy.y)
        in_range = d <= rng
        if not np.any(in_range):
            continue
        idx = np.nonzero(in_range)[0]
        vis = _los_visible_batch(
            block_elev,
            float(enemy.row),
            float(enemy.col),
            float(enemy.alt_m),
            cand_rows[idx],
            cand_cols[idx],
            cand_z[idx],
            d[idx],
            cell_m,
            config,
        )
        engage[idx[vis]] += 1
    return engage
