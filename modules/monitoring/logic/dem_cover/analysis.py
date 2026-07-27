"""Core cover-position analysis (GUI-independent).

Given a DEM, a target-area polygon, sampled enemy positions and a Ref point,
``CoverAnalyzer`` finds one recommended "suitable area": terrain that is

  * within weapon range of the enemy set (so it is tactically relevant),
  * hidden (defiladed) from most in-range enemies (엄폐 / cover), and
  * next to firing positions that *do* have line of sight (reverse slope),

biased toward the operator's Ref point. The whole thing is pure NumPy/SciPy and
returns an :class:`AnalysisResult` that both the GUI and any downstream code can
consume without importing matplotlib.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import time

import numpy as np
from scipy import ndimage

from .config import CoverConfig
from .dem import DemGrid
from .geometry import Polygon2D, simplify_region_to_polygon
from .sampling import EnemyPoint, sample_enemy_positions
from .visibility import accumulate_engagement


@dataclass(frozen=True)
class RefPoint:
    x: float
    y: float
    ground_m: float
    lat: float
    lon: float


@dataclass
class AnalysisResult:
    # inputs echoed back
    enemies: list[EnemyPoint]
    ref: RefPoint
    polygon: Polygon2D

    # full-grid analysis products (H, W)
    candidate_mask: np.ndarray
    range_count: np.ndarray      # in-range enemies per cell
    engage_count: np.ndarray     # in-range enemies with LOS per cell (= exposure = targets)
    cover_frac: np.ndarray       # fraction of in-range enemies that CANNOT see the cell (NaN off-candidate)
    fire_score: np.ndarray       # 0..1 proximity to a firing position
    suitability: np.ndarray      # combined score (NaN where not a gated candidate)
    firing_mask: np.ndarray      # cells with LOS to >=1 in-range enemy
    suitable_mask: np.ndarray    # gated + thresholded cells
    chosen_mask: np.ndarray      # THE recommended blob

    # simplified, operationally-usable polygon of the recommended area
    recommended_polygon: list          # list[(x, y)] native, 4..6 vertices
    recommended_polygon_lla: list      # list[(lat, lon)] matching vertices

    # recommended-area summary
    centroid_xy: tuple[float, float]
    centroid_lla: tuple[float, float, float]
    rep_xy: tuple[float, float]
    rep_lla: tuple[float, float, float]
    area_m2: float
    cell_count: int
    mean_cover: float
    min_cover: float
    engageable_enemies: int
    fire_position_xy: tuple[float, float] | None
    nearest_firing_dist_m: float
    dist_to_ref_m: float
    nearest_enemy_dist_m: float
    has_fire_access: bool

    elapsed_s: float
    notes: list[str] = field(default_factory=list)

    def summary(self) -> dict:
        # Coerce non-finite values (e.g. inf when no firing spot exists, NaN
        # ground) to None so json.dumps(..., allow_nan=False) stays valid.
        def num(value, digits=1):
            v = float(value)
            return round(v, digits) if np.isfinite(v) else None

        lat, lon, alt = self.rep_lla
        return {
            "recommendedPosition": {
                "x": num(self.rep_xy[0], 2),
                "y": num(self.rep_xy[1], 2),
                "lat": num(lat, 7),
                "lon": num(lon, 7),
                "groundM": num(alt, 2),
            },
            "centroidLatLon": [num(self.centroid_lla[0], 7), num(self.centroid_lla[1], 7)],
            "recommendedAreaPolygon": [
                {"lat": round(la, 7), "lon": round(lo, 7), "x": round(px, 2), "y": round(py, 2)}
                for (px, py), (la, lo) in zip(self.recommended_polygon, self.recommended_polygon_lla)
            ],
            "recommendedAreaEdgesM": self._edge_lengths(),
            "areaM2": num(self.area_m2, 1),
            "cellCount": int(self.cell_count),
            "meanCoverPct": num(self.mean_cover * 100.0, 1),
            "minCoverPct": num(self.min_cover * 100.0, 1),
            "engageableEnemies": int(self.engageable_enemies),
            "enemyCount": len(self.enemies),
            "nearestFiringDistM": num(self.nearest_firing_dist_m, 1),
            "distToRefM": num(self.dist_to_ref_m, 1),
            "nearestEnemyDistM": num(self.nearest_enemy_dist_m, 1),
            "hasFireAccess": bool(self.has_fire_access),
            "elapsedS": round(self.elapsed_s, 3),
            "notes": list(self.notes),
        }

    def _edge_lengths(self) -> list[float]:
        poly = self.recommended_polygon
        n = len(poly)
        out: list[float] = []
        for i in range(n):
            x0, y0 = poly[i]
            x1, y1 = poly[(i + 1) % n]
            out.append(round(float(np.hypot(x1 - x0, y1 - y0)), 1))
        return out


class CoverAnalyzer:
    def __init__(self, dem: DemGrid, config: CoverConfig | None = None) -> None:
        self.dem = dem
        self.config = config or CoverConfig()

    # -- helpers -----------------------------------------------------------
    def make_ref(self, x: float, y: float) -> RefPoint:
        # Clamp a Ref that lands outside the DEM to the nearest in-bounds point
        # so its ground elevation is valid instead of NaN.
        cx = float(np.clip(x, self.dem.x_min, self.dem.x_max))
        cy = float(np.clip(y, self.dem.y_min, self.dem.y_max))
        ground = self.dem.sample_native(cx, cy)
        if not np.isfinite(ground):
            ground = 0.0
        lat, lon = self.dem.native_to_latlon(cx, cy)
        return RefPoint(cx, cy, float(ground), lat, lon)

    def sample_enemies(
        self, polygon: Polygon2D, rng: np.random.Generator | None = None
    ) -> list[EnemyPoint]:
        return sample_enemy_positions(self.dem, polygon, self.config, rng)

    # -- main entry --------------------------------------------------------
    def analyze(
        self,
        polygon: Polygon2D,
        enemies: list[EnemyPoint],
        ref: RefPoint,
    ) -> AnalysisResult:
        cfg = self.config
        dem = self.dem
        notes: list[str] = []
        t0 = time.perf_counter()

        if not enemies:
            raise ValueError(
                "No enemy positions were sampled inside the target area "
                "(polygon may be zero-area/degenerate or over nodata)."
            )

        X, Y = dem.X, dem.Y
        H, W = dem.elev.shape
        rng_m = float(cfg.weapon_range_m)

        # 1) range_count + nearest enemy distance over the whole grid.
        range_count = np.zeros((H, W), dtype=np.int32)
        nearest_enemy = np.full((H, W), np.inf, dtype=np.float64)
        ex = np.array([e.x for e in enemies])
        ey = np.array([e.y for e in enemies])
        for x, y in zip(ex, ey):
            d = np.hypot(X - x, Y - y)
            range_count += (d <= rng_m)
            np.minimum(nearest_enemy, d, out=nearest_enemy)

        # 2) candidate mask: valid terrain, within range of >=1 enemy,
        #    and (optionally) outside the enemy area itself.
        in_range_valid = dem.valid & (range_count >= 1)
        enemy_poly_mask = polygon.contains_points(X, Y)
        candidate = in_range_valid
        if cfg.exclude_enemy_polygon:
            outside = in_range_valid & (~enemy_poly_mask)
            if np.any(outside):
                candidate = outside
            else:
                # Nothing in range lies outside the target area (e.g. the area
                # spans the whole map). Fall back to allowing cover inside it.
                notes.append("Target area covers all in-range terrain; allowing cover positions inside it.")
        if not np.any(candidate):
            raise ValueError("No terrain lies within weapon range of the enemy set.")

        cand_rows, cand_cols = np.nonzero(candidate)
        cand_x = X[candidate]
        cand_y = Y[candidate]
        cand_ground = dem.elev[candidate].astype(np.float64)
        cand_z = cand_ground + float(cfg.friendly_height_m)

        # 3) engagement / exposure via symmetric LOS.
        engage_cand = accumulate_engagement(
            dem, enemies,
            cand_rows.astype(np.float64), cand_cols.astype(np.float64),
            cand_x, cand_y, cand_z, cfg,
        )
        engage_count = np.zeros((H, W), dtype=np.int32)
        engage_count[candidate] = engage_cand

        # 4) cover fraction = share of in-range enemies that cannot see the cell.
        cover_frac = np.full((H, W), np.nan, dtype=np.float64)
        rc = range_count[candidate].astype(np.float64)
        cover_frac[candidate] = 1.0 - (engage_cand / np.maximum(rc, 1.0))

        # 5) firing positions and reverse-slope access proximity.
        firing_mask = candidate & (engage_count >= 1)
        if np.any(firing_mask):
            fire_dist_cells = ndimage.distance_transform_edt(~firing_mask)
            fire_dist_m = fire_dist_cells * dem.cell_m
        else:
            fire_dist_m = np.full((H, W), np.inf, dtype=np.float64)
            notes.append("No firing positions with line of sight were found in range.")
        with np.errstate(invalid="ignore"):
            fire_score = np.clip(1.0 - fire_dist_m / float(cfg.fire_move_radius_m), 0.0, 1.0)

        # 6) Ref influence (directional by default; distance is still reported).
        dist_to_ref = np.hypot(X - ref.x, Y - ref.y)
        ref_score = self._ref_score(X, Y, ex, ey, ref, dist_to_ref, cfg)

        # 6b) Stand-off: farther from the nearest enemy is safer (0 at the enemy,
        #     1 at the weapon-range limit). Keeps you away from the enemy front.
        standoff_score = np.clip(nearest_enemy / rng_m, 0.0, 1.0)

        # 7) gated suitability score.
        gate = candidate & (cover_frac >= cfg.cover_min_frac)
        if not np.any(gate):
            # Relax: keep the best-covered candidates instead of a fixed threshold.
            thr = float(np.nanpercentile(cover_frac[candidate], 60.0))
            gate = candidate & (cover_frac >= thr)
            notes.append(
                f"cover_min_frac={cfg.cover_min_frac:.2f} was too strict; "
                f"relaxed to cover >= {thr*100:.0f}%."
            )

        score = (
            cfg.weight_cover * np.nan_to_num(cover_frac, nan=0.0)
            + cfg.weight_fire * fire_score
            + cfg.weight_ref * ref_score
            + cfg.weight_standoff * standoff_score
        )
        suitability = np.full((H, W), np.nan, dtype=np.float64)
        suitability[gate] = score[gate]

        # 8) broad suitable region = top-percentile gated cells; the single
        #    recommended zone is a compact blob anchored at the best cell.
        gate_scores = suitability[gate]
        thr_score = float(np.nanpercentile(gate_scores, cfg.select_percentile))
        suitable_mask = gate & (suitability >= thr_score)
        if not np.any(suitable_mask):
            suitable_mask = gate & (suitability >= float(np.nanmax(gate_scores)))

        chosen_mask, chosen_notes = self._select_compact(
            suitable_mask, suitability, fire_score, dem, cfg
        )
        notes.extend(chosen_notes)

        # 9) summarise the recommended area.
        result = self._summarise(
            dem, cfg, enemies, ref, polygon,
            candidate, range_count, engage_count, cover_frac,
            fire_score, suitability, firing_mask, suitable_mask, chosen_mask,
            nearest_enemy, dist_to_ref, notes, t0,
        )
        return result

    # -- Ref scoring -------------------------------------------------------
    def _ref_score(self, X, Y, ex, ey, ref, dist_to_ref, cfg):
        """Ref term over the grid. Default 'direction': reward cells that lie on
        the Ref's side of the enemy centroid (which way the cover forms),
        independent of distance."""
        mode = str(getattr(cfg, "ref_mode", "direction")).lower()
        if mode == "none":
            return np.zeros_like(X)
        if mode == "proximity":
            return np.exp(-dist_to_ref / float(cfg.ref_decay_m))
        # direction: angular alignment of (cell - enemy_centroid) with (ref - enemy_centroid)
        ecx, ecy = float(np.mean(ex)), float(np.mean(ey))
        dx, dy = float(ref.x - ecx), float(ref.y - ecy)
        dnorm = float(np.hypot(dx, dy))
        if dnorm < 1e-6:  # Ref at the enemy centroid -> no direction
            return np.full_like(X, 0.5)
        vx, vy = X - ecx, Y - ecy
        vnorm = np.hypot(vx, vy)
        with np.errstate(invalid="ignore", divide="ignore"):
            align = (vx * dx + vy * dy) / (vnorm * dnorm)
        align = np.where(np.isfinite(align), align, 0.0)
        return np.clip((align + 1.0) / 2.0, 0.0, 1.0)  # 1 = same side, 0 = opposite

    # -- compact zone selection -------------------------------------------
    def _select_compact(self, suitable_mask, suitability, fire_score, dem, cfg):
        """Anchor the recommendation at the best-scoring cell and grow a small,
        connected zone around it (radius-clipped) so the result is one usable
        battle position rather than a sprawling region."""
        notes: list[str] = []
        if not np.any(suitable_mask):
            return suitable_mask, ["No suitable cells were found."]

        scored = np.where(suitable_mask, suitability, -np.inf)
        br, bc = np.unravel_index(int(np.argmax(scored)), scored.shape)

        structure = np.ones((3, 3), dtype=int)  # 8-connectivity
        labels, n = ndimage.label(suitable_mask, structure=structure)
        lab = int(labels[br, bc])
        blob = (labels == lab) if lab > 0 else suitable_mask

        d = np.hypot(dem.X - dem.X[br, bc], dem.Y - dem.Y[br, bc])
        chosen = blob & (d <= float(cfg.recommend_radius_m))
        if int(chosen.sum()) < cfg.min_area_cells:
            chosen = blob  # suitable region already smaller than the clip disk

        if not bool(np.any(fire_score[chosen] > 0.0)):
            notes.append(
                "Recommended area has no firing position within "
                f"{cfg.fire_move_radius_m:.0f} m; cover only."
            )
        return chosen, notes

    # -- summary -----------------------------------------------------------
    def _summarise(
        self, dem, cfg, enemies, ref, polygon,
        candidate, range_count, engage_count, cover_frac,
        fire_score, suitability, firing_mask, suitable_mask, chosen_mask,
        nearest_enemy, dist_to_ref, notes, t0,
    ) -> AnalysisResult:
        X, Y = dem.X, dem.Y
        rows, cols = np.nonzero(chosen_mask)
        cx = float(np.mean(X[chosen_mask]))
        cy = float(np.mean(Y[chosen_mask]))
        clat, clon = dem.native_to_latlon(cx, cy)
        cground = dem.sample_native(cx, cy)

        # Representative point = highest-scoring cell in the chosen blob.
        blob_scores = np.where(chosen_mask, suitability, -np.inf)
        rr, rc = np.unravel_index(int(np.argmax(blob_scores)), blob_scores.shape)
        rx, ry = float(X[rr, rc]), float(Y[rr, rc])
        rlat, rlon = dem.native_to_latlon(rx, ry)
        rground = dem.sample_native(rx, ry)

        # Cover stats over the chosen area.
        cover_vals = cover_frac[chosen_mask]
        cover_vals = cover_vals[np.isfinite(cover_vals)]
        mean_cover = float(np.mean(cover_vals)) if cover_vals.size else 0.0
        min_cover = float(np.min(cover_vals)) if cover_vals.size else 0.0

        # Best firing position reachable from the chosen cover (reverse slope).
        blob_dist_m = ndimage.distance_transform_edt(~chosen_mask) * dem.cell_m
        # Distance to the nearest firing cell is defined whenever any firing cell
        # exists (independent of whether it is within the move radius).
        nearest_firing_dist = (
            float(np.min(blob_dist_m[firing_mask])) if np.any(firing_mask) else float("inf")
        )
        near_fire = firing_mask & (blob_dist_m <= float(cfg.fire_move_radius_m))
        engageable = 0
        fire_xy = None
        if np.any(near_fire):
            weighted = np.where(near_fire, engage_count, -1)
            fr, fc = np.unravel_index(int(np.argmax(weighted)), weighted.shape)
            engageable = int(engage_count[fr, fc])
            fire_xy = (float(X[fr, fc]), float(Y[fr, fc]))

        area_m2 = float(chosen_mask.sum()) * dem.cell_x_m * dem.cell_y_m
        dist_ref = float(dist_to_ref[rr, rc])
        nearest_en = float(nearest_enemy[rr, rc])
        has_fire = bool(np.any(fire_score[chosen_mask] > 0.0))

        # Simplify the chosen blob into a usable 4..6-vertex polygon.
        rec_poly = simplify_region_to_polygon(
            X[chosen_mask], Y[chosen_mask],
            max_vertices=cfg.simplify_max_vertices,
            min_vertices=cfg.simplify_min_vertices,
            area_tol=cfg.simplify_area_tol,
        )
        rec_poly_lla = [tuple(dem.native_to_latlon(px, py)) for px, py in rec_poly]

        return AnalysisResult(
            enemies=enemies,
            ref=ref,
            polygon=polygon,
            candidate_mask=candidate,
            range_count=range_count,
            engage_count=engage_count,
            cover_frac=cover_frac,
            fire_score=fire_score,
            suitability=suitability,
            firing_mask=firing_mask,
            suitable_mask=suitable_mask,
            chosen_mask=chosen_mask,
            recommended_polygon=rec_poly,
            recommended_polygon_lla=rec_poly_lla,
            centroid_xy=(cx, cy),
            centroid_lla=(clat, clon, float(cground)),
            rep_xy=(rx, ry),
            rep_lla=(rlat, rlon, float(rground)),
            area_m2=area_m2,
            cell_count=int(chosen_mask.sum()),
            mean_cover=mean_cover,
            min_cover=min_cover,
            engageable_enemies=engageable,
            fire_position_xy=fire_xy,
            nearest_firing_dist_m=nearest_firing_dist,
            dist_to_ref_m=dist_ref,
            nearest_enemy_dist_m=nearest_en,
            has_fire_access=has_fire,
            elapsed_s=time.perf_counter() - t0,
            notes=notes,
        )
