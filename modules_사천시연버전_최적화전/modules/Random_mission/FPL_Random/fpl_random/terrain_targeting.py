from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import rasterio
from rasterio.windows import Window, from_bounds
from rasterio.warp import transform as raster_transform, transform_bounds
from scipy.ndimage import binary_closing, binary_dilation, binary_opening, gaussian_filter, label, maximum_filter, uniform_filter


@dataclass(frozen=True)
class DeployableMaskConfig:
    flat_slope_deg: float = 10.0
    rough_std_thresh: float = 3.0
    opening_size: int = 1
    closing_size: int = 2
    dilate_pixels: int = 1
    min_area_m2: float = 90.0 * 90.0 * 5.0


@dataclass(frozen=True)
class RadarConfig:
    peak_elev_threshold: float = 300.0
    ring_min_m: float = 200.0
    ring_max_m: float = 500.0
    slope_limit_deg: float = 15.0
    radar_max_range_m: float = 8000.0
    sector_width_deg: float = 120.0
    angle_step_deg: float = 10.0
    target_radius_m: float = 1200.0
    target_sample_step: int = 2
    max_ranked_candidates: int = 50
    los_observer_height_m: float = 10.0
    los_target_height_m: float = 30.0


@dataclass(frozen=True)
class SamConfig:
    min_dist_m: float = 200.0
    max_dist_m: float = 900.0
    max_slope_deg: float = 15.0
    max_rough_std: float = 5.0
    candidate_sample_step: int = 1
    max_ranked_candidates: int = 50


@dataclass(frozen=True)
class TerrainData:
    dem: np.ndarray
    transform: rasterio.Affine
    bounds: rasterio.coords.BoundingBox
    res_x: float
    res_y: float
    shift_x: float
    shift_y: float
    extent_local: tuple[float, float, float, float]
    slope_deg: np.ndarray
    rough_std: np.ndarray
    aspect_deg: np.ndarray
    deployable_mask: np.ndarray
    hillshade: np.ndarray


@dataclass(frozen=True)
class TerrainBundle:
    terrain: TerrainData
    crs: Any
    path: Path


@dataclass(frozen=True)
class ReferencePoint:
    local_x: float
    local_y: float
    world_x: float
    world_y: float
    row: int
    col: int
    elev_m: float


@dataclass(frozen=True)
class RadarCandidate:
    row: int
    col: int
    x: float
    y: float
    elev_m: float
    best_score: int
    best_angle_deg: float
    dist_to_click_m: float
    center_visible: bool
    total_score: float
    coverage_score: float
    elevation_score: float
    slope_score: float
    peak_ring_score: float
    reach_score: float


@dataclass(frozen=True)
class RadarSearchResult:
    peaks: list[tuple[int, int, float]]
    target_points: list[tuple[float, float, int, int]]
    ranked_candidates: list[RadarCandidate]
    selected_candidates: list[RadarCandidate]


@dataclass(frozen=True)
class SamCandidate:
    row: int
    col: int
    x: float
    y: float
    dist_to_radar_m: float
    parent_radar_index: int
    slope_deg: float
    rough_std_m: float
    on_deployable_mask: bool
    total_score: float
    reverse_slope_score: float
    distance_score: float


def default_mask_config() -> DeployableMaskConfig:
    return DeployableMaskConfig()


def default_radar_config() -> RadarConfig:
    return RadarConfig()


def default_sam_config() -> SamConfig:
    return SamConfig()


def load_terrain_bundle(
    path: Path,
    mask_config: DeployableMaskConfig,
    clip_bounds_wgs84: tuple[float, float, float, float] | None = None,
    clip_margin_m: float = 0.0,
) -> TerrainBundle:
    with rasterio.open(path) as dataset:
        crs = dataset.crs
        if clip_bounds_wgs84 is None:
            dem = dataset.read(1).astype(float)
            valid_mask = dataset.read_masks(1)
            transform = dataset.transform
            bounds = dataset.bounds
        else:
            lat_min, lat_max, lon_min, lon_max = clip_bounds_wgs84
            left, bottom, right, top = transform_bounds(
                "EPSG:4326",
                crs,
                float(lon_min),
                float(lat_min),
                float(lon_max),
                float(lat_max),
                densify_pts=21,
            )
            left -= float(clip_margin_m)
            bottom -= float(clip_margin_m)
            right += float(clip_margin_m)
            top += float(clip_margin_m)
            full_window = Window(0, 0, dataset.width, dataset.height)
            window = from_bounds(left, bottom, right, top, dataset.transform)
            window = window.round_offsets().round_lengths().intersection(full_window)
            if window.width <= 0 or window.height <= 0:
                raise ValueError("Target Random DEM clip is empty")
            dem = dataset.read(1, window=window).astype(float)
            valid_mask = dataset.read_masks(1, window=window)
            transform = dataset.window_transform(window)
            win_left, win_bottom, win_right, win_top = dataset.window_bounds(window)
            bounds = rasterio.coords.BoundingBox(
                left=float(win_left),
                bottom=float(win_bottom),
                right=float(win_right),
                top=float(win_top),
            )

    dem[valid_mask == 0] = np.nan
    res_x = float(transform.a)
    res_y = float(-transform.e)
    shift_x = float(bounds.left)
    shift_y = float(bounds.bottom)
    extent_local = (0.0, float(bounds.right - bounds.left), 0.0, float(bounds.top - bounds.bottom))
    slope_deg = compute_slope_deg(dem, res_x, res_y)
    rough_std = compute_rough_std(dem)
    aspect_deg = compute_aspect_deg(dem, transform)
    deployable_mask = build_deployable_mask(dem, slope_deg, rough_std, transform, mask_config)
    hillshade = hillshade_simple(dem, res_x, res_y)
    terrain = TerrainData(
        dem=dem,
        transform=transform,
        bounds=bounds,
        res_x=res_x,
        res_y=res_y,
        shift_x=shift_x,
        shift_y=shift_y,
        extent_local=extent_local,
        slope_deg=slope_deg,
        rough_std=rough_std,
        aspect_deg=aspect_deg,
        deployable_mask=deployable_mask,
        hillshade=hillshade,
    )
    return TerrainBundle(terrain=terrain, crs=crs, path=Path(path))


def lonlat_to_local(lat: float, lon: float, bundle: TerrainBundle) -> tuple[float, float] | None:
    xs, ys = raster_transform("EPSG:4326", bundle.crs, [float(lon)], [float(lat)])
    if not xs or not ys:
        return None
    return world_to_local(float(xs[0]), float(ys[0]), bundle.terrain.shift_x, bundle.terrain.shift_y)


def world_to_lonlat(x_world: float, y_world: float, bundle: TerrainBundle) -> tuple[float, float] | None:
    lons, lats = raster_transform(bundle.crs, "EPSG:4326", [float(x_world)], [float(y_world)])
    if not lons or not lats:
        return None
    return float(lats[0]), float(lons[0])


def compute_slope_deg(dem: np.ndarray, res_x: float, res_y: float) -> np.ndarray:
    dzdx = np.gradient(dem, axis=1) / res_x
    dzdy = np.gradient(dem, axis=0) / res_y
    slope_rad = np.arctan(np.sqrt(dzdx ** 2 + dzdy ** 2))
    return np.degrees(slope_rad)


def compute_rough_std(dem: np.ndarray) -> np.ndarray:
    arr = np.nan_to_num(dem, nan=float(np.nanmean(dem)))
    mean3 = uniform_filter(arr, size=3, mode="nearest")
    mean2 = uniform_filter(arr * arr, size=3, mode="nearest")
    return np.sqrt(np.maximum(mean2 - mean3 ** 2, 0.0))


def compute_aspect_deg(dem: np.ndarray, transform: rasterio.Affine) -> np.ndarray:
    grad_north, grad_east = np.gradient(dem, -transform.e, transform.a)
    aspect_deg = np.degrees(np.arctan2(-grad_east, -grad_north))
    return (aspect_deg + 360.0) % 360.0


def build_deployable_mask(
    dem: np.ndarray,
    slope_deg: np.ndarray,
    rough_std: np.ndarray,
    transform: rasterio.Affine,
    mask_config: DeployableMaskConfig,
) -> np.ndarray:
    valid = np.isfinite(dem)
    mask = valid & (slope_deg <= mask_config.flat_slope_deg) & (rough_std <= mask_config.rough_std_thresh)
    if mask_config.opening_size > 0:
        size = 1 + 2 * mask_config.opening_size
        mask = binary_opening(mask, structure=np.ones((size, size), dtype=bool))
    if mask_config.closing_size > 0:
        size = 1 + 2 * mask_config.closing_size
        mask = binary_closing(mask, structure=np.ones((size, size), dtype=bool))
    if mask_config.dilate_pixels > 0:
        size = 1 + 2 * mask_config.dilate_pixels
        mask = binary_dilation(mask, structure=np.ones((size, size), dtype=bool))

    pixel_area = abs(transform.a) * abs(transform.e)
    labeled, n_labels = label(mask)
    keep = np.zeros_like(mask, dtype=bool)
    for idx in range(1, n_labels + 1):
        count = np.count_nonzero(labeled == idx)
        if count * pixel_area >= mask_config.min_area_m2:
            keep |= labeled == idx
    return keep


def hillshade_simple(dem: np.ndarray, res_x: float, res_y: float, az_deg: float = 315.0, alt_deg: float = 45.0) -> np.ndarray:
    grad_x, grad_y = np.gradient(dem, res_x, res_y)
    slope_vis = np.pi / 2.0 - np.arctan(np.hypot(grad_x, grad_y))
    aspect_vis = np.arctan2(-grad_x, grad_y)
    az = np.deg2rad(az_deg)
    alt = np.deg2rad(alt_deg)
    hill = np.sin(alt) * np.sin(slope_vis) + np.cos(alt) * np.cos(slope_vis) * np.cos(az - aspect_vis)
    hill_min = float(np.nanmin(hill))
    hill_max = float(np.nanmax(hill))
    return (hill - hill_min) / (hill_max - hill_min + 1e-6)


def rc_to_xy(row: int, col: int, transform: rasterio.Affine) -> tuple[float, float]:
    x = transform.c + col * transform.a + row * transform.b
    y = transform.f + col * transform.d + row * transform.e
    return float(x), float(y)


def xy_to_rc(x: float, y: float, transform: rasterio.Affine) -> tuple[int, int]:
    col = int(round((x - transform.c) / transform.a))
    row = int(round((y - transform.f) / transform.e))
    return row, col


def world_to_local(x_world: float, y_world: float, shift_x: float, shift_y: float) -> tuple[float, float]:
    return x_world - shift_x, y_world - shift_y


def local_to_world(x_local: float, y_local: float, shift_x: float, shift_y: float) -> tuple[float, float]:
    return x_local + shift_x, y_local + shift_y


def make_reference_point(terrain: TerrainData, local_x: float, local_y: float) -> ReferencePoint:
    x_min, x_max = terrain.extent_local[0], terrain.extent_local[1]
    y_min, y_max = terrain.extent_local[2], terrain.extent_local[3]
    if not (x_min <= local_x <= x_max and y_min <= local_y <= y_max):
        raise ValueError("Reference point is outside the DEM extent")
    world_x, world_y = local_to_world(local_x, local_y, terrain.shift_x, terrain.shift_y)
    row, col = xy_to_rc(world_x, world_y, terrain.transform)
    if row < 0 or col < 0 or row >= terrain.dem.shape[0] or col >= terrain.dem.shape[1]:
        raise ValueError("Reference point does not map to a valid grid cell")
    if not np.isfinite(terrain.dem[row, col]):
        raise ValueError("Reference point falls on an invalid DEM cell")
    return ReferencePoint(
        local_x=float(local_x),
        local_y=float(local_y),
        world_x=float(world_x),
        world_y=float(world_y),
        row=row,
        col=col,
        elev_m=float(terrain.dem[row, col]),
    )


def bresenham_line(row0: int, col0: int, row1: int, col1: int) -> list[tuple[int, int]]:
    points: list[tuple[int, int]] = []
    d_row = abs(int(row1) - int(row0))
    d_col = abs(int(col1) - int(col0))
    s_row = 1 if row1 >= row0 else -1
    s_col = 1 if col1 >= col0 else -1
    row = int(row0)
    col = int(col0)
    if d_col > d_row:
        err = d_col // 2
        for _ in range(d_col + 1):
            points.append((row, col))
            col += s_col
            err -= d_row
            if err < 0:
                row += s_row
                err += d_col
    else:
        err = d_row // 2
        for _ in range(d_row + 1):
            points.append((row, col))
            row += s_row
            err -= d_col
            if err < 0:
                col += s_col
                err += d_row
    return points


def los_visible(
    dem: np.ndarray,
    observer_row: int,
    observer_col: int,
    target_row: int,
    target_col: int,
    obs_height_m: float = 10.0,
    tgt_height_m: float = 30.0,
) -> bool:
    line = bresenham_line(observer_row, observer_col, target_row, target_col)
    if len(line) <= 2:
        return True
    obs_elev = dem[observer_row, observer_col] + obs_height_m
    tgt_elev = dem[target_row, target_col] + tgt_height_m
    steps = len(line) - 1
    for idx, (row, col) in enumerate(line[1:-1], start=1):
        if not np.isfinite(dem[row, col]):
            continue
        ratio = idx / steps
        interp = obs_elev + ratio * (tgt_elev - obs_elev)
        if dem[row, col] + 0.5 > interp:
            return False
    return True


def search_radar_candidates(
    terrain: TerrainData,
    reference: ReferencePoint,
    radar_config: RadarConfig,
) -> RadarSearchResult:
    peaks = detect_peaks_over_threshold(terrain.dem, radar_config.peak_elev_threshold)
    ring_candidates = build_ring_candidates(terrain, peaks, radar_config)
    target_points = build_target_points(terrain, reference, radar_config.target_radius_m, radar_config.target_sample_step)
    if not ring_candidates:
        return RadarSearchResult(peaks=peaks, target_points=target_points, ranked_candidates=[], selected_candidates=[])

    elev_values = np.array([terrain.dem[row, col] for row, col in ring_candidates], dtype=float)
    elev_min = float(np.nanmin(elev_values))
    elev_max = float(np.nanmax(elev_values))
    max_target_points = max(1, len(target_points))
    ranked_candidates: list[RadarCandidate] = []
    for row, col in ring_candidates:
        x, y = rc_to_xy(row, col, terrain.transform)
        best_sector_hits, best_angle = evaluate_sector_coverage_for_candidate(x, y, target_points, radar_config)
        center_visible = los_visible(
            terrain.dem,
            row,
            col,
            reference.row,
            reference.col,
            obs_height_m=radar_config.los_observer_height_m,
            tgt_height_m=radar_config.los_target_height_m,
        )
        dist_to_click = math.hypot(x - reference.world_x, y - reference.world_y)
        coverage_score = best_sector_hits / max_target_points
        elevation_score = normalize_value(float(terrain.dem[row, col]), elev_min, elev_max)
        slope_score = smooth_descending_score(float(terrain.slope_deg[row, col]), radar_config.slope_limit_deg * 2.0)
        peak_ring_score = score_peak_ring_alignment(x, y, terrain, peaks, radar_config)
        reach_score = score_radar_reach(dist_to_click, radar_config.radar_max_range_m, radar_config.target_radius_m)
        visibility_score = 1.0 if center_visible else 0.0
        total_score = (
            0.38 * coverage_score
            + 0.22 * visibility_score
            + 0.16 * reach_score
            + 0.14 * elevation_score
            + 0.06 * slope_score
            + 0.04 * peak_ring_score
        )
        ranked_candidates.append(
            RadarCandidate(
                row=row,
                col=col,
                x=float(x),
                y=float(y),
                elev_m=float(terrain.dem[row, col]),
                best_score=int(best_sector_hits),
                best_angle_deg=float(best_angle),
                dist_to_click_m=float(dist_to_click),
                center_visible=bool(center_visible),
                total_score=float(total_score),
                coverage_score=float(coverage_score),
                elevation_score=float(elevation_score),
                slope_score=float(slope_score),
                peak_ring_score=float(peak_ring_score),
                reach_score=float(reach_score),
            )
        )

    ranked_candidates.sort(
        key=lambda item: (item.total_score, int(item.center_visible), item.best_score, item.elev_m),
        reverse=True,
    )
    selected_candidates = ranked_candidates[: radar_config.max_ranked_candidates]
    return RadarSearchResult(
        peaks=peaks,
        target_points=target_points,
        ranked_candidates=ranked_candidates,
        selected_candidates=selected_candidates,
    )


def detect_peaks_over_threshold(dem: np.ndarray, min_elev: float) -> list[tuple[int, int, float]]:
    base = float(np.nanmean(dem))
    smooth = gaussian_filter(np.nan_to_num(dem, nan=base), sigma=1)
    neighborhood = maximum_filter(smooth, size=3)
    mask = (smooth == neighborhood) & (dem >= min_elev) & np.isfinite(dem)
    indices = np.argwhere(mask)
    return [(int(row), int(col), float(dem[row, col])) for row, col in indices]


def build_ring_candidates(
    terrain: TerrainData,
    peaks: list[tuple[int, int, float]],
    radar_config: RadarConfig,
) -> list[tuple[int, int]]:
    candidates: list[tuple[int, int]] = []
    seen_cells: set[tuple[int, int]] = set()
    rows, cols = terrain.dem.shape
    for peak_row, peak_col, _ in peaks:
        row_min = max(0, int(peak_row - radar_config.ring_max_m / terrain.res_y))
        row_max = min(rows - 1, int(peak_row + radar_config.ring_max_m / terrain.res_y))
        col_span = int(radar_config.ring_max_m / terrain.res_x)
        for row in range(row_min, row_max + 1):
            delta_y = (row - peak_row) * terrain.res_y
            if abs(delta_y) > radar_config.ring_max_m:
                continue
            col_min = max(0, peak_col - col_span)
            col_max = min(cols - 1, peak_col + col_span)
            for col in range(col_min, col_max + 1):
                if not np.isfinite(terrain.dem[row, col]):
                    continue
                delta_x = (col - peak_col) * terrain.res_x
                dist = math.hypot(delta_x, delta_y)
                if dist < radar_config.ring_min_m or dist > radar_config.ring_max_m:
                    continue
                if terrain.dem[row, col] < radar_config.peak_elev_threshold:
                    continue
                if terrain.slope_deg[row, col] > radar_config.slope_limit_deg:
                    continue
                key = (row, col)
                if key in seen_cells:
                    continue
                seen_cells.add(key)
                candidates.append(key)
    return candidates


def build_target_points(
    terrain: TerrainData,
    reference: ReferencePoint,
    radius_m: float,
    step: int,
) -> list[tuple[float, float, int, int]]:
    target_points: list[tuple[float, float, int, int]] = []
    row_radius = int(radius_m / terrain.res_y) + 1
    col_radius = int(radius_m / terrain.res_x) + 1
    row_min = max(0, reference.row - row_radius)
    row_max = min(terrain.dem.shape[0] - 1, reference.row + row_radius)
    col_min = max(0, reference.col - col_radius)
    col_max = min(terrain.dem.shape[1] - 1, reference.col + col_radius)
    for row in range(row_min, row_max + 1, max(1, step)):
        delta_y = (row - reference.row) * terrain.res_y
        for col in range(col_min, col_max + 1, max(1, step)):
            if not np.isfinite(terrain.dem[row, col]):
                continue
            delta_x = (col - reference.col) * terrain.res_x
            if math.hypot(delta_x, delta_y) > radius_m:
                continue
            x, y = rc_to_xy(row, col, terrain.transform)
            target_points.append((x, y, row, col))
    if not target_points:
        target_points.append((reference.world_x, reference.world_y, reference.row, reference.col))
    return target_points


def evaluate_sector_coverage_for_candidate(
    candidate_x: float,
    candidate_y: float,
    target_points: list[tuple[float, float, int, int]],
    radar_config: RadarConfig,
) -> tuple[int, float]:
    xs = np.array([point[0] for point in target_points], dtype=float)
    ys = np.array([point[1] for point in target_points], dtype=float)
    delta_x = xs - candidate_x
    delta_y = ys - candidate_y
    dist = np.hypot(delta_x, delta_y)
    theta = (np.degrees(np.arctan2(delta_y, delta_x)) + 360.0) % 360.0
    best_score = 0
    best_angle = 0.0
    for angle in np.arange(0.0, 360.0, radar_config.angle_step_deg):
        low = angle_wrap_deg(angle - radar_config.sector_width_deg / 2.0)
        high = angle_wrap_deg(angle + radar_config.sector_width_deg / 2.0)
        if low <= high:
            angle_mask = (theta >= low) & (theta <= high)
        else:
            angle_mask = (theta >= low) | (theta <= high)
        dist_mask = (dist > 0.0) & (dist <= radar_config.radar_max_range_m)
        score = int(np.count_nonzero(angle_mask & dist_mask))
        if score > best_score:
            best_score = score
            best_angle = float(angle)
    return best_score, best_angle


def score_peak_ring_alignment(
    candidate_x: float,
    candidate_y: float,
    terrain: TerrainData,
    peaks: list[tuple[int, int, float]],
    radar_config: RadarConfig,
) -> float:
    if not peaks:
        return 0.5
    peak_distances = []
    for row, col, _ in peaks:
        peak_x, peak_y = rc_to_xy(row, col, terrain.transform)
        peak_distances.append(math.hypot(candidate_x - peak_x, candidate_y - peak_y))
    nearest_peak_dist = min(peak_distances)
    if radar_config.ring_min_m <= nearest_peak_dist <= radar_config.ring_max_m:
        return 1.0
    if nearest_peak_dist < radar_config.ring_min_m:
        return clamp01(nearest_peak_dist / max(radar_config.ring_min_m, 1.0))
    over = nearest_peak_dist - radar_config.ring_max_m
    return clamp01(1.0 - over / max(radar_config.ring_max_m, 1.0))


def score_radar_reach(dist_to_click: float, max_range_m: float, target_radius_m: float) -> float:
    safe_range = max(max_range_m - target_radius_m, 1.0)
    if dist_to_click <= safe_range:
        return 1.0
    over = dist_to_click - safe_range
    return clamp01(1.0 - over / max(max_range_m, 1.0))


def search_sam_candidates(
    terrain: TerrainData,
    reference: ReferencePoint,
    radar_candidates: list[RadarCandidate],
    sam_config: SamConfig,
) -> list[SamCandidate]:
    if not radar_candidates:
        return []
    candidate_map: dict[tuple[int, int], SamCandidate] = {}
    step = max(1, sam_config.candidate_sample_step)
    ideal_dist = (sam_config.min_dist_m + sam_config.max_dist_m) / 2.0
    dist_halfspan = max((sam_config.max_dist_m - sam_config.min_dist_m) / 2.0, terrain.res_x)
    for radar_index, radar in enumerate(radar_candidates, start=1):
        radar_to_click_angle_deg = (math.degrees(math.atan2(reference.world_y - radar.y, reference.world_x - radar.x)) + 360.0) % 360.0
        row_min = max(0, int(radar.row - sam_config.max_dist_m / terrain.res_y))
        row_max = min(terrain.dem.shape[0] - 1, int(radar.row + sam_config.max_dist_m / terrain.res_y))
        col_span = int(sam_config.max_dist_m / terrain.res_x)
        for row in range(row_min, row_max + 1, step):
            delta_y = (row - radar.row) * terrain.res_y
            if abs(delta_y) > sam_config.max_dist_m:
                continue
            col_min = max(0, radar.col - col_span)
            col_max = min(terrain.dem.shape[1] - 1, radar.col + col_span)
            for col in range(col_min, col_max + 1, step):
                if not np.isfinite(terrain.dem[row, col]):
                    continue
                delta_x = (col - radar.col) * terrain.res_x
                dist = math.hypot(delta_x, delta_y)
                if dist <= 0.0 or dist > sam_config.max_dist_m:
                    continue
                reverse_slope_score = score_reverse_slope(terrain.aspect_deg[row, col], radar_to_click_angle_deg)
                slope_score = smooth_descending_score(float(terrain.slope_deg[row, col]), sam_config.max_slope_deg * 2.0)
                rough_score = smooth_descending_score(float(terrain.rough_std[row, col]), sam_config.max_rough_std * 2.0)
                on_deployable_mask = bool(terrain.deployable_mask[row, col])
                mask_score = 1.0 if on_deployable_mask else 0.0
                distance_score = score_preferred_distance(dist, ideal_dist, dist_halfspan)
                total_score = (
                    0.30 * reverse_slope_score
                    + 0.22 * slope_score
                    + 0.22 * rough_score
                    + 0.16 * mask_score
                    + 0.06 * distance_score
                    + 0.04 * radar.total_score
                )
                x, y = rc_to_xy(row, col, terrain.transform)
                candidate = SamCandidate(
                    row=row,
                    col=col,
                    x=float(x),
                    y=float(y),
                    dist_to_radar_m=float(dist),
                    parent_radar_index=radar_index,
                    slope_deg=float(terrain.slope_deg[row, col]),
                    rough_std_m=float(terrain.rough_std[row, col]),
                    on_deployable_mask=on_deployable_mask,
                    total_score=float(total_score),
                    reverse_slope_score=float(reverse_slope_score),
                    distance_score=float(distance_score),
                )
                key = (row, col)
                previous = candidate_map.get(key)
                if previous is None or candidate.total_score > previous.total_score:
                    candidate_map[key] = candidate
    ranked = sorted(candidate_map.values(), key=lambda item: (item.total_score, item.on_deployable_mask, -item.dist_to_radar_m), reverse=True)
    return ranked[: sam_config.max_ranked_candidates]


def score_reverse_slope(aspect_deg: float, radar_angle_deg: float) -> float:
    angle_diff = angle_diff_abs_deg(aspect_deg, radar_angle_deg)
    return clamp01(angle_diff / 180.0)


def score_preferred_distance(dist: float, ideal_dist: float, halfspan: float) -> float:
    if halfspan <= 0.0:
        return 0.0
    return clamp01(1.0 - abs(dist - ideal_dist) / halfspan)


def normalize_value(value: float, vmin: float, vmax: float) -> float:
    if vmax <= vmin:
        return 0.5
    return clamp01((value - vmin) / (vmax - vmin))


def smooth_descending_score(value: float, pivot: float) -> float:
    if pivot <= 0.0:
        return 0.0
    return clamp01(1.0 - value / pivot)


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def angle_wrap_deg(angle: float) -> float:
    return (angle + 360.0) % 360.0


def angle_diff_abs_deg(angle_a: float, angle_b: float) -> float:
    return abs((angle_a - angle_b + 180.0) % 360.0 - 180.0)
