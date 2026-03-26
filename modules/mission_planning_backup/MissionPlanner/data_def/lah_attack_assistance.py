from __future__ import annotations

import argparse
import json
import math
import os
from collections import OrderedDict
from dataclasses import dataclass
from functools import lru_cache
from typing import Iterable, List, Optional, Sequence, Set, Tuple

import numpy as np

try:
    from shapely.geometry import MultiPolygon, Polygon
    from shapely.ops import unary_union

    HAS_SHAPELY = True
except ImportError:
    HAS_SHAPELY = False
    Polygon = None  # type: ignore
    MultiPolygon = None  # type: ignore
    unary_union = None  # type: ignore

try:
    from osgeo import gdal
except ImportError as exc:
    raise SystemExit(
        "GDAL for Python is required. Install it via 'pip install GDAL' and rerun."
    ) from exc


HALF_ARC_DEGREES = 20.0  # legacy tuning placeholder
NUM_ARC_RAYS = 720
ARC_SAMPLE_STEP_PX = 0.5
ANALYSIS_RADIUS_METERS = 2000.0
ENEMY_HEIGHT_OFFSET = 10.0
POLYGON_SIMPLIFY_TOLERANCE_M = 50.0
DANGER_MIN_CELLS = 40
ATTACK_CANDIDATE_COUNT = 3
ANALYSIS_MARGIN_METERS = 1000.0

_ELEVATION_CACHE_MAX = 4
_ELEVATION_CACHE: "OrderedDict[Tuple[Any, ...], Tuple[np.ndarray, Tuple[float, ...], Tuple[str, ...]]]" = OrderedDict()


@dataclass
class ArcResult:
    world_x: np.ndarray
    world_y: np.ndarray
    visible_mask: np.ndarray
    valid_mask: np.ndarray
    boundary: Tuple[np.ndarray, np.ndarray]
    pixel_size_m: float
    enemy_world: Tuple[float, float]


@dataclass
class RasterInfo:
    path: str
    bounds: Tuple[float, float, float, float]
    projection: Optional[str]
    nodata: Optional[float]
    pixel_size_x: Optional[float]
    pixel_size_y: Optional[float]


def _list_tif_files(directory: str) -> List[str]:
    tif_candidates: List[str] = []
    if not os.path.isdir(directory):
        return tif_candidates
    for entry in os.listdir(directory):
        if entry.lower().endswith(".tif"):
            tif_candidates.append(os.path.join(directory, entry))
    return tif_candidates


def _raster_signature(raster_paths: Sequence[str]) -> Tuple[Tuple[str, int, int], ...]:
    signature: List[Tuple[str, int, int]] = []
    for path in raster_paths:
        abspath = os.path.abspath(path)
        try:
            stat = os.stat(abspath)
            signature.append((abspath, int(stat.st_mtime_ns), int(stat.st_size)))
        except OSError:
            signature.append((abspath, 0, 0))
    return tuple(signature)


def detect_raster_paths(preferred_path: Optional[str] = None) -> List[str]:
    return list(_detect_raster_paths_cached(preferred_path))


@lru_cache(maxsize=8)
def _detect_raster_paths_cached(preferred_path: Optional[str] = None) -> Tuple[str, ...]:
    """
    Returns every available GeoTIFF path. If preferred_path points to a file it is used directly,
    if it points to a directory the *.tif files inside are returned. With no preferred_path the
    default resource/ (then resources/) directories are scanned.
    """
    candidates: List[str] = []
    if preferred_path:
        if os.path.isfile(preferred_path):
            candidates = [preferred_path]
        elif os.path.isdir(preferred_path):
            candidates = _list_tif_files(preferred_path)
        else:
            raise FileNotFoundError(
                f"--raster-path '{preferred_path}' is neither a file nor a directory."
            )
    else:
        candidates = _list_tif_files("resource")
        candidates.extend(_list_tif_files("resources"))
    candidates = sorted(set(os.path.abspath(path) for path in candidates))
    if not candidates:
        raise FileNotFoundError(
            "No GeoTIFF (*.tif) files found. Place them under 'resource/' (or provide --raster-path)."
        )
    return tuple(candidates)


def detect_raster_path(resources_dir: str = "resource") -> str:
    """
    Backwards-compatible helper that returns the first available raster.
    """
    return detect_raster_paths(resources_dir)[0]


def _dataset_bounds_from_transform(
    geotransform: Sequence[float], width: int, height: int
) -> Tuple[float, float, float, float]:
    corners = [
        (geotransform[0], geotransform[3]),
        (geotransform[0] + width * geotransform[1], geotransform[3] + width * geotransform[4]),
        (geotransform[0] + height * geotransform[2], geotransform[3] + height * geotransform[5]),
        (
            geotransform[0] + width * geotransform[1] + height * geotransform[2],
            geotransform[3] + width * geotransform[4] + height * geotransform[5],
        ),
    ]
    xs = [pt[0] for pt in corners]
    ys = [pt[1] for pt in corners]
    return min(xs), min(ys), max(xs), max(ys)


def _gather_raster_infos(raster_paths: Sequence[str]) -> List[RasterInfo]:
    return list(_gather_raster_infos_cached(_raster_signature(raster_paths)))


@lru_cache(maxsize=8)
def _gather_raster_infos_cached(
    raster_signature: Tuple[Tuple[str, int, int], ...],
) -> Tuple[RasterInfo, ...]:
    infos: List[RasterInfo] = []
    for path, _mtime_ns, _size in raster_signature:
        dataset = gdal.Open(path, gdal.GA_ReadOnly)
        if dataset is None:
            continue
        geotransform = dataset.GetGeoTransform(can_return_null=True)
        if not geotransform:
            dataset = None
            continue
        bounds = _dataset_bounds_from_transform(geotransform, dataset.RasterXSize, dataset.RasterYSize)
        band = dataset.GetRasterBand(1)
        nodata = band.GetNoDataValue()
        info = RasterInfo(
            path=path,
            bounds=bounds,
            projection=dataset.GetProjection(),
            nodata=nodata,
            pixel_size_x=abs(geotransform[1]) if geotransform[1] else None,
            pixel_size_y=abs(geotransform[5]) if geotransform[5] else None,
        )
        infos.append(info)
        dataset = None
    if not infos:
        raise RuntimeError("Failed to read metadata from any GeoTIFF resources.")
    return tuple(infos)


def _bounds_intersect(a: Tuple[float, float, float, float], b: Tuple[float, float, float, float]) -> bool:
    return not (a[2] < b[0] or a[0] > b[2] or a[3] < b[1] or a[1] > b[3])


def _point_in_bounds(point: Tuple[float, float], bounds: Tuple[float, float, float, float]) -> bool:
    x, y = point
    return bounds[0] <= x <= bounds[2] and bounds[1] <= y <= bounds[3]


def _analysis_bounds(center_world: Tuple[float, float], radius_m: float, margin_m: float) -> Tuple[float, float, float, float]:
    lon, lat = center_world
    lat_scale = meters_per_degree_lat(lat)
    lon_scale = meters_per_degree_lon(lat)
    lat_delta = (radius_m + margin_m) / max(lat_scale, 1e-6)
    lon_delta = (radius_m + margin_m) / max(lon_scale, 1e-6)
    min_lon = max(-180.0, lon - lon_delta)
    max_lon = min(180.0, lon + lon_delta)
    min_lat = max(-90.0, lat - lat_delta)
    max_lat = min(90.0, lat + lat_delta)
    return (min_lon, min_lat, max_lon, max_lat)


def load_elevation(
    raster_paths: Sequence[str],
    center_world: Tuple[float, float],
    radius_m: float,
    margin_m: float = ANALYSIS_MARGIN_METERS,
) -> Tuple[np.ndarray, Optional[Sequence[float]], List[str]]:
    if not raster_paths:
        raise FileNotFoundError("No GeoTIFF resources were provided.")

    raster_sig = _raster_signature(raster_paths)
    infos = _gather_raster_infos_cached(raster_sig)
    bounds = _analysis_bounds(center_world, radius_m, margin_m)
    cache_key = (
        raster_sig,
        tuple(round(float(value), 6) for value in bounds),
    )
    cached = _ELEVATION_CACHE.get(cache_key)
    if cached is not None:
        _ELEVATION_CACHE.move_to_end(cache_key)
        elevation_cached, geotransform_cached, used_paths_cached = cached
        return elevation_cached, geotransform_cached, list(used_paths_cached)
    intersecting_infos = [info for info in infos if _bounds_intersect(info.bounds, bounds)]
    if not intersecting_infos:
        raise RuntimeError(
            "Analysis bounds extend outside the available GeoTIFF coverage. "
            "Add more tiles or adjust --radius-m."
        )

    covering_info = next((info for info in intersecting_infos if _point_in_bounds(center_world, info.bounds)), None)
    if covering_info is None:
        covering_info = next((info for info in infos if _point_in_bounds(center_world, info.bounds)), None)
    if covering_info is None:
        raise RuntimeError(
            "The enemy coordinate is outside every provided GeoTIFF tile. "
            "Verify the --enemy-lat/--enemy-lon inputs."
        )

    nodata_values: Set[float] = {
        info.nodata
        for info in intersecting_infos
        if info.nodata is not None and not math.isnan(info.nodata)
    }
    dst_nodata = next(iter(nodata_values)) if nodata_values else -32767.0
    dst_srs = covering_info.projection or intersecting_infos[0].projection or None

    warp_options = gdal.WarpOptions(
        format="MEM",
        outputBounds=bounds,
        dstSRS=dst_srs if dst_srs else None,
        errorThreshold=0.0,
        multithread=True,
        resampleAlg=gdal.GRA_Bilinear,
        dstNodata=dst_nodata,
    )
    mosaic = gdal.Warp("", [info.path for info in intersecting_infos], options=warp_options)
    if mosaic is None:
        raise RuntimeError("GDAL failed to build an in-memory mosaic for the requested area.")

    band = mosaic.GetRasterBand(1)
    elevation = band.ReadAsArray().astype(float)
    band_nodata = band.GetNoDataValue()
    nodata_values = set(nodata_values)
    if band_nodata is not None and not math.isnan(band_nodata):
        nodata_values.add(band_nodata)
    for nodata in nodata_values:
        elevation[elevation == nodata] = np.nan

    geotransform = mosaic.GetGeoTransform(can_return_null=True)
    if not geotransform:
        raise RuntimeError("Mosaic GeoTIFF is missing georeferencing information.")
    band = None
    mosaic = None
    used_paths = [info.path for info in intersecting_infos]
    elevation.setflags(write=False)
    geotransform_tuple = tuple(float(value) for value in geotransform)
    _ELEVATION_CACHE[cache_key] = (elevation, geotransform_tuple, tuple(used_paths))
    _ELEVATION_CACHE.move_to_end(cache_key)
    while len(_ELEVATION_CACHE) > _ELEVATION_CACHE_MAX:
        _ELEVATION_CACHE.popitem(last=False)
    return elevation, geotransform_tuple, used_paths


def world_to_pixel(x: float, y: float, geotransform: Optional[Sequence[float]]):
    if not geotransform:
        return x, y

    det = geotransform[1] * geotransform[5] - geotransform[2] * geotransform[4]
    if det == 0:
        raise ValueError("Invalid geotransform (determinant = 0).")

    px = (geotransform[5] * (x - geotransform[0]) - geotransform[2] * (y - geotransform[3])) / det
    py = (-geotransform[4] * (x - geotransform[0]) + geotransform[1] * (y - geotransform[3])) / det
    return px, py


def pixel_to_world_point(px: float, py: float, geotransform: Optional[Sequence[float]]):
    if not geotransform:
        return px, py
    x = geotransform[0] + px * geotransform[1] + py * geotransform[2]
    y = geotransform[3] + px * geotransform[4] + py * geotransform[5]
    return x, y


def pixel_to_world_arrays(xs: np.ndarray, ys: np.ndarray, geotransform: Optional[Sequence[float]]):
    xs = np.asarray(xs, dtype=float)
    ys = np.asarray(ys, dtype=float)
    if not geotransform:
        return xs, ys
    x_world = geotransform[0] + xs * geotransform[1] + ys * geotransform[2]
    y_world = geotransform[3] + xs * geotransform[4] + ys * geotransform[5]
    return x_world, y_world


def bilinear_sample(array: np.ndarray, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
    h, w = array.shape
    x0 = np.floor(xs).astype(int)
    y0 = np.floor(ys).astype(int)
    x1 = x0 + 1
    y1 = y0 + 1

    valid = (x0 >= 0) & (x1 < w) & (y0 >= 0) & (y1 < h)
    result = np.full(xs.shape, np.nan, dtype=float)
    if not np.any(valid):
        return result

    xv = xs[valid]
    yv = ys[valid]
    x0v = x0[valid]
    x1v = x1[valid]
    y0v = y0[valid]
    y1v = y1[valid]

    q11 = array[y0v, x0v]
    q21 = array[y0v, x1v]
    q12 = array[y1v, x0v]
    q22 = array[y1v, x1v]

    dx = xv - x0v
    dy = yv - y0v

    result[valid] = (
        q11 * (1 - dx) * (1 - dy)
        + q21 * dx * (1 - dy)
        + q12 * (1 - dx) * dy
        + q22 * dx * dy
    )
    return result


def meters_per_degree_lat(lat_deg: float) -> float:
    lat_rad = math.radians(lat_deg)
    return (
        111132.92
        - 559.82 * math.cos(2 * lat_rad)
        + 1.175 * math.cos(4 * lat_rad)
        - 0.0023 * math.cos(6 * lat_rad)
    )


def meters_per_degree_lon(lat_deg: float) -> float:
    lat_rad = math.radians(lat_deg)
    return (
        111412.84 * math.cos(lat_rad)
        - 93.5 * math.cos(3 * lat_rad)
        + 0.118 * math.cos(5 * lat_rad)
    )


def latlon_distance_m(dx_deg: float, dy_deg: float, ref_lat_deg: float) -> float:
    dx_m = dx_deg * meters_per_degree_lon(ref_lat_deg)
    dy_m = dy_deg * meters_per_degree_lat(ref_lat_deg)
    return math.hypot(dx_m, dy_m)


def estimate_pixel_size_meters(
    geotransform: Optional[Sequence[float]], px: float, py: float
) -> float:
    if not geotransform:
        return 30.0

    x0, y0 = pixel_to_world_point(px, py, geotransform)
    x1, y1 = pixel_to_world_point(px + 1.0, py, geotransform)
    x2, y2 = pixel_to_world_point(px, py + 1.0, geotransform)

    lat_mean_x = (y0 + y1) / 2.0
    lat_mean_y = (y0 + y2) / 2.0
    spacing_x = latlon_distance_m(x1 - x0, y1 - y0, lat_mean_x)
    spacing_y = latlon_distance_m(x2 - x0, y2 - y0, lat_mean_y)

    samples = [v for v in (spacing_x, spacing_y) if v > 0]
    if not samples:
        return 30.0
    return sum(samples) / len(samples)


def classify_visibility(values: np.ndarray, origin_height: float, distances: np.ndarray) -> np.ndarray:
    values = np.asarray(values, dtype=float)
    distances = np.asarray(distances, dtype=float)
    visible = np.zeros(values.shape, dtype=bool)
    max_angle = -np.inf
    for idx in range(len(values)):
        if distances[idx] <= 0 or np.isnan(values[idx]):
            continue
        angle = math.atan2(values[idx] - origin_height, distances[idx])
        if angle > max_angle:
            visible[idx] = True
            max_angle = angle
    return visible


def meters_to_degree_tolerance(lat_deg: Optional[float], pixel_size_m: float) -> float:
    if lat_deg is None:
        return POLYGON_SIMPLIFY_TOLERANCE_M / max(pixel_size_m, 1.0)
    lat_scale = meters_per_degree_lat(lat_deg)
    lon_scale = meters_per_degree_lon(lat_deg)
    avg_scale = (lat_scale + lon_scale) / 2.0
    if avg_scale <= 0:
        return POLYGON_SIMPLIFY_TOLERANCE_M / max(pixel_size_m, 1.0)
    return POLYGON_SIMPLIFY_TOLERANCE_M / avg_scale


def create_cell_quad(world_x: np.ndarray, world_y: np.ndarray, i: int, j: int):
    coords = [
        (world_x[i, j], world_y[i, j]),
        (world_x[i + 1, j], world_y[i + 1, j]),
        (world_x[i + 1, j + 1], world_y[i + 1, j + 1]),
        (world_x[i, j + 1], world_y[i, j + 1]),
    ]
    for x, y in coords:
        if np.isnan(x) or np.isnan(y):
            return None
    return coords


def convex_hull(points: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
    if not points:
        return []
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: List[Tuple[float, float]] = []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)

    upper: List[Tuple[float, float]] = []
    for p in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)

    hull = lower[:-1] + upper[:-1]
    if hull and hull[0] != hull[-1]:
        hull.append(hull[0])
    return hull


def fallback_polygons_from_mask(mask: np.ndarray, world_x: np.ndarray, world_y: np.ndarray) -> List[List[Tuple[float, float]]]:
    rows, cols = mask.shape
    visited = np.zeros_like(mask, dtype=bool)
    polygons: List[List[Tuple[float, float]]] = []
    neighbors = [
        (-1, 0),
        (1, 0),
        (0, -1),
        (0, 1),
        (-1, -1),
        (-1, 1),
        (1, -1),
        (1, 1),
    ]

    for i in range(rows):
        for j in range(cols):
            if not mask[i, j] or visited[i, j]:
                continue
            stack = [(i, j)]
            component_cells = []
            while stack:
                ci, cj = stack.pop()
                if ci < 0 or cj < 0 or ci >= rows or cj >= cols:
                    continue
                if visited[ci, cj] or not mask[ci, cj]:
                    continue
                visited[ci, cj] = True
                component_cells.append((ci, cj))
                for di, dj in neighbors:
                    ni, nj = ci + di, cj + dj
                    if 0 <= ni < rows and 0 <= nj < cols and not visited[ni, nj]:
                        stack.append((ni, nj))

            if len(component_cells) < DANGER_MIN_CELLS:
                continue

            centers = []
            for ci, cj in component_cells:
                quad = create_cell_quad(world_x, world_y, ci, cj)
                if not quad:
                    continue
                cx = sum(x for x, _ in quad) / 4.0
                cy = sum(y for _, y in quad) / 4.0
                centers.append((cx, cy))
            hull = convex_hull(centers)
            if len(hull) >= 3:
                polygons.append(hull)

    return polygons


def shapely_polygons_from_mask(
    mask: np.ndarray,
    world_x: np.ndarray,
    world_y: np.ndarray,
    tolerance: float,
) -> List[List[Tuple[float, float]]]:
    if not HAS_SHAPELY:
        return []
    rows, cols = mask.shape
    cell_polys = []
    for i in range(rows):
        for j in range(cols):
            if not mask[i, j]:
                continue
            quad = create_cell_quad(world_x, world_y, i, j)
            if not quad:
                continue
            cell_polys.append(Polygon(quad))
    if not cell_polys:
        return []
    merged = unary_union(cell_polys)
    if tolerance > 0:
        merged = merged.simplify(tolerance, preserve_topology=True)
    if merged.is_empty:
        return []
    geoms = merged.geoms if isinstance(merged, MultiPolygon) else [merged]
    polygons = []
    for geom in geoms:
        if geom.area <= 0:
            continue
        coords = [(float(x), float(y)) for x, y in geom.exterior.coords[:-1]]
        if len(coords) >= 3:
            if coords[0] != coords[-1]:
                coords.append(coords[0])
            polygons.append(coords)
    return polygons


def build_danger_polygons(
    cell_data: np.ndarray,
    world_x: np.ndarray,
    world_y: np.ndarray,
    arc_result: ArcResult,
    geotransform: Optional[Sequence[float]],
) -> List[List[Tuple[float, float]]]:
    mask = np.nan_to_num(cell_data) > 0.5
    if not np.any(mask):
        return []

    lat_ref = arc_result.enemy_world[1] if geotransform else None
    tolerance = meters_to_degree_tolerance(lat_ref, arc_result.pixel_size_m)

    polygons = shapely_polygons_from_mask(mask, world_x, world_y, tolerance)
    if polygons:
        return polygons
    return fallback_polygons_from_mask(mask, world_x, world_y)


def polygon_centroid(coords: Sequence[Tuple[float, float]]) -> Optional[Tuple[float, float]]:
    if not coords:
        return None
    pts = list(coords)
    if len(pts) > 1 and pts[0] == pts[-1]:
        pts = pts[:-1]
    if not pts:
        return None
    if len(pts) == 1:
        return pts[0]
    area = 0.0
    cx = 0.0
    cy = 0.0
    for i in range(len(pts)):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) % len(pts)]
        cross = x0 * y1 - x1 * y0
        area += cross
        cx += (x0 + x1) * cross
        cy += (y0 + y1) * cross
    area *= 0.5
    if abs(area) < 1e-12:
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        return (sum(xs) / len(xs), sum(ys) / len(ys))
    factor = 1.0 / (6.0 * area)
    return (cx * factor, cy * factor)


def world_distance_m(
    point_a: Tuple[float, float],
    point_b: Tuple[float, float],
    geotransform: Optional[Sequence[float]],
) -> float:
    if geotransform:
        ref_lat = (point_a[1] + point_b[1]) / 2.0
        dx = point_a[0] - point_b[0]
        dy = point_a[1] - point_b[1]
        return latlon_distance_m(dx, dy, ref_lat)
    return math.hypot(point_a[0] - point_b[0], point_a[1] - point_b[1])


def compute_cover_disk(
    elevation: np.ndarray,
    geotransform: Optional[Sequence[float]],
    enemy_pixel: Tuple[float, float],
    radius_m: float = ANALYSIS_RADIUS_METERS,
    num_rays: int = NUM_ARC_RAYS,
    step_px: float = ARC_SAMPLE_STEP_PX,
    enemy_height_offset: float = ENEMY_HEIGHT_OFFSET,
) -> ArcResult:
    enemy_px, enemy_py = enemy_pixel
    enemy_elev = bilinear_sample(elevation, np.array([enemy_px]), np.array([enemy_py]))[0]
    if np.isnan(enemy_elev):
        raise ValueError("Enemy point lies outside valid elevation data.")
    origin_height = enemy_elev + enemy_height_offset

    pixel_size_m = estimate_pixel_size_meters(geotransform, enemy_px, enemy_py)
    if pixel_size_m <= 0:
        pixel_size_m = 30.0
    radius_px = radius_m / pixel_size_m
    step_px = max(step_px, 1e-3)
    num_steps = max(int(math.ceil(radius_px / step_px)) + 1, 3)
    distances_px = np.linspace(0.0, radius_px, num_steps)
    distances_m = distances_px * pixel_size_m

    angles = np.linspace(0.0, 2 * math.pi, num_rays, endpoint=False)
    world_x_rows: List[np.ndarray] = []
    world_y_rows: List[np.ndarray] = []
    visible_rows: List[np.ndarray] = []
    valid_rows: List[np.ndarray] = []

    for theta in angles:
        xs = enemy_px + np.cos(theta) * distances_px
        ys = enemy_py + np.sin(theta) * distances_px
        samples = bilinear_sample(elevation, xs, ys)
        valid_mask = ~np.isnan(samples)
        visible_mask = classify_visibility(samples, origin_height, distances_m) & valid_mask
        visible_mask[0] = False

        world_x, world_y = pixel_to_world_arrays(xs, ys, geotransform)
        world_x[~valid_mask] = np.nan
        world_y[~valid_mask] = np.nan

        world_x_rows.append(world_x)
        world_y_rows.append(world_y)
        visible_rows.append(visible_mask)
        valid_rows.append(valid_mask)

    world_x = np.vstack(world_x_rows)
    world_y = np.vstack(world_y_rows)
    visible_mask = np.vstack(visible_rows)
    valid_mask = np.vstack(valid_rows)

    boundary_x = np.append(world_x[:, -1], world_x[0, -1])
    boundary_y = np.append(world_y[:, -1], world_y[0, -1])
    enemy_world = pixel_to_world_point(enemy_px, enemy_py, geotransform)

    return ArcResult(
        world_x=world_x,
        world_y=world_y,
        visible_mask=visible_mask,
        valid_mask=valid_mask,
        boundary=(boundary_x, boundary_y),
        pixel_size_m=pixel_size_m,
        enemy_world=enemy_world,
    )


def compute_cell_data(arc: ArcResult) -> np.ndarray:
    visible_int = arc.visible_mask.astype(int)
    valid = arc.valid_mask

    cell_valid = (
        valid[:-1, :-1]
        & valid[1:, :-1]
        & valid[:-1, 1:]
        & valid[1:, 1:]
    )
    visible_votes = (
        visible_int[:-1, :-1]
        + visible_int[1:, :-1]
        + visible_int[:-1, 1:]
        + visible_int[1:, 1:]
    )
    return np.where(cell_valid, (visible_votes >= 2).astype(float), np.nan)


def sample_elevation_at_world(
    elevation: np.ndarray,
    world_point: Tuple[float, float],
    geotransform: Optional[Sequence[float]],
) -> float:
    if not geotransform:
        return float("nan")
    try:
        px, py = world_to_pixel(world_point[0], world_point[1], geotransform)
    except ValueError:
        return float("nan")
    sample = bilinear_sample(
        elevation,
        np.array([px]),
        np.array([py]),
    )[0]
    if np.isnan(sample):
        return float("nan")
    return float(sample)


def choose_attack_point(
    polygons: Iterable[List[Tuple[float, float]]],
    friendly_world: Tuple[float, float],
    enemy_world: Tuple[float, float],
    geotransform: Optional[Sequence[float]],
) -> Optional[dict]:
    centroid_infos = []
    for poly in polygons:
        centroid = polygon_centroid(poly)
        if centroid is None or not all(math.isfinite(val) for val in centroid):
            continue
        friendly_distance = world_distance_m(centroid, friendly_world, geotransform)
        enemy_distance = world_distance_m(centroid, enemy_world, geotransform)
        centroid_infos.append(
            {
                "centroid": centroid,
                "friendly_distance": friendly_distance,
                "enemy_distance": enemy_distance,
            }
        )
    if not centroid_infos:
        return None
    centroid_infos.sort(key=lambda info: info["friendly_distance"])
    candidate_count = min(ATTACK_CANDIDATE_COUNT, len(centroid_infos))
    candidates = centroid_infos[:candidate_count]
    best = max(candidates, key=lambda info: info["enemy_distance"])
    return best


def ensure_point_inside(
    world_point: Tuple[float, float],
    geotransform: Optional[Sequence[float]],
    elevation: np.ndarray,
) -> Tuple[float, float]:
    px, py = world_to_pixel(world_point[0], world_point[1], geotransform)
    height, width = elevation.shape
    if not (0 <= px < width and 0 <= py < height):
        raise ValueError(
            f"Point {world_point} lies outside the raster extent "
            f"(pixel coords ~ {(px, py)})."
        )
    return px, py


def parse_args():
    parser = argparse.ArgumentParser(
        description="Compute attack recommendation point using LOS polygons.",
    )
    parser.add_argument("--friendly-lat", type=float, required=True, help="Friendly latitude (degrees).")
    parser.add_argument("--friendly-lon", type=float, required=True, help="Friendly longitude (degrees).")
    parser.add_argument("--enemy-lat", type=float, required=True, help="Enemy latitude (degrees).")
    parser.add_argument("--enemy-lon", type=float, required=True, help="Enemy longitude (degrees).")
    parser.add_argument(
        "--raster-path",
        type=str,
        default=None,
        help="GeoTIFF file or directory containing GeoTIFF tiles. Defaults to every *.tif under resource/ (then resources/).",
    )
    parser.add_argument("--candidate-count", type=int, default=ATTACK_CANDIDATE_COUNT, help="How many friendly-nearest polygons to evaluate before choosing the farthest-from-enemy point.")
    parser.add_argument("--radius-m", type=float, default=ANALYSIS_RADIUS_METERS, help="LOS analysis radius in meters.")
    parser.add_argument("--num-rays", type=int, default=NUM_ARC_RAYS, help="Number of radial rays for LOS sampling.")
    parser.add_argument("--output-json", action="store_true", help="Emit machine-readable JSON instead of text.")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.candidate_count <= 0:
        raise SystemExit("candidate-count must be positive.")

    global ATTACK_CANDIDATE_COUNT
    ATTACK_CANDIDATE_COUNT = args.candidate_count

    friendly_world = (args.friendly_lon, args.friendly_lat)
    enemy_world = (args.enemy_lon, args.enemy_lat)

    raster_paths = detect_raster_paths(args.raster_path)
    elevation, geotransform, used_rasters = load_elevation(
        raster_paths,
        enemy_world,
        radius_m=args.radius_m,
    )
    if not used_rasters:
        raise SystemExit("No GeoTIFF tiles overlapped the requested analysis bounds.")
    if geotransform is None:
        raise SystemExit("GeoTIFF mosaic is missing georeferencing (GeoTransform).")

    enemy_px = ensure_point_inside(enemy_world, geotransform, elevation)
    # Friendly point may lie outside the raster; only ensure the enemy (analysis center) is within bounds.
    try:
        friendly_px = world_to_pixel(friendly_world[0], friendly_world[1], geotransform)
    except ValueError:
        friendly_px = None

    arc = compute_cover_disk(
        elevation,
        geotransform,
        enemy_pixel=enemy_px,
        radius_m=args.radius_m,
        num_rays=args.num_rays,
    )

    cell_data = compute_cell_data(arc)
    polygons = build_danger_polygons(
        cell_data,
        arc.world_x,
        arc.world_y,
        arc,
        geotransform,
    )
    if not polygons:
        raise SystemExit("No danger/cover polygons detected; cannot provide recommendation.")

    best = choose_attack_point(polygons, friendly_world, enemy_world, geotransform)
    if not best:
        raise SystemExit("Failed to derive a centroid-based recommendation.")

    best_point = best["centroid"]
    altitude = sample_elevation_at_world(elevation, best_point, geotransform)
    altitude_int = None
    if math.isfinite(altitude):
        try:
            altitude_int = int(round(altitude))
        except (ValueError, OverflowError):
            altitude_int = None
    raster_sources_abs = [os.path.abspath(path) for path in used_rasters]
    result = {
        "attack_point": {
            "lon": best_point[0],
            "lat": best_point[1],
            "alt_m": altitude_int,
        },
        "friendly_point": {"lon": friendly_world[0], "lat": friendly_world[1]},
        "enemy_point": {"lon": enemy_world[0], "lat": enemy_world[1]},
        "distance_friendly_m": best["friendly_distance"],
        "distance_enemy_m": best["enemy_distance"],
        "raster_path": raster_sources_abs[0],
        "raster_sources": raster_sources_abs,
    }

    if args.output_json:
        print(json.dumps(result, indent=2, ensure_ascii=False))
    else:
        alt_text = f"{altitude_int} m" if altitude_int is not None else "unknown"
        raster_label = ", ".join(os.path.basename(path) for path in raster_sources_abs)
        print(f"Raster(s)        : {raster_label}")
        print(f"Friendly (lat,lon): ({friendly_world[1]:.6f}, {friendly_world[0]:.6f})")
        print(f"Enemy    (lat,lon): ({enemy_world[1]:.6f}, {enemy_world[0]:.6f})")
        print(
            f"Attack point (lat,lon,alt): "
            f"({best_point[1]:.6f}, {best_point[0]:.6f}, {alt_text})"
        )
        print(f"Friendly distance : {best['friendly_distance']:.1f} m")
        print(f"Enemy distance    : {best['enemy_distance']:.1f} m")


if __name__ == "__main__":
    main()
