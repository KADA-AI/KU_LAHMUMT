"""Canonical native-DEM point-to-point LOS API.

The cover planner uses vectorised private kernels for candidate sweeps.  SIM
needs the same result for a small number of moving pairs.  This module exposes
that native kernel without making SIM reproduce CRS, PixelIsArea, curvature,
clearance, or nodata rules independently.
"""

from __future__ import annotations

from functools import lru_cache
import math
from pathlib import Path
from typing import Any

import numpy as np

from modules.common.regional_dem import select_regional_dem
from modules.common.terrain_los import TERRAIN_LOS_POLICY_VERSION

from .config import CoverConfig
from .dem_native import NativeDemRaster
from .visibility import _los_visible_batch


def _fractional_cell(dem: NativeDemRaster, x: float, y: float) -> tuple[float, float]:
    row_edge, col_edge = dem.native_to_rowcol(float(x), float(y))
    return float(row_edge) - 0.5, float(col_edge) - 0.5


@lru_cache(maxsize=4)
def _load_native_dem(
    path_text: str,
    modified_ns: int,
    size_bytes: int,
) -> NativeDemRaster:
    del modified_ns, size_bytes
    return NativeDemRaster(Path(path_text))


def _dem_for_pair(
    resource_dir: str | Path,
    observer_latitude: float,
    observer_longitude: float,
    target_latitude: float,
    target_longitude: float,
) -> tuple[NativeDemRaster | None, Path | None, str | None]:
    observer_spec = select_regional_dem(observer_latitude, observer_longitude)
    target_spec = select_regional_dem(target_latitude, target_longitude)
    if observer_spec is None or target_spec is None:
        return None, None, "OUTSIDE_REGISTERED_DEM"
    if observer_spec.filename.lower() != target_spec.filename.lower():
        return None, None, "CROSS_DEM_RAY_UNSUPPORTED"
    path = (Path(resource_dir) / observer_spec.filename).resolve()
    if not path.is_file():
        return None, path, "DEM_FILE_MISSING"
    try:
        stat = path.stat()
        dem = _load_native_dem(str(path), int(stat.st_mtime_ns), int(stat.st_size))
    except Exception as exc:  # fail closed at deployment/runtime boundary
        return None, path, f"DEM_LOAD_FAILED:{type(exc).__name__}"
    return dem, path, None


def clear_regional_los_cache() -> None:
    _load_native_dem.cache_clear()


def evaluate_regional_los(
    *,
    resource_dir: str | Path,
    observer_latitude: float,
    observer_longitude: float,
    observer_altitude_m: float,
    target_latitude: float,
    target_longitude: float,
    target_altitude_m: float,
    observer_height_m: float = 0.0,
    target_height_m: float = 0.0,
    max_range_m: float | None = None,
    reject_nodata: bool = False,
    config: CoverConfig | None = None,
) -> dict[str, Any]:
    """Evaluate one LOS ray with the planner's native DEM kernel.

    ``observer_height_m``/``target_height_m`` raise a ground entity to its
    physical observation point while preserving an already-higher MSL input.
    ``visible=None`` is reserved for unavailable/unknown terrain.  With
    ``reject_nodata=False`` (enemy exposure) nodata never manufactures cover;
    with ``True`` (communication certification) the ray fails unknown.

    ``evaluated`` says whether a ray was actually traced.  Two short-circuits -
    ``OUT_OF_RANGE`` and ``ENDPOINT_NOT_ABOVE_TERRAIN`` - return
    ``visible=False`` without sampling any terrain, which is indistinguishable
    from ``TERRAIN_BLOCKED`` to a caller that only reads the boolean.  Anything
    certifying *concealment* must require ``visible is False and evaluated is
    True``; "no ray was traced" is not proof of cover.
    """

    coordinates = (
        observer_latitude,
        observer_longitude,
        observer_altitude_m,
        target_latitude,
        target_longitude,
        target_altitude_m,
    )
    if not all(math.isfinite(float(value)) for value in coordinates):
        return {
            "visible": None,
            "demAvailable": False,
            "demSources": [],
            "reason": "NONFINITE_ENDPOINT",
            "evaluated": False,
            "policyVersion": TERRAIN_LOS_POLICY_VERSION,
        }

    dem, path, error = _dem_for_pair(
        resource_dir,
        float(observer_latitude),
        float(observer_longitude),
        float(target_latitude),
        float(target_longitude),
    )
    if dem is None:
        return {
            "visible": None,
            "demAvailable": False,
            "demSources": [path.name] if path is not None else [],
            "demPath": str(path) if path is not None else None,
            "reason": error or "DEM_UNAVAILABLE",
            "evaluated": False,
            "policyVersion": TERRAIN_LOS_POLICY_VERSION,
        }

    observer_x, observer_y = dem.latlon_to_native(
        float(observer_latitude), float(observer_longitude)
    )
    target_x, target_y = dem.latlon_to_native(
        float(target_latitude), float(target_longitude)
    )
    if not dem.contains_native(observer_x, observer_y) or not dem.contains_native(
        target_x, target_y
    ):
        return {
            "visible": None,
            "demAvailable": False,
            "demSources": [path.name],
            "demPath": str(path),
            "reason": "ENDPOINT_OUTSIDE_DEM_RASTER",
            "evaluated": False,
            "policyVersion": TERRAIN_LOS_POLICY_VERSION,
        }

    observer_ground_m = float(dem.sample_native(observer_x, observer_y))
    target_ground_m = float(dem.sample_native(target_x, target_y))
    if not math.isfinite(observer_ground_m) or not math.isfinite(target_ground_m):
        return {
            "visible": None,
            "demAvailable": False,
            "demSources": [path.name],
            "demPath": str(path),
            "reason": "ENDPOINT_NODATA",
            "evaluated": False,
            "policyVersion": TERRAIN_LOS_POLICY_VERSION,
        }

    observer_ray_altitude_m = max(
        float(observer_altitude_m),
        observer_ground_m + max(0.0, float(observer_height_m)),
    )
    target_ray_altitude_m = max(
        float(target_altitude_m),
        target_ground_m + max(0.0, float(target_height_m)),
    )
    horizontal_distance_m = float(
        math.hypot(target_x - observer_x, target_y - observer_y)
    )
    if max_range_m is not None and horizontal_distance_m > max(
        0.0, float(max_range_m)
    ):
        return {
            "visible": False,
            "demAvailable": True,
            "demSources": [path.name],
            "demPath": str(path),
            "reason": "OUT_OF_RANGE",
            "evaluated": False,
            "horizontalDistanceM": horizontal_distance_m,
            "observerGroundM": observer_ground_m,
            "targetGroundM": target_ground_m,
            "observerRayAlt": observer_ray_altitude_m,
            "targetRayAlt": target_ray_altitude_m,
            "policyVersion": TERRAIN_LOS_POLICY_VERSION,
        }

    # A live aircraft below native terrain must never be certified as visible.
    if observer_ray_altitude_m <= observer_ground_m or target_ray_altitude_m <= target_ground_m:
        return {
            "visible": False,
            "demAvailable": True,
            "demSources": [path.name],
            "demPath": str(path),
            "reason": "ENDPOINT_NOT_ABOVE_TERRAIN",
            "evaluated": False,
            "horizontalDistanceM": horizontal_distance_m,
            "observerGroundM": observer_ground_m,
            "targetGroundM": target_ground_m,
            "observerRayAlt": observer_ray_altitude_m,
            "targetRayAlt": target_ray_altitude_m,
            "policyVersion": TERRAIN_LOS_POLICY_VERSION,
        }

    observer_row, observer_col = _fractional_cell(dem, observer_x, observer_y)
    target_row, target_col = _fractional_cell(dem, target_x, target_y)
    base_config = config or CoverConfig(dem_path=str(path))
    span = abs(target_row - observer_row) + abs(target_col - observer_col)
    full_ray_steps = int(span * float(base_config.los_samples_per_cell)) + 2
    precise_config = base_config.with_overrides(
        dem_path=str(path),
        los_max_steps=max(int(base_config.los_max_steps), full_ray_steps),
    )

    target_rows = np.asarray([target_row], dtype=np.float64)
    target_cols = np.asarray([target_col], dtype=np.float64)
    visible = bool(
        _los_visible_batch(
            dem.block_elev,
            float(observer_row),
            float(observer_col),
            float(observer_ray_altitude_m),
            target_rows,
            target_cols,
            np.asarray([target_ray_altitude_m], dtype=np.float64),
            np.asarray([horizontal_distance_m], dtype=np.float64),
            float(dem.cell_m),
            precise_config,
        )[0]
    )

    if bool(reject_nodata):
        steps = int(
            np.clip(
                int(span * float(precise_config.los_samples_per_cell)) + 1,
                2,
                int(precise_config.los_max_steps),
            )
        )
        if steps > 2:
            fractions = np.linspace(0.0, 1.0, steps, dtype=np.float64)
            rows = observer_row + fractions * (target_row - observer_row)
            cols = observer_col + fractions * (target_col - observer_col)
            row_indices = np.clip(
                np.rint(rows).astype(np.int32), 0, int(dem.height) - 1
            )
            col_indices = np.clip(
                np.rint(cols).astype(np.int32), 0, int(dem.width) - 1
            )
            interior = dem.block_elev[row_indices[1:-1], col_indices[1:-1]]
            if not bool(np.all(np.isfinite(interior) & (interior >= -1000.0))):
                return {
                    "visible": None,
                    "demAvailable": False,
                    "demSources": [path.name],
                    "demPath": str(path),
                    "reason": "RAY_CROSSES_NODATA",
            "evaluated": False,
                    "horizontalDistanceM": horizontal_distance_m,
                    "observerGroundM": observer_ground_m,
                    "targetGroundM": target_ground_m,
                    "observerRayAlt": observer_ray_altitude_m,
                    "targetRayAlt": target_ray_altitude_m,
                    "policyVersion": TERRAIN_LOS_POLICY_VERSION,
                }

    return {
        "visible": visible,
        "demAvailable": True,
        "demSources": [path.name],
        "demPath": str(path),
        "reason": "VISIBLE" if visible else "TERRAIN_BLOCKED",
        "evaluated": True,
        "horizontalDistanceM": horizontal_distance_m,
        "observerGroundM": observer_ground_m,
        "targetGroundM": target_ground_m,
        "observerRayAlt": observer_ray_altitude_m,
        "targetRayAlt": target_ray_altitude_m,
        "policyVersion": TERRAIN_LOS_POLICY_VERSION,
    }


__all__ = ["clear_regional_los_cache", "evaluate_regional_los"]
