from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import threading
from typing import Optional

try:
    import numpy as np
    import rasterio
    from rasterio.enums import Resampling
    from rasterio.io import MemoryFile
    from rasterio.transform import from_bounds
    from rasterio.vrt import WarpedVRT
    from rasterio.windows import Window, bounds as window_bounds_fn, from_bounds as window_from_bounds
except Exception:  # pragma: no cover - optional dependency
    rasterio = None


WEB_MERCATOR_HALF = 20037508.342789244


def _tile_bounds_mercator(z: int, x: int, y: int) -> tuple[float, float, float, float]:
    n = 1 << z
    tile_span = (2 * WEB_MERCATOR_HALF) / n
    west = -WEB_MERCATOR_HALF + (x * tile_span)
    east = west + tile_span
    north = WEB_MERCATOR_HALF - (y * tile_span)
    south = north - tile_span
    return (west, south, east, north)


def _encode_terrarium(elevation: "np.ndarray") -> "np.ndarray":
    value = np.clip(elevation + 32768.0, 0, 65535.996)
    r = np.floor(value / 256.0)
    g = np.floor(value - (r * 256.0))
    b = np.floor((value - np.floor(value)) * 256.0)
    rgb = np.stack([r, g, b]).astype("uint8")
    return rgb


@dataclass
class DemTileProvider:
    dem_dir: Path
    tile_size: int = 256
    max_zoom: int = 12

    def __post_init__(self) -> None:
        self._lock = threading.Lock()
        self._datasets = []
        self._vrts = []
        if rasterio is None:
            return
        for tif in sorted(self.dem_dir.glob("*.tif")):
            try:
                dataset = rasterio.open(tif)
            except Exception:
                continue
            vrt = WarpedVRT(dataset, crs="EPSG:3857", resampling=Resampling.bilinear)
            self._datasets.append(dataset)
            self._vrts.append(vrt)

    @property
    def available(self) -> bool:
        return bool(self._vrts)

    def get_tile(self, z: int, x: int, y: int) -> Optional[bytes]:
        if not self.available or z > self.max_zoom:
            return None
        bounds = _tile_bounds_mercator(z, x, y)
        candidates = [vrt for vrt in self._vrts if _bounds_intersect(bounds, vrt.bounds)]
        if not candidates:
            return None
        output = np.full((self.tile_size, self.tile_size), np.nan, dtype="float32")
        with self._lock:
            for vrt in candidates:
                intersection = _bounds_intersection(bounds, vrt.bounds)
                if intersection is None:
                    continue
                window = window_from_bounds(*intersection, transform=vrt.transform)
                row_off = int(math.floor(window.row_off))
                col_off = int(math.floor(window.col_off))
                row_end = int(math.ceil(window.row_off + window.height))
                col_end = int(math.ceil(window.col_off + window.width))
                row_off = max(0, row_off)
                col_off = max(0, col_off)
                row_end = min(vrt.height, row_end)
                col_end = min(vrt.width, col_end)
                if row_end <= row_off or col_end <= col_off:
                    continue
                window = Window(col_off, row_off, col_end - col_off, row_end - row_off)
                window_bounds = window_bounds_fn(window, transform=vrt.transform)
                row_start, row_end, col_start, col_end = _tile_window(
                    bounds, window_bounds, self.tile_size
                )
                if row_end <= row_start or col_end <= col_start:
                    continue
                data = vrt.read(
                    1,
                    window=window,
                    out_shape=(row_end - row_start, col_end - col_start),
                    resampling=Resampling.bilinear,
                    masked=True,
                )
                if isinstance(data, np.ma.MaskedArray):
                    values = data.data.astype("float32")
                    valid = ~data.mask & np.isfinite(values)
                else:
                    values = data.astype("float32")
                    valid = np.isfinite(values)
                tile_slice = (slice(row_start, row_end), slice(col_start, col_end))
                chunk = output[tile_slice]
                output[tile_slice] = np.where(np.isnan(chunk) & valid, values, chunk)
        elevation = np.where(np.isfinite(output), output, 0.0)
        rgb = _encode_terrarium(elevation)
        transform = from_bounds(*bounds, self.tile_size, self.tile_size)
        with MemoryFile() as memfile:
            with memfile.open(
                driver="PNG",
                width=self.tile_size,
                height=self.tile_size,
                count=3,
                dtype="uint8",
                crs="EPSG:3857",
                transform=transform,
            ) as dataset:
                dataset.write(rgb)
            return memfile.read()


def _bounds_intersect(
    a: tuple[float, float, float, float], b: tuple[float, float, float, float]
) -> bool:
    return not (a[2] <= b[0] or a[0] >= b[2] or a[3] <= b[1] or a[1] >= b[3])


def _bounds_intersection(
    a: tuple[float, float, float, float], b: tuple[float, float, float, float]
) -> Optional[tuple[float, float, float, float]]:
    west = max(a[0], b[0])
    south = max(a[1], b[1])
    east = min(a[2], b[2])
    north = min(a[3], b[3])
    if west >= east or south >= north:
        return None
    return (west, south, east, north)


def _tile_window(
    tile_bounds: tuple[float, float, float, float],
    intersection: tuple[float, float, float, float],
    tile_size: int,
) -> tuple[int, int, int, int]:
    west, south, east, north = tile_bounds
    i_west, i_south, i_east, i_north = intersection
    width = east - west
    height = north - south
    if width <= 0 or height <= 0:
        return (0, 0, 0, 0)
    col_start = int(math.floor((i_west - west) / width * tile_size))
    col_end = int(math.ceil((i_east - west) / width * tile_size))
    row_start = int(math.floor((north - i_north) / height * tile_size))
    row_end = int(math.ceil((north - i_south) / height * tile_size))
    col_start = max(0, min(tile_size, col_start))
    col_end = max(0, min(tile_size, col_end))
    row_start = max(0, min(tile_size, row_start))
    row_end = max(0, min(tile_size, row_end))
    return (row_start, row_end, col_start, col_end)


def load_dem_provider(dem_dir: Path, tile_size: int = 256, max_zoom: int = 12) -> DemTileProvider:
    return DemTileProvider(dem_dir=dem_dir, tile_size=tile_size, max_zoom=max_zoom)
