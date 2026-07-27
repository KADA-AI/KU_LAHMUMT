from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
from pathlib import Path
from threading import Lock
from typing import Optional, Tuple

from modules.common.regional_dem import regional_dem_paths

try:
    import rasterio
    from rasterio.warp import transform, transform_bounds
except Exception as exc:  # pragma: no cover - handled by runtime guard
    rasterio = None
    transform = None
    transform_bounds = None
    _IMPORT_ERROR = exc
else:
    _IMPORT_ERROR = None

WGS84 = "EPSG:4326"
_CACHE_MISS = object()
_CELL_CACHE_MAX = 200_000


def _require_rasterio() -> None:
    if rasterio is None:
        raise RuntimeError(f"rasterio not available: {_IMPORT_ERROR}")


@dataclass
class _DemTile:
    path: Path
    dataset: "rasterio.DatasetReader"
    bounds_wgs84: Tuple[float, float, float, float]


class DEMSampler:
    def __init__(self, dem_dir: Path) -> None:
        _require_rasterio()
        self._tiles: list[_DemTile] = []
        self._cell_cache: OrderedDict[tuple[str, int, int], Optional[float]] = OrderedDict()
        self._cache_lock = Lock()
        for path in regional_dem_paths(dem_dir):
            ds = rasterio.open(path)
            bounds = transform_bounds(
                ds.crs,
                WGS84,
                ds.bounds.left,
                ds.bounds.bottom,
                ds.bounds.right,
                ds.bounds.top,
                densify_pts=21,
            )
            self._tiles.append(_DemTile(path=path, dataset=ds, bounds_wgs84=bounds))

    def _cache_get(self, key: tuple[str, int, int]) -> object:
        with self._cache_lock:
            if key not in self._cell_cache:
                return _CACHE_MISS
            value = self._cell_cache.pop(key)
            self._cell_cache[key] = value
            return value

    def _cache_set(self, key: tuple[str, int, int], value: Optional[float]) -> None:
        with self._cache_lock:
            if key in self._cell_cache:
                self._cell_cache.pop(key)
            self._cell_cache[key] = value
            while len(self._cell_cache) > _CELL_CACHE_MAX:
                self._cell_cache.popitem(last=False)

    def sample(self, lat: float, lon: float) -> Optional[float]:
        _require_rasterio()
        for tile in self._tiles:
            west, south, east, north = tile.bounds_wgs84
            if not (west <= lon <= east and south <= lat <= north):
                continue
            xs, ys = transform(WGS84, tile.dataset.crs, [lon], [lat])
            x, y = xs[0], ys[0]
            b = tile.dataset.bounds
            if not (b.left <= x <= b.right and b.bottom <= y <= b.top):
                continue
            try:
                row, col = tile.dataset.index(x, y)
            except Exception:
                continue
            if row < 0 or col < 0 or row >= tile.dataset.height or col >= tile.dataset.width:
                continue
            cache_key = (str(tile.path), int(row), int(col))
            cached = self._cache_get(cache_key)
            if cached is not _CACHE_MISS:
                return cached
            value = next(tile.dataset.sample([(x, y)]))[0]
            if hasattr(value, "mask") and getattr(value, "mask"):
                self._cache_set(cache_key, None)
                return None
            if tile.dataset.nodata is not None and float(value) == float(tile.dataset.nodata):
                self._cache_set(cache_key, None)
                return None
            parsed = float(value)
            self._cache_set(cache_key, parsed)
            return parsed
        return None


_SAMPLER: Optional[DEMSampler] = None
_SAMPLER_LOCK = Lock()


def _dem_dir() -> Path:
    return Path(__file__).resolve().parents[4] / "resource"


def _get_sampler() -> DEMSampler:
    global _SAMPLER
    if _SAMPLER is None:
        with _SAMPLER_LOCK:
            if _SAMPLER is None:
                dem_dir = _dem_dir()
                if not dem_dir.exists():
                    raise RuntimeError(f"DEM directory not found: {dem_dir}")
                _SAMPLER = DEMSampler(dem_dir)
    return _SAMPLER


def ground_elevation_m(lat: float, lon: float) -> Optional[float]:
    return _get_sampler().sample(lat, lon)


def altitude_agl_m(lat: float, lon: float, offset_m: float) -> int:
    elevation = ground_elevation_m(lat, lon)
    if elevation is None:
        raise RuntimeError(f"DEM coverage missing for lat={lat}, lon={lon}")
    return int(round(elevation + offset_m))
