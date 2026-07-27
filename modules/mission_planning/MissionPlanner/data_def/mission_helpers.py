"""
mission_helpers.py
공통 유틸리티 · 지도-JS 브릿지 · 간단한 ‘임무 메타’ 다이얼로그
"""

import random, folium, json
import math
import os
import re
import sys
import threading
import time
from collections import namedtuple
from datetime import datetime, timezone
from functools import lru_cache
from pathlib import Path
from typing import Any, Dict, Iterable, Tuple

import numpy as np
from PIL import Image
from affine import Affine
try:
    from pyproj import Transformer
except Exception:
    Transformer = None
try:
    import tifffile
except Exception:
    tifffile = None

from branca.colormap import linear
from PyQt5.QtCore import QObject, pyqtSignal, pyqtSlot
from PyQt5.QtWidgets import (QDialog, QGridLayout, QLabel, QComboBox,
                             QDialogButtonBox, QDoubleSpinBox)
from folium import CircleMarker   # folium 원형 마커 import

from modules.common.regional_dem import (
    REGIONAL_DEM_EPSG_BY_NAME,
    REGIONAL_DEM_FILENAMES,
    REGIONAL_DEM_SPECS,
    regional_dem_inventory,
    select_regional_dem,
)

_CANONICAL_NAME = "modules.mission_planning.MissionPlanner.data_def.mission_helpers"
if __name__ == "data_def.mission_helpers":
    sys.modules.setdefault(_CANONICAL_NAME, sys.modules[__name__])
elif __name__ == _CANONICAL_NAME:
    sys.modules.setdefault("data_def.mission_helpers", sys.modules[__name__])

try:
    from .id_allocator import reserve_individual_mission_ids
except Exception:
    from data_def.id_allocator import reserve_individual_mission_ids  # type: ignore


_PROJECT_ROOT = Path(__file__).resolve().parents[4]
_DEM_DIR = _PROJECT_ROOT / "resource"
_DEM_TILE_RE = re.compile(r"([ns])(\d+)_([ew])(\d+)", re.IGNORECASE)
_REGIONAL_DEM_EPSG_BY_NAME = dict(REGIONAL_DEM_EPSG_BY_NAME)
_REGIONAL_DEM_PRIORITY = tuple(REGIONAL_DEM_FILENAMES)
_REGIONAL_DEM_BOUNDS_BY_NAME = {
    spec.filename.lower(): spec.latlon_bounds for spec in REGIONAL_DEM_SPECS
}
_DEM_USAGE_LOCK = threading.Lock()
_DEM_USAGE_LOGGED_KEYS = set()
_DEM_MISS_LOGGED_KEYS = set()
_DEM_PIXEL_CACHE_MAX = 262144
_DEM_PIXEL_CACHE_LOCK = threading.Lock()
_DEM_PIXEL_CACHE: Dict[Tuple[str, int, int], float] = {}
_DEM_CACHE_SIGNATURE_LOCK = threading.Lock()
_DEM_CACHE_SIGNATURE_CHECK_INTERVAL_SEC = 1.0
_DEM_CACHE_SIGNATURE_LAST_CHECKED = 0.0
_DEM_LOAD_INFLIGHT_LOCK = threading.Lock()
_DEM_LOAD_INFLIGHT: Dict[str, Tuple[threading.Event, Dict[str, Any]]] = {}
_TERRAIN_MANY_METRICS_LOCAL = threading.local()
BoundingBox = namedtuple("BoundingBox", "left bottom right top")

_MODEL_PIXEL_SCALE_TAG = 33550
_MODEL_TIEPOINT_TAG = 33922
_GDAL_NODATA_TAG = 42113


def reset_terrain_elev_many_metrics() -> None:
    _TERRAIN_MANY_METRICS_LOCAL.metrics = {
        "demTileResolveMs": 0.0,
        "demTileCandidateIndexMs": 0.0,
        "demTileCandidateAssignMs": 0.0,
        "demTileApplyMs": 0.0,
        "demTileFallbackScanMs": 0.0,
        "demTileLoadMs": 0.0,
        "demTileLoadWaitMs": 0.0,
        "demNativeTransformMs": 0.0,
        "demRowColTransformMs": 0.0,
        "demPixelReadMs": 0.0,
        "demCacheReadMs": 0.0,
        "demCacheWriteMs": 0.0,
        "demTileCount": 0,
        "demTileCandidateCount": 0,
        "demTileFallbackCandidateCount": 0,
        "demTileApplyCallCount": 0,
        "demTileLoadLeaderCount": 0,
        "demTileLoadWaiterCount": 0,
        "demTileLoadTimeoutCount": 0,
        "demResolvedByTile": 0,
        "demPixelCacheHitCount": 0,
        "demPixelCacheMissCount": 0,
        "demPixelCacheUniqueMissCount": 0,
        "demPixelCacheClearCount": 0,
    }


def get_terrain_elev_many_metrics(*, reset: bool = False) -> Dict[str, Any]:
    metrics = getattr(_TERRAIN_MANY_METRICS_LOCAL, "metrics", None)
    if not isinstance(metrics, dict):
        reset_terrain_elev_many_metrics()
        metrics = getattr(_TERRAIN_MANY_METRICS_LOCAL, "metrics", {})
    out = dict(metrics)
    for key in (
        "demTileResolveMs",
        "demTileCandidateIndexMs",
        "demTileCandidateAssignMs",
        "demTileApplyMs",
        "demTileFallbackScanMs",
        "demTileLoadMs",
        "demTileLoadWaitMs",
        "demNativeTransformMs",
        "demRowColTransformMs",
        "demPixelReadMs",
        "demCacheReadMs",
        "demCacheWriteMs",
    ):
        out[key] = round(float(out.get(key) or 0.0), 3)
    if reset:
        reset_terrain_elev_many_metrics()
    return out


def _record_terrain_elev_many_metrics(**values: float | int) -> None:
    metrics = getattr(_TERRAIN_MANY_METRICS_LOCAL, "metrics", None)
    if not isinstance(metrics, dict):
        reset_terrain_elev_many_metrics()
        metrics = getattr(_TERRAIN_MANY_METRICS_LOCAL, "metrics", {})
    for key, value in values.items():
        if key == "demTileCount":
            metrics[key] = max(int(metrics.get(key, 0) or 0), int(value or 0))
        elif key in {
            "demTileResolveMs",
            "demTileCandidateIndexMs",
            "demTileCandidateAssignMs",
            "demTileApplyMs",
            "demTileFallbackScanMs",
            "demTileLoadMs",
            "demTileLoadWaitMs",
            "demNativeTransformMs",
            "demRowColTransformMs",
            "demPixelReadMs",
            "demCacheReadMs",
            "demCacheWriteMs",
        }:
            metrics[key] = float(metrics.get(key, 0.0) or 0.0) + float(value or 0.0)
        else:
            metrics[key] = int(metrics.get(key, 0) or 0) + int(value or 0)


def _scan_dem_tiles() -> Tuple[Tuple[Path, Tuple[float, float, float, float]], ...]:
    """
    resource/ 밑의 GeoTIFF 타일 목록을 (Path, (lat0, lat1, lon0, lon1)) 형태로 돌려준다.
    파일명이 n37_e127_* 같이 규칙을 따라야 범위 계산 가능.
    """
    if not _DEM_DIR.exists():
        raise FileNotFoundError(f"DEM 디렉터리를 찾을 수 없습니다: {_DEM_DIR}")

    tiles = []
    for name in _REGIONAL_DEM_PRIORITY:
        tif = _DEM_DIR / name
        if not tif.exists():
            continue
        bounds_llh = _REGIONAL_DEM_BOUNDS_BY_NAME.get(tif.name.lower())
        if bounds_llh is not None:
            tiles.append((tif, bounds_llh))

    if not tiles:
        expected = ", ".join(_REGIONAL_DEM_PRIORITY)
        raise FileNotFoundError(
            f"Operational DEM files were not found under {_DEM_DIR}: {expected}"
        )
    return tuple(tiles)


@lru_cache(maxsize=1)
def _available_dem_tiles():
    return _scan_dem_tiles()


def _terrain_data_signature_uncached() -> Tuple[Tuple[str, int, int], ...]:
    rows: list[Tuple[str, int, int]] = []
    seen: set[str] = set()
    candidates: list[Path] = []
    for name in _REGIONAL_DEM_PRIORITY:
        candidates.append(_DEM_DIR / name)
    for path in candidates:
        try:
            resolved = str(Path(path).resolve())
        except Exception:
            resolved = str(path)
        if resolved in seen:
            continue
        seen.add(resolved)
        try:
            path_obj = Path(path)
            if not path_obj.exists():
                continue
            stat = path_obj.stat()
            rows.append((resolved, int(stat.st_mtime_ns), int(stat.st_size)))
        except Exception:
            rows.append((resolved, 0, 0))
    return tuple(rows)


@lru_cache(maxsize=1)
def terrain_data_signature() -> Tuple[Tuple[str, int, int], ...]:
    return _terrain_data_signature_uncached()


def ensure_terrain_cache_current(*, force: bool = False) -> Tuple[Tuple[str, int, int], ...]:
    """Invalidate DEM caches if the backing GeoTIFF set changed on disk."""
    global _DEM_CACHE_SIGNATURE_LAST_CHECKED
    now = time.monotonic()
    with _DEM_CACHE_SIGNATURE_LOCK:
        if (
            not force
            and _DEM_CACHE_SIGNATURE_LAST_CHECKED > 0.0
            and now - _DEM_CACHE_SIGNATURE_LAST_CHECKED < _DEM_CACHE_SIGNATURE_CHECK_INTERVAL_SEC
        ):
            return terrain_data_signature()
        _DEM_CACHE_SIGNATURE_LAST_CHECKED = now
    fresh_signature = _terrain_data_signature_uncached()
    cached_signature = terrain_data_signature()
    if fresh_signature != cached_signature:
        for cache_func in (_terrain_elev_cached, _load_dem_data, _available_dem_tiles, terrain_data_signature):
            cache_clear = getattr(cache_func, "cache_clear", None)
            if callable(cache_clear):
                try:
                    cache_clear()
                except Exception:
                    pass
        with _DEM_PIXEL_CACHE_LOCK:
            _DEM_PIXEL_CACHE.clear()
        with _DEM_USAGE_LOCK:
            _DEM_USAGE_LOGGED_KEYS.clear()
            _DEM_MISS_LOGGED_KEYS.clear()
        return terrain_data_signature()
    return cached_signature


def _transform_from_tags(scale_tag, tiepoint_tag) -> Affine:
    if scale_tag is None or tiepoint_tag is None:
        raise ValueError("GeoTIFF metadata is missing ModelPixelScale or ModelTiepoint tags.")
    if len(scale_tag) < 2 or len(tiepoint_tag) < 6:
        raise ValueError("Incomplete GeoTIFF tags for affine transform.")

    sx = float(scale_tag[0])
    sy = float(scale_tag[1])
    px = float(tiepoint_tag[0])
    py = float(tiepoint_tag[1])
    mx = float(tiepoint_tag[3])
    my = float(tiepoint_tag[4])

    # The bundled rasters are PixelIsArea GeoTIFFs. Their model tiepoint maps
    # the raster's upper-left grid corner directly; applying an extra half-cell
    # shift samples the neighbouring 10/30 m cell on steep terrain.
    origin_x = mx - sx * px
    origin_y = my + sy * py
    return Affine(sx, 0.0, origin_x, 0.0, -sy, origin_y)


def _bounds_from_transform(shape: Tuple[int, int], transform: Affine) -> BoundingBox:
    height, width = shape
    corners = (
        transform * (0, 0),
        transform * (0, height),
        transform * (width, 0),
        transform * (width, height),
    )
    xs = [pt[0] for pt in corners]
    ys = [pt[1] for pt in corners]
    return BoundingBox(min(xs), min(ys), max(xs), max(ys))


def _load_dem_metadata(path: Path) -> Tuple[Affine, BoundingBox, float | None]:
    """Read GeoTIFF georeferencing metadata without materializing the raster band."""
    if not path.exists():
        raise FileNotFoundError(f"DEM 파일을 찾을 수 없습니다: {path}")

    try:
        with Image.open(path) as img:
            width, height = img.size
            scale_tag = img.tag_v2.get(_MODEL_PIXEL_SCALE_TAG)
            tiepoint_tag = img.tag_v2.get(_MODEL_TIEPOINT_TAG)
            transform = _transform_from_tags(scale_tag, tiepoint_tag)
            bounds = _bounds_from_transform((int(height), int(width)), transform)
            nodata_tag = img.tag_v2.get(_GDAL_NODATA_TAG)
            try:
                nodata = float(nodata_tag) if nodata_tag is not None else None
            except (TypeError, ValueError):
                nodata = None
            return transform, bounds, nodata
    except Exception:
        if tifffile is None:
            raise
        with tifffile.TiffFile(str(path)) as tif:
            page = tif.pages[0]
            shape = page.shape
            scale_tag = page.tags.get(_MODEL_PIXEL_SCALE_TAG)
            tiepoint_tag = page.tags.get(_MODEL_TIEPOINT_TAG)
            scale_value = scale_tag.value if scale_tag is not None else None
            tiepoint_value = tiepoint_tag.value if tiepoint_tag is not None else None
            transform = _transform_from_tags(scale_value, tiepoint_value)
            bounds = _bounds_from_transform((int(shape[0]), int(shape[1])), transform)
            nodata_tag = page.tags.get(_GDAL_NODATA_TAG)
            nodata_value = nodata_tag.value if nodata_tag is not None else None
            try:
                nodata = float(nodata_value) if nodata_value is not None else None
            except (TypeError, ValueError):
                nodata = None
            return transform, bounds, nodata


def _regional_dem_epsg(path: Path) -> int | None:
    try:
        return _REGIONAL_DEM_EPSG_BY_NAME.get(path.name.lower())
    except Exception:
        return None


@lru_cache(maxsize=None)
def _transformer(src_epsg: int, dst_epsg: int):
    if Transformer is None:
        return None
    try:
        return Transformer.from_crs(f"EPSG:{int(src_epsg)}", f"EPSG:{int(dst_epsg)}", always_xy=True)
    except Exception:
        return None


def _native_to_lonlat(path: Path, x: float, y: float) -> Tuple[float, float] | None:
    epsg = _regional_dem_epsg(path)
    if epsg is None:
        return float(x), float(y)
    transformer = _transformer(int(epsg), 4326)
    if transformer is None:
        return None
    lon, lat = transformer.transform(float(x), float(y))
    return float(lon), float(lat)


def _lonlat_to_native(path: Path, lon: float, lat: float) -> Tuple[float, float] | None:
    epsg = _regional_dem_epsg(path)
    if epsg is None:
        return float(lon), float(lat)
    transformer = _transformer(4326, int(epsg))
    if transformer is None:
        return None
    x, y = transformer.transform(float(lon), float(lat))
    return float(x), float(y)


def _regional_dem_latlon_bounds(path: Path) -> Tuple[float, float, float, float] | None:
    epsg = _regional_dem_epsg(path)
    if epsg is None:
        return None
    if Transformer is None:
        return None
    try:
        _transform, bounds, _nodata = _load_dem_metadata(path)
        corners = (
            (bounds.left, bounds.bottom),
            (bounds.left, bounds.top),
            (bounds.right, bounds.bottom),
            (bounds.right, bounds.top),
        )
        lonlat = [_native_to_lonlat(path, x, y) for x, y in corners]
        lonlat = [item for item in lonlat if item is not None]
        if not lonlat:
            return None
        lons = [float(item[0]) for item in lonlat]
        lats = [float(item[1]) for item in lonlat]
        return (min(lats), max(lats), min(lons), max(lons))
    except Exception:
        return None


def _load_dem_data_uncached(path: Path):
    """단일 GeoTIFF 타일을 캐시와 함께 로드."""
    if not path.exists():
        raise FileNotFoundError(f"DEM 파일을 찾을 수 없습니다: {path}")

    try:
        with Image.open(path) as img:
            band = np.array(img)
            scale_tag = img.tag_v2.get(_MODEL_PIXEL_SCALE_TAG)
            tiepoint_tag = img.tag_v2.get(_MODEL_TIEPOINT_TAG)
            transform = _transform_from_tags(scale_tag, tiepoint_tag)
            bounds = _bounds_from_transform(band.shape, transform)
            nodata_tag = img.tag_v2.get(_GDAL_NODATA_TAG)
            try:
                nodata = float(nodata_tag) if nodata_tag is not None else None
            except (TypeError, ValueError):
                nodata = None
    except Exception:
        if tifffile is None:
            raise
        with tifffile.TiffFile(str(path)) as tif:
            page = tif.pages[0]
            band = page.asarray()
            scale_tag = page.tags.get(_MODEL_PIXEL_SCALE_TAG)
            tiepoint_tag = page.tags.get(_MODEL_TIEPOINT_TAG)
            scale_value = scale_tag.value if scale_tag is not None else None
            tiepoint_value = tiepoint_tag.value if tiepoint_tag is not None else None
            transform = _transform_from_tags(scale_value, tiepoint_value)
            bounds = _bounds_from_transform(band.shape, transform)
            nodata_tag = page.tags.get(_GDAL_NODATA_TAG)
            nodata_value = nodata_tag.value if nodata_tag is not None else None
            try:
                nodata = float(nodata_value) if nodata_value is not None else None
            except (TypeError, ValueError):
                nodata = None

    return band, transform, bounds, nodata


@lru_cache(maxsize=None)
def _load_dem_data_cached(path_key: str):
    return _load_dem_data_uncached(Path(path_key))


def _load_dem_data_cache_key(path: Path) -> str:
    try:
        return str(Path(path).resolve())
    except Exception:
        return str(Path(path))


def _load_dem_data(path: Path):
    """Load a DEM tile with per-path single-flight protection."""
    key = _load_dem_data_cache_key(path)
    is_leader = False
    with _DEM_LOAD_INFLIGHT_LOCK:
        inflight = _DEM_LOAD_INFLIGHT.get(key)
        if inflight is None:
            event = threading.Event()
            state: Dict[str, Any] = {}
            _DEM_LOAD_INFLIGHT[key] = (event, state)
            is_leader = True
        else:
            event, state = inflight

    if is_leader:
        _record_terrain_elev_many_metrics(demTileLoadLeaderCount=1)
        try:
            result = _load_dem_data_cached(key)
            state["result"] = result
            return result
        except BaseException as exc:
            state["exception"] = exc
            raise
        finally:
            with _DEM_LOAD_INFLIGHT_LOCK:
                current = _DEM_LOAD_INFLIGHT.get(key)
                if current is not None and current[0] is event:
                    _DEM_LOAD_INFLIGHT.pop(key, None)
                event.set()

    wait_started = time.perf_counter()
    _record_terrain_elev_many_metrics(demTileLoadWaiterCount=1)
    completed = event.wait(timeout=30.0)
    _record_terrain_elev_many_metrics(
        demTileLoadWaitMs=(time.perf_counter() - wait_started) * 1000.0,
    )
    if not completed:
        _record_terrain_elev_many_metrics(demTileLoadTimeoutCount=1)
        return _load_dem_data_cached(key)
    exception = state.get("exception")
    if exception is not None:
        raise exception
    if "result" in state:
        return state["result"]
    return _load_dem_data_cached(key)


def _clear_load_dem_data_cache() -> None:
    _load_dem_data_cached.cache_clear()
    with _DEM_LOAD_INFLIGHT_LOCK:
        _DEM_LOAD_INFLIGHT.clear()


_load_dem_data.cache_info = _load_dem_data_cached.cache_info  # type: ignore[attr-defined]
_load_dem_data.cache_clear = _clear_load_dem_data_cache  # type: ignore[attr-defined]


def _candidate_tiles(lat: float, lon: float) -> Iterable[Path]:
    """주어진 좌표를 포함할 수 있는 타일 Path 후보 리스트."""
    for path, (lat0, lat1, lon0, lon1) in _available_dem_tiles():
        if lat0 <= lat <= lat1 and lon0 <= lon <= lon1:
            yield path


def _tile_bounds_contains(bounds: Tuple[float, float, float, float], lat: float, lon: float) -> bool:
    lat0, lat1, lon0, lon1 = bounds
    return float(lat0) <= lat <= float(lat1) and float(lon0) <= lon <= float(lon1)


def _tile_bounds_intersects(
    bounds: Tuple[float, float, float, float],
    query_bounds: Tuple[float, float, float, float],
) -> bool:
    lat0, lat1, lon0, lon1 = (float(v) for v in bounds)
    q_lat0, q_lat1, q_lon0, q_lon1 = (float(v) for v in query_bounds)
    return not (lat1 < q_lat0 or lat0 > q_lat1 or lon1 < q_lon0 or lon0 > q_lon1)


def _dem_warmup_bbox_max_tiles() -> int:
    raw = os.environ.get("MISSION_PLAN_DEM_WARMUP_BBOX_MAX_TILES", "32")
    try:
        return max(0, int(float(raw)))
    except Exception:
        return 32


def _select_dem_warmup_tiles_for_points(
    pairs: list[Tuple[float, float]],
    tiles: list[Tuple[Path, Tuple[float, float, float, float]]],
) -> list[Path]:
    if not pairs or not tiles:
        return []
    finite_pairs = [
        (float(lat), float(lon))
        for lat, lon in pairs
        if math.isfinite(float(lat)) and math.isfinite(float(lon))
    ]
    if not finite_pairs:
        return []

    min_lat = min(lat for lat, _lon in finite_pairs)
    max_lat = max(lat for lat, _lon in finite_pairs)
    min_lon = min(lon for _lat, lon in finite_pairs)
    max_lon = max(lon for _lat, lon in finite_pairs)
    query_bounds = (min_lat, max_lat, min_lon, max_lon)
    max_tiles = _dem_warmup_bbox_max_tiles()
    selected: list[Path] = []
    seen: set[str] = set()

    def add_path(path: Path) -> None:
        if max_tiles and len(selected) >= max_tiles:
            return
        try:
            key = str(Path(path).resolve())
        except Exception:
            key = str(path)
        if key in seen:
            return
        seen.add(key)
        selected.append(path)

    # Direct coordinate hits first, then the full bbox. This keeps narrow
    # scenarios cheap while still priming sweep/lineSearch tiles between points.
    for path, bounds in tiles:
        if any(_tile_bounds_contains(bounds, lat, lon) for lat, lon in finite_pairs):
            add_path(path)
    for path, bounds in tiles:
        if _tile_bounds_intersects(bounds, query_bounds):
            add_path(path)
    return selected


def _build_dem_tile_candidate_index(
    tiles: list[Tuple[Path, Tuple[float, float, float, float]]],
) -> tuple[list[int], Dict[tuple[int, int], list[int]]]:
    priority_indices: list[int] = []
    bucket_map: Dict[tuple[int, int], list[int]] = {}
    for tile_idx, (path, bounds) in enumerate(tiles):
        lat0, lat1, lon0, lon1 = bounds
        is_degree_tile = _DEM_TILE_RE.search(path.stem) is not None and _regional_dem_epsg(path) is None
        if not is_degree_tile:
            priority_indices.append(tile_idx)
            continue
        try:
            lat_start = int(math.floor(float(lat0)))
            lat_end = int(math.floor(float(lat1)))
            lon_start = int(math.floor(float(lon0)))
            lon_end = int(math.floor(float(lon1)))
        except Exception:
            priority_indices.append(tile_idx)
            continue
        for lat_key in range(min(lat_start, lat_end), max(lat_start, lat_end) + 1):
            for lon_key in range(min(lon_start, lon_end), max(lon_start, lon_end) + 1):
                bucket_map.setdefault((lat_key, lon_key), []).append(tile_idx)
    return priority_indices, bucket_map


@lru_cache(maxsize=8)
def _dem_usage_log_path_cached(
    scenario_info_mtime_ns: int,
    db_root_hint: str,
) -> Path | None:
    """Resolve the audit path once per active scenario.

    Terrain sampling can call the audit guard thousands of times while a route
    is being profiled.  Resolving the DB root on every point needlessly stats,
    locks, and validates the scenario tree even though only one event per DEM
    is written.  The current-scenario file mtime keeps this cache sensitive to
    an external scenario switch; the environment hint covers in-process root
    changes before the settings file timestamp is observed.
    """

    try:
        from modules.common import db_paths
        log_dir = db_paths.get_db_subpath("DSS_Internal")
    except Exception:
        db_root = db_root_hint or os.environ.get("KU_MISSION_DB_ROOT")
        if not db_root:
            return None
        log_dir = Path(db_root) / "DSS_Internal"

    try:
        log_dir.mkdir(parents=True, exist_ok=True)
        return log_dir / "dem_usage.jsonl"
    except Exception:
        return None


def _dem_usage_log_path() -> Path | None:
    """Return the scenario-specific DEM audit path without hot-loop I/O."""

    db_root_hint = str(os.environ.get("KU_MISSION_DB_ROOT") or "")
    scenario_info_mtime_ns = 0
    try:
        from modules.common import db_paths

        scenario_info_mtime_ns = int(db_paths.INFO_PATH.stat().st_mtime_ns)
    except Exception:
        pass
    return _dem_usage_log_path_cached(scenario_info_mtime_ns, db_root_hint)


def _dem_usage_latlon_bounds(path: Path) -> Dict[str, float] | None:
    try:
        bounds_llh = None
        if _regional_dem_epsg(path) is not None:
            bounds_llh = _regional_dem_latlon_bounds(path)
        if bounds_llh is None:
            for tile_path, tile_bounds in _available_dem_tiles():
                if tile_path == path:
                    bounds_llh = tile_bounds
                    break
        if bounds_llh is None:
            return None
        lat0, lat1, lon0, lon1 = bounds_llh
        return {
            "minLatitude": float(lat0),
            "maxLatitude": float(lat1),
            "minLongitude": float(lon0),
            "maxLongitude": float(lon1),
        }
    except Exception:
        return None


def _append_dem_usage_event(payload: Dict[str, Any]) -> None:
    log_path = _dem_usage_log_path()
    if log_path is None:
        return
    try:
        with log_path.open("a", encoding="utf-8") as fh:
            fh.write(json.dumps(payload, ensure_ascii=False, sort_keys=True) + "\n")
    except Exception:
        pass


def _dem_inventory_payload() -> Dict[str, Any]:
    """Return a JSON-ready snapshot of the operational DEM deployment."""

    try:
        return dict(regional_dem_inventory(_DEM_DIR))
    except Exception as exc:
        return {
            "resourceDir": str(_DEM_DIR),
            "expectedDemNames": list(_REGIONAL_DEM_PRIORITY),
            "expectedDemPaths": [str(_DEM_DIR / name) for name in _REGIONAL_DEM_PRIORITY],
            "availableDemNames": [],
            "availableDemPaths": [],
            "missingDemNames": list(_REGIONAL_DEM_PRIORITY),
            "missingDemPaths": [str(_DEM_DIR / name) for name in _REGIONAL_DEM_PRIORITY],
            "detectedTifNames": [],
            "detectedTifPaths": [],
            "unregisteredTifNames": [],
            "unregisteredTifPaths": [],
            "inventoryError": f"{type(exc).__name__}: {exc}",
        }


def _dem_coordinate_diagnostic(lat: float, lon: float) -> Dict[str, Any]:
    inventory = _dem_inventory_payload()
    try:
        spec = select_regional_dem(float(lat), float(lon))
    except Exception:
        spec = None

    expected_path = _DEM_DIR / spec.filename if spec is not None else None
    if spec is None:
        reason = "coordinate_outside_operational_coverage"
    elif expected_path is None or not expected_path.is_file():
        reason = "required_dem_file_missing"
    else:
        reason = "required_dem_present_but_unresolved"

    inventory.update(
        {
            "reason": reason,
            "expectedDemName": spec.filename if spec is not None else None,
            "expectedDemPath": str(expected_path) if expected_path is not None else None,
            "expectedDemExists": bool(expected_path and expected_path.is_file()),
            "sample": {
                "latitude": float(lat),
                "longitude": float(lon),
            },
        }
    )
    return inventory


def _log_dem_inventory_once(inventory: Dict[str, Any] | None = None) -> None:
    snapshot = dict(inventory or _dem_inventory_payload())
    missing_names = tuple(str(name) for name in snapshot.get("missingDemNames", ()) or ())
    if not missing_names:
        return
    log_path = _dem_usage_log_path()
    if log_path is None:
        return
    key = (str(log_path), "inventory_incomplete", missing_names)
    with _DEM_USAGE_LOCK:
        if key in _DEM_MISS_LOGGED_KEYS:
            return
        _DEM_MISS_LOGGED_KEYS.add(key)
    snapshot.update(
        {
            "event": "dem_inventory_incomplete",
            "source": "mission_planning",
            "timestampUtc": datetime.now(timezone.utc).isoformat(),
            "reason": "operational_dem_files_missing",
        }
    )
    _append_dem_usage_event(snapshot)


def _log_dem_usage_once(
    path: Path,
    lat: float,
    lon: float,
    native_x: float,
    native_y: float,
    value: float,
) -> None:
    log_path = _dem_usage_log_path()
    if log_path is None:
        return
    key = (str(log_path), path.name)
    with _DEM_USAGE_LOCK:
        if key in _DEM_USAGE_LOGGED_KEYS:
            return
        _DEM_USAGE_LOGGED_KEYS.add(key)

    epsg = _regional_dem_epsg(path)
    payload: Dict[str, Any] = {
        "event": "dem_source_selected",
        "source": "mission_planning",
        "timestampUtc": datetime.now(timezone.utc).isoformat(),
        "demName": path.name,
        "demPath": str(path),
        "epsg": int(epsg) if epsg is not None else 4326,
        "sample": {
            "latitude": float(lat),
            "longitude": float(lon),
            "altitude": float(value),
            "nativeX": float(native_x),
            "nativeY": float(native_y),
        },
    }
    bounds = _dem_usage_latlon_bounds(path)
    if bounds is not None:
        payload["latlonBounds"] = bounds
    _append_dem_usage_event(payload)


def _log_dem_miss_once(lat: float, lon: float) -> None:
    log_path = _dem_usage_log_path()
    if log_path is None:
        return
    diagnostic = _dem_coordinate_diagnostic(lat, lon)
    reason = str(diagnostic.get("reason") or "dem_source_missing")
    expected_name = str(diagnostic.get("expectedDemName") or "outside_operational_coverage")
    key = (str(log_path), "missing", reason, expected_name)
    with _DEM_USAGE_LOCK:
        if key in _DEM_MISS_LOGGED_KEYS:
            return
        _DEM_MISS_LOGGED_KEYS.add(key)

    diagnostic.update(
        {
            "event": "dem_source_missing",
            "source": "mission_planning",
            "timestampUtc": datetime.now(timezone.utc).isoformat(),
        }
    )
    _append_dem_usage_event(diagnostic)


def _log_dem_error_once(lat: float, lon: float, exc: BaseException) -> None:
    log_path = _dem_usage_log_path()
    if log_path is None:
        return
    diagnostic = _dem_coordinate_diagnostic(lat, lon)
    expected_name = str(diagnostic.get("expectedDemName") or "outside_operational_coverage")
    error_type = type(exc).__name__
    key = (str(log_path), "read_error", expected_name, error_type)
    with _DEM_USAGE_LOCK:
        if key in _DEM_MISS_LOGGED_KEYS:
            return
        _DEM_MISS_LOGGED_KEYS.add(key)
    diagnostic.update(
        {
            "event": "dem_source_error",
            "source": "mission_planning",
            "timestampUtc": datetime.now(timezone.utc).isoformat(),
            "reason": "required_dem_read_error",
            "errorType": error_type,
            "error": str(exc)[:1000],
        }
    )
    _append_dem_usage_event(diagnostic)


def _log_dem_nodata_once(path: Path, lat: float, lon: float, value: float) -> None:
    log_path = _dem_usage_log_path()
    if log_path is None:
        return
    key = (str(log_path), "nodata", path.name)
    with _DEM_USAGE_LOCK:
        if key in _DEM_MISS_LOGGED_KEYS:
            return
        _DEM_MISS_LOGGED_KEYS.add(key)
    diagnostic = _dem_coordinate_diagnostic(lat, lon)
    diagnostic.update(
        {
            "event": "dem_source_nodata",
            "source": "mission_planning",
            "timestampUtc": datetime.now(timezone.utc).isoformat(),
            "reason": "sample_is_nodata",
            "demName": path.name,
            "demPath": str(path),
            "sampleValue": float(value) if math.isfinite(float(value)) else None,
        }
    )
    _append_dem_usage_event(diagnostic)


@lru_cache(maxsize=65536)
def _terrain_elev_cached(lat: float, lon: float) -> float:
    """지형(DEM)에서 가져온 GeoTIFF 고도(m). 없으면 0."""
    chosen_tile = None
    for path in _candidate_tiles(lat, lon):
        band, transform, bounds, nodata = _load_dem_data(path)
        native_xy = _lonlat_to_native(path, lon, lat)
        if native_xy is None:
            continue
        native_x, native_y = native_xy
        if bounds.bottom <= native_y <= bounds.top and bounds.left <= native_x <= bounds.right:
            chosen_tile = (path, band, transform, bounds, nodata, native_x, native_y)
            break
    else:
        for path, _approx in _available_dem_tiles():
            band, transform, bounds, nodata = _load_dem_data(path)
            native_xy = _lonlat_to_native(path, lon, lat)
            if native_xy is None:
                continue
            native_x, native_y = native_xy
            if bounds.bottom <= native_y <= bounds.top and bounds.left <= native_x <= bounds.right:
                chosen_tile = (path, band, transform, bounds, nodata, native_x, native_y)
                break
    if chosen_tile is None:
        _log_dem_miss_once(lat, lon)
        return 0.0

    _path, band, transform, bounds, nodata, native_x, native_y = chosen_tile

    col_f, row_f = (~transform) * (native_x, native_y)
    max_row, max_col = band.shape[0] - 1, band.shape[1] - 1
    # The inverse transform yields cell-CORNER coordinates: an integer sits on a
    # cell boundary and a centre lands on .5.  Rounding those straight to an
    # index picks the neighbouring cell half the time - a whole cell of offset,
    # which on a slope reads tens of metres off and made the cover analyser and
    # this lookup disagree about the same ground.
    row = int(max(0, min(max_row, math.floor(row_f))))
    col = int(max(0, min(max_col, math.floor(col_f))))

    value = float(band[row, col])
    if math.isnan(value):
        _log_dem_nodata_once(_path, lat, lon, value)
        return 0.0
    if nodata is not None and math.isclose(value, nodata, abs_tol=1e-3):
        _log_dem_nodata_once(_path, lat, lon, value)
        return 0.0
    _log_dem_usage_once(_path, lat, lon, native_x, native_y, value)
    return value


def terrain_elev(lat: float, lon: float) -> float:
    try:
        ensure_terrain_cache_current()
        return _terrain_elev_cached(float(lat), float(lon))
    except Exception as exc:
        _log_dem_error_once(float(lat), float(lon), exc)
        return 0.0


def terrain_source_name(lat: float, lon: float) -> str | None:
    """Return the selected operational DEM name for a WGS84 point."""

    try:
        spec = select_regional_dem(float(lat), float(lon))
    except Exception:
        return None
    if spec is None or not (_DEM_DIR / spec.filename).is_file():
        return None
    return str(spec.filename)


def _transform_lonlat_arrays(path: Path, lons: np.ndarray, lats: np.ndarray) -> Tuple[np.ndarray, np.ndarray] | None:
    epsg = _regional_dem_epsg(path)
    if epsg is None:
        return lons.astype(float, copy=False), lats.astype(float, copy=False)
    transformer = _transformer(4326, int(epsg))
    if transformer is None:
        return None
    try:
        xs, ys = transformer.transform(lons, lats)
        return np.asarray(xs, dtype=float), np.asarray(ys, dtype=float)
    except Exception:
        return None


def _transform_to_rows_cols(
    transform: Affine,
    xs: np.ndarray,
    ys: np.ndarray,
    shape: Tuple[int, int],
) -> Tuple[np.ndarray, np.ndarray]:
    inv = ~transform
    cols_f = (float(inv.a) * xs) + (float(inv.b) * ys) + float(inv.c)
    rows_f = (float(inv.d) * xs) + (float(inv.e) * ys) + float(inv.f)
    max_row, max_col = int(shape[0]) - 1, int(shape[1]) - 1
    # Corner-based fractional indices: floor selects the containing cell, the
    # same rule the cover analyser applies.  ``rint`` would straddle boundaries.
    rows = np.floor(rows_f).astype(np.int64)
    cols = np.floor(cols_f).astype(np.int64)
    rows = np.clip(rows, 0, max_row)
    cols = np.clip(cols, 0, max_col)
    return rows, cols


def _read_dem_pixels_cached(
    path: Path,
    band: np.ndarray,
    rows: np.ndarray,
    cols: np.ndarray,
    nodata: float | None,
) -> np.ndarray:
    values = np.zeros(len(rows), dtype=float)
    missing_positions_by_key: Dict[Tuple[str, int, int], list[int]] = {}
    path_key = str(path)
    cache_keys = [(path_key, int(row), int(col)) for row, col in zip(rows, cols)]

    cache_read_started = time.perf_counter()
    with _DEM_PIXEL_CACHE_LOCK:
        for idx, key in enumerate(cache_keys):
            cached = _DEM_PIXEL_CACHE.get(key)
            if cached is None:
                missing_positions_by_key.setdefault(key, []).append(idx)
            else:
                values[idx] = float(cached)
    missing_position_count = sum(len(items) for items in missing_positions_by_key.values())
    unique_missing_keys = list(missing_positions_by_key.keys())
    _record_terrain_elev_many_metrics(
        demCacheReadMs=(time.perf_counter() - cache_read_started) * 1000.0,
        demPixelCacheHitCount=max(0, len(cache_keys) - missing_position_count),
        demPixelCacheMissCount=missing_position_count,
        demPixelCacheUniqueMissCount=len(unique_missing_keys),
    )

    if unique_missing_keys:
        miss_rows = np.asarray([key[1] for key in unique_missing_keys], dtype=np.int64)
        miss_cols = np.asarray([key[2] for key in unique_missing_keys], dtype=np.int64)
        pixel_read_started = time.perf_counter()
        raw = np.asarray(band[miss_rows, miss_cols], dtype=float)
        _record_terrain_elev_many_metrics(
            demPixelReadMs=(time.perf_counter() - pixel_read_started) * 1000.0,
        )
        invalid = np.isnan(raw)
        if nodata is not None:
            try:
                invalid |= np.isclose(raw, float(nodata), atol=1e-3)
            except Exception:
                pass
        raw = np.where(invalid, 0.0, raw)
        for key, value in zip(unique_missing_keys, raw):
            for idx in missing_positions_by_key.get(key, []):
                values[idx] = float(value)
        cache_write_started = time.perf_counter()
        with _DEM_PIXEL_CACHE_LOCK:
            if len(_DEM_PIXEL_CACHE) + len(unique_missing_keys) >= _DEM_PIXEL_CACHE_MAX:
                _DEM_PIXEL_CACHE.clear()
                _record_terrain_elev_many_metrics(demPixelCacheClearCount=1)
            for key, value in zip(unique_missing_keys, raw):
                _DEM_PIXEL_CACHE[key] = float(value)
        _record_terrain_elev_many_metrics(
            demCacheWriteMs=(time.perf_counter() - cache_write_started) * 1000.0,
        )

    return values


def _apply_dem_tile_to_indices(
    path: Path,
    indices: list[int],
    pairs: list[Tuple[float, float]],
    results: list[float],
    resolved: list[bool],
) -> None:
    if not indices:
        return
    apply_started = time.perf_counter()
    try:
        try:
            load_started = time.perf_counter()
            band, transform, bounds, nodata = _load_dem_data(path)
            _record_terrain_elev_many_metrics(
                demTileLoadMs=(time.perf_counter() - load_started) * 1000.0,
            )
        except Exception:
            return

        lats = np.asarray([pairs[idx][0] for idx in indices], dtype=float)
        lons = np.asarray([pairs[idx][1] for idx in indices], dtype=float)
        transform_started = time.perf_counter()
        native = _transform_lonlat_arrays(path, lons, lats)
        _record_terrain_elev_many_metrics(
            demNativeTransformMs=(time.perf_counter() - transform_started) * 1000.0,
        )
        if native is None:
            return
        xs, ys = native
        inside = (
            (ys >= float(bounds.bottom))
            & (ys <= float(bounds.top))
            & (xs >= float(bounds.left))
            & (xs <= float(bounds.right))
        )
        if not bool(np.any(inside)):
            return

        local_positions = np.nonzero(inside)[0]
        chosen_indices = [indices[int(pos)] for pos in local_positions]
        chosen_xs = xs[local_positions]
        chosen_ys = ys[local_positions]
        rowcol_started = time.perf_counter()
        rows, cols = _transform_to_rows_cols(transform, chosen_xs, chosen_ys, band.shape)
        _record_terrain_elev_many_metrics(
            demRowColTransformMs=(time.perf_counter() - rowcol_started) * 1000.0,
        )
        values = _read_dem_pixels_cached(path, band, rows, cols, nodata)

        usage_logged = False
        newly_resolved = 0
        for global_idx, native_x, native_y, value in zip(chosen_indices, chosen_xs, chosen_ys, values):
            was_resolved = bool(resolved[global_idx])
            results[global_idx] = float(value)
            resolved[global_idx] = True
            if not was_resolved:
                newly_resolved += 1
            if not usage_logged and float(value) != 0.0:
                lat, lon = pairs[global_idx]
                _log_dem_usage_once(path, lat, lon, float(native_x), float(native_y), float(value))
                usage_logged = True
        _record_terrain_elev_many_metrics(demResolvedByTile=max(0, newly_resolved))
    finally:
        _record_terrain_elev_many_metrics(
            demTileApplyMs=(time.perf_counter() - apply_started) * 1000.0,
            demTileApplyCallCount=1,
        )


def terrain_elev_many(coords: Iterable[Any]) -> list[float]:
    ensure_terrain_cache_current()
    pairs: list[Tuple[float, float]] = []
    for item in coords or []:
        pair = _coerce_lat_lon(item)
        if pair is None:
            pairs.append((math.nan, math.nan))
        else:
            pairs.append(pair)
    if not pairs:
        return []

    results = [0.0 for _ in pairs]
    resolved = [False for _ in pairs]
    unresolved = [idx for idx, (lat, lon) in enumerate(pairs) if math.isfinite(lat) and math.isfinite(lon)]

    try:
        tiles = list(_available_dem_tiles())
    except Exception:
        tiles = []
    _record_terrain_elev_many_metrics(demTileCount=len(tiles))

    tile_resolve_started = time.perf_counter()
    tile_candidate_map: Dict[int, list[int]] = {idx: [] for idx in range(len(tiles))}
    tile_index_started = time.perf_counter()
    priority_tile_indices, tile_bucket_map = _build_dem_tile_candidate_index(tiles)
    _record_terrain_elev_many_metrics(
        demTileCandidateIndexMs=(time.perf_counter() - tile_index_started) * 1000.0,
    )
    candidate_assign_started = time.perf_counter()
    tile_candidate_count = 0
    for idx in unresolved:
        lat, lon = pairs[idx]
        assigned_tile_indices: set[int] = set()

        def assign_candidate(tile_idx: int) -> None:
            nonlocal tile_candidate_count
            if tile_idx in assigned_tile_indices:
                return
            tile_candidate_map[tile_idx].append(idx)
            assigned_tile_indices.add(tile_idx)
            tile_candidate_count += 1

        for tile_idx in priority_tile_indices:
            _path, bounds = tiles[tile_idx]
            if _tile_bounds_contains(bounds, lat, lon):
                assign_candidate(tile_idx)
        try:
            bucket_key = (int(math.floor(float(lat))), int(math.floor(float(lon))))
            bucket_candidates = tile_bucket_map.get(bucket_key, ())
        except Exception:
            bucket_candidates = ()
        for tile_idx in bucket_candidates:
            _path, bounds = tiles[tile_idx]
            if _tile_bounds_contains(bounds, lat, lon):
                assign_candidate(tile_idx)
                break
    _record_terrain_elev_many_metrics(
        demTileCandidateAssignMs=(time.perf_counter() - candidate_assign_started) * 1000.0,
        demTileCandidateCount=tile_candidate_count,
    )

    for tile_idx, (path, _bounds) in enumerate(tiles):
        candidates = [idx for idx in tile_candidate_map.get(tile_idx, []) if not resolved[idx]]
        _apply_dem_tile_to_indices(path, candidates, pairs, results, resolved)

    remaining = [idx for idx in unresolved if not resolved[idx]]
    fallback_started = time.perf_counter()
    fallback_candidate_count = 0
    if remaining:
        for path, _approx in tiles:
            candidates = [idx for idx in remaining if not resolved[idx]]
            if not candidates:
                break
            fallback_candidate_count += len(candidates)
            _apply_dem_tile_to_indices(path, candidates, pairs, results, resolved)
    _record_terrain_elev_many_metrics(
        demTileFallbackScanMs=(time.perf_counter() - fallback_started) * 1000.0,
        demTileFallbackCandidateCount=fallback_candidate_count,
    )
    _record_terrain_elev_many_metrics(
        demTileResolveMs=(time.perf_counter() - tile_resolve_started) * 1000.0,
    )

    for idx in unresolved:
        if not resolved[idx]:
            lat, lon = pairs[idx]
            _log_dem_miss_once(lat, lon)
    return results


def _dem_requirement_summary(pairs: list[Tuple[float, float]]) -> Dict[str, Any]:
    inventory = _dem_inventory_payload()
    requested_names: list[str] = []
    requested_paths: list[str] = []
    missing_required_names: list[str] = []
    missing_required_paths: list[str] = []
    missing_samples: list[Dict[str, Any]] = []
    missing_point_count = 0
    outside_point_count = 0
    first_missing_by_key: Dict[str, Tuple[float, float]] = {}

    for lat, lon in pairs:
        try:
            spec = select_regional_dem(float(lat), float(lon))
        except Exception:
            spec = None
        if spec is None:
            outside_point_count += 1
            key = "outside_operational_coverage"
            first_missing_by_key.setdefault(key, (float(lat), float(lon)))
            if len(missing_samples) < 8:
                missing_samples.append(
                    {
                        "latitude": float(lat),
                        "longitude": float(lon),
                        "reason": "coordinate_outside_operational_coverage",
                        "expectedDemName": None,
                        "expectedDemPath": None,
                    }
                )
            continue

        expected_path = _DEM_DIR / spec.filename
        if spec.filename not in requested_names:
            requested_names.append(spec.filename)
            requested_paths.append(str(expected_path))
        if expected_path.is_file():
            continue

        missing_point_count += 1
        if spec.filename not in missing_required_names:
            missing_required_names.append(spec.filename)
            missing_required_paths.append(str(expected_path))
        first_missing_by_key.setdefault(spec.filename, (float(lat), float(lon)))
        if len(missing_samples) < 8:
            missing_samples.append(
                {
                    "latitude": float(lat),
                    "longitude": float(lon),
                    "reason": "required_dem_file_missing",
                    "expectedDemName": spec.filename,
                    "expectedDemPath": str(expected_path),
                }
            )

    _log_dem_inventory_once(inventory)
    for lat, lon in first_missing_by_key.values():
        _log_dem_miss_once(lat, lon)

    return {
        **inventory,
        "requestedDemNames": requested_names,
        "requestedDemPaths": requested_paths,
        "missingRequiredDemNames": missing_required_names,
        "missingRequiredDemPaths": missing_required_paths,
        "missingRequiredPointCount": int(missing_point_count),
        "outsideOperationalCoveragePointCount": int(outside_point_count),
        "unresolvedRequirementPointCount": int(missing_point_count + outside_point_count),
        "unresolvedRequirementSamples": missing_samples,
    }


def warm_terrain_cache(coords: Iterable[Any] | None = None) -> Dict[str, Any]:
    """Prime DEM data and return explicit deployment/coverage diagnostics."""
    pairs: list[Tuple[float, float]] = []
    for item in coords or []:
        pair = _coerce_lat_lon(item)
        if pair is not None:
            pairs.append(pair)

    requirement = _dem_requirement_summary(pairs)
    loaded_tiles: list[str] = []
    bbox_loaded_tiles: list[str] = []
    load_errors: list[Dict[str, Any]] = []
    checked_epsg: set[int] = set()
    for name in requirement.get("requestedDemNames", ()) or ():
        epsg = _REGIONAL_DEM_EPSG_BY_NAME.get(str(name).lower())
        if epsg is None or int(epsg) in checked_epsg:
            continue
        checked_epsg.add(int(epsg))
        if _transformer(4326, int(epsg)) is None:
            load_errors.append(
                {
                    "stage": "coordinate_transform",
                    "demName": str(name),
                    "errorType": "TransformerUnavailable",
                    "error": f"pyproj Transformer EPSG:4326 -> EPSG:{int(epsg)} is unavailable",
                }
            )
    try:
        tiles = list(_available_dem_tiles())
    except Exception as exc:
        tiles = []
        load_errors.append(
            {
                "stage": "scan",
                "errorType": type(exc).__name__,
                "error": str(exc)[:1000],
                "resourceDir": str(_DEM_DIR),
            }
        )

    if pairs:
        for path in _select_dem_warmup_tiles_for_points(pairs, tiles):
            try:
                _load_dem_data(path)
                path_text = str(path)
                loaded_tiles.append(path_text)
                bbox_loaded_tiles.append(path_text)
            except Exception as exc:
                load_errors.append(
                    {
                        "stage": "load",
                        "demName": path.name,
                        "demPath": str(path),
                        "errorType": type(exc).__name__,
                        "error": str(exc)[:1000],
                    }
                )
                for lat, lon in pairs:
                    spec = select_regional_dem(float(lat), float(lon))
                    if spec is not None and spec.filename.lower() == path.name.lower():
                        _log_dem_error_once(lat, lon, exc)
                        break
        try:
            terrain_elev_many(pairs)
        except Exception as exc:
            load_errors.append(
                {
                    "stage": "batch_sample",
                    "errorType": type(exc).__name__,
                    "error": str(exc)[:1000],
                }
            )
            if pairs:
                _log_dem_error_once(pairs[0][0], pairs[0][1], exc)
    else:
        priority_names = {str(name).lower() for name in _REGIONAL_DEM_PRIORITY}
        paths = [path for path, _bounds in tiles if path.name.lower() in priority_names]
        if not paths and tiles:
            paths = [tiles[0][0]]
        for path in paths:
            try:
                _load_dem_data(path)
                loaded_tiles.append(str(path))
            except Exception as exc:
                load_errors.append(
                    {
                        "stage": "load",
                        "demName": path.name,
                        "demPath": str(path),
                        "errorType": type(exc).__name__,
                        "error": str(exc)[:1000],
                    }
                )

    info = terrain_cache_info()
    info["warmup"] = {
        "inputPoints": int(len(pairs)),
        "loadedTiles": loaded_tiles,
        "bboxLoadedTiles": bbox_loaded_tiles,
        **requirement,
        "loadErrors": load_errors,
        "inventoryComplete": not bool(requirement.get("missingDemNames")),
        "requestedCoverageReady": not bool(requirement.get("unresolvedRequirementPointCount"))
        and not bool(load_errors),
    }
    return info


def _cache_info_dict(cache_func: Any) -> Dict[str, Any]:
    cache_info = getattr(cache_func, "cache_info", None)
    if not callable(cache_info):
        return {}
    try:
        info = cache_info()
    except Exception:
        return {}
    return {
        "hits": int(getattr(info, "hits", 0)),
        "misses": int(getattr(info, "misses", 0)),
        "maxsize": getattr(info, "maxsize", None),
        "currsize": int(getattr(info, "currsize", 0)),
    }


def terrain_cache_info() -> Dict[str, Dict[str, Any]]:
    ensure_terrain_cache_current()
    terrain_info = _cache_info_dict(_terrain_elev_cached)
    hits = int(terrain_info.get("hits", 0) or 0)
    misses = int(terrain_info.get("misses", 0) or 0)
    total = hits + misses
    terrain_info["hit_ratio"] = round(float(hits) / float(total), 6) if total else 0.0
    return {
        "terrain_elev": terrain_info,
        "terrain_pixel": {
            "maxsize": int(_DEM_PIXEL_CACHE_MAX),
            "currsize": int(len(_DEM_PIXEL_CACHE)),
        },
        "load_dem_data": _cache_info_dict(_load_dem_data),
        "available_dem_tiles": _cache_info_dict(_available_dem_tiles),
    }


def clear_terrain_cache() -> None:
    global _DEM_CACHE_SIGNATURE_LAST_CHECKED
    for cache_func in (_terrain_elev_cached, _load_dem_data, _available_dem_tiles, terrain_data_signature):
        cache_clear = getattr(cache_func, "cache_clear", None)
        if callable(cache_clear):
            try:
                cache_clear()
            except Exception:
                pass
    with _DEM_PIXEL_CACHE_LOCK:
        _DEM_PIXEL_CACHE.clear()
    with _DEM_USAGE_LOCK:
        _DEM_USAGE_LOGGED_KEYS.clear()
        _DEM_MISS_LOGGED_KEYS.clear()
    with _DEM_CACHE_SIGNATURE_LOCK:
        _DEM_CACHE_SIGNATURE_LAST_CHECKED = 0.0


def _coerce_lat_lon(item: Any) -> Tuple[float, float] | None:
    if isinstance(item, dict):
        lat = item.get("latitude", item.get("lat"))
        lon = item.get("longitude", item.get("lon"))
    else:
        try:
            lat, lon = item[0], item[1]
        except Exception:
            return None
    try:
        return float(lat), float(lon)
    except (TypeError, ValueError):
        return None


def terrain_precision_probe(
    coords: Iterable[Any],
    precisions: Iterable[int] = (4, 5, 6),
) -> Dict[str, Any]:
    pairs = []
    for item in coords or []:
        pair = _coerce_lat_lon(item)
        if pair is not None:
            pairs.append(pair)

    exact_unique = len(set(pairs))
    rounded_unique = {}
    duplicate_reduction = {}
    for precision_raw in precisions:
        precision = int(precision_raw)
        rounded_count = len({(round(lat, precision), round(lon, precision)) for lat, lon in pairs})
        rounded_unique[str(precision)] = rounded_count
        duplicate_reduction[str(precision)] = max(0, exact_unique - rounded_count)

    return {
        "sample_count": len(pairs),
        "exact_unique": exact_unique,
        "rounded_unique": rounded_unique,
        "duplicate_reduction": duplicate_reduction,
    }

def rand_coord() -> dict:
    """임의 좌표 (위·경·고도) 하나 생성"""
    return {
        "latitude":  round(random.uniform(-90,  90), 6),
        "longitude": round(random.uniform(-180, 180), 6),
        "altitude":  round(random.uniform(50,  500), 1),
    }

def _corridor_polygon(path_ll, width_m):
    """
    path_ll : [(lat, lon), ...]   (최소 2점)
    width_m : 전체 폭 [m]
    반환     : corridor 바깥 경계 좌표 리스트 (Polygon)
    - 근사치 : 소규모 지역이므로 평면으로 간주
    """
    half = width_m / 2.0
    poly_left  = []
    poly_right = []

    for i in range(len(path_ll)-1):
        lat1, lon1 = path_ll[i]
        lat2, lon2 = path_ll[i+1]

        # 단위 벡터 (동-북)
        dx = (lon2 - lon1) * 111_000 * math.cos(math.radians((lat1+lat2)/2))
        dy = (lat2 - lat1) * 111_000
        L  = math.hypot(dx, dy)
        if L == 0:  # 동일 점
            continue
        ux, uy = dx/L, dy/L
        # 좌/우 수직 방향 (CW, CCW)
        px, py =  uy, -ux

        # 좌우 offset (단위: 위도/경도)
        dlat = (py * half) / 111_000
        dlon = (px * half) / (111_000 * math.cos(math.radians(lat1)))

        # 세그먼트 시작, 끝 점 offset
        left_start   = (lat1 + dlat, lon1 + dlon)
        right_start  = (lat1 - dlat, lon1 - dlon)
        left_end     = (lat2 + dlat, lon2 + dlon)
        right_end    = (lat2 - dlat, lon2 - dlon)

        if i == 0:
            poly_left.append(left_start)
            poly_right.append(right_start)
        poly_left.append(left_end)
        poly_right.append(right_end)

    return poly_left + poly_right[::-1]   # 폐곡선

def make_individual_mission(tmp_idx: int | None = None) -> dict:
    """
    ▣ 새 Individual Mission 기본 골격을 만든다.
      · IndividualMissionID  : 900 000 001~  (id_allocator.next_individual_mission_id)
      · PathID               : 0  (= 미정 → 이후 aircraftID 확정 시 next_path_id(aid)로 덮어쓰기)
    """
    im_id = int(reserve_individual_mission_ids(1)[0])

    return {
        "individualMissionID": im_id,          # uint32 순차 ID
        "isDone": False,
        "relatedMission": {
            "relatedMissionType": 0,
            "inputMissionID":    0,
            "priorMissionID":    0,
        },
        "individualMissionInfo": {
            "individualMissionType": 0,        # 0 = 미지정
            "patternType":          0,
            "autoZoomIn":           True,
            "coordinateList":       [],
            "lineList":             [],
            "areaList":             [],
            "targetID":             0,
        },
        "pathID": 0,                           # ★ aircraftID 확정 뒤에 덮어쓴다
    }

def add_mission_shapes(fmap, missions):
    color_scale = linear.Set1_09.scale(0, 8)  # 최대 9종 색
    ac_colors = {}

    # 현재 임무 목록을 순차적으로 돌며, 각 임무의 도형과 점들을 지도에 추가합니다.
    for m in missions:
        aid = m.get("aircraftID", 0)
        if aid not in ac_colors:
            ac_colors[aid] = color_scale(len(ac_colors))
        color = ac_colors[aid]

        info = m["individualMissionInfo"]
        mtype = info["individualMissionType"]

        # 1) 영역 수색 -> Polygon
        if mtype == 1 and info.get("areaList"):
            for area in info["areaList"]:
                coords = [(c["latitude"], c["longitude"]) for c in area["coordinateList"]]
                if len(coords) >= 3:  # Polygon은 최소 3점 이상이어야 함
                    folium.Polygon(locations=coords,
                                   color=color, weight=2,
                                   fill=True, fill_opacity=0.2,
                                   tooltip=f"A/C {aid} : Area").add_to(fmap)
                # 영역 수색에서 점 마커 추가
                for p in coords:
                    folium.CircleMarker(p, radius=4, color=color,
                                         fill=True, fill_opacity=0.9).add_to(fmap)

        # 2) 통로 정찰 -> Line + 폭 표시(간단하게 PolyLine 두께로 표현)
        elif mtype == 2 and info.get("lineList"):
            for line in info["lineList"]:
                # ── ① 좌표 추출 ─────────────────────────
                coords = [(c["latitude"], c["longitude"])
                        for c in line["coordinateList"]]

                if len(coords) < 2:          # 좌표가 2개 미만이면 skip
                    continue

                w_m = line["width"]

                # ── ② 폭(m) → Polygon 면적 생성 ────────
                corridor_poly = _corridor_polygon(coords, w_m)
                folium.Polygon(
                    locations=corridor_poly,
                    color=color, weight=1,
                    fill=True, fill_opacity=0.2,
                    tooltip=f"A/C {aid} : {w_m} m Corridor"
                ).add_to(fmap)

                # ── ③ 중심선 & 마커 시각화 ─────────────
                folium.PolyLine(coords, color=color,
                                weight=2, dash_array="4,4").add_to(fmap)
                for lat, lon in coords:
                    folium.CircleMarker([lat, lon], radius=4,
                                        color=color, fill=True,
                                        fill_opacity=0.9).add_to(fmap)

        # 3) 이동 -> 궤적 연결선
        if mtype == 3 and info.get("coordinateList"):
            coords = [(c["latitude"], c["longitude"]) for c in info["coordinateList"]]
            if len(coords) >= 2:  # Line으로 연결된 경로여야 하므로 최소 2점 이상이어야 함
                folium.PolyLine(locations=coords,
                                color=color, weight=3,
                                dash_array="5,10",
                                tooltip=f"A/C {aid} : Route").add_to(fmap)
            # 이동에서 점 마커 추가
            for p in coords:
                folium.CircleMarker(p, radius=4, color=color,
                                     fill=True, fill_opacity=0.9).add_to(fmap)

# ────────────────── 지도 ↔ Python 브릿지 ──────────────────
class MapBridge(QObject):
    pointClicked = pyqtSignal(float, float)

    @pyqtSlot(float, float)
    def sendPoint(self, lat, lon):
        self.pointClicked.emit(lat, lon)

bridge = MapBridge()  # 단일 인스턴스 사용

# ────────────────── 임무 메타 다이얼로그 ──────────────────
class MissionMetaDialog(QDialog):
    def __init__(self, next_id: int, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Mission Info")
        self.resize(300, 150)

        lay = QGridLayout(self)
        lay.addWidget(QLabel(f"IndividualMissionID: {next_id}"), 0, 0, 1, 2)

        lay.addWidget(QLabel("Mission Type"), 1, 0)
        self.cmb = QComboBox()
        self.cmb.addItems(["Area Search (4 pts)", "Corridor (3 pts)", "Move (5 pts)"])
        lay.addWidget(self.cmb, 1, 1)

        lay.addWidget(QLabel("Width (m)"), 2, 0)
        self.spin = QDoubleSpinBox()
        self.spin.setRange(1, 1000)
        self.spin.setValue(100)
        lay.addWidget(self.spin, 2, 1)
        self.spin.setEnabled(False)

        self.cmb.currentIndexChanged.connect(
            lambda i: self.spin.setEnabled(i == 1)
        )

        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        lay.addWidget(bb, 3, 0, 1, 2)

    def get_result(self):
        typ = self.cmb.currentIndex()
        need = {0: 4, 1: 3, 2: 5}[typ]
        width = self.spin.value() if typ == 1 else None
        return typ, need, width


# mission_helpers.py  (맨 아래에 추가)
from datetime import datetime, timezone

def now_ms_since_2000() -> int:
    """2000-01-01 00:00:00 UTC 기준 경과 millisecond"""
    epoch2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
    return int((datetime.utcnow().replace(tzinfo=timezone.utc) - epoch2000).total_seconds() * 1000)
