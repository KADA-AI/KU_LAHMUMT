from __future__ import annotations

from collections import OrderedDict
from dataclasses import asdict, dataclass
import hashlib
import math
import os
import threading
from typing import Any, Callable, Iterable, Sequence


DEFAULT_MAX_ENTRIES = 4096
KEY_VERSION = "line-search-geometry-v1"

XYKey = tuple[tuple[float, float], ...]
CoordTuple = tuple[float, float, int]

_LOCK = threading.Lock()
_CACHE: OrderedDict[tuple[str, XYKey], tuple[CoordTuple, ...]] = OrderedDict()
_STATS = {
    "hits": 0,
    "misses": 0,
    "stores": 0,
    "evictions": 0,
    "skips": 0,
    "coordsReturned": 0,
}


@dataclass(frozen=True)
class LineSearchGeometryCacheResult:
    coords: list[dict[str, float]]
    hit: bool
    skipped: bool
    keyHash: str | None
    rowCount: int
    coordCount: int

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _enabled() -> bool:
    return str(os.environ.get("DSS_LINE_SEARCH_GEOMETRY_CACHE", "1")).strip().lower() not in {"0", "false", "off", "no"}


def _max_entries() -> int:
    try:
        value = int(os.environ.get("DSS_LINE_SEARCH_GEOMETRY_CACHE_MAX", str(DEFAULT_MAX_ENTRIES)))
    except Exception:
        value = DEFAULT_MAX_ENTRIES
    return max(1, int(value))


def _to_float(value: Any) -> float | None:
    try:
        if value is None:
            return None
        out = float(value)
    except Exception:
        return None
    if not math.isfinite(out):
        return None
    return out


def _rows_key(rows_xy: Iterable[Sequence[float]]) -> tuple[XYKey | None, list[tuple[float, float]]]:
    rows: list[tuple[float, float]] = []
    for row in rows_xy or []:
        if not isinstance(row, (tuple, list)) or len(row) < 2:
            return None, rows
        x = _to_float(row[0])
        y = _to_float(row[1])
        if x is None or y is None:
            return None, rows
        rows.append((float(x), float(y)))
    if not rows:
        return tuple(), rows
    return tuple(rows), rows


def _key_hash(key: tuple[str, XYKey]) -> str:
    digest = hashlib.sha256()
    digest.update(key[0].encode("utf-8"))
    for x, y in key[1]:
        digest.update(float(x).hex().encode("ascii"))
        digest.update(b",")
        digest.update(float(y).hex().encode("ascii"))
        digest.update(b";")
    return digest.hexdigest()


def _clone_coords(rows: tuple[CoordTuple, ...]) -> list[dict[str, float]]:
    return [
        {
            "latitude": float(lat),
            "longitude": float(lon),
            "altitude": int(alt),
        }
        for lat, lon, alt in rows
    ]


def _freeze_coords(coords: list[dict[str, Any]]) -> tuple[CoordTuple, ...]:
    frozen: list[CoordTuple] = []
    for coord in coords or []:
        if not isinstance(coord, dict):
            continue
        lat = _to_float(coord.get("latitude"))
        lon = _to_float(coord.get("longitude"))
        alt = _to_float(coord.get("altitude"))
        if lat is None or lon is None or alt is None:
            continue
        frozen.append((float(lat), float(lon), int(round(float(alt)))))
    return tuple(frozen)


def get_or_build_line_search_coords(
    rows_xy: Iterable[Sequence[float]],
    builder: Callable[[list[tuple[float, float]]], list[dict[str, float]]],
    *,
    namespace: str = KEY_VERSION,
) -> LineSearchGeometryCacheResult:
    rows_input = list(rows_xy or [])
    rows_key, rows_list = _rows_key(rows_input)
    if rows_key is None:
        with _LOCK:
            _STATS["skips"] += 1
        coords = builder(rows_input)  # type: ignore[arg-type]
        return LineSearchGeometryCacheResult(
            coords=list(coords),
            hit=False,
            skipped=True,
            keyHash=None,
            rowCount=0,
            coordCount=len(coords),
        )

    key = (str(namespace or KEY_VERSION), rows_key)
    key_hash = _key_hash(key)

    if not _enabled():
        with _LOCK:
            _STATS["skips"] += 1
        coords = builder(rows_list)
        return LineSearchGeometryCacheResult(
            coords=list(coords),
            hit=False,
            skipped=True,
            keyHash=key_hash,
            rowCount=len(rows_list),
            coordCount=len(coords),
        )

    with _LOCK:
        cached = _CACHE.get(key)
        if cached is not None:
            _CACHE.move_to_end(key)
            _STATS["hits"] += 1
            _STATS["coordsReturned"] += len(cached)
            return LineSearchGeometryCacheResult(
                coords=_clone_coords(cached),
                hit=True,
                skipped=False,
                keyHash=key_hash,
                rowCount=len(rows_list),
                coordCount=len(cached),
            )

    built = builder(rows_list)
    frozen = _freeze_coords(built)
    if len(frozen) != len(built):
        with _LOCK:
            _STATS["skips"] += 1
        return LineSearchGeometryCacheResult(
            coords=list(built),
            hit=False,
            skipped=True,
            keyHash=key_hash,
            rowCount=len(rows_list),
            coordCount=len(built),
        )

    with _LOCK:
        _CACHE[key] = frozen
        _CACHE.move_to_end(key)
        _STATS["misses"] += 1
        _STATS["stores"] += 1
        _STATS["coordsReturned"] += len(frozen)
        max_entries = _max_entries()
        while len(_CACHE) > max_entries:
            _CACHE.popitem(last=False)
            _STATS["evictions"] += 1
    return LineSearchGeometryCacheResult(
        coords=_clone_coords(frozen),
        hit=False,
        skipped=False,
        keyHash=key_hash,
        rowCount=len(rows_list),
        coordCount=len(frozen),
    )


def snapshot_line_search_geometry_cache_stats() -> dict[str, Any]:
    with _LOCK:
        stats = dict(_STATS)
        stats["entries"] = len(_CACHE)
        stats["enabled"] = _enabled()
        stats["maxEntries"] = _max_entries()
    return stats


def clear_line_search_geometry_cache() -> None:
    with _LOCK:
        _CACHE.clear()
        for key in list(_STATS.keys()):
            _STATS[key] = 0
