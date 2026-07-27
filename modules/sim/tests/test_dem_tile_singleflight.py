from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import threading

import pytest

from modules.sim.map.dem_tiles import DemTileProvider


class _CountingLock:
    """Lock that signals after a requested number of critical-section entries."""

    def __init__(self, expected_entries: int) -> None:
        self._lock = threading.Lock()
        self._expected_entries = int(expected_entries)
        self._entries = 0
        self.reached = threading.Event()

    def __enter__(self) -> "_CountingLock":
        self._lock.acquire()
        self._entries += 1
        if self._entries >= self._expected_entries:
            self.reached.set()
        return self

    def __exit__(self, _exc_type, _exc, _traceback) -> None:
        self._lock.release()


def _available_provider(tmp_path: Path, request_count: int) -> DemTileProvider:
    provider = DemTileProvider(tmp_path, cache_max_tiles=4)
    # The renderer is replaced by each test; one sentinel VRT only makes the
    # provider available without requiring a real raster fixture.
    provider._vrts = [object()]
    provider._cache_lock = _CountingLock(request_count)
    return provider


def test_concurrent_cold_requests_render_same_tile_once(monkeypatch, tmp_path: Path) -> None:
    request_count = 4
    provider = _available_provider(tmp_path, request_count)
    release_render = threading.Event()
    render_calls: list[tuple[int, int, int]] = []

    def render(z: int, x: int, y: int) -> bytes:
        render_calls.append((z, x, y))
        if not release_render.wait(timeout=2.0):
            raise TimeoutError("test did not release tile renderer")
        return b"shared-tile"

    monkeypatch.setattr(provider, "_render_tile", render)
    with ThreadPoolExecutor(max_workers=request_count) as pool:
        futures = [pool.submit(provider.get_tile, 14, 13990, 6316) for _ in range(request_count)]
        try:
            assert provider._cache_lock.reached.wait(timeout=2.0)
        finally:
            release_render.set()
        results = [future.result(timeout=2.0) for future in futures]

    assert results == [b"shared-tile"] * request_count
    assert render_calls == [(14, 13990, 6316)]
    assert provider.get_tile(14, 13990, 6316) == b"shared-tile"
    assert render_calls == [(14, 13990, 6316)]


def test_render_exception_wakes_all_waiters_and_allows_retry(monkeypatch, tmp_path: Path) -> None:
    request_count = 3
    provider = _available_provider(tmp_path, request_count)
    release_render = threading.Event()
    render_calls = 0

    def failing_render(_z: int, _x: int, _y: int) -> bytes:
        nonlocal render_calls
        render_calls += 1
        if not release_render.wait(timeout=2.0):
            raise TimeoutError("test did not release tile renderer")
        raise ValueError("synthetic DEM failure")

    monkeypatch.setattr(provider, "_render_tile", failing_render)
    with ThreadPoolExecutor(max_workers=request_count) as pool:
        futures = [pool.submit(provider.get_tile, 14, 13990, 6316) for _ in range(request_count)]
        try:
            assert provider._cache_lock.reached.wait(timeout=2.0)
        finally:
            release_render.set()
        for future in futures:
            with pytest.raises(ValueError, match="synthetic DEM failure"):
                future.result(timeout=2.0)

    assert render_calls == 1
    assert provider._tile_inflight == {}

    monkeypatch.setattr(provider, "_render_tile", lambda *_args: b"retry-ok")
    assert provider.get_tile(14, 13990, 6316) == b"retry-ok"

