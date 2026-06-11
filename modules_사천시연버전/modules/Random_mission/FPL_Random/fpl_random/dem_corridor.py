from __future__ import annotations

import heapq
import math
import os
import pickle
import tempfile
from collections import OrderedDict
from datetime import datetime
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from threading import Lock
from typing import Dict, List, Optional, Sequence, Tuple

from .areas import LatLon
from . import config

try:
    import numpy as np
    import rasterio
    from rasterio.enums import Resampling
    from rasterio.transform import Affine
    from rasterio.warp import transform
    from rasterio.windows import from_bounds
except Exception:
    np = None
    rasterio = None
    Resampling = None
    Affine = None
    transform = None
    from_bounds = None

try:
    import fiona
except Exception:
    fiona = None

try:
    import whitebox
except Exception:
    whitebox = None

# Patch WhiteboxTools stdout decoding to avoid cp949 decode errors on Windows.
if whitebox is not None:
    try:
        import subprocess
        import whitebox.whitebox_tools as _wbtools

        def _safe_popen(*args, **kwargs):
            kwargs.setdefault("encoding", "utf-8")
            kwargs.setdefault("errors", "replace")
            return subprocess.Popen(*args, **kwargs)

        _wbtools.Popen = _safe_popen
    except Exception:
        pass


PointXY = Tuple[float, float]

_NETWORK_CACHE: Dict[Tuple[str, int], CorridorNetwork] = {}
_NETWORK_FAIL: set[Tuple[str, int]] = set()
_CACHE_VERSION = 1
_DEM_DATASET_CACHE: Dict[str, "rasterio.DatasetReader"] = {}
_DEM_DATASET_CACHE_LOCK = Lock()
_DEM_CELL_CACHE: OrderedDict[Tuple[str, int, int], Optional[float]] = OrderedDict()
_DEM_CELL_CACHE_LOCK = Lock()
_DEM_CELL_CACHE_MAX = 200_000
_CACHE_MISS = object()


def _log(message: str) -> None:
    if not getattr(config, "DEM_CORRIDOR_LOG_ENABLE", False):
        return
    stamp = datetime.now().strftime("%H:%M:%S")
    line = f"[{stamp}] {message}"
    print(line)
    log_path = Path(getattr(config, "DEM_CORRIDOR_LOG_FILE", "database/corridor_cache/corridor.log"))
    try:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        with log_path.open("a", encoding="utf-8") as fh:
            fh.write(line + "\n")
    except Exception:
        return


def _cache_path(dem_path: Path, threshold: int) -> Path:
    cache_dir = Path(getattr(config, "DEM_CORRIDOR_CACHE_DIR", "database/corridor_cache"))
    cache_dir.mkdir(parents=True, exist_ok=True)
    stem = dem_path.stem.replace(" ", "_")
    return cache_dir / f"corridor_{stem}_th{threshold}.pkl"


def _get_dem_dataset(dem_path: Path):
    if rasterio is None:
        return None
    key = str(Path(dem_path).resolve())
    with _DEM_DATASET_CACHE_LOCK:
        ds = _DEM_DATASET_CACHE.get(key)
        if ds is not None and not getattr(ds, "closed", False):
            return ds
        ds = rasterio.open(key)
        _DEM_DATASET_CACHE[key] = ds
        return ds


def _dem_cache_get(key: Tuple[str, int, int]) -> object:
    with _DEM_CELL_CACHE_LOCK:
        if key not in _DEM_CELL_CACHE:
            return _CACHE_MISS
        value = _DEM_CELL_CACHE.pop(key)
        _DEM_CELL_CACHE[key] = value
        return value


def _dem_cache_set(key: Tuple[str, int, int], value: Optional[float]) -> None:
    with _DEM_CELL_CACHE_LOCK:
        if key in _DEM_CELL_CACHE:
            _DEM_CELL_CACHE.pop(key)
        _DEM_CELL_CACHE[key] = value
        while len(_DEM_CELL_CACHE) > _DEM_CELL_CACHE_MAX:
            _DEM_CELL_CACHE.popitem(last=False)


def _sample_cached_dem_value(ds, dem_key: str, x: float, y: float) -> Optional[float]:
    try:
        row, col = ds.index(x, y)
    except Exception:
        return None
    if row < 0 or col < 0 or row >= ds.height or col >= ds.width:
        return None
    cache_key = (dem_key, int(row), int(col))
    cached = _dem_cache_get(cache_key)
    if cached is not _CACHE_MISS:
        return cached
    value = next(ds.sample([(x, y)]))[0]
    if hasattr(value, "mask") and getattr(value, "mask"):
        _dem_cache_set(cache_key, None)
        return None
    if ds.nodata is not None and float(value) == float(ds.nodata):
        _dem_cache_set(cache_key, None)
        return None
    parsed = float(value)
    _dem_cache_set(cache_key, parsed)
    return parsed


def _load_network_cache(dem_path: Path, threshold: int) -> Optional[CorridorNetwork]:
    if not getattr(config, "DEM_CORRIDOR_CACHE_ENABLE", True):
        return None
    path = _cache_path(dem_path, threshold)
    if not path.exists():
        return None
    try:
        with path.open("rb") as fh:
            payload = pickle.load(fh)
    except Exception:
        _log("[DEM] cache load failed")
        return None
    if not isinstance(payload, dict) or payload.get("version") != _CACHE_VERSION:
        return None
    try:
        stat = dem_path.stat()
    except Exception:
        return None
    if payload.get("dem_mtime") != stat.st_mtime_ns or payload.get("dem_size") != stat.st_size:
        return None
    if payload.get("threshold") != threshold:
        return None
    crs_wkt = payload.get("crs_wkt")
    nodes = payload.get("nodes") or []
    edges = payload.get("edges") or []
    if not crs_wkt or not nodes or not edges:
        return None
    graph: Dict[PointXY, Dict[PointXY, float]] = defaultdict(dict)
    for a, b, dist in edges:
        a_t = tuple(a)
        b_t = tuple(b)
        graph[a_t][b_t] = float(dist)
        graph[b_t][a_t] = float(dist)
    _log(f"[DEM] cache hit {path} nodes={len(nodes)} edges={len(edges)}")
    return CorridorNetwork(crs_wkt=str(crs_wkt), graph=graph, nodes=[tuple(n) for n in nodes])


def _save_network_cache(network: CorridorNetwork, dem_path: Path, threshold: int) -> None:
    if not getattr(config, "DEM_CORRIDOR_CACHE_ENABLE", True):
        return
    try:
        stat = dem_path.stat()
    except Exception:
        return
    edges = []
    seen = set()
    for a, neighbors in network.graph.items():
        for b, dist in neighbors.items():
            key = (a, b) if a <= b else (b, a)
            if key in seen:
                continue
            seen.add(key)
            edges.append((a, b, float(dist)))
    payload = {
        "version": _CACHE_VERSION,
        "dem_mtime": stat.st_mtime_ns,
        "dem_size": stat.st_size,
        "threshold": threshold,
        "crs_wkt": network.crs_wkt,
        "nodes": list(network.nodes),
        "edges": edges,
    }
    try:
        cache_path = _cache_path(dem_path, threshold)
        with cache_path.open("wb") as fh:
            pickle.dump(payload, fh, protocol=pickle.HIGHEST_PROTOCOL)
        _log(f"[DEM] cache saved {cache_path}")
    except Exception:
        _log("[DEM] cache save failed")
        return


@dataclass(frozen=True)
class CorridorLine:
    points: List[LatLon]
    length_m: float


@dataclass(frozen=True)
class CorridorNetwork:
    crs_wkt: str
    graph: Dict[PointXY, Dict[PointXY, float]]
    nodes: List[PointXY]


def _utm_distance(a: PointXY, b: PointXY) -> float:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    return (dx * dx + dy * dy) ** 0.5


def _nearest_point_on_polyline(point: PointXY, line: Sequence[PointXY]) -> Tuple[float, int, float, PointXY]:
    best_dist = float("inf")
    best_index = 0
    best_t = 0.0
    best_point = line[0]
    px, py = point
    for i in range(len(line) - 1):
        ax, ay = line[i]
        bx, by = line[i + 1]
        abx = bx - ax
        aby = by - ay
        ab2 = abx * abx + aby * aby
        if ab2 <= 0.0:
            t = 0.0
            proj = (ax, ay)
        else:
            t = ((px - ax) * abx + (py - ay) * aby) / ab2
            t = max(0.0, min(1.0, t))
            proj = (ax + abx * t, ay + aby * t)
        dist = _utm_distance(point, proj)
        if dist < best_dist:
            best_dist = dist
            best_index = i
            best_t = t
            best_point = proj
    return best_dist, best_index, best_t, best_point


def _polyline_length(line: Sequence[PointXY]) -> float:
    if len(line) < 2:
        return 0.0
    return sum(_utm_distance(line[i], line[i + 1]) for i in range(len(line) - 1))


def _extract_lines_from_feature(geom: dict) -> List[List[PointXY]]:
    if not geom:
        return []
    gtype = geom.get("type")
    coords = geom.get("coordinates")
    if not coords:
        return []
    if gtype == "LineString":
        return [coords]
    if gtype == "MultiLineString":
        return list(coords)
    return []


def _build_graph(lines: Sequence[Sequence[PointXY]]) -> Tuple[Dict[PointXY, Dict[PointXY, float]], List[PointXY]]:
    graph: Dict[PointXY, Dict[PointXY, float]] = defaultdict(dict)
    nodes: List[PointXY] = []
    seen = set()
    for line in lines:
        if len(line) < 2:
            continue
        for i in range(len(line) - 1):
            a = tuple(line[i])
            b = tuple(line[i + 1])
            dist = _utm_distance(a, b)
            if dist <= 0.0:
                continue
            prev = graph[a].get(b)
            if prev is None or dist < prev:
                graph[a][b] = dist
                graph[b][a] = dist
            if a not in seen:
                seen.add(a)
                nodes.append(a)
            if b not in seen:
                seen.add(b)
                nodes.append(b)
    return graph, nodes


def _closest_node(nodes: Sequence[PointXY], target: PointXY) -> Optional[PointXY]:
    if not nodes:
        return None
    best = nodes[0]
    best_dist = _utm_distance(best, target)
    for node in nodes[1:]:
        dist = _utm_distance(node, target)
        if dist < best_dist:
            best = node
            best_dist = dist
    return best


def _dijkstra_path(
    graph: Dict[PointXY, Dict[PointXY, float]],
    start: PointXY,
    goal: PointXY,
) -> List[PointXY]:
    if start not in graph or goal not in graph:
        return []
    distances: Dict[PointXY, float] = {start: 0.0}
    previous: Dict[PointXY, Optional[PointXY]] = {start: None}
    heap: List[Tuple[float, PointXY]] = [(0.0, start)]
    while heap:
        dist, node = heapq.heappop(heap)
        if node == goal:
            break
        if dist != distances.get(node, float("inf")):
            continue
        for neighbor, weight in graph.get(node, {}).items():
            cand = dist + weight
            if cand < distances.get(neighbor, float("inf")):
                distances[neighbor] = cand
                previous[neighbor] = node
                heapq.heappush(heap, (cand, neighbor))
    if goal not in distances:
        return []
    path: List[PointXY] = []
    node = goal
    while node is not None:
        path.append(node)
        node = previous.get(node)
    path.reverse()
    return path


def _connected_component(
    graph: Dict[PointXY, Dict[PointXY, float]],
    start: PointXY,
) -> List[PointXY]:
    if start not in graph:
        return []
    seen = {start}
    stack = [start]
    while stack:
        node = stack.pop()
        for neighbor in graph.get(node, {}):
            if neighbor in seen:
                continue
            seen.add(neighbor)
            stack.append(neighbor)
    return list(seen)


def _build_stream_centerlines(dem_path: Path, threshold: int) -> Optional[Tuple[List[List[PointXY]], str]]:
    if rasterio is None or whitebox is None or fiona is None:
        return None
    if not dem_path.exists():
        return None
    dem_path = dem_path.resolve()

    # Prevent WhiteboxTools from downloading testdata (which can fail on CP949).
    try:
        pkg_dir = Path(whitebox.__file__).resolve().parent
        testdata_dir = pkg_dir / "testdata"
        testdata_dir.mkdir(parents=True, exist_ok=True)
    except Exception:
        pass

    if not os.environ.get("WBT_PATH"):
        exe_guess = Path(whitebox.__file__).resolve().parent / "whitebox_tools.exe"
        if exe_guess.exists():
            os.environ["WBT_PATH"] = str(exe_guess)
            _log(f"[DEM] WBT_PATH set to {exe_guess}")
        else:
            exe_guess = Path(whitebox.__file__).resolve().parent / "WBT" / "whitebox_tools.exe"
            if exe_guess.exists():
                os.environ["WBT_PATH"] = str(exe_guess)
                _log(f"[DEM] WBT_PATH set to {exe_guess}")

    with rasterio.open(dem_path) as ds:
        dem_crs = ds.crs
        if dem_crs is None:
            return None
        crs_wkt = dem_crs.to_wkt()

    with tempfile.TemporaryDirectory() as tmpdir:
        tmpdir = Path(tmpdir)
        filled = tmpdir / "filled.tif"
        flow_acc = tmpdir / "flow_acc.tif"
        d8_ptr = tmpdir / "d8_ptr.tif"
        streams_rast = tmpdir / "streams.tif"
        streams_vec = tmpdir / "streams.shp"

        wbt = whitebox.WhiteboxTools()
        wbt.verbose = False
        try:
            wbt.set_default_callback(lambda *_: None)
        except Exception:
            pass
        wbt.work_dir = str(tmpdir)
        try:
            wbt.set_working_dir(str(tmpdir))
        except Exception:
            pass

        def _run_tool(tool: str, args: List[str]) -> bool:
            try:
                return wbt.run_tool(tool, args) == 0
            except Exception as exc:
                _log(f"[DEM] {tool} exception: {exc}")
                return False

        def _ensure_file(path: Path, label: str) -> bool:
            try:
                if not path.exists():
                    _log(f"[DEM] {label} missing: {path}")
                    return False
                if path.stat().st_size <= 0:
                    _log(f"[DEM] {label} empty: {path}")
                    return False
            except Exception as exc:
                _log(f"[DEM] {label} stat failed: {exc}")
                return False
            return True

        def _log_tmpdir() -> None:
            try:
                entries = sorted(p.name for p in tmpdir.glob("*"))
                if len(entries) > 20:
                    entries = entries[:20] + ["..."]
                _log(f"[DEM] tmpdir contents: {entries}")
            except Exception:
                return

        try:
            if not _run_tool(
                "fill_depressions",
                [
                    f'--dem="{dem_path}"',
                    f'--output="{filled}"',
                    "--fix_flats",
                ],
            ):
                _log("[DEM] fill_depressions failed")
                return None
            if not _ensure_file(filled, "filled"):
                _log_tmpdir()
                return None
            if not _run_tool(
                "d8_pointer",
                [
                    f'--dem="{filled}"',
                    f'--output="{d8_ptr}"',
                ],
            ):
                _log("[DEM] d8_pointer failed")
                return None
            if not _ensure_file(d8_ptr, "d8_ptr"):
                _log_tmpdir()
                return None
            if not _run_tool(
                "d8_flow_accumulation",
                [
                    f'--input="{filled}"',
                    f'--output="{flow_acc}"',
                    "--out_type=cells",
                ],
            ):
                _log("[DEM] d8_flow_accumulation failed")
                return None
            if not _ensure_file(flow_acc, "flow_acc"):
                _log_tmpdir()
                return None
            if not _run_tool(
                "extract_streams",
                [
                    f'--flow_accum="{flow_acc}"',
                    f'--output="{streams_rast}"',
                    f'--threshold="{threshold}"',
                ],
            ):
                _log("[DEM] extract_streams failed")
                return None
            if not _ensure_file(streams_rast, "streams_rast"):
                _log_tmpdir()
                return None
            if not _run_tool(
                "raster_streams_to_vector",
                [
                    f'--streams="{streams_rast}"',
                    f'--d8_pntr="{d8_ptr}"',
                    f'--output="{streams_vec}"',
                ],
            ):
                _log("[DEM] raster_streams_to_vector failed")
                return None
            if not streams_vec.exists():
                _log("[DEM] streams_vec missing after raster_streams_to_vector")
                _log_tmpdir()
                return None
        except Exception as exc:
            _log(f"[DEM] whitebox exception: {exc}")
            return None

        lines: List[List[PointXY]] = []
        try:
            with fiona.open(streams_vec, "r") as shp:
                for feature in shp:
                    lines.extend(_extract_lines_from_feature(feature.get("geometry") or {}))
        except Exception as exc:
            _log(f"[DEM] fiona read failed: {exc}")
            _log_tmpdir()
            return None

    if not lines:
        return None
    return lines, crs_wkt


def _pick_best_line(
    lines: Sequence[Sequence[PointXY]],
    start_xy: PointXY,
    min_length_m: float,
) -> Optional[List[PointXY]]:
    best_line = None
    best_score = float("inf")
    for line in lines:
        if len(line) < 2:
            continue
        length = _polyline_length(line)
        if length < min_length_m:
            continue
        dist, _, _, _ = _nearest_point_on_polyline(start_xy, line)
        if dist < best_score:
            best_score = dist
            best_line = list(line)
    if best_line is None:
        longest = max(lines, key=_polyline_length, default=None)
        if longest and _polyline_length(longest) >= min_length_m:
            best_line = list(longest)
    return best_line


def _orient_line_from_start(line: List[PointXY], start_xy: PointXY) -> List[PointXY]:
    if len(line) < 2:
        return line
    _, idx, t, proj = _nearest_point_on_polyline(start_xy, line)
    insert = (proj,)
    if t <= 0.0:
        line_with = line
        proj_idx = idx
    elif t >= 1.0:
        line_with = line
        proj_idx = idx + 1
    else:
        line_with = line[: idx + 1] + list(insert) + line[idx + 1 :]
        proj_idx = idx + 1

    forward = line_with[proj_idx:]
    backward = list(reversed(line_with[: proj_idx + 1]))
    if _polyline_length(forward) >= _polyline_length(backward):
        return forward
    return backward


def _resample_polyline(points: Sequence[PointXY], sample_count: int) -> List[PointXY]:
    if sample_count <= 0 or len(points) < 2:
        return list(points)
    total = _polyline_length(points)
    if total <= 0:
        return list(points)
    step = total / max(1, sample_count - 1)
    out: List[PointXY] = []
    seg_idx = 0
    seg_start = points[0]
    seg_end = points[1]
    seg_len = _utm_distance(seg_start, seg_end)
    dist_along = 0.0
    for i in range(sample_count):
        target = step * i
        while dist_along + seg_len < target and seg_idx < len(points) - 2:
            dist_along += seg_len
            seg_idx += 1
            seg_start = points[seg_idx]
            seg_end = points[seg_idx + 1]
            seg_len = _utm_distance(seg_start, seg_end)
        if seg_len <= 0:
            out.append(seg_start)
            continue
        t = (target - dist_along) / seg_len
        t = max(0.0, min(1.0, t))
        x = seg_start[0] + (seg_end[0] - seg_start[0]) * t
        y = seg_start[1] + (seg_end[1] - seg_start[1]) * t
        out.append((x, y))
    return out


def _write_debug_outputs(
    dem_path: Path,
    utm_points: Sequence[PointXY],
    out_png: Optional[Path],
    out_csv: Optional[Path],
    sample_count: int,
) -> None:
    if rasterio is None or not utm_points:
        return
    out_dir = None
    if out_png:
        out_dir = out_png.parent
    if out_csv:
        out_dir = out_csv.parent if out_dir is None else out_dir
    if out_dir:
        out_dir.mkdir(parents=True, exist_ok=True)

    with rasterio.open(dem_path) as ds:
        samples = _resample_polyline(utm_points, sample_count)
        elev = [val[0] for val in ds.sample(samples)]
        dists: List[float] = [0.0]
        for i in range(1, len(samples)):
            dists.append(dists[-1] + _utm_distance(samples[i - 1], samples[i]))

        if out_csv:
            with out_csv.open("w", encoding="utf-8") as fh:
                fh.write("distance_m,elevation\n")
                for dist, z in zip(dists, elev):
                    fh.write(f"{dist:.2f},{z}\n")

        if out_png:
            try:
                import numpy as np
                import matplotlib.pyplot as plt
                from rasterio.windows import from_bounds
            except Exception:
                return
            xs = [p[0] for p in utm_points]
            ys = [p[1] for p in utm_points]
            minx, maxx = min(xs), max(xs)
            miny, maxy = min(ys), max(ys)
            pad_x = (maxx - minx) * 0.1 if maxx > minx else 500.0
            pad_y = (maxy - miny) * 0.1 if maxy > miny else 500.0
            window = from_bounds(minx - pad_x, miny - pad_y, maxx + pad_x, maxy + pad_y, ds.transform)
            out_h = 800
            out_w = 800
            data = ds.read(1, window=window, out_shape=(out_h, out_w))
            if ds.nodata is not None:
                data = np.where(data == ds.nodata, np.nan, data)
            left, bottom, right, top = rasterio.windows.bounds(window, ds.transform)
            fig, ax = plt.subplots(1, 1, figsize=(7, 7))
            ax.imshow(
                data,
                cmap="terrain",
                extent=(left, right, bottom, top),
                origin="upper",
                alpha=0.85,
            )
            ax.plot(xs, ys, color="#0a63ff", linewidth=2.0)
            ax.set_title("DEM Corridor Preview")
            ax.set_xlabel("Easting (m)")
            ax.set_ylabel("Northing (m)")
            fig.tight_layout()
            fig.savefig(out_png, dpi=140)
            plt.close(fig)


def build_corridor_line(
    start: LatLon,
    *,
    dem_path: Optional[Path] = None,
    flow_threshold: Optional[int] = None,
    min_length_m: Optional[float] = None,
) -> Optional[CorridorLine]:
    if rasterio is None or transform is None or whitebox is None or fiona is None:
        return None

    dem_path = Path(dem_path or config.DEM_CORRIDOR_FILE)
    threshold = int(flow_threshold or config.DEM_CORRIDOR_FLOW_ACC_THRESHOLD)
    min_length = float(min_length_m or config.DEM_CORRIDOR_MIN_LENGTH_M)

    built = _build_stream_centerlines(dem_path, threshold)
    if not built:
        return None
    lines, crs_wkt = built

    xs, ys = transform("EPSG:4326", crs_wkt, [start.longitude], [start.latitude])
    start_xy = (xs[0], ys[0])

    best_line = _pick_best_line(lines, start_xy, min_length)
    if not best_line:
        return None
    oriented = _orient_line_from_start(best_line, start_xy)

    xs = [p[0] for p in oriented]
    ys = [p[1] for p in oriented]
    lons, lats = transform(crs_wkt, "EPSG:4326", xs, ys)
    points = [LatLon(lat, lon) for lat, lon in zip(lats, lons)]

    return CorridorLine(points=points, length_m=_polyline_length(oriented))


def build_corridor_network(
    *,
    dem_path: Optional[Path] = None,
    flow_threshold: Optional[int] = None,
) -> Optional[CorridorNetwork]:
    if rasterio is None or transform is None:
        return None
    dem_path = Path(dem_path or config.DEM_CORRIDOR_FILE).resolve()
    threshold = int(flow_threshold or config.DEM_CORRIDOR_FLOW_ACC_THRESHOLD)
    cache_key = (str(dem_path.resolve()), threshold)
    cached = _NETWORK_CACHE.get(cache_key)
    if cached is not None:
        _log(f"[DEM] cache hit memory nodes={len(cached.nodes)}")
        return cached
    cached_disk = _load_network_cache(dem_path, threshold)
    if cached_disk is not None:
        _NETWORK_CACHE[cache_key] = cached_disk
        return cached_disk
    if cache_key in _NETWORK_FAIL:
        return None
    if whitebox is None or fiona is None:
        return None
    _log(f"[DEM] building network from DEM {dem_path} (threshold={threshold})")

    built = _build_stream_centerlines(dem_path, threshold)
    if not built:
        _log("[DEM] network build failed (centerlines)")
        _NETWORK_FAIL.add(cache_key)
        return None
    lines, crs_wkt = built
    graph, nodes = _build_graph(lines)
    if not nodes:
        _log("[DEM] network build failed (empty graph)")
        _NETWORK_FAIL.add(cache_key)
        return None
    network = CorridorNetwork(crs_wkt=crs_wkt, graph=graph, nodes=nodes)
    _NETWORK_CACHE[cache_key] = network
    _save_network_cache(network, dem_path, threshold)
    return network


def snap_to_network(
    network: CorridorNetwork,
    point: LatLon,
) -> Tuple[PointXY, Optional[PointXY]]:
    xs, ys = transform("EPSG:4326", network.crs_wkt, [point.longitude], [point.latitude])
    utm = (xs[0], ys[0])
    node = _closest_node(network.nodes, utm)
    return utm, node


def network_component(
    network: CorridorNetwork,
    start_node: PointXY,
) -> List[PointXY]:
    return _connected_component(network.graph, start_node)


def path_between_nodes(
    network: CorridorNetwork,
    start_node: PointXY,
    goal_node: PointXY,
    *,
    start_xy: Optional[PointXY] = None,
    goal_xy: Optional[PointXY] = None,
) -> List[PointXY]:
    path_nodes = _dijkstra_path(network.graph, start_node, goal_node)
    if not path_nodes:
        return []
    if start_xy and _utm_distance(start_xy, path_nodes[0]) > 1.0:
        path_nodes = [start_xy] + path_nodes
    if goal_xy and _utm_distance(goal_xy, path_nodes[-1]) > 1.0:
        path_nodes = path_nodes + [goal_xy]
    return path_nodes


def latlon_from_utm(
    network: CorridorNetwork,
    points: Sequence[PointXY],
) -> List[LatLon]:
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    lons, lats = transform(network.crs_wkt, "EPSG:4326", xs, ys)
    return [LatLon(lat, lon) for lat, lon in zip(lats, lons)]


def path_elevation_stats(
    dem_path: Path,
    points: Sequence[LatLon],
    *,
    sample_count: int = 120,
) -> Optional[Tuple[float, float]]:
    if rasterio is None or transform is None or not points:
        return None
    dem_path = Path(dem_path).resolve()
    if sample_count <= 1 or len(points) <= sample_count:
        samples = list(points)
    else:
        step = (len(points) - 1) / max(1, sample_count - 1)
        samples = [points[int(round(i * step))] for i in range(sample_count)]
    lats = [p.latitude for p in samples]
    lons = [p.longitude for p in samples]
    ds = _get_dem_dataset(dem_path)
    if ds is None:
        return None
    xs, ys = transform("EPSG:4326", ds.crs, lons, lats)
    dem_key = str(dem_path)
    elev = [_sample_cached_dem_value(ds, dem_key, x, y) for x, y in zip(xs, ys)]
    if not elev:
        return None
    values = [float(v) for v in elev if v is not None]
    if not values:
        return None
    mean_val = sum(values) / len(values)
    max_val = max(values)
    return mean_val, max_val


def _nearest_valid_cell(
    costs,
    start_rc: Tuple[int, int],
    *,
    max_radius: int = 6,
) -> Optional[Tuple[int, int]]:
    rows, cols = costs.shape
    sr, sc = start_rc
    if 0 <= sr < rows and 0 <= sc < cols and math.isfinite(float(costs[sr, sc])):
        return sr, sc
    for radius in range(1, max_radius + 1):
        rmin = max(0, sr - radius)
        rmax = min(rows - 1, sr + radius)
        cmin = max(0, sc - radius)
        cmax = min(cols - 1, sc + radius)
        for rr in range(rmin, rmax + 1):
            for cc in range(cmin, cmax + 1):
                if math.isfinite(float(costs[rr, cc])):
                    return rr, cc
    return None


def _a_star_grid(
    costs,
    start_rc: Tuple[int, int],
    goal_rc: Tuple[int, int],
) -> List[Tuple[int, int]]:
    if start_rc == goal_rc:
        return [start_rc]
    rows, cols = costs.shape
    sr, sc = start_rc
    gr, gc = goal_rc
    if not (0 <= sr < rows and 0 <= sc < cols and 0 <= gr < rows and 0 <= gc < cols):
        return []
    if not math.isfinite(float(costs[sr, sc])) or not math.isfinite(float(costs[gr, gc])):
        return []

    neighbors = (
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (-1, -1, 2 ** 0.5),
        (-1, 1, 2 ** 0.5),
        (1, -1, 2 ** 0.5),
        (1, 1, 2 ** 0.5),
    )
    def heuristic(r: int, c: int) -> float:
        return math.hypot(gr - r, gc - c)

    open_heap: List[Tuple[float, float, Tuple[int, int]]] = []
    gscore: Dict[Tuple[int, int], float] = {start_rc: 0.0}
    came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
    heapq.heappush(open_heap, (heuristic(sr, sc), 0.0, start_rc))

    while open_heap:
        _f, g, (r, c) = heapq.heappop(open_heap)
        if (r, c) == goal_rc:
            break
        if g > gscore.get((r, c), float("inf")):
            continue
        for dr, dc, step in neighbors:
            nr = r + dr
            nc = c + dc
            if nr < 0 or nr >= rows or nc < 0 or nc >= cols:
                continue
            cell_cost = float(costs[nr, nc])
            if not math.isfinite(cell_cost):
                continue
            ng = g + cell_cost * step
            if ng < gscore.get((nr, nc), float("inf")):
                gscore[(nr, nc)] = ng
                came_from[(nr, nc)] = (r, c)
                heapq.heappush(open_heap, (ng + heuristic(nr, nc), ng, (nr, nc)))

    if goal_rc not in gscore:
        return []
    path: List[Tuple[int, int]] = [goal_rc]
    node = goal_rc
    while node != start_rc:
        node = came_from.get(node)
        if node is None:
            return []
        path.append(node)
    path.reverse()
    return path


def low_terrain_path(
    start: LatLon,
    goal: LatLon,
    *,
    dem_path: Optional[Path] = None,
    grid_size: int = 160,
    buffer_m: float = 4000.0,
    elev_weight: float = 6.0,
) -> Optional[List[LatLon]]:
    if rasterio is None or transform is None or np is None or from_bounds is None or Resampling is None or Affine is None:
        return None
    dem_path = Path(dem_path or config.DEM_CORRIDOR_FILE).resolve()
    if not dem_path.exists():
        return None
    grid_size = max(32, int(grid_size))
    buffer_m = max(0.0, float(buffer_m))
    elev_weight = max(0.0, float(elev_weight))

    with rasterio.open(dem_path) as ds:
        if ds.crs is None:
            return None
        xs, ys = transform(
            "EPSG:4326",
            ds.crs,
            [start.longitude, goal.longitude],
            [start.latitude, goal.latitude],
        )
        sx, gx = xs
        sy, gy = ys
        minx = min(sx, gx) - buffer_m
        maxx = max(sx, gx) + buffer_m
        miny = min(sy, gy) - buffer_m
        maxy = max(sy, gy) + buffer_m
        window = from_bounds(minx, miny, maxx, maxy, ds.transform)
        data = ds.read(
            1,
            window=window,
            out_shape=(grid_size, grid_size),
            resampling=Resampling.bilinear,
        )
        if ds.nodata is not None:
            data = np.where(data == ds.nodata, np.nan, data)
        if not np.isfinite(data).any():
            return None

        transform_win = ds.window_transform(window)
        scale_x = window.width / data.shape[1]
        scale_y = window.height / data.shape[0]
        transform_scaled = transform_win * Affine.scale(scale_x, scale_y)
        inv = ~transform_scaled
        start_col, start_row = inv * (sx, sy)
        goal_col, goal_row = inv * (gx, gy)
        start_rc = (int(round(start_row)), int(round(start_col)))
        goal_rc = (int(round(goal_row)), int(round(goal_col)))
        rows, cols = data.shape
        if not (0 <= start_rc[0] < rows and 0 <= start_rc[1] < cols and 0 <= goal_rc[0] < rows and 0 <= goal_rc[1] < cols):
            return None

        elev_min = float(np.nanmin(data))
        elev_max = float(np.nanmax(data))
        elev_range = max(1.0, elev_max - elev_min)
        elev_norm = (data - elev_min) / elev_range
        costs = 1.0 + elev_weight * elev_norm
        costs = np.where(np.isfinite(costs), costs, np.inf)

        start_rc = _nearest_valid_cell(costs, start_rc) or start_rc
        goal_rc = _nearest_valid_cell(costs, goal_rc) or goal_rc
        path_rc = _a_star_grid(costs, start_rc, goal_rc)
        if not path_rc:
            return None

        xs_path: List[float] = []
        ys_path: List[float] = []
        for r, c in path_rc:
            x, y = transform_scaled * (c + 0.5, r + 0.5)
            xs_path.append(x)
            ys_path.append(y)
        lons, lats = transform(ds.crs, "EPSG:4326", xs_path, ys_path)
        return [LatLon(lat, lon) for lat, lon in zip(lats, lons)]
