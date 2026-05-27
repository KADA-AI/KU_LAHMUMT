"""
mission_helpers.py
공통 유틸리티 · 지도-JS 브릿지 · 간단한 ‘임무 메타’ 다이얼로그
"""

import random, folium, json
import math
import os
import re
import threading
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
try:
    from .id_allocator import next_individual_mission_id, next_path_id
except Exception:
    from data_def.id_allocator import next_individual_mission_id, next_path_id  # type: ignore


_PROJECT_ROOT = Path(__file__).resolve().parents[4]
_DEM_DIR = _PROJECT_ROOT / "resource"
_DEM_TILE_RE = re.compile(r"([ns])(\d+)_([ew])(\d+)", re.IGNORECASE)
_REGIONAL_DEM_EPSG_BY_NAME = {
    "inje_10m.tif": 32652,
}
_REGIONAL_DEM_PRIORITY = (
    "Inje_10m.tif",
)
_DEM_USAGE_LOCK = threading.Lock()
_DEM_USAGE_LOGGED_KEYS = set()
_DEM_MISS_LOGGED_KEYS = set()
BoundingBox = namedtuple("BoundingBox", "left bottom right top")

_MODEL_PIXEL_SCALE_TAG = 33550
_MODEL_TIEPOINT_TAG = 33922
_GDAL_NODATA_TAG = 42113


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
        bounds_llh = _regional_dem_latlon_bounds(tif)
        if bounds_llh is not None:
            tiles.append((tif, bounds_llh))

    for tif in sorted(_DEM_DIR.glob("*.tif")):
        match = _DEM_TILE_RE.search(tif.stem)
        if not match:
            continue
        lat_sign = 1 if match.group(1).lower() == "n" else -1
        lon_sign = 1 if match.group(3).lower() == "e" else -1
        lat0 = lat_sign * int(match.group(2))
        lon0 = lon_sign * int(match.group(4))
        lat1 = lat0 + lat_sign
        lon1 = lon0 + lon_sign
        tiles.append((tif, (min(lat0, lat1), max(lat0, lat1),
                            min(lon0, lon1), max(lon0, lon1))))

    if not tiles:
        raise FileNotFoundError(f"resource/ 아래에서 사용할 GeoTIFF (*.tif)를 찾지 못했습니다: {_DEM_DIR}")
    return tuple(tiles)


@lru_cache(maxsize=1)
def _available_dem_tiles():
    return _scan_dem_tiles()


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

    # GeoTIFF tiepoints report pixel centers; convert to the upper-left corner.
    origin_x = mx - sx * (px + 0.5)
    origin_y = my + sy * (py + 0.5)
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
        _band, transform, bounds, _nodata = _load_dem_data(path)
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


@lru_cache(maxsize=None)
def _load_dem_data(path: Path):
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


def _candidate_tiles(lat: float, lon: float) -> Iterable[Path]:
    """주어진 좌표를 포함할 수 있는 타일 Path 후보 리스트."""
    for path, (lat0, lat1, lon0, lon1) in _available_dem_tiles():
        if lat0 <= lat <= lat1 and lon0 <= lon <= lon1:
            yield path


def _dem_usage_log_path() -> Path | None:
    try:
        from modules.common import db_paths
        log_dir = db_paths.get_db_subpath("DSS_Internal")
    except Exception:
        db_root = os.environ.get("KU_MISSION_DB_ROOT")
        if not db_root:
            return None
        log_dir = Path(db_root) / "DSS_Internal"

    try:
        log_dir.mkdir(parents=True, exist_ok=True)
        return log_dir / "dem_usage.jsonl"
    except Exception:
        return None


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
    key = (str(log_path), "missing")
    with _DEM_USAGE_LOCK:
        if key in _DEM_MISS_LOGGED_KEYS:
            return
        _DEM_MISS_LOGGED_KEYS.add(key)

    _append_dem_usage_event({
        "event": "dem_source_missing",
        "source": "mission_planning",
        "timestampUtc": datetime.now(timezone.utc).isoformat(),
        "sample": {
            "latitude": float(lat),
            "longitude": float(lon),
        },
    })


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
    row = int(max(0, min(max_row, round(row_f))))
    col = int(max(0, min(max_col, round(col_f))))

    value = float(band[row, col])
    if math.isnan(value):
        return 0.0
    if nodata is not None and math.isclose(value, nodata, abs_tol=1e-3):
        return 0.0
    _log_dem_usage_once(_path, lat, lon, native_x, native_y, value)
    return value


def terrain_elev(lat: float, lon: float) -> float:
    try:
        return _terrain_elev_cached(float(lat), float(lon))
    except Exception:
        return 0.0


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
    terrain_info = _cache_info_dict(_terrain_elev_cached)
    hits = int(terrain_info.get("hits", 0) or 0)
    misses = int(terrain_info.get("misses", 0) or 0)
    total = hits + misses
    terrain_info["hit_ratio"] = round(float(hits) / float(total), 6) if total else 0.0
    return {
        "terrain_elev": terrain_info,
        "load_dem_data": _cache_info_dict(_load_dem_data),
        "available_dem_tiles": _cache_info_dict(_available_dem_tiles),
    }


def clear_terrain_cache() -> None:
    for cache_func in (_terrain_elev_cached, _load_dem_data, _available_dem_tiles):
        cache_clear = getattr(cache_func, "cache_clear", None)
        if callable(cache_clear):
            try:
                cache_clear()
            except Exception:
                pass


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
    im_id = next_individual_mission_id()

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
