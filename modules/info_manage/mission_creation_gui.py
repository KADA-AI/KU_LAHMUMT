# -*- coding: utf-8 -*-
from __future__ import annotations

import argparse
import json
import math
import os
import re
import shutil
import sys
import tempfile
import threading
import webbrowser
from datetime import datetime, timezone
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse

PROJECT_ROOT = Path(__file__).resolve().parents[2]
for _path in (PROJECT_ROOT, PROJECT_ROOT / "modules", PROJECT_ROOT / "modules" / "common"):
    _path_text = str(_path)
    if _path.exists() and _path_text not in sys.path:
        sys.path.insert(0, _path_text)

from modules.common import db_paths  # noqa: E402
from modules.common.regional_dem import regional_dem_paths  # noqa: E402
from modules.info_manage.auto_mission_generator import (  # noqa: E402
    generate_random_mission_state,
    reference_path_for_package,
    scenario_label_for_package,
)
from modules.sim.config import (  # noqa: E402
    DEFAULT_CENTER_LAT,
    DEFAULT_CENTER_LON,
    DEFAULT_START_ZOOM,
    RESOURCE_DIR,
    WEB_DIR,
)
from modules.sim.map.mbtiles import MBTiles  # noqa: E402

EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
AIRCRAFT_SEQUENCE = (4, 5, 6)
DEFAULT_PORT = int(os.getenv("MISSION_CREATION_PORT", "8211") or 8211)
DEFAULT_HOST = os.getenv("MISSION_CREATION_HOST", "127.0.0.1") or "127.0.0.1"
_AUTO_SAVE_LOCK = threading.RLock()

PACKAGE_TYPES = {
    1: "대기갑항공타격작전",
    2: "지상작전부대 기동여건 보장 작전",
    3: "공중강습작전부대 엄호 작전",
    4: "항공지원작전-중요시설 방호",
    5: "도시지역 작전",
}

REGION_TYPES = {
    0: "지정되지 않음",
    1: "전술집결지",
    2: "통제권변경지역",
    3: "ACP",
    4: "공격대기지역",
    5: "전투진지",
    6: "목표지역",
    7: "경계지역",
    8: "탑재지대",
    9: "착륙지대",
    10: "중요시설",
    11: "도서지역",
}

MISSION_TYPES = {
    0: "Not used",
    1: "협업기동임무",
    2: "협업수색공격임무",
    3: "협업경계임무",
    4: "협업공중부대엄호임무",
    5: "협업지상부대엄호임무",
    6: "협업도심수색공격임무",
    7: "편대비행모드",
}


def _now_ms_2000() -> int:
    return int((datetime.now(timezone.utc) - EPOCH_2000).total_seconds() * 1000)


def _as_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except Exception:
        return int(default)


def _as_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return float(default)


def _point_lat(point: dict[str, Any]) -> float:
    return _as_float(point.get("latitude", point.get("lat")))


def _point_lon(point: dict[str, Any]) -> float:
    return _as_float(point.get("longitude", point.get("lon", point.get("lng"))))


def _coordinate(point: dict[str, Any], altitude: int) -> dict[str, Any]:
    return {
        "latitude": round(_point_lat(point), 8),
        "longitude": round(_point_lon(point), 8),
        "altitude": max(0, min(50000, int(round(altitude)))),
    }


def _area_latlon(point: dict[str, Any]) -> dict[str, float]:
    return {
        "latitude": round(_point_lat(point), 8),
        "longitude": round(_point_lon(point), 8),
    }


def _area_self_intersects(points: list[dict[str, Any]]) -> bool:
    if len(points) < 4:
        return False
    center_lat = sum(_point_lat(point) for point in points) / len(points)
    origin = (_point_lon(points[0]), _point_lat(points[0]))
    cos_lat = math.cos(math.radians(center_lat)) or 1.0

    def xy(point: dict[str, Any]) -> tuple[float, float]:
        return ((_point_lon(point) - origin[0]) * cos_lat, _point_lat(point) - origin[1])

    coords = [xy(point) for point in points]

    def orient(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]) -> float:
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    def crosses(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float], d: tuple[float, float]) -> bool:
        eps = 1e-12
        return (orient(a, b, c) * orient(a, b, d) < -eps) and (orient(c, d, a) * orient(c, d, b) < -eps)

    edge_count = len(coords)
    for left_idx in range(edge_count):
        left_next = (left_idx + 1) % edge_count
        for right_idx in range(left_idx + 1, edge_count):
            right_next = (right_idx + 1) % edge_count
            if left_idx == right_idx or left_next == right_idx or right_next == left_idx:
                continue
            if left_idx == 0 and right_next == 0:
                continue
            if crosses(coords[left_idx], coords[left_next], coords[right_idx], coords[right_next]):
                return True
    return False


def _normalize_area_points(points: Any) -> list[dict[str, Any]]:
    normalized = [point for point in (points or []) if isinstance(point, dict)]
    if len(normalized) < 3 or not _area_self_intersects(normalized):
        return normalized
    center_lat = sum(_point_lat(point) for point in normalized) / len(normalized)
    center_lon = sum(_point_lon(point) for point in normalized) / len(normalized)
    cos_lat = math.cos(math.radians(center_lat)) or 1.0
    return sorted(
        normalized,
        key=lambda point: math.atan2(
            _point_lat(point) - center_lat,
            (_point_lon(point) - center_lon) * cos_lat,
        ),
    )


def _json_response(handler: BaseHTTPRequestHandler, status: HTTPStatus | int, payload: Any) -> None:
    body = json.dumps(payload, ensure_ascii=False, indent=2).encode("utf-8")
    handler.send_response(int(status))
    handler.send_header("Content-Type", "application/json; charset=utf-8")
    handler.send_header("Cache-Control", "no-store")
    handler.send_header("Access-Control-Allow-Origin", "*")
    handler.send_header("Content-Length", str(len(body)))
    handler.end_headers()
    handler.wfile.write(body)


def _read_json_body(handler: BaseHTTPRequestHandler) -> dict[str, Any]:
    length = int(handler.headers.get("Content-Length") or "0")
    raw = handler.rfile.read(length) if length > 0 else b"{}"
    try:
        data = json.loads(raw.decode("utf-8"))
    except Exception as exc:
        raise ValueError(f"invalid JSON: {exc}") from exc
    if not isinstance(data, dict):
        raise ValueError("JSON body must be an object")
    return data


class DemSampler:
    def __init__(self, tif_paths: list[Path]) -> None:
        self._datasets: list[Any] = []
        self._transform = None
        try:
            import rasterio
            from rasterio.warp import transform

            self._transform = transform
            for path in tif_paths:
                if path.exists():
                    self._datasets.append(rasterio.open(path))
        except Exception:
            self._datasets = []
            self._transform = None

    @property
    def available(self) -> bool:
        return bool(self._datasets and self._transform)

    def close(self) -> None:
        for ds in self._datasets:
            try:
                ds.close()
            except Exception:
                pass
        self._datasets.clear()

    def sample_ground_m(self, lat: float, lon: float) -> int | None:
        if not self.available:
            return None
        for ds in self._datasets:
            try:
                if str(ds.crs).upper() == "EPSG:4326":
                    x, y = float(lon), float(lat)
                else:
                    xs, ys = self._transform("EPSG:4326", ds.crs, [float(lon)], [float(lat)])
                    x, y = float(xs[0]), float(ys[0])
                bounds = ds.bounds
                if not (bounds.left <= x <= bounds.right and bounds.bottom <= y <= bounds.top):
                    continue
                value = next(ds.sample([(x, y)]))[0]
                nodata = ds.nodata
                if nodata is not None and float(value) == float(nodata):
                    continue
                if not math.isfinite(float(value)):
                    continue
                return int(max(0, round(float(value))))
            except Exception:
                continue
        return None


def load_mission_region_boxes(tif_paths: list[Path]) -> list[dict[str, Any]]:
    regions: list[dict[str, Any]] = []
    try:
        import rasterio
        from rasterio.warp import transform_bounds
    except Exception:
        return regions
    for path in tif_paths:
        if not path.exists():
            continue
        try:
            with rasterio.open(path) as ds:
                west, south, east, north = transform_bounds(ds.crs, "EPSG:4326", *ds.bounds, densify_pts=21)
        except Exception:
            continue
        regions.append(
            {
                "label": path.stem,
                "bounds": [float(west), float(south), float(east), float(north)],
            }
        )
    return regions


class MissionCreationServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address: tuple[str, int]):
        mission_tifs = list(regional_dem_paths(RESOURCE_DIR))
        self.mbtiles = MBTiles(RESOURCE_DIR / "korea.mbtiles")
        self.dem = DemSampler(mission_tifs)
        self.mission_regions = load_mission_region_boxes(mission_tifs)
        super().__init__(address, MissionCreationHandler)

    @property
    def base_url(self) -> str:
        host, port = self.server_address
        browse_host = "127.0.0.1" if str(host) in {"0.0.0.0", "::"} else str(host)
        return f"http://{browse_host}:{port}/"

    def close(self) -> None:
        try:
            self.mbtiles.close()
        except Exception:
            pass
        try:
            self.dem.close()
        except Exception:
            pass


class MissionCreationHandler(BaseHTTPRequestHandler):
    server: MissionCreationServer

    def do_OPTIONS(self) -> None:
        self.send_response(HTTPStatus.NO_CONTENT)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path in ("/", "/index.html", "/mission_creator.html"):
            self._serve_index()
            return
        if path == "/api/config":
            self._serve_config()
            return
        if path == "/api/elevation":
            self._serve_elevation()
            return
        if path == "/api/scenarios":
            self._serve_scenarios()
            return
        if path.startswith("/tiles/"):
            self._serve_tile(path)
            return
        if path.startswith("/vendor/"):
            self._serve_vendor(path)
            return
        self._send_text(HTTPStatus.NOT_FOUND, "not found")

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        if path == "/api/save":
            self._handle_save()
            return
        if path == "/api/auto_generate":
            self._handle_auto_generate()
            return
        if path == "/api/load_scenario":
            self._handle_load_scenario()
            return
        self._send_text(HTTPStatus.NOT_FOUND, "not found")

    def log_message(self, fmt: str, *args: object) -> None:
        return

    def _serve_config(self) -> None:
        info = self.server.mbtiles.info
        _json_response(
            self,
            HTTPStatus.OK,
            {
                "ok": True,
                "tileUrl": f"{self.server.base_url}tiles/{{z}}/{{x}}/{{y}}.pbf",
                "minZoom": info.min_zoom,
                "maxZoom": info.max_zoom,
                "center": [DEFAULT_CENTER_LON, DEFAULT_CENTER_LAT],
                "zoom": DEFAULT_START_ZOOM,
                "demAvailable": self.server.dem.available,
                "dbRoot": str(db_paths.get_active_db_root()),
                "packageTypes": PACKAGE_TYPES,
                "regionTypes": REGION_TYPES,
                "missionTypes": MISSION_TYPES,
                "missionRegions": self.server.mission_regions,
            },
        )

    def _serve_index(self) -> None:
        info = self.server.mbtiles.info
        html = _build_html(
            tile_url=f"{self.server.base_url}tiles/{{z}}/{{x}}/{{y}}.pbf",
            min_zoom=info.min_zoom,
            max_zoom=info.max_zoom,
            center_lat=DEFAULT_CENTER_LAT,
            center_lon=DEFAULT_CENTER_LON,
            start_zoom=DEFAULT_START_ZOOM,
            dem_available=self.server.dem.available,
            mission_regions=self.server.mission_regions,
        )
        self._send_bytes(HTTPStatus.OK, html.encode("utf-8"), "text/html; charset=utf-8")

    def _serve_elevation(self) -> None:
        query = parse_qs(urlparse(self.path).query or "")
        lat = _as_float((query.get("lat") or ["0"])[0])
        lon = _as_float((query.get("lon") or ["0"])[0])
        ground = self.server.dem.sample_ground_m(lat, lon)
        _json_response(self, HTTPStatus.OK, {"ok": True, "ground": ground, "demAvailable": self.server.dem.available})

    def _serve_scenarios(self) -> None:
        try:
            scenarios = list_saved_scenarios()
        except Exception as exc:
            _json_response(self, HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc), "scenarios": []})
            return
        _json_response(self, HTTPStatus.OK, {"ok": True, "scenarios": scenarios})

    def _handle_save(self) -> None:
        try:
            body = _read_json_body(self)
            if isinstance(body.get("state"), dict):
                state = body["state"]
                save_name = body.get("saveName")
            else:
                state = body
                save_name = body.get("saveName")
            result = save_state(state, self.server.dem, save_name)
        except Exception as exc:
            _json_response(self, HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc)})
            return
        _json_response(self, HTTPStatus.OK, result)

    def _handle_auto_generate(self) -> None:
        try:
            body = _read_json_body(self)
            package_type = _as_int(body.get("packageType"), 1)
            seed_value = body.get("seed")
            if seed_value in (None, ""):
                seed = None
            else:
                try:
                    seed = int(seed_value)
                except (TypeError, ValueError) as exc:
                    raise ValueError("Seed는 정수여야 합니다.") from exc
            generated = generate_random_mission_state(package_type, seed=seed)
            state = generated["state"]
            result = save_random_state(
                state,
                self.server.dem,
                scenario_label=generated["metadata"]["scenarioLabel"],
            )
            result["state"] = state
            result["seed"] = generated["metadata"]["seed"]
            result["generation"] = generated["metadata"]
        except Exception as exc:
            _json_response(self, HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc)})
            return
        _json_response(self, HTTPStatus.OK, result)

    def _handle_load_scenario(self) -> None:
        try:
            body = _read_json_body(self)
            save_name = body.get("saveName") or body.get("name")
            result = load_saved_scenario(save_name, self.server.dem)
        except Exception as exc:
            _json_response(self, HTTPStatus.BAD_REQUEST, {"ok": False, "error": str(exc)})
            return
        _json_response(self, HTTPStatus.OK, result)

    def _serve_tile(self, path: str) -> None:
        parts = path.strip("/").split("/")
        if len(parts) != 4:
            self._send_text(HTTPStatus.NOT_FOUND, "invalid tile path")
            return
        try:
            z = int(parts[1])
            x = int(parts[2])
            y = int(Path(parts[3]).stem)
        except Exception:
            self._send_text(HTTPStatus.BAD_REQUEST, "invalid tile coordinate")
            return
        data = self.server.mbtiles.get_tile(z, x, y)
        if not data:
            self._send_text(HTTPStatus.NOT_FOUND, "tile not found")
            return
        fmt = (self.server.mbtiles.info.tile_format or "pbf").lower()
        content_type = "application/vnd.mapbox-vector-tile" if fmt == "pbf" else f"image/{fmt}"
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Access-Control-Allow-Origin", "*")
        if fmt == "pbf" and data[:2] == b"\x1f\x8b":
            self.send_header("Content-Encoding", "gzip")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _serve_vendor(self, path: str) -> None:
        target = (WEB_DIR / path.lstrip("/")).resolve()
        vendor_root = (WEB_DIR / "vendor").resolve()
        try:
            target.relative_to(vendor_root)
        except ValueError:
            self._send_text(HTTPStatus.FORBIDDEN, "forbidden")
            return
        if not target.exists() or not target.is_file():
            self._send_text(HTTPStatus.NOT_FOUND, "missing vendor file")
            return
        ctype = "text/css; charset=utf-8" if target.suffix == ".css" else "application/javascript; charset=utf-8"
        self._send_bytes(HTTPStatus.OK, target.read_bytes(), ctype)

    def _send_text(self, status: HTTPStatus, text: str) -> None:
        self._send_bytes(status, text.encode("utf-8"), "text/plain; charset=utf-8")

    def _send_bytes(self, status: HTTPStatus, data: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def _next_package_id() -> int:
    ids: list[int] = []
    for name in ("InputMissionPlan", "MissionReferenceInfo"):
        directory = db_paths.ensure_db_payload(name)
        for path in directory.glob("*.json"):
            if path.stem.isdigit():
                ids.append(int(path.stem))
    generated_roots = [_generated_scenario_root(), _random_mission_root()]
    for root in generated_roots:
        if not root.exists():
            continue
        for path in root.glob("*/0201_*.json"):
            payload = _load_json_file(path)
            package_id = _as_int(
                payload.get(
                    "inputMissionPackageID",
                    payload.get("InputMissionPackageID"),
                ),
                0,
            )
            if package_id > 0:
                ids.append(package_id)
    return (max(ids) + 1) if ids else 1


def _auto_save_name() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def _sanitize_save_name(value: Any) -> str:
    name = str(value or "").strip()
    if not name:
        name = _auto_save_name()
    name = re.sub(r'[<>:"/\\|?*\x00-\x1f]+', "_", name)
    name = re.sub(r"\s+", "_", name).strip(" ._")
    return name or _auto_save_name()


def _relative_to_project(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(PROJECT_ROOT)).replace("\\", "/")
    except Exception:
        return str(path)


def _generated_scenario_root() -> Path:
    info_path = PROJECT_ROOT / "settings" / "current_scenario.json"
    base = PROJECT_ROOT / "Logs"
    try:
        info = json.loads(info_path.read_text(encoding="utf-8"))
        raw_base = info.get("base_root")
        if raw_base:
            base = Path(str(raw_base))
    except Exception:
        pass
    return base / "GeneratedScenario"


def _random_mission_root() -> Path:
    override = str(os.getenv("MISSION_CREATION_RANDOM_ROOT") or "").strip()
    return Path(override).resolve() if override else PROJECT_ROOT / "Logs" / "Random_mission"


def _rtv_mission_root() -> Path:
    override = str(os.getenv("MISSION_CREATION_RTV_ROOT") or "").strip()
    return Path(override).resolve() if override else PROJECT_ROOT / "Logs" / "RTV_mission"


def _scenario_roots() -> tuple[tuple[str, str, Path], ...]:
    return (
        ("manual", "수동", _generated_scenario_root()),
        ("random", "자동", _random_mission_root()),
    )


def _json_file_with_prefix(directory: Path, prefix: str) -> Path | None:
    exact = directory / f"{prefix}_{directory.name}.json"
    if exact.exists() and exact.is_file():
        return exact
    candidates = [path for path in directory.glob(f"{prefix}_*.json") if path.is_file()]
    if not candidates:
        return None
    return max(candidates, key=lambda path: path.stat().st_mtime)


def _load_json_file(path: Path | None) -> dict[str, Any]:
    if not path or not path.exists():
        return {}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return {}
    return data if isinstance(data, dict) else {}


def _safe_scenario_dir(save_name: Any) -> Path:
    raw_name = str(save_name or "").strip()
    storage_key = ""
    name = raw_name
    if ":" in raw_name:
        possible_key, possible_name = raw_name.split(":", 1)
        if possible_key in {row[0] for row in _scenario_roots()}:
            storage_key, name = possible_key, possible_name
    if not name:
        raise ValueError("불러올 시나리오 이름이 없습니다.")
    for key, _label, root_value in _scenario_roots():
        if storage_key and key != storage_key:
            continue
        root = root_value.resolve()
        target = (root / name).resolve()
        try:
            target.relative_to(root)
        except ValueError as exc:
            raise ValueError("허용된 시나리오 저장 폴더만 불러올 수 있습니다.") from exc
        if target.exists() and target.is_dir():
            return target
    raise ValueError(f"시나리오 폴더를 찾을 수 없습니다: {name}")


def _rtv_path_for_scenario(directory: Path) -> Path | None:
    internal = _json_file_with_prefix(directory, "rtv")
    if internal:
        return internal
    try:
        directory.resolve().relative_to(_random_mission_root().resolve())
    except ValueError:
        return None
    archived = _rtv_mission_root() / f"{directory.name}.json"
    return archived if archived.exists() and archived.is_file() else None


def _coord_value(coord: dict[str, Any], lower: str, upper: str | None = None, default: float = 0.0) -> float:
    upper = upper or lower[:1].upper() + lower[1:]
    return _as_float(coord.get(lower, coord.get(upper)), default)


def _point_from_coord(coord: Any) -> dict[str, Any] | None:
    if not isinstance(coord, dict):
        return None
    lat = _coord_value(coord, "latitude")
    lon = _coord_value(coord, "longitude")
    if not (math.isfinite(lat) and math.isfinite(lon)):
        return None
    point: dict[str, Any] = {"latitude": round(float(lat), 8), "longitude": round(float(lon), 8)}
    if "altitude" in coord or "Altitude" in coord:
        alt = _coord_value(coord, "altitude")
        if math.isfinite(alt):
            point["altitude"] = int(round(float(alt)))
    return point


def _points_from_coords(coords: Any) -> list[dict[str, Any]]:
    if not isinstance(coords, list):
        return []
    out: list[dict[str, Any]] = []
    for coord in coords:
        point = _point_from_coord(coord)
        if point is not None:
            out.append(point)
    return out


def _first_list(payload: dict[str, Any], *keys: str) -> list[Any]:
    for key in keys:
        value = payload.get(key)
        if isinstance(value, list):
            return value
    return []


def _first_dict(payload: dict[str, Any], *keys: str) -> dict[str, Any]:
    for key in keys:
        value = payload.get(key)
        if isinstance(value, dict):
            return value
    return {}


def _infer_mission_altitude(missions: list[dict[str, Any]]) -> int:
    for mission in missions:
        detail = _first_dict(mission, "missionDetail", "MissionDetail")
        for coord in _first_list(detail, "coordinateList", "CoordinateList"):
            point = _point_from_coord(coord)
            if point and point.get("altitude") is not None:
                return int(point["altitude"])
        for line in _first_list(detail, "lineList", "LineList", "Polylines"):
            if not isinstance(line, dict):
                continue
            for coord in _first_list(line, "coordinateList", "CoordinateList"):
                point = _point_from_coord(coord)
                if point and point.get("altitude") is not None:
                    return int(point["altitude"])
        for area in _first_list(detail, "areaList", "AreaList"):
            if not isinstance(area, dict):
                continue
            for coord in _first_list(area, "coordinateList", "CoordinateList"):
                point = _point_from_coord(coord)
                if point and point.get("altitude") is not None:
                    return int(point["altitude"])
    return 0


def _infer_ref_agl(reference_points: list[dict[str, Any]], dem: DemSampler) -> int:
    samples: list[float] = []
    if not dem.available:
        return 1000
    for point in reference_points:
        if point.get("altitude") is None:
            continue
        ground = dem.sample_ground_m(_point_lat(point), _point_lon(point))
        if ground is None:
            continue
        agl = float(point["altitude"]) - float(ground)
        if math.isfinite(agl):
            samples.append(agl)
    if not samples:
        return 1000
    return max(0, min(50000, int(round(sum(samples) / len(samples)))))


def _state_from_0203(payload_0203: dict[str, Any], dem: DemSampler) -> dict[str, Any]:
    take_over = [
        point for point in (
            _point_from_coord(_first_dict(entry, "coordinate", "CoordinateList", "Coordinate"))
            for entry in _first_list(payload_0203, "takeOverInfoList", "TakeOverInfoList")
            if isinstance(entry, dict)
        ) if point is not None
    ]
    hand_over = [
        point for point in (
            _point_from_coord(_first_dict(entry, "coordinate", "CoordinateList", "Coordinate"))
            for entry in _first_list(payload_0203, "handOverInfoList", "HandOverInfoList")
            if isinstance(entry, dict)
        ) if point is not None
    ]
    rtb = _points_from_coords(_first_list(payload_0203, "rtbCoordinateList", "RTBCoordinateList"))

    flight_areas: list[list[dict[str, Any]]] = []
    prohibited_areas: list[list[dict[str, Any]]] = []
    area_lower = 0
    area_upper = 5000
    for area in _first_list(payload_0203, "flightAreaList", "FlightAreaList"):
        if not isinstance(area, dict):
            continue
        points = _points_from_coords(_first_list(area, "areaLatLonList", "AreaLatLonList"))
        if len(points) >= 3:
            flight_areas.append(points)
        limits = _first_dict(area, "altitudeLimits", "AltitudeLimits")
        if limits:
            area_lower = max(0, min(50000, _as_int(limits.get("lowerLimit", limits.get("LowerLimit")), area_lower)))
            area_upper = max(area_lower, min(50000, _as_int(limits.get("upperLimit", limits.get("UpperLimit")), area_upper)))
    for area in _first_list(payload_0203, "prohibitedAreaList", "ProhibitedAreaList"):
        if not isinstance(area, dict):
            continue
        points = _points_from_coords(_first_list(area, "areaLatLonList", "AreaLatLonList"))
        if len(points) >= 3:
            prohibited_areas.append(points)
        limits = _first_dict(area, "altitudeLimits", "AltitudeLimits")
        if limits:
            area_lower = max(0, min(50000, _as_int(limits.get("lowerLimit", limits.get("LowerLimit")), area_lower)))
            area_upper = max(area_lower, min(50000, _as_int(limits.get("upperLimit", limits.get("UpperLimit")), area_upper)))

    return {
        "takeOver": take_over,
        "handOver": hand_over,
        "rtb": rtb,
        "flightAreas": flight_areas,
        "prohibitedAreas": prohibited_areas,
        "refAgl": _infer_ref_agl(take_over + hand_over + rtb, dem),
        "areaLower": area_lower,
        "areaUpper": area_upper,
    }


def _state_from_0201(payload_0201: dict[str, Any]) -> dict[str, Any]:
    missions: list[dict[str, Any]] = []
    raw_missions = _first_list(payload_0201, "inputMissionList", "InputMissionList")
    for mission in raw_missions:
        if not isinstance(mission, dict):
            continue
        detail = _first_dict(mission, "missionDetail", "MissionDetail")
        line_list = []
        for line in _first_list(detail, "lineList", "LineList", "Polylines"):
            if not isinstance(line, dict):
                continue
            points = _points_from_coords(_first_list(line, "coordinateList", "CoordinateList"))
            if len(points) >= 2:
                line_list.append({"width": max(0, min(50000, _as_int(line.get("width", line.get("Width")), 1000))), "points": points})
        area_list = []
        for area in _first_list(detail, "areaList", "AreaList"):
            if not isinstance(area, dict):
                continue
            points = _points_from_coords(_first_list(area, "coordinateList", "CoordinateList"))
            if len(points) >= 3:
                area_list.append({"isHole": bool(area.get("isHole", area.get("IsHole", False))), "points": points})
        coordinate_list = _points_from_coords(_first_list(detail, "coordinateList", "CoordinateList"))
        missions.append(
            {
                "inputMissionType": _as_int(mission.get("inputMissionType", mission.get("InputMissionType")), 0),
                "regionType": _as_int(mission.get("regionType", mission.get("RegionType")), 0),
                "lineList": line_list,
                "areaList": area_list,
                "coordinateList": coordinate_list,
                "isDone": bool(mission.get("isDone", mission.get("IsDone", False))),
            }
        )
    package_type = _as_int(payload_0201.get("inputMissionPackageType", payload_0201.get("InputMissionPackageType")), 1)
    if package_type not in PACKAGE_TYPES:
        package_type = 1
    return {
        "packageID": _as_int(payload_0201.get("inputMissionPackageID", payload_0201.get("InputMissionPackageID")), 0) or None,
        "packageType": package_type,
        "missions": missions,
        "missionAlt": _infer_mission_altitude(raw_missions),
    }


def _target_state_from_payload(payload: dict[str, Any]) -> list[dict[str, Any]]:
    targets = []
    for idx, target in enumerate(_first_list(payload, "targetList", "TargetList"), start=1):
        if not isinstance(target, dict):
            continue
        location = _point_from_coord(_first_dict(target, "location", "Location", "coordinate", "Coordinate"))
        if location is None:
            path = _first_list(target, "path", "Path")
            location = _point_from_coord(path[0]) if path else None
        if location is None:
            continue
        targets.append(
            {
                "targetID": _as_int(target.get("targetID", target.get("TargetID")), idx) or idx,
                "targetType": max(1, min(6, _as_int(target.get("targetType", target.get("TargetType")), 1))),
                "inputMissionID": max(1, _as_int(target.get("inputMissionID", target.get("InputMissionID")), 1)),
                "location": location,
            }
        )
    return targets


def _target_payload_for_scenario(directory: Path) -> tuple[dict[str, Any], Path | None]:
    target_path = _json_file_with_prefix(directory, "TargetInfo")
    payload = _load_json_file(target_path)
    if payload.get("targetList") or payload.get("TargetList"):
        return payload, target_path
    rtv_path = _rtv_path_for_scenario(directory)
    rtv_payload = _load_json_file(rtv_path)
    for key in ("TargetInfo", "targetInfo"):
        nested = rtv_payload.get(key)
        if isinstance(nested, dict) and (nested.get("targetList") or nested.get("TargetList")):
            return nested, rtv_path
    return {}, target_path or rtv_path


def list_saved_scenarios() -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for storage_key, storage_label, root in _scenario_roots():
        if not root.exists():
            continue
        for directory in root.iterdir():
            if not directory.is_dir():
                continue
            path_0201 = _json_file_with_prefix(directory, "0201")
            path_0203 = _json_file_with_prefix(directory, "0203")
            if not path_0201 or not path_0203:
                continue
            payload_0201 = _load_json_file(path_0201)
            target_payload, _target_path = _target_payload_for_scenario(directory)
            paths = [path for path in (path_0201, path_0203, _rtv_path_for_scenario(directory)) if path]
            modified_ts = max((path.stat().st_mtime for path in paths), default=directory.stat().st_mtime)
            modified = datetime.fromtimestamp(modified_ts)
            package_type = _as_int(payload_0201.get("inputMissionPackageType", payload_0201.get("InputMissionPackageType")), 0)
            rows.append(
                {
                    "name": directory.name,
                    "loadKey": f"{storage_key}:{directory.name}",
                    "storage": storage_key,
                    "storageLabel": storage_label,
                    "label": f"[{storage_label}] {directory.name}",
                    "modified": modified.isoformat(timespec="seconds"),
                    "modifiedText": modified.strftime("%Y-%m-%d %H:%M:%S"),
                    "packageID": _as_int(payload_0201.get("inputMissionPackageID", payload_0201.get("InputMissionPackageID")), 0),
                    "packageType": package_type,
                    "packageTypeLabel": PACKAGE_TYPES.get(package_type, "-"),
                    "missionCount": len(_first_list(payload_0201, "inputMissionList", "InputMissionList")),
                    "targetCount": len(_first_list(target_payload, "targetList", "TargetList")),
                    "path": _relative_to_project(directory),
                }
            )
    rows.sort(key=lambda row: str(row.get("modified", "")), reverse=True)
    return rows


def load_saved_scenario(save_name: Any, dem: DemSampler) -> dict[str, Any]:
    directory = _safe_scenario_dir(save_name)
    path_0201 = _json_file_with_prefix(directory, "0201")
    path_0203 = _json_file_with_prefix(directory, "0203")
    if not path_0201 or not path_0203:
        raise ValueError("0201/0203 저장 파일을 모두 찾을 수 없습니다.")
    payload_0201 = _load_json_file(path_0201)
    payload_0203 = _load_json_file(path_0203)
    if not payload_0201 or not payload_0203:
        raise ValueError("0201/0203 저장 파일을 읽을 수 없습니다.")
    target_payload, target_path = _target_payload_for_scenario(directory)
    state = {
        "packageType": 1,
        "takeOver": [],
        "handOver": [],
        "rtb": [],
        "flightAreas": [],
        "prohibitedAreas": [],
        "missions": [],
        "targets": [],
        "guidedMeta": {},
        "demEnabled": True,
        "refAgl": 1000,
        "missionAlt": 0,
        "areaLower": 0,
        "areaUpper": 5000,
    }
    state.update(_state_from_0203(payload_0203, dem))
    state.update(_state_from_0201(payload_0201))
    state["targets"] = _target_state_from_payload(target_payload)
    path_rtv = _rtv_path_for_scenario(directory)
    return {
        "ok": True,
        "saveName": directory.name,
        "state": state,
        "outputDir": _relative_to_project(directory),
        "inputMissionPlanRelativePath": _relative_to_project(path_0201),
        "missionReferenceInfoRelativePath": _relative_to_project(path_0203),
        "targetInfoRelativePath": _relative_to_project(target_path) if target_path and target_path.exists() else "",
        "rtvScenarioRelativePath": _relative_to_project(path_rtv) if path_rtv else "",
        "missionCount": len(state["missions"]),
        "targetCount": len(state["targets"]),
    }


def _reference_coord(point: dict[str, Any], agl: int, dem_enabled: bool, dem: DemSampler) -> dict[str, Any]:
    ground = dem.sample_ground_m(_point_lat(point), _point_lon(point)) if dem_enabled else None
    return _coordinate(point, int(agl) + int(ground or 0))


def _mission_coord(point: dict[str, Any], altitude: int) -> dict[str, Any]:
    return _coordinate(point, int(altitude))


def _validate_state(state: dict[str, Any]) -> None:
    if len(state.get("takeOver") or []) != 3:
        raise ValueError("TakeOverInfoList[] 점 3개가 필요합니다.")
    if len(state.get("handOver") or []) != 3:
        raise ValueError("HandOverInfoList[] 점 3개가 필요합니다.")
    if not state.get("rtb"):
        raise ValueError("RTBCoordinateList[]가 1개 이상 필요합니다.")
    if not state.get("flightAreas"):
        raise ValueError("FlightAreaList[] 영역이 필요합니다.")
    if not state.get("prohibitedAreas"):
        raise ValueError("ProhibitedAreaList[] 영역이 필요합니다.")
    if not state.get("missions"):
        raise ValueError("InputMissionList[] 임무가 1개 이상 필요합니다.")


def build_payloads(state: dict[str, Any], dem: DemSampler) -> tuple[int, dict[str, Any], dict[str, Any]]:
    _validate_state(state)
    package_id = _as_int(state.get("packageID"), 0) or _next_package_id()
    timestamp = _now_ms_2000()
    package_type = _as_int(state.get("packageType"), 1)
    if package_type not in PACKAGE_TYPES:
        package_type = 1
    ref_agl = max(0, min(50000, _as_int(state.get("refAgl"), 1000)))
    mission_alt = max(0, min(50000, _as_int(state.get("missionAlt"), 0)))
    lower = max(0, min(50000, _as_int(state.get("areaLower"), 0)))
    upper = max(lower, min(50000, _as_int(state.get("areaUpper"), 5000)))
    dem_enabled = bool(state.get("demEnabled", True))

    take_over = [
        {"aircraftID": AIRCRAFT_SEQUENCE[idx], "coordinate": _reference_coord(point, ref_agl, dem_enabled, dem)}
        for idx, point in enumerate((state.get("takeOver") or [])[:3])
    ]
    hand_over = [
        {"aircraftID": AIRCRAFT_SEQUENCE[idx], "coordinate": _reference_coord(point, ref_agl, dem_enabled, dem)}
        for idx, point in enumerate((state.get("handOver") or [])[:3])
    ]
    rtb = [_reference_coord(point, ref_agl, dem_enabled, dem) for point in state.get("rtb") or []]
    flight_area_list = []
    for idx, area in enumerate(state.get("flightAreas") or [], start=1):
        points = _normalize_area_points(area)
        if len(points) < 3:
            continue
        flight_area_list.append(
            {
                "flightAreaID": idx,
                "areaLatLonList": [_area_latlon(point) for point in points],
                "altitudeLimits": {"lowerLimit": lower, "upperLimit": upper},
            }
        )
    prohibited_area_list = []
    for idx, area in enumerate(state.get("prohibitedAreas") or [], start=1):
        points = _normalize_area_points(area)
        if len(points) < 3:
            continue
        prohibited_area_list.append(
            {
                "prohibitedAreaID": idx,
                "areaLatLonList": [_area_latlon(point) for point in points],
                "altitudeLimits": {"lowerLimit": lower, "upperLimit": upper},
            }
        )

    payload_0203 = {
        "timestamp": timestamp,
        "missionReferencePackageID": package_id,
        "inputTimestamp": timestamp,
        "takeOverInfoList": take_over,
        "handOverInfoList": hand_over,
        "rtbCoordinateList": rtb,
        "flightAreaList": flight_area_list,
        "prohibitedAreaList": prohibited_area_list,
    }

    missions = []
    for idx, mission in enumerate(state.get("missions") or [], start=1):
        mission_type = _as_int(mission.get("inputMissionType", mission.get("missionType")), 0)
        region_type = _as_int(mission.get("regionType"), 0)
        line_list = []
        for line in mission.get("lineList") or []:
            points = line.get("points") or line.get("coordinateList") or []
            if len(points) >= 2:
                line_list.append(
                    {
                        "width": max(0, min(50000, _as_int(line.get("width"), 1000))),
                        "coordinateList": [_mission_coord(point, mission_alt) for point in points],
                    }
                )
        area_list = []
        for area in mission.get("areaList") or []:
            points = _normalize_area_points(area.get("points") or area.get("coordinateList") or [])
            if len(points) >= 3:
                area_list.append(
                    {
                        "isHole": bool(area.get("isHole", False)),
                        "coordinateList": [_mission_coord(point, mission_alt) for point in points],
                    }
                )
        coordinate_list = [
            _mission_coord(point, mission_alt)
            for point in mission.get("coordinateList") or []
        ]
        missions.append(
            {
                "inputMissionID": idx,
                "inputMissionType": mission_type,
                "regionType": region_type,
                "isDone": bool(mission.get("isDone", False)),
                "missionDetail": {
                    "coordinateList": coordinate_list,
                    "lineList": line_list,
                    "areaList": area_list,
                },
            }
        )

    payload_0201 = {
        "timestamp": timestamp,
        "inputMissionPackageID": package_id,
        "inputMissionPackageType": package_type,
        "mainSensor": 1,
        "availableAircraftList": [{"aircraftID": aircraft_id} for aircraft_id in range(1, 7)],
        "inputMissionList": missions,
    }
    return package_id, payload_0201, payload_0203


def build_target_info_payload(
    state: dict[str, Any],
    package_id: int,
    dem: DemSampler,
    mission_ids: set[int],
) -> dict[str, Any]:
    fallback_mission_id = min(mission_ids) if mission_ids else 1
    targets = []
    for idx, target in enumerate(state.get("targets") or [], start=1):
        location_src = target.get("location") or target
        lat = round(_point_lat(location_src), 8)
        lon = round(_point_lon(location_src), 8)
        if not (math.isfinite(lat) and math.isfinite(lon)):
            continue
        altitude = location_src.get("altitude")
        if altitude is None:
            altitude = dem.sample_ground_m(lat, lon)
        coord = {
            "latitude": float(lat),
            "longitude": float(lon),
            "altitude": max(0, min(50000, _as_int(altitude, 0))),
        }
        target_type = _as_int(target.get("targetType"), 1)
        if target_type not in {1, 2, 3, 4, 5, 6}:
            target_type = 1
        input_mission_id = _as_int(target.get("inputMissionID"), fallback_mission_id)
        if input_mission_id not in mission_ids:
            input_mission_id = fallback_mission_id
        targets.append(
            {
                "targetID": idx,
                "targetType": target_type,
                "inputMissionID": input_mission_id,
                "location": coord,
                "path": [coord],
            }
        )
    return {
        "timestamp": _now_ms_2000(),
        "inputMissionPackageID": int(package_id),
        "missionReferencePackageID": int(package_id),
        "targetList": targets,
    }


def build_rtv_scenario_file(
    *,
    path_0201: Path,
    path_0203: Path,
    target_payload: dict[str, Any],
    scenario_name: str,
    output_path: Path,
    template_path: Path | None = None,
) -> None:
    from modules.Random_mission.RTV.build_scenario import DEFAULT_TEMPLATE, build_scenario

    with tempfile.TemporaryDirectory(prefix="mission_creator_targetinfo_") as tmp_dir:
        target_path = Path(tmp_dir) / "TargetInfo.json"
        target_path.write_text(json.dumps(target_payload, ensure_ascii=False, indent=2), encoding="utf-8")
        scenario = build_scenario(
            template_path=template_path or DEFAULT_TEMPLATE,
            imp_path=path_0201,
            mr_path=path_0203,
            tgt_path=target_path,
            scenario_name=scenario_name,
        )
    output_path.write_text(json.dumps(scenario, ensure_ascii=False, indent=2), encoding="utf-8")


def _next_random_scenario_name(scenario_label: str) -> str:
    safe_label = _sanitize_save_name(scenario_label)
    pattern = re.compile(rf"^{re.escape(safe_label)}_(\d+)$")
    indexes: list[int] = []
    root = _random_mission_root()
    if root.exists():
        for directory in root.iterdir():
            if not directory.is_dir():
                continue
            match = pattern.fullmatch(directory.name)
            if match:
                indexes.append(int(match.group(1)))
    rtv_root = _rtv_mission_root()
    if rtv_root.exists():
        for path in rtv_root.glob(f"{safe_label}_*.json"):
            match = pattern.fullmatch(path.stem)
            if match:
                indexes.append(int(match.group(1)))
    return f"{safe_label}_{max(indexes, default=0) + 1}"


def save_random_state(
    state: dict[str, Any],
    dem: DemSampler,
    *,
    scenario_label: str,
) -> dict[str, Any]:
    """자동 생성 결과를 별도 bundle과 Logs/RTV_mission에 원자적으로 저장한다."""

    package_type = _as_int(state.get("packageType"), 0)
    expected_label = scenario_label_for_package(package_type)
    if _sanitize_save_name(scenario_label) != _sanitize_save_name(expected_label):
        raise ValueError("시나리오 Type과 자동 저장 이름이 일치하지 않습니다.")

    with _AUTO_SAVE_LOCK:
        save_name = _next_random_scenario_name(expected_label)
        package_id = _next_package_id()
        state["packageID"] = package_id
        _package_id, payload_0201, payload_0203 = build_payloads(state, dem)
        mission_ids = {
            _as_int(mission.get("inputMissionID"), 0)
            for mission in payload_0201.get("inputMissionList") or []
        }
        mission_ids.discard(0)
        target_payload = build_target_info_payload(state, package_id, dem, mission_ids)

        random_root = _random_mission_root()
        rtv_root = _rtv_mission_root()
        random_root.mkdir(parents=True, exist_ok=True)
        rtv_root.mkdir(parents=True, exist_ok=True)
        output_dir = random_root / save_name
        path_rtv = rtv_root / f"{save_name}.json"
        if output_dir.exists() or path_rtv.exists():
            raise FileExistsError(f"자동 시나리오 저장 이름이 이미 존재합니다: {save_name}")

        temp_dir = Path(tempfile.mkdtemp(prefix=".auto_mission_", dir=str(random_root)))
        committed = False
        rtv_committed = False
        try:
            temp_0201 = temp_dir / f"0201_{save_name}.json"
            temp_0203 = temp_dir / f"0203_{save_name}.json"
            temp_target = temp_dir / f"TargetInfo_{save_name}.json"
            temp_rtv = temp_dir / f"{save_name}.json"
            temp_0201.write_text(json.dumps(payload_0201, ensure_ascii=False, indent=2), encoding="utf-8")
            temp_0203.write_text(json.dumps(payload_0203, ensure_ascii=False, indent=2), encoding="utf-8")
            temp_target.write_text(json.dumps(target_payload, ensure_ascii=False, indent=2), encoding="utf-8")
            build_rtv_scenario_file(
                path_0201=temp_0201,
                path_0203=temp_0203,
                target_payload=target_payload,
                scenario_name=save_name,
                output_path=temp_rtv,
                template_path=reference_path_for_package(package_type),
            )
            temp_dir.replace(output_dir)
            committed = True
            (output_dir / temp_rtv.name).replace(path_rtv)
            rtv_committed = True
        except Exception:
            if committed and output_dir.exists():
                shutil.rmtree(output_dir, ignore_errors=True)
            elif temp_dir.exists():
                shutil.rmtree(temp_dir, ignore_errors=True)
            if rtv_committed and path_rtv.exists():
                try:
                    path_rtv.unlink()
                except OSError:
                    pass
            raise

        path_0201 = output_dir / f"0201_{save_name}.json"
        path_0203 = output_dir / f"0203_{save_name}.json"
        path_target = output_dir / f"TargetInfo_{save_name}.json"
        return {
            "ok": True,
            "storage": "random",
            "packageID": package_id,
            "saveName": save_name,
            "outputDir": _relative_to_project(output_dir),
            "inputMissionPlanPath": str(path_0201),
            "missionReferenceInfoPath": str(path_0203),
            "targetInfoPath": str(path_target),
            "rtvScenarioPath": str(path_rtv),
            "inputMissionPlanRelativePath": _relative_to_project(path_0201),
            "missionReferenceInfoRelativePath": _relative_to_project(path_0203),
            "targetInfoRelativePath": _relative_to_project(path_target),
            "rtvScenarioRelativePath": _relative_to_project(path_rtv),
            "missionCount": len(payload_0201.get("inputMissionList") or []),
            "targetCount": len(target_payload.get("targetList") or []),
        }


def save_state(state: dict[str, Any], dem: DemSampler, save_name: str | None = None) -> dict[str, Any]:
    package_id, payload_0201, payload_0203 = build_payloads(state, dem)
    safe_name = _sanitize_save_name(save_name)
    output_dir = _generated_scenario_root() / safe_name
    output_dir.mkdir(parents=True, exist_ok=True)
    path_0201 = output_dir / f"0201_{safe_name}.json"
    path_0203 = output_dir / f"0203_{safe_name}.json"
    path_target = output_dir / f"TargetInfo_{safe_name}.json"
    path_rtv = output_dir / f"rtv_{safe_name}.json"
    path_0201.write_text(json.dumps(payload_0201, ensure_ascii=False, indent=2), encoding="utf-8")
    path_0203.write_text(json.dumps(payload_0203, ensure_ascii=False, indent=2), encoding="utf-8")
    mission_ids = {
        _as_int(mission.get("inputMissionID"), 0)
        for mission in payload_0201.get("inputMissionList") or []
    }
    mission_ids.discard(0)
    target_payload = build_target_info_payload(state, package_id, dem, mission_ids)
    path_target.write_text(json.dumps(target_payload, ensure_ascii=False, indent=2), encoding="utf-8")
    build_rtv_scenario_file(
        path_0201=path_0201,
        path_0203=path_0203,
        target_payload=target_payload,
        scenario_name=f"rtv_{safe_name}",
        output_path=path_rtv,
    )
    return {
        "ok": True,
        "packageID": package_id,
        "saveName": safe_name,
        "outputDir": _relative_to_project(output_dir),
        "inputMissionPlanPath": str(path_0201),
        "missionReferenceInfoPath": str(path_0203),
        "targetInfoPath": str(path_target),
        "rtvScenarioPath": str(path_rtv),
        "inputMissionPlanRelativePath": _relative_to_project(path_0201),
        "missionReferenceInfoRelativePath": _relative_to_project(path_0203),
        "targetInfoRelativePath": _relative_to_project(path_target),
        "rtvScenarioRelativePath": _relative_to_project(path_rtv),
        "targetCount": len(target_payload.get("targetList") or []),
    }


def _build_html(
    *,
    tile_url: str,
    min_zoom: int,
    max_zoom: int,
    center_lat: float,
    center_lon: float,
    start_zoom: float,
    dem_available: bool,
    mission_regions: list[dict[str, Any]] | None = None,
) -> str:
    config = json.dumps(
        {
            "tileUrl": tile_url,
            "minZoom": int(min_zoom),
            "maxZoom": int(max_zoom),
            "center": [float(center_lon), float(center_lat)],
            "zoom": float(start_zoom),
            "demAvailable": bool(dem_available),
            "packageTypes": PACKAGE_TYPES,
            "regionTypes": REGION_TYPES,
            "missionTypes": MISSION_TYPES,
            "missionRegions": mission_regions or [],
        },
        ensure_ascii=False,
    )
    return f"""<!doctype html>
<html lang="ko">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>임무 생성 GUI</title>
  <link rel="stylesheet" href="/vendor/maplibre-gl.css" />
  <style>
    :root {{ color-scheme: dark; }}
    html, body {{ margin: 0; width: 100%; height: 100%; overflow: hidden; font-family: "Segoe UI", "Malgun Gothic", sans-serif; background: #111715; color: #eef5f1; }}
    #app {{ display: grid; grid-template-columns: minmax(0, 1fr) 340px; height: 100%; }}
    #map {{ width: 100%; height: 100%; }}
    .center-ruler {{ position: absolute; inset: 0; z-index: 7; pointer-events: none; color: rgba(238, 245, 241, 0.78); font-family: Consolas, "Malgun Gothic", monospace; font-size: 11px; }}
    .center-ruler .ruler-h {{ position: absolute; left: 50%; top: 50%; width: min(34vw, 360px); height: 0; transform: translate(-50%, -50%); border-top: 1px solid rgba(238, 245, 241, 0.35); }}
    .center-ruler .ruler-v {{ position: absolute; left: 50%; top: 50%; width: 0; height: min(34vh, 260px); transform: translate(-50%, -50%); border-left: 1px solid rgba(238, 245, 241, 0.35); }}
    .center-ruler .ruler-h::before, .center-ruler .ruler-h::after {{ content: ""; position: absolute; top: -5px; width: 1px; height: 10px; background: rgba(238, 245, 241, 0.38); }}
    .center-ruler .ruler-h::before {{ left: 0; }}
    .center-ruler .ruler-h::after {{ right: 0; }}
    .center-ruler .ruler-v::before, .center-ruler .ruler-v::after {{ content: ""; position: absolute; left: -5px; width: 10px; height: 1px; background: rgba(238, 245, 241, 0.38); }}
    .center-ruler .ruler-v::before {{ top: 0; }}
    .center-ruler .ruler-v::after {{ bottom: 0; }}
    .center-ruler .ruler-center {{ position: absolute; left: 50%; top: 50%; width: 7px; height: 7px; transform: translate(-50%, -50%); border: 1px solid rgba(238, 245, 241, 0.45); border-radius: 50%; background: rgba(15, 22, 21, 0.18); }}
    .center-ruler .ruler-label {{ position: absolute; padding: 2px 5px; border-radius: 4px; background: rgba(12, 18, 18, 0.56); border: 1px solid rgba(238, 245, 241, 0.14); white-space: nowrap; }}
    .center-ruler .ruler-h .ruler-label {{ left: 50%; top: 7px; transform: translateX(-50%); }}
    .center-ruler .ruler-v .ruler-label {{ left: 8px; top: 50%; transform: translateY(-50%); }}
    aside {{ border-left: 1px solid rgba(214, 229, 222, 0.18); background: #17201d; display: flex; flex-direction: column; min-width: 0; }}
    .toolbar {{ position: absolute; z-index: 10; top: 12px; left: 12px; right: 360px; display: flex; flex-wrap: wrap; gap: 8px; align-items: center; pointer-events: none; }}
    .toolbar > * {{ pointer-events: auto; }}
    button, select, input {{ border: 1px solid rgba(214, 229, 222, 0.26); background: #22312d; color: #eef5f1; border-radius: 6px; height: 30px; padding: 0 10px; font: inherit; }}
    button {{ cursor: pointer; font-weight: 700; }}
    button:hover {{ background: #2e4540; }}
    button:disabled {{ cursor: default; opacity: 0.45; background: #202927; }}
    button.primary {{ background: #28665f; border-color: #4ba89d; }}
    button.warn {{ background: #5d3635; border-color: #99605e; }}
    input[type="number"] {{ width: 76px; }}
    input.auto-seed {{ width: 92px; }}
    label {{ display: inline-flex; gap: 5px; align-items: center; font-size: 12px; color: #c8d8d2; }}
    .instruction {{ position: absolute; z-index: 10; left: 12px; bottom: 14px; max-width: calc(100% - 390px); background: rgba(15, 22, 21, 0.86); border: 1px solid rgba(214,229,222,0.22); padding: 9px 11px; border-radius: 6px; color: #f6fbf8; }}
    .side-head {{ padding: 14px 14px 10px; border-bottom: 1px solid rgba(214, 229, 222, 0.16); }}
    .side-title {{ font-weight: 800; font-size: 16px; }}
    .side-sub {{ color: #9fb0aa; font-size: 12px; margin-top: 4px; }}
    .mode-card {{ margin: 12px 14px 0; padding: 10px 11px; border: 1px solid rgba(214, 229, 222, 0.18); border-radius: 6px; background: rgba(24, 35, 32, 0.94); }}
    .mode-eyebrow {{ color: #8fbab0; font-size: 11px; font-weight: 800; letter-spacing: 0; }}
    .mode-title {{ margin-top: 4px; color: #f7fbf9; font-size: 15px; font-weight: 800; }}
    .mode-detail {{ margin-top: 5px; color: #cddbd6; font-size: 12px; line-height: 1.45; }}
    .mode-progress {{ margin-top: 7px; color: #9fb0aa; font-size: 12px; }}
    .workflow-card {{ margin: 10px 14px 0; display: grid; gap: 8px; }}
    .workflow-row {{ padding: 9px 10px; border: 1px solid rgba(214, 229, 222, 0.14); border-radius: 6px; background: rgba(18, 27, 25, 0.78); }}
    .workflow-row.is-active {{ border-color: rgba(242, 213, 92, 0.58); background: rgba(45, 42, 26, 0.72); }}
    .workflow-row.is-done {{ border-color: rgba(91, 190, 145, 0.54); background: rgba(25, 43, 35, 0.76); }}
    .workflow-top {{ display: flex; justify-content: space-between; gap: 8px; align-items: center; }}
    .workflow-name {{ color: #eef5f1; font-size: 13px; font-weight: 800; }}
    .workflow-status {{ flex: none; color: #fff0ad; font-size: 12px; font-weight: 800; }}
    .workflow-row.is-done .workflow-status {{ color: #8ee2b9; }}
    .workflow-detail {{ margin-top: 5px; color: #aebdb7; font-size: 12px; line-height: 1.42; }}
    .choice-panel {{ margin: 10px 14px 0; padding: 10px 11px; border: 1px solid rgba(242, 213, 92, 0.42); border-radius: 6px; background: rgba(34, 42, 34, 0.96); display: none; }}
    .choice-panel.is-active {{ display: block; }}
    .choice-title {{ color: #fff0ad; font-size: 13px; font-weight: 800; }}
    .choice-detail {{ margin-top: 5px; color: #d7d1b4; font-size: 12px; line-height: 1.45; }}
    .choice-grid {{ display: grid; grid-template-columns: 1fr; gap: 6px; margin-top: 9px; }}
    .choice-grid button {{ height: auto; min-height: 32px; padding: 7px 9px; text-align: left; white-space: normal; line-height: 1.25; }}
    .choice-grid button.secondary {{ background: #2a302f; border-color: rgba(214, 229, 222, 0.22); color: #ccd8d4; }}
    .save-panel, .load-panel {{ margin: 10px 14px 0; padding: 10px 11px; border: 1px solid rgba(91, 190, 145, 0.46); border-radius: 6px; background: rgba(23, 39, 32, 0.96); display: none; }}
    .save-panel.is-active, .load-panel.is-active {{ display: block; }}
    .save-title, .load-title {{ color: #9df0c3; font-size: 13px; font-weight: 800; }}
    .save-detail, .load-detail {{ margin-top: 5px; color: #c9ded5; font-size: 12px; line-height: 1.45; }}
    .save-panel input, .load-panel select {{ width: 100%; box-sizing: border-box; margin-top: 9px; height: 32px; }}
    .save-actions, .load-actions {{ display: flex; gap: 7px; margin-top: 9px; }}
    .save-actions button, .load-actions button {{ flex: 1; }}
    .load-meta {{ min-height: 18px; margin-top: 7px; color: #9fb0aa; font-size: 12px; line-height: 1.35; }}
    .summary {{ padding: 12px 14px; white-space: pre-wrap; overflow: auto; font-family: Consolas, "Malgun Gothic", monospace; font-size: 12px; line-height: 1.55; flex: 1; }}
    .type-picker {{ position: absolute; width: 296px; height: 296px; transform: translate(-50%, -50%); pointer-events: none; opacity: 0; transition: opacity 120ms ease; z-index: 12; }}
    .type-picker.is-active {{ opacity: 1; pointer-events: auto; }}
    .type-picker-center {{ position: absolute; left: 132px; top: 132px; width: 32px; height: 32px; border-radius: 50%; background: rgba(16, 22, 24, 0.88); border: 1px solid rgba(220, 235, 240, 0.55); }}
    .type-picker button {{ position: absolute; width: 68px; height: 68px; border-radius: 50%; background: rgba(26, 44, 46, 0.97); box-shadow: 0 8px 20px rgba(0,0,0,.32); display: flex; flex-direction: column; align-items: center; justify-content: center; gap: 1px; padding: 0; line-height: 1.05; }}
    .type-no {{ color: #fff0ad; font-size: 13px; font-weight: 900; }}
    .type-name {{ color: #f6fbf8; font-size: 10.5px; font-weight: 800; text-align: center; }}
    .region-picker {{ position: absolute; width: 384px; height: 384px; transform: translate(-50%, -50%); pointer-events: none; opacity: 0; transition: opacity 120ms ease; z-index: 13; }}
    .region-picker.is-active {{ opacity: 1; pointer-events: auto; }}
    .region-picker-center {{ position: absolute; left: 174px; top: 174px; width: 36px; height: 36px; border-radius: 50%; background: rgba(16, 22, 24, 0.9); border: 1px solid rgba(242, 213, 92, 0.65); color: #fff0ad; display: flex; align-items: center; justify-content: center; font-size: 10px; font-weight: 900; }}
    .region-picker button {{ position: absolute; width: 72px; height: 72px; border-radius: 50%; background: rgba(48, 42, 25, 0.97); border-color: rgba(242, 213, 92, 0.48); box-shadow: 0 8px 20px rgba(0,0,0,.34); display: flex; flex-direction: column; align-items: center; justify-content: center; gap: 1px; padding: 0; line-height: 1.05; }}
    .region-no {{ color: #fff0ad; font-size: 12px; font-weight: 900; }}
    .region-name {{ color: #f8f0cf; font-size: 9.5px; font-weight: 800; text-align: center; }}
    .hole-picker {{ position: absolute; width: 184px; height: 96px; transform: translate(-50%, -50%); pointer-events: none; opacity: 0; transition: opacity 120ms ease; z-index: 14; }}
    .hole-picker.is-active {{ opacity: 1; pointer-events: auto; }}
    .hole-picker button {{ position: absolute; width: 82px; height: 82px; border-radius: 50%; background: rgba(47, 37, 24, 0.98); border-color: rgba(242, 213, 92, 0.58); box-shadow: 0 8px 20px rgba(0,0,0,.36); display: flex; flex-direction: column; align-items: center; justify-content: center; gap: 2px; padding: 0; line-height: 1.05; }}
    .hole-picker button[data-hole="false"] {{ left: 0; top: 7px; }}
    .hole-picker button[data-hole="true"] {{ right: 0; top: 7px; }}
    .hole-main {{ color: #fff0ad; font-size: 12px; font-weight: 900; }}
    .hole-sub {{ color: #f8f0cf; font-size: 10px; font-weight: 800; text-align: center; }}
    .width-tip {{ position: absolute; left: 12px; bottom: 58px; z-index: 11; min-width: 170px; padding: 7px 10px; border-radius: 6px; color: #f5fbff; background: rgba(11,18,20,.82); border: 1px solid rgba(220,230,235,.32); display: none; }}
    .width-tip.is-active {{ display: block; }}
    .cursor-support {{ position: absolute; z-index: 18; min-width: 172px; padding: 7px 9px; border-radius: 6px; color: #eef8f5; background: rgba(10, 16, 18, 0.88); border: 1px solid rgba(201, 222, 213, 0.32); box-shadow: 0 8px 18px rgba(0,0,0,.24); pointer-events: none; opacity: 0; transform: translate(12px, 14px); transition: opacity 80ms ease; font-family: Consolas, "Malgun Gothic", monospace; font-size: 11px; line-height: 1.35; }}
    .cursor-support.is-active {{ opacity: 1; }}
    .cursor-support .coord-title {{ color: #9df0c3; font-family: "Malgun Gothic", sans-serif; font-size: 11px; font-weight: 800; }}
    .cursor-support .coord-row {{ white-space: nowrap; }}
  </style>
</head>
<body>
  <div id="app">
    <main style="position:relative; min-width:0;">
      <div id="map"></div>
      <div id="centerRuler" class="center-ruler" aria-hidden="true">
        <div id="centerRulerH" class="ruler-h"><span id="centerRulerHLabel" class="ruler-label">가로 -</span></div>
        <div id="centerRulerV" class="ruler-v"><span id="centerRulerVLabel" class="ruler-label">세로 -</span></div>
        <div class="ruler-center"></div>
      </div>
      <div class="toolbar">
        <button id="undo" title="마지막 입력 되돌리기 (Ctrl+Z)" disabled>Undo</button>
        <button id="autoGenerate" class="primary" title="선택한 Type을 자동 생성하고 별도 폴더에 즉시 저장합니다.">자동 생성+저장</button>
        <label>Seed <input id="autoSeed" class="auto-seed" type="number" step="1" placeholder="비우면 매번 랜덤" /></label>
        <button id="addMission">임무 추가</button>
        <button id="addTarget">적 배치</button>
        <label>적 Type <select id="targetType"><option value="1">1 전차</option><option value="2">2 장갑차</option><option value="3">3 방사포</option><option value="4">4 곡사포</option><option value="5">5 고정고사포</option><option value="6">6 군인</option></select></label>
        <label>관련 임무 <select id="targetMission"><option value="0">자동</option></select></label>
        <button id="save" class="primary">저장</button>
        <button id="loadScenario">불러오기</button>
        <button id="reset" class="warn">초기화</button>
        <label>Type <select id="packageType"></select></label>
        <label>0203 AGL <input id="refAgl" type="number" min="0" max="50000" value="1000" /></label>
        <label><input id="demEnabled" type="checkbox" checked /> DEM</label>
        <label>0201 ALT <input id="missionAlt" type="number" min="0" max="50000" value="0" /></label>
        <label>Area ALT <input id="areaLower" type="number" min="0" max="50000" value="0" /> <input id="areaUpper" type="number" min="0" max="50000" value="5000" /></label>
      </div>
      <div id="instruction" class="instruction">지도 로딩 중</div>
      <div id="typePicker" class="type-picker" aria-hidden="true">
        <div class="type-picker-center"></div>
        <button type="button" data-type="1" title="협업기동임무"><span class="type-no">1</span><span class="type-name">기동</span></button>
        <button type="button" data-type="2" title="협업수색공격임무"><span class="type-no">2</span><span class="type-name">수색<br>공격</span></button>
        <button type="button" data-type="3" title="협업경계임무"><span class="type-no">3</span><span class="type-name">경계</span></button>
        <button type="button" data-type="4" title="협업공중부대엄호임무"><span class="type-no">4</span><span class="type-name">공중<br>엄호</span></button>
        <button type="button" data-type="5" title="협업지상부대엄호임무"><span class="type-no">5</span><span class="type-name">지상<br>엄호</span></button>
        <button type="button" data-type="6" title="협업도심수색공격임무"><span class="type-no">6</span><span class="type-name">도심<br>수색</span></button>
        <button type="button" data-type="7" title="편대비행모드"><span class="type-no">7</span><span class="type-name">편대</span></button>
        <button type="button" data-type="0" title="Not used"><span class="type-no">0</span><span class="type-name">미사용</span></button>
      </div>
      <div id="regionPicker" class="region-picker" aria-hidden="true">
        <div class="region-picker-center">지역</div>
      </div>
      <div id="holePicker" class="hole-picker" aria-hidden="true">
        <button type="button" data-hole="false" title="일반 Area">
          <span class="hole-main">false</span>
          <span class="hole-sub">일반<br>Area</span>
        </button>
        <button type="button" data-hole="true" title="Hole Area">
          <span class="hole-main">true</span>
          <span class="hole-sub">Hole<br>Area</span>
        </button>
      </div>
      <div id="widthTip" class="width-tip">width: 0m</div>
      <div id="cursorSupport" class="cursor-support" aria-hidden="true"></div>
    </main>
    <aside>
      <div class="side-head">
        <div class="side-title">임무 생성 GUI</div>
        <div id="saveStatus" class="side-sub">저장 대기</div>
      </div>
      <section class="mode-card" aria-live="polite">
        <div class="mode-eyebrow">현재 입력 모드</div>
        <div id="modeTitle" class="mode-title">대기</div>
        <div id="modeDetail" class="mode-detail">지도 로딩 중</div>
        <div id="modeProgress" class="mode-progress">-</div>
      </section>
      <section class="workflow-card" aria-live="polite">
        <div id="flow0203" class="workflow-row">
          <div class="workflow-top">
            <span class="workflow-name">0203 기준정보</span>
            <strong id="flow0203Status" class="workflow-status">미시작</strong>
          </div>
          <div id="flow0203Detail" class="workflow-detail">TakeOver부터 입력하세요.</div>
        </div>
        <div id="flow0201" class="workflow-row">
          <div class="workflow-top">
            <span class="workflow-name">0201 임무목록</span>
            <strong id="flow0201Status" class="workflow-status">미시작</strong>
          </div>
          <div id="flow0201Detail" class="workflow-detail">0203 완료 후 임무 추가를 누르세요.</div>
        </div>
      </section>
      <section id="savePanel" class="save-panel" hidden>
        <div class="save-title">저장 이름</div>
        <div class="save-detail">비워두면 날짜시간으로 자동 생성됩니다. 저장 위치: current_scenario 기준 Logs/GeneratedScenario</div>
        <input id="saveNameInput" type="text" placeholder="예: 표적공격_초안" autocomplete="off" />
        <div class="save-actions">
          <button id="saveConfirm" class="primary" type="button">저장 실행</button>
          <button id="saveCancel" type="button">취소</button>
        </div>
      </section>
      <section id="loadPanel" class="load-panel" hidden>
        <div class="load-title">시나리오 불러오기</div>
        <div class="load-detail">수동 GeneratedScenario와 Logs/Random_mission 자동 생성본을 현재 지도에 다시 표시합니다.</div>
        <select id="scenarioSelect"></select>
        <div id="loadMeta" class="load-meta">목록을 불러오는 중</div>
        <div class="load-actions">
          <button id="loadConfirm" class="primary" type="button">불러오기</button>
          <button id="loadRefresh" type="button">새로고침</button>
          <button id="loadCancel" type="button">취소</button>
        </div>
      </section>
      <section id="choicePanel" class="choice-panel" hidden></section>
      <div id="summary" class="summary"></div>
    </aside>
  </div>
  <script src="/vendor/maplibre-gl.js"></script>
  <script>
    const config = {config};
    const AIRCRAFT_IDS = [4, 5, 6];
    const state = {{ packageType: 1, takeOver: [], handOver: [], rtb: [], flightAreas: [], prohibitedAreas: [], missions: [], targets: [], guidedMeta: {{}} }};
    let stage = "idle";
    let activePoints = [];
    let activeMission = null;
    let pendingMissionLngLat = null;
    let pendingRegionMissionType = null;
    let pendingRegionLngLat = null;
    let pendingHoleScreenPoint = null;
    let widthLine = [];
    let widthDragging = false;
    let widthValue = 0;
    let suppressMapClickUntil = 0;
    const UNDO_LIMIT = 80;
    const undoStack = [];
    let cursorLngLat = null;
    let cursorPoint = null;
    let cursorElevationText = config.demAvailable ? "고도 조회 중" : "고도 N/A";
    let cursorElevationTimer = null;
    let cursorElevationSeq = 0;
    const instruction = document.getElementById("instruction");
    const summary = document.getElementById("summary");
    const saveStatus = document.getElementById("saveStatus");
    const undoButton = document.getElementById("undo");
    const typePicker = document.getElementById("typePicker");
    const regionPicker = document.getElementById("regionPicker");
    const holePicker = document.getElementById("holePicker");
    const widthTip = document.getElementById("widthTip");
    const cursorSupport = document.getElementById("cursorSupport");
    const centerRulerH = document.getElementById("centerRulerH");
    const centerRulerV = document.getElementById("centerRulerV");
    const centerRulerHLabel = document.getElementById("centerRulerHLabel");
    const centerRulerVLabel = document.getElementById("centerRulerVLabel");
    const modeTitle = document.getElementById("modeTitle");
    const modeDetail = document.getElementById("modeDetail");
    const modeProgress = document.getElementById("modeProgress");
    const choicePanel = document.getElementById("choicePanel");
    const savePanel = document.getElementById("savePanel");
    const saveNameInput = document.getElementById("saveNameInput");
    const saveConfirm = document.getElementById("saveConfirm");
    const saveCancel = document.getElementById("saveCancel");
    const loadPanel = document.getElementById("loadPanel");
    const scenarioSelect = document.getElementById("scenarioSelect");
    const loadMeta = document.getElementById("loadMeta");
    const loadConfirm = document.getElementById("loadConfirm");
    const loadRefresh = document.getElementById("loadRefresh");
    const loadCancel = document.getElementById("loadCancel");
    const flow0203 = document.getElementById("flow0203");
    const flow0203Status = document.getElementById("flow0203Status");
    const flow0203Detail = document.getElementById("flow0203Detail");
    const flow0201 = document.getElementById("flow0201");
    const flow0201Status = document.getElementById("flow0201Status");
    const flow0201Detail = document.getElementById("flow0201Detail");
    const targetTypeSelect = document.getElementById("targetType");
    const targetMissionSelect = document.getElementById("targetMission");
    const autoGenerateButton = document.getElementById("autoGenerate");
    const autoSeedInput = document.getElementById("autoSeed");

    const packageSelect = document.getElementById("packageType");
    Object.entries(config.packageTypes).forEach(([code, label]) => {{
      const opt = document.createElement("option");
      opt.value = code;
      opt.textContent = `${{code}}: ${{label}}`;
      packageSelect.appendChild(opt);
    }});

    const style = {{
      version: 8,
      sources: {{ mbtiles: {{ type: "vector", tiles: [config.tileUrl], minzoom: config.minZoom, maxzoom: config.maxZoom }} }},
      glyphs: "https://demotiles.maplibre.org/font/{{fontstack}}/{{range}}.pbf",
      layers: [
        {{ id: "background", type: "background", paint: {{ "background-color": "#17201b" }} }},
        {{ id: "landcover", type: "fill", source: "mbtiles", "source-layer": "landcover", paint: {{ "fill-color": "#31402c", "fill-opacity": 0.76 }} }},
        {{ id: "landuse", type: "fill", source: "mbtiles", "source-layer": "landuse", paint: {{ "fill-color": "#3d4934", "fill-opacity": 0.62 }} }},
        {{ id: "park", type: "fill", source: "mbtiles", "source-layer": "park", paint: {{ "fill-color": "#455c3f", "fill-opacity": 0.76 }} }},
        {{ id: "water", type: "fill", source: "mbtiles", "source-layer": "water", paint: {{ "fill-color": "#234a63" }} }},
        {{ id: "waterway", type: "line", source: "mbtiles", "source-layer": "waterway", paint: {{ "line-color": "#3d6e86", "line-width": 1 }} }},
        {{ id: "boundary", type: "line", source: "mbtiles", "source-layer": "boundary", paint: {{ "line-color": "#70806f", "line-width": 1, "line-dasharray": [2, 2] }} }},
        {{ id: "transportation", type: "line", source: "mbtiles", "source-layer": "transportation", paint: {{ "line-color": "#6a7168", "line-width": ["interpolate", ["linear"], ["zoom"], 8, .45, 13, 1.35] }} }},
        {{ id: "building", type: "fill", source: "mbtiles", "source-layer": "building", minzoom: 13, paint: {{ "fill-color": "#6e7565", "fill-opacity": .5 }} }}
      ]
    }};
    const map = new maplibregl.Map({{ container: "map", style, center: config.center, zoom: config.zoom, minZoom: config.minZoom, maxZoom: config.maxZoom, attributionControl: false }});
    map.addControl(new maplibregl.NavigationControl({{ showCompass: false }}), "top-right");

    function cloneValue(value) {{
      if (value === undefined) return null;
      return JSON.parse(JSON.stringify(value));
    }}
    function updateUndoButton() {{
      if (!undoButton) return;
      undoButton.disabled = undoStack.length === 0;
      undoButton.title = undoStack.length ? `마지막 입력 되돌리기 (Ctrl+Z) - ${{undoStack[undoStack.length - 1].label}}` : "되돌릴 입력이 없습니다";
    }}
    function captureUndoSnapshot(label) {{
      return {{
        label: label || "입력",
        state: cloneValue(state),
        stage,
        activePoints: cloneValue(activePoints),
        activeMission: cloneValue(activeMission),
        pendingMissionLngLat: cloneValue(pendingMissionLngLat),
        pendingRegionMissionType,
        pendingRegionLngLat: cloneValue(pendingRegionLngLat),
        pendingHoleScreenPoint: cloneValue(pendingHoleScreenPoint),
        widthLine: cloneValue(widthLine),
        widthValue: Number(widthValue || 0)
      }};
    }}
    function pushUndo(label) {{
      undoStack.push(captureUndoSnapshot(label));
      if (undoStack.length > UNDO_LIMIT) undoStack.shift();
      updateUndoButton();
    }}
    function restoreStateObject(snapshotState) {{
      Object.keys(state).forEach(key => delete state[key]);
      Object.assign(state, cloneValue(snapshotState) || {{}});
    }}
    function undoLast() {{
      const snapshot = undoStack.pop();
      if (!snapshot) return showNotice("되돌릴 입력이 없습니다.");
      restoreStateObject(snapshot.state);
      stage = snapshot.stage || "idle";
      activePoints = cloneValue(snapshot.activePoints) || [];
      activeMission = cloneValue(snapshot.activeMission);
      pendingMissionLngLat = cloneValue(snapshot.pendingMissionLngLat);
      pendingRegionMissionType = snapshot.pendingRegionMissionType;
      pendingRegionLngLat = cloneValue(snapshot.pendingRegionLngLat);
      pendingHoleScreenPoint = cloneValue(snapshot.pendingHoleScreenPoint);
      widthLine = cloneValue(snapshot.widthLine) || [];
      widthValue = Number(snapshot.widthValue || 0);
      widthDragging = false;
      hideChoicePanel();
      hideTypePicker();
      hideRegionPicker();
      hideHolePicker();
      hideSavePanel();
      if (stage === "width") {{
        widthTip.classList.add("is-active");
        setWidthPreview(widthValue);
      }} else {{
        clearWidthPreview();
      }}
      updateUndoButton();
      setInstruction(`되돌림: ${{snapshot.label}}`);
      render();
      return true;
    }}

    function distanceMeters(a, b) {{
      const rad = Math.PI / 180;
      const lat1 = Number(a.lat) * rad;
      const lat2 = Number(b.lat) * rad;
      const dLat = lat2 - lat1;
      const dLon = (Number(b.lng) - Number(a.lng)) * rad;
      const h = Math.sin(dLat / 2) ** 2 + Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) ** 2;
      return 6371000 * 2 * Math.atan2(Math.sqrt(h), Math.sqrt(Math.max(0, 1 - h)));
    }}
    function formatRulerDistance(meters) {{
      const value = Math.max(0, Number(meters) || 0);
      if (value >= 10000) return `${{(value / 1000).toFixed(1)}} km`;
      if (value >= 1000) return `${{(value / 1000).toFixed(2)}} km`;
      return `${{Math.round(value)}} m`;
    }}
    function updateCenterRuler() {{
      const canvas = map.getCanvas();
      const w = canvas.clientWidth || 0;
      const h = canvas.clientHeight || 0;
      if (!w || !h) return;
      const hLen = Math.max(120, Math.min(360, Math.round(w * 0.34)));
      const vLen = Math.max(100, Math.min(260, Math.round(h * 0.34)));
      centerRulerH.style.width = `${{hLen}}px`;
      centerRulerV.style.height = `${{vLen}}px`;
      const cx = w / 2;
      const cy = h / 2;
      const left = map.unproject([cx - hLen / 2, cy]);
      const right = map.unproject([cx + hLen / 2, cy]);
      const top = map.unproject([cx, cy - vLen / 2]);
      const bottom = map.unproject([cx, cy + vLen / 2]);
      centerRulerHLabel.textContent = `가로 ${{formatRulerDistance(distanceMeters(left, right))}}`;
      centerRulerVLabel.textContent = `세로 ${{formatRulerDistance(distanceMeters(top, bottom))}}`;
    }}

    function missionRegionGeoJson() {{
      const features = [];
      (config.missionRegions || []).forEach((region) => {{
        const b = region.bounds || [];
        if (b.length !== 4) return;
        const west = Number(b[0]), south = Number(b[1]), east = Number(b[2]), north = Number(b[3]);
        if (![west, south, east, north].every(Number.isFinite)) return;
        features.push({{
          type: "Feature",
          properties: {{ label: region.label || "mission-region" }},
          geometry: {{
            type: "Polygon",
            coordinates: [[
              [west, south],
              [east, south],
              [east, north],
              [west, north],
              [west, south]
            ]]
          }}
        }});
      }});
      return {{ type: "FeatureCollection", features }};
    }}

    const STAGE_LABELS = {{
      idle: "대기",
      takeover: "0203 기준정보 - TakeOver",
      handover: "0203 기준정보 - HandOver",
      rtb: "0203 기준정보 - RTB",
      flight_area: "0203 기준정보 - FlightArea",
      prohibited_area: "0203 기준정보 - ProhibitedArea",
      mission_pick: "0201 임무 입력 - Type",
      region_pick: "0201 임무 입력 - RegionType",
      mission_line: "0201 임무 입력 - LineList",
      mission_area: "0201 임무 입력 - AreaList",
      hole_pick: "0201 임무 입력 - isHole",
      target_place: "적 배치",
      mission_complete: "0201 임무 시퀀스 완료",
      width: "Line width 입력"
    }};
    const REFERENCE_STAGES = new Set(["takeover", "handover", "rtb", "flight_area", "prohibited_area"]);
    const MISSION_STAGES = new Set(["mission_pick", "region_pick", "mission_line", "mission_area", "hole_pick", "width"]);
    const GUIDED_MISSION_STEPS = {{
      1: [
        {{ missionType: 1, regionType: 3, shape: "line", requiredPoints: 2, label: "협업기동임무 / ACP", detail: "Line 점 2개를 찍으면 width 입력으로 넘어갑니다." }},
        {{ missionType: 1, regionType: 4, shape: "line", requiredPoints: 2, carryLast: true, label: "협업기동임무 / 공격대기지역", detail: "이전 임무 마지막점이 시작점입니다. 점 1개를 추가로 찍으면 width 입력으로 넘어갑니다." }},
        {{ missionType: 2, regionType: 4, shape: "area", label: "협업수색공격임무 / 공격대기지역", detail: "Area 점을 3개 이상 찍고 우클릭으로 완료합니다." }},
        {{ missionType: 2, regionType: 6, shape: "area", label: "협업수색공격임무 / 목표지역", detail: "Area 점을 3개 이상 찍고 우클릭으로 완료합니다." }},
        {{ missionType: 1, regionType: 3, shape: "line", requiredPoints: 2, label: "협업기동임무 / ACP", detail: "Line 점 2개를 찍으면 width 입력으로 넘어갑니다." }},
        {{ missionType: 1, regionType: 2, shape: "line", requiredPoints: 2, label: "협업기동임무 / 통제권변경지역", detail: "Line 점 2개를 찍으면 width 입력으로 넘어갑니다." }}
      ],
      2: [
        {{ missionType: 1, regionType: 3, shape: "line", requiredPoints: 2, label: "협업기동임무 / ACP", detail: "Line 점 2개를 찍으면 width 입력으로 넘어갑니다." }},
        {{ missionType: 1, regionType: 6, shape: "line", requiredPoints: 2, carryLast: true, label: "협업기동임무 / 목표지역", detail: "이전 임무 마지막점이 시작점입니다. 점 1개를 추가로 찍으면 width 입력으로 넘어갑니다." }},
        {{ missionType: 5, regionType: 6, shape: "area", label: "협업지상부대엄호임무 / 목표지역", detail: "Area 점을 3개 이상 찍고 우클릭으로 완료합니다." }},
        {{ missionType: 1, regionType: 7, shape: "line", requiredPoints: 2, mode: "package2_boundary_line", label: "협업기동임무 / 경계지역", detail: "경계지역 Line을 1~3개 입력합니다. 각 Line은 점 2개와 width를 입력합니다." }},
        {{ missionType: 3, regionType: 7, shape: "area", mode: "package2_boundary_area", label: "협업경계임무 / 경계지역", detail: "앞에서 입력한 경계지역 Line 개수만큼 Area를 입력합니다." }},
        {{ missionType: 1, regionType: 6, shape: "line", requiredPoints: 2, mode: "package2_target_line", label: "협업기동임무 / 목표지역", detail: "경계지역 Line 개수만큼 목표지역 Line을 입력합니다. 각 Line은 점 2개와 width를 입력합니다." }},
        {{ missionType: 1, regionType: 3, shape: "line", requiredPoints: 2, label: "협업기동임무 / ACP", detail: "Line 점 2개를 찍으면 width 입력으로 넘어갑니다." }},
        {{ missionType: 1, regionType: 2, shape: "line", requiredPoints: 2, label: "협업기동임무 / 통제권변경지역", detail: "Line 점 2개를 찍으면 width 입력으로 넘어갑니다." }}
      ],
      3: [
        {{ missionType: 1, regionType: 8, shape: "line", requiredPoints: 2, label: "협업기동임무 / 탑재지대", detail: "Line 점 2개를 찍으면 width 입력으로 넘어갑니다." }},
        {{ missionType: 1, regionType: 3, shape: "line", requiredPoints: 2, carryLast: true, label: "협업기동임무 / ACP", detail: "이전 임무 마지막점이 시작점입니다. 점 1개를 추가로 찍으면 width 입력으로 넘어갑니다." }},
        {{ missionType: 1, regionType: 9, shape: "line", requiredPoints: 2, carryLast: true, label: "협업기동임무 / 착륙지대", detail: "이전 임무 마지막점이 시작점입니다. 점 1개를 추가로 찍으면 width 입력으로 넘어갑니다." }},
        {{ missionType: 4, regionType: 9, shape: "area", label: "협업공중부대엄호임무 / 착륙지대", detail: "Area 점을 3개 이상 찍고 우클릭으로 완료합니다." }},
        {{ missionType: 1, regionType: 7, shape: "line", requiredPoints: 2, mode: "package3_boundary_line", label: "협업기동임무 / 경계지역", detail: "경계지역 Line을 1~3개 입력합니다. 각 Line은 점 2개와 width를 입력합니다." }},
        {{ missionType: 3, regionType: 7, shape: "area", mode: "package3_boundary_area", label: "협업경계임무 / 경계지역", detail: "앞에서 입력한 경계지역 Line 개수만큼 Area를 입력합니다." }},
        {{ missionType: 1, regionType: 9, shape: "line", requiredPoints: 2, mode: "package3_target_line", label: "협업기동임무 / 착륙지대", detail: "경계지역 Line 개수만큼 착륙지대 Line을 입력합니다. 각 Line은 점 2개와 width를 입력합니다." }},
        {{ missionType: 1, regionType: 3, shape: "line", requiredPoints: 2, label: "협업기동임무 / ACP", detail: "Line 점 2개를 찍으면 width 입력으로 넘어갑니다." }},
        {{ missionType: 1, regionType: 2, shape: "line", requiredPoints: 2, label: "협업기동임무 / 통제권변경지역", detail: "Line 점 2개를 찍으면 width 입력으로 넘어갑니다." }}
      ],
      4: [
        {{ missionType: 1, regionType: 4, shape: "line", requiredPoints: 2, label: "협업기동임무 / 공격대기지역", detail: "Line 점 2개를 찍으면 width 입력으로 넘어갑니다." }},
        {{ missionType: 2, regionType: 4, shape: "area", label: "협업수색공격임무 / 공격대기지역", detail: "Area 점을 3개 이상 찍고 우클릭으로 완료합니다." }},
        {{ missionType: 1, regionType: 7, shape: "line", requiredPoints: 2, label: "협업기동임무 / 경계지역", detail: "Line 점 2개를 찍으면 width 입력으로 넘어갑니다." }},
        {{ missionType: 3, regionType: 7, shape: "area", mode: "package4_boundary_area", forceHoleChoice: true, label: "협업경계임무 / 경계지역", detail: "경계지역 일반 Area와 내부 목표지역 Hole Area를 순서대로 입력합니다." }},
        {{ missionType: 1, regionType: 4, shape: "line", requiredPoints: 2, label: "협업기동임무 / 공격대기지역", detail: "Line 점 2개를 찍으면 width 입력으로 넘어갑니다." }},
        {{ missionType: 1, regionType: 3, shape: "line", requiredPoints: 2, label: "협업기동임무 / ACP", detail: "Line 점 2개를 찍으면 width 입력으로 넘어갑니다." }},
        {{ missionType: 1, regionType: 2, shape: "line", requiredPoints: 2, label: "협업기동임무 / 통제권변경지역", detail: "Line 점 2개를 찍으면 width 입력으로 넘어갑니다." }}
      ],
      5: [
        {{ missionType: 1, regionType: 4, shape: "line", requiredPoints: 2, label: "협업기동임무 / 공격대기지역", detail: "Line 점 2개를 찍으면 width 입력으로 넘어갑니다." }},
        {{ missionType: 2, regionType: 4, shape: "area", label: "협업수색공격임무 / 공격대기지역", detail: "Area 점을 3개 이상 찍고 우클릭으로 완료합니다." }},
        {{ missionType: 1, regionType: 11, shape: "line", requiredPoints: 2, label: "협업기동임무 / 도서지역", detail: "Line 점 2개를 찍으면 width 입력으로 넘어갑니다." }},
        {{ missionType: 6, regionType: 11, shape: "area", label: "협업도심수색공격임무 / 도서지역", detail: "Area 점을 3개 이상 찍고 우클릭으로 완료합니다." }},
        {{ missionType: 1, regionType: 4, shape: "line", requiredPoints: 2, label: "협업기동임무 / 공격대기지역", detail: "Line 점 2개를 찍으면 width 입력으로 넘어갑니다." }},
        {{ missionType: 1, regionType: 3, shape: "line", requiredPoints: 2, label: "협업기동임무 / ACP", detail: "Line 점 2개를 찍으면 width 입력으로 넘어갑니다." }},
        {{ missionType: 1, regionType: 2, shape: "line", requiredPoints: 2, label: "협업기동임무 / 통제권변경지역", detail: "Line 점 2개를 찍으면 width 입력으로 넘어갑니다." }}
      ]
    }};
    function point(lngLat) {{ return {{ latitude: Number(lngLat.lat.toFixed(8)), longitude: Number(lngLat.lng.toFixed(8)) }}; }}
    function positionCursorSupport() {{
      if (!cursorPoint) return;
      const canvas = map.getCanvas();
      const left = Math.max(8, Math.min(cursorPoint.x + 12, canvas.clientWidth - 190));
      const top = Math.max(8, Math.min(cursorPoint.y + 14, canvas.clientHeight - 78));
      cursorSupport.style.left = `${{left}}px`;
      cursorSupport.style.top = `${{top}}px`;
    }}
    function renderCursorSupport() {{
      if (!cursorLngLat || !cursorPoint) return;
      positionCursorSupport();
      const lat = Number(cursorLngLat.lat).toFixed(6);
      const lon = Number(cursorLngLat.lng).toFixed(6);
      cursorSupport.innerHTML = `<div class="coord-title">마우스 위치</div><div class="coord-row">위도 ${{lat}}</div><div class="coord-row">경도 ${{lon}}</div><div class="coord-row">${{cursorElevationText}}</div>`;
      cursorSupport.classList.add("is-active");
      cursorSupport.setAttribute("aria-hidden", "false");
    }}
    function hideCursorSupport() {{
      if (cursorElevationTimer) clearTimeout(cursorElevationTimer);
      cursorElevationSeq += 1;
      cursorLngLat = null;
      cursorPoint = null;
      cursorSupport.classList.remove("is-active");
      cursorSupport.setAttribute("aria-hidden", "true");
    }}
    function scheduleCursorElevation(lngLat) {{
      if (!config.demAvailable) {{
        cursorElevationText = "고도 N/A";
        return;
      }}
      if (cursorElevationTimer) clearTimeout(cursorElevationTimer);
      cursorElevationText = "고도 조회 중";
      const lat = Number(lngLat.lat).toFixed(8);
      const lon = Number(lngLat.lng).toFixed(8);
      const seq = ++cursorElevationSeq;
      cursorElevationTimer = setTimeout(() => {{
        fetch(`/api/elevation?lat=${{encodeURIComponent(lat)}}&lon=${{encodeURIComponent(lon)}}`)
          .then(res => res.json())
          .then(data => {{
            if (seq !== cursorElevationSeq) return;
            cursorElevationText = data.ground === null || data.ground === undefined ? "고도 N/A" : `고도 ${{Math.round(Number(data.ground))}}m`;
            renderCursorSupport();
          }})
          .catch(() => {{
            if (seq !== cursorElevationSeq) return;
            cursorElevationText = "고도 N/A";
            renderCursorSupport();
          }});
      }}, 120);
    }}
    function updateCursorSupport(e) {{
      cursorLngLat = e.lngLat;
      cursorPoint = e.point;
      scheduleCursorElevation(e.lngLat);
      renderCursorSupport();
    }}
    function stageLabel(value) {{ return STAGE_LABELS[value] || value; }}
    function referenceStarted() {{
      return Boolean(state.takeOver.length || state.handOver.length || state.rtb.length || state.flightAreas.length || state.prohibitedAreas.length || REFERENCE_STAGES.has(stage));
    }}
    function referenceReady() {{
      return state.takeOver.length >= 3 && state.handOver.length >= 3 && state.rtb.length > 0 && state.flightAreas.length > 0 && state.prohibitedAreas.length > 0;
    }}
    function missionStarted() {{
      return Boolean(state.missions.length || activeMission || MISSION_STAGES.has(stage));
    }}
    function setWorkflowRow(row, statusEl, detailEl, stateClass, status, detail) {{
      row.classList.remove("is-active", "is-done");
      if (stateClass) row.classList.add(stateClass);
      statusEl.textContent = status;
      detailEl.textContent = detail;
    }}
    function updateWorkflowPanel() {{
      const refDone = referenceReady();
      const refActive = REFERENCE_STAGES.has(stage);
      const refStatus = refDone ? "완료" : refActive ? "입력중" : referenceStarted() ? "부분 입력" : "미시작";
      const refClass = refDone ? "is-done" : refActive ? "is-active" : "";
      const refDetail = `TakeOver ${{Math.min(state.takeOver.length, 3)}}/3 · HandOver ${{Math.min(state.handOver.length, 3)}}/3 · RTB ${{state.rtb.length}} · FlightArea ${{state.flightAreas.length}} · Prohibited ${{state.prohibitedAreas.length}}`;
      setWorkflowRow(flow0203, flow0203Status, flow0203Detail, refClass, refStatus, refDetail);

      const missionActive = MISSION_STAGES.has(stage);
      const missionStatus = missionActive ? "입력중" : state.missions.length ? "작성됨" : "미시작";
      const missionClass = missionActive ? "is-active" : state.missions.length ? "is-done" : "";
      const activeName = activeMission
        ? (config.missionTypes[activeMission.inputMissionType] || `type=${{activeMission.inputMissionType}}`)
        : pendingRegionMissionType !== null
          ? (config.missionTypes[pendingRegionMissionType] || `type=${{pendingRegionMissionType}}`)
          : "";
      const missionDetail = missionActive
        ? `${{activeName || "임무 Type 선택 중"}} · 현재 점 ${{activePoints.length}}개 · 저장된 임무 ${{state.missions.length}}개`
        : `저장된 임무 ${{state.missions.length}}개${{refDone ? " · 임무 추가 가능" : " · 0203 기준정보 먼저 완료"}}`;
      setWorkflowRow(flow0201, flow0201Status, flow0201Detail, missionClass, missionStatus, missionDetail);
    }}
    function progressLabel() {{
      if (stage === "takeover") return `TakeOverInfoList: ${{state.takeOver.length}}/3`;
      if (stage === "handover") return `HandOverInfoList: ${{state.handOver.length}}/3`;
      if (stage === "rtb") return `RTBCoordinateList: ${{state.rtb.length}}개`;
      if (stage === "flight_area") return `FlightArea 점: ${{activePoints.length}}개`;
      if (stage === "prohibited_area") return `ProhibitedArea 점: ${{activePoints.length}}개`;
      if (stage === "region_pick") return "RegionType 선택 중";
      if (stage === "mission_line") return `LineList 점: ${{activePoints.length}}개`;
      if (stage === "mission_area") return `AreaList 점: ${{activePoints.length}}개`;
      if (stage === "hole_pick") return "isHole 선택 중";
      if (stage === "target_place") return `TargetInfo: ${{state.targets.length}}개`;
      if (stage === "mission_complete") return `InputMissionList: ${{state.missions.length}}개 완료`;
      if (stage === "width") return `width: ${{widthValue || 0}}m`;
      return `InputMissionList: ${{state.missions.length}}개`;
    }}
    function updateModePanel(text) {{
      modeTitle.textContent = stageLabel(stage);
      if (typeof text === "string") modeDetail.textContent = text;
      modeProgress.textContent = progressLabel();
      updateWorkflowPanel();
    }}
    function setInstruction(text) {{
      instruction.textContent = text;
      updateModePanel(text);
    }}
    function showNotice(text) {{
      saveStatus.textContent = text;
      setInstruction(text);
      return false;
    }}
    function hideChoicePanel() {{
      choicePanel.classList.remove("is-active");
      choicePanel.hidden = true;
      choicePanel.replaceChildren();
    }}
    function hideSavePanel() {{
      savePanel.classList.remove("is-active");
      savePanel.hidden = true;
    }}
    function hideLoadPanel() {{
      loadPanel.classList.remove("is-active");
      loadPanel.hidden = true;
    }}
    function showSavePanel() {{
      hideChoicePanel();
      hideLoadPanel();
      hideTypePicker();
      hideRegionPicker();
      hideHolePicker();
      saveNameInput.value = "";
      savePanel.hidden = false;
      savePanel.classList.add("is-active");
      saveStatus.textContent = "저장 이름 입력 대기";
      setInstruction("저장 이름을 입력하세요. 비워두면 날짜시간으로 자동 생성됩니다.");
      setTimeout(() => saveNameInput.focus(), 0);
    }}
    function scenarioOptionLabel(item) {{
      const typeText = item.packageType ? `Type ${{item.packageType}}` : "Type -";
      const missionText = `${{item.missionCount || 0}}개 임무`;
      const targetText = `${{item.targetCount || 0}}개 적`;
      return `${{item.label || item.name}} · ${{typeText}} · ${{missionText}} · ${{targetText}} · ${{item.modifiedText || ""}}`;
    }}
    function updateLoadMeta() {{
      const selected = scenarioSelect.selectedOptions[0];
      if (!selected) {{
        loadMeta.textContent = "불러올 저장본이 없습니다.";
        loadConfirm.disabled = true;
        return;
      }}
      loadConfirm.disabled = false;
      loadMeta.textContent = selected.dataset.meta || "";
    }}
    async function refreshScenarioList() {{
      scenarioSelect.replaceChildren();
      loadMeta.textContent = "목록을 불러오는 중...";
      loadConfirm.disabled = true;
      try {{
        const res = await fetch("/api/scenarios", {{ cache: "no-store" }});
        const data = await res.json();
        if (!data.ok) throw new Error(data.error || "시나리오 목록을 불러오지 못했습니다.");
        const rows = data.scenarios || [];
        rows.forEach((item) => {{
          const opt = document.createElement("option");
          opt.value = item.loadKey || item.name;
          opt.textContent = scenarioOptionLabel(item);
          opt.dataset.meta = `${{item.path || "Logs/GeneratedScenario"}} · PackageID=${{item.packageID || "-"}} · ${{item.packageTypeLabel || ""}}`;
          scenarioSelect.appendChild(opt);
        }});
        if (!rows.length) {{
          const opt = document.createElement("option");
          opt.value = "";
          opt.textContent = "저장된 시나리오 없음";
          scenarioSelect.appendChild(opt);
        }}
        updateLoadMeta();
      }} catch (err) {{
        loadMeta.textContent = err.message || "목록 로딩 실패";
        loadConfirm.disabled = true;
      }}
    }}
    function showLoadPanel() {{
      hideChoicePanel();
      hideSavePanel();
      hideTypePicker();
      hideRegionPicker();
      hideHolePicker();
      loadPanel.hidden = false;
      loadPanel.classList.add("is-active");
      saveStatus.textContent = "불러오기 대기";
      setInstruction("저장된 시나리오를 선택한 뒤 불러오기를 누르세요.");
      refreshScenarioList();
    }}
    function renderChoicePanel(title, detail, choices) {{
      choicePanel.replaceChildren();
      const titleEl = document.createElement("div");
      titleEl.className = "choice-title";
      titleEl.textContent = title;
      const detailEl = document.createElement("div");
      detailEl.className = "choice-detail";
      detailEl.textContent = detail;
      const grid = document.createElement("div");
      grid.className = "choice-grid";
      choices.forEach((choice) => {{
        const button = document.createElement("button");
        button.type = "button";
        button.textContent = choice.label;
        if (choice.secondary) button.classList.add("secondary");
        button.addEventListener("click", choice.action);
        grid.appendChild(button);
      }});
      choicePanel.append(titleEl, detailEl, grid);
      choicePanel.hidden = false;
      choicePanel.classList.add("is-active");
      updateModePanel(detail);
    }}
    function setStage(next, text) {{
      stage = next;
      hideChoicePanel();
      hideSavePanel();
      hideLoadPanel();
      setInstruction(text);
      hideTypePicker();
      hideRegionPicker();
      hideHolePicker();
      if (stage !== "width") clearWidthPreview();
      render();
    }}
    function finishStage() {{
      if (stage === "rtb") {{
        if (!state.rtb.length) return showNotice("RTBCoordinateList[]가 1개 이상 필요합니다.");
        return setStage("flight_area", "FlightAreaList[]를 지정해주세요(점 N개). 좌클릭으로 점을 찍고 우클릭으로 영역 생성.");
      }}
      if (stage === "flight_area") return finalizeReferenceArea(false);
      if (stage === "prohibited_area") return finalizeReferenceArea(true);
      if (stage === "mission_line") return beginWidth();
      if (stage === "mission_area") return finalizeMissionArea();
      if (stage === "target_place") return startNextMissionInput("적 배치를 종료했습니다. 다음 임무 시작 위치를 클릭하세요.");
      if (stage === "width") return showNotice("라인 주변을 드래그해서 width를 확정하세요.");
    }}
    function begin0203Input(text) {{
      state.takeOver = []; state.handOver = []; state.rtb = []; state.flightAreas = []; state.prohibitedAreas = [];
      state.guidedMeta = {{}};
      activePoints = []; activeMission = null; pendingRegionMissionType = null; pendingRegionLngLat = null;
      setStage("takeover", text || "TakeOverInfoList[]를 선정하세요(점 3개). UAV4, UAV5, UAV6 순서.");
    }}
    function addMission() {{
      if (!referenceReady()) return showNotice("0203 기준정보를 먼저 완료하세요. 오른쪽 0203 기준정보가 '완료'가 되면 0201 임무를 추가할 수 있습니다.");
      if (guidedSteps()) {{
        if (MISSION_STAGES.has(stage)) return showNotice("현재 작전 타입은 고정 임무 시퀀스입니다. 진행 중인 단계를 먼저 완료하세요.");
        if (!currentGuidedStep()) return showNotice(`Type ${{selectedPackageType()}} 0201 임무 시퀀스가 이미 완료되었습니다.`);
        return startGuidedMissionInput("현재 작전 타입은 고정 임무 시퀀스를 따릅니다.");
      }}
      if (!["idle", "mission_pick", "target_place"].includes(stage)) return showNotice("현재 입력 단계를 완료한 뒤 임무를 추가하세요.");
      activePoints = []; activeMission = null; pendingRegionMissionType = null; pendingRegionLngLat = null;
      startNextMissionInput("지도에서 임무 시작 위치를 클릭한 뒤 원형 Type 버튼을 선택하세요.");
    }}
    function startNextMissionInput(text) {{
      if (guidedSteps()) return startGuidedMissionInput(text);
      activePoints = [];
      activeMission = null;
      pendingRegionMissionType = null;
      pendingRegionLngLat = null;
      setStage("mission_pick", text || "다음 임무 시작 위치를 클릭한 뒤 Type을 선택하세요. 저장하려면 오른쪽 위 저장 버튼을 누르세요.");
    }}
    function selectedPackageType() {{
      return Number(packageSelect.value || state.packageType || 1);
    }}
    function guidedSteps() {{
      return GUIDED_MISSION_STEPS[selectedPackageType()] || null;
    }}
    function guidedStepCount() {{
      const steps = guidedSteps();
      return steps ? steps.length : 0;
    }}
    function guidedMeta() {{
      if (!state.guidedMeta || typeof state.guidedMeta !== "object") state.guidedMeta = {{}};
      return state.guidedMeta;
    }}
    function package2BoundaryLineMission() {{
      const mission = state.missions[3];
      return mission && mission.inputMissionType === 1 && mission.regionType === 7 ? mission : null;
    }}
    function package2BoundaryMission() {{
      const mission = state.missions[4];
      return mission && mission.inputMissionType === 3 && mission.regionType === 7 ? mission : null;
    }}
    function package2TargetLineMission() {{
      const mission = state.missions[5];
      return mission && mission.inputMissionType === 1 && mission.regionType === 6 ? mission : null;
    }}
    function ensurePackage2BoundaryLineMission() {{
      let mission = package2BoundaryLineMission();
      if (!mission) {{
        mission = {{ inputMissionType: 1, regionType: 7, areaList: [], lineList: [], coordinateList: [] }};
        if (state.missions.length <= 3) state.missions.push(mission);
        else state.missions.splice(3, 0, mission);
      }}
      if (!Array.isArray(mission.areaList)) mission.areaList = [];
      if (!Array.isArray(mission.lineList)) mission.lineList = [];
      if (!Array.isArray(mission.coordinateList)) mission.coordinateList = [];
      return mission;
    }}
    function ensurePackage2BoundaryMission() {{
      let mission = package2BoundaryMission();
      if (!mission) {{
        mission = {{ inputMissionType: 3, regionType: 7, areaList: [], lineList: [], coordinateList: [] }};
        if (state.missions.length <= 4) state.missions.push(mission);
        else state.missions.splice(4, 0, mission);
      }}
      if (!Array.isArray(mission.areaList)) mission.areaList = [];
      if (!Array.isArray(mission.lineList)) mission.lineList = [];
      if (!Array.isArray(mission.coordinateList)) mission.coordinateList = [];
      return mission;
    }}
    function ensurePackage2TargetLineMission() {{
      let mission = package2TargetLineMission();
      if (!mission) {{
        mission = {{ inputMissionType: 1, regionType: 6, areaList: [], lineList: [], coordinateList: [] }};
        if (state.missions.length <= 5) state.missions.push(mission);
        else state.missions.splice(5, 0, mission);
      }}
      if (!Array.isArray(mission.areaList)) mission.areaList = [];
      if (!Array.isArray(mission.lineList)) mission.lineList = [];
      if (!Array.isArray(mission.coordinateList)) mission.coordinateList = [];
      return mission;
    }}
    function package2BoundaryLineCount() {{
      const mission = package2BoundaryLineMission();
      return mission && Array.isArray(mission.lineList) ? mission.lineList.length : 0;
    }}
    function package2BoundaryCount() {{
      const mission = package2BoundaryMission();
      return mission && Array.isArray(mission.areaList) ? mission.areaList.length : 0;
    }}
    function package2TargetLineCount() {{
      const mission = package2TargetLineMission();
      return mission && Array.isArray(mission.lineList) ? mission.lineList.length : 0;
    }}
    function package3BoundaryMission() {{
      const mission = state.missions[5];
      return mission && mission.inputMissionType === 3 && mission.regionType === 7 ? mission : null;
    }}
    function package3TargetLineMission() {{
      const mission = state.missions[6];
      return mission && mission.inputMissionType === 1 && mission.regionType === 9 ? mission : null;
    }}
    function package3BoundaryLineMission() {{
      const mission = state.missions[4];
      return mission && mission.inputMissionType === 1 && mission.regionType === 7 ? mission : null;
    }}
    function ensurePackage3BoundaryLineMission() {{
      let mission = package3BoundaryLineMission();
      if (!mission) {{
        mission = {{ inputMissionType: 1, regionType: 7, areaList: [], lineList: [], coordinateList: [] }};
        if (state.missions.length <= 4) state.missions.push(mission);
        else state.missions.splice(4, 0, mission);
      }}
      if (!Array.isArray(mission.areaList)) mission.areaList = [];
      if (!Array.isArray(mission.lineList)) mission.lineList = [];
      if (!Array.isArray(mission.coordinateList)) mission.coordinateList = [];
      return mission;
    }}
    function ensurePackage3BoundaryMission() {{
      let mission = package3BoundaryMission();
      if (!mission) {{
        mission = {{ inputMissionType: 3, regionType: 7, areaList: [], lineList: [], coordinateList: [] }};
        if (state.missions.length <= 5) state.missions.push(mission);
        else state.missions.splice(5, 0, mission);
      }}
      if (!Array.isArray(mission.areaList)) mission.areaList = [];
      if (!Array.isArray(mission.lineList)) mission.lineList = [];
      if (!Array.isArray(mission.coordinateList)) mission.coordinateList = [];
      return mission;
    }}
    function ensurePackage3TargetLineMission() {{
      let mission = package3TargetLineMission();
      if (!mission) {{
        mission = {{ inputMissionType: 1, regionType: 9, areaList: [], lineList: [], coordinateList: [] }};
        if (state.missions.length <= 6) state.missions.push(mission);
        else state.missions.splice(6, 0, mission);
      }}
      if (!Array.isArray(mission.areaList)) mission.areaList = [];
      if (!Array.isArray(mission.lineList)) mission.lineList = [];
      if (!Array.isArray(mission.coordinateList)) mission.coordinateList = [];
      return mission;
    }}
    function package3BoundaryLineCount() {{
      const mission = package3BoundaryLineMission();
      return mission && Array.isArray(mission.lineList) ? mission.lineList.length : 0;
    }}
    function package3BoundaryCount() {{
      const mission = package3BoundaryMission();
      return mission && Array.isArray(mission.areaList) ? mission.areaList.length : 0;
    }}
    function package3TargetLineCount() {{
      const mission = package3TargetLineMission();
      return mission && Array.isArray(mission.lineList) ? mission.lineList.length : 0;
    }}
    function package4BoundaryMission() {{
      const mission = state.missions[3];
      return mission && mission.inputMissionType === 3 && mission.regionType === 7 ? mission : null;
    }}
    function ensurePackage4BoundaryMission() {{
      let mission = package4BoundaryMission();
      if (!mission) {{
        mission = {{ inputMissionType: 3, regionType: 7, areaList: [], lineList: [], coordinateList: [] }};
        if (state.missions.length <= 3) state.missions.push(mission);
        else state.missions.splice(3, 0, mission);
      }}
      if (!Array.isArray(mission.areaList)) mission.areaList = [];
      if (!Array.isArray(mission.lineList)) mission.lineList = [];
      if (!Array.isArray(mission.coordinateList)) mission.coordinateList = [];
      return mission;
    }}
    function package4BoundaryAreas() {{
      const mission = package4BoundaryMission();
      return mission && Array.isArray(mission.areaList) ? mission.areaList : [];
    }}
    function package4BoundaryHasOuter() {{
      return package4BoundaryAreas().some(row => row && !Boolean(row.isHole));
    }}
    function package4BoundaryHasHole() {{
      return package4BoundaryAreas().some(row => row && Boolean(row.isHole));
    }}
    function currentPackage2GuidedStep() {{
      const steps = GUIDED_MISSION_STEPS[2] || [];
      const missionCount = state.missions.length;
      if (missionCount < 3) {{
        const step = steps[missionCount];
        return step ? {{ ...step, stepIndex: missionCount }} : null;
      }}
      const meta = guidedMeta();
      const boundaryLineCount = package2BoundaryLineCount();
      if (boundaryLineCount >= 3) meta.package2BoundaryLineDone = true;
      if (!meta.package2BoundaryLineDone) {{
        const nextLine = Math.min(boundaryLineCount + 1, 3);
        return {{
          ...steps[3],
          stepIndex: 3,
          repeatIndex: nextLine,
          repeatTotal: 3,
          detail: `경계지역 Line ${{nextLine}}/3을 입력합니다. 점 2개를 찍고 width를 입력합니다.`
        }};
      }}
      const repeatTotal = Math.max(1, Math.min(3, boundaryLineCount));
      const boundaryCount = package2BoundaryCount();
      if (boundaryCount >= repeatTotal) meta.package2BoundaryDone = true;
      if (boundaryCount < repeatTotal) {{
        const nextArea = boundaryCount + 1;
        return {{
          ...steps[4],
          stepIndex: 4,
          repeatIndex: nextArea,
          repeatTotal,
          detail: `경계지역 Line ${{repeatTotal}}개와 순서대로 매칭할 경계 Area ${{nextArea}}/${{repeatTotal}}을 입력합니다. Area 점을 3개 이상 찍고 우클릭으로 완료합니다.`
        }};
      }}
      const targetLineCount = package2TargetLineCount();
      if (targetLineCount < repeatTotal) {{
        const nextLine = targetLineCount + 1;
        return {{
          ...steps[5],
          stepIndex: 5,
          repeatIndex: nextLine,
          repeatTotal,
          detail: `경계지역 Line ${{repeatTotal}}개 기준 목표지역 Line ${{nextLine}}/${{repeatTotal}}을 입력합니다. 점 2개를 찍고 width를 입력합니다.`
        }};
      }}
      const nextStepIndex = 6 + Math.max(0, missionCount - 6);
      const step = steps[nextStepIndex];
      return step ? {{ ...step, stepIndex: nextStepIndex }} : null;
    }}
    function currentPackage3GuidedStep() {{
      const steps = GUIDED_MISSION_STEPS[3] || [];
      const missionCount = state.missions.length;
      if (missionCount < 4) {{
        const step = steps[missionCount];
        return step ? {{ ...step, stepIndex: missionCount }} : null;
      }}
      const meta = guidedMeta();
      const boundaryLineCount = package3BoundaryLineCount();
      if (boundaryLineCount >= 3) meta.package3BoundaryLineDone = true;
      if (!meta.package3BoundaryLineDone) {{
        const nextLine = Math.min(boundaryLineCount + 1, 3);
        return {{
          ...steps[4],
          stepIndex: 4,
          repeatIndex: nextLine,
          repeatTotal: 3,
          detail: `경계지역 Line ${{nextLine}}/3을 입력합니다. 점 2개를 찍고 width를 입력합니다.`
        }};
      }}
      const repeatTotal = Math.max(1, Math.min(3, boundaryLineCount));
      const boundaryCount = package3BoundaryCount();
      if (boundaryCount >= repeatTotal) meta.package3BoundaryDone = true;
      if (boundaryCount < repeatTotal) {{
        const nextArea = boundaryCount + 1;
        return {{
          ...steps[5],
          stepIndex: 5,
          repeatIndex: nextArea,
          repeatTotal,
          detail: `경계지역 Line ${{repeatTotal}}개와 순서대로 매칭할 경계 Area ${{nextArea}}/${{repeatTotal}}을 입력합니다. Area 점을 3개 이상 찍고 우클릭으로 완료합니다.`
        }};
      }}
      const targetLineCount = package3TargetLineCount();
      if (targetLineCount < repeatTotal) {{
        const nextLine = targetLineCount + 1;
        return {{
          ...steps[6],
          stepIndex: 6,
          repeatIndex: nextLine,
          repeatTotal,
          detail: `경계지역 Line ${{repeatTotal}}개 기준 착륙지대 Line ${{nextLine}}/${{repeatTotal}}을 입력합니다. 점 2개를 찍고 width를 입력합니다.`
        }};
      }}
      const nextStepIndex = 7 + Math.max(0, missionCount - 7);
      const step = steps[nextStepIndex];
      return step ? {{ ...step, stepIndex: nextStepIndex }} : null;
    }}
    function currentPackage4GuidedStep() {{
      const steps = GUIDED_MISSION_STEPS[4] || [];
      const missionCount = state.missions.length;
      if (missionCount < 3) {{
        const step = steps[missionCount];
        return step ? {{ ...step, stepIndex: missionCount }} : null;
      }}
      const hasOuter = package4BoundaryHasOuter();
      const hasHole = package4BoundaryHasHole();
      if (!hasOuter || !hasHole) {{
        const expectHole = hasOuter && !hasHole;
        return {{
          ...steps[3],
          stepIndex: 3,
          repeatIndex: expectHole ? 2 : 1,
          repeatTotal: 2,
          expectedHole: expectHole,
          detail: expectHole
            ? "목표지역으로 사용할 내부 Area를 입력하고 Hole Area(true)를 선택합니다."
            : "경계지역 외곽 Area를 입력하고 일반 Area(false)를 선택합니다."
        }};
      }}
      guidedMeta().package4BoundaryDone = true;
      const nextStepIndex = 4 + Math.max(0, missionCount - 4);
      const step = steps[nextStepIndex];
      return step ? {{ ...step, stepIndex: nextStepIndex }} : null;
    }}
    function guidedStepTitle(stepIndex, step) {{
      const repeat = step.repeatTotal ? ` (${{step.repeatIndex}}/${{step.repeatTotal}})` : "";
      return `Type ${{selectedPackageType()}} 임무 ${{stepIndex + 1}}/${{guidedStepCount()}}${{repeat}} · ${{step.label}}`;
    }}
    function lastMissionPoint() {{
      const mission = state.missions[state.missions.length - 1];
      if (!mission) return null;
      const lines = mission.lineList || [];
      if (lines.length) {{
        const pts = lines[lines.length - 1].points || [];
        return pts.length ? {{ ...pts[pts.length - 1] }} : null;
      }}
      const areas = mission.areaList || [];
      if (areas.length) {{
        const pts = areas[areas.length - 1].points || [];
        return pts.length ? {{ ...pts[pts.length - 1] }} : null;
      }}
      const coords = mission.coordinateList || [];
      return coords.length ? {{ ...coords[coords.length - 1] }} : null;
    }}
    function currentGuidedStep() {{
      const steps = guidedSteps();
      if (!steps) return null;
      if (selectedPackageType() === 2) return currentPackage2GuidedStep();
      if (selectedPackageType() === 3) return currentPackage3GuidedStep();
      if (selectedPackageType() === 4) return currentPackage4GuidedStep();
      const stepIndex = state.missions.length;
      return steps[stepIndex] ? {{ ...steps[stepIndex], stepIndex }} : null;
    }}
    function startGuidedMissionInput(text) {{
      const guided = currentGuidedStep();
      activePoints = [];
      activeMission = null;
      pendingRegionMissionType = null;
      pendingRegionLngLat = null;
      if (!guided) {{
        return setStage("mission_complete", text || `Type ${{selectedPackageType()}} 0201 임무 시퀀스가 완료되었습니다. 저장하거나 적 배치를 진행하세요.`);
      }}
      if (guided.carryLast) {{
        const lastPoint = lastMissionPoint();
        if (!lastPoint) return showNotice("이전 임무 마지막점을 찾을 수 없어 다음 시퀀스를 시작할 수 없습니다.");
        activePoints = [lastPoint];
      }}
      activeMission = {{ inputMissionType: guided.missionType, regionType: guided.regionType, guided: true, guidedStepIndex: guided.stepIndex, guidedMode: guided.mode || "", forceHoleChoice: Boolean(guided.forceHoleChoice), expectedHole: typeof guided.expectedHole === "boolean" ? guided.expectedHole : null }};
      const prefix = text ? `${{text}} ` : "";
      const instructionText = `${{prefix}}${{guidedStepTitle(guided.stepIndex, guided)}}: ${{guided.detail}}`;
      return setStage(guided.shape === "line" ? "mission_line" : "mission_area", instructionText);
    }}
    function finishGuidedMission(message) {{
      if (guidedSteps()) return startGuidedMissionInput(message);
      return startNextMissionInput(message);
    }}
    function updateTargetMissionOptions() {{
      const previous = targetMissionSelect.value || "0";
      targetMissionSelect.replaceChildren();
      const auto = document.createElement("option");
      auto.value = "0";
      auto.textContent = "자동";
      targetMissionSelect.appendChild(auto);
      state.missions.forEach((mission, idx) => {{
        const missionId = idx + 1;
        const opt = document.createElement("option");
        opt.value = String(missionId);
        opt.textContent = `#${{missionId}} type=${{mission.inputMissionType}}`;
        targetMissionSelect.appendChild(opt);
      }});
      const values = Array.from(targetMissionSelect.options).map(opt => opt.value);
      targetMissionSelect.value = values.includes(previous) ? previous : "0";
    }}
    function selectedTargetMissionID() {{
      const selected = Number(targetMissionSelect.value || 0);
      if (Number.isFinite(selected) && selected > 0) return selected;
      return Math.max(1, state.missions.length);
    }}
    function startTargetPlacement() {{
      if (!state.missions.length) return showNotice("0201 임무를 먼저 1개 이상 입력하세요. 적은 관련 임무 ID가 필요합니다.");
      activePoints = [];
      activeMission = null;
      pendingRegionMissionType = null;
      pendingRegionLngLat = null;
      setStage("target_place", "지도에서 적 위치를 클릭하세요. 상단의 적 Type과 관련 임무를 기준으로 저장됩니다.");
    }}
    function addTargetAt(lngLat) {{
      const p = point(lngLat);
      const targetID = state.targets.length + 1;
      const targetType = Number(targetTypeSelect.value || 1);
      const inputMissionID = selectedTargetMissionID();
      state.targets.push({{ targetID, targetType, inputMissionID, location: p }});
      setInstruction(`적 #${{targetID}} 배치 완료 · type=${{targetType}} · mission=${{inputMissionID}}`);
      render();
    }}
    function resetAll() {{
      pushUndo("초기화");
      state.takeOver = []; state.handOver = []; state.rtb = []; state.flightAreas = []; state.prohibitedAreas = []; state.missions = []; state.targets = [];
      activePoints = []; activeMission = null; pendingRegionMissionType = null; pendingRegionLngLat = null; state.packageID = null; state.guidedMeta = {{}};
      begin0203Input("초기화 완료. TakeOverInfoList[]부터 다시 입력하세요(점 3개). UAV4, UAV5, UAV6 순서.");
    }}
    function finalizeReferenceArea(prohibited) {{
      if (activePoints.length < 3) return showNotice("영역은 점 3개 이상 필요합니다.");
      const areaPoints = normalizeAreaPoints(activePoints);
      if (prohibited) {{
        state.prohibitedAreas.push(areaPoints);
        activePoints = [];
        startNextMissionInput("0203 입력 완료. 이제 지도에서 첫 임무 시작 위치를 클릭하세요.");
      }} else {{
        state.flightAreas.push(areaPoints);
        activePoints = [];
        setStage("prohibited_area", "ProhibitedAreaList[]를 지정해주세요(점 N개). 좌클릭 후 우클릭으로 영역 생성.");
      }}
    }}
    function selectMissionType(missionType, lngLat) {{
      showRegionPicker(missionType, lngLat);
    }}
    function chooseRegionType(missionType, lngLat, region) {{
      if (!Number.isFinite(region)) return setStage("idle", "임무 추가가 취소되었습니다.");
      activePoints = [point(lngLat)];
      activeMission = {{ inputMissionType: missionType, regionType: region }};
      pendingRegionMissionType = null;
      pendingRegionLngLat = null;
      if (missionType === 0) {{
        state.missions.push({{ inputMissionType: 0, regionType: region, lineList: [], areaList: [], coordinateList: [] }});
        activePoints = []; activeMission = null;
        return startNextMissionInput("Not used 임무가 추가되었습니다. 다음 임무 시작 위치를 클릭하세요.");
      }}
      if (missionType === 1 || missionType === 7) return setStage("mission_line", `${{config.missionTypes[missionType]}}: LineList[] 점을 추가하고 우클릭 후 width를 드래그하세요.`);
      return setStage("mission_area", `${{config.missionTypes[missionType]}}: AreaList[] 점을 추가하고 우클릭으로 영역 생성.`);
    }}
    function finalizeMissionArea() {{
      if (!activeMission || activePoints.length < 3) return showNotice("AreaList[]는 점 3개 이상 필요합니다.");
      if (activeMission.guided && activeMission.forceHoleChoice) return showHolePicker();
      if (activeMission.guided) return completeMissionArea(false);
      if (activeMission.inputMissionType === 2 || activeMission.inputMissionType === 3) {{
        return showHolePicker();
      }}
      return completeMissionArea(false);
    }}
    function promptPackage2BoundaryLineDecision(count) {{
      stage = "mission_complete";
      hideTypePicker();
      hideRegionPicker();
      hideHolePicker();
      clearWidthPreview();
      setInstruction(`경계지역 Line ${{count}}/3 저장 완료. 경계 Line을 더 추가하거나 경계 Area 입력으로 진행하세요.`);
      renderChoicePanel("경계지역 Line 입력", `현재 경계 Line ${{count}}개가 같은 협업기동임무 lineList에 저장됩니다.`, [
        {{
          label: `경계 Line 추가 (${{count + 1}}/3)`,
          action: () => {{
            hideChoicePanel();
            guidedMeta().package2BoundaryLineDone = false;
            startGuidedMissionInput(`경계 Line ${{count + 1}}/3 입력을 시작합니다.`);
          }}
        }},
        {{
          label: "Area 입력으로 진행",
          secondary: true,
          action: () => {{
            pushUndo("경계 Line 입력 종료");
            guidedMeta().package2BoundaryLineDone = true;
            hideChoicePanel();
            startGuidedMissionInput(`경계 Line ${{count}}개 기준으로 경계 Area 입력을 시작합니다.`);
          }}
        }}
      ]);
      render();
    }}
    function completePackage2BoundaryLine(width) {{
      const mission = ensurePackage2BoundaryLineMission();
      mission.lineList.push({{ width, points: activePoints.slice() }});
      activePoints = [];
      activeMission = null;
      widthLine = [];
      suppressMapClickUntil = Date.now() + 350;
      const count = mission.lineList.length;
      if (count >= 3) {{
        guidedMeta().package2BoundaryLineDone = true;
        return finishGuidedMission(`경계지역 Line ${{count}}/3 저장 완료.`);
      }}
      return promptPackage2BoundaryLineDecision(count);
    }}
    function completePackage2BoundaryArea(areaPoints, isHole) {{
      const targetTotal = Math.max(1, Math.min(3, package2BoundaryLineCount()));
      const mission = ensurePackage2BoundaryMission();
      mission.areaList.push({{ isHole: Boolean(isHole), points: areaPoints }});
      activePoints = [];
      activeMission = null;
      const count = mission.areaList.length;
      if (count < targetTotal) {{
        return startGuidedMissionInput(`경계 Area ${{count}}/${{targetTotal}} 저장 완료.`);
      }}
      if (count >= targetTotal) {{
        guidedMeta().package2BoundaryDone = true;
        return finishGuidedMission(`경계 Area ${{count}}/${{targetTotal}} 저장 완료.`);
      }}
    }}
    function completePackage2TargetLine(width) {{
      const targetTotal = Math.max(1, Math.min(3, package2BoundaryLineCount() || package2BoundaryCount()));
      const mission = ensurePackage2TargetLineMission();
      mission.lineList.push({{ width, points: activePoints.slice() }});
      activePoints = [];
      activeMission = null;
      widthLine = [];
      suppressMapClickUntil = Date.now() + 350;
      const count = mission.lineList.length;
      if (count < targetTotal) {{
        return startGuidedMissionInput(`목표지역 Line ${{count}}/${{targetTotal}} 저장 완료.`);
      }}
      return finishGuidedMission(`목표지역 Line ${{count}}/${{targetTotal}} 저장 완료.`);
    }}
    function promptPackage3BoundaryLineDecision(count) {{
      stage = "mission_complete";
      hideTypePicker();
      hideRegionPicker();
      hideHolePicker();
      clearWidthPreview();
      setInstruction(`경계지역 Line ${{count}}/3 저장 완료. 경계 Line을 더 추가하거나 경계 Area 입력으로 진행하세요.`);
      renderChoicePanel("경계지역 Line 입력", `현재 경계 Line ${{count}}개가 같은 협업기동임무 lineList에 저장됩니다.`, [
        {{
          label: `경계 Line 추가 (${{count + 1}}/3)`,
          action: () => {{
            hideChoicePanel();
            guidedMeta().package3BoundaryLineDone = false;
            startGuidedMissionInput(`경계 Line ${{count + 1}}/3 입력을 시작합니다.`);
          }}
        }},
        {{
          label: "Area 입력으로 진행",
          secondary: true,
          action: () => {{
            pushUndo("경계 Line 입력 종료");
            guidedMeta().package3BoundaryLineDone = true;
            hideChoicePanel();
            startGuidedMissionInput(`경계 Line ${{count}}개 기준으로 경계 Area 입력을 시작합니다.`);
          }}
        }}
      ]);
      render();
    }}
    function completePackage3BoundaryLine(width) {{
      const mission = ensurePackage3BoundaryLineMission();
      mission.lineList.push({{ width, points: activePoints.slice() }});
      activePoints = [];
      activeMission = null;
      widthLine = [];
      suppressMapClickUntil = Date.now() + 350;
      const count = mission.lineList.length;
      if (count >= 3) {{
        guidedMeta().package3BoundaryLineDone = true;
        return finishGuidedMission(`경계지역 Line ${{count}}/3 저장 완료.`);
      }}
      return promptPackage3BoundaryLineDecision(count);
    }}
    function completePackage3BoundaryArea(areaPoints, isHole) {{
      const targetTotal = Math.max(1, Math.min(3, package3BoundaryLineCount()));
      const mission = ensurePackage3BoundaryMission();
      mission.areaList.push({{ isHole: Boolean(isHole), points: areaPoints }});
      activePoints = [];
      activeMission = null;
      const count = mission.areaList.length;
      if (count < targetTotal) {{
        return startGuidedMissionInput(`경계 Area ${{count}}/${{targetTotal}} 저장 완료.`);
      }}
      if (count >= targetTotal) {{
        guidedMeta().package3BoundaryDone = true;
        return finishGuidedMission(`경계 Area ${{count}}/${{targetTotal}} 저장 완료.`);
      }}
    }}
    function completePackage3TargetLine(width) {{
      const targetTotal = Math.max(1, Math.min(3, package3BoundaryLineCount() || package3BoundaryCount()));
      const mission = ensurePackage3TargetLineMission();
      mission.lineList.push({{ width, points: activePoints.slice() }});
      activePoints = [];
      activeMission = null;
      widthLine = [];
      suppressMapClickUntil = Date.now() + 350;
      const count = mission.lineList.length;
      if (count < targetTotal) {{
        return startGuidedMissionInput(`착륙지대 Line ${{count}}/${{targetTotal}} 저장 완료.`);
      }}
      return finishGuidedMission(`착륙지대 Line ${{count}}/${{targetTotal}} 저장 완료.`);
    }}
    function completePackage4BoundaryArea(areaPoints, isHole) {{
      const mission = ensurePackage4BoundaryMission();
      mission.areaList.push({{ isHole: Boolean(isHole), points: areaPoints }});
      activePoints = [];
      activeMission = null;
      const hasOuter = package4BoundaryHasOuter();
      const hasHole = package4BoundaryHasHole();
      if (!hasOuter) {{
        return startGuidedMissionInput("목표지역 Hole Area 저장 완료. 경계지역 일반 Area를 이어서 입력하세요.");
      }}
      if (!hasHole) {{
        return startGuidedMissionInput("경계지역 일반 Area 저장 완료. 목표지역 Hole Area를 이어서 입력하세요.");
      }}
      guidedMeta().package4BoundaryDone = true;
      return finishGuidedMission("경계지역 일반 Area와 목표지역 Hole Area 저장 완료.");
    }}
    function completeMissionArea(isHole) {{
      const wasGuided = Boolean(activeMission && activeMission.guided);
      const areaPoints = normalizeAreaPoints(activePoints);
      hideHolePicker();
      if (wasGuided && selectedPackageType() === 2 && activeMission.guidedMode === "package2_boundary_area") {{
        return completePackage2BoundaryArea(areaPoints, isHole);
      }}
      if (wasGuided && selectedPackageType() === 3 && activeMission.guidedMode === "package3_boundary_area") {{
        return completePackage3BoundaryArea(areaPoints, isHole);
      }}
      if (wasGuided && selectedPackageType() === 4 && activeMission.guidedMode === "package4_boundary_area") {{
        return completePackage4BoundaryArea(areaPoints, isHole);
      }}
      state.missions.push({{ inputMissionType: activeMission.inputMissionType, regionType: activeMission.regionType, areaList: [{{ isHole, points: areaPoints }}], lineList: [], coordinateList: [] }});
      activePoints = []; activeMission = null;
      if (wasGuided) return finishGuidedMission("Area mission 추가 완료.");
      startNextMissionInput("Area mission 추가 완료. 다음 임무 시작 위치를 클릭하세요.");
    }}
    function beginWidth() {{
      if (!activeMission || activePoints.length < 2) return showNotice("LineList[]는 점 2개 이상 필요합니다.");
      widthLine = activePoints.map(p => [p.longitude, p.latitude]);
      widthValue = 0;
      stage = "width";
      hideChoicePanel();
      widthTip.classList.add("is-active");
      setWidthPreview(0);
      setInstruction("마우스로 중심선에서 바깥쪽으로 드래그하세요. 좌우 폭 영역이 실시간으로 표시됩니다.");
      render();
    }}
    function finishWidth(width) {{
      const wasGuided = Boolean(activeMission && activeMission.guided);
      if (wasGuided && selectedPackageType() === 2 && activeMission.guidedMode === "package2_boundary_line") {{
        return completePackage2BoundaryLine(width);
      }}
      if (wasGuided && selectedPackageType() === 2 && activeMission.guidedMode === "package2_target_line") {{
        return completePackage2TargetLine(width);
      }}
      if (wasGuided && selectedPackageType() === 3 && activeMission.guidedMode === "package3_boundary_line") {{
        return completePackage3BoundaryLine(width);
      }}
      if (wasGuided && selectedPackageType() === 3 && activeMission.guidedMode === "package3_target_line") {{
        return completePackage3TargetLine(width);
      }}
      state.missions.push({{ inputMissionType: activeMission.inputMissionType, regionType: activeMission.regionType, lineList: [{{ width, points: activePoints.slice() }}], areaList: [], coordinateList: [] }});
      activePoints = []; activeMission = null; widthLine = [];
      suppressMapClickUntil = Date.now() + 350;
      if (wasGuided) return finishGuidedMission(`Line mission 추가 완료(width=${{width}}m).`);
      startNextMissionInput(`Line mission 추가 완료(width=${{width}}m). 다음 임무 시작 위치를 클릭하세요.`);
    }}
    const REGION_SHORT_NAMES = {{
      0: ["지정", "없음"],
      1: ["전술", "집결"],
      2: ["통제권", "변경"],
      3: ["ACP"],
      4: ["공격", "대기"],
      5: ["전투", "진지"],
      6: ["목표", "지역"],
      7: ["경계", "지역"],
      8: ["탑재", "지대"],
      9: ["착륙", "지대"],
      10: ["중요", "시설"],
      11: ["도서", "지역"]
    }};
    function addLabelLines(parent, className, lines) {{
      const span = document.createElement("span");
      span.className = className;
      (lines || []).forEach((line, idx) => {{
        if (idx) span.appendChild(document.createElement("br"));
        span.appendChild(document.createTextNode(line));
      }});
      parent.appendChild(span);
    }}
    function showTypePicker(screenPoint, lngLat) {{
      hideRegionPicker();
      pendingMissionLngLat = lngLat;
      activePoints = [point(lngLat)];
      const r = 108, cx = 148, cy = 148, half = 34;
      Array.from(typePicker.querySelectorAll("button")).forEach((button, idx) => {{
        const angle = (-90 + idx * 45) * Math.PI / 180;
        button.style.left = `${{cx + Math.cos(angle) * r - half}}px`;
        button.style.top = `${{cy + Math.sin(angle) * r - half}}px`;
      }});
      typePicker.style.left = `${{screenPoint.x}}px`;
      typePicker.style.top = `${{screenPoint.y}}px`;
      typePicker.classList.add("is-active");
      typePicker.setAttribute("aria-hidden", "false");
      setInstruction("Type을 선택하세요. 선택한 위치는 흰 점으로 표시됩니다.");
      render();
    }}
    function showRegionPicker(missionType, lngLat) {{
      pendingRegionMissionType = missionType;
      pendingRegionLngLat = lngLat;
      activePoints = [point(lngLat)];
      regionPicker.replaceChildren();
      const center = document.createElement("div");
      center.className = "region-picker-center";
      center.textContent = "지역";
      regionPicker.appendChild(center);
      const entries = Object.entries(config.regionTypes);
      const r = 150, cx = 192, cy = 192, half = 36;
      entries.forEach(([code, label], idx) => {{
        const button = document.createElement("button");
        button.type = "button";
        button.dataset.region = code;
        button.title = `${{code}}: ${{label}}`;
        const no = document.createElement("span");
        no.className = "region-no";
        no.textContent = code;
        button.appendChild(no);
        addLabelLines(button, "region-name", REGION_SHORT_NAMES[Number(code)] || [label]);
        const angle = (-90 + idx * (360 / entries.length)) * Math.PI / 180;
        button.style.left = `${{cx + Math.cos(angle) * r - half}}px`;
        button.style.top = `${{cy + Math.sin(angle) * r - half}}px`;
        regionPicker.appendChild(button);
      }});
      const screenPoint = map.project(lngLat);
      regionPicker.style.left = `${{screenPoint.x}}px`;
      regionPicker.style.top = `${{screenPoint.y}}px`;
      regionPicker.classList.add("is-active");
      regionPicker.setAttribute("aria-hidden", "false");
      stage = "region_pick";
      hideChoicePanel();
      setInstruction(`${{config.missionTypes[missionType] || "선택 임무"}}: RegionType을 원형 메뉴에서 선택하세요.`);
      render();
    }}
    function hideTypePicker() {{
      typePicker.classList.remove("is-active");
      typePicker.setAttribute("aria-hidden", "true");
    }}
    function hideRegionPicker() {{
      regionPicker.classList.remove("is-active");
      regionPicker.setAttribute("aria-hidden", "true");
    }}
    function showHolePicker() {{
      let pointForMenu = pendingHoleScreenPoint;
      if (!pointForMenu && activePoints.length) {{
        const last = activePoints[activePoints.length - 1];
        pointForMenu = map.project([last.longitude, last.latitude]);
      }}
      if (!pointForMenu) pointForMenu = {{ x: map.getCanvas().clientWidth / 2, y: map.getCanvas().clientHeight / 2 }};
      holePicker.style.left = `${{pointForMenu.x}}px`;
      holePicker.style.top = `${{pointForMenu.y}}px`;
      holePicker.classList.add("is-active");
      holePicker.setAttribute("aria-hidden", "false");
      stage = "hole_pick";
      const expected = activeMission ? activeMission.expectedHole : null;
      if (expected === true) setInstruction("목표지역은 Hole Area(true)를 선택하세요.");
      else if (expected === false) setInstruction("경계지역 외곽은 일반 Area(false)를 선택하세요.");
      else setInstruction("isHole 값을 선택하세요. 일반 Area 또는 Hole Area를 고르면 저장됩니다.");
      render();
    }}
    function hideHolePicker() {{
      holePicker.classList.remove("is-active");
      holePicker.setAttribute("aria-hidden", "true");
      pendingHoleScreenPoint = null;
    }}
    function asXY(coord, origin) {{
      const rad = Math.PI / 180, R = 6378137;
      return {{ x: (coord[0]-origin[0])*rad*Math.cos(origin[1]*rad)*R, y: (coord[1]-origin[1])*rad*R }};
    }}
    function fromXY(point, origin) {{
      const rad = Math.PI / 180, R = 6378137;
      const cosLat = Math.cos(origin[1] * rad) || 1;
      return [origin[0] + point.x / (rad * cosLat * R), origin[1] + point.y / (rad * R)];
    }}
    function lineBufferRing(line, fullWidth) {{
      if (!line || line.length < 2 || !Number.isFinite(fullWidth) || fullWidth <= 0) return null;
      const halfWidth = Math.max(0.5, Math.min(25000, fullWidth / 2));
      const origin = line[0];
      const points = line.map(coord => asXY(coord, origin));
      const normals = [];
      for (let i=0; i<points.length-1; i++) {{
        const a = points[i], b = points[i+1];
        const dx = b.x - a.x, dy = b.y - a.y;
        const len = Math.sqrt(dx*dx + dy*dy);
        if (!len) {{
          normals.push({{ x: 0, y: 0 }});
          continue;
        }}
        normals.push({{ x: -dy / len, y: dx / len }});
      }}
      const left = [], right = [];
      for (let i=0; i<points.length; i++) {{
        const prev = normals[Math.max(0, i - 1)] || {{ x: 0, y: 0 }};
        const next = normals[Math.min(normals.length - 1, i)] || prev;
        let nx = prev.x + next.x;
        let ny = prev.y + next.y;
        const nLen = Math.sqrt(nx*nx + ny*ny);
        if (nLen < 0.000001) {{ nx = next.x || prev.x; ny = next.y || prev.y; }}
        else {{ nx /= nLen; ny /= nLen; }}
        left.push({{ x: points[i].x + nx * halfWidth, y: points[i].y + ny * halfWidth }});
        right.push({{ x: points[i].x - nx * halfWidth, y: points[i].y - ny * halfWidth }});
      }}
      const ring = left.concat(right.reverse()).map(p => fromXY(p, origin));
      if (ring.length < 4) return null;
      ring.push(ring[0]);
      return ring;
    }}
    function distanceToLineMeters(lngLat, line) {{
      if (!line || line.length < 2) return 0;
      const origin = line[0], p = asXY([lngLat.lng, lngLat.lat], origin);
      let best = Infinity;
      for (let i=0; i<line.length-1; i++) {{
        const a = asXY(line[i], origin), b = asXY(line[i+1], origin);
        const vx = b.x-a.x, vy = b.y-a.y, wx = p.x-a.x, wy = p.y-a.y;
        const len2 = vx*vx + vy*vy;
        const t = Math.max(0, Math.min(1, len2 ? (wx*vx + wy*vy)/len2 : 0));
        const dx = p.x - (a.x + t*vx), dy = p.y - (a.y + t*vy);
        best = Math.min(best, Math.sqrt(dx*dx + dy*dy));
      }}
      return Number.isFinite(best) ? best : 0;
    }}
    function updateWidth(lngLat) {{
      const halfWidth = Math.max(1, Math.round(distanceToLineMeters(lngLat, widthLine)));
      widthValue = Math.max(2, Math.min(50000, halfWidth * 2));
      widthTip.textContent = `width: ${{widthValue}}m / 좌우 ${{Math.round(widthValue / 2)}}m`;
      setWidthPreview(widthValue);
      updateModePanel();
    }}
    function setWidthPreview(fullWidth) {{
      const features = [];
      const ring = lineBufferRing(widthLine, fullWidth);
      if (ring) features.push({{ type: "Feature", properties: {{ kind: "width_area" }}, geometry: {{ type: "Polygon", coordinates: [ring] }} }});
      if (widthLine.length >= 2) features.push({{ type: "Feature", properties: {{ kind: "width_center" }}, geometry: {{ type: "LineString", coordinates: widthLine }} }});
      if (map.getSource("width-preview")) map.getSource("width-preview").setData({{ type: "FeatureCollection", features }});
    }}
    function clearWidthPreview() {{
      widthTip.classList.remove("is-active");
      widthValue = 0;
      if (map.getSource("width-preview")) map.getSource("width-preview").setData({{ type: "FeatureCollection", features: [] }});
    }}
    function pointCoord(p) {{ return [p.longitude, p.latitude]; }}
    function areaSelfIntersects(pts) {{
      if (!pts || pts.length < 4) return false;
      const center = centerCoord(pts) || pointCoord(pts[0]);
      const coords = pts.map(p => asXY(pointCoord(p), center));
      function orient(a, b, c) {{
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
      }}
      function crosses(a, b, c, d) {{
        const eps = 1e-9;
        return (orient(a, b, c) * orient(a, b, d) < -eps) && (orient(c, d, a) * orient(c, d, b) < -eps);
      }}
      const n = coords.length;
      for (let i = 0; i < n; i++) {{
        const i2 = (i + 1) % n;
        for (let j = i + 1; j < n; j++) {{
          const j2 = (j + 1) % n;
          if (i === j || i2 === j || j2 === i) continue;
          if (i === 0 && j2 === 0) continue;
          if (crosses(coords[i], coords[i2], coords[j], coords[j2])) return true;
        }}
      }}
      return false;
    }}
    function normalizeAreaPoints(pts) {{
      const clean = (pts || []).filter(p => p && Number.isFinite(Number(p.latitude)) && Number.isFinite(Number(p.longitude)));
      if (clean.length < 3 || !areaSelfIntersects(clean)) return clean.slice();
      const center = centerCoord(clean) || pointCoord(clean[0]);
      return clean.slice().sort((a, b) => {{
        const pa = asXY(pointCoord(a), center);
        const pb = asXY(pointCoord(b), center);
        return Math.atan2(pa.y, pa.x) - Math.atan2(pb.y, pb.x);
      }});
    }}
    function centerCoord(pts) {{
      if (!pts || !pts.length) return null;
      let lon = 0, lat = 0;
      pts.forEach(p => {{ lon += Number(p.longitude || 0); lat += Number(p.latitude || 0); }});
      return [lon / pts.length, lat / pts.length];
    }}
    function lineCenterCoord(pts) {{
      if (!pts || !pts.length) return null;
      if (pts.length === 1) return pointCoord(pts[0]);
      const mid = Math.floor((pts.length - 1) / 2);
      const a = pts[mid], b = pts[mid + 1] || pts[mid];
      return [(Number(a.longitude) + Number(b.longitude)) / 2, (Number(a.latitude) + Number(b.latitude)) / 2];
    }}
    function addFeature(features, geometry, kind, props = {{}}) {{ features.push({{ type: "Feature", properties: {{ kind, ...props }}, geometry }}); }}
    function addPoint(features, p, kind, label = "") {{
      const props = label ? {{ label }} : {{}};
      addFeature(features, {{ type: "Point", coordinates: pointCoord(p) }}, kind, props);
    }}
    function addLabel(features, coord, label, kind = "label") {{
      if (!coord || !label) return;
      addFeature(features, {{ type: "Point", coordinates: coord }}, kind, {{ label }});
    }}
    function addLine(features, pts, kind) {{ if (pts.length >= 2) addFeature(features, {{ type: "LineString", coordinates: pts.map(pointCoord) }}, kind); }}
    function addLineCorridor(features, pts, width, kind) {{
      if (!pts || pts.length < 2) return;
      const coords = pts.map(pointCoord);
      const ring = lineBufferRing(coords, Number(width || 0));
      if (ring) addFeature(features, {{ type: "Polygon", coordinates: [ring] }}, kind);
    }}
    function addPoly(features, pts, kind) {{
      if (pts.length < 3) return addLine(features, pts, "active");
      const coords = pts.map(pointCoord);
      coords.push(coords[0]);
      addFeature(features, {{ type: "Polygon", coordinates: [coords] }}, kind);
    }}
    function render() {{
      state.packageType = Number(packageSelect.value || 1);
      updateTargetMissionOptions();
      const features = [];
      state.takeOver.forEach((p, i) => addPoint(features, p, "takeover", `TO UAV${{AIRCRAFT_IDS[i] || i + 1}}`));
      state.handOver.forEach((p, i) => addPoint(features, p, "handover", `HO UAV${{AIRCRAFT_IDS[i] || i + 1}}`));
      state.rtb.forEach((p, i) => addPoint(features, p, "rtb", `RTB ${{i + 1}}`));
      state.flightAreas.forEach((a, i) => {{
        addPoly(features, a, "flight");
        a.forEach((p, j) => addPoint(features, p, "flight_vertex", `F${{i + 1}}-${{j + 1}}`));
        addLabel(features, centerCoord(a), `FlightArea ${{i + 1}}`, "area_label");
      }});
      state.prohibitedAreas.forEach((a, i) => {{
        addPoly(features, a, "prohibited");
        a.forEach((p, j) => addPoint(features, p, "prohibited_vertex", `P${{i + 1}}-${{j + 1}}`));
        addLabel(features, centerCoord(a), `Prohibited ${{i + 1}}`, "area_label");
      }});
      state.missions.forEach((m, mi) => {{
        const missionNo = mi + 1;
        const missionType = config.missionTypes[m.inputMissionType] || `Type ${{m.inputMissionType}}`;
        (m.lineList || []).forEach((l, li) => {{
          const pts = l.points || [];
          addLineCorridor(features, l.points || [], l.width, "mission_line_area");
          addLine(features, l.points || [], "mission_line");
          pts.forEach((p, pi) => addPoint(features, p, "mission_vertex", `M${{missionNo}} L${{li + 1}}-${{pi + 1}}`));
          addLabel(features, lineCenterCoord(pts), `M${{missionNo}} Line · ${{missionType}} · w=${{l.width || 0}}m`, "mission_label");
        }});
        (m.areaList || []).forEach((a, ai) => {{
          const pts = a.points || [];
          addPoly(features, pts, "mission_area");
          pts.forEach((p, pi) => addPoint(features, p, "mission_vertex", `M${{missionNo}} A${{ai + 1}}-${{pi + 1}}`));
          addLabel(features, centerCoord(pts), `M${{missionNo}} Area · ${{missionType}}${{a.isHole ? " · Hole" : ""}}`, "mission_label");
        }});
      }});
      state.targets.forEach((target, i) => {{
        const loc = target.location || target;
        const label = `T${{target.targetID || i + 1}} type=${{target.targetType || 1}} M${{target.inputMissionID || 1}}`;
        addPoint(features, loc, "target", label);
      }});
      if (activePoints.length) {{
        activePoints.forEach((p, i) => addPoint(features, p, "active", `입력점 ${{i + 1}}`));
        if (["flight_area","prohibited_area","mission_area"].includes(stage)) addPoly(features, activePoints, "active");
        else addLine(features, activePoints, "active");
      }}
      const source = map.getSource("creator");
      if (source) source.setData({{ type: "FeatureCollection", features }});
      updateModePanel();
      const refStatus = referenceReady() ? "완료" : referenceStarted() ? "미완료" : "미시작";
      const missionStatus = state.missions.length ? `${{state.missions.length}}개 작성됨` : "미시작";
      const steps = guidedSteps();
      const nextStep = steps ? currentGuidedStep() : null;
      const nextStepRepeat = nextStep && nextStep.repeatTotal ? ` (${{nextStep.repeatIndex}}/${{nextStep.repeatTotal}})` : "";
      const guidedProgress = steps
        ? `Type ${{state.packageType}} 시퀀스: ${{nextStep ? nextStep.stepIndex + 1 : steps.length}}/${{steps.length}}${{nextStep ? ` · 다음: ${{nextStep.label}}${{nextStepRepeat}}` : " · 완료"}}`
        : "";
      summary.textContent = [
        `InputMissionPackageType: ${{state.packageType}}`,
        `Package ID: ${{state.packageID || "(저장 시 발급)"}}`,
        "",
        `[0203 기준정보] ${{refStatus}}`,
        `TakeOverInfoList: ${{state.takeOver.length}}/3`,
        `HandOverInfoList: ${{state.handOver.length}}/3`,
        `RTBCoordinateList: ${{state.rtb.length}}`,
        `FlightAreaList: ${{state.flightAreas.length}}`,
        `ProhibitedAreaList: ${{state.prohibitedAreas.length}}`,
        "",
        `[0201 임무목록] ${{missionStatus}}`,
        ...(steps ? [guidedProgress] : []),
        ...state.missions.map((m, i) => `  - #${{i+1}} type=${{m.inputMissionType}}(${{config.missionTypes[m.inputMissionType] || "-"}}) region=${{m.regionType}} line=${{(m.lineList || []).length}} area=${{(m.areaList || []).length}}`),
        "",
        `[TargetInfo / RTV 적 배치] ${{state.targets.length}}개`,
        ...state.targets.map((t, i) => `  - T${{t.targetID || i+1}} type=${{t.targetType || 1}} mission=${{t.inputMissionID || 1}} lat=${{(t.location || t).latitude}} lon=${{(t.location || t).longitude}}`)
      ].join("\\n");
    }}
    function extendBoundsWithPoint(bounds, point) {{
      if (!point) return false;
      const lat = Number(point.latitude);
      const lon = Number(point.longitude);
      if (!Number.isFinite(lat) || !Number.isFinite(lon)) return false;
      bounds.extend([lon, lat]);
      return true;
    }}
    function fitStateToMap() {{
      const bounds = new maplibregl.LngLatBounds();
      let count = 0;
      const add = (point) => {{ if (extendBoundsWithPoint(bounds, point)) count += 1; }};
      state.takeOver.forEach(add);
      state.handOver.forEach(add);
      state.rtb.forEach(add);
      state.flightAreas.forEach(area => (area || []).forEach(add));
      state.prohibitedAreas.forEach(area => (area || []).forEach(add));
      state.missions.forEach(mission => {{
        (mission.lineList || []).forEach(line => (line.points || []).forEach(add));
        (mission.areaList || []).forEach(area => (area.points || []).forEach(add));
        (mission.coordinateList || []).forEach(add);
      }});
      state.targets.forEach(target => add(target.location || target));
      if (count === 1) {{
        map.easeTo({{ center: bounds.getCenter(), zoom: Math.max(map.getZoom(), 12), duration: 350 }});
      }} else if (count > 1) {{
        map.fitBounds(bounds, {{ padding: {{ top: 70, right: 390, bottom: 70, left: 70 }}, maxZoom: 14, duration: 450 }});
      }}
    }}
    function setNumericInput(id, value, fallback) {{
      const el = document.getElementById(id);
      if (!el) return;
      const number = Number(value);
      el.value = String(Number.isFinite(number) ? number : fallback);
    }}
    function applyLoadedScenario(data) {{
      const loaded = data.state || {{}};
      pushUndo("시나리오 불러오기 전");
      restoreStateObject({{ ...loaded, guidedMeta: loaded.guidedMeta || {{}} }});
      activePoints = [];
      activeMission = null;
      pendingMissionLngLat = null;
      pendingRegionMissionType = null;
      pendingRegionLngLat = null;
      pendingHoleScreenPoint = null;
      widthLine = [];
      widthValue = 0;
      hideLoadPanel();
      hideChoicePanel();
      hideTypePicker();
      hideRegionPicker();
      hideHolePicker();
      clearWidthPreview();
      state.packageType = Number(state.packageType || 1);
      packageSelect.value = String(state.packageType);
      setNumericInput("refAgl", state.refAgl, 1000);
      setNumericInput("missionAlt", state.missionAlt, 0);
      setNumericInput("areaLower", state.areaLower, 0);
      setNumericInput("areaUpper", state.areaUpper, 5000);
      const demInput = document.getElementById("demEnabled");
      if (demInput) demInput.checked = Boolean(state.demEnabled !== false);
      stage = state.missions.length ? "mission_complete" : "mission_pick";
      const targetText = Number(data.targetCount || 0) ? ` · 적 ${{data.targetCount}}개` : "";
      saveStatus.textContent = `불러오기 완료: ${{data.saveName || ""}}`;
      saveStatus.title = `0201: ${{data.inputMissionPlanRelativePath || ""}}\n0203: ${{data.missionReferenceInfoRelativePath || ""}}\nTargetInfo: ${{data.targetInfoRelativePath || ""}}\nRTV: ${{data.rtvScenarioRelativePath || ""}}`;
      setInstruction(`불러오기 완료: ${{data.outputDir || "Logs/GeneratedScenario"}} · 임무 ${{data.missionCount || state.missions.length}}개${{targetText}}`);
      render();
      setTimeout(fitStateToMap, 60);
    }}
    async function performLoad() {{
      const saveName = scenarioSelect.value;
      if (!saveName) return showNotice("불러올 저장본을 선택하세요.");
      loadMeta.textContent = "불러오는 중...";
      loadConfirm.disabled = true;
      try {{
        const res = await fetch("/api/load_scenario", {{ method: "POST", headers: {{ "Content-Type": "application/json" }}, body: JSON.stringify({{ saveName }}) }});
        const data = await res.json();
        if (!data.ok) throw new Error(data.error || "시나리오 불러오기 실패");
        applyLoadedScenario(data);
      }} catch (err) {{
        loadMeta.textContent = err.message || "불러오기 실패";
        saveStatus.textContent = "불러오기 실패";
        loadConfirm.disabled = false;
        return showNotice(err.message || "불러오기 실패");
      }}
    }}
    async function performSave(saveName) {{
      hideSavePanel();
      hideLoadPanel();
      state.packageType = Number(packageSelect.value || 1);
      state.refAgl = Number(document.getElementById("refAgl").value || 1000);
      state.missionAlt = Number(document.getElementById("missionAlt").value || 0);
      state.areaLower = Number(document.getElementById("areaLower").value || 0);
      state.areaUpper = Number(document.getElementById("areaUpper").value || 5000);
      state.demEnabled = Boolean(document.getElementById("demEnabled").checked);
      saveStatus.textContent = "저장 중...";
      const res = await fetch("/api/save", {{ method: "POST", headers: {{ "Content-Type": "application/json" }}, body: JSON.stringify({{ state, saveName }}) }});
      const data = await res.json();
      if (!data.ok) {{ saveStatus.textContent = "저장 실패"; return showNotice(data.error || "저장 실패"); }}
      state.packageID = data.packageID;
      saveStatus.textContent = `저장 완료: ${{data.saveName}}`;
      saveStatus.title = `0201: ${{data.inputMissionPlanRelativePath || data.inputMissionPlanPath}}\n0203: ${{data.missionReferenceInfoRelativePath || data.missionReferenceInfoPath}}\nTargetInfo: ${{data.targetInfoRelativePath || data.targetInfoPath}}\nRTV: ${{data.rtvScenarioRelativePath || data.rtvScenarioPath}}`;
      setInstruction(`저장 완료: ${{data.outputDir || "Logs/GeneratedScenario"}} · RTV 적 ${{data.targetCount || 0}}개`);
      render();
    }}

    async function performAutoGenerate() {{
      const packageType = Number(packageSelect.value || 1);
      const seedText = String(autoSeedInput.value || "").trim();
      if (seedText && !Number.isInteger(Number(seedText))) return showNotice("Seed는 정수로 입력하세요.");
      hideSavePanel();
      hideLoadPanel();
      hideChoicePanel();
      autoGenerateButton.disabled = true;
      autoGenerateButton.textContent = "생성 중...";
      saveStatus.textContent = `Type ${{packageType}} 자동 생성 중...`;
      setInstruction("RTV 기준 영역을 변형하고 0201·0203·TargetInfo·RTV를 생성하고 있습니다.");
      try {{
        const body = {{ packageType }};
        if (seedText) body.seed = Number(seedText);
        const res = await fetch("/api/auto_generate", {{
          method: "POST",
          headers: {{ "Content-Type": "application/json" }},
          body: JSON.stringify(body)
        }});
        const data = await res.json();
        if (!data.ok) throw new Error(data.error || "자동 생성 실패");
        applyLoadedScenario(data);
        // Do not feed the resolved seed back into the next request.  Keeping
        // this field empty makes every subsequent click generate a fresh
        // scenario; an operator can still type an integer for a one-off replay.
        autoSeedInput.value = "";
        autoSeedInput.title = `최근 생성 Seed: ${{data.seed}} · 다음 생성은 새 랜덤`;
        saveStatus.textContent = `자동 생성 완료: ${{data.saveName}}`;
        saveStatus.title = `0201: ${{data.inputMissionPlanRelativePath || ""}}\n0203: ${{data.missionReferenceInfoRelativePath || ""}}\nTargetInfo: ${{data.targetInfoRelativePath || ""}}\nRTV: ${{data.rtvScenarioRelativePath || ""}}`;
        const primaryCount = Number((data.generation || {{}}).primaryTargetCount || 0);
        setInstruction(`자동 생성·저장 완료: ${{data.outputDir}} · 목표지역 적 ${{primaryCount}}개 · 전체 적 ${{data.targetCount || 0}}개`);
      }} catch (err) {{
        saveStatus.textContent = "자동 생성 실패";
        showNotice(err.message || "자동 생성 실패");
      }} finally {{
        autoGenerateButton.disabled = false;
        autoGenerateButton.textContent = "자동 생성+저장";
      }}
    }}

    map.on("load", () => {{
      map.addSource("mission-regions", {{ type: "geojson", data: missionRegionGeoJson() }});
      map.addLayer({{
        id: "mission-regions-fill",
        type: "fill",
        source: "mission-regions",
        paint: {{
          "fill-color": "#f2d55c",
          "fill-opacity": 0.14
        }}
      }});
      map.addLayer({{
        id: "mission-regions-line",
        type: "line",
        source: "mission-regions",
        paint: {{
          "line-color": "#f2d55c",
          "line-width": 2,
          "line-dasharray": [3, 2]
        }}
      }});
      map.addLayer({{
        id: "mission-regions-label",
        type: "symbol",
        source: "mission-regions",
        layout: {{
          "text-field": ["get", "label"],
          "text-size": 13,
          "text-font": ["Noto Sans Regular"],
          "text-anchor": "center",
          "text-allow-overlap": true
        }},
        paint: {{
          "text-color": "#ffe78c",
          "text-halo-color": "rgba(17, 23, 21, 0.85)",
          "text-halo-width": 1.5
        }}
      }});
      map.addSource("creator", {{ type: "geojson", data: {{ type: "FeatureCollection", features: [] }} }});
      map.addLayer({{ id: "creator-area-fill", type: "fill", source: "creator", filter: ["==", ["geometry-type"], "Polygon"], paint: {{ "fill-color": ["match", ["get", "kind"], "flight", "#2fbcc3", "prohibited", "#e0524d", "mission_area", "#e2c34b", "mission_line_area", "#f2d55c", "active", "#ffffff", "#88c0d0"], "fill-opacity": ["match", ["get", "kind"], "prohibited", .28, "mission_line_area", .14, "active", .16, .22] }} }});
      map.addLayer({{ id: "creator-area-line", type: "line", source: "creator", filter: ["==", ["geometry-type"], "Polygon"], paint: {{ "line-color": ["match", ["get", "kind"], "flight", "#34d5df", "prohibited", "#ff6961", "mission_area", "#f2d55c", "mission_line_area", "#f2d55c", "active", "#ffffff", "#c6edf0"], "line-width": 2 }} }});
      map.addLayer({{ id: "creator-lines", type: "line", source: "creator", filter: ["==", ["geometry-type"], "LineString"], paint: {{ "line-color": ["match", ["get", "kind"], "mission_line", "#f2d55c", "active", "#ffffff", "#8bd7ff"], "line-width": 3 }} }});
      map.addLayer({{ id: "creator-points", type: "circle", source: "creator", filter: ["==", ["geometry-type"], "Point"], paint: {{
        "circle-radius": ["match", ["get", "kind"], "active", 6, "target", 7, "area_label", 0, "mission_label", 0, "label", 0, 5],
        "circle-color": ["match", ["get", "kind"], "takeover", "#44d3ff", "handover", "#79e184", "rtb", "#f2d55c", "flight_vertex", "#34d5df", "prohibited_vertex", "#ff6961", "mission_vertex", "#f2d55c", "target", "#ff4d4d", "active", "#ffffff", "#e6eef2"],
        "circle-opacity": ["match", ["get", "kind"], "area_label", 0, "mission_label", 0, "label", 0, 1],
        "circle-stroke-color": "#0e1414",
        "circle-stroke-width": 1.5
      }} }});
      map.addLayer({{ id: "creator-labels", type: "symbol", source: "creator", filter: ["has", "label"], layout: {{
        "text-field": ["get", "label"],
        "text-size": 10,
        "text-offset": [0, 1.05],
        "text-anchor": "top",
        "text-allow-overlap": true,
        "text-ignore-placement": true
      }}, paint: {{
        "text-color": ["match", ["get", "kind"], "takeover", "#8ee7ff", "handover", "#a3f2b2", "rtb", "#ffe78c", "mission_label", "#fff0ad", "area_label", "#dffcf8", "target", "#ffb0a8", "active", "#ffffff", "#e8f0ed"],
        "text-halo-color": "rgba(13, 19, 18, 0.92)",
        "text-halo-width": 1.4
      }} }});
      map.addSource("width-preview", {{ type: "geojson", data: {{ type: "FeatureCollection", features: [] }} }});
      map.addLayer({{ id: "width-preview-fill", type: "fill", source: "width-preview", filter: ["==", ["geometry-type"], "Polygon"], paint: {{ "fill-color": "#58d6c6", "fill-opacity": .28 }} }});
      map.addLayer({{ id: "width-preview-outline", type: "line", source: "width-preview", filter: ["==", ["geometry-type"], "Polygon"], paint: {{ "line-color": "#bffff2", "line-width": 2 }} }});
      map.addLayer({{ id: "width-preview-center", type: "line", source: "width-preview", filter: ["==", ["geometry-type"], "LineString"], paint: {{ "line-color": "#ffffff", "line-width": 2, "line-dasharray": [1.5, 1.5] }} }});
      updateCenterRuler();
      begin0203Input();
    }});
    map.on("move", updateCenterRuler);
    map.on("zoom", updateCenterRuler);
    map.on("resize", updateCenterRuler);
    map.on("click", e => {{
      if (Date.now() < suppressMapClickUntil) {{
        if (e.preventDefault) e.preventDefault();
        return;
      }}
      if (["mission_pick", "target_place", "takeover", "handover", "rtb", "mission_line", "flight_area", "prohibited_area", "mission_area"].includes(stage)) {{
        pushUndo("지도 입력");
      }}
      if (stage === "mission_pick") return showTypePicker(e.point, e.lngLat);
      if (stage === "target_place") return addTargetAt(e.lngLat);
      if (stage === "width") return;
      const p = point(e.lngLat);
      if (stage === "takeover") {{ state.takeOver.push(p); if (state.takeOver.length >= 3) {{ setStage("handover", "HandOverInfoList[]를 입력하세요(점 3개). UAV4, UAV5, UAV6 순서."); }} else setInstruction(`TakeOverInfoList[] ${{state.takeOver.length}}/3 입력됨.`); }}
      else if (stage === "handover") {{ state.handOver.push(p); if (state.handOver.length >= 3) {{ setStage("rtb", "RTBCoordinateList[]를 입력하세요(점 N개). 우클릭으로 종료."); }} else setInstruction(`HandOverInfoList[] ${{state.handOver.length}}/3 입력됨.`); }}
      else if (stage === "rtb") {{ state.rtb.push(p); setInstruction(`RTBCoordinateList[] ${{state.rtb.length}}개 입력됨.`); }}
      else if (stage === "mission_line") {{
        activePoints.push(p);
        if (activeMission && activeMission.guided) {{
          const guided = currentGuidedStep();
          const required = guided ? Number(guided.requiredPoints || 2) : 2;
          if (activePoints.length >= required) return beginWidth();
          setInstruction(`Line 점 ${{activePoints.length}}/${{required}} 입력됨.`);
        }} else {{
          setInstruction(`점 ${{activePoints.length}}개 입력됨. 우클릭으로 완료.`);
        }}
      }}
      else if (["flight_area","prohibited_area","mission_area"].includes(stage)) {{ activePoints.push(p); setInstruction(`점 ${{activePoints.length}}개 입력됨. 우클릭으로 완료.`); }}
      render();
    }});
    map.on("contextmenu", e => {{ e.preventDefault(); pendingHoleScreenPoint = e.point; if (stage !== "width") {{ if (["rtb", "flight_area", "prohibited_area", "mission_line", "mission_area", "target_place"].includes(stage)) pushUndo("단계 완료"); finishStage(); }} }});
    map.on("mousedown", e => {{ if (stage !== "width" || e.originalEvent.button !== 0) return; widthDragging = true; map.dragPan.disable(); updateWidth(e.lngLat); e.preventDefault(); }});
    map.on("mousemove", e => {{
      updateCursorSupport(e);
      if (stage === "width") updateWidth(e.lngLat);
    }});
    map.getCanvas().addEventListener("mouseleave", hideCursorSupport);
    map.on("mouseup", e => {{ if (!widthDragging) return; widthDragging = false; map.dragPan.enable(); if (e.originalEvent) {{ e.originalEvent.preventDefault(); e.originalEvent.stopPropagation(); }} updateWidth(e.lngLat); pushUndo("width 확정"); finishWidth(widthValue); }});
    typePicker.addEventListener("click", e => {{
      e.stopPropagation();
      const button = e.target.closest("button[data-type]");
      if (!button || !pendingMissionLngLat) return;
      hideTypePicker();
      selectMissionType(Number(button.dataset.type), pendingMissionLngLat);
    }});
    regionPicker.addEventListener("click", e => {{
      e.stopPropagation();
      const button = e.target.closest("button[data-region]");
      if (!button || !pendingRegionLngLat || pendingRegionMissionType === null) return;
      hideRegionPicker();
      chooseRegionType(pendingRegionMissionType, pendingRegionLngLat, Number(button.dataset.region));
    }});
    holePicker.addEventListener("click", e => {{
      e.stopPropagation();
      const button = e.target.closest("button[data-hole]");
      if (!button) return;
      completeMissionArea(button.dataset.hole === "true");
    }});
    undoButton.addEventListener("click", undoLast);
    document.addEventListener("keydown", e => {{
      const tag = (document.activeElement && document.activeElement.tagName || "").toUpperCase();
      if ((e.ctrlKey || e.metaKey) && !e.shiftKey && String(e.key || "").toLowerCase() === "z" && !["INPUT", "TEXTAREA", "SELECT"].includes(tag)) {{
        e.preventDefault();
        undoLast();
      }}
    }});
    document.getElementById("addMission").addEventListener("click", addMission);
    autoGenerateButton.addEventListener("click", performAutoGenerate);
    autoSeedInput.addEventListener("keydown", e => {{ if (e.key === "Enter") performAutoGenerate(); }});
    document.getElementById("addTarget").addEventListener("click", startTargetPlacement);
    document.getElementById("save").addEventListener("click", showSavePanel);
    document.getElementById("loadScenario").addEventListener("click", showLoadPanel);
    saveConfirm.addEventListener("click", () => performSave(saveNameInput.value.trim()));
    saveCancel.addEventListener("click", () => {{ hideSavePanel(); setInstruction("저장이 취소되었습니다."); }});
    saveNameInput.addEventListener("keydown", e => {{
      if (e.key === "Enter") performSave(saveNameInput.value.trim());
      if (e.key === "Escape") {{ hideSavePanel(); setInstruction("저장이 취소되었습니다."); }}
    }});
    loadConfirm.addEventListener("click", performLoad);
    loadRefresh.addEventListener("click", refreshScenarioList);
    loadCancel.addEventListener("click", () => {{ hideLoadPanel(); setInstruction("불러오기가 취소되었습니다."); }});
    scenarioSelect.addEventListener("change", updateLoadMeta);
    document.getElementById("reset").addEventListener("click", resetAll);
    packageSelect.addEventListener("change", render);
  </script>
</body>
</html>"""


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="HTML-based 0201/0203 mission creation GUI")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--open", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    server = MissionCreationServer((str(args.host), int(args.port)))
    url = server.base_url
    print(f"[MISSION-CREATION] serving {url}", flush=True)
    print(f"[MISSION-CREATION] db root {db_paths.get_active_db_root()}", flush=True)
    if args.open:
        threading.Timer(0.6, lambda: webbrowser.open(url)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.close()
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
