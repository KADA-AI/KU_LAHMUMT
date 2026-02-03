from __future__ import annotations

import json
import mimetypes
import os
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional
from urllib.parse import parse_qs, urlparse

from ..config import (
    DEFAULT_BOUNDS,
    DEFAULT_CENTER_LAT,
    DEFAULT_CENTER_LON,
    DEFAULT_START_ZOOM,
    DEM_DIR,
    DEM_ENCODING,
    DEM_EXAGGERATION,
    DEM_MAX_ZOOM,
    DEM_PITCH_THRESHOLD,
    DEM_TILE_SIZE,
    MBTILES_PATH,
    RESOURCE_DIR,
    WEB_DIR,
)
from ..integration.integration_service import IntegrationService
from ..mission.mission_loader import load_flight_paths
from ..runtime.sim_service import SimulationService
from ..map.dem_tiles import load_dem_provider
from ..map.mbtiles import MBTiles


class MapHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(
        self,
        server_address: tuple[str, int],
        mbtiles: MBTiles,
        web_dir: Path,
        dem_provider,
        integration: IntegrationService,
        sim_service: SimulationService,
    ):
        super().__init__(server_address, MapRequestHandler)
        self.mbtiles = mbtiles
        self.web_dir = Path(web_dir)
        self.dem_provider = dem_provider
        self.integration = integration
        self.sim = sim_service


class MapServer:
    def __init__(self, host: str, port: int) -> None:
        self.mbtiles = MBTiles(MBTILES_PATH)
        self.dem_provider = None
        if DEM_DIR.exists():
            provider = load_dem_provider(DEM_DIR, tile_size=DEM_TILE_SIZE, max_zoom=DEM_MAX_ZOOM)
            if provider.available:
                self.dem_provider = provider
        self.integration = IntegrationService()
        self.sim = SimulationService()
        try:
            self.sim.set_integration(self.integration)
        except Exception:
            pass
        self._server = MapHTTPServer(
            (host, port),
            self.mbtiles,
            WEB_DIR,
            self.dem_provider,
            self.integration,
            self.sim,
        )
        self._thread = None

    @property
    def url(self) -> str:
        host, port = self._server.server_address
        return f"http://{host}:{port}/"

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        import threading

        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if not self._thread or not self._thread.is_alive():
            return
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=2)
        self.mbtiles.close()
        try:
            if self.integration:
                self.integration.shutdown()
        except Exception:
            pass
        try:
            if self.sim:
                self.sim.shutdown()
        except Exception:
            pass


class MapRequestHandler(BaseHTTPRequestHandler):
    server: MapHTTPServer

    def do_GET(self) -> None:
        path = urlparse(self.path).path
        if path.startswith("/api/integration/"):
            self._serve_integration_get(path)
            return
        if path.startswith("/api/sim/"):
            self._serve_sim_get(path)
            return
        if path.startswith("/api/mission/"):
            self._serve_mission_get(path)
            return
        if path in ("/", "/index.html"):
            self._serve_index()
            return
        if path == "/favicon.ico":
            self._serve_favicon()
            return
        if path.startswith("/tiles/"):
            self._serve_tile(path)
            return
        if path.startswith("/dem/"):
            self._serve_dem(path)
            return
        if path.startswith("/resources/"):
            self._serve_resource(path)
            return
        if path.endswith(".js") or path.endswith(".css"):
            self._serve_static(path)
            return
        self._send_text(HTTPStatus.NOT_FOUND, "Not found")

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        if path.startswith("/api/integration/"):
            self._serve_integration_post(path)
            return
        if path.startswith("/api/sim/"):
            self._serve_sim_post(path)
            return
        if path.startswith("/api/mission/"):
            self._serve_mission_post(path)
            return
        self._send_text(HTTPStatus.NOT_FOUND, "Not found")

    def log_message(self, fmt: str, *args: object) -> None:
        return

    def _serve_index(self) -> None:
        template_path = self.server.web_dir / "index.html"
        if not template_path.exists():
            self._send_text(HTTPStatus.NOT_FOUND, "Missing index.html")
            return
        html = template_path.read_text(encoding="utf-8")
        info = self.server.mbtiles.info
        center_lat, center_lon, start_zoom = info.start_view(
            DEFAULT_CENTER_LAT, DEFAULT_CENTER_LON, DEFAULT_START_ZOOM
        )
        # Force default view (Jipo-ri) instead of MBTiles metadata.
        center_lat = DEFAULT_CENTER_LAT
        center_lon = DEFAULT_CENTER_LON
        start_zoom = DEFAULT_START_ZOOM
        bounds_json = _format_bounds(DEFAULT_BOUNDS)
        base_url = self._base_url()
        ext = _normalize_format(info.tile_format)
        tile_url = f"{base_url}/tiles/{{z}}/{{x}}/{{y}}.{ext}"
        dem_url = f"{base_url}/dem/{{z}}/{{x}}/{{y}}.png"
        dem_available = "1" if self.server.dem_provider else "0"
        html = (
            html.replace("__TILE_URL__", tile_url)
            .replace("__MIN_ZOOM__", str(info.min_zoom))
            .replace("__MAX_ZOOM__", str(info.max_zoom))
            .replace("__CENTER_LAT__", f"{center_lat:.6f}")
            .replace("__CENTER_LON__", f"{center_lon:.6f}")
            .replace("__START_ZOOM__", str(start_zoom))
            .replace("__BOUNDS_JSON__", bounds_json)
            .replace("__DEM_URL__", dem_url)
            .replace("__DEM_AVAILABLE__", dem_available)
            .replace("__DEM_TILE_SIZE__", str(DEM_TILE_SIZE))
            .replace("__DEM_MAX_ZOOM__", str(DEM_MAX_ZOOM))
            .replace("__DEM_ENCODING__", DEM_ENCODING)
            .replace("__DEM_EXAGGERATION__", str(DEM_EXAGGERATION))
            .replace("__DEM_PITCH_THRESHOLD__", str(DEM_PITCH_THRESHOLD))
        )
        self._send_bytes(HTTPStatus.OK, html.encode("utf-8"), "text/html; charset=utf-8")

    def _serve_integration_get(self, path: str) -> None:
        integration = getattr(self.server, "integration", None)
        if integration is None:
            self._send_json({"ok": False, "error": "Integration unavailable"}, HTTPStatus.SERVICE_UNAVAILABLE)
            return
        parsed = urlparse(self.path)
        if parsed.path == "/api/integration/state":
            self._send_json(integration.get_state())
            return
        if parsed.path == "/api/integration/payload":
            query = parse_qs(parsed.query or "")
            msg_id = (query.get("msgId") or [""])[0]
            payload_type = (query.get("type") or ["tx"])[0]
            self._send_json(integration.get_payload(msg_id, payload_type))
            return
        self._send_json({"ok": False, "error": "Not found"}, HTTPStatus.NOT_FOUND)

    def _serve_integration_post(self, path: str) -> None:
        integration = getattr(self.server, "integration", None)
        if integration is None:
            self._send_json({"ok": False, "error": "Integration unavailable"}, HTTPStatus.SERVICE_UNAVAILABLE)
            return
        body = self._read_json()
        if path == "/api/integration/send":
            msg_id = (body or {}).get("msgId")
            result = integration.toggle_send(msg_id)
            self._send_json(result)
            return
        if path == "/api/integration/generate":
            msg_id = (body or {}).get("msgId")
            result = integration.generate(msg_id)
            self._send_json(result)
            return
        if path == "/api/integration/send_custom":
            msg_id = (body or {}).get("msgId")
            payload = (body or {}).get("body")
            result = integration.send_custom(msg_id, payload)
            self._send_json(result)
            return
        self._send_json({"ok": False, "error": "Not found"}, HTTPStatus.NOT_FOUND)

    def _serve_mission_get(self, path: str) -> None:
        if path == "/api/mission/ping":
            self._send_json({"ok": True})
            return
        self._send_json({"ok": False, "error": "Not found"}, HTTPStatus.NOT_FOUND)

    def _serve_mission_post(self, path: str) -> None:
        if path != "/api/mission/load":
            self._send_json({"ok": False, "error": "Not found"}, HTTPStatus.NOT_FOUND)
            return
        body = self._read_json()
        base_path = (body or {}).get("path", "")
        result = load_flight_paths(str(base_path), project_root=Path(__file__).resolve().parents[3])
        self._send_json(result)

    def _serve_sim_get(self, path: str) -> None:
        sim = getattr(self.server, "sim", None)
        if sim is None:
            self._send_json({"ok": False, "error": "Simulation unavailable"}, HTTPStatus.SERVICE_UNAVAILABLE)
            return
        parsed = urlparse(self.path)
        if parsed.path == "/api/sim/state":
            query = parse_qs(parsed.query or "")
            since_val = (query.get("since") or [None])[0]
            since = None
            if since_val is not None:
                try:
                    since = int(float(since_val))
                except Exception:
                    since = None
            self._send_json(sim.build_snapshot(since_step=since))
            return
        if path == "/api/sim/ping":
            self._send_json({"ok": True})
            return
        self._send_json({"ok": False, "error": "Not found"}, HTTPStatus.NOT_FOUND)

    def _serve_sim_post(self, path: str) -> None:
        sim = getattr(self.server, "sim", None)
        if sim is None:
            self._send_json({"ok": False, "error": "Simulation unavailable"}, HTTPStatus.SERVICE_UNAVAILABLE)
            return
        body = self._read_json()
        if path == "/api/sim/mission":
            result = sim.load_mission(body or {})
            self._send_json(result)
            return
        if path == "/api/sim/targets/add":
            result = sim.add_target(body or {})
            self._send_json(result)
            return
        if path == "/api/sim/targets/clear":
            result = sim.clear_targets()
            self._send_json(result)
            return
        if path == "/api/sim/next_mission":
            aircraft_id = None
            payload = body or {}
            for key in ("aircraftId", "aircraftID"):
                if key in payload:
                    try:
                        aircraft_id = int(payload.get(key))
                    except Exception:
                        aircraft_id = None
                    break
            if aircraft_id is None and "agent" in payload:
                agent = str(payload.get("agent") or "").strip().upper()
                if agent.startswith("LAH"):
                    try:
                        idx = int(agent.replace("LAH", ""))
                    except Exception:
                        idx = None
                    if idx and 1 <= idx <= 3:
                        aircraft_id = idx
                elif agent.startswith("UAV"):
                    try:
                        idx = int(agent.replace("UAV", ""))
                    except Exception:
                        idx = None
                    if idx and 1 <= idx <= 3:
                        aircraft_id = idx + 3
            advanced = sim.advance_input_mission(aircraft_id)
            self._send_json({"ok": True, "advanced": advanced})
            return
        if path == "/api/sim/play":
            self._send_json(sim.play())
            return
        if path == "/api/sim/pause":
            self._send_json(sim.pause())
            return
        if path == "/api/sim/stop":
            self._send_json(sim.stop())
            return
        if path == "/api/sim/clear":
            self._send_json(sim.clear())
            return
        if path == "/api/sim/reset":
            self._send_json(sim.reset())
            return
        if path == "/api/sim/speed":
            speed = (body or {}).get("speed")
            value = sim.set_speed_factor(speed if speed is not None else 1.0)
            self._send_json({"ok": True, "speedFactor": value})
            return
        self._send_json({"ok": False, "error": "Not found"}, HTTPStatus.NOT_FOUND)

    def _serve_tile(self, path: str) -> None:
        parts = path.split("/")
        if len(parts) < 5:
            self._send_text(HTTPStatus.NOT_FOUND, "Invalid tile path")
            return
        try:
            z = int(parts[2])
            x = int(parts[3])
            y_str, _ext = os.path.splitext(parts[4])
            y = int(y_str)
        except ValueError:
            self._send_text(HTTPStatus.BAD_REQUEST, "Invalid tile coordinates")
            return
        data = self.server.mbtiles.get_tile(z, x, y)
        if data is None:
            self._send_text(HTTPStatus.NOT_FOUND, "Tile not found")
            return
        content_type = _content_type(self.server.mbtiles.info.tile_format)
        encoding = None
        if self.server.mbtiles.info.tile_format == "pbf" and data[:2] == b"\x1f\x8b":
            encoding = "gzip"
        self._send_bytes(
            HTTPStatus.OK,
            data,
            content_type,
            encoding=encoding,
            cache_control="public, max-age=3600",
        )

    def _serve_dem(self, path: str) -> None:
        provider = self.server.dem_provider
        if provider is None:
            self._send_text(HTTPStatus.NOT_FOUND, "DEM unavailable")
            return
        parts = path.split("/")
        if len(parts) < 5:
            self._send_text(HTTPStatus.NOT_FOUND, "Invalid DEM tile path")
            return
        try:
            z = int(parts[2])
            x = int(parts[3])
            y_str, _ext = os.path.splitext(parts[4])
            y = int(y_str)
        except ValueError:
            self._send_text(HTTPStatus.BAD_REQUEST, "Invalid DEM tile coordinates")
            return
        data = provider.get_tile(z, x, y)
        if data is None:
            self._send_text(HTTPStatus.NOT_FOUND, "DEM tile not found")
            return
        self._send_bytes(
            HTTPStatus.OK,
            data,
            "image/png",
            cache_control="public, max-age=3600",
        )

    def _serve_static(self, path: str) -> None:
        file_path = _safe_join(self.server.web_dir, path)
        if file_path is None or not file_path.is_file():
            self._send_text(HTTPStatus.NOT_FOUND, "Not found")
            return
        mime, _ = mimetypes.guess_type(file_path)
        content_type = _ensure_utf8_content_type(mime or "application/octet-stream")
        self._send_bytes(HTTPStatus.OK, file_path.read_bytes(), content_type)

    def _serve_resource(self, path: str) -> None:
        resource_path = path[len("/resources/") :]
        file_path = _safe_join(RESOURCE_DIR, resource_path)
        if file_path is None or not file_path.is_file():
            self._send_text(HTTPStatus.NOT_FOUND, "Not found")
            return
        mime, _ = mimetypes.guess_type(file_path)
        content_type = _ensure_utf8_content_type(mime or "application/octet-stream")
        self._send_bytes(
            HTTPStatus.OK,
            file_path.read_bytes(),
            content_type,
            cache_control="public, max-age=3600",
        )

    def _serve_favicon(self) -> None:
        file_path = _safe_join(self.server.web_dir, "/favicon.ico")
        if file_path and file_path.is_file():
            self._send_bytes(HTTPStatus.OK, file_path.read_bytes(), "image/x-icon")
            return
        self.send_response(HTTPStatus.NO_CONTENT)
        self.end_headers()

    def _read_json(self) -> dict:
        length = 0
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            length = 0
        if length <= 0:
            return {}
        raw = self.rfile.read(length)
        try:
            return json.loads(raw.decode("utf-8"))
        except Exception:
            return {}

    def _base_url(self) -> str:
        host = self.headers.get("Host")
        if host:
            return f"http://{host}"
        host, port = self.server.server_address
        if host in ("0.0.0.0", "::"):
            host = "127.0.0.1"
        return f"http://{host}:{port}"

    def _send_text(self, status: HTTPStatus, message: str) -> None:
        self._send_bytes(status, message.encode("utf-8"), "text/plain; charset=utf-8")

    def _send_bytes(
        self,
        status: HTTPStatus,
        data: bytes,
        content_type: str,
        encoding: Optional[str] = None,
        cache_control: Optional[str] = None,
    ) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        if encoding:
            self.send_header("Content-Encoding", encoding)
        if cache_control:
            self.send_header("Cache-Control", cache_control)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            return

    def _send_json(self, payload: dict, status: HTTPStatus = HTTPStatus.OK) -> None:
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self._send_bytes(status, data, "application/json; charset=utf-8")


def _safe_join(base_dir: Path, request_path: str) -> Optional[Path]:
    safe_path = os.path.normpath(request_path).lstrip("/\\")
    full = (base_dir / safe_path).resolve()
    if base_dir != full and base_dir not in full.parents:
        return None
    return full


def _normalize_format(tile_format: str) -> str:
    fmt = (tile_format or "png").lower()
    if fmt == "jpeg":
        return "jpg"
    return fmt


def _content_type(tile_format: str) -> str:
    fmt = _normalize_format(tile_format)
    if fmt in ("jpg", "jpeg"):
        return "image/jpeg"
    if fmt == "png":
        return "image/png"
    if fmt == "webp":
        return "image/webp"
    if fmt == "pbf":
        return "application/vnd.mapbox-vector-tile"
    return "application/octet-stream"


def _ensure_utf8_content_type(content_type: str) -> str:
    if "charset=" in content_type.lower():
        return content_type
    base = content_type.split(";", 1)[0].strip().lower()
    if base.startswith("text/"):
        return f"{base}; charset=utf-8"
    if base in ("application/javascript", "text/javascript", "application/json", "application/xml", "image/svg+xml"):
        return f"{base}; charset=utf-8"
    if base.endswith("+json") or base.endswith("+xml"):
        return f"{base}; charset=utf-8"
    return content_type


def _format_bounds(bounds: Optional[tuple[float, float, float, float]]) -> str:
    if bounds is None:
        return "null"
    min_lon, min_lat, max_lon, max_lat = bounds
    return json.dumps([[min_lon, min_lat], [max_lon, max_lat]])
