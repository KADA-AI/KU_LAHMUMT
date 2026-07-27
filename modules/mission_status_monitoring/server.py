from __future__ import annotations

import argparse
import ipaddress
import json
import mimetypes
import os
import sys
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

if __package__ in (None, ""):
    root = Path(__file__).resolve().parents[2]
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))

from modules.mission_status_monitoring.receiver import ReadOnly0401Integration
from modules.mission_status_monitoring.service import MissionStatusService
from modules.monitoring.logic.coverage_settings import (
    load_coverage_settings,
    save_coverage_settings,
)
from modules.sim.config import (
    DEFAULT_BOUNDS,
    DEFAULT_CENTER_LAT,
    DEFAULT_CENTER_LON,
    DEFAULT_START_ZOOM,
    MBTILES_PATH,
)
from modules.sim.map.mbtiles import MBTiles


MODULE_DIR = Path(__file__).resolve().parent
WEB_DIR = MODULE_DIR / "web"
VENDOR_DIR = MODULE_DIR.parent / "sim" / "web" / "vendor"


class MissionStatusHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, address: tuple[str, int], integration: ReadOnly0401Integration) -> None:
        super().__init__(address, MissionStatusHandler)
        self.mbtiles = MBTiles(MBTILES_PATH)
        self.integration = integration
        self.service = MissionStatusService(integration)

    def close_resources(self) -> None:
        try:
            self.mbtiles.close()
        except Exception:
            pass
        try:
            self.integration.shutdown()
        except Exception:
            pass


class MissionStatusHandler(BaseHTTPRequestHandler):
    server: MissionStatusHTTPServer

    def log_message(self, fmt: str, *args: object) -> None:
        return

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        path = parsed.path
        if path == "/api/health":
            self._json({"ok": True, "service": "ku-mission-status-monitor", "version": 1})
            return
        if path == "/api/config":
            info = self.server.mbtiles.info
            tile_ext = "pbf" if info.tile_format == "pbf" else info.tile_format
            # MapLibre can overzoom the highest available vector tile.  Keep the
            # source tile limit intact while allowing operators to inspect a
            # detection/footprint several levels closer.
            interactive_max_zoom = min(24, max(22, int(info.max_zoom) + 4))
            self._json(
                {
                    "tileUrl": f"/tiles/{{z}}/{{x}}/{{y}}.{tile_ext}",
                    "minZoom": info.min_zoom,
                    "tileMaxZoom": info.max_zoom,
                    "maxZoom": interactive_max_zoom,
                    "center": [DEFAULT_CENTER_LON, DEFAULT_CENTER_LAT],
                    "zoom": DEFAULT_START_ZOOM,
                    "bounds": list(DEFAULT_BOUNDS),
                }
            )
            return
        if path == "/api/state":
            try:
                self._json(self.server.service.state())
            except Exception as exc:
                self._json({"ok": False, "error": str(exc)}, HTTPStatus.INTERNAL_SERVER_ERROR)
            return
        if path == "/api/coverage-settings":
            self._json({"ok": True, **load_coverage_settings()})
            return
        if path == "/api/mission":
            since = (parse_qs(parsed.query).get("since") or [None])[0]
            try:
                self._json(self.server.service.mission(since))
            except Exception as exc:
                self._json({"ok": False, "error": str(exc)}, HTTPStatus.INTERNAL_SERVER_ERROR)
            return
        if path.startswith("/tiles/"):
            self._tile(path)
            return
        if path.startswith("/vendor/"):
            self._file(VENDOR_DIR / path.removeprefix("/vendor/"), VENDOR_DIR)
            return
        if path in ("/", "/index.html"):
            self._file(WEB_DIR / "index.html", WEB_DIR)
            return
        if path.startswith("/web/"):
            self._file(WEB_DIR / path.removeprefix("/web/"), WEB_DIR)
            return
        if path in ("/app.js", "/style.css"):
            self._file(WEB_DIR / path.lstrip("/"), WEB_DIR)
            return
        self._text(HTTPStatus.NOT_FOUND, "Not found")

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        if path not in {"/api/shutdown", "/api/coverage-settings"}:
            self._json({"ok": False, "error": "Not found"}, HTTPStatus.NOT_FOUND)
            return
        if not self._is_loopback():
            self._json({"ok": False, "error": "Local access only"}, HTTPStatus.FORBIDDEN)
            return
        if path == "/api/coverage-settings":
            try:
                payload = self._read_json()
                if "footprint_interpolation_hz" not in payload:
                    raise ValueError("footprint_interpolation_hz is required")
                settings = save_coverage_settings(
                    {"footprint_interpolation_hz": payload["footprint_interpolation_hz"]}
                )
                self._json({"ok": True, **settings})
            except (ValueError, json.JSONDecodeError) as exc:
                self._json({"ok": False, "error": str(exc)}, HTTPStatus.BAD_REQUEST)
            except OSError as exc:
                self._json({"ok": False, "error": str(exc)}, HTTPStatus.INTERNAL_SERVER_ERROR)
            return
        self._json({"ok": True, "shuttingDown": True})
        threading.Thread(target=self.server.shutdown, daemon=True).start()

    def _is_loopback(self) -> bool:
        try:
            return ipaddress.ip_address(self.client_address[0]).is_loopback
        except Exception:
            return False

    def _read_json(self) -> dict[str, object]:
        try:
            length = int(self.headers.get("Content-Length", "0"))
        except ValueError as exc:
            raise ValueError("Invalid Content-Length") from exc
        if length <= 0 or length > 65536:
            raise ValueError("Invalid request body")
        payload = json.loads(self.rfile.read(length).decode("utf-8"))
        if not isinstance(payload, dict):
            raise ValueError("JSON object is required")
        return payload

    def _tile(self, path: str) -> None:
        parts = path.split("/")
        if len(parts) < 5:
            self._text(HTTPStatus.BAD_REQUEST, "Invalid tile path")
            return
        try:
            z, x = int(parts[2]), int(parts[3])
            y = int(os.path.splitext(parts[4])[0])
        except ValueError:
            self._text(HTTPStatus.BAD_REQUEST, "Invalid tile coordinates")
            return
        data = self.server.mbtiles.get_tile(z, x, y)
        if data is None:
            self._text(HTTPStatus.NOT_FOUND, "Tile not found")
            return
        content_type = "application/x-protobuf" if self.server.mbtiles.info.tile_format == "pbf" else "image/png"
        headers = {"Cache-Control": "public, max-age=3600"}
        if data[:2] == b"\x1f\x8b":
            headers["Content-Encoding"] = "gzip"
        self._bytes(HTTPStatus.OK, data, content_type, headers)

    def _file(self, path: Path, base: Path) -> None:
        try:
            resolved = path.resolve()
            resolved.relative_to(base.resolve())
        except Exception:
            self._text(HTTPStatus.FORBIDDEN, "Forbidden")
            return
        if not resolved.is_file():
            self._text(HTTPStatus.NOT_FOUND, "Not found")
            return
        content_type = mimetypes.guess_type(resolved.name)[0] or "application/octet-stream"
        self._bytes(HTTPStatus.OK, resolved.read_bytes(), content_type, {"Cache-Control": "no-cache"})

    def _json(self, value: object, status: HTTPStatus = HTTPStatus.OK) -> None:
        data = json.dumps(value, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self._bytes(status, data, "application/json; charset=utf-8", {"Cache-Control": "no-store"})

    def _text(self, status: HTTPStatus, value: str) -> None:
        self._bytes(status, value.encode("utf-8"), "text/plain; charset=utf-8", {})

    def _bytes(self, status: HTTPStatus, data: bytes, content_type: str, headers: dict[str, str]) -> None:
        self.send_response(int(status))
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        for key, value in headers.items():
            self.send_header(key, value)
        self.end_headers()
        self.wfile.write(data)


def main() -> int:
    parser = argparse.ArgumentParser(description="KU real-time mission status monitoring")
    parser.add_argument("--host", default=os.environ.get("MISSION_STATUS_MONITOR_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("MISSION_STATUS_MONITOR_PORT", "8300")))
    args = parser.parse_args()
    integration = ReadOnly0401Integration()
    server = MissionStatusHTTPServer((args.host, args.port), integration)
    browse_host = "127.0.0.1" if args.host in {"0.0.0.0", "::"} else args.host
    print(f"Mission status monitoring: http://{browse_host}:{server.server_address[1]}/", flush=True)
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        server.close_resources()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
