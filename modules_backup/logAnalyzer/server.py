from __future__ import annotations

import json
import mimetypes
import os
import sqlite3
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional
from urllib.parse import parse_qs, urlparse

from .config import (
    BOUNDS,
    CENTER_LAT,
    CENTER_LON,
    MBTILES_PATH,
    SIM_VENDOR_DIR,
    START_ZOOM,
    WEB_DIR,
    resolve_logs_dir,
)
from .log_parser import list_scenarios, parse_scenario
from .validator import validate_scenario, validate_scenario_raw
from .dynamics_analysis import analyze_scenario_dynamics, save_scenario_dynamics_report


# ---------------------------------------------------------------------------
# MBTiles helper (lightweight, avoids coupling to sim.map.mbtiles)
# ---------------------------------------------------------------------------

class _MBTilesReader:
    """Minimal thread-safe reader for serving vector tiles from an mbtiles file."""

    def __init__(self, path: Path) -> None:
        self.path = Path(path)
        self._lock = threading.Lock()
        self._conn: sqlite3.Connection | None = None
        self._scheme = "tms"
        if self.path.exists():
            try:
                self._conn = sqlite3.connect(str(self.path), check_same_thread=False)
                self._conn.row_factory = sqlite3.Row
                self._load_scheme()
            except Exception:
                self._conn = None

    @property
    def available(self) -> bool:
        return self._conn is not None

    def get_tile(self, z: int, x: int, y: int) -> bytes | None:
        if self._conn is None or z < 0 or x < 0 or y < 0:
            return None
        tile_y = y
        if self._scheme == "tms":
            tile_y = (1 << z) - 1 - y
        with self._lock:
            try:
                row = self._conn.execute(
                    "SELECT tile_data FROM tiles WHERE zoom_level = ? AND tile_column = ? AND tile_row = ?",
                    (z, x, tile_y),
                ).fetchone()
            except Exception:
                return None
        if row is None:
            return None
        return row["tile_data"]

    def close(self) -> None:
        with self._lock:
            if self._conn is not None:
                try:
                    self._conn.close()
                except Exception:
                    pass
                self._conn = None

    def _load_scheme(self) -> None:
        if self._conn is None:
            return
        try:
            row = self._conn.execute(
                "SELECT value FROM metadata WHERE name = 'scheme'"
            ).fetchone()
            if row:
                self._scheme = str(row["value"]).lower()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# HTTP server
# ---------------------------------------------------------------------------

def _scan_folder_mtime(folder: Path) -> float:
    """Return the max mtime of JSON/JSONL files in a scenario folder (non-recursive shallow scan)."""
    best = 0.0
    try:
        for sub in (folder, folder / "MissionPlan", folder / "IndividualMissionPlan",
                     folder / "FlightPath", folder / "DSS_Internal",
                     folder / "simlog_0401", folder / "0401",
                     folder / "InputMissionPlan", folder / "MissionReferenceInfo",
                     folder / "DSS_Internal" / "replan_request_transport"):
            if not sub.is_dir():
                continue
            for f in sub.iterdir():
                if f.suffix in (".json", ".jsonl") and f.is_file():
                    try:
                        mt = f.stat().st_mtime
                        if mt > best:
                            best = mt
                    except Exception:
                        pass
    except Exception:
        pass
    return best


class _ScenarioCache:
    """Version-tracked cache for a single scenario."""
    __slots__ = ("data", "version", "folder_mtime")

    def __init__(self, data: dict, folder_mtime: float) -> None:
        self.data = data
        self.version = 1
        self.folder_mtime = folder_mtime

    def update(self, data: dict, folder_mtime: float) -> None:
        self.data = data
        self.folder_mtime = folder_mtime
        self.version += 1


class LogAnalyzerHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(
        self,
        server_address: tuple[str, int],
        web_dir: Path,
        sim_vendor_dir: Path,
        logs_dir: Path,
        mbtiles: _MBTilesReader,
    ):
        super().__init__(server_address, LogAnalyzerHandler)
        self.web_dir = Path(web_dir)
        self.sim_vendor_dir = Path(sim_vendor_dir)
        self.logs_dir = Path(logs_dir)
        self.mbtiles = mbtiles
        self.scenario_cache: dict[str, _ScenarioCache] = {}
        self._cache_lock = threading.Lock()

    def refresh_logs_dir(self) -> Path:
        next_logs_dir = Path(resolve_logs_dir())
        if next_logs_dir != self.logs_dir:
            self.logs_dir = next_logs_dir
            with self._cache_lock:
                self.scenario_cache.clear()
        return self.logs_dir


class LogAnalyzerServer:
    """Top-level wrapper — mirrors MapServer's API."""

    def __init__(self, host: str, port: int) -> None:
        self._mbtiles = _MBTilesReader(MBTILES_PATH)
        self._server = LogAnalyzerHTTPServer(
            (host, port),
            WEB_DIR,
            SIM_VENDOR_DIR,
            resolve_logs_dir(),
            self._mbtiles,
        )
        self._thread: threading.Thread | None = None

    @property
    def url(self) -> str:
        host, port = self._server.server_address
        return f"http://{host}:{port}/"

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if not self._thread or not self._thread.is_alive():
            return
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=2)
        self._mbtiles.close()


# ---------------------------------------------------------------------------
# Request handler
# ---------------------------------------------------------------------------

class LogAnalyzerHandler(BaseHTTPRequestHandler):
    server: LogAnalyzerHTTPServer

    def do_GET(self) -> None:
        path = urlparse(self.path).path

        if path in ("/", "/index.html"):
            self._serve_index()
            return
        if path == "/api/scenarios":
            self._serve_scenarios()
            return
        if path == "/api/scenario":
            self._serve_scenario()
            return
        if path == "/api/scenario/poll":
            self._serve_scenario_poll()
            return
        if path == "/api/scenario/validate":
            self._serve_scenario_validate()
            return
        if path == "/api/scenario/dynamics":
            self._serve_scenario_dynamics()
            return
        if path.startswith("/tiles/"):
            self._serve_tile(path)
            return
        if path.startswith("/vendor/"):
            self._serve_vendor(path)
            return
        # Fall-through: serve from WEB_DIR
        self._serve_static(path)

    def do_POST(self) -> None:
        path = urlparse(self.path).path

        if path == "/api/scenario/dynamics/save":
            self._serve_scenario_dynamics_save()
            return
        self._send_text(HTTPStatus.NOT_FOUND, "Not found")

    def log_message(self, fmt: str, *args: object) -> None:
        return

    def _current_logs_dir(self) -> Path:
        return self.server.refresh_logs_dir()

    def _request_base_url(self) -> str:
        scheme = str(self.headers.get("X-Forwarded-Proto") or "http").split(",")[0].strip() or "http"
        host = str(self.headers.get("Host") or "").strip()
        if host:
            return f"{scheme}://{host}"
        server_host, server_port = self.server.server_address
        bind_host = str(server_host).strip() or "127.0.0.1"
        if bind_host in {"0.0.0.0", "::", "[::]"}:
            bind_host = "127.0.0.1"
        return f"{scheme}://{bind_host}:{int(server_port)}"

    # ---- index ----

    def _serve_index(self) -> None:
        template_path = self.server.web_dir / "index.html"
        if not template_path.exists():
            self._send_text(HTTPStatus.NOT_FOUND, "Missing index.html")
            return
        html = template_path.read_text(encoding="utf-8")
        bounds_json = json.dumps([
            [BOUNDS[0], BOUNDS[1]],
            [BOUNDS[2], BOUNDS[3]],
        ])
        tile_url = f"{self._request_base_url()}/tiles/{{z}}/{{x}}/{{y}}.pbf"
        html = (
            html.replace("__TILE_URL__", tile_url)
            .replace("__CENTER_LAT__", f"{CENTER_LAT:.6f}")
            .replace("__CENTER_LON__", f"{CENTER_LON:.6f}")
            .replace("__START_ZOOM__", str(START_ZOOM))
            .replace("__MIN_ZOOM__", "6")
            .replace("__MAX_ZOOM__", "18")
            .replace("__BOUNDS_JSON__", bounds_json)
        )
        self._send_bytes(HTTPStatus.OK, html.encode("utf-8"), "text/html; charset=utf-8")

    # ---- API ----

    def _serve_scenarios(self) -> None:
        try:
            data = list_scenarios(self._current_logs_dir())
        except Exception as exc:
            self._send_json({"ok": False, "error": str(exc)}, HTTPStatus.INTERNAL_SERVER_ERROR)
            return
        self._send_json({"ok": True, "data": data})

    def _resolve_scenario(self, name: str) -> tuple[_ScenarioCache | None, str | None]:
        """Load or refresh a scenario, returning (cache_entry, error)."""
        scenario_path = self._current_logs_dir() / name
        sbc3 = scenario_path / "SBC3" if (scenario_path / "SBC3").is_dir() else scenario_path
        if not scenario_path.is_dir():
            return None, f"Scenario not found: {name}"

        folder_mtime = _scan_folder_mtime(sbc3)
        with self.server._cache_lock:
            cached = self.server.scenario_cache.get(name)
            if cached is not None and cached.folder_mtime >= folder_mtime:
                return cached, None

        try:
            result = parse_scenario(scenario_path)
        except Exception as exc:
            return None, str(exc)

        with self.server._cache_lock:
            existing = self.server.scenario_cache.get(name)
            if existing is not None:
                existing.update(result, folder_mtime)
                return existing, None
            entry = _ScenarioCache(result, folder_mtime)
            self.server.scenario_cache[name] = entry
            return entry, None

    def _serve_scenario(self) -> None:
        parsed = urlparse(self.path)
        query = parse_qs(parsed.query or "")
        name = (query.get("name") or [""])[0].strip()
        if not name:
            self._send_json({"ok": False, "error": "name parameter required"}, HTTPStatus.BAD_REQUEST)
            return

        entry, error = self._resolve_scenario(name)
        if error:
            self._send_json({"ok": False, "error": error}, HTTPStatus.NOT_FOUND)
            return
        self._send_json({"ok": True, "version": entry.version, "data": entry.data})

    def _serve_scenario_poll(self) -> None:
        """Lightweight poll endpoint. Re-parses only if files changed.
        Returns full data if version differs, otherwise just {changed: false}.
        """
        parsed = urlparse(self.path)
        query = parse_qs(parsed.query or "")
        name = (query.get("name") or [""])[0].strip()
        client_version = 0
        try:
            client_version = int((query.get("v") or ["0"])[0])
        except Exception:
            pass

        if not name:
            self._send_json({"ok": False, "error": "name required"}, HTTPStatus.BAD_REQUEST)
            return

        entry, error = self._resolve_scenario(name)
        if error:
            self._send_json({"ok": False, "error": error}, HTTPStatus.NOT_FOUND)
            return

        if entry.version == client_version:
            self._send_json({"ok": True, "changed": False, "version": entry.version})
        else:
            self._send_json({"ok": True, "changed": True, "version": entry.version, "data": entry.data})

    # ---- validation ----

    def _serve_scenario_validate(self) -> None:
        """Run ICD validation on a scenario and return the issues list."""
        parsed = urlparse(self.path)
        query = parse_qs(parsed.query or "")
        name = (query.get("name") or [""])[0].strip()
        if not name:
            self._send_json({"ok": False, "error": "name parameter required"}, HTTPStatus.BAD_REQUEST)
            return

        scenario_path = self._current_logs_dir() / name
        if not scenario_path.is_dir():
            self._send_json({"ok": False, "error": f"Scenario not found: {name}"}, HTTPStatus.NOT_FOUND)
            return

        # First try raw file validation for deeper checks (loiterProperty etc.)
        issues: list[dict] = []
        try:
            issues = validate_scenario_raw(scenario_path)
        except Exception:
            pass

        # Also run parsed-data validation to catch cross-reference issues
        # Force cache refresh to avoid stale data
        with self.server._cache_lock:
            self.server.scenario_cache.pop(name, None)
        entry, error = self._resolve_scenario(name)
        if entry is not None:
            try:
                parsed_issues = validate_scenario(entry.data)
                # Merge, avoiding exact duplicates by (location, field, message)
                existing_keys = {(i["location"], i["field"], i["message"]) for i in issues}
                for pi in parsed_issues:
                    key = (pi["location"], pi["field"], pi["message"])
                    if key not in existing_keys:
                        issues.append(pi)
                        existing_keys.add(key)
            except Exception:
                pass

        # Sort: errors first, then warnings, then info
        severity_order = {"error": 0, "warning": 1, "info": 2}
        issues.sort(key=lambda i: (severity_order.get(i.get("severity"), 9), i.get("location", "")))

        summary = {
            "error": sum(1 for i in issues if i["severity"] == "error"),
            "warning": sum(1 for i in issues if i["severity"] == "warning"),
            "info": sum(1 for i in issues if i["severity"] == "info"),
        }

        self._send_json({
            "ok": True,
            "scenario": name,
            "summary": summary,
            "total": len(issues),
            "issues": issues,
        })

    def _serve_scenario_dynamics(self) -> None:
        """Run read-only 0401 dynamics analysis for a scenario."""
        parsed = urlparse(self.path)
        query = parse_qs(parsed.query or "")
        name = (query.get("name") or [""])[0].strip()
        if not name:
            self._send_json({"ok": False, "error": "name parameter required"}, HTTPStatus.BAD_REQUEST)
            return

        scenario_path = self._current_logs_dir() / name
        if not scenario_path.is_dir():
            self._send_json({"ok": False, "error": f"Scenario not found: {name}"}, HTTPStatus.NOT_FOUND)
            return

        try:
            result = analyze_scenario_dynamics(scenario_path)
        except Exception as exc:
            self._send_json({"ok": False, "error": str(exc)}, HTTPStatus.INTERNAL_SERVER_ERROR)
            return
        status = HTTPStatus.OK if result.get("ok") else HTTPStatus.BAD_REQUEST
        self._send_json(result, status)

    def _serve_scenario_dynamics_save(self) -> None:
        """Persist a read-only dynamics analysis report for later planner use."""
        parsed = urlparse(self.path)
        query = parse_qs(parsed.query or "")
        name = (query.get("name") or [""])[0].strip()
        if not name:
            self._send_json({"ok": False, "error": "name parameter required"}, HTTPStatus.BAD_REQUEST)
            return

        scenario_path = self._current_logs_dir() / name
        if not scenario_path.is_dir():
            self._send_json({"ok": False, "error": f"Scenario not found: {name}"}, HTTPStatus.NOT_FOUND)
            return

        try:
            saved = save_scenario_dynamics_report(scenario_path)
        except Exception as exc:
            self._send_json({"ok": False, "error": str(exc)}, HTTPStatus.INTERNAL_SERVER_ERROR)
            return
        self._send_json(saved)

    # ---- tiles ----

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
        content_type = "application/vnd.mapbox-vector-tile"
        encoding = None
        if data[:2] == b"\x1f\x8b":
            encoding = "gzip"
        self._send_bytes(
            HTTPStatus.OK,
            data,
            content_type,
            encoding=encoding,
            cache_control="public, max-age=3600",
        )

    # ---- vendor ----

    def _serve_vendor(self, path: str) -> None:
        rel = path[len("/vendor/"):]
        file_path = _safe_join(self.server.sim_vendor_dir, rel)
        if file_path is None or not file_path.is_file():
            self._send_text(HTTPStatus.NOT_FOUND, "Not found")
            return
        mime, _ = mimetypes.guess_type(str(file_path))
        content_type = _ensure_utf8_content_type(mime or "application/octet-stream")
        self._send_bytes(
            HTTPStatus.OK,
            file_path.read_bytes(),
            content_type,
            cache_control="public, max-age=3600",
        )

    # ---- static ----

    def _serve_static(self, path: str) -> None:
        file_path = _safe_join(self.server.web_dir, path)
        if file_path is None or not file_path.is_file():
            self._send_text(HTTPStatus.NOT_FOUND, "Not found")
            return
        mime, _ = mimetypes.guess_type(str(file_path))
        content_type = _ensure_utf8_content_type(mime or "application/octet-stream")
        self._send_bytes(HTTPStatus.OK, file_path.read_bytes(), content_type)

    # ---- response helpers ----

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


# ---------------------------------------------------------------------------
# Utilities
# ---------------------------------------------------------------------------

def _safe_join(base_dir: Path, request_path: str) -> Optional[Path]:
    safe_path = os.path.normpath(request_path).lstrip("/\\")
    full = (base_dir / safe_path).resolve()
    if base_dir.resolve() != full and base_dir.resolve() not in full.parents:
        return None
    return full


def _ensure_utf8_content_type(content_type: str) -> str:
    if "charset=" in content_type.lower():
        return content_type
    base = content_type.split(";", 1)[0].strip().lower()
    if base.startswith("text/"):
        return f"{base}; charset=utf-8"
    if base in (
        "application/javascript",
        "text/javascript",
        "application/json",
        "application/xml",
        "image/svg+xml",
    ):
        return f"{base}; charset=utf-8"
    if base.endswith("+json") or base.endswith("+xml"):
        return f"{base}; charset=utf-8"
    return content_type
