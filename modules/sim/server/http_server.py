from __future__ import annotations

import json
import math
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
    DEM_TILE_CACHE_MAX,
    DEM_TILE_SIZE,
    MBTILES_PATH,
    RESOURCE_DIR,
    WEB_DIR,
)
from ..integration.integration_service import IntegrationService
from ..mission.mission_loader import load_flight_paths
from ..mission.mission_plan_loader import (
    build_mission_plan_payload,
    normalize_input_mission_plan_float_fields,
)
from ..mission.input_mission_reissue import (
    load_type1_target_order_candidates,
    now_ms_2000,
    prepare_reissued_input_mission_0201,
    prepare_type1_new_target_input_mission_0201,
    prepare_type1_target_order_input_mission_0201,
)
from ..mission.remaining_area_loader import build_remaining_area_snapshot
from ..mission.mission_validator import validate_mission_payload, validate_mission_plan_result
from ..runtime.sim_service import SimulationService
from ..tune.log_dynamics_calibrator import (
    analyze_log_path,
    analyze_uploaded_0401_files,
    apply_proposal,
    current_status,
    disable_profile,
)
from ..map.dem_tiles import load_dem_provider
from ..map.mbtiles import MBTiles


def _agent_label_from_aircraft(aircraft_id: int) -> Optional[str]:
    if 1 <= aircraft_id <= 3:
        return f"LAH{aircraft_id}"
    if 4 <= aircraft_id <= 6:
        return f"UAV{aircraft_id - 3}"
    return None


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
            provider = load_dem_provider(
                DEM_DIR,
                tile_size=DEM_TILE_SIZE,
                max_zoom=DEM_MAX_ZOOM,
                cache_max_tiles=DEM_TILE_CACHE_MAX,
            )
            if provider.available:
                self.dem_provider = provider
        self.integration = IntegrationService()
        self.sim = SimulationService()
        try:
            self.sim.set_integration(self.integration)
        except Exception:
            pass
        try:
            self.integration.set_sim_service(self.sim)
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
        dem_bounds_json = _format_bounds(_dem_bounds_lonlat(self.server.dem_provider))
        html = (
            html.replace("__TILE_URL__", tile_url)
            .replace("__MIN_ZOOM__", str(info.min_zoom))
            .replace("__MAX_ZOOM__", str(info.max_zoom))
            .replace("__CENTER_LAT__", f"{center_lat:.6f}")
            .replace("__CENTER_LON__", f"{center_lon:.6f}")
            .replace("__START_ZOOM__", str(start_zoom))
            .replace("__BOUNDS_JSON__", bounds_json)
            .replace("__DEM_URL__", dem_url)
            .replace("__DEM_BOUNDS__", dem_bounds_json)
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
        if parsed.path == "/api/integration/settings":
            self._send_json(integration.get_settings())
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
        if path == "/api/integration/activate":
            ready = integration.ensure_ready()
            status = HTTPStatus.OK if ready else HTTPStatus.SERVICE_UNAVAILABLE
            self._send_json(
                {
                    "ok": ready,
                    "enabled": integration.enabled,
                    "error": integration.error,
                },
                status,
            )
            return
        if path == "/api/integration/generate":
            msg_id = (body or {}).get("msgId")
            result = integration.generate(msg_id)
            self._send_json(result)
            return
        if path == "/api/integration/reset":
            result = integration.reset_state()
            self._send_json(result)
            return
        if path == "/api/integration/send_custom":
            msg_id = (body or {}).get("msgId")
            payload = (body or {}).get("body")
            result = integration.send_custom(msg_id, payload)
            self._send_json(result)
            return
        if path == "/api/integration/payload_observation":
            result = integration.record_payload_observation(body or {})
            status = HTTPStatus.OK if result.get("ok") else HTTPStatus.BAD_REQUEST
            self._send_json(result, status)
            return
        if path == "/api/integration/settings":
            result = integration.update_settings(body or {})
            status = HTTPStatus.OK if result.get("ok") else HTTPStatus.BAD_REQUEST
            self._send_json(result, status)
            return
        self._send_json({"ok": False, "error": "Not found"}, HTTPStatus.NOT_FOUND)

    def _serve_mission_get(self, path: str) -> None:
        if path == "/api/mission/ping":
            self._send_json({"ok": True})
            return
        self._send_json({"ok": False, "error": "Not found"}, HTTPStatus.NOT_FOUND)

    def _serve_mission_post(self, path: str) -> None:
        if path == "/api/mission/load":
            body = self._read_json()
            base_path = (body or {}).get("path", "")
            result = load_flight_paths(str(base_path), project_root=Path(__file__).resolve().parents[3])
            self._send_json(result)
            return
        if path == "/api/mission/plan_load":
            body = self._read_json()
            mission_plan_id = (body or {}).get("missionPlanID") or (body or {}).get("missionPlanId")
            raw = body or {}
            if any(key in raw for key in ("preserveState", "preserve_state", "keepState", "keep_state")):
                preserve_state = bool(
                    raw.get("preserveState")
                    or raw.get("preserve_state")
                    or raw.get("keepState")
                    or raw.get("keep_state")
                )
            else:
                preserve_state = True
            skip_if_loaded = bool(
                raw.get("skipIfMissionPlanAlreadyLoaded")
                or raw.get("skip_if_mission_plan_already_loaded")
            )
            try:
                mission_plan_id = int(mission_plan_id)
            except Exception:
                mission_plan_id = None
            if mission_plan_id is None:
                self._send_json({"ok": False, "error": "missionPlanID required"}, HTTPStatus.BAD_REQUEST)
                return
            sim = getattr(self.server, "sim", None)
            if sim is None:
                self._send_json({"ok": False, "error": "Simulation unavailable"}, HTTPStatus.SERVICE_UNAVAILABLE)
                return
            integration = getattr(self.server, "integration", None)
            has_pending = getattr(integration, "has_pending_next_collab_transition", None)
            if callable(has_pending) and bool(has_pending()):
                self._send_json(
                    {
                        "ok": True,
                        "deferred": True,
                        "missionPlanID": int(mission_plan_id),
                        "message": "Waiting for the validated next-collaborative mission plan.",
                    },
                    HTTPStatus.ACCEPTED,
                )
                return
            plan_result = build_mission_plan_payload(mission_plan_id)
            if not plan_result.get("ok"):
                self._send_json(plan_result, HTTPStatus.BAD_REQUEST)
                return
            validation = validate_mission_plan_result(plan_result)
            payload = dict(plan_result.get("payload") or {})
            payload["missionPlanID"] = int(mission_plan_id)
            if preserve_state:
                payload["preserveState"] = True
            if skip_if_loaded:
                payload["skipIfMissionPlanAlreadyLoaded"] = True
            sim_result = sim.load_mission(payload)
            ok = bool(sim_result.get("ok"))
            response = dict(plan_result)
            response["ok"] = ok
            response["sim"] = sim_result
            response["validation"] = validation
            if not ok and sim_result.get("error"):
                response["error"] = sim_result.get("error")
            self._send_json(response)
            return
        self._send_json({"ok": False, "error": "Not found"}, HTTPStatus.NOT_FOUND)

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
        if parsed.path == "/api/sim/remaining_areas":
            query = parse_qs(parsed.query or "")
            plan_val = (query.get("missionPlanID") or query.get("missionPlanId") or [None])[0]
            plan_id = None
            if plan_val is not None:
                try:
                    plan_id = int(float(plan_val))
                except Exception:
                    self._send_json(
                        {"ok": False, "error": "missionPlanID must be an integer."},
                        HTTPStatus.BAD_REQUEST,
                    )
                    return
            else:
                resolve_plan_id = getattr(sim, "current_mission_plan_id", None)
                if callable(resolve_plan_id):
                    plan_id = resolve_plan_id()
            self._send_json(build_remaining_area_snapshot(mission_plan_id=plan_id))
            return
        if parsed.path == "/api/sim/dynamics_calibration/status":
            self._send_json(current_status())
            return
        if parsed.path == "/api/sim/type1_target_order":
            query = parse_qs(parsed.query or "")
            source_value = (
                query.get("sourcePackageID")
                or query.get("sourcePackageId")
                or [None]
            )[0]
            source_package_id = None
            if source_value is not None:
                try:
                    source_package_id = int(source_value)
                except Exception:
                    self._send_json(
                        {"ok": False, "error": "sourcePackageID must be an integer."},
                        HTTPStatus.BAD_REQUEST,
                    )
                    return
            result = load_type1_target_order_candidates(
                source_package_id=source_package_id,
            )
            self._send_json(
                result,
                HTTPStatus.OK if result.get("ok") else HTTPStatus.BAD_REQUEST,
            )
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
            payload = body or {}
            for plan in payload.get("inputMissionPlans") or []:
                if isinstance(plan, dict):
                    normalize_input_mission_plan_float_fields(plan)
            validation = validate_mission_payload(payload)
            result = sim.load_mission(payload)
            result["validation"] = validation
            self._send_json(result)
            return
        if path == "/api/sim/targets/add":
            result = sim.add_target(body or {})
            self._send_json(result)
            return
        if path == "/api/sim/roi/add":
            result = sim.add_roi_mock(body or {})
            self._send_json(result)
            return
        if path == "/api/sim/targets/clear":
            result = sim.clear_targets()
            self._send_json(result)
            return
        if path == "/api/sim/type1_new_target_0201":
            integration = getattr(self.server, "integration", None)
            if integration is None:
                self._send_json(
                    {"ok": False, "error": "Integration unavailable"},
                    HTTPStatus.SERVICE_UNAVAILABLE,
                )
                return
            if not integration.ensure_ready():
                self._send_json(
                    {"ok": False, "error": integration.error or "Messenger not ready"},
                    HTTPStatus.SERVICE_UNAVAILABLE,
                )
                return

            payload = body or {}
            source_package_id = None
            for key in ("sourcePackageID", "sourcePackageId", "inputMissionPackageID"):
                if key not in payload:
                    continue
                try:
                    source_package_id = int(payload.get(key))
                except Exception:
                    source_package_id = None
                break

            timestamp = now_ms_2000()
            result = prepare_type1_new_target_input_mission_0201(
                coordinate_list=payload.get("coordinateList"),
                source_package_id=source_package_id,
                now_ms=lambda: timestamp,
            )
            if not result.get("ok"):
                self._send_json(result, HTTPStatus.BAD_REQUEST)
                return

            send_body = {
                "timestamp": int(result.get("timestamp") or timestamp),
                "inputMissionPackageID": int(result.get("newPackageID")),
            }
            send_result = integration.send_custom("0201", send_body)
            response = dict(result)
            response["sendBody"] = send_body
            response["send"] = send_result
            if not send_result.get("ok"):
                response["ok"] = False
                response["error"] = send_result.get("error") or "0201 send failed"
                self._send_json(response, HTTPStatus.SERVICE_UNAVAILABLE)
                return
            response["message"] = (
                f"Type-1 new target 0201 sent with InputMissionPlan "
                f"{send_body['inputMissionPackageID']}"
            )
            self._send_json(response)
            return
        if path == "/api/sim/type1_target_order_0201":
            integration = getattr(self.server, "integration", None)
            if integration is None:
                self._send_json(
                    {"ok": False, "error": "Integration unavailable"},
                    HTTPStatus.SERVICE_UNAVAILABLE,
                )
                return
            if not integration.ensure_ready():
                self._send_json(
                    {"ok": False, "error": integration.error or "Messenger not ready"},
                    HTTPStatus.SERVICE_UNAVAILABLE,
                )
                return

            payload = body or {}
            source_package_id = None
            for key in ("sourcePackageID", "sourcePackageId", "inputMissionPackageID"):
                if key not in payload:
                    continue
                try:
                    source_package_id = int(payload.get(key))
                except Exception:
                    source_package_id = None
                break

            timestamp = now_ms_2000()
            result = prepare_type1_target_order_input_mission_0201(
                ordered_target_input_mission_ids=payload.get("targetInputMissionIDOrder"),
                source_package_id=source_package_id,
                now_ms=lambda: timestamp,
            )
            if not result.get("ok"):
                self._send_json(result, HTTPStatus.BAD_REQUEST)
                return

            send_body = {
                "timestamp": int(result.get("timestamp") or timestamp),
                "inputMissionPackageID": int(result.get("newPackageID")),
            }
            send_result = integration.send_custom("0201", send_body)
            response = dict(result)
            response["sendBody"] = send_body
            response["send"] = send_result
            if not send_result.get("ok"):
                response["ok"] = False
                response["error"] = send_result.get("error") or "0201 send failed"
                self._send_json(response, HTTPStatus.SERVICE_UNAVAILABLE)
                return
            response["message"] = (
                f"Type-1 target order 0201 sent with InputMissionPlan "
                f"{send_body['inputMissionPackageID']}"
            )
            self._send_json(response)
            return
        if path == "/api/sim/reissue_input_0201":
            integration = getattr(self.server, "integration", None)
            if integration is None:
                self._send_json(
                    {"ok": False, "error": "Integration unavailable"},
                    HTTPStatus.SERVICE_UNAVAILABLE,
                )
                return
            if not integration.ensure_ready():
                self._send_json(
                    {"ok": False, "error": integration.error or "Messenger not ready"},
                    HTTPStatus.SERVICE_UNAVAILABLE,
                )
                return
            payload = body or {}
            source_package_id = None
            for key in ("sourcePackageID", "sourcePackageId", "inputMissionPackageID", "inputMissionPackageId"):
                if key not in payload:
                    continue
                try:
                    source_package_id = int(payload.get(key))
                except Exception:
                    source_package_id = None
                break
            if source_package_id is None:
                resolver = getattr(integration, "_resolve_reexecute_source_package_id", None)
                if callable(resolver):
                    try:
                        source_package_id = resolver()
                    except Exception:
                        source_package_id = None

            timestamp = now_ms_2000()
            result = prepare_reissued_input_mission_0201(
                source_package_id=source_package_id,
                now_ms=lambda: timestamp,
            )
            if not result.get("ok"):
                self._send_json(result, HTTPStatus.BAD_REQUEST)
                return

            send_body = {
                "timestamp": int(result.get("timestamp") or timestamp),
                "inputMissionPackageID": int(result.get("newPackageID")),
            }
            send_result = integration.send_custom("0201", send_body)
            response = dict(result)
            response["sendBody"] = send_body
            response["send"] = send_result
            if not send_result.get("ok"):
                response["ok"] = False
                response["error"] = send_result.get("error") or "0201 send failed"
                self._send_json(response, HTTPStatus.SERVICE_UNAVAILABLE)
                return
            response["message"] = f"0201 sent with InputMissionPlan {send_body['inputMissionPackageID']}"
            self._send_json(response)
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
        if path == "/api/sim/skip_to_mission_start":
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
            result = sim.skip_to_current_mission_start(aircraft_id)
            status = HTTPStatus.OK if result.get("ok") else HTTPStatus.BAD_REQUEST
            self._send_json(result, status)
            return
        if path == "/api/sim/agent_state":
            payload = body or {}
            label = None
            agent = payload.get("agent") or payload.get("label")
            if isinstance(agent, str) and agent.strip():
                label = agent.strip().upper()
            aircraft_id = None
            if label is None:
                for key in ("aircraftId", "aircraftID"):
                    if key in payload:
                        try:
                            aircraft_id = int(payload.get(key))
                        except Exception:
                            aircraft_id = None
                        break
                if aircraft_id is not None:
                    label = _agent_label_from_aircraft(aircraft_id)
            result = sim.update_agent_state(label or "", payload)
            self._send_json(result)
            return
        if path == "/api/sim/force_command":
            payload = body or {}
            aircraft_id = payload.get("aircraftID") or payload.get("aircraftId")
            mandatory_type = payload.get("mandatoryType")
            try:
                aircraft_id = int(aircraft_id)
            except Exception:
                aircraft_id = None
            try:
                mandatory_type = int(mandatory_type)
            except Exception:
                mandatory_type = 0
            if aircraft_id is None:
                self._send_json({"ok": False, "error": "aircraftID required"})
                return
            result = sim.apply_force_command(aircraft_id, mandatory_type)
            self._send_json(result)
            return
        if path == "/api/sim/shinil_mission_profile":
            payload = body or {}
            enabled = payload.get("enabled")
            if not isinstance(enabled, bool):
                self._send_json(
                    {"ok": False, "error": "enabled must be a boolean"},
                    HTTPStatus.BAD_REQUEST,
                )
                return
            self._send_json(sim.set_shinil_mission_profile(enabled))
            return
        if path == "/api/sim/auto_target_placement":
            payload = body or {}
            enabled = payload.get("enabled")
            if not isinstance(enabled, bool):
                self._send_json(
                    {"ok": False, "error": "enabled must be a boolean"},
                    HTTPStatus.BAD_REQUEST,
                )
                return
            apply_current = payload.get("applyCurrent", True)
            if not isinstance(apply_current, bool):
                self._send_json(
                    {"ok": False, "error": "applyCurrent must be a boolean"},
                    HTTPStatus.BAD_REQUEST,
                )
                return
            self._send_json(
                sim.set_auto_target_placement(enabled, apply_current=apply_current)
            )
            return
        if path == "/api/sim/play":
            settings_payload = (body or {}).get("integrationSettings")
            settings_result = None
            integration = getattr(self.server, "integration", None)
            if integration is not None and isinstance(settings_payload, dict):
                settings_result = integration.update_settings(settings_payload)
                if not settings_result.get("ok"):
                    self._send_json(settings_result, HTTPStatus.BAD_REQUEST)
                    return
            if integration is not None and not integration.ensure_ready():
                self._send_json(
                    {
                        "ok": False,
                        "error": integration.error or "Integration unavailable",
                        "integrationSettings": settings_result,
                    },
                    HTTPStatus.SERVICE_UNAVAILABLE,
                )
                return
            result = sim.play()
            if settings_result is not None:
                result["integrationSettings"] = settings_result
            self._send_json(result)
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
        if path == "/api/sim/dynamics_calibration/analyze":
            payload = body or {}
            try:
                result = analyze_log_path(str(payload.get("path") or ""))
                self._send_json(result)
            except FileNotFoundError as exc:
                self._send_json({"ok": False, "error": str(exc)}, HTTPStatus.NOT_FOUND)
            except Exception as exc:
                self._send_json({"ok": False, "error": str(exc)}, HTTPStatus.BAD_REQUEST)
            return
        if path == "/api/sim/dynamics_calibration/analyze_upload":
            payload = body or {}
            try:
                result = analyze_uploaded_0401_files(payload.get("files") if isinstance(payload, dict) else [])
                self._send_json(result)
            except FileNotFoundError as exc:
                self._send_json({"ok": False, "error": str(exc)}, HTTPStatus.NOT_FOUND)
            except Exception as exc:
                self._send_json({"ok": False, "error": str(exc)}, HTTPStatus.BAD_REQUEST)
            return
        if path == "/api/sim/dynamics_calibration/apply":
            payload = body or {}
            proposal = payload.get("proposal")
            try:
                result = apply_proposal(proposal if isinstance(proposal, dict) else {})
                self._send_json(result)
            except Exception as exc:
                self._send_json({"ok": False, "error": str(exc)}, HTTPStatus.BAD_REQUEST)
            return
        if path == "/api/sim/dynamics_calibration/disable":
            try:
                self._send_json(disable_profile())
            except Exception as exc:
                self._send_json({"ok": False, "error": str(exc)}, HTTPStatus.BAD_REQUEST)
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


def _mercator_to_lonlat(x: float, y: float) -> tuple[float, float]:
    lon = (float(x) / 20037508.342789244) * 180.0
    lat = (float(y) / 20037508.342789244) * 180.0
    lat = 180.0 / math.pi * (2.0 * math.atan(math.exp(lat * math.pi / 180.0)) - (math.pi / 2.0))
    return lon, lat


def _dem_bounds_lonlat(provider) -> Optional[tuple[float, float, float, float]]:
    if provider is None:
        return None
    bounds = getattr(provider, "bounds_mercator", None)
    if not bounds:
        return None
    west, south, east, north = bounds
    min_lon, min_lat = _mercator_to_lonlat(west, south)
    max_lon, max_lat = _mercator_to_lonlat(east, north)
    return (min_lon, min_lat, max_lon, max_lat)
