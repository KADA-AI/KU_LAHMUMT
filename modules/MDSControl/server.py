from __future__ import annotations

import argparse
import json
import mimetypes
import os
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import unquote, urlparse


MODULE_DIR = Path(__file__).resolve().parent
WEB_DIR = MODULE_DIR / "web"


def _project_root() -> Path:
    current = Path(__file__).resolve()
    for parent in current.parents:
        if (parent / "run.py").exists():
            return parent
    return current.parents[2]


PROJECT_ROOT = _project_root()
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from modules.mission_planning.MissionPlanner.runtime_settings import (  # noqa: E402
    canonicalize_runtime_payload,
    clear_runtime_settings_cache,
    load_runtime_settings,
    settings_path as mission_settings_path,
)
from modules.monitoring.logic.replan_runtime_settings import (  # noqa: E402
    defaults_path as replan_defaults_path,
    load_recommended_defaults,
    load_replan_settings,
    save_recommended_defaults,
    save_replan_settings,
    settings_path as replan_settings_path,
)


def _json_response(handler: BaseHTTPRequestHandler, status: int, payload: Any) -> None:
    body = json.dumps(payload, ensure_ascii=False, indent=2).encode("utf-8")
    handler.send_response(status)
    handler.send_header("Content-Type", "application/json; charset=utf-8")
    handler.send_header("Cache-Control", "no-store")
    handler.send_header("Content-Length", str(len(body)))
    handler.end_headers()
    handler.wfile.write(body)


def _read_json_body(handler: BaseHTTPRequestHandler) -> dict[str, Any]:
    length = int(handler.headers.get("Content-Length") or "0")
    raw = handler.rfile.read(length) if length > 0 else b"{}"
    try:
        payload = json.loads(raw.decode("utf-8"))
    except Exception as exc:
        raise ValueError(f"invalid JSON body: {exc}") from exc
    if not isinstance(payload, dict):
        raise ValueError("JSON body must be an object")
    return payload


def _save_mission_settings(payload: dict[str, Any]) -> dict[str, Any]:
    current = load_runtime_settings()
    merged = _deep_merge(current, payload)
    normalized = canonicalize_runtime_payload(merged)
    path = mission_settings_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(normalized, ensure_ascii=False, indent=2), encoding="utf-8")
    clear_runtime_settings_cache()
    return normalized


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    merged = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge(dict(merged[key]), value)
        else:
            merged[key] = value
    return merged


def _settings_payload() -> dict[str, Any]:
    return {
        "mission": load_runtime_settings(),
        "replan": load_replan_settings(),
        "recommendedReplan": load_recommended_defaults(),
        "paths": {
            "mission": str(mission_settings_path()),
            "replan": str(replan_settings_path()),
            "replanDefaults": str(replan_defaults_path()),
        },
    }


class MDSControlHandler(BaseHTTPRequestHandler):
    server_version = "MDSControl/1.0"

    def log_message(self, fmt: str, *args: Any) -> None:
        sys.stderr.write("[MDSControl] " + (fmt % args) + "\n")

    def do_GET(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        if parsed.path == "/api/settings":
            _json_response(self, 200, _settings_payload())
            return
        self._serve_static(parsed.path)

    def do_POST(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        try:
            if parsed.path == "/api/settings":
                payload = _read_json_body(self)
                result: dict[str, Any] = {}
                if isinstance(payload.get("mission"), dict):
                    result["mission"] = _save_mission_settings(payload["mission"])
                if isinstance(payload.get("replan"), dict):
                    result["replan"] = save_replan_settings(payload["replan"])
                if not result:
                    raise ValueError("nothing to save")
                result["paths"] = _settings_payload()["paths"]
                _json_response(self, 200, result)
                return
            if parsed.path == "/api/replan/recommended":
                payload = _read_json_body(self)
                replan = payload.get("replan")
                if not isinstance(replan, dict):
                    raise ValueError("replan object is required")
                _json_response(self, 200, {"recommendedReplan": save_recommended_defaults(replan)})
                return
            _json_response(self, 404, {"error": "not found"})
        except ValueError as exc:
            _json_response(self, 400, {"error": str(exc)})
        except Exception as exc:
            _json_response(self, 500, {"error": str(exc)})

    def _serve_static(self, raw_path: str) -> None:
        rel = unquote(raw_path.lstrip("/")) or "index.html"
        if rel.endswith("/"):
            rel += "index.html"
        target = (WEB_DIR / rel).resolve()
        try:
            target.relative_to(WEB_DIR.resolve())
        except ValueError:
            self.send_error(403)
            return
        if not target.exists() or not target.is_file():
            self.send_error(404)
            return
        content_type = mimetypes.guess_type(str(target))[0] or "application/octet-stream"
        data = target.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", f"{content_type}; charset=utf-8" if content_type.startswith("text/") else content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def main() -> None:
    parser = argparse.ArgumentParser(description="MDS mission planning control surface")
    parser.add_argument("--host", default=os.getenv("MDS_CONTROL_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.getenv("MDS_CONTROL_PORT", "8200")))
    args = parser.parse_args()

    server = ThreadingHTTPServer((str(args.host), int(args.port)), MDSControlHandler)
    url = f"http://{args.host}:{args.port}/"
    print(f"MDSControl server listening on {url}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()

