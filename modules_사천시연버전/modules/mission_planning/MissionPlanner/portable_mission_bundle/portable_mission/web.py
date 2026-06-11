from __future__ import annotations

from pathlib import Path

from flask import Flask, jsonify, render_template, request

from .service import MissionService


def create_app(bundle_root: Path | None = None) -> Flask:
    root = bundle_root or Path(__file__).resolve().parents[1]
    template_dir = root / "portable_mission" / "templates"
    static_dir = root / "portable_mission" / "static"
    app = Flask(
        __name__,
        template_folder=str(template_dir),
        static_folder=str(static_dir),
        static_url_path="/static",
    )
    service = MissionService(root)

    @app.get("/")
    def index():
        return render_template("index.html")

    @app.get("/api/health")
    def api_health():
        return jsonify(service.health())

    @app.get("/api/model")
    def api_model():
        return jsonify(service.model_info())

    @app.get("/api/dems")
    def api_dems():
        return jsonify({"items": service.list_dems()})

    @app.get("/api/dem-preview")
    def api_dem_preview():
        name = request.args.get("name", "").strip()
        if not name:
            return _error("Query parameter 'name' is required.", 400)
        try:
            return jsonify(service.dem_preview(name))
        except FileNotFoundError as exc:
            return _error(str(exc), 404)
        except ValueError as exc:
            return _error(str(exc), 400)

    @app.post("/api/dems/upload")
    def api_upload_dem():
        file_storage = request.files.get("file")
        if file_storage is None:
            return _error("Form field 'file' is required.", 400)
        try:
            return jsonify(service.upload_dem(file_storage))
        except ValueError as exc:
            return _error(str(exc), 400)

    @app.post("/api/sessions")
    def api_create_session():
        payload = request.get_json(force=True, silent=True) or {}
        try:
            result = service.create_session(
                dem_name=payload.get("dem_name", ""),
                roi=payload.get("roi") or {},
                altitude_m=payload.get("altitude_m"),
                hex_step=payload.get("hex_step"),
                max_steps=payload.get("max_steps"),
                max_goals=payload.get("max_goals"),
            )
        except FileNotFoundError as exc:
            return _error(str(exc), 404)
        except (RuntimeError, ValueError) as exc:
            return _error(str(exc), 400)
        return jsonify(result)

    @app.post("/api/simulate")
    def api_simulate():
        payload = request.get_json(force=True, silent=True) or {}
        try:
            result = service.simulate(
                session_id=str(payload.get("session_id", "")),
                start_cell=payload.get("start_cell"),
                goal_cells=payload.get("goal_cells") or [],
                deterministic=bool(payload.get("deterministic", True)),
            )
        except FileNotFoundError as exc:
            return _error(str(exc), 404)
        except (RuntimeError, ValueError) as exc:
            return _error(str(exc), 400)
        return jsonify(result)

    return app


def _error(message: str, status: int):
    return jsonify({"error": message}), status
