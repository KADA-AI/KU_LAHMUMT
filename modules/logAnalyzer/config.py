from __future__ import annotations

import json
import os
import socket
from pathlib import Path


def _env_path(name: str, fallback: Path) -> Path:
    value = os.getenv(name)
    if not value:
        return fallback
    return Path(value).expanduser().resolve()


def _env_int(name: str, fallback: int) -> int:
    value = os.getenv(name)
    if value is None or value == "":
        return fallback
    try:
        return int(value)
    except ValueError:
        return fallback


def _env_float(name: str, fallback: float) -> float:
    value = os.getenv(name)
    if value is None or value == "":
        return fallback
    try:
        return float(value)
    except ValueError:
        return fallback


LOG_ANALYZER_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = LOG_ANALYZER_DIR.parents[1]
CURRENT_SCENARIO_INFO_PATH = PROJECT_ROOT / "current_scenario.json"

# Map defaults (same as sim — Jipo-ri focus)
CENTER_LAT = _env_float("LOG_ANALYZER_CENTER_LAT", 38.057393)
CENTER_LON = _env_float("LOG_ANALYZER_CENTER_LON", 127.410630)
START_ZOOM = _env_float("LOG_ANALYZER_START_ZOOM", 11.5)
BOUNDS = (
    127.130588,  # min lon
    37.836639,   # min lat
    127.690671,  # max lon
    38.278147,   # max lat
)

SERVER_HOST = os.getenv("LOG_ANALYZER_HOST", "0.0.0.0")
SERVER_PORT = _env_int("LOG_ANALYZER_PORT", 8100)

WEB_DIR = _env_path("LOG_ANALYZER_WEB_DIR", LOG_ANALYZER_DIR / "web")
SIM_VENDOR_DIR = PROJECT_ROOT / "modules" / "sim" / "web" / "vendor"

# MBTiles path — search resource directories, same strategy as sim config.
_RESOURCE_DIR = _env_path("SIM_RESOURCE_DIR", PROJECT_ROOT / "resource")


def _find_mbtiles() -> Path:
    explicit = os.getenv("LOG_ANALYZER_MBTILES_PATH") or os.getenv("SIM_MBTILES_PATH")
    if explicit:
        p = Path(explicit).expanduser().resolve()
        if p.exists():
            return p
    candidates = [
        _RESOURCE_DIR / "korea.mbtiles",
        PROJECT_ROOT / "resource" / "korea.mbtiles",
    ]
    for c in candidates:
        if c.exists():
            return c
    # Return first candidate even if missing — caller must handle.
    return candidates[0]


MBTILES_PATH = _find_mbtiles()


def _load_current_scenario_info() -> dict:
    try:
        payload = json.loads(CURRENT_SCENARIO_INFO_PATH.read_text(encoding="utf-8"))
    except Exception:
        return {}
    return payload if isinstance(payload, dict) else {}


def _resolve_existing_path(value: object | None) -> Path | None:
    if value in (None, ""):
        return None
    try:
        path = Path(str(value)).expanduser().resolve()
    except Exception:
        return None
    return path


def resolve_logs_dir() -> Path:
    env_value = os.getenv("LOG_ANALYZER_LOGS_DIR")
    if env_value:
        return Path(env_value).expanduser().resolve()

    info = _load_current_scenario_info()
    base_root = _resolve_existing_path(info.get("base_root"))
    if base_root is not None:
        return base_root

    scenario_dir = _resolve_existing_path(info.get("scenario_dir"))
    if scenario_dir is not None:
        return scenario_dir.parent

    db_root = _resolve_existing_path(info.get("db_root"))
    if db_root is not None:
        db_parent = db_root.parent
        if db_parent.name.upper() == str(info.get("agency") or "").upper() and db_parent.parent.exists():
            return db_parent.parent
        return db_parent

    return (PROJECT_ROOT / "Logs").resolve()


LOGS_DIR = resolve_logs_dir()


def resolve_server_binding(
    host: str | None = None,
    port: int | None = None,
    *,
    search_span: int = 24,
) -> tuple[str, int]:
    bind_host = str(host or SERVER_HOST or "0.0.0.0").strip() or "0.0.0.0"
    preferred_port = int(port if port is not None else SERVER_PORT)
    if preferred_port < 0:
        preferred_port = 0

    def _can_bind(candidate_port: int) -> bool:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.bind((bind_host, int(candidate_port)))
            return True
        except OSError:
            return False
        finally:
            try:
                sock.close()
            except Exception:
                pass

    if preferred_port == 0:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            sock.bind((bind_host, 0))
            return bind_host, int(sock.getsockname()[1])
        finally:
            try:
                sock.close()
            except Exception:
                pass

    for candidate in range(preferred_port, preferred_port + max(1, int(search_span))):
        if _can_bind(candidate):
            return bind_host, int(candidate)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.bind((bind_host, 0))
        return bind_host, int(sock.getsockname()[1])
    finally:
        try:
            sock.close()
        except Exception:
            pass
