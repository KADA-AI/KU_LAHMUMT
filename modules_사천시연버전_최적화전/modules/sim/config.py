from __future__ import annotations

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


SIM_DIR = Path(__file__).resolve().parent
ROOT_DIR = SIM_DIR.parents[1]
RESOURCE_DIR = _env_path("SIM_RESOURCE_DIR", ROOT_DIR / "resource")
WEB_DIR = _env_path("SIM_WEB_DIR", SIM_DIR / "web")

MBTILES_PATH = _env_path("SIM_MBTILES_PATH", RESOURCE_DIR / "korea.mbtiles")
DEM_DIR = _env_path("SIM_DEM_DIR", RESOURCE_DIR)

SERVER_HOST = os.getenv("SIM_SERVER_HOST", "0.0.0.0")
SERVER_PORT = _env_int("SIM_SERVER_PORT", 8000)

# Map defaults (Jipo-ri focus)
DEFAULT_CENTER_LAT = _env_float("SIM_CENTER_LAT", 38.057393)
DEFAULT_CENTER_LON = _env_float("SIM_CENTER_LON", 127.410630)
DEFAULT_START_ZOOM = _env_float("SIM_START_ZOOM", 11.5)
DEFAULT_BOUNDS = (
    127.130588,  # min lon
    37.836639,   # min lat
    127.690671,  # max lon
    38.278147,   # max lat
)

# DEM settings
DEM_TILE_SIZE = _env_int("SIM_DEM_TILE_SIZE", 256)
DEM_MAX_ZOOM = _env_int("SIM_DEM_MAX_ZOOM", 8)
DEM_ENCODING = os.getenv("SIM_DEM_ENCODING", "terrarium")
# Keep true terrain height (1.0 = no vertical exaggeration).
DEM_EXAGGERATION = 1.0
DEM_PITCH_THRESHOLD = _env_float("SIM_DEM_PITCH_THRESHOLD", 2.0)

# Simulation timing defaults (runtime)
SIM_BASE_DT = _env_float("SIM_BASE_DT", 0.01)
SIM_TIME_SCALE = _env_float("SIM_TIME_SCALE", 20.0)
SIM_INTERNAL_STEP_HZ = _env_float("SIM_INTERNAL_STEP_HZ", 15.0)
SIM_0401_IDLE_HZ = _env_float("SIM_0401_IDLE_HZ", 5.0)
SIM_0401_ACTIVE_HZ = _env_float("SIM_0401_ACTIVE_HZ", 5.0)
SIM_POS_TOL = _env_float("SIM_POS_TOL", 30.0)
SIM_SPEED_UAV = _env_float("SIM_SPEED_UAV", 90.0)
SIM_SPEED_LAH = _env_float("SIM_SPEED_LAH", 60.0)
SIM_HISTORY_MAX = _env_int("SIM_HISTORY_MAX", 2000)
SIM_HISTORY_SAMPLE_HZ = _env_float("SIM_HISTORY_SAMPLE_HZ", 24.0)
SIM_HISTORY_RESPONSE_MAX = _env_int("SIM_HISTORY_RESPONSE_MAX", 120)
SIM_MAX_STEPS_PER_LOOP = _env_int("SIM_MAX_STEPS_PER_LOOP", 8)
SIM_0402_HISTORY_MAX = _env_int("SIM_0402_HISTORY_MAX", 200)
SIM_PROJECTILE_MAX = _env_int("SIM_PROJECTILE_MAX", 120)
SIM_PROJECTILE_SPEED_GUN = _env_float("SIM_PROJECTILE_SPEED_GUN", 700.0)
SIM_PROJECTILE_SPEED_MISSILE = _env_float("SIM_PROJECTILE_SPEED_MISSILE", 900.0)
SIM_PROJECTILE_HIT_RADIUS_GUN = _env_float("SIM_PROJECTILE_HIT_RADIUS_GUN", 15.0)
SIM_PROJECTILE_HIT_RADIUS_MISSILE = _env_float("SIM_PROJECTILE_HIT_RADIUS_MISSILE", 35.0)
SIM_LAH_AUTO_ATTACK = _env_int("SIM_LAH_AUTO_ATTACK", 0)
SIM_ENEMY_HIT_SCALE = _env_float("SIM_ENEMY_HIT_SCALE", 0.0)

# Auto-tracking defaults
SIM_UAV_DETECT_RANGE_M = _env_float("SIM_UAV_DETECT_RANGE_M", 5000.0)
SIM_TRACK_LOITER_RADIUS_M = _env_float("SIM_TRACK_LOITER_RADIUS_M", 900.0)
SIM_TRACK_LOITER_SPEED_MPS = _env_float("SIM_TRACK_LOITER_SPEED_MPS", 70.0)
SIM_TRACK_ALT_BUFFER_M = _env_float("SIM_TRACK_ALT_BUFFER_M", 80.0)
SIM_TRACK_LOST_TIMEOUT_S = _env_float("SIM_TRACK_LOST_TIMEOUT_S", 6.0)
SIM_AUTO_TRACK_ALWAYS = _env_int("SIM_AUTO_TRACK_ALWAYS", 1)
SIM_AUTO_TRACK_TAKEOVER = _env_int("SIM_AUTO_TRACK_TAKEOVER", 1)
SIM_MULTI_TARGET_PREVIEW_SEC = _env_float("SIM_MULTI_TARGET_PREVIEW_SEC", 1.0)
SIM_ROI_GAZE_DURATION_S = _env_float("SIM_ROI_GAZE_DURATION_S", 5.0)
SIM_INPUT_ADVANCE_GUARD_SEC = _env_float("SIM_INPUT_ADVANCE_GUARD_SEC", 10.0)
SIM_FRIENDLY_HIT_SCALE = _env_float("SIM_FRIENDLY_HIT_SCALE", 0.4)


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def wrap_deg(a: float) -> float:
    a %= 360.0
    if a < 0:
        a += 360.0
    return a


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
