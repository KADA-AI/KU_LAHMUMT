from __future__ import annotations

import os
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

SERVER_HOST = os.getenv("SIM_SERVER_HOST", "203.252.161.43")
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
SIM_POS_TOL = _env_float("SIM_POS_TOL", 30.0)
SIM_SPEED_UAV = _env_float("SIM_SPEED_UAV", 90.0)
SIM_SPEED_LAH = _env_float("SIM_SPEED_LAH", 60.0)


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def wrap_deg(a: float) -> float:
    a %= 360.0
    if a < 0:
        a += 360.0
    return a
