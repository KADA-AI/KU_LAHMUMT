from __future__ import annotations

from pathlib import Path
from typing import Dict
try:
    from modules.mission_planning.MissionPlanner.runtime_settings import fov_db_path
except Exception:
    fov_db_path = None

PACKAGE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = Path(__file__).resolve().parents[3]
MISSION_PLANNER_DIR = PROJECT_ROOT / "modules" / "mission_planning" / "MissionPlanner"
OUTPUT_ROOT = PACKAGE_DIR / "output"
FOV_DB_PATH = fov_db_path() if fov_db_path is not None else PROJECT_ROOT / "resource" / "db" / "fov_db.csv"

WINDOW_TITLE = "다음 영역 임무수행 모드"
FLOW_MODE_ENV_KEY = "MISSION_NEXT_AREA_FLOW_MODE"

R_EARTH_M = 6_378_137.0
ORIGIN_LAT = 37.8535
ORIGIN_LON = 127.4465
DEFAULT_ALT_M = 0.0
INITIAL_HALF_SPAN_M = 2_000.0
MIN_HALF_SPAN_M = 125.0
MAX_HALF_SPAN_M = 20_000.0

UAV_IDS = [4, 5, 6]
UAV_COLORS: Dict[int, str] = {
    4: "#e53935",
    5: "#1d4ed8",
    6: "#0f9d58",
}

MODE_IDLE = "idle"
MODE_DRAW_AREA = "draw_area"
MODE_DRAW_LINE = "draw_line"
MODE_LINE_WIDTH_PENDING = "line_width_pending"
MODE_MISSION_READY = "mission_ready"
MODE_PLACE_UAV = "place_uav"
MODE_SET_UAV_HEADING = "set_uav_heading"
MODE_RESULT_READY = "result_ready"

MISSION_AREA = "area"
MISSION_LINE = "line"

TURN_RADIUS_BY_SPEED_MPS: Dict[float, float] = {
    30.0: 340.0,
    40.0: 450.0,
    50.0: 560.0,
}
TURN_PREVIEW_SPEED_MPS = 40.0
TURN_PREVIEW_RADIUS_M = TURN_RADIUS_BY_SPEED_MPS[TURN_PREVIEW_SPEED_MPS]
TURN_PREVIEW_BANK_DEG = 30.0
TURN_PREVIEW_HORIZON_S = 5.0
MAKE_PATH_INTERVAL_M = 1_000.0

STAGE2_GRID_SIZE_SMALL_M = 40.0
STAGE2_GRID_SIZE_MEDIUM_M = 70.0
STAGE2_GRID_SIZE_LARGE_M = 120.0
STAGE2_GRID_BOUND_SMALL_M = 1_000.0
STAGE2_GRID_BOUND_MEDIUM_M = 2_000.0
STAGE2_MIN_CELL_AREA_RATIO = 0.12
STAGE2_DEFAULT_AREA_RATE_M2PS = 1_800.0
STAGE2_ANCHOR_BLEND = 0.35
STAGE2_MAX_SWATH_WIDTH_M = 500.0
STAGE2_SMOOTH_BUFFER_RATIO = 0.42
STAGE2_SIMPLIFY_RATIO = 0.90
STAGE2_SIMPLIFY_MIN_M = 18.0
STAGE2_PAIR_RELAX_BUFFER_RATIO = 0.16
STAGE2_PAIR_SIMPLIFY_RATIO = 0.38
STAGE2_OVERLAP_BUFFER_RATIO = 0.10
