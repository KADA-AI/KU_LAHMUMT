"""Shared constants, imports, and FOV DB helpers."""
from __future__ import annotations

import copy  # noqa: F401
import itertools  # noqa: F401
import math  # noqa: F401
import os  # noqa: F401
import sys  # noqa: F401
import traceback  # noqa: F401
from dataclasses import dataclass, field  # noqa: F401
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple  # noqa: F401

from PyQt5.QtCore import QPoint, QPointF, QRectF, Qt, pyqtSignal  # noqa: F401
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QPolygonF  # noqa: F401
from PyQt5.QtWidgets import (  # noqa: F401
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QFrame,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QPlainTextEdit,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)
from shapely.geometry import LineString, MultiPolygon, Point, Polygon, box  # noqa: F401
from shapely.ops import split as geom_split, unary_union  # noqa: F401

from modules.mission_planning.MissionPlanner.planning_enhanced.algo import run_split_pipeline, review_overflow_areas  # noqa: F401
from modules.mission_planning.MissionPlanner.planning_enhanced.algo.area_review import review_assigned_areas_local  # noqa: F401
from modules.mission_planning.MissionPlanner.planning_enhanced.algo.split_runner import assign_split_result_by_takeover_distance  # noqa: F401
from modules.mission_planning.MissionPlanner.planning_enhanced.io.export_0302 import (  # noqa: F401
    build_0302_packages_from_split_with_lah,
    save_0302_packages,
)
from modules.mission_planning.MissionPlanner.planning_enhanced.io.export_0303_0304 import (  # noqa: F401
    build_0303_0304_from_0302_packages,
    save_0303_plans,
    save_0304_plans,
)
from modules.mission_planning.MissionPlanner.planning_enhanced.models import SplitPiece, SplitRunResult  # noqa: F401
from modules.mission_planning.MissionPlanner.planning_enhanced.pathing import (  # noqa: F401
    calculate_expected_velocity,
    generate_expected_paths,
)
from modules.mission_planning.MissionPlanner.planning_enhanced.scheduling import run_milp_scheduling  # noqa: F401
from modules.mission_planning.MissionPlanner.planning_enhanced.type_decider import (  # noqa: F401
    PROFILE_DEFAULT,
    apply_logic_type_decider,
)
try:
    from modules.mission_planning.MissionPlanner.runtime_settings import (  # noqa: F401
        get_runtime_area_auto_fov_from_db,
        get_runtime_area_review_max_segment_m,
        fov_db_path,
        get_runtime_float,
        get_runtime_int,
        get_runtime_str,
        load_fov_db_rows,
    )
except Exception:
    from modules.mission_planning.MissionPlanner.runtime_settings import (  # type: ignore  # noqa: F401
        get_runtime_area_auto_fov_from_db,
        get_runtime_area_review_max_segment_m,
        fov_db_path,
        get_runtime_float,
        get_runtime_int,
        get_runtime_str,
        load_fov_db_rows,
    )

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
_R = 6_378_137.0
_ORIGIN_LAT = 37.8535
_ORIGIN_LON = 127.4465
_DEFAULT_ALT = 0.0
_INITIAL_HALF_SPAN_M = 2_000.0
_MIN_HALF_SPAN_M = 125.0
_MAX_HALF_SPAN_M = 20_000.0
_UAV_IDS = [4, 5, 6]
_MISSION_PLANNER_DIR = Path(__file__).resolve().parents[2] / "MissionPlanner"
_PROJECT_ROOT = Path(__file__).resolve().parents[4]
_FOV_DB_PATH = fov_db_path()

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

UAV_COLORS: Dict[int, str] = {
    4: "#e53935",
    5: "#1d4ed8",
    6: "#0f9d58",
}

TURN_RADIUS_BY_SPEED_MPS: Dict[float, float] = {
    30.0: 340.0,
    40.0: 450.0,
    50.0: 560.0,
}
def _turn_radius_for_speed(speed_mps: float) -> float:
    """TURN_RADIUS_BY_SPEED_MPS 테이블에서 선형 보간하여 선회 반경(m)을 반환."""
    keys = sorted(TURN_RADIUS_BY_SPEED_MPS.keys())
    if speed_mps <= keys[0]:
        return TURN_RADIUS_BY_SPEED_MPS[keys[0]]
    if speed_mps >= keys[-1]:
        return TURN_RADIUS_BY_SPEED_MPS[keys[-1]]
    for i in range(len(keys) - 1):
        lo, hi = keys[i], keys[i + 1]
        if lo <= speed_mps <= hi:
            t = (speed_mps - lo) / (hi - lo)
            return TURN_RADIUS_BY_SPEED_MPS[lo] + t * (TURN_RADIUS_BY_SPEED_MPS[hi] - TURN_RADIUS_BY_SPEED_MPS[lo])
    return TURN_RADIUS_BY_SPEED_MPS[keys[-1]]

TURN_PREVIEW_SPEED_MPS = 40.0
TURN_PREVIEW_RADIUS_M = TURN_RADIUS_BY_SPEED_MPS[TURN_PREVIEW_SPEED_MPS]
TURN_PREVIEW_BANK_DEG = 30.0
TURN_PREVIEW_HORIZON_S = 5.0
ASSIGNMENT_PATH_ARC_STEP_S = 1.0
ASSIGNMENT_PATH1_ANGLE_WEIGHT = 2.0
ASSIGNMENT_PATH1_TURN_TIME_WEIGHT = 1.0

STAGE2_GRID_SIZE_SMALL_M = 40.0
STAGE2_GRID_SIZE_MEDIUM_M = 70.0
STAGE2_GRID_SIZE_LARGE_M = 120.0
STAGE2_GRID_BOUND_SMALL_M = 1_000.0
STAGE2_GRID_BOUND_MEDIUM_M = 2_000.0
STAGE2_MIN_CELL_AREA_RATIO = 0.12
STAGE2_DEFAULT_AREA_RATE_M2PS = 1800.0
STAGE2_ANCHOR_BLEND = 0.35
STAGE2_MAX_SWATH_WIDTH_M = 500.0
STAGE2_SMOOTH_BUFFER_RATIO = 0.42
STAGE2_SIMPLIFY_RATIO = 0.90
STAGE2_SIMPLIFY_MIN_M = 18.0
STAGE2_PAIR_RELAX_BUFFER_RATIO = 0.16
STAGE2_PAIR_SIMPLIFY_RATIO = 0.38
STAGE2_OVERLAP_BUFFER_RATIO = 0.10
NEXT_COLLAB_SWEEP_STEP_RATIO = 0.60
NEXT_COLLAB_ENTRY_TPRIME_TARGET_SEP_RATIO = 0.30
NEXT_COLLAB_ENTRY_TPRIME_RATIO_SCALE = 0.50
NEXT_COLLAB_AREA_PATH0_TRIGGER_SEP_M = 3000.0
NEXT_COLLAB_AREA_PATH0_TARGET_SEP_RATIO = 0.20
NEXT_COLLAB_TAKEOVER_FIRST_STEP_RATIO = 0.40


# ---------------------------------------------------------------------------
# FOV DB helpers
# ---------------------------------------------------------------------------
def _cached_fov_db_rows() -> Tuple[Tuple[float, float, float, float, float], ...]:
    rows: List[Tuple[float, float, float, float, float]] = []
    try:
        for row in load_fov_db_rows():
            try:
                width_m = float(row.get("width", 0.0) or 0.0)
                sep_m = float(row.get("sep", 0.0) or 0.0)
                vel = float(row.get("vel", 0.0) or 0.0)
                fov = float(row.get("fov", 0.0) or 0.0)
                foot_m = float(row.get("foot", 0.0) or 0.0)
            except Exception:
                continue
            if width_m > 0.0:
                rows.append((float(width_m), float(sep_m), float(vel), float(fov), float(foot_m)))
    except Exception:
        rows = []
    rows.sort(key=lambda item: (float(item[0]), float(item[1])))
    return tuple(rows)


def _largest_sep_covering_db_row_for_width(width_m: float) -> Optional[Dict[str, float]]:
    rows = _cached_fov_db_rows()
    target_m = max(0.0, float(width_m))
    if target_m <= 0.0 or not rows:
        return None
    matching = [r for r in rows if float(r[0]) >= target_m]
    if not matching:
        matching = list(rows)
    best = max(matching, key=lambda r: float(r[1]))
    db_width_m = float(best[0])
    sep_m = float(best[1])
    vel = float(best[2])
    fov = float(best[3])
    return {
        "width": float(db_width_m),
        "sep": float(sep_m),
        "vel": float(vel),
        "fov": float(fov),
        "foot": float(best[4]) if len(best) > 4 else 0.0,
    }


def _prepare_legacy_missionplanner_path() -> None:
    candidate = str(_MISSION_PLANNER_DIR)
    if candidate not in sys.path:
        sys.path.insert(0, candidate)
