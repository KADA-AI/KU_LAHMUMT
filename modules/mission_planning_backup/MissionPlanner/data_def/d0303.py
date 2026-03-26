from __future__ import annotations
import os
import math
import csv
import threading
from collections import OrderedDict
from copy import deepcopy
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Dict, List, Optional, Tuple
from .mission_helpers import now_ms_since_2000, terrain_elev
from .id_allocator import (
    next_waypoint_id as _next_waypoint_id,
    reserve_waypoint_block as _reserve_waypoint_block,
)
from .search_speed import spacing_based_search_speed
try:
    from ..Search_Speed_2 import SearchSpeedCalculator as _SearchSpeedCalculator
except ImportError:
    try:
        from Search_Speed_2 import SearchSpeedCalculator as _SearchSpeedCalculator  # type: ignore
    except ImportError:
        _SearchSpeedCalculator = None

try:
    from ..config import DEFAULT_SWEEP_SEPARATION_M, USE_DB_FOR_CORRIDOR, SWEEP_SPACING_MARGIN, DB_FOV_WEIGHT
except ImportError:
    try:
        from config import DEFAULT_SWEEP_SEPARATION_M, USE_DB_FOR_CORRIDOR, SWEEP_SPACING_MARGIN, DB_FOV_WEIGHT  # type: ignore
    except ImportError:
        from modules.mission_planning.MissionPlanner.config import (  # type: ignore
            DEFAULT_SWEEP_SEPARATION_M,
            USE_DB_FOR_CORRIDOR,
            SWEEP_SPACING_MARGIN,
            DB_FOV_WEIGHT,
        )
try:
    from ..runtime_settings import (
        get_runtime_str as _get_runtime_str,
        get_runtime_area_review_max_segment_m as _get_runtime_area_review_max_segment_m,
        get_runtime_value as _get_runtime_value,
        is_runtime_custom_preset as _is_runtime_custom_preset,
        load_runtime_settings as _load_runtime_settings,
    )
except Exception:
    try:
        from runtime_settings import (  # type: ignore
            get_runtime_str as _get_runtime_str,
            get_runtime_area_review_max_segment_m as _get_runtime_area_review_max_segment_m,
            get_runtime_value as _get_runtime_value,
            is_runtime_custom_preset as _is_runtime_custom_preset,
            load_runtime_settings as _load_runtime_settings,
        )
    except Exception:
        try:
            from modules.mission_planning.MissionPlanner.runtime_settings import (  # type: ignore
                get_runtime_str as _get_runtime_str,
                get_runtime_area_review_max_segment_m as _get_runtime_area_review_max_segment_m,
                get_runtime_value as _get_runtime_value,
                is_runtime_custom_preset as _is_runtime_custom_preset,
                load_runtime_settings as _load_runtime_settings,
            )
        except Exception:
            _get_runtime_str = None
            _get_runtime_area_review_max_segment_m = None
            _get_runtime_value = None
            _is_runtime_custom_preset = None
            _load_runtime_settings = None
try:
    from ...logic_test.dubins_test.dubins_turn_link_logic import Point2D as _DubinsPoint2D, compute_turn_link as _compute_turn_link
except Exception:
    try:
        from modules.mission_planning.logic_test.dubins_test.dubins_turn_link_logic import (  # type: ignore
            Point2D as _DubinsPoint2D,
            compute_turn_link as _compute_turn_link,
        )
    except Exception:
        _DubinsPoint2D = None
        _compute_turn_link = None
try:
    from ..DB import select_best_config as _select_best_config
except ImportError:
    try:
        from DB import select_best_config as _select_best_config  # type: ignore
    except ImportError:
        _select_best_config = None
from UAV_missionPlanning import UAVMissionPlanner
from . import route_planner_algorithms as route_algos
from .coord_transform import llh_to_xy, xy_to_llh
try:
    from ..tools.UAV_pattern.Nadir_BF.area_nadir_bf_planner import build_nadir_bf_overflight_coords
except Exception:
    try:
        from tools.UAV_pattern.Nadir_BF.area_nadir_bf_planner import build_nadir_bf_overflight_coords  # type: ignore
    except Exception:
        try:
            from modules.mission_planning.MissionPlanner.tools.UAV_pattern.Nadir_BF.area_nadir_bf_planner import (  # type: ignore
                build_nadir_bf_overflight_coords,
            )
        except Exception:
            build_nadir_bf_overflight_coords = None
try:
    from ....common.eta import _order_by_next_chain, _time_from_prev_to_curr_s
except ImportError:
    from modules.common.eta import _order_by_next_chain, _time_from_prev_to_curr_s


def _load_fov_db_rows() -> list[dict]:
    global _FOV_DB_ROWS_CACHE
    if _FOV_DB_ROWS_CACHE is not None:
        return _FOV_DB_ROWS_CACHE
    rows: list[dict] = []
    try:
        with _FOV_DB_PATH.open("r", encoding="utf-8-sig", newline="") as fh:
            reader = csv.DictReader(fh)
            for row in reader:
                try:
                    rows.append(
                        {
                            "width": float(row.get("width", 0.0) or 0.0),
                            "vel": float(row.get("vel", 0.0) or 0.0),
                            "fov": float(row.get("fov", 0.0) or 0.0),
                            "sep": float(row.get("sep", 0.0) or 0.0),
                        }
                    )
                except Exception:
                    continue
    except Exception:
        rows = []
    _FOV_DB_ROWS_CACHE = rows
    return rows


def _apply_db_fov_weight(fov_deg: float) -> float:
    try:
        weight = float(DB_FOV_WEIGHT)
    except Exception:
        weight = 1.0
    if weight <= 0.0:
        weight = 1.0
    try:
        fov = float(fov_deg)
    except Exception:
        fov = 0.0
    return max(fov * weight, 0.0)


def _select_nadir_fov_by_altitude(altitude_m: float, default_fov: float) -> float:
    rows = _load_fov_db_rows()
    if not rows:
        return float(default_fov)
    alt_ref = max(0.0, float(altitude_m))
    cands = [r for r in rows if float(r.get("sep", 0.0) or 0.0) <= alt_ref + 1e-9]
    if not cands:
        return float(default_fov)
    best = max(
        cands,
        key=lambda r: (
            float(r.get("fov", 0.0) or 0.0),
            float(r.get("sep", 0.0) or 0.0),
            -abs(float(r.get("sep", 0.0) or 0.0) - alt_ref),
            float(r.get("vel", 0.0) or 0.0),
        ),
    )
    picked = float(best.get("fov", default_fov) or default_fov)
    if picked <= 0.0:
        return float(default_fov)
    return _apply_db_fov_weight(picked)


def _rows_by_width(rows: list[dict], width_ref_m: float) -> list[dict]:
    if not rows:
        return []
    req = max(0.0, float(width_ref_m))
    if req <= 0.0:
        return list(rows)

    cands = [r for r in rows if float(r.get("width", 0.0) or 0.0) + 1e-9 >= req]
    if cands:
        return cands

    nearest_w = min(
        (float(r.get("width", 0.0) or 0.0) for r in rows),
        key=lambda w: abs(w - req),
    )
    return [r for r in rows if abs(float(r.get("width", 0.0) or 0.0) - nearest_w) <= 1e-9]


def _select_balanced_fov_db_row(width_ref_m: float) -> dict | None:
    rows = _rows_by_width(_load_fov_db_rows(), width_ref_m)
    if not rows:
        return None
    if len(rows) == 1:
        return rows[0]

    width_ref = max(0.0, float(width_ref_m))
    fov_values = [float(r.get("fov", 0.0) or 0.0) for r in rows]
    vel_values = [float(r.get("vel", 0.0) or 0.0) for r in rows]
    fov_scale = max(max(fov_values) - min(fov_values), 1e-9)
    vel_scale = max(max(vel_values) - min(vel_values), 1e-9)

    max_fov_row = max(
        rows,
        key=lambda r: (
            float(r.get("fov", 0.0) or 0.0),
            float(r.get("vel", 0.0) or 0.0),
            -max(float(r.get("width", 0.0) or 0.0) - width_ref, 0.0),
            float(r.get("sep", 0.0) or 0.0),
        ),
    )
    max_vel_row = max(
        rows,
        key=lambda r: (
            float(r.get("vel", 0.0) or 0.0),
            float(r.get("fov", 0.0) or 0.0),
            -max(float(r.get("width", 0.0) or 0.0) - width_ref, 0.0),
            float(r.get("sep", 0.0) or 0.0),
        ),
    )
    target_fov = (
        float(max_fov_row.get("fov", 0.0) or 0.0)
        + float(max_vel_row.get("fov", 0.0) or 0.0)
    ) / 2.0
    target_vel = (
        float(max_fov_row.get("vel", 0.0) or 0.0)
        + float(max_vel_row.get("vel", 0.0) or 0.0)
    ) / 2.0

    return min(
        rows,
        key=lambda r: (
            (
                ((float(r.get("fov", 0.0) or 0.0) - target_fov) / fov_scale) ** 2
                + ((float(r.get("vel", 0.0) or 0.0) - target_vel) / vel_scale) ** 2
            ),
            max(float(r.get("width", 0.0) or 0.0) - width_ref, 0.0),
            -float(r.get("sep", 0.0) or 0.0),
            -float(r.get("fov", 0.0) or 0.0),
            -float(r.get("vel", 0.0) or 0.0),
        ),
    )


def _sw_code(default: str = "MMR") -> str:
    role = (os.environ.get("KU_ROLE") or "").lower()
    return {
        "mission": "MMR",
        "monitoring": "MSM",
        "decision": "MOB",
    }.get(role, default)


# ── 타입 alias (가독용) ─────────────────────────────
Point = Tuple[float, float]
Line  = Tuple[Point, Point]

# ── 고정 상수 ───────────────────────────────────────────
FOV_DEG         = 10
AREA_CUSTOM_FOV_DEG = 10
AREA_NADIR_FOV_DEG = 31.2
SWEEP_ENTRY_OFFSET_M = 500.0
SWEEP_ENTRY_OFFSET_TAKEOVER_M = 200.0
SWEEP_ENTRY_OFFSET_FOLLOWON_M = 300.0
LINE_ENTRY_SKIP_IF_TAKEOVER_WITHIN_M = 500.0
# NOTE: Temporary switch requested by ops.
# When False, entry waypoint generation is disabled from the 2nd mission onward.
ENABLE_FOLLOWON_ENTRY_WP = False
SWEEP_MERGE_HEADING_DEG = 5
AREA_SWEEP_MERGE_HEADING_DEG = 15.0
AREA_FORCE_STRAIGHT_LINESEARCH = True
SWEEP_LINE_INTERP_POINTS = 3  # >=2; controls how many sample points are emitted per sweep line
Altitude = 610
ALTITUDE_LAYERS_M = (610.0, 620.0, 630.0)
DEFAULT_SEARCH_SPEED_MULTIPLIER = 16.0
AREA_SEARCH_SPEED_WEIGHT = 1.2
AREA_FIRST_PACKET_SEARCH_SPEED_SCALE = 1.2
AREA_FIRST_PACKET_SWEEP_GROUP_SCALE = 2.0
POINT_FOV_DEG = 31.2
MIN_SWEEP_LEN_M = 3.0
MIN_ROUTE_SPACING_M = 200.0
SWEEP_ROUTE_WP_SPACING_M = 2000.0
AREA_SWEEP_ROUTE_WP_SPACING_M = 2000.0
LINE_SWEEP_DENSITY_SCALE = 1.18
# 1.5x denser area sweep than the previous baseline (spacing ~= 2/3).
AREA_SWEEP_DENSITY_SCALE = 1.5
AREA_POINT_ANCHOR_LEAD_IN_M = 1000.0
AREA_ROUTE_OFFSET_SCALE = 0.5
AREA_OUTPUT_FOV_SCALE = 3.0
SWEEP_MERGE_MODE = "heading"
ENTRY_HOLD_FOV_DEG = 10.0
ENTRY_HOLD_GIMBAL_PITCH = -90.0
ENTRY_HOLD_GIMBAL_YAW = 0.0
LOITER_RADIUS_M = 800
LOITER_DIRECTION = 1
LOITER_TIME_S = 30
LOITER_SPEED_MPS = 30
DUBINS_TURN_RADIUS_M = 450.0
AREA_DUBINS_ENTRY_LINKS_ENABLED = False
ROUTE_PLANNER_NAME = "dtatrim"
_PROJECT_ROOT = Path(__file__).resolve().parents[4]
_FOV_DB_PATH = _PROJECT_ROOT / "resource" / "db" / "fov_db.csv"
_FOV_DB_ROWS_CACHE: list[dict] | None = None
_SWEEP_DEBUG_CACHE: set[tuple[str, float, float, float]] = set()
_SWEEP_DEBUG_LOCK = threading.Lock()


def _select_corridor_db_config(width: float) -> dict | None:
    if not USE_DB_FOR_CORRIDOR:
        return None
    cfg = _select_balanced_fov_db_row(float(width))
    if cfg is not None:
        return cfg
    if _select_best_config is None:
        return None
    try:
        return _select_best_config(float(width))
    except Exception:
        return None


def _runtime_auto_fov_from_db(default: bool = True) -> bool:
    payload = None
    if _load_runtime_settings is not None:
        try:
            payload = _load_runtime_settings()
        except Exception:
            payload = None
    if _is_runtime_custom_preset is not None:
        try:
            if _is_runtime_custom_preset(payload):
                return False
        except Exception:
            pass
    if _get_runtime_value is None:
        return bool(default)
    try:
        return bool(_get_runtime_value("enhanced_auto_fov_from_db", default, payload))
    except TypeError:
        try:
            return bool(_get_runtime_value("enhanced_auto_fov_from_db", default))
        except Exception:
            return bool(default)
    except Exception:
        return bool(default)


def _runtime_area_review_max_segment_m(default: float = 3000.0) -> float:
    if _get_runtime_area_review_max_segment_m is not None:
        try:
            return float(_get_runtime_area_review_max_segment_m(default))
        except Exception:
            pass
    if _get_runtime_value is None:
        return float(default)
    try:
        value = float(_get_runtime_value("enhanced_area_review_max_segment_m", default))
    except Exception:
        value = float(default)
    return max(value, 0.0)


def _db_velocity_to_cruise_speed_mps(vel_kmh: float) -> float:
    try:
        return round(max(float(vel_kmh), 0.0) / 3.6, 2)
    except Exception:
        return 0.0


def _projected_polygon_span_m(poly_llh: list[tuple[float, float]], bearing_deg: float) -> float:
    if len(poly_llh) < 3:
        return 0.0
    lat0, lon0 = poly_llh[0]
    poly_xy = [llh_to_xy(lat, lon, lat0, lon0) for lat, lon in poly_llh]
    th = math.radians(float(bearing_deg))
    nx, ny = math.cos(th), -math.sin(th)
    vals = [float(nx * x + ny * y) for x, y in poly_xy]
    if not vals:
        return 0.0
    return max(vals) - min(vals)


def _select_area_db_config(poly_llh: list[tuple[float, float]], bearing_deg: float) -> dict | None:
    if not _runtime_auto_fov_from_db():
        return None
    span_m = _projected_polygon_span_m(poly_llh, bearing_deg)
    if span_m <= 0.0:
        return None
    max_segment_m = _runtime_area_review_max_segment_m()
    width_ref_m = span_m
    if max_segment_m > 0.0:
        width_ref_m = min(width_ref_m, max_segment_m)
    width_ref_m = max(float(width_ref_m), 1.0)
    cfg = _select_balanced_fov_db_row(width_ref_m)
    if cfg is None:
        if _select_best_config is None:
            return None
        try:
            cfg = _select_best_config(width_ref_m)
        except Exception:
            return None
    if not isinstance(cfg, dict):
        return None
    return {
        "config": cfg,
        "span_m": float(span_m),
        "width_ref_m": float(width_ref_m),
        "max_segment_m": float(max_segment_m),
    }


def _normalize_altitude(value: Optional[float], default: int = Altitude) -> int:
    """고도를 정수(m)로 정규화."""
    if value is None:
        return int(default)
    try:
        alt = float(value)
    except (TypeError, ValueError):
        alt = float(default)
    return int(round(alt))


@dataclass(frozen=True)
class SweepConfig:
    separation_m: float
    fov_deg: float

SWEEP_GEOMETRY = SweepConfig(
    separation_m=DEFAULT_SWEEP_SEPARATION_M,
    fov_deg=FOV_DEG,
)

def _active_sweep_geometry() -> SweepConfig:
    return SWEEP_GEOMETRY


def set_route_planner(name: str) -> None:
    """Select route planner for type-7 missions."""
    global ROUTE_PLANNER_NAME, SWEEP_MERGE_MODE
    planner = str(name or "").strip().lower() or "dtatrim"
    ROUTE_PLANNER_NAME = planner
    if planner in ("linear", "algo2"):
        SWEEP_MERGE_MODE = "all"
    elif planner in ("algo3",):
        SWEEP_MERGE_MODE = "curve"
    else:
        SWEEP_MERGE_MODE = "heading"


def _plan_route_points(
    base: list[tuple[float, float]],
    *,
    cruise_speed: float,
    heading_tol_deg: float,
) -> list[dict]:
    planner = (ROUTE_PLANNER_NAME or "dtatrim").strip().lower()
    if planner in ("linear", "algo2"):
        return route_algos.plan_route_linear(base, cruise_speed=cruise_speed)
    return route_algos.plan_route_dtatrim(
        base,
        cruise_speed=cruise_speed,
        heading_tol_deg=heading_tol_deg,
    )


def _resample_polyline_xy(
    points_xy: list[tuple[float, float]],
    spacing_m: float,
) -> list[tuple[float, float]]:
    if len(points_xy) <= 1:
        return list(points_xy)
    try:
        spacing = max(float(spacing_m), 1.0)
    except Exception:
        spacing = 500.0

    seg_lengths: list[float] = []
    cumulative: list[float] = [0.0]
    total = 0.0
    for idx in range(len(points_xy) - 1):
        x0, y0 = points_xy[idx]
        x1, y1 = points_xy[idx + 1]
        seg_len = math.hypot(float(x1) - float(x0), float(y1) - float(y0))
        seg_lengths.append(seg_len)
        total += seg_len
        cumulative.append(total)

    if total <= spacing * 1.25:
        return [points_xy[0], points_xy[-1]]

    targets: list[float] = [0.0]
    cursor = spacing
    while cursor < total:
        targets.append(float(cursor))
        cursor += spacing
    if total - targets[-1] > 1e-6:
        targets.append(float(total))

    samples: list[tuple[float, float]] = []
    seg_idx = 0
    for target in targets:
        if target >= total:
            samples.append(points_xy[-1])
            continue
        while seg_idx < len(seg_lengths) - 1 and cumulative[seg_idx + 1] < target:
            seg_idx += 1
        start = points_xy[seg_idx]
        end = points_xy[seg_idx + 1]
        seg_start = cumulative[seg_idx]
        seg_len = max(seg_lengths[seg_idx], 1e-9)
        ratio = max(0.0, min(1.0, (target - seg_start) / seg_len))
        x = float(start[0]) + (float(end[0]) - float(start[0])) * ratio
        y = float(start[1]) + (float(end[1]) - float(start[1])) * ratio
        samples.append((x, y))

    deduped: list[tuple[float, float]] = []
    for point in samples:
        if not deduped:
            deduped.append(point)
            continue
        if math.hypot(point[0] - deduped[-1][0], point[1] - deduped[-1][1]) >= 1.0:
            deduped.append(point)
    if deduped[-1] != points_xy[-1]:
        deduped.append(points_xy[-1])
    return deduped


def _sweep_spacing_m(*, separation_m: float, fov_deg: float, spacing_scale: float = 1.0) -> float:
    """Return physical spacing between sweep strips in meters."""
    base = 2.0 * max(separation_m, 1.0) * math.tan(max(math.radians(fov_deg) / 2.0, 1e-6))
    try:
        margin = float(SWEEP_SPACING_MARGIN)
    except Exception:
        margin = 1.0
    if margin <= 0:
        margin = 1.0
    try:
        effective_scale = float(spacing_scale)
    except Exception:
        effective_scale = 1.0
    if effective_scale <= 0.0:
        effective_scale = 1.0
    return max(base * margin * effective_scale, 1.0)


def _debug_sweep(label: str, *, separation: float, fov: float, spacing: float) -> None:
    """Lightweight debug printer for sweep spacing."""
    try:
        key = (
            str(label),
            round(float(separation), 2),
            round(float(fov), 2),
            round(float(spacing), 2),
        )
        with _SWEEP_DEBUG_LOCK:
            if key in _SWEEP_DEBUG_CACHE:
                return
            _SWEEP_DEBUG_CACHE.add(key)
        approx = max(int(round(1000.0 / max(spacing, 1e-6))), 1)
        print(
            f"[SWEEP][{label}] separation={separation:.2f}m, fov={fov:.2f}°, "
            f"spacing={spacing:.2f}m (~{approx} strips/1km)"
        )
    except Exception:
        pass


def _coord_to_xy(coord: dict, ref: dict) -> Tuple[float, float]:
    lat0 = float(ref.get("latitude", 0.0))
    lon0 = float(ref.get("longitude", 0.0))
    lat = float(coord.get("latitude", lat0))
    lon = float(coord.get("longitude", lon0))
    return llh_to_xy(lat, lon, lat0, lon0)


def _xy_to_coord(x: float, y: float, ref: dict, *, altitude: Optional[float] = None) -> Dict[str, float]:
    lat0 = float(ref.get("latitude", 0.0))
    lon0 = float(ref.get("longitude", 0.0))
    lat, lon = xy_to_llh(x, y, lat0, lon0)
    alt_source = altitude if altitude is not None else ref.get("altitude", Altitude)
    alt = _normalize_altitude(alt_source)
    return {
        "latitude": round(lat, 6),
        "longitude": round(lon, 6),
        "altitude": alt,
    }


def _unit_vec(vec: Tuple[float, float]) -> Tuple[float, float]:
    mag = math.hypot(vec[0], vec[1])
    if mag <= 1e-6:
        return (0.0, 0.0)
    return (vec[0] / mag, vec[1] / mag)


def _heading_deg(vec: Tuple[float, float]) -> float:
    return (math.degrees(math.atan2(vec[1], vec[0])) + 360.0) % 360.0


def _wrap_delta(deg: float) -> float:
    return ((deg + 180.0) % 360.0) - 180.0


_TAU = 2.0 * math.pi


def _mod2pi(x: float) -> float:
    return x - _TAU * math.floor(x / _TAU)


def _dubins_LSL(alpha: float, beta: float, d: float):
    tmp0 = d + math.sin(alpha) - math.sin(beta)
    p2 = 2 + d * d - 2 * math.cos(alpha - beta) + 2 * d * (math.sin(alpha) - math.sin(beta))
    if p2 < -1e-12:
        return None
    p = math.sqrt(max(p2, 0.0))
    tmp1 = math.atan2((math.cos(beta) - math.cos(alpha)), tmp0)
    t = _mod2pi(-alpha + tmp1)
    q = _mod2pi(beta - tmp1)
    return t, p, q


def _dubins_RSR(alpha: float, beta: float, d: float):
    tmp0 = d - math.sin(alpha) + math.sin(beta)
    p2 = 2 + d * d - 2 * math.cos(alpha - beta) + 2 * d * (-math.sin(alpha) + math.sin(beta))
    if p2 < -1e-12:
        return None
    p = math.sqrt(max(p2, 0.0))
    tmp1 = math.atan2((math.cos(alpha) - math.cos(beta)), tmp0)
    t = _mod2pi(alpha - tmp1)
    q = _mod2pi(-beta + tmp1)
    return t, p, q


def _dubins_LSR(alpha: float, beta: float, d: float):
    p2 = -2 + d * d + 2 * math.cos(alpha - beta) + 2 * d * (math.sin(alpha) + math.sin(beta))
    if p2 < -1e-12:
        return None
    p = math.sqrt(max(p2, 0.0))
    tmp2 = (
        math.atan2((-math.cos(alpha) - math.cos(beta)), (d + math.sin(alpha) + math.sin(beta)))
        - math.atan2(-2.0, p)
    )
    t = _mod2pi(-alpha + tmp2)
    q = _mod2pi(-beta + tmp2)
    return t, p, q


def _dubins_RSL(alpha: float, beta: float, d: float):
    p2 = -2 + d * d + 2 * math.cos(alpha - beta) - 2 * d * (math.sin(alpha) + math.sin(beta))
    if p2 < -1e-12:
        return None
    p = math.sqrt(max(p2, 0.0))
    tmp2 = (
        math.atan2((math.cos(alpha) + math.cos(beta)), (d - math.sin(alpha) - math.sin(beta)))
        - math.atan2(2.0, p)
    )
    t = _mod2pi(alpha - tmp2)
    q = _mod2pi(beta - tmp2)
    return t, p, q


def _dubins_RLR(alpha: float, beta: float, d: float):
    tmp0 = (6 - d * d + 2 * math.cos(alpha - beta) + 2 * d * (math.sin(alpha) - math.sin(beta))) / 8
    if abs(tmp0) > 1.0:
        return None
    p = _mod2pi(2 * math.pi - math.acos(tmp0))
    t = _mod2pi(
        alpha
        - math.atan2((math.cos(alpha) - math.cos(beta)), (d - math.sin(alpha) + math.sin(beta)))
        + p / 2
    )
    q = _mod2pi(alpha - beta - t + p)
    return t, p, q


def _dubins_LRL(alpha: float, beta: float, d: float):
    tmp0 = (6 - d * d + 2 * math.cos(alpha - beta) + 2 * d * (-math.sin(alpha) + math.sin(beta))) / 8
    if abs(tmp0) > 1.0:
        return None
    p = _mod2pi(2 * math.pi - math.acos(tmp0))
    t = _mod2pi(
        -alpha
        - math.atan2((math.cos(alpha) - math.cos(beta)), (d + math.sin(alpha) - math.sin(beta)))
        + p / 2
    )
    q = _mod2pi(beta - alpha - t + p)
    return t, p, q


@dataclass
class _DubinsPath:
    q0: Tuple[float, float, float]
    rho: float
    theta: float
    alpha: float
    beta: float
    d: float
    word: str
    params: Tuple[float, float, float]
    length: float


def _dubins_shortest_path(
    q0: Tuple[float, float, float],
    q1: Tuple[float, float, float],
    rho: float,
) -> Optional[_DubinsPath]:
    if rho <= 0:
        return None

    x0, y0, th0 = q0
    x1, y1, th1 = q1
    dx = x1 - x0
    dy = y1 - y0
    D = math.hypot(dx, dy)
    if D < 1e-9:
        return None

    d = D / rho
    theta = math.atan2(dy, dx)
    alpha = _mod2pi(th0 - theta)
    beta = _mod2pi(th1 - theta)

    candidates = [
        ("LSL", _dubins_LSL),
        ("LSR", _dubins_LSR),
        ("RSL", _dubins_RSL),
        ("RSR", _dubins_RSR),
        ("RLR", _dubins_RLR),
        ("LRL", _dubins_LRL),
    ]

    best: Optional[_DubinsPath] = None
    for word, fn in candidates:
        res = fn(alpha, beta, d)
        if res is None:
            continue
        t, p, q = res
        length = (t + p + q) * rho
        if best is None or length < best.length:
            best = _DubinsPath(
                q0=q0, rho=rho, theta=theta, alpha=alpha, beta=beta, d=d,
                word=word, params=(t, p, q), length=length,
            )
    return best


def _advance_segment(x: float, y: float, hd: float, seg_type: str, seg_len: float):
    if seg_type == "S":
        x += seg_len * math.cos(hd)
        y += seg_len * math.sin(hd)
    elif seg_type == "L":
        x += math.sin(hd + seg_len) - math.sin(hd)
        y += -math.cos(hd + seg_len) + math.cos(hd)
        hd = _mod2pi(hd + seg_len)
    elif seg_type == "R":
        x += -math.sin(hd - seg_len) + math.sin(hd)
        y += math.cos(hd - seg_len) - math.cos(hd)
        hd = _mod2pi(hd - seg_len)
    else:
        raise RuntimeError(f"unknown segment type: {seg_type}")
    return x, y, hd


def _dubins_point_at(path: _DubinsPath, dist: float) -> Tuple[float, float]:
    if dist <= 0.0:
        x, y = 0.0, 0.0
    else:
        ds = dist / path.rho
        x, y, hd = 0.0, 0.0, path.alpha
        for seg_type, seg_len in zip(path.word, path.params):
            if ds > seg_len:
                x, y, hd = _advance_segment(x, y, hd, seg_type, seg_len)
                ds -= seg_len
            else:
                x, y, hd = _advance_segment(x, y, hd, seg_type, ds)
                ds = 0.0
                break
    cg = math.cos(path.theta)
    sg = math.sin(path.theta)
    X = path.q0[0] + path.rho * (cg * x - sg * y)
    Y = path.q0[1] + path.rho * (sg * x + cg * y)
    return X, Y


def _dubins_three_points_xy(
    q0: Tuple[float, float, float],
    q1: Tuple[float, float, float],
    rho: float,
) -> list[Tuple[float, float]]:
    path = _dubins_shortest_path(q0, q1, rho)
    if path is None:
        mid = ((q0[0] + q1[0]) * 0.5, (q0[1] + q1[1]) * 0.5)
        return [(q0[0], q0[1]), mid, (q1[0], q1[1])]
    length = path.length
    return [
        _dubins_point_at(path, 0.0),
        _dubins_point_at(path, length * 0.5),
        _dubins_point_at(path, length),
    ]


def _dubins_three_points_llh(
    start_coord: dict,
    end_coord: dict,
    heading0_rad: float,
    heading1_rad: float,
    rho: float,
) -> list[Tuple[float, float]]:
    lat0 = float(start_coord.get("latitude", 0.0))
    lon0 = float(start_coord.get("longitude", 0.0))
    lat1 = float(end_coord.get("latitude", lat0))
    lon1 = float(end_coord.get("longitude", lon0))
    dx, dy = llh_to_xy(lat1, lon1, lat0, lon0)
    points_xy = _dubins_three_points_xy((0.0, 0.0, heading0_rad), (dx, dy, heading1_rad), rho)
    return [xy_to_llh(x, y, lat0, lon0) for x, y in points_xy]

SENSOR_NONE     = 0
SENSOR_EO_IR    = 1        # 예) EO/IR 센서

# WaypointPassType
PASS_NONE = 0
PASS_FLYBY = 1
PASS_LOITER = 2
PASS_FLYOVER = 3

# Fly-over options
FLYOVER_ENTRY_OFFSET = False
FLYOVER_DUBINS_PREFIX = False
FLYOVER_ALL_WPS = False

def set_flyover_options(
    *,
    entry_offset: bool = False,
    dubins_prefix: bool = False,
    all_wps: bool = False,
) -> None:
    global FLYOVER_ENTRY_OFFSET, FLYOVER_DUBINS_PREFIX, FLYOVER_ALL_WPS
    FLYOVER_ENTRY_OFFSET = bool(entry_offset)
    FLYOVER_DUBINS_PREFIX = bool(dubins_prefix)
    FLYOVER_ALL_WPS = bool(all_wps)


def _turn_radius_m_for_speed(speed_mps: float) -> float:
    try:
        fixed_radius = float(DUBINS_TURN_RADIUS_M)
    except Exception:
        fixed_radius = 0.0
    if fixed_radius > 0.0:
        return fixed_radius
    table = (
        (30.0, 340.0),
        (40.0, 450.0),
        (50.0, 560.0),
    )
    try:
        spd = float(speed_mps)
    except Exception:
        spd = 40.0
    if spd <= table[0][0]:
        return table[0][1]
    if spd >= table[-1][0]:
        return table[-1][1]
    for (s0, r0), (s1, r1) in zip(table[:-1], table[1:]):
        if s0 <= spd <= s1:
            alpha = (spd - s0) / max(1e-6, (s1 - s0))
            return r0 + (r1 - r0) * alpha
    return table[1][1]


def _dubins_link_available() -> bool:
    return bool(AREA_DUBINS_ENTRY_LINKS_ENABLED and _DubinsPoint2D is not None and _compute_turn_link is not None)


def _coord_with_altitude(
    coord: dict,
    altitude_fn: Callable[[float, float], int],
) -> OrderedDict:
    lat = float(coord.get("latitude", 0.0))
    lon = float(coord.get("longitude", 0.0))
    alt = coord.get("altitude")
    if alt is None:
        alt_i = int(altitude_fn(lat, lon))
    else:
        alt_i = _normalize_altitude(alt)
    return OrderedDict([
        ("latitude", round(lat, 6)),
        ("longitude", round(lon, 6)),
        ("altitude", int(alt_i)),
    ])


def _make_area_link_wp(
    coord: dict,
    speed_mps: float,
    *,
    target_coord: dict | None = None,
    fov: float | None = None,
) -> OrderedDict:
    try:
        link_fov = float(fov) if fov is not None else float(POINT_FOV_DEG)
    except Exception:
        link_fov = float(POINT_FOV_DEG)
    if link_fov <= 0.0:
        link_fov = float(POINT_FOV_DEG)

    filming = _mk_filming(
        operation_mode=OPMODE_HOLD,
        fov=ENTRY_HOLD_FOV_DEG,
        sensor=SENSOR_EO_IR,
        gimbal_pitch=ENTRY_HOLD_GIMBAL_PITCH,
        gimbal_yaw=ENTRY_HOLD_GIMBAL_YAW,
    )
    if isinstance(target_coord, dict):
        try:
            target_lat = float(target_coord.get("latitude", 0.0))
            target_lon = float(target_coord.get("longitude", 0.0))
            filming = OrderedDict([
                ("fieldOfView", link_fov),
                ("sensorType", SENSOR_EO_IR),
                ("operationMode", OPMODE_POINT),
                ("coordinateOrientation", OrderedDict([
                    ("coordinate", OrderedDict([
                        ("latitude", round(target_lat, 6)),
                        ("longitude", round(target_lon, 6)),
                        ("altitude", 0),
                    ]))
                ])),
            ])
        except Exception:
            pass

    return OrderedDict([
        ("waypointID", 0),
        ("_area_dubins_link", True),
        ("_flyover_dubins_prefix", True),
        ("coordinate", coord),
        ("speed", float(speed_mps)),
        ("eta", 0),
        ("ecf", 0.0),
        ("nextWaypointID", 0),
        ("waypointPassType", PASS_FLYBY),
        ("filmingProperty", filming),
    ])


def _build_straight_entry_wp(
    *,
    first_coord: dict,
    second_coord: dict,
    offset_m: float,
    cruise_speed_mps: float,
    altitude_fn: Callable[[float, float], int],
) -> OrderedDict | None:
    if not (isinstance(first_coord, dict) and isinstance(second_coord, dict)):
        return None

    lat0 = float(first_coord.get("latitude", 0.0))
    lon0 = float(first_coord.get("longitude", 0.0))
    lat1 = float(second_coord.get("latitude", lat0))
    lon1 = float(second_coord.get("longitude", lon0))
    vec_x, vec_y = llh_to_xy(lat1, lon1, lat0, lon0)
    norm = math.hypot(vec_x, vec_y)
    if norm < 1.0:
        return None

    try:
        effective_offset_m = max(float(offset_m), 0.0)
    except Exception:
        effective_offset_m = 0.0
    if effective_offset_m < 1.0:
        return None

    ux, uy = vec_x / norm, vec_y / norm
    entry_lat, entry_lon = xy_to_llh(-ux * effective_offset_m, -uy * effective_offset_m, lat0, lon0)
    entry_coord = OrderedDict([
        ("latitude", round(entry_lat, 6)),
        ("longitude", round(entry_lon, 6)),
        ("altitude", int(altitude_fn(entry_lat, entry_lon))),
    ])
    return OrderedDict([
        ("waypointID", 0),
        ("_flyover_entry_offset", True),
        ("coordinate", entry_coord),
        ("speed", float(cruise_speed_mps)),
        ("eta", 0),
        ("ecf", 0.0),
        ("nextWaypointID", 0),
        ("waypointPassType", PASS_FLYBY),
        ("filmingProperty", _mk_filming(
            operation_mode=OPMODE_HOLD,
            fov=ENTRY_HOLD_FOV_DEG,
            sensor=SENSOR_EO_IR,
            gimbal_pitch=ENTRY_HOLD_GIMBAL_PITCH,
            gimbal_yaw=ENTRY_HOLD_GIMBAL_YAW,
        )),
    ])


def _build_area_dubins_entry_wps(
    *,
    prev_start_coord: dict,
    prev_end_coord: dict,
    next_first_coord: dict,
    next_second_coord: dict,
    turn_radius_m: float,
    cruise_speed_mps: float,
    altitude_fn: Callable[[float, float], int],
    target_coord: dict | None = None,
    target_fov: float | None = None,
    min_link_gap_m: float | None = None,
) -> list[OrderedDict]:
    if _compute_turn_link is None or _DubinsPoint2D is None:
        return []
    if not (
        isinstance(prev_start_coord, dict)
        and isinstance(prev_end_coord, dict)
        and isinstance(next_first_coord, dict)
        and isinstance(next_second_coord, dict)
    ):
        return []

    origin_lat = float(prev_end_coord.get("latitude", 0.0))
    origin_lon = float(prev_end_coord.get("longitude", 0.0))
    prev_start_x, prev_start_y = llh_to_xy(
        float(prev_start_coord.get("latitude", 0.0)),
        float(prev_start_coord.get("longitude", 0.0)),
        origin_lat,
        origin_lon,
    )
    next_first_x, next_first_y = llh_to_xy(
        float(next_first_coord.get("latitude", 0.0)),
        float(next_first_coord.get("longitude", 0.0)),
        origin_lat,
        origin_lon,
    )
    next_second_x, next_second_y = llh_to_xy(
        float(next_second_coord.get("latitude", 0.0)),
        float(next_second_coord.get("longitude", 0.0)),
        origin_lat,
        origin_lon,
    )
    if math.hypot(prev_start_x, prev_start_y) < 1.0:
        return []
    if math.hypot(next_second_x - next_first_x, next_second_y - next_first_y) < 1.0:
        return []

    try:
        result = _compute_turn_link(
            prev_start=_DubinsPoint2D(prev_start_x, prev_start_y),
            prev_end=_DubinsPoint2D(0.0, 0.0),
            next_start=_DubinsPoint2D(next_first_x, next_first_y),
            next_end=_DubinsPoint2D(next_second_x, next_second_y),
            radius_m=max(1.0, float(turn_radius_m)),
            sample_step_m=10.0,
            allow_ccc=True,
            path_policy="all_shortest",
        )
    except Exception:
        return []

    try:
        effective_min_gap_m = float(min_link_gap_m) if min_link_gap_m is not None else (float(turn_radius_m) * 0.25)
    except Exception:
        effective_min_gap_m = float(turn_radius_m) * 0.25
    effective_min_gap_m = max(120.0, effective_min_gap_m)

    candidates: list[tuple[dict, float, float]] = []
    for link_point in (result.link_point_1, result.link_point_2):
        lat, lon = xy_to_llh(link_point.x, link_point.y, origin_lat, origin_lon)
        coord = _coord_with_altitude({"latitude": lat, "longitude": lon}, altitude_fn)
        d_prev = _dist_between_coords(prev_end_coord, coord)
        d_next = _dist_between_coords(coord, next_first_coord)
        if d_prev <= 1.0 or d_next <= 1.0:
            continue
        candidates.append((coord, d_prev, d_next))

    prefix: list[OrderedDict] = []
    previous_coord = prev_end_coord
    for coord, d_prev, d_next in candidates:
        if d_prev + 1e-6 < effective_min_gap_m or d_next + 1e-6 < effective_min_gap_m:
            continue
        if _dist_between_coords(previous_coord, coord) <= 1.0:
            continue
        if prefix and _dist_between_coords(prefix[-1].get("coordinate") or {}, coord) <= 1.0:
            continue
        prefix.append(_make_area_link_wp(
            coord,
            cruise_speed_mps,
            target_coord=target_coord,
            fov=target_fov,
        ))
        previous_coord = coord
    return prefix


def _safe_float_or_none(value) -> float | None:
    try:
        result = float(value)
    except Exception:
        return None
    return result if math.isfinite(result) else None


def _dubins_transition_prefix_wps(
    *,
    start_coord: dict,
    end_coord: dict,
    start_heading_rad: float,
    end_heading_rad: float,
    rho_m: float,
    speed_mps: float,
    altitude_fn,
) -> list[OrderedDict]:
    lat0 = _safe_float_or_none(start_coord.get("latitude"))
    lon0 = _safe_float_or_none(start_coord.get("longitude"))
    lat1 = _safe_float_or_none(end_coord.get("latitude"))
    lon1 = _safe_float_or_none(end_coord.get("longitude"))
    if None in (lat0, lon0, lat1, lon1):
        return []
    points = _dubins_three_points_llh(
        {"latitude": lat0, "longitude": lon0},
        {"latitude": lat1, "longitude": lon1},
        float(start_heading_rad),
        float(end_heading_rad),
        max(1.0, float(rho_m)),
    )
    if len(points) < 3:
        return []
    prefix: list[OrderedDict] = []
    for lat, lon in points[1:-1]:
        coord = OrderedDict([
            ("latitude", round(float(lat), 6)),
            ("longitude", round(float(lon), 6)),
            ("altitude", int(altitude_fn(float(lat), float(lon)))),
        ])
        prefix.append(OrderedDict([
            ("waypointID", 0),
            ("_flyover_dubins_prefix", True),
            ("coordinate", coord),
            ("speed", float(speed_mps)),
            ("eta", 0),
            ("ecf", 0.0),
            ("nextWaypointID", 0),
            ("waypointPassType", PASS_FLYBY),
            ("filmingProperty", _mk_filming(
                operation_mode=OPMODE_HOLD,
                fov=ENTRY_HOLD_FOV_DEG,
                sensor=SENSOR_EO_IR,
                gimbal_pitch=ENTRY_HOLD_GIMBAL_PITCH,
                gimbal_yaw=ENTRY_HOLD_GIMBAL_YAW,
            )),
        ]))
    return prefix

# OperationMode
OPMODE_NONE   = 0
OPMODE_POINT  = 1
OPMODE_LINE   = 2
OPMODE_TRACK  = 3
OPMODE_HOLD   = 4
OPMODE_SWEEP  = 5

MISSION_DISPATCH = {
    0: "없음", 
    1: "표적추적", # ID 필요 -> 패턴은 1
    2: "표적공격", # 헬기만 -> ID 필요, 패턴은 2번
    3: "영역수색", # 정해진 횟수만큼 영역에 대한 촬영 계획 세움 -> 패턴은 3,4,5,6,7,8,9
    4: "영역경계", # 정해진 시간동안 영역에 대한 촬영 계획 세움-> 패턴은 3,4,5,6,7,8,9
    5: "좌표점정찰", # 점만 보는 형태 -> 패턴은 1이 될 듯
    6: "통로정찰", # 중심선 기준 좌우 스윕 -> 패턴은 4번
    7: "이동", # 이동 계획 세움 -> 카메라들은 직하방으로 고정
    8: "엄호", # TBD
    9: "은엄폐", # 헬기의 유지 포지션 설정해줌
}

PATTERN_DISPATCH = {
    0:  "없음",
    1:  "표적중심선회·추적", # 0303의 Operation Mode를 3번으로, WaypointPassType가 3 -> 비행 계획은 LoiterProperty 가 생성되어야 함, 선회반경은 1000으로 고정하고 Direction도 0 : None, 1 : CW, 2 : CCW -> Time은 5분 고정 -> Speed는 40고정
    2:  "은엄폐후공격", #유인기에서 할것 TBD
    3:  "직하방-BF촬영", # 직하방 촬영으로 고정하고 비행계획 알고리즘을 CPP 알고리즘으로 대체해서 촬영 중심점 = 비행 경로인 상황임, 한 라인찍고 다음라인을 찍기위해 도는 선회 부분이 중요함
    4:  "이격-BF촬영", # 지금과 동일한 상태
    5:  "구간왕복-BF촬영", #TBD
    6:  "선형반복주사-BF촬영", #TBD
    7:  "직상공순회촬영", #TBD
    8:  "구간중심종단-선형반복주사촬영", #TBD
    9:  "구간중심종단-자동반복주사촬영", #TBD
    10: "목적지이동", #카메라를 직하방으로 고정함 -> 지금과 동일하게 이동 경로 계획함
    11: "대상엄호", #TBD
    12: "은엄폐", #해당 점에서 고도를 최대한(안전고도까지) 낮춰서 숨어있음(유인기만 사용)
}

def _dem_alt(lat: float, lon: float) -> int:
    """DEM 기반 고도를 정수(m)로 반환."""
    return int(round(terrain_elev(lat, lon)))


def _aircraft_alt_offset_m(aircraft_id: int) -> float:
    """Return per-aircraft altitude offset layer (610/620/630m)."""
    try:
        idx = (int(aircraft_id) - 1) % len(ALTITUDE_LAYERS_M)
    except Exception:
        idx = 0
    return float(ALTITUDE_LAYERS_M[idx])


def _collect_ref_points_from_info(info: dict) -> list[tuple[float, float]]:
    points: list[tuple[float, float]] = []

    for c in info.get("coordinateList") or []:
        try:
            points.append((float(c["latitude"]), float(c["longitude"])))
        except Exception:
            continue

    for line in info.get("lineList") or []:
        for c in line.get("coordinateList") or []:
            try:
                points.append((float(c["latitude"]), float(c["longitude"])))
            except Exception:
                continue

    for area in info.get("areaList") or []:
        for c in area.get("coordinateList") or []:
            try:
                points.append((float(c["latitude"]), float(c["longitude"])))
            except Exception:
                continue

    return points


def _median_ground_m(points: list[tuple[float, float]]) -> float | None:
    if not points:
        return None
    samples: list[int] = []
    for lat, lon in points:
        try:
            samples.append(_dem_alt(lat, lon))
        except Exception:
            continue
    if not samples:
        return None
    samples.sort()
    n = len(samples)
    mid = n // 2
    if n % 2:
        return float(samples[mid])
    return (float(samples[mid - 1]) + float(samples[mid])) / 2.0


def _sample_ground_profile_along_coords(
    coords: list[dict],
    *,
    sample_step_m: float = 120.0,
    max_samples_per_leg: int = 24,
) -> list[float]:
    usable: list[tuple[float, float]] = []
    for coord in coords or []:
        if not isinstance(coord, dict):
            continue
        try:
            usable.append((float(coord["latitude"]), float(coord["longitude"])))
        except Exception:
            continue
    if not usable:
        return []

    out: list[float] = [float(_dem_alt(usable[0][0], usable[0][1]))]
    step_m = max(float(sample_step_m), 1.0)
    max_steps = max(int(max_samples_per_leg), 1)
    for (prev_lat, prev_lon), (curr_lat, curr_lon) in zip(usable[:-1], usable[1:]):
        seg_dist = _dist_between_coords(
            {"latitude": prev_lat, "longitude": prev_lon},
            {"latitude": curr_lat, "longitude": curr_lon},
        )
        subdivisions = max(1, int(math.ceil(seg_dist / step_m)))
        subdivisions = min(subdivisions, max_steps)
        for step_idx in range(1, subdivisions + 1):
            ratio = step_idx / subdivisions
            lat = prev_lat + ((curr_lat - prev_lat) * ratio)
            lon = prev_lon + ((curr_lon - prev_lon) * ratio)
            out.append(float(_dem_alt(lat, lon)))
    return out


def _ground_mid_m_from_coords(
    coords: list[dict],
    *,
    fallback_coord: dict | None = None,
    fallback_ground_ref_m: float | None = None,
) -> float | None:
    samples = _sample_ground_profile_along_coords(coords)
    if not samples and isinstance(fallback_coord, dict):
        lat = _safe_float_or_none(fallback_coord.get("latitude"))
        lon = _safe_float_or_none(fallback_coord.get("longitude"))
        if lat is not None and lon is not None:
            samples.append(float(_dem_alt(lat, lon)))
    if not samples:
        return float(fallback_ground_ref_m) if fallback_ground_ref_m is not None else None
    return (min(samples) + max(samples)) / 2.0


def _apply_segment_altitude_to_search_waypoints(
    wps: list[OrderedDict],
    *,
    altitude_offset_m: float,
    fallback_ground_ref_m: float | None = None,
) -> None:
    offset_m = float(altitude_offset_m)
    for wp in wps or []:
        if not isinstance(wp, dict):
            continue
        fp = wp.get("filmingProperty") or {}
        ls = fp.get("lineSearch") or {}
        line_coords = ls.get("coordinateList") or []
        if not line_coords:
            continue
        coord = wp.get("coordinate") or {}
        if not isinstance(coord, dict):
            continue
        lat = _safe_float_or_none(coord.get("latitude"))
        lon = _safe_float_or_none(coord.get("longitude"))
        if lat is None or lon is None:
            continue
        sample_coords = [coord] + [c for c in line_coords if isinstance(c, dict)]
        ground_mid = _ground_mid_m_from_coords(
            sample_coords,
            fallback_coord=coord,
            fallback_ground_ref_m=fallback_ground_ref_m,
        )
        if ground_mid is None:
            continue
        wp["coordinate"] = OrderedDict([
            ("latitude", round(lat, 6)),
            ("longitude", round(lon, 6)),
            ("altitude", int(round(ground_mid + offset_m))),
        ])


def _align_point_anchor_altitude_with_search_waypoints(
    wps: list[OrderedDict],
) -> None:
    if not wps:
        return

    next_line_altitude_by_idx: dict[int, int] = {}
    next_altitude: int | None = None
    for idx in range(len(wps) - 1, -1, -1):
        wp = wps[idx]
        coord = wp.get("coordinate") or {}
        if _has_line_search(wp):
            alt = _safe_float_or_none(coord.get("altitude"))
            if alt is not None:
                next_altitude = int(round(alt))
        if next_altitude is not None:
            next_line_altitude_by_idx[idx] = next_altitude

    for idx, wp in enumerate(wps):
        fp = wp.get("filmingProperty") or {}
        if int(fp.get("operationMode", OPMODE_NONE) or OPMODE_NONE) != OPMODE_POINT:
            continue
        if "coordinateOrientation" not in fp:
            continue
        coord = wp.get("coordinate") or {}
        if not isinstance(coord, dict) or "latitude" not in coord or "longitude" not in coord:
            continue
        target_altitude = next_line_altitude_by_idx.get(idx)
        if target_altitude is None:
            continue
        wp["coordinate"] = OrderedDict([
            ("latitude", round(float(coord.get("latitude", 0.0)), 6)),
            ("longitude", round(float(coord.get("longitude", 0.0)), 6)),
            ("altitude", int(target_altitude)),
        ])


def _sweep_d_values(d_min: float, d_max: float, spacing_m: float) -> list[float]:
    d_values: list[float] = []
    current = d_min
    # ensure we always include the last strip by overshooting slightly
    while current <= d_max + spacing_m * 0.25:
        d_values.append(current)
        current += spacing_m
    if not d_values:
        return [d_min, d_max]
    if len(d_values) == 1 and d_max > d_min:
        d_values.append(d_max)
    else:
        last = d_values[-1]
        if d_max - last > spacing_m * 0.25:
            d_values.append(d_max)
    return d_values


def _band_ranges(d_min: float, d_max: float, max_width_m: float) -> list[tuple[float, float]]:
    span = d_max - d_min
    if max_width_m <= 0.0 or span <= max_width_m:
        return [(d_min, d_max)]
    bands = int(math.ceil(span / max_width_m))
    band_width = span / bands
    ranges: list[tuple[float, float]] = []
    for idx in range(bands):
        start = d_min + band_width * idx
        end = d_min + band_width * (idx + 1)
        ranges.append((start, end))
    return ranges


def _coerce_anchor_llh(anchor_llh: object, fallback_llh: tuple[float, float]) -> tuple[float, float]:
    """Accept either (lat, lon) tuple/list or {'latitude','longitude'} dict."""
    lat_fallback, lon_fallback = fallback_llh
    if isinstance(anchor_llh, dict):
        try:
            lat = float(anchor_llh.get("latitude", lat_fallback))
        except Exception:
            lat = float(lat_fallback)
        try:
            lon = float(anchor_llh.get("longitude", lon_fallback))
        except Exception:
            lon = float(lon_fallback)
        return lat, lon
    if isinstance(anchor_llh, (list, tuple)) and len(anchor_llh) >= 2:
        try:
            lat = float(anchor_llh[0])
        except Exception:
            lat = float(lat_fallback)
        try:
            lon = float(anchor_llh[1])
        except Exception:
            lon = float(lon_fallback)
        return lat, lon
    return float(lat_fallback), float(lon_fallback)


def _dedup_xy_hits(hits: list[tuple[float, float]], eps_m: float = 0.01) -> list[tuple[float, float]]:
    unique: list[tuple[float, float]] = []
    eps2 = float(eps_m) * float(eps_m)
    for x, y in hits:
        duplicated = False
        for ux, uy in unique:
            if (x - ux) * (x - ux) + (y - uy) * (y - uy) <= eps2:
                duplicated = True
                break
        if not duplicated:
            unique.append((x, y))
    return unique


def _pick_sweep_endpoints(
    hits: list[tuple[float, float]],
    tx: float,
    ty: float,
) -> tuple[tuple[float, float], tuple[float, float]] | None:
    unique_hits = _dedup_xy_hits(hits)
    if len(unique_hits) < 2:
        return None
    unique_hits.sort(key=lambda p: tx * p[0] + ty * p[1])
    return unique_hits[0], unique_hits[-1]


def _fallback_poly_long_axis_sweep(poly_llh: list[tuple[float, float]]) -> list[dict]:
    if len(poly_llh) < 2:
        return []
    lat0, lon0 = poly_llh[0]
    pts_xy = [llh_to_xy(lat, lon, lat0, lon0) for lat, lon in poly_llh]
    pts_xy = _dedup_xy_hits(pts_xy, eps_m=0.1)
    if len(pts_xy) < 2:
        return []

    best_pair: tuple[tuple[float, float], tuple[float, float]] | None = None
    best_dist2 = -1.0
    for i in range(len(pts_xy)):
        for j in range(i + 1, len(pts_xy)):
            ax, ay = pts_xy[i]
            bx, by = pts_xy[j]
            dist2 = (bx - ax) * (bx - ax) + (by - ay) * (by - ay)
            if dist2 > best_dist2:
                best_dist2 = dist2
                best_pair = (pts_xy[i], pts_xy[j])
    if best_pair is None:
        return []

    out: list[dict] = []
    for x, y in best_pair:
        lat, lon = xy_to_llh(x, y, lat0, lon0)
        out.append({
            "latitude": lat,
            "longitude": lon,
            "altitude": _dem_alt(lat, lon),
        })
    return out


def _poly_sweeps_banded(
    poly_llh: list[tuple[float, float]],
    anchor_llh: object,
    bearing_deg: float,
    fov_deg: float,
    separation_m: float,
    max_width_m: float,
    spacing_scale: float = 1.0,
) -> tuple[list[list[dict]], list[tuple[float, float]]]:
    lat0, lon0 = poly_llh[0]
    poly_xy = [llh_to_xy(lat, lon, lat0, lon0) for lat, lon in poly_llh]
    anchor_lat, anchor_lon = _coerce_anchor_llh(anchor_llh, (lat0, lon0))
    _ = llh_to_xy(anchor_lat, anchor_lon, lat0, lon0)

    th = math.radians(bearing_deg)
    tx, ty = math.sin(th), math.cos(th)             # bearing 방향
    nx, ny = math.cos(th), -math.sin(th)            # bearing sweep 의 수직 노멀
    proj_norm = [nx * x + ny * y for x, y in poly_xy]
    proj_along = [tx * x + ty * y for x, y in poly_xy]
    d_min, d_max = min(proj_norm), max(proj_norm)
    a_min, a_max = min(proj_along), max(proj_along)

    spacing_m = _sweep_spacing_m(separation_m=separation_m, fov_deg=fov_deg, spacing_scale=spacing_scale)
    edges = [(poly_xy[i], poly_xy[(i + 1) % len(poly_xy)]) for i in range(len(poly_xy))]
    band_ranges_raw = _band_ranges(a_min, a_max, max_width_m)
    band_ranges: list[tuple[float, float]] = []
    band_coord_lists: list[list[dict]] = []

    d_values = _sweep_d_values(d_min, d_max, spacing_m)
    for band_min, band_max in band_ranges_raw:
        coord_list: list[dict] = []
        for d in d_values:
            hits = []
            for a, b in edges:
                f1, f2 = nx * a[0] + ny * a[1] - d, nx * b[0] + ny * b[1] - d
                if f1 * f2 <= 0 and f1 != f2:
                    t = f1 / (f1 - f2)
                    x = a[0] + t * (b[0] - a[0])
                    y = a[1] + t * (b[1] - a[1])
                    hits.append((x, y))
            picked = _pick_sweep_endpoints(hits, tx, ty)
            if picked is not None:
                p1, p2 = picked
                a1 = tx * p1[0] + ty * p1[1]
                a2 = tx * p2[0] + ty * p2[1]
                seg_min, seg_max = (a1, a2) if a1 <= a2 else (a2, a1)
                if seg_max < band_min or seg_min > band_max:
                    continue
                denom = a2 - a1
                if abs(denom) < 1e-6:
                    if band_min <= a1 <= band_max:
                        for x, y in (p1, p2):
                            lat, lon = xy_to_llh(x, y, lat0, lon0)
                            coord_list.append({
                                "latitude": lat,
                                "longitude": lon,
                                "altitude": _dem_alt(lat, lon),
                            })
                    continue
                clip_min = max(seg_min, band_min)
                clip_max = min(seg_max, band_max)
                t0 = (clip_min - a1) / denom
                t1 = (clip_max - a1) / denom
                t_low, t_high = (t0, t1) if t0 <= t1 else (t1, t0)
                t_low = max(0.0, min(1.0, t_low))
                t_high = max(0.0, min(1.0, t_high))
                if t_high <= t_low + 1e-9:
                    continue
                c1 = (p1[0] + (p2[0] - p1[0]) * t_low, p1[1] + (p2[1] - p1[1]) * t_low)
                c2 = (p1[0] + (p2[0] - p1[0]) * t_high, p1[1] + (p2[1] - p1[1]) * t_high)
                for x, y in (c1, c2):
                    lat, lon = xy_to_llh(x, y, lat0, lon0)
                    coord_list.append({
                        "latitude": lat,
                        "longitude": lon,
                        "altitude": _dem_alt(lat, lon),
                    })
        if coord_list:
            band_coord_lists.append(coord_list)
            band_ranges.append((band_min, band_max))
    return band_coord_lists, band_ranges

def _poly_sweeps_general(
    poly_llh: list[tuple[float,float]],
    anchor_llh: object,
    bearing_deg: float,
    fov_deg: float,
    separation_m: float,
    spacing_scale: float = 1.0,
) -> list[dict]:
    """
    ▸ Convex polygon LLH → bearing 과 평행한 띠 스윕 → lineSearch.coordinateList
    ▸ 리턴: [{"latitude":…, "longitude":…, "altitude":5}, …]  (짝수/홀수=한 라인)
    """
    lat0, lon0 = poly_llh[0]
    poly_xy = [llh_to_xy(lat, lon, lat0, lon0) for lat, lon in poly_llh]
    anchor_lat, anchor_lon = _coerce_anchor_llh(anchor_llh, (lat0, lon0))
    anchor_xy = llh_to_xy(anchor_lat, anchor_lon, lat0, lon0)

    th = math.radians(bearing_deg)
    tx, ty = math.sin(th), math.cos(th)
    nx, ny =  math.cos(th), -math.sin(th)            # bearing sweep 의 수직 노멀
    proj = [nx*x + ny*y for x, y in poly_xy]
    d_min, d_max = min(proj), max(proj)

    spacing_m = _sweep_spacing_m(separation_m=separation_m, fov_deg=fov_deg, spacing_scale=spacing_scale)
    d_values = _sweep_d_values(d_min, d_max, spacing_m)

    # 다각형 edge 리스트
    edges = [(poly_xy[i], poly_xy[(i+1)%len(poly_xy)]) for i in range(len(poly_xy))]
    coord_list: list[dict] = []

    for d in d_values:
        # 직선: n·x = d  ↔  (−ny, nx) 방향 unit 벡터
        hits = []
        for a,b in edges:
            f1, f2 = nx*a[0]+ny*a[1]-d, nx*b[0]+ny*b[1]-d
            if f1*f2 <= 0 and f1 != f2:
                t = f1/(f1-f2)
                x = a[0] + t*(b[0]-a[0]);   y = a[1] + t*(b[1]-a[1])
                hits.append((x,y))
        picked = _pick_sweep_endpoints(hits, tx, ty)
        if picked is not None:
            # 스윕 선은 교차점 두 개를 연결
            p1, p2 = picked
            for x,y in (p1,p2):
                lat, lon = xy_to_llh(x,y,lat0,lon0)
                coord_list.append({
                    "latitude": lat,
                    "longitude": lon,
                    "altitude": _dem_alt(lat, lon),
                })
    return coord_list

def _mk_filming(operation_mode: int = OPMODE_NONE,
                fov: float = FOV_DEG,
                sensor: int = SENSOR_NONE,
                line_search: OrderedDict | None = None,
                gimbal_pitch: float | None = None,
                gimbal_yaw: float | None = None) -> OrderedDict:
    """
    filmingProperty 블록 생성 헬퍼

    • OPMODE_LINE  → lineSearch 필드 삽입  
    • OPMODE_HOLD → aircraftFixed  블록 삽입
                  ↳ gimbalPitch / gimbalYaw 포함
    """
    fp = OrderedDict([
        ("fieldOfView",   fov),
        ("sensorType",    sensor),
        ("operationMode", operation_mode),
    ])

    # ① 선형 탐색(lineSearch)
    if operation_mode == OPMODE_LINE and line_search is not None:
        fp["lineSearch"] = line_search

    # ② 고정 촬영(aircraftFixed + 짐벌 각도)
    if operation_mode == OPMODE_HOLD:
        # 기본값:  직하방 -90°, Yaw 0°
        gimbal_pitch = -90.0 if gimbal_pitch is None else gimbal_pitch
        gimbal_yaw   =   0.0 if gimbal_yaw   is None else gimbal_yaw
        fp["aircraftFixed"] = OrderedDict([
            ("gimbalPitch", gimbal_pitch),
            ("gimbalYaw",   gimbal_yaw),
        ])
    return fp


def _scale_output_fov_inplace(wps: list[OrderedDict], *, scale: float) -> None:
    try:
        effective_scale = float(scale)
    except Exception:
        return
    if effective_scale <= 0.0 or abs(effective_scale - 1.0) <= 1e-9:
        return
    for wp in wps:
        fp = wp.get("filmingProperty")
        if not isinstance(fp, dict):
            continue
        try:
            base_fov = float(fp.get("fieldOfView", FOV_DEG) or FOV_DEG)
        except Exception:
            continue
        fp["fieldOfView"] = round(base_fov * effective_scale, 6)


def _has_line_search(wp: dict) -> bool:
    fp = wp.get("filmingProperty") or {}
    if int(fp.get("operationMode", OPMODE_NONE)) != OPMODE_LINE:
        return False
    line_search = fp.get("lineSearch") or {}
    coords = line_search.get("coordinateList") or []
    return bool(coords)


def _angle_between(v1: tuple[float, float], v2: tuple[float, float]) -> float:
    x1, y1 = v1
    x2, y2 = v2
    n1 = math.hypot(x1, y1)
    n2 = math.hypot(x2, y2)
    if n1 <= 1e-6 or n2 <= 1e-6:
        return 0.0
    cos_th = max(-1.0, min(1.0, (x1 * x2 + y1 * y2) / (n1 * n2)))
    return math.degrees(math.acos(cos_th))


def _vector_between_coords(coord_from: dict, coord_to: dict) -> tuple[float, float]:
    lat_a = float(coord_from.get("latitude", 0.0))
    lon_a = float(coord_from.get("longitude", 0.0))
    lat_b = float(coord_to.get("latitude", lat_a))
    lon_b = float(coord_to.get("longitude", lon_a))
    return llh_to_xy(lat_b, lon_b, lat_a, lon_a)


def _heading_rad_between_coords(coord_from: dict, coord_to: dict) -> float:
    dx, dy = _vector_between_coords(coord_from, coord_to)
    return math.atan2(dy, dx)


def _dist_between_coords(coord_from: dict, coord_to: dict) -> float:
    dx, dy = _vector_between_coords(coord_from, coord_to)
    return math.hypot(dx, dy)


def _coord_midpoint(coord_list: list[dict]) -> dict | None:
    if not coord_list:
        return None
    if len(coord_list) == 1:
        return deepcopy(coord_list[0])
    first = coord_list[0]
    last = coord_list[-1]
    return {
        "latitude": (float(first.get("latitude", 0.0)) + float(last.get("latitude", 0.0))) * 0.5,
        "longitude": (float(first.get("longitude", 0.0)) + float(last.get("longitude", 0.0))) * 0.5,
        "altitude": int(round((float(first.get("altitude", 0.0)) + float(last.get("altitude", 0.0))) * 0.5)),
    }


def _area_wp_coord_from_sweep(
    coords: list[dict],
    *,
    bearing_deg: float | None,
    offset_m: float,
    altitude_fn: Callable[[float, float], int],
    reference_coord: dict | None = None,
    offset_sign: int | None = None,
) -> OrderedDict | None:
    center = _coord_midpoint(coords)
    if not isinstance(center, dict):
        return None

    center_lat = float(center.get("latitude", 0.0))
    center_lon = float(center.get("longitude", 0.0))
    try:
        effective_offset_m = max(float(offset_m), 0.0)
    except Exception:
        effective_offset_m = 0.0

    shifted_lat = center_lat
    shifted_lon = center_lon
    try:
        if bearing_deg is not None:
            th = math.radians(float(bearing_deg))
            neg_lat, neg_lon = xy_to_llh(
                -math.cos(th) * effective_offset_m,
                math.sin(th) * effective_offset_m,
                center_lat,
                center_lon,
            )
            pos_lat, pos_lon = xy_to_llh(
                math.cos(th) * effective_offset_m,
                -math.sin(th) * effective_offset_m,
                center_lat,
                center_lon,
            )
            forced_sign = 0
            if offset_sign is not None:
                try:
                    forced_sign = 1 if int(offset_sign) >= 0 else -1
                except Exception:
                    forced_sign = 0
            if forced_sign > 0:
                shifted_lat, shifted_lon = pos_lat, pos_lon
            elif forced_sign < 0:
                shifted_lat, shifted_lon = neg_lat, neg_lon
            if (
                forced_sign == 0
                and
                isinstance(reference_coord, dict)
                and "latitude" in reference_coord
                and "longitude" in reference_coord
            ):
                neg_coord = {"latitude": neg_lat, "longitude": neg_lon}
                pos_coord = {"latitude": pos_lat, "longitude": pos_lon}
                if _dist_between_coords(reference_coord, pos_coord) + 1e-6 < _dist_between_coords(reference_coord, neg_coord):
                    shifted_lat, shifted_lon = pos_lat, pos_lon
                else:
                    shifted_lat, shifted_lon = neg_lat, neg_lon
            elif forced_sign == 0:
                shifted_lat, shifted_lon = neg_lat, neg_lon
    except Exception:
        shifted_lat = center_lat
        shifted_lon = center_lon

    return OrderedDict([
        ("latitude", round(float(shifted_lat), 6)),
        ("longitude", round(float(shifted_lon), 6)),
        ("altitude", altitude_fn(float(shifted_lat), float(shifted_lon))),
    ])


def _infer_area_offset_sign_from_sweep(
    coords: list[dict],
    *,
    bearing_deg: float | None,
    offset_m: float,
    reference_coord: dict | None = None,
) -> int:
    anchor = _area_wp_coord_from_sweep(
        coords,
        bearing_deg=bearing_deg,
        offset_m=offset_m,
        altitude_fn=lambda lat, lon: 0,
        reference_coord=reference_coord,
    )
    if not anchor or bearing_deg is None or len(coords) < 2:
        return -1
    center = _coord_midpoint(coords)
    if not center:
        return -1
    th = math.radians(float(bearing_deg))
    nx, ny = math.cos(th), -math.sin(th)
    anchor_x, anchor_y = llh_to_xy(
        float(anchor.get("latitude", 0.0)),
        float(anchor.get("longitude", 0.0)),
        float(center.get("latitude", 0.0)),
        float(center.get("longitude", 0.0)),
    )
    signed = (anchor_x * nx) + (anchor_y * ny)
    return 1 if signed >= 0.0 else -1


def _wp_effective_start_coord(wp: dict) -> dict:
    fp = wp.get("filmingProperty") or {}
    ls = fp.get("lineSearch") or {}
    coords = ls.get("coordinateList") or []
    if coords:
        return dict(coords[0])
    orient = fp.get("coordinateOrientation") or {}
    orient_coord = orient.get("coordinate")
    if isinstance(orient_coord, dict) and "latitude" in orient_coord and "longitude" in orient_coord:
        return dict(orient_coord)
    return dict(wp.get("coordinate") or {})


def _wp_effective_end_coord(wp: dict) -> dict:
    fp = wp.get("filmingProperty") or {}
    ls = fp.get("lineSearch") or {}
    coords = ls.get("coordinateList") or []
    if coords:
        return dict(coords[-1])
    return dict(wp.get("coordinate") or {})


def _orient_sweep_items(
    sweep_items: list[dict],
    reference_coord: dict | None,
) -> list[dict]:
    if len(sweep_items) <= 1:
        return sweep_items

    items = []
    for item in sweep_items:
        copied = dict(item)
        copied["coords"] = deepcopy(item.get("coords") or [])
        items.append(copied)

    if reference_coord and "latitude" in reference_coord and "longitude" in reference_coord:
        def _best_item_start_dist(item: dict) -> float:
            coords = item.get("coords") or []
            if len(coords) >= 2:
                return min(
                    _dist_between_coords(reference_coord, coords[0]),
                    _dist_between_coords(reference_coord, coords[-1]),
                )
            anchor = item.get("coord") or _coord_midpoint(coords) or {}
            return _dist_between_coords(reference_coord, anchor)

        if _best_item_start_dist(items[-1]) + 1e-6 < _best_item_start_dist(items[0]):
            items.reverse()

        prev_target = reference_coord
        for item in items:
            coords = item.get("coords") or []
            if len(coords) >= 2:
                first_coord = coords[0]
                last_coord = coords[-1]
                if _dist_between_coords(prev_target, last_coord) + 1e-6 < _dist_between_coords(prev_target, first_coord):
                    coords.reverse()
                prev_target = coords[-1]
                item["coords"] = coords

    return items


def _apply_oriented_sweep_items_to_wps(
    wps: list[OrderedDict],
    sweep_items: list[dict],
) -> None:
    if not wps or not sweep_items:
        return

    ordered_wps = [item.get("wp") for item in sweep_items if isinstance(item.get("wp"), dict)]
    if not ordered_wps:
        return

    ordered_ids = {id(wp) for wp in ordered_wps}
    remainder = [wp for wp in wps if id(wp) not in ordered_ids]
    wps[:] = ordered_wps + remainder

    index_by_id = {id(wp): idx for idx, wp in enumerate(wps)}
    for item in sweep_items:
        wp = item.get("wp")
        if not isinstance(wp, dict):
            continue
        item["idx"] = index_by_id.get(id(wp), item.get("idx", 0))
        item["coord"] = wp.get("coordinate") or item.get("coord") or {}
        item["fp"] = wp.get("filmingProperty") or item.get("fp") or OrderedDict()


def _reorder_line_search_subset_inplace(
    wps: list[OrderedDict],
    *,
    reference_coord: dict | None,
) -> None:
    if not wps:
        return

    line_positions = [idx for idx, wp in enumerate(wps) if _has_line_search(wp)]
    if len(line_positions) <= 1:
        return

    first_line_pos = line_positions[0]
    prefix = wps[:first_line_pos]
    line_wps = [wps[idx] for idx in line_positions]
    suffix_positions = set(line_positions)
    suffix = [wp for idx, wp in enumerate(wps[first_line_pos:]) if (idx + first_line_pos) not in suffix_positions]

    sweep_items: list[dict] = []
    for idx, wp in enumerate(line_wps):
        fp = wp.get("filmingProperty") or OrderedDict()
        ls = fp.get("lineSearch") or {}
        coords = deepcopy(ls.get("coordinateList") or [])
        sweep_items.append({
            "idx": idx,
            "wp": wp,
            "fp": fp,
            "coord": wp.get("coordinate") or {},
            "coords": coords,
            "search_speed": ls.get("searchSpeed"),
            "fov": fp.get("fieldOfView", FOV_DEG),
            "interp_points": _interp_points_hint(ls.get("interpolationPoints")),
            "sweep_idx": wp.get("_sweepIdx"),
        })

    sweep_items = _orient_sweep_items(sweep_items, reference_coord)
    ordered_line_wps: list[OrderedDict] = []
    for item in sweep_items:
        wp = item.get("wp")
        if not isinstance(wp, dict):
            continue
        fp = wp.get("filmingProperty") or OrderedDict()
        ls = fp.get("lineSearch") or OrderedDict()
        coords = deepcopy(item.get("coords") or [])
        if coords:
            ls["coordinateList"] = coords
            fp["lineSearch"] = ls
            wp["filmingProperty"] = fp
        ordered_line_wps.append(wp)

    wps[:] = prefix + ordered_line_wps + suffix


def _split_record_groups_by_spacing(
    records: list[dict],
    groups: list[list[int]],
    *,
    anchor_coord: dict | None,
    spacing_m: float,
    merge_short_tail: bool = False,
) -> list[list[int]]:
    try:
        max_spacing_m = max(float(spacing_m), 1.0)
    except Exception:
        max_spacing_m = float(SWEEP_ROUTE_WP_SPACING_M)
    if not groups or max_spacing_m <= 0.0:
        return groups

    out: list[list[int]] = []
    group_anchor = anchor_coord if isinstance(anchor_coord, dict) else None
    for group in groups:
        if len(group) <= 1:
            out.append(group)
            if group:
                anchor_candidate = records[group[-1]].get("coord") or records[group[-1]]["wp"].get("coordinate") or {}
                if isinstance(anchor_candidate, dict) and anchor_candidate:
                    group_anchor = anchor_candidate
            continue

        subgroup: list[int] = []
        progressed = 0.0
        prev_coord = group_anchor
        for pos in group:
            curr_coord = records[pos].get("coord") or records[pos]["wp"].get("coordinate") or {}
            step = _dist_between_coords(prev_coord, curr_coord) if prev_coord else 0.0
            projected = progressed + step
            if subgroup and projected > max_spacing_m and progressed >= 1.0:
                out.append(subgroup)
                subgroup = [pos]
                progressed = step
            else:
                subgroup.append(pos)
                progressed = projected
            prev_coord = curr_coord
        if subgroup:
            out.append(subgroup)
            anchor_candidate = records[subgroup[-1]].get("coord") or records[subgroup[-1]]["wp"].get("coordinate") or {}
            if isinstance(anchor_candidate, dict) and anchor_candidate:
                group_anchor = anchor_candidate
    if merge_short_tail:
        while len(out) >= 2:
            prev_anchor = records[out[-2][-1]].get("coord") or records[out[-2][-1]]["wp"].get("coordinate") or {}
            tail_anchor = records[out[-1][-1]].get("coord") or records[out[-1][-1]]["wp"].get("coordinate") or {}
            if not (isinstance(prev_anchor, dict) and prev_anchor and isinstance(tail_anchor, dict) and tail_anchor):
                break
            tail_gap_m = _dist_between_coords(prev_anchor, tail_anchor)
            if tail_gap_m + 1e-6 >= max_spacing_m:
                break
            out[-1] = out[-2] + out[-1]
            out.pop(-2)
    return out


def _line_route_wp_spacing_m() -> float:
    try:
        base_spacing = float(SWEEP_ROUTE_WP_SPACING_M)
    except Exception:
        base_spacing = 2000.0
    try:
        density = float(LINE_SWEEP_DENSITY_SCALE)
    except Exception:
        density = 1.0
    if density <= 0.0:
        density = 1.0
    return max(base_spacing / density, 1.0)


def _area_raw_spacing_scale() -> float:
    try:
        fov_scale = float(AREA_OUTPUT_FOV_SCALE)
    except Exception:
        fov_scale = 1.0
    if fov_scale <= 0.0:
        fov_scale = 1.0
    try:
        density = float(AREA_SWEEP_DENSITY_SCALE)
    except Exception:
        density = 1.0
    if density <= 0.0:
        density = 1.0
    return max(fov_scale / density, 1e-6)


def _is_required_wp(wp: dict) -> bool:
    if wp.get("_area_dubins_link"):
        return True
    if wp.get("waypointPassType") == PASS_LOITER:
        return True
    fp = wp.get("filmingProperty") or {}
    if not fp:
        return False
    return int(fp.get("operationMode", OPMODE_NONE)) != OPMODE_NONE


def _simplify_area_wps(
    wps: list[OrderedDict],
    *,
    angle_deg: float,
    min_spacing_m: float,
) -> list[OrderedDict]:
    if len(wps) <= 2:
        return wps
    keep = [0]
    last_keep = 0
    for idx in range(1, len(wps) - 1):
        if _is_required_wp(wps[idx]):
            keep.append(idx)
            last_keep = idx
            continue
        prev_coord = wps[last_keep].get("coordinate") or {}
        curr_coord = wps[idx].get("coordinate") or {}
        next_coord = wps[idx + 1].get("coordinate") or {}
        dist = _dist_between_coords(prev_coord, curr_coord)
        angle = _angle_between(
            _vector_between_coords(prev_coord, curr_coord),
            _vector_between_coords(curr_coord, next_coord),
        )
        if dist >= min_spacing_m or angle >= angle_deg:
            keep.append(idx)
            last_keep = idx
    keep.append(len(wps) - 1)
    keep = sorted(set(keep))
    return [wps[i] for i in keep]


def _reposition_area_wps_by_sep(
    wps: list[OrderedDict],
    *,
    offset_m: float,
    altitude_fn: Callable[[float, float], int],
    bearing_deg: float | None = None,
    reference_coord: dict | None = None,
    offset_sign: int | None = None,
) -> None:
    if not wps:
        return

    def _align_route_wps_to_endpoint_axis() -> None:
        route_positions = [idx for idx, wp in enumerate(wps) if _has_line_search(wp)]
        if len(route_positions) < 3:
            return

        first_coord = wps[route_positions[0]].get("coordinate") or {}
        last_coord = wps[route_positions[-1]].get("coordinate") or {}
        route_dx, route_dy = _vector_between_coords(first_coord, last_coord)
        route_norm = math.hypot(route_dx, route_dy)
        if route_norm < 1.0:
            return

        ux = route_dx / route_norm
        uy = route_dy / route_norm
        origin_lat = float(first_coord.get("latitude", 0.0))
        origin_lon = float(first_coord.get("longitude", 0.0))
        previous_progress = 0.0

        for pos in route_positions[1:-1]:
            curr_coord = wps[pos].get("coordinate") or {}
            curr_lat = float(curr_coord.get("latitude", origin_lat))
            curr_lon = float(curr_coord.get("longitude", origin_lon))
            curr_x, curr_y = llh_to_xy(curr_lat, curr_lon, origin_lat, origin_lon)
            progress_m = max(previous_progress, (curr_x * ux) + (curr_y * uy))
            progress_m = min(progress_m, route_norm)
            projected_lat, projected_lon = xy_to_llh(
                progress_m * ux,
                progress_m * uy,
                origin_lat,
                origin_lon,
            )
            altitude = curr_coord.get("altitude")
            if altitude is None:
                altitude = altitude_fn(projected_lat, projected_lon)
            wps[pos]["coordinate"] = OrderedDict([
                ("latitude", round(projected_lat, 6)),
                ("longitude", round(projected_lon, 6)),
                ("altitude", int(round(float(altitude)))),
            ])
            previous_progress = progress_m

    if bearing_deg is not None:
        try:
            effective_offset_m = max(float(offset_m), 0.0)
        except Exception:
            effective_offset_m = 0.0
        for wp in wps:
            fp = wp.get("filmingProperty") or {}
            ls = fp.get("lineSearch") or {}
            coords = ls.get("coordinateList") or []
            shifted = _area_wp_coord_from_sweep(
                coords,
                bearing_deg=bearing_deg,
                offset_m=effective_offset_m,
                altitude_fn=altitude_fn,
                reference_coord=reference_coord,
                offset_sign=offset_sign,
            )
            if shifted is not None:
                wp["coordinate"] = shifted
        _align_route_wps_to_endpoint_axis()
        return

    if len(wps) < 2:
        return

    centers: list[dict | None] = []
    for wp in wps:
        fp = wp.get("filmingProperty") or {}
        ls = fp.get("lineSearch") or {}
        coords = ls.get("coordinateList") or []
        centers.append(_coord_midpoint(coords) if coords else dict(wp.get("coordinate") or {}))

    first_center = centers[0] if isinstance(centers[0], dict) else None
    if not first_center:
        return

    sample_centers = [center for center in centers[: min(len(centers), 5)] if isinstance(center, dict)]
    if len(sample_centers) < 2:
        return

    dir_x = 0.0
    dir_y = 0.0
    for idx in range(len(sample_centers) - 1):
        vec_x, vec_y = _vector_between_coords(sample_centers[idx], sample_centers[idx + 1])
        seg_norm = math.hypot(vec_x, vec_y)
        if seg_norm < 1.0:
            continue
        dir_x += vec_x / seg_norm
        dir_y += vec_y / seg_norm

    norm = math.hypot(dir_x, dir_y)
    if norm < 1.0:
        vec_x, vec_y = _vector_between_coords(sample_centers[0], sample_centers[-1])
        norm = math.hypot(vec_x, vec_y)
        if norm < 1.0:
            return
        dir_x, dir_y = vec_x, vec_y

    ux = dir_x / norm
    uy = dir_y / norm
    origin_lat = float(first_center.get("latitude", 0.0))
    origin_lon = float(first_center.get("longitude", 0.0))
    effective_offset_m = max(float(offset_m), 0.0)
    sign = -1.0
    if offset_sign is not None:
        try:
            sign = 1.0 if int(offset_sign) >= 0 else -1.0
        except Exception:
            sign = -1.0
    offset_x = sign * ux * effective_offset_m
    offset_y = sign * uy * effective_offset_m

    for wp, center in zip(wps, centers):
        if not isinstance(center, dict):
            continue
        base_lat = float(center.get("latitude", 0.0))
        base_lon = float(center.get("longitude", 0.0))
        center_x, center_y = llh_to_xy(base_lat, base_lon, origin_lat, origin_lon)
        progress_m = (center_x * ux) + (center_y * uy)
        shifted_lat, shifted_lon = xy_to_llh(
            offset_x + (progress_m * ux),
            offset_y + (progress_m * uy),
            origin_lat,
            origin_lon,
        )
        wp["coordinate"] = OrderedDict([
            ("latitude", round(shifted_lat, 6)),
            ("longitude", round(shifted_lon, 6)),
            ("altitude", altitude_fn(shifted_lat, shifted_lon)),
        ])
    _align_route_wps_to_endpoint_axis()


def _collapse_area_wps_to_two_points(
    wps: list[OrderedDict],
    *,
    full_coords: list[dict] | None,
    search_speed: float | None,
    interp_points: int | None,
    altitude_fn: Callable[[float, float], int],
    bearing_deg: float | None = None,
    offset_m: float = 0.0,
    reference_coord: dict | None = None,
    offset_sign: int | None = None,
) -> list[OrderedDict]:
    if not wps:
        return wps

    try:
        area_route_spacing_m = max(float(AREA_SWEEP_ROUTE_WP_SPACING_M), 1.0)
    except Exception:
        area_route_spacing_m = 2000.0

    line_positions = [idx for idx, wp in enumerate(wps) if _has_line_search(wp)]
    if not line_positions:
        return wps

    line_wps = [wps[idx] for idx in line_positions]
    first_line_wp = line_wps[0]
    first_coord = dict(wps[0].get("coordinate") or {})

    first_fp = first_line_wp.get("filmingProperty") or {}
    first_ls = first_fp.get("lineSearch") or {}
    first_ls_coords = deepcopy(first_ls.get("coordinateList") or [])
    if not first_ls_coords:
        first_ls_coords = deepcopy(full_coords or [])
    if not first_ls_coords:
        return wps

    sweep_start = dict(first_ls_coords[0])
    orientation_coord = OrderedDict([
        ("latitude", float(sweep_start.get("latitude", 0.0))),
        ("longitude", float(sweep_start.get("longitude", 0.0))),
        ("altitude", int(round(float(sweep_start.get("altitude", 0.0) or 0.0)))),
    ])

    merged_coords = deepcopy(full_coords or [])
    if not merged_coords:
        for wp in line_wps:
            coords = deepcopy(((wp.get("filmingProperty") or {}).get("lineSearch") or {}).get("coordinateList") or [])
            if coords:
                merged_coords.extend(coords)
    if not merged_coords:
        return wps

    first_waypoint = OrderedDict(deepcopy(wps[0]))
    first_anchor_coord = _area_wp_coord_from_sweep(
        first_ls_coords,
        bearing_deg=bearing_deg,
        offset_m=offset_m,
        altitude_fn=altitude_fn,
        reference_coord=reference_coord,
        offset_sign=offset_sign,
    )
    if first_anchor_coord is None:
        first_anchor_coord = OrderedDict([
            ("latitude", round(float(first_coord.get("latitude", 0.0)), 6)),
            ("longitude", round(float(first_coord.get("longitude", 0.0)), 6)),
            ("altitude", altitude_fn(float(first_coord.get("latitude", 0.0)), float(first_coord.get("longitude", 0.0)))),
        ])
    first_waypoint["coordinate"] = first_anchor_coord
    first_waypoint["waypointPassType"] = PASS_FLYBY
    first_waypoint["filmingProperty"] = OrderedDict([
        ("fieldOfView", float(first_fp.get("fieldOfView", FOV_DEG) or FOV_DEG)),
        ("sensorType", int(first_fp.get("sensorType", SENSOR_EO_IR) or SENSOR_EO_IR)),
        ("operationMode", OPMODE_POINT),
        ("coordinateOrientation", OrderedDict([
            ("coordinate", orientation_coord),
        ])),
    ])

    groups: list[list[int]] = []
    current_group: list[int] = [line_positions[0]]
    progressed = 0.0
    prev_coord = wps[line_positions[0]].get("coordinate") or {}
    for pos in line_positions[1:]:
        curr_coord = wps[pos].get("coordinate") or {}
        step = _dist_between_coords(prev_coord, curr_coord)
        projected = progressed + step
        if current_group and projected > area_route_spacing_m and progressed >= 1.0:
            groups.append(current_group)
            current_group = [pos]
            progressed = step
        else:
            current_group.append(pos)
            progressed = projected
        prev_coord = curr_coord
    if current_group:
        groups.append(current_group)
    if len(groups) >= 2 and len(groups[0]) == 1 and groups[0][0] == line_positions[0]:
        groups[1] = groups[0] + groups[1]
        groups = groups[1:]
    while len(groups) >= 2:
        prev_anchor = wps[groups[-2][-1]].get("coordinate") or {}
        tail_anchor = wps[groups[-1][-1]].get("coordinate") or {}
        if not (isinstance(prev_anchor, dict) and prev_anchor and isinstance(tail_anchor, dict) and tail_anchor):
            break
        tail_gap_m = _dist_between_coords(prev_anchor, tail_anchor)
        if tail_gap_m + 1e-6 >= area_route_spacing_m:
            break
        groups[-1] = groups[-2] + groups[-1]
        groups.pop(-2)

    result: list[OrderedDict] = [first_waypoint]
    for group_idx, group in enumerate(groups):
        rep_idx = group[-1]
        rep_src = OrderedDict(deepcopy(wps[rep_idx]))
        rep_coord = dict(rep_src.get("coordinate") or {})
        rep_fp = rep_src.get("filmingProperty") or {}
        chunk_coords: list[dict] = []
        chunk_speeds: list[float] = []
        chunk_fov = rep_fp.get("fieldOfView", first_fp.get("fieldOfView", FOV_DEG))
        chunk_sensor = rep_fp.get("sensorType", first_fp.get("sensorType", SENSOR_EO_IR))
        points_hint = interp_points
        for pos in group:
            wp = wps[pos]
            fp = wp.get("filmingProperty") or {}
            ls = fp.get("lineSearch") or {}
            coords = deepcopy(ls.get("coordinateList") or [])
            if coords:
                chunk_coords.extend(coords)
            speed = ls.get("searchSpeed")
            if speed is not None:
                try:
                    chunk_speeds.append(float(speed))
                except Exception:
                    pass
            if points_hint is None:
                points_hint = ls.get("interpolationPoints")
            if "fieldOfView" in fp:
                chunk_fov = fp.get("fieldOfView", chunk_fov)
            if "sensorType" in fp:
                chunk_sensor = fp.get("sensorType", chunk_sensor)
        if not chunk_coords:
            continue
        line_speed = round(sum(chunk_speeds) / max(len(chunk_speeds), 1), 2) if chunk_speeds else None
        if line_speed is None:
            line_speed = search_speed
        if line_speed is None:
            line_speed = rep_src.get("speed", 0.0)
        if points_hint is None:
            points_hint = SWEEP_LINE_INTERP_POINTS
        anchor_source_coords = chunk_coords
        if group_idx == 0 and first_ls_coords:
            # Keep the first line waypoint tied to the first actual sweep line.
            anchor_source_coords = first_ls_coords
        rep_anchor_coord = _area_wp_coord_from_sweep(
            anchor_source_coords,
            bearing_deg=bearing_deg,
            offset_m=offset_m,
            altitude_fn=altitude_fn,
            reference_coord=reference_coord,
            offset_sign=offset_sign,
        )
        if rep_anchor_coord is None:
            rep_anchor_coord = OrderedDict([
                ("latitude", round(float(rep_coord.get("latitude", 0.0)), 6)),
                ("longitude", round(float(rep_coord.get("longitude", 0.0)), 6)),
                ("altitude", altitude_fn(float(rep_coord.get("latitude", 0.0)), float(rep_coord.get("longitude", 0.0)))),
            ])
        rep_src["coordinate"] = rep_anchor_coord
        rep_src["waypointPassType"] = PASS_FLYOVER if group_idx == len(groups) - 1 else PASS_FLYBY
        rep_src["filmingProperty"] = _mk_filming(
            operation_mode=OPMODE_LINE,
            fov=float(chunk_fov or FOV_DEG),
            sensor=int(chunk_sensor or SENSOR_EO_IR),
            line_search=OrderedDict([
                ("coordinateList", chunk_coords),
                ("searchSpeed", float(line_speed)),
                ("interpolationPoints", int(points_hint)),
            ]),
        )
        result.append(rep_src)

    if len(result) == 1:
        last_fp = line_wps[-1].get("filmingProperty") or {}
        last_ls = last_fp.get("lineSearch") or {}
        line_speed = search_speed
        if line_speed is None:
            line_speed = last_ls.get("searchSpeed")
        if line_speed is None:
            line_speed = first_ls.get("searchSpeed")
        if line_speed is None:
            line_speed = wps[-1].get("speed", 0.0)
        points_hint = interp_points
        if points_hint is None:
            points_hint = last_ls.get("interpolationPoints")
        if points_hint is None:
            points_hint = first_ls.get("interpolationPoints")
        if points_hint is None:
            points_hint = SWEEP_LINE_INTERP_POINTS
        fallback_waypoint = OrderedDict(deepcopy(wps[-1]))
        fallback_anchor_coord = _area_wp_coord_from_sweep(
            merged_coords,
            bearing_deg=bearing_deg,
            offset_m=offset_m,
            altitude_fn=altitude_fn,
            reference_coord=reference_coord,
            offset_sign=offset_sign,
        )
        if fallback_anchor_coord is not None:
            fallback_waypoint["coordinate"] = fallback_anchor_coord
        fallback_waypoint["waypointPassType"] = PASS_FLYOVER
        fallback_waypoint["filmingProperty"] = _mk_filming(
            operation_mode=OPMODE_LINE,
            fov=float(last_fp.get("fieldOfView", first_fp.get("fieldOfView", FOV_DEG)) or FOV_DEG),
            sensor=int(last_fp.get("sensorType", first_fp.get("sensorType", SENSOR_EO_IR)) or SENSOR_EO_IR),
            line_search=OrderedDict([
                ("coordinateList", merged_coords),
                ("searchSpeed", float(line_speed)),
                ("interpolationPoints", int(points_hint)),
            ]),
        )
        result.append(fallback_waypoint)

    route_positions = [idx for idx, wp in enumerate(result) if _has_line_search(wp)]
    if len(route_positions) >= 2:
        first_route_pos = route_positions[0]
        start_anchor = result[first_route_pos].get("coordinate") or {}
        end_anchor = result[route_positions[-1]].get("coordinate") or {}
        route_dx, route_dy = _vector_between_coords(start_anchor, end_anchor)
        route_norm = math.hypot(route_dx, route_dy)
        if route_norm >= 1.0:
            ux = route_dx / route_norm
            uy = route_dy / route_norm
            origin_lat = float(start_anchor.get("latitude", 0.0))
            origin_lon = float(start_anchor.get("longitude", 0.0))
            previous_progress = 0.0
            for pos in route_positions[1:]:
                curr_anchor = result[pos].get("coordinate") or {}
                curr_x, curr_y = llh_to_xy(
                    float(curr_anchor.get("latitude", origin_lat)),
                    float(curr_anchor.get("longitude", origin_lon)),
                    origin_lat,
                    origin_lon,
                )
                progress_m = max(previous_progress, (curr_x * ux) + (curr_y * uy))
                projected_lat, projected_lon = xy_to_llh(
                    progress_m * ux,
                    progress_m * uy,
                    origin_lat,
                    origin_lon,
                )
                result[pos]["coordinate"] = OrderedDict([
                    ("latitude", round(projected_lat, 6)),
                    ("longitude", round(projected_lon, 6)),
                    ("altitude", altitude_fn(projected_lat, projected_lon)),
                ])
                previous_progress = progress_m

    if result and route_positions:
        first_route_pos = route_positions[0]
        first_route_anchor = result[first_route_pos].get("coordinate") or {}
        forward_coord: dict | None = None
        if len(route_positions) >= 2:
            candidate = result[route_positions[1]].get("coordinate") or {}
            if isinstance(candidate, dict) and candidate:
                forward_coord = candidate
        if forward_coord is None:
            candidate = (
                ((result[0].get("filmingProperty") or {}).get("coordinateOrientation") or {}).get("coordinate")
                or {}
            )
            if isinstance(candidate, dict) and candidate:
                forward_coord = candidate
        if forward_coord is not None:
            vec_x, vec_y = _vector_between_coords(first_route_anchor, forward_coord)
            vec_norm = math.hypot(vec_x, vec_y)
            if vec_norm >= 1.0:
                lead_in_m = max(float(AREA_POINT_ANCHOR_LEAD_IN_M), 0.0)
                origin_lat = float(first_route_anchor.get("latitude", 0.0))
                origin_lon = float(first_route_anchor.get("longitude", 0.0))
                anchor_lat, anchor_lon = xy_to_llh(
                    -(vec_x / vec_norm) * lead_in_m,
                    -(vec_y / vec_norm) * lead_in_m,
                    origin_lat,
                    origin_lon,
                )
                result[0]["coordinate"] = OrderedDict([
                    ("latitude", round(anchor_lat, 6)),
                    ("longitude", round(anchor_lon, 6)),
                    ("altitude", altitude_fn(anchor_lat, anchor_lon)),
                ])

    return result


def _pack_area_wps_by_spacing(
    wps: list[OrderedDict],
    *,
    route_spacing_m: float,
    spacing_line_m: float,
    cruise_speed_mps: float,
    search_speed_weight: float,
    default_search_speed: float,
    post_entry_search_speed_scale: float = 1.0,
    post_entry_route_spacing_scale: float = 1.0,
) -> list[OrderedDict]:
    if not wps:
        return wps

    try:
        target_spacing_m = max(float(route_spacing_m), 1.0)
    except Exception:
        target_spacing_m = float(AREA_SWEEP_ROUTE_WP_SPACING_M)
    try:
        route_spacing_scale = float(post_entry_route_spacing_scale)
    except Exception:
        route_spacing_scale = 1.0
    if route_spacing_scale <= 0.0:
        route_spacing_scale = 1.0
    target_spacing_m *= route_spacing_scale

    try:
        line_search_speed_scale = float(post_entry_search_speed_scale)
    except Exception:
        line_search_speed_scale = 1.0
    if line_search_speed_scale <= 0.0:
        line_search_speed_scale = 1.0

    line_positions = [idx for idx, wp in enumerate(wps) if _has_line_search(wp)]
    if not line_positions:
        return wps

    first_wp = OrderedDict(deepcopy(wps[line_positions[0]]))
    first_fp = first_wp.get("filmingProperty") or {}
    first_ls = first_fp.get("lineSearch") or {}
    first_coords = deepcopy(first_ls.get("coordinateList") or [])
    if not first_coords:
        return wps

    first_coord = wps[line_positions[0]].get("coordinate") or {}
    last_coord = wps[line_positions[-1]].get("coordinate") or {}
    progress_by_pos: dict[int, float] = {}

    route_dx, route_dy = _vector_between_coords(first_coord, last_coord)
    route_norm = math.hypot(route_dx, route_dy)
    if route_norm >= 1.0:
        ux = route_dx / route_norm
        uy = route_dy / route_norm
        origin_lat = float(first_coord.get("latitude", 0.0))
        origin_lon = float(first_coord.get("longitude", 0.0))
        previous_progress = 0.0
        for pos in line_positions:
            curr_coord = wps[pos].get("coordinate") or {}
            curr_lat = float(curr_coord.get("latitude", origin_lat))
            curr_lon = float(curr_coord.get("longitude", origin_lon))
            curr_x, curr_y = llh_to_xy(curr_lat, curr_lon, origin_lat, origin_lon)
            progress_m = max(previous_progress, (curr_x * ux) + (curr_y * uy))
            progress_m = min(progress_m, route_norm)
            progress_by_pos[pos] = progress_m
            previous_progress = progress_m
    else:
        progressed = 0.0
        prev_coord = first_coord
        for pos in line_positions:
            curr_coord = wps[pos].get("coordinate") or {}
            if pos != line_positions[0]:
                progressed += _dist_between_coords(prev_coord, curr_coord)
            progress_by_pos[pos] = progressed
            prev_coord = curr_coord

    sweep_start = dict(first_coords[0])
    first_point = OrderedDict(deepcopy(first_wp))
    first_point["waypointPassType"] = PASS_FLYBY
    first_point["filmingProperty"] = OrderedDict([
        ("fieldOfView", float(first_fp.get("fieldOfView", FOV_DEG) or FOV_DEG)),
        ("sensorType", int(first_fp.get("sensorType", SENSOR_EO_IR) or SENSOR_EO_IR)),
        ("operationMode", OPMODE_POINT),
        ("coordinateOrientation", OrderedDict([
            ("coordinate", OrderedDict([
                ("latitude", float(sweep_start.get("latitude", 0.0))),
                ("longitude", float(sweep_start.get("longitude", 0.0))),
                ("altitude", int(round(float(sweep_start.get("altitude", 0.0) or 0.0)))),
            ])),
        ])),
    ])

    groups: list[list[int]] = []
    current_group: list[int] = [line_positions[0]]
    anchor_progress = float(progress_by_pos.get(line_positions[0], 0.0))
    for pos in line_positions[1:]:
        candidate_progress = float(progress_by_pos.get(pos, anchor_progress))
        last_progress = float(progress_by_pos.get(current_group[-1], anchor_progress))
        if (
            current_group
            and (candidate_progress - anchor_progress) > target_spacing_m
            and (last_progress - anchor_progress) >= 1.0
        ):
            groups.append(current_group)
            anchor_progress = last_progress
            current_group = [pos]
        else:
            current_group.append(pos)
    if current_group:
        groups.append(current_group)

    while len(groups) >= 2:
        prev_progress = float(progress_by_pos.get(groups[-2][-1], 0.0))
        tail_progress = float(progress_by_pos.get(groups[-1][-1], prev_progress))
        tail_gap_m = tail_progress - prev_progress
        if tail_gap_m + 1e-6 >= target_spacing_m:
            break
        groups[-1] = groups[-2] + groups[-1]
        groups.pop(-2)

    result: list[OrderedDict] = [first_point]
    for group_idx, group in enumerate(groups):
        rep_idx = group[-1]
        rep_src = OrderedDict(deepcopy(wps[rep_idx]))
        rep_fp = rep_src.get("filmingProperty") or {}
        chunk_coords: list[dict] = []
        chunk_speeds: list[float] = []
        chunk_fov = rep_fp.get("fieldOfView", first_fp.get("fieldOfView", FOV_DEG))
        chunk_sensor = rep_fp.get("sensorType", first_fp.get("sensorType", SENSOR_EO_IR))
        points_hint = None

        for pos in group:
            wp = wps[pos]
            fp = wp.get("filmingProperty") or {}
            ls = fp.get("lineSearch") or {}
            coords = deepcopy(ls.get("coordinateList") or [])
            if coords:
                chunk_coords.extend(coords)
            speed = ls.get("searchSpeed")
            if speed is not None:
                try:
                    chunk_speeds.append(float(speed))
                except Exception:
                    pass
            if points_hint is None:
                points_hint = _interp_points_hint(ls.get("interpolationPoints"))
            if "fieldOfView" in fp:
                chunk_fov = fp.get("fieldOfView", chunk_fov)
            if "sensorType" in fp:
                chunk_sensor = fp.get("sensorType", chunk_sensor)

        if not chunk_coords:
            continue

        sweep_width_m = _avg_sweep_width_m(chunk_coords, points_per_line=points_hint)
        line_speed = spacing_based_search_speed(
            sweep_len_m=sweep_width_m,
            spacing_m=spacing_line_m,
            cruise_speed_mps=cruise_speed_mps,
            weight=float(search_speed_weight) * line_search_speed_scale,
        )
        if line_speed is None and chunk_speeds:
            line_speed = round(sum(chunk_speeds) / len(chunk_speeds), 2)
        if line_speed is None:
            line_speed = default_search_speed
        if points_hint is None:
            points_hint = SWEEP_LINE_INTERP_POINTS

        rep_src["waypointPassType"] = PASS_FLYOVER if group_idx == len(groups) - 1 else PASS_FLYBY
        rep_src["filmingProperty"] = _mk_filming(
            operation_mode=OPMODE_LINE,
            fov=float(chunk_fov or FOV_DEG),
            sensor=int(chunk_sensor or SENSOR_EO_IR),
            line_search=OrderedDict([
                ("coordinateList", chunk_coords),
                ("searchSpeed", float(line_speed)),
                ("interpolationPoints", int(points_hint)),
            ]),
        )
        result.append(rep_src)

    return result


def _interp_points_hint(value: object) -> int | None:
    """Convert a stored interpolation hint into an int ≥ 2."""
    try:
        points = int(value)
    except (TypeError, ValueError):
        return None
    return points if points >= 2 else None


def _to_straight_coords_if_area(
    coords: list[dict],
    *,
    force_straight: bool,
) -> list[dict]:
    if not force_straight or len(coords) < 2:
        return coords
    return [deepcopy(coords[0]), deepcopy(coords[-1])]


def _collect_sweep_spans(coords: list[dict], points_per_line: int | None) -> list[float]:
    """Return individual sweep spans extracted from the coordinate list."""
    if not coords or len(coords) < 2:
        return []
    chunk = max(points_per_line or 2, 2)
    spans: list[float] = []
    idx = 0
    while idx < len(coords) - 1:
        start = coords[idx]
        end_idx = min(idx + chunk - 1, len(coords) - 1)
        end = coords[end_idx]
        lat0 = float(start.get("latitude", 0.0))
        lon0 = float(start.get("longitude", 0.0))
        lat1 = float(end.get("latitude", lat0))
        lon1 = float(end.get("longitude", lon0))
        dx, dy = llh_to_xy(lat1, lon1, lat0, lon0)
        dist = math.hypot(dx, dy)
        if dist > 0.0:
            spans.append(dist)
        idx = end_idx + 1

    return spans


def _avg_sweep_width_m(coords: list[dict], *, points_per_line: int | None = None) -> float | None:
    """
    Estimate the average span of each sweep line in meters.

    When coordinates include interpolated mid-points, `points_per_line` tells the helper
    how many samples belong to a single sweep (>=2). The default pairs entries (0-1, 2-3, ...).
    """
    spans = _collect_sweep_spans(coords, points_per_line)
    if not spans:
        return None
    return round(sum(spans) / len(spans), 2)


def _subdivide_segment(start: dict, end: dict, points: int) -> list[dict]:
    """Generate evenly spaced coordinates along start-end inclusive."""
    if points <= 2:
        return [deepcopy(start), deepcopy(end)]
    lat0 = float(start.get("latitude", 0.0))
    lon0 = float(start.get("longitude", 0.0))
    alt0 = float(start.get("altitude", Altitude))
    lat1 = float(end.get("latitude", lat0))
    lon1 = float(end.get("longitude", lon0))
    alt1 = float(end.get("altitude", alt0))
    dx, dy = llh_to_xy(lat1, lon1, lat0, lon0)
    coords: list[dict] = []
    for idx in range(points):
        if idx == 0:
            coords.append(deepcopy(start))
            continue
        if idx == points - 1:
            coords.append(deepcopy(end))
            continue
        t = idx / (points - 1)
        xi = dx * t
        yi = dy * t
        lat_i, lon_i = xy_to_llh(xi, yi, lat0, lon0)
        alt_i = alt0 + (alt1 - alt0) * t
        coords.append(OrderedDict([
            ("latitude", round(lat_i, 6)),
            ("longitude", round(lon_i, 6)),
            ("altitude", int(round(alt_i))),
        ]))
    return coords


def _interpolate_line_coords(coords: list[dict], points: int) -> list[dict]:
    """Split sweep line coordinate pairs into interpolated segments."""
    if points <= 2 or not coords:
        return coords
    result: list[dict] = []
    for idx in range(0, len(coords), 2):
        start = coords[idx]
        end = coords[idx + 1] if idx + 1 < len(coords) else start
        subdivided = _subdivide_segment(start, end, points)
        if result and subdivided and result[-1] == subdivided[0]:
            subdivided = subdivided[1:]
        result.extend(deepcopy(coord) for coord in subdivided)
    return result


class _WPAllocator:
    def __init__(self, start: int | None = None) -> None:
        self._local_next = start
        self._use_global = start is None

    def alloc(self) -> int:
        if self._use_global:
            return int(_next_waypoint_id())
        if self._local_next is None:
            raise RuntimeError("Waypoint allocator misconfigured (local start unset)")
        if self._local_next <= 0:
            raise RuntimeError("WaypointID pool exhausted")
        wid = self._local_next
        self._local_next += 1
        return wid

def _index_refpoints(ref0203: dict | None):
    """
    0203에서 기체ID→좌표 맵 구성.
    return: (to_map, ho_map)  각 값은 {aid: {"latitude":..,"longitude":..,"altitude":..}}
    """
    if not ref0203:
        return {}, {}
    to_map = {}
    for it in ref0203.get("takeOverInfoList", []) or []:
        aid = it.get("aircraftID")
        c   = it.get("coordinate") or {}
        if isinstance(aid, int) and 1 <= aid <= 6 and "latitude" in c and "longitude" in c:
            to_map[aid] = {
                "latitude": float(c.get("latitude", 0.0)),
                "longitude": float(c.get("longitude", 0.0)),
                "altitude": _normalize_altitude(c.get("altitude", Altitude)),
            }
    ho_map = {}
    for it in ref0203.get("handOverInfoList", []) or []:
        aid = it.get("aircraftID")
        c   = it.get("coordinate") or {}
        if isinstance(aid, int) and 1 <= aid <= 6 and "latitude" in c and "longitude" in c:
            ho_map[aid] = {
                "latitude": float(c.get("latitude", 0.0)),
                "longitude": float(c.get("longitude", 0.0)),
                "altitude": _normalize_altitude(c.get("altitude", Altitude)),
            }
    return to_map, ho_map


def _eta_ms_llh(c1: dict, c2: dict, speed_mps: float) -> int:
    """
    간단한 구면 근사로 거리→ETA(s) 계산. speed_mps<=0이면 0.
    c* = {"latitude": float, "longitude": float}
    """
    try:
        lat1, lon1 = float(c1["latitude"]), float(c1["longitude"])
        lat2, lon2 = float(c2["latitude"]), float(c2["longitude"])
    except Exception:
        return 0
    DEG_M = 111_132.0
    dx = (lon2 - lon1) * DEG_M * math.cos(math.radians((lat1 + lat2) / 2.0))
    dy = (lat2 - lat1) * DEG_M
    dist_m = math.hypot(dx, dy)
    if speed_mps and speed_mps > 0.0:
        return int(round(dist_m / speed_mps))
    return 0


def _annotate_eta_ms_inplace(waypoints: list[OrderedDict], default_speed_mps: float) -> None:
    # NOTE: historical function name kept for compatibility; ETA is emitted in seconds.
    if not waypoints:
        return

    # d0303 builds waypoint objects before final waypointID / nextWaypointID allocation.
    # At that stage many WPs still carry waypointID=0, nextWaypointID=0 placeholders.
    # If we try to restore order from the next-chain too early, only a tail subset can be
    # observed and pre-filled placeholder ETA values (e.g. 2500) leak into the final file.
    # Monitoring then interprets them as mission planned time, which is why some missions
    # show absurd values such as 5000s.
    waypoint_ids = [int(wp.get("waypointID", 0) or 0) for wp in waypoints if isinstance(wp, dict)]
    next_ids = [int(wp.get("nextWaypointID", 0) or 0) for wp in waypoints if isinstance(wp, dict)]
    has_resolved_chain = (
        len(waypoint_ids) == len(waypoints)
        and len(set(waypoint_ids)) == len(waypoints)
        and all(wid > 0 for wid in waypoint_ids)
        and any(nid > 0 for nid in next_ids)
    )
    ordered = _order_by_next_chain(waypoints) if has_resolved_chain else list(waypoints)
    if not ordered:
        return

    # ETA is stored as cumulative seconds within this flight path.
    for wp in waypoints:
        wp["eta"] = 0
    ordered[0]["eta"] = 0
    acc_s = 0.0
    for i in range(1, len(ordered)):
        dt_s = _time_from_prev_to_curr_s(ordered[i - 1], ordered[i], default_speed_mps=default_speed_mps)
        acc_s += dt_s
        cum_s = int(round(acc_s))
        ordered[i]["eta"] = max(0, cum_s)

def _calc_search_speed_from_paths(
    cur_coord: dict,
    next_coord: dict,
    line_coords: list[dict],
    cruise_speed: float,
    weight: float | None = None,
) -> float | None:
    if _SearchSpeedCalculator is None:
        return None
    if not cur_coord or not next_coord:
        return None
    lat0 = cur_coord.get("latitude")
    lon0 = cur_coord.get("longitude")
    if lat0 is None or lon0 is None:
        if line_coords:
            lat0 = line_coords[0].get("latitude")
            lon0 = line_coords[0].get("longitude")
    if lat0 is None or lon0 is None:
        return None

    def _to_xyz(coord: dict) -> tuple[float, float, float] | None:
        if not coord or "latitude" not in coord or "longitude" not in coord:
            return None
        x, y = llh_to_xy(coord["latitude"], coord["longitude"], lat0, lon0)
        alt = float(coord.get("altitude") or 0.0)
        return (x, y, alt)

    u0 = _to_xyz(cur_coord)
    u1 = _to_xyz(next_coord)
    if u0 is None or u1 is None:
        return None

    sweep_pts: list[tuple[float, float, float]] = []
    for c in line_coords:
        pt = _to_xyz(c)
        if pt is not None:
            sweep_pts.append(pt)
    if len(sweep_pts) < 2:
        return None

    try:
        calc = _SearchSpeedCalculator([u0, u1], sweep_pts, float(cruise_speed))
        speed = float(calc.compute_search_speed())
    except Exception:
        return None
    try:
        effective_weight = float(weight) if weight is not None else 1.0
    except Exception:
        effective_weight = 1.0
    if effective_weight <= 0.0:
        effective_weight = 1.0
    return round(max(0.0, speed * effective_weight), 2)


def _apply_line_search_speed_from_paths(wps: list[OrderedDict], cruise_speed: float) -> None:
    if _SearchSpeedCalculator is None or len(wps) < 2:
        return
    for idx in range(1, len(wps)):
        wp = wps[idx]
        if wp.get("_skip_path_search_speed_recalc"):
            continue
        prev_wp = wps[idx - 1]
        fp = wp.get("filmingProperty") or OrderedDict()
        line_search = fp.get("lineSearch")
        if not line_search:
            continue
        coords = line_search.get("coordinateList") or []
        if not coords:
            continue
        speed = _calc_search_speed_from_paths(
            prev_wp.get("coordinate") or {},
            wp.get("coordinate") or {},
            coords,
            cruise_speed,
            weight=_safe_float_or_none(wp.get("_path_search_speed_weight")),
        )
        if speed is None:
            continue
        line_search["searchSpeed"] = speed


def _recompute_eta_ecf_inplace(waypoints: list[OrderedDict], default_speed_mps: float) -> None:
    if not waypoints:
        return
    for wp in waypoints:
        wp.setdefault("isDone", False)
    _apply_line_search_speed_from_paths(waypoints, default_speed_mps)
    _annotate_eta_ms_inplace(waypoints, default_speed_mps=default_speed_mps)
    total_eta = sum(max(0, int(wp.get("eta", 0))) for wp in waypoints) or 1
    cum = 0
    for wp in waypoints:
        step_eta = max(0, int(wp.get("eta", 0)))
        cum += step_eta
        wp["ecf"] = round(min(cum / total_eta, 1.0), 2)
    waypoints[-1]["ecf"] = 1.0


def _drop_redundant_close_area_entry_anchor_inplace(
    prev_tail_coord: dict | None,
    wps: list[OrderedDict],
    *,
    min_gap_m: float,
) -> bool:
    if not isinstance(prev_tail_coord, dict) or len(wps) < 2:
        return False

    first_wp = wps[0]
    second_wp = wps[1]
    first_fp = first_wp.get("filmingProperty") or {}
    if int(first_fp.get("operationMode", OPMODE_NONE) or OPMODE_NONE) != OPMODE_POINT:
        return False
    if "coordinateOrientation" not in first_fp:
        return False

    first_coord = first_wp.get("coordinate") or {}
    if not (isinstance(first_coord, dict) and "latitude" in first_coord and "longitude" in first_coord):
        return False

    if _dist_between_coords(prev_tail_coord, first_coord) + 1e-6 >= max(float(min_gap_m), 1.0):
        return False

    first_target = _wp_effective_start_coord(first_wp)
    second_target = _wp_effective_start_coord(second_wp)
    if not first_target or not second_target:
        return False
    if _dist_between_coords(first_target, second_target) > 20.0:
        return False

    wps.pop(0)
    return True


def _append_area_transition_links_inplace(
    prev_pkt: dict,
    next_pkt: dict,
    *,
    altitude_fn: Callable[[float, float], int],
) -> int:
    if not _dubins_link_available():
        return 0

    prev_wps = prev_pkt.get("wplist") or []
    next_wps = next_pkt.get("wplist") or []
    if len(prev_wps) < 2 or len(next_wps) < 2:
        return 0

    prev_start_coord = prev_wps[-2].get("coordinate") or {}
    prev_end_coord = prev_wps[-1].get("coordinate") or {}
    next_first_coord = next_wps[0].get("coordinate") or {}
    next_second_coord = next_wps[1].get("coordinate") or {}
    if not all(
        isinstance(coord, dict) and "latitude" in coord and "longitude" in coord
        for coord in (prev_start_coord, prev_end_coord, next_first_coord, next_second_coord)
    ):
        return 0

    target_coord = _wp_effective_start_coord(next_wps[0])
    next_fp = next_wps[0].get("filmingProperty") or {}
    try:
        target_fov = float(next_fp.get("fieldOfView", POINT_FOV_DEG) or POINT_FOV_DEG)
    except Exception:
        target_fov = float(POINT_FOV_DEG)

    try:
        cruise_speed_mps = float(
            prev_pkt.get(
                "_mission_cruise_speed_mps",
                next_pkt.get("_mission_cruise_speed_mps", 40.0),
            ) or 40.0
        )
    except Exception:
        cruise_speed_mps = 40.0

    turn_radius_m = _turn_radius_m_for_speed(cruise_speed_mps)
    link_wps = _build_area_dubins_entry_wps(
        prev_start_coord=prev_start_coord,
        prev_end_coord=prev_end_coord,
        next_first_coord=next_first_coord,
        next_second_coord=next_second_coord,
        turn_radius_m=turn_radius_m,
        cruise_speed_mps=cruise_speed_mps,
        altitude_fn=altitude_fn,
        target_coord=target_coord,
        target_fov=target_fov,
        min_link_gap_m=max(80.0, turn_radius_m * 0.2),
    )
    if not link_wps:
        return 0

    for link_wp in link_wps:
        link_wp.setdefault("isDone", False)
    prev_wps.extend(link_wps)
    _recompute_eta_ecf_inplace(prev_wps, default_speed_mps=cruise_speed_mps)
    return len(link_wps)


def build_flight_plans(
    missions: list[dict],
    wp_alloc: _WPAllocator | None = None,
    cruise_speed: float = 30.0,
    turn_step_deg: float = 45.0,
    ref0203: dict | None = None,
) -> list[dict]:
    with _SWEEP_DEBUG_LOCK:
        _SWEEP_DEBUG_CACHE.clear()
    wp_alloc = wp_alloc or _WPAllocator()
    now_ms = now_ms_since_2000()
    take_over_map, _ = _index_refpoints(ref0203)
    is_alg2 = (ROUTE_PLANNER_NAME or "").strip().lower() in ("linear", "algo2")

    # ── 상수 ───────────────────────────────────────────────
    SENSOR, OPMODE = 1, 2
    DEFAULT_SEARCH_SPEED = round(cruise_speed * DEFAULT_SEARCH_SPEED_MULTIPLIER, 2)
    geom = _active_sweep_geometry()
    ALT_M = geom.separation_m
    geom_fov_deg = geom.fov_deg
    sweep_spacing_m = _sweep_spacing_m(separation_m=ALT_M, fov_deg=geom_fov_deg)
    def _wp_alt(
        lat: float,
        lon: float,
        fallback: float | int | None = None,
        ground_ref_m: float | None = None,
    ) -> int:
        base = fallback if fallback is not None else Altitude
        try:
            base_alt = float(base)
        except Exception:
            base_alt = float(Altitude)
        if ground_ref_m is None:
            ground = float(_dem_alt(lat, lon))
        else:
            try:
                ground = float(ground_ref_m)
            except Exception:
                ground = float(_dem_alt(lat, lon))
        return int(round(ground + base_alt))
    DEG_M = 111_132
    # ── 마지막점용 POINT 촬영 블록 생성기 ─────────────────
    def _mk_point_filming_for_coord(coord: dict) -> OrderedDict:
        lat = float(coord.get("latitude", 0.0))
        lon = float(coord.get("longitude", 0.0))
        return OrderedDict([
            ("fieldOfView", POINT_FOV_DEG),
            ("sensorType", SENSOR_EO_IR),
            ("operationMode", OPMODE_POINT),
            ("coordinateOrientation", OrderedDict([
                ("coordinate", OrderedDict([
                    ("latitude",  lat),
                    ("longitude", lon),
                    ("altitude",  0),
                ]))
            ])),
        ])

    packets: list[dict] = []

    formation_offsets = [(-100, -100, 0), (100, -100, 0)]

    def _formation_key(miss: dict) -> int | None:
        rel = miss.get("relatedMission") if isinstance(miss.get("relatedMission"), dict) else {}
        raw = rel.get("inputMissionID") or miss.get("inputMissionID") or miss.get("pathID")
        try:
            return int(raw)
        except Exception:
            return None

    def _formation_role(info: dict) -> Optional[str]:
        if not isinstance(info, dict):
            return None
        if info.get("individualMissionType") != 7:
            return None
        line_list = info.get("lineList") or []
        if not line_list:
            return None
        try:
            width_val = float(line_list[0].get("width", 0))
        except Exception:
            return None
        width_tag = int(round(width_val))
        if width_tag == 0:
            return "leader"
        if width_tag == 1:
            return "follower"
        return None

    def _is_formation_line(info: dict) -> bool:
        if not isinstance(info, dict):
            return False
        line_list = info.get("lineList") or []
        if not line_list:
            return False
        try:
            width_val = float(line_list[0].get("width", 0))
        except Exception:
            return False
        return width_val <= 1.0

    formation_groups: dict[int, dict[str, list[int]]] = {}
    for miss in missions:
        aid = miss.get("aircraftID")
        if aid not in (4, 5, 6):
            continue
        info = miss.get("individualMissionInfo") if isinstance(miss.get("individualMissionInfo"), dict) else {}
        role = _formation_role(info)
        if role is None and _is_formation_line(info):
            role = "follower"
        if role is None:
            continue
        key = _formation_key(miss)
        if key is None:
            continue
        bucket = formation_groups.setdefault(key, {"leader": [], "follower": []})
        bucket[role].append(int(aid))

    formation_group_sorted: dict[int, dict[str, list[int]]] = {}
    formation_leaders: dict[int, int] = {}
    for key, bucket in formation_groups.items():
        leaders = sorted(set(bucket.get("leader") or []))
        followers = sorted(set(bucket.get("follower") or []))
        formation_group_sorted[key] = {"leader": leaders, "follower": followers}
        preferred = None
        for cid in (4, 5, 6):
            if cid in leaders or cid in followers:
                preferred = cid
                break
        if preferred is not None:
            formation_leaders[key] = preferred
        elif leaders:
            formation_leaders[key] = leaders[0]
        elif followers:
            formation_leaders[key] = followers[0]

    formation_followers: list[tuple[int, dict]] = []

    def _apply_formation_filming(wplist: list[OrderedDict]) -> None:
        for wp in wplist:
            wp["filmingProperty"] = _mk_filming(
                operation_mode=OPMODE_HOLD,
                sensor=SENSOR_EO_IR,
                fov=31.2,
            )

    # ────────────────────────── 1) 미션 → 패킷 ──────────────────────────
    seen_aircraft: set[int] = set()
    seen_area_aircraft: set[int] = set()

    for miss in missions:
        aid = miss["aircraftID"]
        if aid not in (4, 5, 6):
            continue
        is_first_mission_for_aircraft = aid not in seen_aircraft
        seen_aircraft.add(aid)

        # 첫 임무는 take-over(있으면) 또는 기본 오프셋을 사용하고,
        # 두 번째 임무부터는 line/area 공통으로 300m 오프셋을 사용한다.
        if is_first_mission_for_aircraft:
            entry_anchor = take_over_map.get(aid)
            entry_offset_m = (
                SWEEP_ENTRY_OFFSET_TAKEOVER_M
                if entry_anchor is not None
                else SWEEP_ENTRY_OFFSET_M
            )
            entry_disabled = False
        else:
            entry_anchor = None
            entry_offset_m = SWEEP_ENTRY_OFFSET_FOLLOWON_M
            entry_disabled = not ENABLE_FOLLOWON_ENTRY_WP

        info = miss["individualMissionInfo"]
        mission_alt_offset_m = _aircraft_alt_offset_m(aid)
        mission_ground_ref_m = _median_ground_m(_collect_ref_points_from_info(info))

        def _mission_wp_alt(lat: float, lon: float) -> int:
            return _wp_alt(
                lat,
                lon,
                mission_alt_offset_m,
                mission_ground_ref_m,
            )

        try:
            mission_pattern_type = int(info.get("patternType", 0) or 0)
        except Exception:
            mission_pattern_type = 0
        try:
            mission_sep_hint = float(info.get("SEP", 0.0) or 0.0)
        except Exception:
            mission_sep_hint = 0.0
        try:
            mission_fov_hint = float(info.get("FOV", 0.0) or 0.0)
        except Exception:
            mission_fov_hint = 0.0
        try:
            mission_speed_hint_kmh = float(info.get("SPEED", 0.0) or 0.0)
        except Exception:
            mission_speed_hint_kmh = 0.0
        mtype = info.get("individualMissionType")
        wplist: list[OrderedDict] = []
        full_sweep_coords: list[dict] | None = None
        full_sweep_speed: float | None = None
        full_sweep_interp_points: int | None = None
        is_area_mission = False
        mission_sep_m = ALT_M
        mission_fov_deg = geom_fov_deg
        mission_cruise_speed = cruise_speed
        mission_default_search_speed = DEFAULT_SEARCH_SPEED
        mission_spacing_line = sweep_spacing_m
        mission_search_speed_weight = 1.0
        if mission_sep_hint > 0.0:
            mission_sep_m = mission_sep_hint
        if mission_fov_hint > 0.0:
            mission_fov_deg = mission_fov_hint
        if mission_speed_hint_kmh > 0.0:
            mission_cruise_speed = round(mission_speed_hint_kmh / 3.6, 2)
            mission_default_search_speed = round(
                mission_cruise_speed * DEFAULT_SEARCH_SPEED_MULTIPLIER, 2
            )

        formation_info: OrderedDict | None = None
        formation_is_flight = False
        formation_allow_empty = False

        formation_key = _formation_key(miss)
        formation_role = _formation_role(info)
        legacy_form = False
        if formation_role is None and _is_formation_line(info):
            formation_role = "follower"
            legacy_form = True

        leader_id = formation_leaders.get(formation_key) if formation_key is not None else None
        followers_all = (formation_group_sorted.get(formation_key) or {}).get("follower") or []
        if leader_id is not None:
            if aid == leader_id:
                formation_role = "leader"
            elif formation_role is None and aid in followers_all:
                formation_role = "follower"

        if leader_id is not None and formation_role == "follower" and aid != leader_id:
            followers = [fid for fid in followers_all if fid != leader_id]
            follower_idx = followers.index(aid) if aid in followers else 0
            dx, dy, dz = formation_offsets[min(follower_idx, len(formation_offsets) - 1)]
            formation_info = OrderedDict([
                ("leaderAircraftID", int(leader_id)),
                ("formation", OrderedDict([
                    ("dX", int(dx)),
                    ("dY", int(dy)),
                    ("dZ", int(dz)),
                ])),
            ])
            formation_is_flight = True
            formation_allow_empty = True
            packet = {
                "pathID": miss["pathID"],
                "aircraftID": aid,
                "wplist": [],
                "isFormationFlight": True,
                "formationInfo": formation_info,
                "_formation_key": formation_key,
                "_formation_follower": True,
            }
            packets.append(packet)
            formation_followers.append((formation_key, packet))
            continue

        formation_line_leader = legacy_form and formation_key is not None and formation_leaders.get(formation_key) == aid and mtype in (3, 4, 6)

        # 1-A. 통로정찰 / 영역수색 (type 3·4·6)
        if formation_line_leader:
            line = (info.get("lineList") or [{}])[0]
            coords = line.get("coordinateList") or []
            for coord in coords:
                lat = float(coord.get("latitude", 0.0))
                lon = float(coord.get("longitude", 0.0))
                wplist.append(OrderedDict([
                    ("waypointID", 0),
                    ("coordinate", {"latitude": lat, "longitude": lon, "altitude": _mission_wp_alt(lat, lon)}),
                    ("speed", cruise_speed),
                    ("eta", 0),
                    ("ecf", 0.0),
                    ("nextWaypointID", 0),
                    ("waypointPassType", PASS_FLYBY),
                    ("filmingProperty", _mk_filming(
                        operation_mode=OPMODE_HOLD,
                        sensor=SENSOR_EO_IR,
                    )),
                ]))
        elif mtype in (3, 4, 6):
            base, width = None, 100.0
            spacing_line = sweep_spacing_m
            is_area_mission = False

            # (i) lineList → corridor
            if info.get("lineList"):
                line = info["lineList"][0]
                width = line["width"]
                base = [(c["latitude"], c["longitude"]) for c in line["coordinateList"]]
                db_cfg = None
                if mtype == 6 and not (mission_sep_hint > 0.0 and mission_fov_hint > 0.0):
                    db_cfg = _select_corridor_db_config(width)
                if db_cfg:
                    mission_sep_m = float(db_cfg["sep"])
                    mission_fov_deg = _apply_db_fov_weight(float(db_cfg["fov"]))
                    vel = float(db_cfg.get("vel", 0.0) or 0.0)
                    if vel > 0:
                        mission_cruise_speed = _db_velocity_to_cruise_speed_mps(vel)
                        mission_default_search_speed = round(
                            mission_cruise_speed * DEFAULT_SEARCH_SPEED_MULTIPLIER, 2
                        )
                spacing_line = _sweep_spacing_m(separation_m=mission_sep_m, fov_deg=mission_fov_deg)
                _debug_sweep("LINE", separation=mission_sep_m, fov=mission_fov_deg, spacing=spacing_line)

            # (ii) areaList
            elif info.get("areaList"):
                is_area_mission = True
                mission_search_speed_weight = float(AREA_SEARCH_SPEED_WEIGHT)
                if mission_pattern_type != 3 and mission_fov_hint <= 0.0:
                    try:
                        mission_fov_deg = float(AREA_CUSTOM_FOV_DEG)
                    except Exception:
                        pass
                pts = [(p["latitude"], p["longitude"]) for p in info["areaList"][0]["coordinateList"]]

                area_sweep_mode = _runtime_area_sweep_mode()
                if area_sweep_mode == "vertical":
                    sweep_bearing_raw = info.get("BEARING")
                    move_bearing_raw = (
                        info.get("MOVE_BEARING")
                        if info.get("MOVE_BEARING") is not None
                        else (
                            miss.get("phaseMoveBearing_deg")
                            if miss.get("phaseMoveBearing_deg") is not None
                            else miss.get("move_bearing_deg")
                        )
                    )
                    if sweep_bearing_raw is None and move_bearing_raw is not None:
                        try:
                            sweep_bearing_raw = (float(move_bearing_raw) + 90.0) % 360.0
                        except Exception:
                            sweep_bearing_raw = None
                    if sweep_bearing_raw is None:
                        sweep_bearing_raw = miss.get("splitBearing_deg")
                    if sweep_bearing_raw is None:
                        sweep_bearing_raw = miss.get("bearing_deg")
                    if sweep_bearing_raw is None:
                        sweep_bearing_raw = 180.0
                else:
                    sweep_bearing_raw = (
                        miss.get("phaseMoveBearing_deg")
                        if miss.get("phaseMoveBearing_deg") is not None
                        else miss.get("move_bearing_deg")
                    )
                    if sweep_bearing_raw is None:
                        sweep_bearing_raw = info.get("MOVE_BEARING")
                    if sweep_bearing_raw is None:
                        sweep_bearing_raw = info.get("BEARING")
                    if sweep_bearing_raw is None:
                        sweep_bearing_raw = miss.get("bearing_deg", 90.0)
                try:
                    bearing = float(sweep_bearing_raw)
                except Exception:
                    bearing = 90.0
                th = math.radians(bearing)
                sweep_nx, sweep_ny = math.cos(th), -math.sin(th)
                area_db_meta = None
                if mission_pattern_type != 3:
                    area_db_meta = _select_area_db_config(pts, bearing)
                    if area_db_meta:
                        db_cfg = area_db_meta["config"]
                        mission_sep_m = float(db_cfg.get("sep", mission_sep_m) or mission_sep_m)
                        mission_fov_deg = _apply_db_fov_weight(
                            float(db_cfg.get("fov", mission_fov_deg) or mission_fov_deg)
                        )
                        try:
                            print(
                                f"[SWEEP][AREA-DB] missionID={miss.get('individualMissionID')} "
                                f"widthRef={area_db_meta['width_ref_m']:.2f}m "
                                f"dbWidth={float(db_cfg.get('width', 0.0) or 0.0):.2f}m "
                                f"fov={mission_fov_deg:.2f} sep={mission_sep_m:.2f}"
                            )
                        except Exception:
                            pass

                prev_pt = miss.get("prevPoint", pts[0])

                strip_spacing = mission_sep_m
                # patternType 3: area는 lineSearch 없이 직하방 고정 촬영으로 계획
                if mission_pattern_type == 3:
                    area_spacing_scale = _area_raw_spacing_scale()
                    nadir_sep_ref = float(mission_sep_hint) if mission_sep_hint > 0.0 else float(mission_alt_offset_m)
                    nadir_fov = (
                        float(mission_fov_hint)
                        if mission_fov_hint > 0.0
                        else _select_nadir_fov_by_altitude(nadir_sep_ref, float(AREA_NADIR_FOV_DEG))
                    )
                    strip_spacing = _sweep_spacing_m(
                        separation_m=nadir_sep_ref,
                        fov_deg=nadir_fov,
                        spacing_scale=area_spacing_scale,
                    )
                    _debug_sweep(
                        "AREA-NADIR",
                        separation=nadir_sep_ref,
                        fov=nadir_fov,
                        spacing=strip_spacing,
                    )
                    if build_nadir_bf_overflight_coords is None:
                        raise RuntimeError("BF nadir planner import failed (patternType=3 requires BF-only planner).")

                    nadir_coords, runtime_fov = build_nadir_bf_overflight_coords(
                        polygon_llh=pts,
                        anchor_llh=prev_pt,
                        bearing_deg=bearing,
                        separation_m=nadir_sep_ref,
                        fov_deg=nadir_fov,
                        min_segment_m=MIN_SWEEP_LEN_M,
                        spacing_margin=float(SWEEP_SPACING_MARGIN) * area_spacing_scale,
                        altitude_fn=lambda lat, lon: _mission_wp_alt(lat, lon),
                        cruise_alt_m=float(mission_alt_offset_m),
                        r_min=200.0,
                        goal_length_m=300.0,
                    )
                    if runtime_fov and runtime_fov > 0:
                        nadir_fov = float(runtime_fov)
                    if not nadir_coords:
                        raise RuntimeError(
                            f"BF nadir planner returned empty path (missionID={miss.get('individualMissionID')})."
                        )
                    print(
                        f"[SWEEP][AREA-NADIR][bf-only] missionID={miss.get('individualMissionID')} "
                        f"points={len(nadir_coords)} fov={nadir_fov:.1f}"
                    )

                    for point in nadir_coords:
                        lat = float(point.get("latitude", 0.0))
                        lon = float(point.get("longitude", 0.0))
                        alt = int(point.get("altitude", _mission_wp_alt(lat, lon)))
                        wplist.append(OrderedDict([
                            ("waypointID", 0),
                            ("coordinate", {
                                "latitude": lat,
                                "longitude": lon,
                                "altitude": alt,
                            }),
                            ("speed", mission_cruise_speed),
                            ("eta", 2500),
                            ("ecf", 0.0),
                            ("nextWaypointID", 0),
                            ("waypointPassType", PASS_FLYOVER),
                            ("filmingProperty", _mk_filming(
                                operation_mode=OPMODE_HOLD,
                                sensor=SENSOR_EO_IR,
                                fov=nadir_fov,
                            )),
                        ]))
                else:
                    area_spacing_scale = _area_raw_spacing_scale()
                    spacing_line = _sweep_spacing_m(
                        separation_m=mission_sep_m,
                        fov_deg=mission_fov_deg,
                        spacing_scale=area_spacing_scale,
                    )
                    mission_spacing_line = spacing_line
                    strip_spacing = spacing_line
                    _debug_sweep("AREA", separation=mission_sep_m, fov=mission_fov_deg, spacing=spacing_line)
                    banded_coords = [_poly_sweeps_general(
                        poly_llh=pts,
                        anchor_llh=prev_pt,
                        bearing_deg=bearing,
                        fov_deg=mission_fov_deg,
                        separation_m=mission_sep_m,
                        spacing_scale=area_spacing_scale,
                    )]
                    if not any(coord_list for coord_list in banded_coords):
                        fallback_coords = _fallback_poly_long_axis_sweep(pts)
                        if fallback_coords:
                            banded_coords = [fallback_coords]
                    if not banded_coords:
                        continue

                    line_counter = 0
                    last_off_xy: tuple[float, float] | None = None
                    lat0, lon0 = pts[0]

                    for band_idx, coord_list in enumerate(banded_coords):
                        lines = [coord_list[i:i+2] for i in range(0, len(coord_list), 2)]
                        if not lines:
                            continue
                        # 짧은 스윕 필터
                        MIN_SWEEP_LEN = MIN_SWEEP_LEN_M
                        filtered = []
                        lat0, lon0 = lines[0][0]["latitude"], lines[0][0]["longitude"]
                        for ln in lines:
                            s_lat, s_lon = ln[0]["latitude"], ln[0]["longitude"]
                            e_lat, e_lon = ln[1]["latitude"], ln[1]["longitude"]
                            dx = (e_lon - s_lon) * 111_132 * math.cos(math.radians((s_lat + e_lat) / 2))
                            dy = (e_lat - s_lat) * 111_132
                            if math.hypot(dx, dy) >= MIN_SWEEP_LEN:
                                filtered.append(ln)
                        lines = filtered
                        if not lines:
                            fallback_coords = _fallback_poly_long_axis_sweep(pts)
                            if len(fallback_coords) >= 2:
                                lines = [fallback_coords[:2]]
                            else:
                                continue
                        lat0, lon0 = lines[0][0]["latitude"], lines[0][0]["longitude"]

                        for ln in lines:
                            idx = line_counter
                            line_counter += 1
                            s, e = ln
                            s_xy = llh_to_xy(s['latitude'], s['longitude'], lat0, lon0)
                            e_xy = llh_to_xy(e['latitude'], e['longitude'], lat0, lon0)

                            mid_xy = ((s_xy[0] + e_xy[0]) / 2, (s_xy[1] + e_xy[1]) / 2)
                            off_xy = (
                                mid_xy[0] - sweep_nx * strip_spacing,
                                mid_xy[1] - sweep_ny * strip_spacing,
                            )
                            last_off_xy = off_xy
                            off_lat, off_lon = xy_to_llh(*off_xy, lat0, lon0)

                            sweep = [e, s] if idx % 2 else [s, e]
                            sweep_width = _avg_sweep_width_m(sweep)
                            sweep_interp_points = _interp_points_hint(SWEEP_LINE_INTERP_POINTS) or 2
                            sweep_speed = spacing_based_search_speed(
                                sweep_len_m=sweep_width,
                                spacing_m=spacing_line,
                                cruise_speed_mps=mission_cruise_speed,
                                weight=mission_search_speed_weight,
                            )
                            if sweep_speed is None:
                                sweep_speed = mission_default_search_speed

                            wplist.append(OrderedDict([
                                ("waypointID", 0),
                                ("_path_search_speed_weight", float(mission_search_speed_weight)),
                                ("coordinate", {"latitude": off_lat, "longitude": off_lon, "altitude": _mission_wp_alt(off_lat, off_lon)}),
                                ("speed", mission_cruise_speed),
                                ("eta", 2500),
                                ("ecf", 0.0),
                                ("nextWaypointID", 0),
                                ("waypointPassType", PASS_FLYBY),
                                ("filmingProperty", _mk_filming(
                                    operation_mode=OPMODE_LINE,
                                    fov=mission_fov_deg,
                                    sensor=SENSOR_EO_IR,
                                    line_search=OrderedDict([
                                        ("coordinateList", _interpolate_line_coords(sweep, sweep_interp_points)),
                                        ("searchSpeed", sweep_speed),
                                        ("interpolationPoints", sweep_interp_points),
                                    ]),
                                )),
                            ]))

                # ➌ 종료 WP (Loiter + POINT 촬영)
            # (iii) Corridor-planner
            if base and len(base) >= 2:
                planner = UAVMissionPlanner(
                    base, corridor_width=width, separation=mission_sep_m,
                    fov_deg=mission_fov_deg, cruise_speed=mission_cruise_speed, crs="lla",
                )
                use_centerline = ROUTE_PLANNER_NAME in ("linear", "algo2")
                if use_centerline:
                    base_xy: list[tuple[float, float]] = []
                    proj_fwd = getattr(planner, "_proj_fwd", None)
                    if proj_fwd is not None:
                        for lat, lon in base:
                            x, y = proj_fwd(lon, lat)
                            base_xy.append((x, y))
                    else:
                        lat0, lon0 = base[0]
                        base_xy = [llh_to_xy(lat, lon, lat0, lon0) for lat, lon in base]

                    sweep_mid_xy: list[tuple[float, float]] = []
                    for sw in planner.sweeps:
                        sweep_mid_xy.append(((sw[0][0] + sw[1][0]) / 2, (sw[0][1] + sw[1][1]) / 2))

                    def _select_sweep_indices(
                        points_xy: list[tuple[float, float]],
                        sweep_midpoints: list[tuple[float, float]],
                    ) -> list[int]:
                        if not points_xy or not sweep_midpoints:
                            return []
                        total_pts = len(points_xy)
                        total_sweeps = len(sweep_midpoints)
                        if total_sweeps <= total_pts:
                            return list(range(total_sweeps))
                        selected: list[int] = []
                        start_idx = 0
                        for pos, pt in enumerate(points_xy):
                            remaining = total_pts - pos - 1
                            max_idx = total_sweeps - 1 - remaining
                            if max_idx < start_idx:
                                max_idx = start_idx
                            best_idx = start_idx
                            best_dist = None
                            for idx in range(start_idx, max_idx + 1):
                                dx = sweep_midpoints[idx][0] - pt[0]
                                dy = sweep_midpoints[idx][1] - pt[1]
                                dist = dx * dx + dy * dy
                                if best_dist is None or dist < best_dist:
                                    best_dist = dist
                                    best_idx = idx
                            selected.append(best_idx)
                            start_idx = best_idx + 1
                        return selected

                    sampled_base_xy = _resample_polyline_xy(base_xy, _line_route_wp_spacing_m())
                    sweep_indices = _select_sweep_indices(sampled_base_xy, sweep_mid_xy)
                    if not sweep_indices:
                        sweep_indices = list(range(len(planner.sweeps)))
                    anchor_list = planner.offset_wps

                    merged_coords: list[dict] = []
                    merged_sweep_speeds: list[float] = []
                    for sw_idx, sw in enumerate(planner.sweeps):
                        s_xy, e_xy = sw
                        if sw_idx % 2:
                            s_xy, e_xy = e_xy, s_xy
                        s_lat, s_lon = planner._proj_back(s_xy[0], s_xy[1])[::-1]
                        e_lat, e_lon = planner._proj_back(e_xy[0], e_xy[1])[::-1]
                        merged_coords.append({
                            "latitude": s_lat,
                            "longitude": s_lon,
                            "altitude": _dem_alt(s_lat, s_lon),
                        })
                        merged_coords.append({
                            "latitude": e_lat,
                            "longitude": e_lon,
                            "altitude": _dem_alt(e_lat, e_lon),
                        })
                        sweep_speed = spacing_based_search_speed(
                            sweep_len_m=_avg_sweep_width_m([
                                {"latitude": s_lat, "longitude": s_lon, "altitude": _dem_alt(s_lat, s_lon)},
                                {"latitude": e_lat, "longitude": e_lon, "altitude": _dem_alt(e_lat, e_lon)},
                            ]),
                            spacing_m=spacing_line,
                            cruise_speed_mps=mission_cruise_speed,
                        )
                        if sweep_speed is None:
                            sweep_speed = mission_default_search_speed
                        merged_sweep_speeds.append(float(sweep_speed))
                    if merged_coords:
                        full_sweep_coords = merged_coords
                        full_sweep_interp_points = _interp_points_hint(SWEEP_LINE_INTERP_POINTS) or 2
                        if _runtime_uav_plan_mode() == "dub_path":
                            if merged_sweep_speeds:
                                full_sweep_speed = round(
                                    sum(merged_sweep_speeds) / max(len(merged_sweep_speeds), 1),
                                    2,
                                )
                        if full_sweep_speed is None:
                            merged_width = _avg_sweep_width_m(
                                merged_coords,
                                points_per_line=2,
                            )
                            full_sweep_speed = spacing_based_search_speed(
                                sweep_len_m=merged_width,
                                spacing_m=spacing_line,
                                cruise_speed_mps=mission_cruise_speed,
                            )
                        if full_sweep_speed is None:
                            full_sweep_speed = mission_default_search_speed
                else:
                    sweep_indices = list(range(len(planner.sweeps)))
                    anchor_list = planner.orange_pts

                last_anchor_xy: tuple[float, float] | None = None
                last_first_xy: tuple[float, float] | None = None
                for idx in sweep_indices:
                    if idx >= len(planner.sweeps) or idx >= len(anchor_list):
                        continue
                    anchor_xy = anchor_list[idx]
                    sw = planner.sweeps[idx]

                    s_xy, e_xy = sw
                    if idx % 2:
                        s_xy, e_xy = e_xy, s_xy
                    w_lat, w_lon = planner._proj_back(anchor_xy[0], anchor_xy[1])[::-1]

                    s_lat, s_lon = planner._proj_back(s_xy[0], s_xy[1])[::-1]
                    e_lat, e_lon = planner._proj_back(e_xy[0], e_xy[1])[::-1]
                    coord_list = [
                        {"latitude": s_lat, "longitude": s_lon, "altitude": _dem_alt(s_lat, s_lon)},
                        {"latitude": e_lat, "longitude": e_lon, "altitude": _dem_alt(e_lat, e_lon)},
                    ]

                    sweep_interp_points = _interp_points_hint(SWEEP_LINE_INTERP_POINTS) or 2
                    coord_width = _avg_sweep_width_m(coord_list)
                    coord_speed = spacing_based_search_speed(
                        sweep_len_m=coord_width,
                        spacing_m=spacing_line,
                        cruise_speed_mps=mission_cruise_speed,
                    )
                    if coord_speed is None:
                        coord_speed = mission_default_search_speed

                    wplist.append(OrderedDict([
                        ("waypointID", 0),
                        ("coordinate", {"latitude": w_lat, "longitude": w_lon, "altitude": _mission_wp_alt(w_lat, w_lon)}),
                        ("speed", cruise_speed),
                        ("eta", 2500),
                        ("ecf", 0.0),
                        ("nextWaypointID", 0),
                        ("waypointPassType", PASS_FLYBY),
                        ("filmingProperty", _mk_filming(
                            operation_mode=OPMODE_LINE,
                            fov=mission_fov_deg,
                            sensor=SENSOR_EO_IR,
                            line_search=OrderedDict([
                                ("coordinateList", _interpolate_line_coords(coord_list, sweep_interp_points)),
                                ("searchSpeed", coord_speed),
                                ("interpolationPoints", sweep_interp_points),
                            ]),
                        )),
                        ("_sweepIdx", int(idx)),
                    ]))

                    last_anchor_xy = anchor_xy
                    last_first_xy = s_xy

        # 1-B. 좌표점정찰 / hold 미션 (type 5)
        elif mtype == 5:
            base: list[tuple[float, float]] = []
            if info.get("coordinateList"):
                base = [(c["latitude"], c["longitude"]) for c in info["coordinateList"]]
            elif info.get("lineList"):
                base = [(c["latitude"], c["longitude"]) for c in info["lineList"][0]["coordinateList"]]

            try:
                hold_req_s = float(info.get("holdingReqTime", 0.0) or 0.0)
            except Exception:
                hold_req_s = 0.0
            pass_type = PASS_LOITER if hold_req_s > 0.05 else PASS_FLYBY

            if len(base) == 1:
                lat, lon = base[0]
                coord = {"latitude": lat, "longitude": lon, "altitude": _mission_wp_alt(lat, lon)}
                wplist.append(OrderedDict([
                    ("waypointID", 0),
                    ("coordinate", coord),
                    ("speed", mission_cruise_speed),
                    ("eta", 0),
                    ("ecf", 1.0),
                    ("nextWaypointID", 0),
                    ("waypointPassType", pass_type),
                    ("filmingProperty", _mk_point_filming_for_coord(coord)),
                ]))
            elif len(base) >= 2:
                raw_pts = _plan_route_points(
                    base,
                    cruise_speed=mission_cruise_speed,
                    heading_tol_deg=turn_step_deg,
                )
                simp: list[dict] = [raw_pts[0]]
                for p in raw_pts[1:]:
                    d = math.hypot(
                        (p["lon"] - simp[-1]["lon"]) * DEG_M * math.cos(
                            math.radians((p["lat"] + simp[-1]["lat"]) / 2)),
                        (p["lat"] - simp[-1]["lat"]) * DEG_M
                    )
                    if d >= MIN_ROUTE_SPACING_M or p is raw_pts[-1]:
                        simp.append(p)

                for idx, p in enumerate(simp):
                    coord = {
                        "latitude": p["lat"],
                        "longitude": p["lon"],
                        "altitude": _mission_wp_alt(p["lat"], p["lon"]),
                    }
                    is_last = idx == len(simp) - 1
                    wplist.append(OrderedDict([
                        ("waypointID", 0),
                        ("coordinate", coord),
                        ("speed", mission_cruise_speed),
                        ("eta", p["eta_ms"]),
                        ("ecf", 0.0),
                        ("nextWaypointID", 0),
                        ("waypointPassType", pass_type if is_last else PASS_FLYBY),
                        ("filmingProperty", _mk_point_filming_for_coord(coord) if is_last else {}),
                    ]))

        # 1-C. 이동 미션 (type 7)
        elif mtype == 7:
            base: list[tuple[float, float]] = []
            if info.get("coordinateList"):
                base = [(c["latitude"], c["longitude"]) for c in info["coordinateList"]]
            elif info.get("lineList"):
                base = [(c["latitude"], c["longitude"]) for c in info["lineList"][0]["coordinateList"]]

            if len(base) == 1:
                lat, lon = base[0]
                wplist.append(OrderedDict([
                    ("waypointID", 0),
                    ("coordinate", {"latitude": lat, "longitude": lon, "altitude": _mission_wp_alt(lat, lon)}),
                    ("speed", cruise_speed), ("eta", 0), ("ecf", 1.0),
                    ("nextWaypointID", 0), ("waypointPassType", 1),
                    ("filmingProperty", {}),
                ]))
            elif len(base) >= 2:
                raw_pts = _plan_route_points(
                    base,
                    cruise_speed=cruise_speed,
                    heading_tol_deg=turn_step_deg,
                )
                MIN_SPACING_M = MIN_ROUTE_SPACING_M
                simp: list[dict] = [raw_pts[0]]
                for p in raw_pts[1:]:
                    d = math.hypot(
                        (p["lon"] - simp[-1]["lon"]) * DEG_M * math.cos(
                            math.radians((p["lat"] + simp[-1]["lat"]) / 2)),
                        (p["lat"] - simp[-1]["lat"]) * DEG_M
                    )
                    if d >= MIN_SPACING_M or p is raw_pts[-1]:
                        simp.append(p)

                for p in simp:
                    wplist.append(OrderedDict([
                        ("waypointID", 0),
                        ("coordinate", {"latitude": p["lat"], "longitude": p["lon"], "altitude": _mission_wp_alt(p["lat"], p["lon"])}),
                        ("speed", cruise_speed),
                        ("eta", p["eta_ms"]),
                        ("ecf", 0.0),
                        ("nextWaypointID", 0),
                        ("waypointPassType", 1),
                        ("filmingProperty", _mk_filming(
                            operation_mode=OPMODE_HOLD,
                            sensor=SENSOR_EO_IR
                        )),
                    ]))

        if leader_id is not None and aid == leader_id:
            formation_info = OrderedDict([
                ("leaderAircraftID", int(leader_id)),
                ("formation", OrderedDict([
                    ("dX", 0),
                    ("dY", 0),
                    ("dZ", 0),
                ])),
            ])
            formation_is_flight = True
            if wplist:
                _apply_formation_filming(wplist)

        skip_area_entry_prefix = False
        if mtype in (3, 4, 6) and is_area_mission:
            skip_area_entry_prefix = aid not in seen_area_aircraft
            seen_area_aircraft.add(aid)

        # 1-D. 패킷 저장
        if wplist or formation_allow_empty:
            packet = {
                "pathID": miss["pathID"],
                "aircraftID": aid,
                "wplist": wplist,
                "_mission_sep_m": float(mission_sep_m),
                "_mission_cruise_speed_mps": float(mission_cruise_speed),
                "_mission_default_search_speed": float(mission_default_search_speed),
                "_mission_spacing_line_m": float(mission_spacing_line),
                "_mission_search_speed_weight": float(mission_search_speed_weight),
                "_mission_alt_offset_m": float(mission_alt_offset_m),
                "_mission_ground_ref_m": mission_ground_ref_m,
                "_entry_anchor": entry_anchor,
                "_entry_offset_m": entry_offset_m,
                "_entry_disabled": bool(entry_disabled),
                "_skip_area_entry_prefix": bool(skip_area_entry_prefix),
            }
            if isinstance(miss.get("prevPoint"), dict):
                packet["_prev_point"] = deepcopy(miss.get("prevPoint"))
            if formation_info:
                packet["formationInfo"] = formation_info
                packet["isFormationFlight"] = bool(formation_is_flight)
                packet["_formation_key"] = formation_key
            if mtype in (3, 4, 6) and is_area_mission:
                packet["_is_area"] = True
                packet["_area_sweep_mode"] = str(area_sweep_mode or "parallel")
                packet["_area_bearing_deg"] = float(bearing)
                if mission_pattern_type == 3:
                    packet["_area_nadir"] = True
            if full_sweep_coords:
                packet["fullSweepCoords"] = full_sweep_coords
                packet["fullSweepSearchSpeed"] = full_sweep_speed
                packet["fullSweepInterpPoints"] = full_sweep_interp_points
            packets.append(packet)

    if formation_followers:
        leader_wplist_by_key: dict[int, list[OrderedDict]] = {}
        for pkt in packets:
            fkey = pkt.get("_formation_key")
            if fkey is None:
                continue
            leader_id = formation_leaders.get(fkey)
            if leader_id is None:
                continue
            if int(pkt.get("aircraftID", 0)) == int(leader_id) and pkt.get("wplist"):
                leader_wplist_by_key[int(fkey)] = pkt["wplist"]
        for fkey, fpkt in formation_followers:
            leader_wplist = leader_wplist_by_key.get(int(fkey))
            if leader_wplist:
                fpkt["wplist"] = deepcopy(leader_wplist)

    # ────────────────────────── 3) WP ID · 링크 · ECF ────────────────
    prev_tail_by_aircraft: Dict[int, dict] = {}
    prev_last_wp_coord_by_aircraft: Dict[int, dict] = {}
    prev_heading_by_aircraft: Dict[int, float] = {}
    prev_area_offset_sign_by_aircraft: Dict[int, int] = {}
    prev_area_pkt_by_aircraft: Dict[int, dict] = {}
    for pkt in packets:
        wps = pkt["wplist"]
        aid = int(pkt.get("aircraftID", 0))
        prev_tail_coord = prev_tail_by_aircraft.get(aid)
        prev_last_wp_coord = prev_last_wp_coord_by_aircraft.get(aid)
        entry_anchor = pkt.get("_entry_anchor") if isinstance(pkt.get("_entry_anchor"), dict) else None
        entry_offset_m = float(pkt.get("_entry_offset_m", SWEEP_ENTRY_OFFSET_M))
        entry_disabled = bool(pkt.get("_entry_disabled"))
        if pkt.get("_formation_follower"):
            continue
        if not wps:
            prev_area_pkt_by_aircraft.pop(aid, None)
            continue
        try:
            packet_cruise_speed = float(pkt.get("_mission_cruise_speed_mps", cruise_speed) or cruise_speed)
        except Exception:
            packet_cruise_speed = float(cruise_speed)
        try:
            packet_default_search_speed = float(
                pkt.get("_mission_default_search_speed", DEFAULT_SEARCH_SPEED) or DEFAULT_SEARCH_SPEED
            )
        except Exception:
            packet_default_search_speed = float(DEFAULT_SEARCH_SPEED)
        try:
            packet_search_speed_weight = float(pkt.get("_mission_search_speed_weight", 1.0) or 1.0)
        except Exception:
            packet_search_speed_weight = 1.0
        try:
            packet_spacing_line = float(pkt.get("_mission_spacing_line_m", sweep_spacing_m) or sweep_spacing_m)
        except Exception:
            packet_spacing_line = float(sweep_spacing_m)

        is_area_pkt = bool(pkt.get("_is_area"))
        is_area_nadir = bool(pkt.get("_area_nadir"))
        area_sweep_mode = str(pkt.get("_area_sweep_mode") or "parallel").strip().lower()
        if not is_area_pkt:
            packet_search_speed_weight = 1.0
        uav_plan_mode = _runtime_uav_plan_mode()
        sweep_indices = [idx for idx, wp in enumerate(wps) if _has_line_search(wp)]
        entry_wp: OrderedDict | None = None
        merge_mode = SWEEP_MERGE_MODE
        merge_heading_deg = AREA_SWEEP_MERGE_HEADING_DEG if is_area_pkt else SWEEP_MERGE_HEADING_DEG
        if is_area_pkt and not is_area_nadir:
            merge_mode = "none"
            # Preserve one route waypoint per sweep line for area missions.
        if len(sweep_indices) >= 2:
            first_idx = sweep_indices[0]
            second_idx = sweep_indices[1]
            first_coord = wps[first_idx].get("coordinate") or {}
            second_coord = wps[second_idx].get("coordinate") or {}

            lat0 = float(first_coord.get("latitude", 0.0))
            lon0 = float(first_coord.get("longitude", 0.0))
            lat1 = float(second_coord.get("latitude", lat0))
            lon1 = float(second_coord.get("longitude", lon0))
            entry_base_is_takeover = False
            dynamic_entry_offset_m = entry_offset_m
            if area_sweep_mode == "parallel" and isinstance(prev_tail_coord, dict):
                p_lat = float(prev_tail_coord.get("latitude", lat0))
                p_lon = float(prev_tail_coord.get("longitude", lon0))
                vec_x, vec_y = llh_to_xy(lat0, lon0, p_lat, p_lon)
                norm = math.hypot(vec_x, vec_y)
                if norm >= 1.0:
                    dynamic_entry_offset_m = max(0.0, norm * 0.5)
                else:
                    vec_x = vec_y = 0.0
            elif entry_anchor is not None:
                vec_x, vec_y = llh_to_xy(lat1, lon1, lat0, lon0)
                entry_base_is_takeover = True
            else:
                vec_x, vec_y = llh_to_xy(lat1, lon1, lat0, lon0)
                entry_base_is_takeover = False
            norm = math.hypot(vec_x, vec_y)
            skip_entry_for_close_takeover = (
                entry_base_is_takeover
                and (not is_area_pkt)
                and norm <= float(LINE_ENTRY_SKIP_IF_TAKEOVER_WITHIN_M)
            )
            if norm < 1.0 and entry_base_is_takeover:
                # Fallback: use sweep direction when the takeover point is too close/invalid.
                vec_x, vec_y = llh_to_xy(lat1, lon1, lat0, lon0)
                norm = math.hypot(vec_x, vec_y)
            if norm >= 1.0 and not is_area_pkt:
                ux, uy = vec_x / norm, vec_y / norm
                if not entry_disabled and not skip_entry_for_close_takeover:
                    entry_xy = (-ux * dynamic_entry_offset_m, -uy * dynamic_entry_offset_m)
                    entry_lat, entry_lon = xy_to_llh(entry_xy[0], entry_xy[1], lat0, lon0)
                    entry_coord = OrderedDict([
                        ("latitude", round(entry_lat, 6)),
                        ("longitude", round(entry_lon, 6)),
                        ("altitude", _mission_wp_alt(entry_lat, entry_lon)),
                    ])
                    entry_wp = OrderedDict([
                        ("waypointID", 0),
                        ("_flyover_entry_offset", True),
                        ("coordinate", entry_coord),
                        ("speed", packet_cruise_speed),
                        ("eta", 0),
                        ("ecf", 0.0),
                        ("nextWaypointID", 0),
                        ("waypointPassType", PASS_FLYBY),
                        ("filmingProperty", _mk_filming(
                            operation_mode=OPMODE_HOLD,
                            fov=ENTRY_HOLD_FOV_DEG,
                            sensor=SENSOR_EO_IR,
                            gimbal_pitch=ENTRY_HOLD_GIMBAL_PITCH,
                            gimbal_yaw=ENTRY_HOLD_GIMBAL_YAW,
                        )),
                    ])

            has_existing_preline_prefix = bool(first_idx > 0)
            initial_first_wp = wps[first_idx]
            initial_first_fp = initial_first_wp.get("filmingProperty") or OrderedDict()
            initial_first_line_search = deepcopy(initial_first_fp.get("lineSearch") or {})
            initial_first_coords = deepcopy(initial_first_line_search.get("coordinateList") or [])
            initial_first_search_speed = initial_first_line_search.get("searchSpeed")
            initial_first_interp_points = _interp_points_hint(initial_first_line_search.get("interpolationPoints"))
            if initial_first_search_speed is None:
                first_width = _avg_sweep_width_m(initial_first_coords, points_per_line=initial_first_interp_points)
                initial_first_search_speed = spacing_based_search_speed(
                    sweep_len_m=first_width,
                    spacing_m=packet_spacing_line,
                    cruise_speed_mps=packet_cruise_speed,
                    weight=packet_search_speed_weight,
                )
            if initial_first_search_speed is None:
                initial_first_search_speed = packet_default_search_speed
            sweep_items: list[dict] = [{
                "idx": first_idx,
                "wp": initial_first_wp,
                "fp": initial_first_fp,
                "coord": initial_first_wp.get("coordinate") or {},
                "coords": initial_first_coords,
                "search_speed": initial_first_search_speed,
                "fov": initial_first_fp.get("fieldOfView", FOV_DEG),
                "interp_points": initial_first_interp_points,
                "sweep_idx": initial_first_wp.get("_sweepIdx"),
            }]
            for idx in sweep_indices[1:]:
                wp = wps[idx]
                fp = wp.get("filmingProperty") or OrderedDict()
                ls = fp.get("lineSearch") or {}
                coords = deepcopy(ls.get("coordinateList") or [])
                if not coords:
                    continue
                search_speed = ls.get("searchSpeed")
                interp_points = _interp_points_hint(ls.get("interpolationPoints"))
                if search_speed is None:
                    width_m = _avg_sweep_width_m(coords, points_per_line=interp_points)
                    search_speed = spacing_based_search_speed(
                        sweep_len_m=width_m,
                        spacing_m=packet_spacing_line,
                        cruise_speed_mps=packet_cruise_speed,
                        weight=packet_search_speed_weight,
                    )
                sweep_items.append({
                    "idx": idx,
                    "wp": wp,
                    "fp": fp,
                    "coord": wp.get("coordinate") or {},
                    "coords": coords,
                    "search_speed": search_speed,
                    "fov": fp.get("fieldOfView", FOV_DEG),
                    "interp_points": interp_points,
                    "sweep_idx": wp.get("_sweepIdx"),
                })

            reference_coord = prev_tail_coord
            if is_area_pkt and isinstance(prev_last_wp_coord, dict):
                reference_coord = prev_last_wp_coord
            if not isinstance(reference_coord, dict):
                reference_coord = pkt.get("_prev_point") if isinstance(pkt.get("_prev_point"), dict) else None
            if not isinstance(reference_coord, dict):
                reference_coord = entry_anchor if isinstance(entry_anchor, dict) else None

            sweep_items = _orient_sweep_items(sweep_items, reference_coord)
            if uav_plan_mode == "dub_path" and is_area_pkt:
                area_bearing_deg = pkt.get("_area_bearing_deg")
                area_sep_offset_m = float(pkt.get("_mission_sep_m", 0.0) or 0.0) * float(AREA_ROUTE_OFFSET_SCALE)
                for item in sweep_items:
                    wp = item["wp"]
                    fp = wp.get("filmingProperty") or OrderedDict()
                    ls = fp.get("lineSearch") or OrderedDict()
                    coords = deepcopy(item.get("coords") or [])
                    if coords:
                        ls["coordinateList"] = coords
                        fp["lineSearch"] = ls
                        shifted = _area_wp_coord_from_sweep(
                            coords,
                            bearing_deg=area_bearing_deg,
                            offset_m=area_sep_offset_m,
                            altitude_fn=_mission_wp_alt,
                            reference_coord=reference_coord,
                        )
                        if shifted is not None:
                            wp["coordinate"] = shifted
                            item["coord"] = deepcopy(shifted)
            _apply_oriented_sweep_items_to_wps(wps, sweep_items)
            first_item = sweep_items[0]
            first_idx = int(first_item["idx"])
            first_wp = first_item["wp"]
            first_fp = first_item["fp"]
            first_coord = first_item.get("coord") or {}
            first_coords = deepcopy(first_item.get("coords") or [])
            first_search_speed = first_item.get("search_speed")
            first_interp_points = first_item.get("interp_points")
            records = sweep_items[1:]
            forced_remove_indices: list[int] = []

            if first_coords and records and not is_area_pkt:
                start_target = deepcopy(first_coords[0])
                first_fov = first_fp.get("fieldOfView", FOV_DEG)
                close_takeover_line_entry = False
                keep_first_line_search = False
                close_takeover_sep_m = 0.0
                close_takeover_route_m = 0.0
                close_takeover_scan_end_idx: int | None = None
                close_takeover_progress_xy: tuple[float, float] | None = None
                takeover_coord: OrderedDict | None = None
                if (
                    uav_plan_mode == "dub_path"
                    and is_alg2
                    and (not is_area_pkt)
                    and isinstance(entry_anchor, dict)
                ):
                    close_takeover_sep_m = float(pkt.get("_mission_sep_m", 0.0) or 0.0)
                    if close_takeover_sep_m > 0.0:
                        takeover_coord = _coord_with_altitude(entry_anchor, _mission_wp_alt)
                        close_takeover_compare_coord = _coord_midpoint(first_coords)
                        if not isinstance(close_takeover_compare_coord, dict):
                            close_takeover_compare_coord = start_target if isinstance(start_target, dict) else first_coord
                        close_takeover_route_m = _dist_between_coords(takeover_coord, close_takeover_compare_coord)
                        if close_takeover_route_m < close_takeover_sep_m:
                            close_takeover_line_entry = True

                if close_takeover_line_entry and takeover_coord is not None:
                    entry_wp = OrderedDict([
                        ("waypointID", 0),
                        ("coordinate", OrderedDict(takeover_coord)),
                        ("speed", packet_cruise_speed),
                        ("eta", 0),
                        ("ecf", 0.0),
                        ("nextWaypointID", 0),
                        ("waypointPassType", PASS_FLYOVER),
                        ("filmingProperty", _mk_filming(
                            operation_mode=OPMODE_HOLD,
                            fov=first_fov,
                            sensor=SENSOR_EO_IR,
                            gimbal_pitch=ENTRY_HOLD_GIMBAL_PITCH,
                            gimbal_yaw=ENTRY_HOLD_GIMBAL_YAW,
                        )),
                    ])
                if has_existing_preline_prefix and not close_takeover_line_entry:
                    keep_first_line_search = True

                elif not close_takeover_line_entry and not (uav_plan_mode == "dub_path" and is_area_pkt):
                    first_wp["filmingProperty"] = OrderedDict([
                        ("fieldOfView", first_fov),
                        ("sensorType", SENSOR_EO_IR),
                        ("operationMode", OPMODE_POINT),
                        ("coordinateOrientation", OrderedDict([
                            ("coordinate", OrderedDict([
                                ("latitude", float(start_target.get("latitude", 0.0))),
                                ("longitude", float(start_target.get("longitude", 0.0))),
                                ("altitude", 0),
                            ]))
                        ])),
                    ])
                if (
                    (not close_takeover_line_entry)
                    and (not keep_first_line_search)
                    and area_sweep_mode == "parallel"
                    and norm >= 1.0
                    and not (uav_plan_mode == "dub_path" and is_area_pkt)
                ):
                    first_shift_m = min(120.0, max(0.0, dynamic_entry_offset_m * 0.2))
                    if first_shift_m >= 1.0:
                        first_shift_xy = (-ux * first_shift_m, -uy * first_shift_m)
                        shift_lat, shift_lon = xy_to_llh(first_shift_xy[0], first_shift_xy[1], lat0, lon0)
                        first_wp["coordinate"] = OrderedDict([
                            ("latitude", round(shift_lat, 6)),
                            ("longitude", round(shift_lon, 6)),
                            ("altitude", _mission_wp_alt(shift_lat, shift_lon)),
                        ])

                def _vector_between(coord_from: dict, coord_to: dict) -> tuple[float, float]:
                    lat_a = float(coord_from.get("latitude", 0.0))
                    lon_a = float(coord_from.get("longitude", 0.0))
                    lat_b = float(coord_to.get("latitude", lat_a))
                    lon_b = float(coord_to.get("longitude", lon_a))
                    return llh_to_xy(lat_b, lon_b, lat_a, lon_a)

                full_coords_all = pkt.get("fullSweepCoords") or []
                first_sweep_idx = first_wp.get("_sweepIdx")
                def _segment_full_coords(start_idx: int | None, end_idx: int | None) -> list[dict] | None:
                    if not full_coords_all or start_idx is None or end_idx is None:
                        return None
                    start_sweep = max(0, int(start_idx))
                    end_sweep = max(0, int(end_idx))
                    max_sweep_idx = max(0, (len(full_coords_all) // 2) - 1)
                    start_sweep = min(start_sweep, max_sweep_idx)
                    end_sweep = min(end_sweep, max_sweep_idx)
                    if start_sweep <= end_sweep:
                        s = start_sweep * 2
                        e = (end_sweep + 1) * 2
                        if s >= len(full_coords_all) or e <= s:
                            return None
                        return deepcopy(full_coords_all[s:min(e, len(full_coords_all))])

                    # When sweep order is reversed by reference_coord, retain the
                    # original per-sweep endpoint direction and only reverse sweep order.
                    pairs: list[dict] = []
                    for sweep_idx in range(start_sweep, end_sweep - 1, -1):
                        s = sweep_idx * 2
                        e = min(s + 2, len(full_coords_all))
                        if s >= len(full_coords_all) or e <= s:
                            continue
                        pairs.extend(deepcopy(full_coords_all[s:e]))
                    return pairs or None

                if close_takeover_line_entry and takeover_coord is not None:
                    try:
                        rep0 = records[0]
                        rep0_sweep_idx = rep0.get("sweep_idx")
                        rep0_coord = rep0.get("coord") or rep0["wp"].get("coordinate") or {}
                        base_progress_xy = _vector_between(first_coord, rep0_coord)
                        base_route_span_m = math.hypot(base_progress_xy[0], base_progress_xy[1])
                        if base_route_span_m >= 1.0:
                            close_takeover_route_m = max(float(close_takeover_sep_m) * 2.0, 1.0)
                            close_takeover_progress_xy = (
                                (float(base_progress_xy[0]) / float(base_route_span_m)) * float(close_takeover_route_m),
                                (float(base_progress_xy[1]) / float(base_route_span_m)) * float(close_takeover_route_m),
                            )
                        if first_sweep_idx is not None and rep0_sweep_idx is not None and full_coords_all:
                            span_count = max(1, int(rep0_sweep_idx) - int(first_sweep_idx) + 1)
                            meters_per_sweep = base_route_span_m / max(float(span_count), 1.0)
                            target_scan_m = close_takeover_route_m + close_takeover_sep_m
                            sweep_count = max(1, int(math.ceil(target_scan_m / max(meters_per_sweep, 1.0))))
                            max_sweep_idx = max(0, (len(full_coords_all) // 2) - 1)
                            close_takeover_scan_end_idx = min(max_sweep_idx, int(first_sweep_idx) + sweep_count - 1)
                    except Exception:
                        close_takeover_progress_xy = None
                        close_takeover_route_m = 0.0
                        close_takeover_scan_end_idx = None

                    if close_takeover_progress_xy is not None and close_takeover_route_m >= 1.0:
                        tk_lat = float(takeover_coord.get("latitude", 0.0))
                        tk_lon = float(takeover_coord.get("longitude", 0.0))
                        wp2_lat, wp2_lon = xy_to_llh(
                            float(close_takeover_progress_xy[0]),
                            float(close_takeover_progress_xy[1]),
                            tk_lat,
                            tk_lon,
                        )
                        first_wp["coordinate"] = OrderedDict([
                            ("latitude", round(wp2_lat, 6)),
                            ("longitude", round(wp2_lon, 6)),
                            ("altitude", _mission_wp_alt(wp2_lat, wp2_lon)),
                        ])

                    initial_scan_coords = _segment_full_coords(first_sweep_idx, close_takeover_scan_end_idx)
                    if initial_scan_coords:
                        initial_scan_spans = _collect_sweep_spans(initial_scan_coords, 2)
                        initial_width = None
                        if initial_scan_spans:
                            initial_width = round(sum(initial_scan_spans) / len(initial_scan_spans), 2)
                        initial_speed = spacing_based_search_speed(
                            sweep_len_m=initial_width,
                            spacing_m=packet_spacing_line,
                            cruise_speed_mps=packet_cruise_speed,
                            weight=packet_search_speed_weight,
                        )
                        if initial_speed is None:
                            initial_speed = first_search_speed
                        if initial_speed is None:
                            initial_speed = packet_default_search_speed
                        first_wp["filmingProperty"] = _mk_filming(
                            operation_mode=OPMODE_LINE,
                            fov=first_fov,
                            sensor=SENSOR_EO_IR,
                            line_search=OrderedDict([
                                ("coordinateList", _interpolate_line_coords(
                                    initial_scan_coords,
                                    SWEEP_LINE_INTERP_POINTS,
                                )),
                                ("searchSpeed", initial_speed),
                                ("interpolationPoints", SWEEP_LINE_INTERP_POINTS),
                            ]),
                        )
                        if close_takeover_scan_end_idx is not None:
                            kept_records = [
                                record for record in records
                                if record.get("sweep_idx") is None or int(record.get("sweep_idx")) > int(close_takeover_scan_end_idx)
                            ]
                            forced_remove_indices.extend(
                                int(record["idx"]) for record in records
                                if int(record["idx"]) not in {int(kept["idx"]) for kept in kept_records}
                            )
                            records = kept_records
                        if close_takeover_progress_xy is not None and close_takeover_route_m >= 1.0:
                            prog_ux = float(close_takeover_progress_xy[0]) / float(close_takeover_route_m)
                            prog_uy = float(close_takeover_progress_xy[1]) / float(close_takeover_route_m)
                            filtered_records: list[dict] = []
                            for record in records:
                                rec_coord = record.get("coord") or record["wp"].get("coordinate") or {}
                                rec_dx, rec_dy = _vector_between(takeover_coord, rec_coord)
                                rec_progress_m = (rec_dx * prog_ux) + (rec_dy * prog_uy)
                                if rec_progress_m > float(close_takeover_route_m) + 1.0:
                                    filtered_records.append(record)
                            forced_remove_indices.extend(
                                int(record["idx"]) for record in records
                                if int(record["idx"]) not in {int(kept["idx"]) for kept in filtered_records}
                            )
                            records = filtered_records
                elif close_takeover_line_entry:
                    first_wp["filmingProperty"] = _mk_filming(
                        operation_mode=OPMODE_HOLD,
                        fov=first_fov,
                        sensor=SENSOR_EO_IR,
                        gimbal_pitch=ENTRY_HOLD_GIMBAL_PITCH,
                        gimbal_yaw=ENTRY_HOLD_GIMBAL_YAW,
                    )

                groups: list[list[int]] = []
                if merge_mode == "all":
                    groups = [list(range(len(records)))] if records else []
                elif merge_mode == "heading":
                    current_group = [0]
                    for pos in range(1, len(records)):
                        prev_vec = _vector_between(records[pos - 2]["coord"], records[pos - 1]["coord"]) if pos >= 2 else None
                        curr_vec = _vector_between(records[pos - 1]["coord"], records[pos]["coord"])
                        angle = 0.0 if prev_vec is None else _angle_between(prev_vec, curr_vec)
                        if angle <= merge_heading_deg:
                            current_group.append(pos)
                        else:
                            groups.append(current_group)
                            current_group = [pos]
                    groups.append(current_group)
                elif merge_mode == "curve":
                    coords_chain = [first_coord] + [record["coord"] for record in records]
                    signs = [0] * len(coords_chain)
                    for idx in range(1, len(coords_chain) - 1):
                        v1 = _vector_between(coords_chain[idx - 1], coords_chain[idx])
                        v2 = _vector_between(coords_chain[idx], coords_chain[idx + 1])
                        angle = _angle_between(v1, v2)
                        if angle <= merge_heading_deg:
                            signs[idx] = 0
                        else:
                            cross = (v1[0] * v2[1]) - (v1[1] * v2[0])
                            signs[idx] = 1 if cross >= 0 else -1
                    for idx in range(1, len(signs) - 1):
                        if signs[idx] == 0 and signs[idx - 1] == signs[idx + 1] != 0:
                            signs[idx] = signs[idx - 1]
                    current_group = [0]
                    current_sign = signs[1] if len(signs) > 1 else 0
                    for pos in range(1, len(records)):
                        sign = signs[pos + 1]
                        if sign == current_sign:
                            current_group.append(pos)
                        else:
                            groups.append(current_group)
                            current_group = [pos]
                            current_sign = sign
                    groups.append(current_group)
                else:
                    groups = [[idx] for idx in range(len(records))]

                groups = _split_record_groups_by_spacing(
                    records,
                    groups,
                    anchor_coord=first_coord,
                    spacing_m=(
                        AREA_SWEEP_ROUTE_WP_SPACING_M
                        if is_area_pkt
                        else _line_route_wp_spacing_m()
                    ),
                    merge_short_tail=not is_area_pkt,
                )
                to_remove: list[int] = list(dict.fromkeys(forced_remove_indices))
                sweep_step = 1
                if records and first_sweep_idx is not None and records[0].get("sweep_idx") is not None:
                    try:
                        sweep_step = 1 if int(records[0].get("sweep_idx")) >= int(first_sweep_idx) else -1
                    except Exception:
                        sweep_step = 1
                prev_sweep_idx = first_sweep_idx
                if keep_first_line_search and prev_sweep_idx is not None:
                    prev_sweep_idx = int(prev_sweep_idx) + int(sweep_step)
                if close_takeover_line_entry and close_takeover_scan_end_idx is not None:
                    prev_sweep_idx = close_takeover_scan_end_idx
                for g_idx, group in enumerate(groups):
                    rep_pos = group[-1]
                    rep = records[rep_pos]
                    rep_fp = rep["fp"]
                    merged_coords: list[dict] = []
                    merged_spans: list[float] = []
                    used_segment_coords = False
                    start_sweep_idx = prev_sweep_idx
                    if g_idx > 0 and start_sweep_idx is not None:
                        start_sweep_idx = int(start_sweep_idx) + int(sweep_step)
                    elif close_takeover_line_entry and start_sweep_idx is not None:
                        start_sweep_idx = int(start_sweep_idx) + int(sweep_step)
                    segment_coords = _segment_full_coords(start_sweep_idx, rep.get("sweep_idx"))
                    if segment_coords:
                        merged_coords.extend(segment_coords)
                        merged_spans.extend(_collect_sweep_spans(segment_coords, 2))
                        used_segment_coords = True
                    elif g_idx == 0 and not keep_first_line_search:
                        merged_coords.extend(deepcopy(first_coords))
                        merged_spans.extend(_collect_sweep_spans(first_coords, first_interp_points))
                    for pos in group:
                        if used_segment_coords:
                            continue
                        coords_copy = deepcopy(records[pos]["coords"])
                        merged_coords.extend(coords_copy)
                        merged_spans.extend(_collect_sweep_spans(
                            records[pos]["coords"],
                            records[pos].get("interp_points"),
                        ))
                    merged_width = None
                    if merged_spans:
                        merged_width = round(sum(merged_spans) / len(merged_spans), 2)
                    rep_speed = spacing_based_search_speed(
                        sweep_len_m=merged_width,
                        spacing_m=packet_spacing_line,
                        cruise_speed_mps=packet_cruise_speed,
                        weight=packet_search_speed_weight,
                    )
                    if rep_speed is None:
                        rep_speed = rep["search_speed"]
                    if rep_speed is None:
                        rep_speed = first_search_speed
                    rep_fp["fieldOfView"] = rep["fov"]
                    rep_fp["sensorType"] = SENSOR_EO_IR
                    rep_fp["operationMode"] = OPMODE_LINE
                    interpolated_coords = _interpolate_line_coords(
                        merged_coords,
                        SWEEP_LINE_INTERP_POINTS,
                    )
                    rep_fp["lineSearch"] = OrderedDict([
                        ("coordinateList", interpolated_coords),
                        ("searchSpeed", rep_speed),
                        ("interpolationPoints", SWEEP_LINE_INTERP_POINTS),
                    ])
                    if uav_plan_mode == "dub_path" and is_area_pkt and g_idx == len(groups) - 1:
                        rep["wp"]["_skip_path_search_speed_recalc"] = True
                        rep["wp"]["waypointPassType"] = PASS_FLYOVER
                    for pos in group:
                        if pos != rep_pos:
                            to_remove.append(records[pos]["idx"])
                    prev_sweep_idx = rep.get("sweep_idx", prev_sweep_idx)

                for idx in sorted(to_remove, reverse=True):
                    del wps[idx]

            if entry_wp is not None:
                wps.insert(0, entry_wp)

            if not is_area_pkt:
                line_positions_after = [idx for idx, wp in enumerate(wps) if _has_line_search(wp)]
                if line_positions_after:
                    ref_coord = None
                    first_line_after = line_positions_after[0]
                    if first_line_after > 0:
                        ref_coord = _wp_effective_end_coord(wps[first_line_after - 1])
                    if not isinstance(ref_coord, dict) or not ref_coord:
                        ref_coord = reference_coord if isinstance(reference_coord, dict) else None
                    _reorder_line_search_subset_inplace(
                        wps,
                        reference_coord=ref_coord,
                    )

        for wp in wps:
            if "_sweepIdx" in wp:
                del wp["_sweepIdx"]
            if "_path_search_speed_weight" in wp:
                del wp["_path_search_speed_weight"]

        if is_area_pkt:
            is_area_nadir = bool(pkt.get("_area_nadir"))
            area_offset_sign: int | None = None
            if not is_area_nadir:
                prev_area_sign = prev_area_offset_sign_by_aircraft.get(aid)
                if prev_area_sign in (-1, 1):
                    area_offset_sign = -int(prev_area_sign)
                else:
                    first_area_line_coords: list[dict] = []
                    for area_wp in wps:
                        area_fp = area_wp.get("filmingProperty") or {}
                        area_ls = area_fp.get("lineSearch") or {}
                        first_area_line_coords = area_ls.get("coordinateList") or []
                        if first_area_line_coords:
                            break
                    area_offset_sign = _infer_area_offset_sign_from_sweep(
                        first_area_line_coords,
                        bearing_deg=pkt.get("_area_bearing_deg"),
                        offset_m=float(pkt.get("_mission_sep_m", 0.0) or 0.0) * float(AREA_ROUTE_OFFSET_SCALE),
                        reference_coord=(
                            prev_last_wp_coord
                            if isinstance(prev_last_wp_coord, dict)
                            else (
                                prev_tail_coord
                                if isinstance(prev_tail_coord, dict)
                                else (
                                    pkt.get("_prev_point")
                                    if isinstance(pkt.get("_prev_point"), dict)
                                    else (
                                        pkt.get("_entry_anchor")
                                        if isinstance(pkt.get("_entry_anchor"), dict)
                                        else None
                                    )
                                )
                            )
                        ),
                    )
            if not is_area_nadir:
                # Preserve the raw ordered area sweep WPs.
                # Area chunk packing below must operate on the aligned/raw route chain
                # rather than on angle/min-spacing simplified representatives.
                pass

            # Area missions: align entry waypoint to the final first leg direction
            # (after area merge/simplify), not to pre-merge sweep geometry.
            if wps and wps[0].get("_flyover_entry_offset"):
                wps.pop(0)
            try:
                area_sep_offset_m = float(pkt.get("_mission_sep_m", 0.0) or 0.0) * float(AREA_ROUTE_OFFSET_SCALE)
            except Exception:
                area_sep_offset_m = 0.0
            area_reference_coord = pkt.get("_prev_point") if isinstance(pkt.get("_prev_point"), dict) else None
            if not isinstance(area_reference_coord, dict):
                area_reference_coord = prev_last_wp_coord if isinstance(prev_last_wp_coord, dict) else None
            if not isinstance(area_reference_coord, dict):
                area_reference_coord = pkt.get("_entry_anchor") if isinstance(pkt.get("_entry_anchor"), dict) else None
            _reposition_area_wps_by_sep(
                wps,
                offset_m=area_sep_offset_m,
                altitude_fn=_mission_wp_alt,
                bearing_deg=pkt.get("_area_bearing_deg"),
                reference_coord=area_reference_coord,
                offset_sign=area_offset_sign,
            )
            if not is_area_nadir:
                is_first_area_packet = bool(pkt.get("_skip_area_entry_prefix"))
                wps[:] = _pack_area_wps_by_spacing(
                    wps,
                    route_spacing_m=AREA_SWEEP_ROUTE_WP_SPACING_M,
                    spacing_line_m=packet_spacing_line,
                    cruise_speed_mps=packet_cruise_speed,
                    search_speed_weight=packet_search_speed_weight,
                    default_search_speed=packet_default_search_speed,
                    post_entry_search_speed_scale=(
                        AREA_FIRST_PACKET_SEARCH_SPEED_SCALE
                        if is_first_area_packet
                        else 1.0
                    ),
                    post_entry_route_spacing_scale=(
                        AREA_FIRST_PACKET_SWEEP_GROUP_SCALE
                        if is_first_area_packet
                        else 1.0
                    ),
                )
            if not is_area_nadir and area_offset_sign in (-1, 1):
                prev_area_offset_sign_by_aircraft[aid] = int(area_offset_sign)
        pkt.pop("_is_area", None)
        pkt.pop("_area_nadir", None)
        pkt.pop("_area_sweep_mode", None)
        pkt.pop("_area_bearing_deg", None)
        pkt.pop("_mission_sep_m", None)
        pkt.pop("_mission_search_speed_weight", None)
        pkt.pop("_entry_anchor", None)
        pkt.pop("_entry_offset_m", None)
        pkt.pop("_entry_disabled", None)
        pkt.pop("_skip_area_entry_prefix", None)
        pkt.pop("_prev_point", None)

        if prev_tail_coord and wps:
            def _same_coord(a: dict, b: dict, tol_m: float = 1.0) -> bool:
                if not a or not b:
                    return False
                if "latitude" not in a or "longitude" not in a:
                    return False
                if "latitude" not in b or "longitude" not in b:
                    return False
                return _dist_between_coords(a, b) <= tol_m

            while wps and _same_coord(prev_tail_coord, wps[0].get("coordinate") or {}):
                wps.pop(0)

            if is_area_pkt and not is_area_nadir:
                try:
                    area_entry_min_gap_m = max(120.0, float(pkt.get("_mission_sep_m", 0.0) or 0.0) * 0.1)
                except Exception:
                    area_entry_min_gap_m = 120.0
                _drop_redundant_close_area_entry_anchor_inplace(
                    prev_tail_coord,
                    wps,
                    min_gap_m=area_entry_min_gap_m,
                )

        if not wps:
            continue

        _apply_segment_altitude_to_search_waypoints(
            wps,
            altitude_offset_m=float(pkt.get("_mission_alt_offset_m", Altitude) or Altitude),
            fallback_ground_ref_m=_safe_float_or_none(pkt.get("_mission_ground_ref_m")),
        )
        _align_point_anchor_altitude_with_search_waypoints(wps)

        for wp in wps:
            if wp.get("waypointPassType") == PASS_LOITER:
                wp["_was_loiter"] = True

        if FLYOVER_ALL_WPS:
            for wp in wps:
                wp["waypointPassType"] = PASS_FLYOVER
        else:
            if FLYOVER_ENTRY_OFFSET:
                for wp in wps:
                    if wp.get("_flyover_entry_offset"):
                        wp["waypointPassType"] = PASS_FLYOVER
            if FLYOVER_DUBINS_PREFIX:
                for wp in wps:
                    if wp.get("_flyover_dubins_prefix"):
                        wp["waypointPassType"] = PASS_FLYOVER

        for wp in wps:
            if "_flyover_entry_offset" in wp:
                del wp["_flyover_entry_offset"]
            if "_flyover_dubins_prefix" in wp:
                del wp["_flyover_dubins_prefix"]
            if "_area_dubins_link" in wp:
                del wp["_area_dubins_link"]

        for wp in wps:
            was_loiter = wp.get("_was_loiter")
            is_loiter = wp.get("waypointPassType") == PASS_LOITER
            if not is_loiter and FLYOVER_ALL_WPS and was_loiter:
                is_loiter = True
            if is_loiter:
                if not wp.get("filmingProperty"):
                    wp["filmingProperty"] = _mk_point_filming_for_coord(wp.get("coordinate") or {})
                wp["loiterProperty"] = OrderedDict([
                    ("radius", LOITER_RADIUS_M),
                    ("direction", LOITER_DIRECTION),
                    ("time", LOITER_TIME_S),
                    ("speed", LOITER_SPEED_MPS),
                ])
            if "_was_loiter" in wp:
                del wp["_was_loiter"]

        _recompute_eta_ecf_inplace(wps, default_speed_mps=packet_cruise_speed)
        if is_area_pkt:
            _scale_output_fov_inplace(wps, scale=float(AREA_OUTPUT_FOV_SCALE))

        for wp in wps:
            if "_skip_path_search_speed_recalc" in wp:
                del wp["_skip_path_search_speed_recalc"]

        if is_area_pkt and not is_area_nadir:
            prev_area_pkt = prev_area_pkt_by_aircraft.get(aid)
            if isinstance(prev_area_pkt, dict):
                _append_area_transition_links_inplace(
                    prev_area_pkt,
                    pkt,
                    altitude_fn=_mission_wp_alt,
                )
            prev_area_pkt_by_aircraft[aid] = pkt
        else:
            prev_area_pkt_by_aircraft.pop(aid, None)

        last_coord = _wp_effective_end_coord(wps[-1])
        if last_coord:
            prev_tail_by_aircraft[aid] = dict(last_coord)
            last_wp_coord = wps[-1].get("coordinate") or {}
            if isinstance(last_wp_coord, dict) and last_wp_coord:
                prev_last_wp_coord_by_aircraft[aid] = dict(last_wp_coord)
            heading_from = None
            last_fp = wps[-1].get("filmingProperty") or {}
            last_ls = last_fp.get("lineSearch") or {}
            last_ls_coords = last_ls.get("coordinateList") or []
            if len(last_ls_coords) >= 2:
                heading_from = last_ls_coords[-2]
            elif len(wps) >= 2:
                heading_from = _wp_effective_end_coord(wps[-2])
            if isinstance(heading_from, dict) and heading_from:
                prev_heading_by_aircraft[aid] = _heading_rad_between_coords(
                    heading_from,
                    last_coord,
                )
            else:
                prev_heading_by_aircraft.pop(aid, None)

    if formation_followers:
        leader_wplist_by_key: dict[int, list[OrderedDict]] = {}
        for pkt in packets:
            fkey = pkt.get("_formation_key")
            if fkey is None:
                continue
            leader_id = formation_leaders.get(fkey)
            if leader_id is None:
                continue
            if int(pkt.get("aircraftID", 0)) == int(leader_id) and pkt.get("wplist"):
                leader_wplist_by_key[int(fkey)] = pkt["wplist"]
        for fkey, fpkt in formation_followers:
            leader_wplist = leader_wplist_by_key.get(int(fkey))
            if leader_wplist:
                fpkt["wplist"] = deepcopy(leader_wplist)

    if getattr(wp_alloc, "_use_global", False):
        total_wp_count = sum(len(pkt.get("wplist") or []) for pkt in packets)
        if total_wp_count > 0:
            wp_alloc = _WPAllocator(start=int(_reserve_waypoint_block(total_wp_count)))

    for pkt in packets:
        wps = pkt.get("wplist") or []
        if not wps:
            continue
        for wp in wps:
            wp["waypointID"] = wp_alloc.alloc()
        for idx in range(len(wps) - 1):
            wps[idx]["nextWaypointID"] = wps[idx + 1]["waypointID"]
        wps[-1]["nextWaypointID"] = 0

    # ────────────────────────── 4) 최종 조립 ─────────────────────────
    result = []
    for pkt in packets:
        pkt.pop("_area_sweep_mode", None)
        pkt.pop("_area_bearing_deg", None)
        pkt.pop("_mission_sep_m", None)
        pkt.pop("_mission_cruise_speed_mps", None)
        pkt.pop("_mission_default_search_speed", None)
        pkt.pop("_mission_spacing_line_m", None)
        pkt.pop("_mission_alt_offset_m", None)
        pkt.pop("_mission_ground_ref_m", None)
        pkt.pop("_entry_anchor", None)
        pkt.pop("_entry_offset_m", None)
        formation_info = pkt.get("formationInfo")
        items = [
            ("timestamp", now_ms),
            ("Source", _sw_code()),
            ("pathID", pkt["pathID"]),
            ("aircraftID", pkt["aircraftID"]),
            ("isFormationFlight", bool(pkt.get("isFormationFlight", False))),
        ]
        if formation_info:
            items.append(("formationInfo", formation_info))
        items.append(("waypointList", pkt["wplist"]))
        result.append(OrderedDict(items))
    return result




def _runtime_area_sweep_mode() -> str:
    if _get_runtime_str is None:
        return "parallel"
    try:
        raw = str(_get_runtime_str("area_sweep_mode", "parallel") or "parallel").strip().lower()
    except Exception:
        return "parallel"
    if raw in {"vertical", "ver", "perpendicular", "orthogonal"}:
        return "vertical"
    if raw in {"nadir", "directdown", "bf_nadir"}:
        return "nadir"
    return "parallel"


def _runtime_uav_plan_mode() -> str:
    if _get_runtime_str is None:
        return "normal"
    try:
        raw = str(_get_runtime_str("uav_plan_mode", "normal") or "normal").strip().lower()
    except Exception:
        return "normal"
    if raw == "dub_path":
        return "dub_path"
    return "normal"
