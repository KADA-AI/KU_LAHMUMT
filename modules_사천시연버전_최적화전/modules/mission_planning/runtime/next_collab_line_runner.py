from __future__ import annotations

import copy
import math
from contextlib import contextmanager
from dataclasses import dataclass
from types import MethodType
from typing import Any, Callable, Dict, List, Optional, Sequence, Tuple

from PyQt5.QtWidgets import QMessageBox
from shapely.geometry import LineString, Point, Polygon

from modules.mission_planning.runtime.next_collab_heading import (
    monitor_heading_to_planner_bearing_deg,
)
try:
    from modules.mission_planning.MissionPlanner.data_def.dubins_turn_link import (
        Pose2D as _DubinsPose2D,
        dubins_shortest_path as _dubins_shortest_path,
    )
except Exception:
    _DubinsPose2D = None
    _dubins_shortest_path = None
from modules.mission_planning.MissionPlanner.planning_enhanced.algo.split_runner import (
    run_split_pipeline,
)
from modules.mission_planning.MissionPlanner.planning_enhanced.models import (
    SplitPiece,
    SplitRunResult,
)
from modules.mission_planning.MissionPlanner.config import SWEEP_SPACING_MARGIN
from modules.mission_planning.MissionPlanner.runtime_settings import (
    MIN_LINE_FOV_DEG,
    apply_runtime_camera_adjusted_fov_deg,
    apply_runtime_camera_adjusted_search_speed,
    format_runtime_camera_fov_adjustment_log,
    get_runtime_bool,
    get_runtime_float,
    load_runtime_settings,
)
from modules.mission_planning.next_area_mode.config import (
    MISSION_LINE,
    MODE_MISSION_READY,
    TURN_PREVIEW_HORIZON_S,
    TURN_PREVIEW_RADIUS_M,
    TURN_PREVIEW_SPEED_MPS,
)
from modules.mission_planning.next_area_mode.planner_window import (
    CanvasState,
    NextAreaPlanningWindow,
    coord_to_xy,
    coords_to_xy,
)


@dataclass
class NextCollabLinePlanResult:
    workflow: str
    split_result: SplitRunResult
    mid_line_segments: List[Dict[str, Any]]
    expected_paths: List[Dict[str, Any]]
    planner_result_text: str


LINE_ROUTE_WP_SPACING_M = 1200.0
LINE_SWEEP_DENSITY_SCALE = 1.2
LINE_ROUTE_OFFSET_SCALE = 1.0
NEXT_COLLAB_ENTRY_TPRIME_TARGET_SEP_RATIO = 0.30
NEXT_COLLAB_ENTRY_TPRIME_RATIO_SCALE = 0.50
NEXT_COLLAB_LINE_DB_WIDTH_WEIGHT = 0.30
NEXT_COLLAB_LINE_DB_SEP_WEIGHT = 0.25
NEXT_COLLAB_LINE_DB_FOV_WEIGHT = 0.45
FOV_DB_SEP_SAFETY_FACTOR = 1.7
LINE_OFFSET_CANDIDATE_SCALES = (0.45, 0.65, 0.85, 1.0)


@dataclass(frozen=True)
class _LineOffsetPlan:
    start_side: float
    end_side: float
    offset_m: float
    score: float
    dubins_length_m: float | None = None
    first_turn_sign: int = 0
    clearance_m: float | None = None

    def side_at(self, progress_ratio: float) -> float:
        t = max(0.0, min(1.0, float(progress_ratio)))
        return float(self.start_side) + ((float(self.end_side) - float(self.start_side)) * t)


class _HeadlessLinePlanner:
    def __init__(self) -> None:
        self.state = CanvasState(line_width_m=300.0)
        self._selected_uav_count = 1
        self._cmpk_payload = None
        self._mrpk_payload = None
        self._fov_db_rows_cache = None
        self._fov_db_widths_cache = None
        self.stage2_ratio_spins = {}
        self._result_text_buffer = ""
        self.canvas = None

    def hide(self) -> None:
        return

    def close(self) -> None:
        return

    def _sync_canvas(self) -> None:
        return

    def _refresh_ui(self) -> None:
        return

    def _append_result(self, text: str) -> None:
        existing = str(getattr(self, "_result_text_buffer", "") or "")
        self._result_text_buffer = f"{existing}\n{text}".strip()

    def _set_result(self, text: str) -> None:
        self._result_text_buffer = str(text or "")


for _name, _value in NextAreaPlanningWindow.__dict__.items():
    if _name.startswith("__"):
        continue
    if hasattr(_HeadlessLinePlanner, _name):
        continue
    if callable(_value):
        setattr(_HeadlessLinePlanner, _name, _value)


def _distance_xy(start_xy: Tuple[float, float], end_xy: Tuple[float, float]) -> float:
    dx = float(end_xy[0]) - float(start_xy[0])
    dy = float(end_xy[1]) - float(start_xy[1])
    return math.hypot(dx, dy)


def _runtime_fov_db_sep_safety_factor(runtime_cfg: Dict[str, Any] | None = None) -> float:
    factor = float(get_runtime_float("fov_db_sep_safety_factor", FOV_DB_SEP_SAFETY_FACTOR, runtime_cfg))
    if factor <= 0.0:
        factor = 1.0
    return float(factor)


def _db_sep_requirement_m(sep_m: float, runtime_cfg: Dict[str, Any] | None = None) -> float:
    factor = _runtime_fov_db_sep_safety_factor(runtime_cfg)
    return max(0.0, float(sep_m or 0.0)) * float(factor)


def _runtime_line_density_scale(runtime_cfg: Dict[str, Any] | None = None) -> float:
    value = float(get_runtime_float("line_density_scale", LINE_SWEEP_DENSITY_SCALE, runtime_cfg))
    return max(value, 1e-6)


def _runtime_line_search_speed_weight(runtime_cfg: Dict[str, Any] | None = None) -> float:
    value = float(get_runtime_float("search_speed_weight", 1.0, runtime_cfg))
    return max(value, 0.1)


def _runtime_line_route_offset_scale(runtime_cfg: Dict[str, Any] | None = None) -> float:
    value = float(get_runtime_float("line_route_offset_scale", LINE_ROUTE_OFFSET_SCALE, runtime_cfg))
    return max(value, 0.0)


def _fov_db_min_sep_for_fov(
    planner: _HeadlessLinePlanner,
    fov_deg: float,
    *,
    runtime_cfg: Dict[str, Any] | None = None,
) -> float:
    try:
        target_fov = float(fov_deg)
    except Exception:
        return 0.0
    if target_fov <= 0.0:
        return 0.0
    rows = planner._fov_db_rows() or []
    if not rows:
        return 0.0

    matches: List[float] = []
    for row in rows:
        try:
            row_fov = float(row.get("fov", 0.0) or 0.0)
            row_sep = float(row.get("sep", 0.0) or 0.0)
        except Exception:
            continue
        if row_fov <= 0.0 or row_sep <= 0.0:
            continue
        if abs(row_fov - target_fov) <= 0.05:
            matches.append(float(row_sep))

    if not matches:
        for row in rows:
            try:
                row_fov = float(row.get("fov", 0.0) or 0.0)
                row_sep = float(row.get("sep", 0.0) or 0.0)
            except Exception:
                continue
            if row_fov <= 0.0 or row_sep <= 0.0:
                continue
            try:
                adjusted_fov = float(
                    apply_runtime_camera_adjusted_fov_deg(
                        row_fov,
                        runtime_cfg,
                        minimum_fov_deg=MIN_LINE_FOV_DEG,
                        context="NEXTCOLLAB LINE OFFSET_DB",
                    )
                )
            except Exception:
                adjusted_fov = row_fov
            if abs(adjusted_fov - target_fov) <= 0.05:
                matches.append(float(row_sep))

    return min(matches) if matches else 0.0


def _route_offset_sep_for_fov(
    planner: _HeadlessLinePlanner,
    fov_deg: float,
    default_sep_m: float,
    *,
    runtime_cfg: Dict[str, Any] | None = None,
) -> float:
    try:
        default_sep = float(default_sep_m)
    except Exception:
        default_sep = 0.0
    min_sep = _fov_db_min_sep_for_fov(planner, float(fov_deg or 0.0), runtime_cfg=runtime_cfg)
    if min_sep > 0.0:
        return float(min_sep)
    return max(float(default_sep), 0.0)


def _runtime_line_route_wp_spacing_m(runtime_cfg: Dict[str, Any] | None = None) -> float:
    value = float(get_runtime_float("uav_wp_interval_m", LINE_ROUTE_WP_SPACING_M, runtime_cfg))
    return max(value, 1.0)


def _sweep_spacing_margin_for_density() -> float:
    try:
        margin = float(SWEEP_SPACING_MARGIN)
    except Exception:
        margin = 1.0
    if margin <= 0.0:
        margin = 1.0
    return max(margin, 1e-6)


def _sweep_spacing_m(*, separation_m: float, fov_deg: float, spacing_scale: float = 1.0) -> float:
    base = 2.0 * max(float(separation_m), 1.0) * math.tan(max(math.radians(float(fov_deg)) / 2.0, 1e-6))
    try:
        effective_margin = float(SWEEP_SPACING_MARGIN)
    except Exception:
        effective_margin = 1.0
    if effective_margin <= 0.0:
        effective_margin = 1.0
    try:
        effective_scale = float(spacing_scale)
    except Exception:
        effective_scale = 1.0
    if effective_scale <= 0.0:
        effective_scale = 1.0
    return max(base * effective_margin * effective_scale, 1.0)


def _line_raw_spacing_scale(runtime_cfg: Dict[str, Any] | None = None) -> float:
    density = _runtime_line_density_scale(runtime_cfg)
    sep_safety = _runtime_fov_db_sep_safety_factor(runtime_cfg)
    if sep_safety <= 0.0:
        sep_safety = 1.0
    return max(1.0 / (density * sep_safety * _sweep_spacing_margin_for_density()), 1e-6)


def _line_sweep_spacing_m(
    *,
    separation_m: float,
    fov_deg: float,
    runtime_cfg: Dict[str, Any] | None = None,
) -> float:
    return _sweep_spacing_m(
        separation_m=float(separation_m),
        fov_deg=float(fov_deg),
        spacing_scale=_line_raw_spacing_scale(runtime_cfg),
    )


def _runtime_line_manual_base_fov_deg(runtime_cfg: Dict[str, Any] | None = None) -> float | None:
    auto_from_db = bool(get_runtime_bool("enhanced_auto_fov_from_db", True, runtime_cfg))
    if auto_from_db:
        return None
    manual_fov_deg = _to_float(
        get_runtime_float(
            "line_custom_fov_deg",
            0.0,
            runtime_cfg,
        )
    )
    if manual_fov_deg is None or manual_fov_deg <= 0.0:
        return None
    return _clamp_line_fov_deg(manual_fov_deg)


def _runtime_line_manual_fov_deg(runtime_cfg: Dict[str, Any] | None = None) -> float | None:
    manual_fov_deg = _runtime_line_manual_base_fov_deg(runtime_cfg)
    if manual_fov_deg is None:
        return None
    adjusted_fov_deg = apply_runtime_camera_adjusted_fov_deg(
        manual_fov_deg,
        runtime_cfg,
        minimum_fov_deg=MIN_LINE_FOV_DEG,
        context="NEXTCOLLAB LINE MANUAL",
    )
    return _clamp_line_fov_deg(adjusted_fov_deg) if adjusted_fov_deg > 0.0 else None


def _bearing_deg_from_xy(start_xy: Tuple[float, float], end_xy: Tuple[float, float]) -> float:
    dx = float(end_xy[0]) - float(start_xy[0])
    dy = float(end_xy[1]) - float(start_xy[1])
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return 0.0
    return float((math.degrees(math.atan2(dx, dy)) + 360.0) % 360.0)


def _bearing_unit_xy(bearing_deg: float) -> Tuple[float, float]:
    theta = math.radians(float(bearing_deg) % 360.0)
    return math.sin(theta), math.cos(theta)


def _bearing_to_yaw_rad(bearing_deg: float) -> float:
    return math.radians(90.0 - (float(bearing_deg) % 360.0))


def _angle_delta_deg(left_deg: float, right_deg: float) -> float:
    return abs(((float(left_deg) - float(right_deg) + 180.0) % 360.0) - 180.0)


def _line_route_frame_xy(
    route_line_xy: Sequence[Tuple[float, float]],
) -> tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float], Tuple[float, float]] | None:
    if len(route_line_xy) < 2:
        return None
    start_xy = (float(route_line_xy[0][0]), float(route_line_xy[0][1]))
    end_xy = (float(route_line_xy[-1][0]), float(route_line_xy[-1][1]))
    dx = float(end_xy[0]) - float(start_xy[0])
    dy = float(end_xy[1]) - float(start_xy[1])
    norm = math.hypot(dx, dy)
    if norm <= 1e-6:
        return None
    unit_xy = (dx / norm, dy / norm)
    normal_xy = (-unit_xy[1], unit_xy[0])
    return start_xy, end_xy, unit_xy, normal_xy


def _project_to_route_ratio_xy(
    point_xy: Tuple[float, float],
    route_start_xy: Tuple[float, float],
    route_end_xy: Tuple[float, float],
) -> tuple[Tuple[float, float], float]:
    sx, sy = float(route_start_xy[0]), float(route_start_xy[1])
    ex, ey = float(route_end_xy[0]), float(route_end_xy[1])
    px, py = float(point_xy[0]), float(point_xy[1])
    dx = ex - sx
    dy = ey - sy
    denom = (dx * dx) + (dy * dy)
    if denom <= 1e-9:
        return (sx, sy), 0.0
    t = (((px - sx) * dx) + ((py - sy) * dy)) / denom
    t = max(0.0, min(1.0, float(t)))
    return (sx + (dx * t), sy + (dy * t)), float(t)


def _route_polyline_rows_xy(
    route_line_xy: Sequence[Tuple[float, float]],
) -> List[Tuple[float, float]]:
    rows: List[Tuple[float, float]] = []
    for row in route_line_xy or []:
        if not isinstance(row, (tuple, list)) or len(row) < 2:
            continue
        try:
            point_xy = (float(row[0]), float(row[1]))
        except Exception:
            continue
        if rows and _distance_xy(rows[-1], point_xy) <= 0.5:
            continue
        rows.append(point_xy)
    return rows


def _project_to_route_polyline_xy(
    point_xy: Tuple[float, float],
    route_line_xy: Sequence[Tuple[float, float]],
) -> tuple[Tuple[float, float], Tuple[float, float], float] | None:
    rows = _route_polyline_rows_xy(route_line_xy)
    if len(rows) < 2:
        return None
    total_len_m = 0.0
    for idx in range(len(rows) - 1):
        total_len_m += _distance_xy(rows[idx], rows[idx + 1])
    if total_len_m <= 1e-6:
        return None

    px, py = float(point_xy[0]), float(point_xy[1])
    walked_m = 0.0
    best: tuple[float, Tuple[float, float], Tuple[float, float], float] | None = None
    for idx in range(len(rows) - 1):
        sx, sy = float(rows[idx][0]), float(rows[idx][1])
        ex, ey = float(rows[idx + 1][0]), float(rows[idx + 1][1])
        dx = ex - sx
        dy = ey - sy
        seg_len_m = math.hypot(dx, dy)
        if seg_len_m <= 1e-6:
            continue
        denom = (dx * dx) + (dy * dy)
        ratio = (((px - sx) * dx) + ((py - sy) * dy)) / max(denom, 1e-9)
        ratio = max(0.0, min(1.0, float(ratio)))
        center_xy = (sx + (dx * ratio), sy + (dy * ratio))
        tangent_xy = (dx / seg_len_m, dy / seg_len_m)
        progress_ratio = max(0.0, min(1.0, (walked_m + (seg_len_m * ratio)) / total_len_m))
        dist_m = _distance_xy((px, py), center_xy)
        if best is None or dist_m < best[0]:
            best = (float(dist_m), center_xy, tangent_xy, float(progress_ratio))
        walked_m += seg_len_m
    if best is None:
        return None
    return best[1], best[2], best[3]


def _line_anchor_from_route_plan_xy(
    sweep_xy: Sequence[Tuple[float, float]],
    *,
    route_line_xy: Sequence[Tuple[float, float]],
    offset_plan: _LineOffsetPlan,
) -> Tuple[float, float] | None:
    if len(sweep_xy) < 2:
        return None
    midpoint_xy = _midpoint_xy(sweep_xy)
    if midpoint_xy is None:
        return None
    projected = _project_to_route_polyline_xy(midpoint_xy, route_line_xy)
    if projected is not None:
        center_xy, tangent_xy, progress_ratio = projected
        normal_xy = (-float(tangent_xy[1]), float(tangent_xy[0]))
        signed_offset_m = float(offset_plan.offset_m) * float(offset_plan.side_at(progress_ratio))
        return (
            float(center_xy[0]) + (float(normal_xy[0]) * signed_offset_m),
            float(center_xy[1]) + (float(normal_xy[1]) * signed_offset_m),
        )

    frame = _line_route_frame_xy(route_line_xy)
    if frame is None:
        return None
    route_start_xy, route_end_xy, _unit_xy, normal_xy = frame
    center_xy, progress_ratio = _project_to_route_ratio_xy(midpoint_xy, route_start_xy, route_end_xy)
    signed_offset_m = float(offset_plan.offset_m) * float(offset_plan.side_at(progress_ratio))
    return (
        float(center_xy[0]) + (float(normal_xy[0]) * signed_offset_m),
        float(center_xy[1]) + (float(normal_xy[1]) * signed_offset_m),
    )


def _line_anchor_clearance_m(
    anchor_xy: Tuple[float, float] | None,
    sweep_xy: Sequence[Tuple[float, float]],
) -> float | None:
    if anchor_xy is None or len(sweep_xy) < 2:
        return None
    endpoints = [
        (float(sweep_xy[0][0]), float(sweep_xy[0][1])),
        (float(sweep_xy[-1][0]), float(sweep_xy[-1][1])),
    ]
    return max(_distance_xy(anchor_xy, endpoint_xy) for endpoint_xy in endpoints)


def _route_endpoint_offset_xy(
    route_line_xy: Sequence[Tuple[float, float]],
    *,
    side: float,
    offset_m: float,
    at_end: bool,
) -> Tuple[float, float] | None:
    rows = _route_polyline_rows_xy(route_line_xy)
    if len(rows) < 2:
        return None
    if at_end:
        endpoint_xy = rows[-1]
        tangent_start_xy = rows[-2]
        tangent_end_xy = rows[-1]
    else:
        endpoint_xy = rows[0]
        tangent_start_xy = rows[0]
        tangent_end_xy = rows[1]
    dx = float(tangent_end_xy[0]) - float(tangent_start_xy[0])
    dy = float(tangent_end_xy[1]) - float(tangent_start_xy[1])
    norm = math.hypot(dx, dy)
    if norm <= 1e-6:
        return None
    tangent_xy = (dx / norm, dy / norm)
    normal_xy = (-float(tangent_xy[1]), float(tangent_xy[0]))
    return (
        float(endpoint_xy[0]) + (normal_xy[0] * float(offset_m) * float(side)),
        float(endpoint_xy[1]) + (normal_xy[1] * float(offset_m) * float(side)),
    )


def _turn_sign_toward_xy(
    *,
    origin_xy: Tuple[float, float],
    heading_deg: float,
    target_xy: Tuple[float, float],
) -> int:
    hx, hy = _bearing_unit_xy(float(heading_deg))
    vx = float(target_xy[0]) - float(origin_xy[0])
    vy = float(target_xy[1]) - float(origin_xy[1])
    norm = math.hypot(vx, vy)
    if norm <= 1e-6:
        return 0
    cross = (hx * (vy / norm)) - (hy * (vx / norm))
    if cross > 0.05:
        return 1
    if cross < -0.05:
        return -1
    return 0


def _first_float(*values: Any) -> float | None:
    for value in values:
        parsed = _to_float(value)
        if parsed is not None and math.isfinite(float(parsed)):
            return float(parsed)
    return None


def _first_xy_from_aircraft_row(row: Dict[str, Any] | None) -> Tuple[float, float] | None:
    if not isinstance(row, dict):
        return None
    for key in ("currentPositionXY", "positionXY"):
        value = row.get(key)
        if isinstance(value, (tuple, list)) and len(value) >= 2:
            try:
                return (float(value[0]), float(value[1]))
            except Exception:
                continue
    return None


def _dubins_length_m(
    *,
    start_xy: Tuple[float, float],
    start_heading_deg: float,
    end_xy: Tuple[float, float],
    end_heading_deg: float,
    radius_m: float | None,
) -> float | None:
    if _DubinsPose2D is None or _dubins_shortest_path is None:
        return None
    if radius_m is None or not math.isfinite(float(radius_m)) or float(radius_m) <= 1.0:
        return None
    try:
        result = _dubins_shortest_path(
            _DubinsPose2D(float(start_xy[0]), float(start_xy[1]), _bearing_to_yaw_rad(start_heading_deg)),
            _DubinsPose2D(float(end_xy[0]), float(end_xy[1]), _bearing_to_yaw_rad(end_heading_deg)),
            float(radius_m),
            allow_ccc=False,
        )
        return float(result.get("length", 0.0) or 0.0)
    except Exception:
        return None


def _score_line_offset_plan(
    *,
    route_start_xy: Tuple[float, float],
    route_end_xy: Tuple[float, float],
    normal_xy: Tuple[float, float],
    start_side: float,
    end_side: float,
    offset_m: float,
    aircraft_row: Dict[str, Any] | None,
    fallback_origin_xy: Tuple[float, float] | None,
    start_anchor_xy: Tuple[float, float] | None = None,
    end_anchor_xy: Tuple[float, float] | None = None,
) -> _LineOffsetPlan:
    if start_anchor_xy is None:
        start_anchor_xy = (
            float(route_start_xy[0]) + (float(normal_xy[0]) * float(offset_m) * float(start_side)),
            float(route_start_xy[1]) + (float(normal_xy[1]) * float(offset_m) * float(start_side)),
        )
    if end_anchor_xy is None:
        end_anchor_xy = (
            float(route_end_xy[0]) + (float(normal_xy[0]) * float(offset_m) * float(end_side)),
            float(route_end_xy[1]) + (float(normal_xy[1]) * float(offset_m) * float(end_side)),
        )
    route_bearing_deg = _bearing_deg_from_xy(route_start_xy, route_end_xy)
    planned_bearing_deg = _bearing_deg_from_xy(start_anchor_xy, end_anchor_xy)
    origin_xy = _first_xy_from_aircraft_row(aircraft_row) or fallback_origin_xy or route_start_xy
    heading_deg = _first_float((aircraft_row or {}).get("headingDeg")) if isinstance(aircraft_row, dict) else None
    if heading_deg is None:
        heading_deg = _bearing_deg_from_xy(origin_xy, start_anchor_xy)
    turn_sign = 0
    try:
        turn_sign = int((aircraft_row or {}).get("turnSign") or 0)
    except Exception:
        turn_sign = 0
    turn_radius_m = _first_float(
        (aircraft_row or {}).get("turnRadiusM") if isinstance(aircraft_row, dict) else None,
        (aircraft_row or {}).get("turnCircleRadiusM") if isinstance(aircraft_row, dict) else None,
        (aircraft_row or {}).get("actualTurnRadiusM") if isinstance(aircraft_row, dict) else None,
        (aircraft_row or {}).get("idealTurnRadiusM") if isinstance(aircraft_row, dict) else None,
        TURN_PREVIEW_RADIUS_M,
    )

    distance_m = _distance_xy(origin_xy, start_anchor_xy)
    initial_turn_sign = _turn_sign_toward_xy(
        origin_xy=origin_xy,
        heading_deg=float(heading_deg),
        target_xy=start_anchor_xy,
    )
    hx, hy = _bearing_unit_xy(float(heading_deg))
    approach_vx = float(start_anchor_xy[0]) - float(origin_xy[0])
    approach_vy = float(start_anchor_xy[1]) - float(origin_xy[1])
    forward_projection_m = (approach_vx * hx) + (approach_vy * hy)
    behind_penalty_m = abs(min(0.0, float(forward_projection_m))) * 2.5
    route_angle_penalty = _angle_delta_deg(planned_bearing_deg, route_bearing_deg) * 6.0
    offset_penalty_m = float(offset_m) * 0.15
    side_switch_penalty_m = 80.0 if float(start_side) != float(end_side) else 0.0

    turn_penalty_m = 0.0
    if turn_sign != 0 and initial_turn_sign != 0:
        if int(turn_sign) == int(initial_turn_sign):
            turn_penalty_m -= min(160.0, max(40.0, float(turn_radius_m or TURN_PREVIEW_RADIUS_M) * 0.25))
        else:
            turn_penalty_m += max(350.0, float(turn_radius_m or TURN_PREVIEW_RADIUS_M) * 0.9)

    dubins_length_m = _dubins_length_m(
        start_xy=origin_xy,
        start_heading_deg=float(heading_deg),
        end_xy=start_anchor_xy,
        end_heading_deg=float(planned_bearing_deg),
        radius_m=turn_radius_m,
    )
    dubins_penalty_m = 0.0
    if dubins_length_m is not None and dubins_length_m > 0.0:
        dubins_penalty_m = max(0.0, float(dubins_length_m) - float(distance_m)) * 0.35

    score = (
        float(distance_m)
        + float(behind_penalty_m)
        + float(route_angle_penalty)
        + float(offset_penalty_m)
        + float(side_switch_penalty_m)
        + float(turn_penalty_m)
        + float(dubins_penalty_m)
    )
    return _LineOffsetPlan(
        start_side=float(start_side),
        end_side=float(end_side),
        offset_m=float(offset_m),
        score=float(score),
        dubins_length_m=dubins_length_m,
        first_turn_sign=int(initial_turn_sign),
    )


def _choose_line_offset_plan(
    route_line_xy: Sequence[Tuple[float, float]],
    *,
    route_offset_m: float,
    db_sep_m: float,
    aircraft_row: Dict[str, Any] | None,
    fallback_origin_xy: Tuple[float, float] | None,
) -> _LineOffsetPlan | None:
    frame = _line_route_frame_xy(route_line_xy)
    if frame is None:
        return None
    route_start_xy, route_end_xy, _unit_xy, normal_xy = frame
    max_offset_m = max(1.0, float(route_offset_m))
    if db_sep_m > 0.0:
        max_offset_m = min(float(max_offset_m), float(db_sep_m))
    candidates: List[_LineOffsetPlan] = []
    side_pairs = ((1.0, 1.0), (-1.0, -1.0), (1.0, -1.0), (-1.0, 1.0))
    for scale in LINE_OFFSET_CANDIDATE_SCALES:
        candidate_offset_m = max(1.0, float(max_offset_m) * float(scale))
        for start_side, end_side in side_pairs:
            start_anchor_xy = _route_endpoint_offset_xy(
                route_line_xy,
                side=float(start_side),
                offset_m=float(candidate_offset_m),
                at_end=False,
            )
            end_anchor_xy = _route_endpoint_offset_xy(
                route_line_xy,
                side=float(end_side),
                offset_m=float(candidate_offset_m),
                at_end=True,
            )
            candidates.append(
                _score_line_offset_plan(
                    route_start_xy=route_start_xy,
                    route_end_xy=route_end_xy,
                    normal_xy=normal_xy,
                    start_side=float(start_side),
                    end_side=float(end_side),
                    offset_m=float(candidate_offset_m),
                    aircraft_row=aircraft_row,
                    fallback_origin_xy=fallback_origin_xy,
                    start_anchor_xy=start_anchor_xy,
                    end_anchor_xy=end_anchor_xy,
                )
            )
    if not candidates:
        return None
    return min(
        candidates,
        key=lambda plan: (
            float(plan.score),
            float(plan.offset_m),
            0 if float(plan.start_side) == float(plan.end_side) else 1,
        ),
    )


def _clamp_line_offset_plan_to_sweep(
    plan: _LineOffsetPlan | None,
    *,
    sweep_xy: Sequence[Tuple[float, float]],
    route_line_xy: Sequence[Tuple[float, float]],
    db_sep_m: float,
) -> _LineOffsetPlan | None:
    if plan is None or db_sep_m <= 0.0 or len(sweep_xy) < 2:
        return plan
    anchor_xy = _line_anchor_from_route_plan_xy(
        sweep_xy,
        route_line_xy=route_line_xy,
        offset_plan=plan,
    )
    clearance_m = _line_anchor_clearance_m(anchor_xy, sweep_xy)
    if clearance_m is None or clearance_m <= float(db_sep_m) + 1e-6:
        return _LineOffsetPlan(
            start_side=plan.start_side,
            end_side=plan.end_side,
            offset_m=plan.offset_m,
            score=plan.score,
            dubins_length_m=plan.dubins_length_m,
            first_turn_sign=plan.first_turn_sign,
            clearance_m=clearance_m,
        )

    lo = 0.0
    hi = float(plan.offset_m)
    best_clearance_m = clearance_m
    for _ in range(32):
        mid = (lo + hi) * 0.5
        candidate = _LineOffsetPlan(
            start_side=plan.start_side,
            end_side=plan.end_side,
            offset_m=float(mid),
            score=plan.score,
            dubins_length_m=plan.dubins_length_m,
            first_turn_sign=plan.first_turn_sign,
        )
        candidate_anchor = _line_anchor_from_route_plan_xy(
            sweep_xy,
            route_line_xy=route_line_xy,
            offset_plan=candidate,
        )
        candidate_clearance_m = _line_anchor_clearance_m(candidate_anchor, sweep_xy)
        if candidate_clearance_m is None:
            hi = mid
            continue
        best_clearance_m = float(candidate_clearance_m)
        if candidate_clearance_m <= float(db_sep_m) + 1e-6:
            lo = mid
        else:
            hi = mid
    return _LineOffsetPlan(
        start_side=plan.start_side,
        end_side=plan.end_side,
        offset_m=max(1.0, float(lo)),
        score=plan.score,
        dubins_length_m=plan.dubins_length_m,
        first_turn_sign=plan.first_turn_sign,
        clearance_m=float(best_clearance_m),
    )


def _ensure(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def _to_float(value: Any) -> float | None:
    try:
        if value is None:
            return None
        return float(value)
    except Exception:
        return None


def _clamp_line_fov_deg(value: Any, default: float | None = None) -> float:
    fov_deg = _to_float(value)
    if fov_deg is None or fov_deg <= 0.0:
        fov_deg = _to_float(default)
    if fov_deg is None or fov_deg <= 0.0:
        fov_deg = float(MIN_LINE_FOV_DEG)
    return max(float(MIN_LINE_FOV_DEG), float(fov_deg))


def _normalize_coordinate(payload: object | None) -> Dict[str, float] | None:
    if not isinstance(payload, dict):
        return None
    lat = _to_float(payload.get("latitude"))
    lon = _to_float(payload.get("longitude"))
    if lat is None or lon is None:
        return None
    out: Dict[str, float] = {"latitude": float(lat), "longitude": float(lon)}
    alt = _to_float(payload.get("altitude"))
    if alt is not None:
        out["altitude"] = float(alt)
    return out


def _normalize_coord_list(payload: object | None) -> List[Dict[str, float]]:
    coords = payload if isinstance(payload, list) else []
    out: List[Dict[str, float]] = []
    for item in coords:
        coord = _normalize_coordinate(item)
        if coord is not None:
            out.append(coord)
    return out


def _choose_width_fallback(detail: Dict[str, Any], mission: Dict[str, Any]) -> float:
    for payload in (detail, mission):
        for key in ("lineWidthM", "width"):
            width = _to_float(payload.get(key))
            if width is not None and width > 0.0:
                return float(width)
    return 1.0


def _normalize_line_specs(
    target_mission: Dict[str, Any],
) -> tuple[Dict[str, Any], List[Dict[str, Any]]]:
    mission = copy.deepcopy(target_mission if isinstance(target_mission, dict) else {})
    detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else None
    if not isinstance(detail, dict):
        detail = mission
        mission = {"missionDetail": copy.deepcopy(detail)}

    width_fallback = _choose_width_fallback(detail, mission)
    line_specs: List[Dict[str, Any]] = []

    for line in detail.get("lineList") or []:
        if not isinstance(line, dict):
            continue
        coords = _normalize_coord_list(line.get("coordinateList"))
        if len(coords) < 2:
            continue
        width_m = _to_float(line.get("width"))
        if width_m is None or width_m <= 0.0:
            width_m = width_fallback
        line_specs.append({"width": float(width_m), "coordinateList": coords})

    coord_list = _normalize_coord_list(detail.get("coordinateList"))
    if not line_specs and len(coord_list) >= 2:
        line_specs.append({"width": float(width_fallback), "coordinateList": coord_list})

    _ensure(bool(line_specs), "next-collab line planner requires valid line geometry.")

    detail = dict(detail)
    detail["lineList"] = copy.deepcopy(line_specs)
    detail["coordinateList"] = copy.deepcopy(coord_list) if len(coord_list) >= 2 else None
    mission["missionDetail"] = detail
    if _to_float(mission.get("inputMissionID")) is None:
        mission["inputMissionID"] = 1
    try:
        mission_type = int(mission.get("inputMissionType"))
    except Exception:
        mission_type = 1
    if mission_type not in (1, 4, 5, 7):
        mission["inputMissionType"] = 1
    return mission, line_specs


def _resolve_source_line_width_m(
    mission: Dict[str, Any],
    detail: Dict[str, Any],
    line_specs: Sequence[Dict[str, Any]],
) -> float:
    for payload in (detail, mission):
        width_m = _to_float(payload.get("sourceLineWidthM"))
        if width_m is not None and width_m > 0.0:
            return float(width_m)
    for spec in line_specs:
        width_m = _to_float(spec.get("width"))
        if width_m is not None and width_m > 0.0:
            return float(width_m)
    return float(_choose_width_fallback(detail, mission))


def _resolve_source_coordinate_list(
    detail: Dict[str, Any],
    line_specs: Sequence[Dict[str, Any]],
) -> List[Dict[str, float]]:
    source_coords = _normalize_coord_list(detail.get("sourceCoordinateList"))
    if len(source_coords) >= 2:
        return source_coords
    for spec in line_specs:
        coords = _normalize_coord_list(spec.get("coordinateList"))
        if len(coords) >= 2:
            return coords
    fallback_coords = _normalize_coord_list(detail.get("coordinateList"))
    if len(fallback_coords) >= 2:
        return fallback_coords
    return []


def _mission_center_xy(line_specs: List[Dict[str, Any]]) -> tuple[float, float] | None:
    points_xy: List[Tuple[float, float]] = []
    for spec in line_specs:
        points_xy.extend(coords_to_xy(spec.get("coordinateList") or []))
    if not points_xy:
        return None
    return (
        sum(float(x) for x, _ in points_xy) / float(len(points_xy)),
        sum(float(y) for _, y in points_xy) / float(len(points_xy)),
    )


def _normalize_aircraft_rows(
    aircraft_entries: List[Dict[str, Any]],
    *,
    mission_center_xy: tuple[float, float] | None,
) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    for entry in aircraft_entries:
        if not isinstance(entry, dict):
            continue
        try:
            aircraft_id = int(entry.get("aircraftID"))
        except Exception:
            continue
        coord = _normalize_coordinate(entry.get("coordinate"))
        position_xy = coord_to_xy(coord) if coord is not None else None
        if aircraft_id <= 0 or coord is None or position_xy is None:
            continue
        current_coord = _normalize_coordinate(entry.get("currentCoordinate"))
        current_position_xy = coord_to_xy(current_coord) if current_coord is not None else None
        heading_deg = _to_float(entry.get("headingDeg"))
        if heading_deg is None:
            heading_deg = _to_float(entry.get("heading"))
        try:
            planner_heading = (
                float(monitor_heading_to_planner_bearing_deg(heading_deg))
                if heading_deg is not None
                else None
            )
        except Exception:
            planner_heading = None
        if planner_heading is None and mission_center_xy is not None:
            planner_heading = _bearing_deg_from_xy(position_xy, mission_center_xy)
        if planner_heading is None:
            planner_heading = 0.0
        turn_sign = 0
        try:
            turn_sign = int(entry.get("turnSign") or 0)
        except Exception:
            turn_sign = 0
        turn_radius_m = _first_float(
            entry.get("turnRadiusM"),
            entry.get("turnCircleRadiusM"),
            entry.get("actualTurnRadiusM"),
            entry.get("idealTurnRadiusM"),
        )
        rows.append(
            {
                "aircraftID": int(aircraft_id),
                "coordinate": {
                    "latitude": float(coord["latitude"]),
                    "longitude": float(coord["longitude"]),
                    "altitude": float(coord.get("altitude", 0.0) or 0.0),
                },
                "positionXY": (float(position_xy[0]), float(position_xy[1])),
                "currentCoordinate": dict(current_coord) if current_coord is not None else None,
                "currentPositionXY": (
                    (float(current_position_xy[0]), float(current_position_xy[1]))
                    if current_position_xy is not None
                    else None
                ),
                "headingDeg": float(planner_heading),
                "speedMps": _first_float(entry.get("speedMps"), entry.get("speed_mps")),
                "turnRateDps": _first_float(entry.get("turnRateDps"), entry.get("turn_rate_dps")),
                "turnSign": int(turn_sign),
                "turnRadiusM": turn_radius_m,
                "idealTurnRadiusM": _first_float(entry.get("idealTurnRadiusM")),
                "actualTurnRadiusM": _first_float(entry.get("actualTurnRadiusM")),
                "turnCircleRadiusM": _first_float(entry.get("turnCircleRadiusM")),
            }
        )
    rows.sort(key=lambda item: int(item["aircraftID"]))
    return rows


def _piece_centerline_xy(piece: SplitPiece) -> List[Tuple[float, float]]:
    data = piece.data if isinstance(piece.data, dict) else {}
    for key in ("Centerline", "coordinateList", "rawCoordinateList"):
        coords = coords_to_xy(data.get(key) or [])
        if len(coords) >= 2:
            return [(float(x), float(y)) for x, y in coords]
    return []


def _piece_target_xy(piece: SplitPiece) -> Tuple[float, float] | None:
    coords_xy = _piece_centerline_xy(piece)
    if len(coords_xy) >= 3:
        try:
            poly = Polygon(coords_xy)
            if not poly.is_empty:
                return float(poly.centroid.x), float(poly.centroid.y)
        except Exception:
            pass
    if len(coords_xy) >= 2:
        try:
            line = LineString(coords_xy)
            mid = line.interpolate(0.5, normalized=True)
            return float(mid.x), float(mid.y)
        except Exception:
            return coords_xy[len(coords_xy) // 2]
    return None


def _piece_width_m(piece: SplitPiece) -> float:
    data = piece.data if isinstance(piece.data, dict) else {}
    width_m = _to_float(data.get("width"))
    if width_m is None or width_m <= 0.0:
        return 1.0
    return float(width_m)


def _midpoint_xy(points_xy: Sequence[Tuple[float, float]]) -> Tuple[float, float] | None:
    rows = [(float(x), float(y)) for x, y in points_xy]
    if not rows:
        return None
    return (
        sum(float(x) for x, _ in rows) / float(len(rows)),
        sum(float(y) for _, y in rows) / float(len(rows)),
    )


def _marker_xy(marker_rows: Any, kind: str) -> Tuple[float, float] | None:
    if not isinstance(marker_rows, list):
        return None
    for row in marker_rows:
        if not isinstance(row, dict):
            continue
        if str(row.get("kind") or "") != str(kind):
            continue
        point_xy = row.get("xy")
        if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2:
            return float(point_xy[0]), float(point_xy[1])
    return None


def _orient_polyline_xy(
    points_xy: Sequence[Tuple[float, float]],
    *,
    reference_xy: Tuple[float, float] | None,
) -> List[Tuple[float, float]]:
    rows = [(float(x), float(y)) for x, y in points_xy]
    if len(rows) < 2 or reference_xy is None:
        return rows
    start_dist = _distance_xy(reference_xy, rows[0])
    end_dist = _distance_xy(reference_xy, rows[-1])
    if end_dist + 1e-6 < start_dist:
        rows.reverse()
    return rows


def _build_line_sweep_items(
    sweep_lines_xy: Sequence[Sequence[Tuple[float, float]]],
    *,
    reference_xy: Tuple[float, float] | None,
    offset_m: float,
    route_line_xy: Sequence[Tuple[float, float]] | None = None,
    offset_plan: _LineOffsetPlan | None = None,
) -> List[Dict[str, Any]]:
    items: List[Dict[str, Any]] = []
    for sweep_xy in sweep_lines_xy:
        line_xy = [(float(x), float(y)) for x, y in sweep_xy]
        if len(line_xy) < 2:
            continue
        anchor_xy = _line_anchor_xy_from_sweep(
            line_xy,
            offset_m=float(offset_m),
            route_line_xy=route_line_xy,
            reference_xy=reference_xy,
            offset_plan=offset_plan,
        )
        if anchor_xy is None:
            continue
        items.append(
            {
                "anchorXY": (float(anchor_xy[0]), float(anchor_xy[1])),
                "sweepXY": line_xy,
            }
        )
    if len(items) >= 2 and reference_xy is not None:
        if _distance_xy(reference_xy, items[-1]["anchorXY"]) + 1e-6 < _distance_xy(reference_xy, items[0]["anchorXY"]):
            items.reverse()
    prev_target_xy = reference_xy
    for item in items:
        line_xy = [(float(x), float(y)) for x, y in item.get("sweepXY") or []]
        if len(line_xy) < 2:
            continue
        if prev_target_xy is not None:
            start_dist = _distance_xy(prev_target_xy, line_xy[0])
            end_dist = _distance_xy(prev_target_xy, line_xy[-1])
            if end_dist + 1e-6 < start_dist:
                line_xy.reverse()
        item["sweepXY"] = line_xy
        prev_target_xy = line_xy[-1]
    return items


def _polyline_length_xy(points_xy: Sequence[Tuple[float, float]]) -> float:
    rows = [(float(x), float(y)) for x, y in points_xy]
    if len(rows) < 2:
        return 0.0
    total = 0.0
    for idx in range(1, len(rows)):
        total += _distance_xy(rows[idx - 1], rows[idx])
    return float(total)


def _resample_polyline_xy(
    points_xy: Sequence[Tuple[float, float]],
    spacing_m: float,
) -> List[Tuple[float, float]]:
    rows = [(float(x), float(y)) for x, y in points_xy]
    if len(rows) < 2:
        return rows
    spacing_m = max(float(spacing_m), 1.0)
    total_len_m = _polyline_length_xy(rows)
    if total_len_m <= spacing_m + 1e-6:
        return [rows[0], rows[-1]]

    targets: List[float] = [0.0]
    cursor = float(spacing_m)
    while cursor < total_len_m - 1e-6:
        targets.append(float(cursor))
        cursor += float(spacing_m)
    if total_len_m - targets[-1] > 1e-6:
        targets.append(float(total_len_m))

    out: List[Tuple[float, float]] = []
    seg_start_idx = 0
    walked_m = 0.0
    for target_m in targets:
        while seg_start_idx < len(rows) - 1:
            seg_len_m = _distance_xy(rows[seg_start_idx], rows[seg_start_idx + 1])
            if walked_m + seg_len_m + 1e-6 >= target_m:
                if seg_len_m <= 1e-6:
                    out.append(rows[seg_start_idx])
                else:
                    ratio = max(0.0, min(1.0, (target_m - walked_m) / seg_len_m))
                    x = float(rows[seg_start_idx][0]) + (
                        (float(rows[seg_start_idx + 1][0]) - float(rows[seg_start_idx][0])) * ratio
                    )
                    y = float(rows[seg_start_idx][1]) + (
                        (float(rows[seg_start_idx + 1][1]) - float(rows[seg_start_idx][1])) * ratio
                    )
                    out.append((x, y))
                break
            walked_m += seg_len_m
            seg_start_idx += 1
        else:
            out.append(rows[-1])
    deduped: List[Tuple[float, float]] = []
    for point_xy in out:
        if deduped and _distance_xy(deduped[-1], point_xy) <= 1.0:
            continue
        deduped.append(point_xy)
    if len(deduped) == 1:
        deduped.append(rows[-1])
    return deduped


def _select_route_sweep_indices(
    route_points_xy: Sequence[Tuple[float, float]],
    sweep_midpoints_xy: Sequence[Tuple[float, float]],
) -> List[int]:
    if not route_points_xy or not sweep_midpoints_xy:
        return []
    total_pts = len(route_points_xy)
    total_sweeps = len(sweep_midpoints_xy)
    if total_sweeps <= total_pts:
        return list(range(total_sweeps))

    selected: List[int] = []
    start_idx = 0
    for pos, point_xy in enumerate(route_points_xy):
        remaining = total_pts - pos - 1
        max_idx = total_sweeps - 1 - remaining
        if max_idx < start_idx:
            max_idx = start_idx
        best_idx = start_idx
        best_dist = None
        for idx in range(start_idx, max_idx + 1):
            dist = _distance_xy(point_xy, sweep_midpoints_xy[idx])
            if best_dist is None or dist < best_dist:
                best_dist = dist
                best_idx = idx
        selected.append(int(best_idx))
        start_idx = int(best_idx) + 1
    if selected:
        selected[-1] = int(total_sweeps - 1)
    return selected


def _line_bearing_deg(points_xy: Sequence[Tuple[float, float]]) -> float:
    if len(points_xy) < 2:
        return 0.0
    return _bearing_deg_from_xy(points_xy[0], points_xy[-1])


def _line_anchor_xy_from_sweep(
    sweep_xy: Sequence[Tuple[float, float]],
    *,
    offset_m: float,
    route_line_xy: Sequence[Tuple[float, float]] | None = None,
    reference_xy: Tuple[float, float] | None = None,
    offset_plan: _LineOffsetPlan | None = None,
) -> Tuple[float, float] | None:
    if len(sweep_xy) < 2:
        return None
    if offset_plan is not None and route_line_xy is not None and len(route_line_xy) >= 2:
        anchor_xy = _line_anchor_from_route_plan_xy(
            sweep_xy,
            route_line_xy=route_line_xy,
            offset_plan=offset_plan,
        )
        if anchor_xy is not None:
            return anchor_xy
    if route_line_xy is not None and len(route_line_xy) >= 2:
        midpoint_xy = _midpoint_xy(sweep_xy)
        if midpoint_xy is not None:
            projected = _project_to_route_polyline_xy(midpoint_xy, route_line_xy)
            if projected is not None:
                center_xy, tangent_xy, _progress_ratio = projected
                normal_xy = (-float(tangent_xy[1]), float(tangent_xy[0]))
                offset_abs_m = max(float(offset_m), 0.0)
                candidates = [
                    (
                        float(center_xy[0]) + (normal_xy[0] * offset_abs_m),
                        float(center_xy[1]) + (normal_xy[1] * offset_abs_m),
                    ),
                    (
                        float(center_xy[0]) - (normal_xy[0] * offset_abs_m),
                        float(center_xy[1]) - (normal_xy[1] * offset_abs_m),
                    ),
                ]
                if reference_xy is not None:
                    return min(candidates, key=lambda anchor: _distance_xy(reference_xy, anchor))
                return candidates[0]
    (x1, y1), (x2, y2) = sweep_xy[0], sweep_xy[-1]
    mid_x = (float(x1) + float(x2)) * 0.5
    mid_y = (float(y1) + float(y2)) * 0.5
    dx = float(x2) - float(x1)
    dy = float(y2) - float(y1)
    norm = math.hypot(dx, dy)
    if norm <= 1e-6:
        return mid_x, mid_y
    ux = dy / norm
    uy = -dx / norm
    return (
        mid_x + (ux * float(max(offset_m, 0.0))),
        mid_y + (uy * float(max(offset_m, 0.0))),
    )


def _linestring_segments_xy(
    planner: _HeadlessLinePlanner,
    geom: Any,
) -> List[LineString]:
    segment_fn = getattr(planner, "_linestring_segments_xy", None)
    if callable(segment_fn):
        try:
            segments = segment_fn(geom)
            return [
                line
                for line in segments
                if isinstance(line, LineString) and float(line.length) > 1e-6
            ]
        except Exception:
            pass
    if geom is None or getattr(geom, "is_empty", True):
        return []
    if isinstance(geom, LineString):
        return [geom] if float(geom.length) > 1e-6 else []
    if str(getattr(geom, "geom_type", "")) not in {"MultiLineString", "GeometryCollection"}:
        return []
    return [
        line
        for line in getattr(geom, "geoms", [])
        if isinstance(line, LineString) and float(line.length) > 1e-6
    ]


def _nearest_linestring_to_xy(
    planner: _HeadlessLinePlanner,
    geom: Any,
    reference_xy: Tuple[float, float] | None,
) -> Optional[LineString]:
    lines = _linestring_segments_xy(planner, geom)
    if not lines:
        return None
    if reference_xy is None:
        return max(lines, key=lambda line: float(line.length))
    ref_point = Point(float(reference_xy[0]), float(reference_xy[1]))
    return min(
        lines,
        key=lambda line: (
            float(line.distance(ref_point)),
            -float(line.length),
        ),
    )


def _orient_line_sweep_for_anchor(
    sweep_xy: Sequence[Tuple[float, float]],
    *,
    anchor_ref_xy: Tuple[float, float] | None,
    offset_m: float,
    route_line_xy: Sequence[Tuple[float, float]] | None = None,
) -> List[Tuple[float, float]]:
    if len(sweep_xy) < 2:
        return [(float(x), float(y)) for x, y in sweep_xy]
    forward = [
        (float(sweep_xy[0][0]), float(sweep_xy[0][1])),
        (float(sweep_xy[-1][0]), float(sweep_xy[-1][1])),
    ]
    reverse = [forward[-1], forward[0]]
    if anchor_ref_xy is None:
        return forward
    anchor_forward = _line_anchor_xy_from_sweep(
        forward,
        offset_m=offset_m,
        route_line_xy=route_line_xy,
        reference_xy=anchor_ref_xy,
    )
    anchor_reverse = _line_anchor_xy_from_sweep(
        reverse,
        offset_m=offset_m,
        route_line_xy=route_line_xy,
        reference_xy=anchor_ref_xy,
    )
    if anchor_forward is None:
        return reverse if anchor_reverse is not None else forward
    if anchor_reverse is None:
        return forward
    if _distance_xy(anchor_reverse, anchor_ref_xy) + 1e-6 < _distance_xy(anchor_forward, anchor_ref_xy):
        return reverse
    return forward


def _point_on_linestring_xy(line: LineString, distance_m: float) -> Tuple[float, float]:
    point = line.interpolate(max(0.0, min(float(line.length), float(distance_m))))
    return float(point.x), float(point.y)


def _unit_xy(dx: float, dy: float) -> Tuple[float, float] | None:
    norm = math.hypot(float(dx), float(dy))
    if norm <= 1e-6:
        return None
    return float(dx) / norm, float(dy) / norm


def _blend_unit_xy(
    first_xy: Tuple[float, float],
    second_xy: Tuple[float, float],
    ratio: float,
) -> Tuple[float, float] | None:
    ax, ay = first_xy
    bx, by = second_xy
    if (float(ax) * float(bx)) + (float(ay) * float(by)) < 0.0:
        bx, by = -float(bx), -float(by)
    ratio = max(0.0, min(1.0, float(ratio)))
    return _unit_xy(
        (float(ax) * (1.0 - ratio)) + (float(bx) * ratio),
        (float(ay) * (1.0 - ratio)) + (float(by) * ratio),
    )


def _centerline_tangent_xy(
    points_xy: Sequence[Tuple[float, float]],
    *,
    distance_m: float,
) -> Tuple[float, float] | None:
    rows = [(float(x), float(y)) for x, y in points_xy]
    if len(rows) < 2:
        return None

    segment_dirs: List[Tuple[float, float]] = []
    segment_lengths: List[float] = []
    for idx in range(1, len(rows)):
        dx = float(rows[idx][0]) - float(rows[idx - 1][0])
        dy = float(rows[idx][1]) - float(rows[idx - 1][1])
        seg_len_m = math.hypot(dx, dy)
        unit_xy = _unit_xy(dx, dy)
        if unit_xy is None or seg_len_m <= 1e-6:
            continue
        segment_dirs.append(unit_xy)
        segment_lengths.append(float(seg_len_m))
    if not segment_dirs:
        return None

    total_len_m = sum(segment_lengths)
    target_m = max(0.0, min(float(total_len_m), float(distance_m)))
    walked_m = 0.0
    for idx, seg_len_m in enumerate(segment_lengths):
        if target_m <= walked_m + seg_len_m + 1e-6 or idx == len(segment_lengths) - 1:
            ratio = 0.0 if seg_len_m <= 1e-6 else (target_m - walked_m) / seg_len_m
            current_dir = segment_dirs[idx]
            start_dir = (
                _blend_unit_xy(segment_dirs[idx - 1], current_dir, 0.5)
                if idx > 0
                else current_dir
            ) or current_dir
            end_dir = (
                _blend_unit_xy(current_dir, segment_dirs[idx + 1], 0.5)
                if idx < len(segment_dirs) - 1
                else current_dir
            ) or current_dir
            return _blend_unit_xy(start_dir, end_dir, ratio) or current_dir
        walked_m += seg_len_m
    return segment_dirs[-1]


def _centerline_sweep_distances(total_len_m: float, sweep_spacing_m: float) -> List[float]:
    total_len_m = float(total_len_m)
    sweep_spacing_m = float(sweep_spacing_m)
    if total_len_m <= 1e-6 or sweep_spacing_m <= 0.0:
        return []
    distances_m: List[float] = [0.0]
    cursor_m = float(sweep_spacing_m)
    while cursor_m < total_len_m - 1e-6:
        distances_m.append(float(cursor_m))
        cursor_m += float(sweep_spacing_m)
    if abs(float(distances_m[-1]) - total_len_m) > max(5.0, sweep_spacing_m * 0.25):
        distances_m.append(total_len_m)
    return distances_m


def _centerline_corridor_sweep_lines_xy(
    planner: _HeadlessLinePlanner,
    piece: SplitPiece,
    *,
    sweep_spacing_m: float,
    route_offset_m: float,
    origin_xy: Tuple[float, float] | None,
    db_sep_m: float = 0.0,
    aircraft_row: Dict[str, Any] | None = None,
) -> tuple[
    List[List[Tuple[float, float]]],
    List[Dict[str, Any]],
    Tuple[float, float] | None,
    _LineOffsetPlan | None,
    float | None,
]:
    piece_poly = planner._piece_polygon_xy(piece)
    centerline_xy = _piece_centerline_xy(piece)
    if piece_poly is None or piece_poly.is_empty or len(centerline_xy) < 2:
        return [], [], None, None, None

    sweep_spacing_m = float(sweep_spacing_m)
    route_offset_m = max(float(route_offset_m), 0.0)
    if sweep_spacing_m <= 0.0:
        return [], [], None, None, None

    centerline_xy = _orient_polyline_xy(centerline_xy, reference_xy=origin_xy)
    line = LineString(centerline_xy)
    if line.is_empty or float(line.length) <= 1e-6:
        return [], [], None, None, None

    min_x, min_y, max_x, max_y = piece_poly.bounds
    extend_m = max(
        200.0,
        math.hypot(float(max_x) - float(min_x), float(max_y) - float(min_y))
        + (float(route_offset_m) * 2.0)
        + (float(sweep_spacing_m) * 2.0),
    )
    distances_m = _centerline_sweep_distances(float(line.length), float(sweep_spacing_m))
    if not distances_m:
        return [], [], None

    route_side_ref_xy = origin_xy
    base_dir_xy: Tuple[float, float] | None = None
    first_anchor_xy: Tuple[float, float] | None = None
    first_anchor_clearance_m: float | None = None
    scan_lines_xy: List[List[Tuple[float, float]]] = []
    line_sweep_items: List[Dict[str, Any]] = []
    offset_plan = _choose_line_offset_plan(
        centerline_xy,
        route_offset_m=float(route_offset_m),
        db_sep_m=float(db_sep_m),
        aircraft_row=aircraft_row,
        fallback_origin_xy=origin_xy,
    )

    for idx, dist_m in enumerate(distances_m):
        center_xy = _point_on_linestring_xy(line, dist_m)
        tangent_xy = _centerline_tangent_xy(
            centerline_xy,
            distance_m=float(dist_m),
        )
        if tangent_xy is None:
            continue
        tx, ty = tangent_xy
        guide_dx, guide_dy = ty, -tx
        guide = LineString(
            [
                (center_xy[0] - (guide_dx * extend_m), center_xy[1] - (guide_dy * extend_m)),
                (center_xy[0] + (guide_dx * extend_m), center_xy[1] + (guide_dy * extend_m)),
            ]
        )
        seg = _nearest_linestring_to_xy(
            planner,
            piece_poly.intersection(guide),
            center_xy,
        )
        if seg is None or len(seg.coords) < 2:
            continue
        forward_sweep_xy = [
            (float(seg.coords[0][0]), float(seg.coords[0][1])),
            (float(seg.coords[-1][0]), float(seg.coords[-1][1])),
        ]
        reverse_sweep_xy = [forward_sweep_xy[-1], forward_sweep_xy[0]]
        if base_dir_xy is None:
            base_sweep_xy = _orient_line_sweep_for_anchor(
                forward_sweep_xy,
                anchor_ref_xy=route_side_ref_xy,
                offset_m=float(route_offset_m),
                route_line_xy=centerline_xy,
            )
            base_dir_xy = (
                float(base_sweep_xy[-1][0]) - float(base_sweep_xy[0][0]),
                float(base_sweep_xy[-1][1]) - float(base_sweep_xy[0][1]),
            )
        else:
            forward_dot = (
                (float(forward_sweep_xy[-1][0]) - float(forward_sweep_xy[0][0])) * float(base_dir_xy[0])
                + (float(forward_sweep_xy[-1][1]) - float(forward_sweep_xy[0][1])) * float(base_dir_xy[1])
            )
            reverse_dot = (
                (float(reverse_sweep_xy[-1][0]) - float(reverse_sweep_xy[0][0])) * float(base_dir_xy[0])
                + (float(reverse_sweep_xy[-1][1]) - float(reverse_sweep_xy[0][1])) * float(base_dir_xy[1])
            )
            base_sweep_xy = forward_sweep_xy if forward_dot >= reverse_dot else reverse_sweep_xy
        if idx == 0:
            offset_plan = _clamp_line_offset_plan_to_sweep(
                offset_plan,
                sweep_xy=base_sweep_xy,
                route_line_xy=centerline_xy,
                db_sep_m=float(db_sep_m),
            )
        anchor_xy = _line_anchor_xy_from_sweep(
            base_sweep_xy,
            offset_m=float(route_offset_m),
            route_line_xy=centerline_xy,
            reference_xy=route_side_ref_xy,
            offset_plan=offset_plan,
        )
        if anchor_xy is not None and first_anchor_xy is None:
            first_anchor_xy = anchor_xy
            first_anchor_clearance_m = _line_anchor_clearance_m(anchor_xy, base_sweep_xy)
        if anchor_xy is not None and route_side_ref_xy is None:
            route_side_ref_xy = anchor_xy
        scan_sweep_xy = list(base_sweep_xy)
        if idx % 2 == 1:
            scan_sweep_xy.reverse()
        scan_lines_xy.append(scan_sweep_xy)
        if anchor_xy is not None:
            line_sweep_items.append(
                {
                    "anchorXY": (float(anchor_xy[0]), float(anchor_xy[1])),
                    "sweepXY": [(float(x), float(y)) for x, y in scan_sweep_xy],
                }
            )

    return scan_lines_xy, line_sweep_items, first_anchor_xy, offset_plan, first_anchor_clearance_m


def _corridor_sweep_lines_xy(
    planner: _HeadlessLinePlanner,
    piece: SplitPiece,
    overlay: Dict[str, Any],
    *,
    sweep_spacing_m: float,
    route_offset_m: float,
    origin_xy: Tuple[float, float] | None,
    db_sep_m: float = 0.0,
    aircraft_row: Dict[str, Any] | None = None,
) -> tuple[
    List[List[Tuple[float, float]]],
    List[Dict[str, Any]],
    Tuple[float, float] | None,
    _LineOffsetPlan | None,
    float | None,
]:
    piece_poly = planner._piece_polygon_xy(piece)
    bounds = planner._overlay_st_bounds(overlay)
    if piece_poly is None or piece_poly.is_empty or bounds is None:
        return [], [], None, None, None
    sweep_spacing_m = float(sweep_spacing_m)
    route_offset_m = max(float(route_offset_m), 0.0)
    if sweep_spacing_m <= 0.0:
        return [], [], None, None, None

    _bearing_deg, ux, uy, vx, vy, min_s, max_s, min_t, max_t = bounds
    mid_t = 0.5 * (float(min_t) + float(max_t))
    min_face_xy = planner._from_st_xy(min_s, mid_t, ux, uy, vx, vy)
    max_face_xy = planner._from_st_xy(max_s, mid_t, ux, uy, vx, vy)

    if origin_xy is not None and _distance_xy(origin_xy, max_face_xy) + 1e-6 < _distance_xy(origin_xy, min_face_xy):
        start_s = float(max_s)
        end_s = float(min_s)
        step_sign = -1.0
    else:
        start_s = float(min_s)
        end_s = float(max_s)
        step_sign = 1.0
    route_axis_xy = [
        planner._from_st_xy(float(start_s), float(mid_t), ux, uy, vx, vy),
        planner._from_st_xy(float(end_s), float(mid_t), ux, uy, vx, vy),
    ]

    s_values: List[float] = [float(start_s)]
    cursor_s = float(start_s)
    while True:
        next_s = float(cursor_s + (step_sign * float(sweep_spacing_m)))
        if (step_sign > 0.0 and next_s >= float(end_s) - 1e-6) or (
            step_sign < 0.0 and next_s <= float(end_s) + 1e-6
        ):
            break
        s_values.append(float(next_s))
        cursor_s = float(next_s)
    if abs(float(s_values[-1]) - float(end_s)) > max(5.0, float(sweep_spacing_m) * 0.25):
        s_values.append(float(end_s))

    pad_t = max(80.0, float(abs(max_t - min_t)) * 0.25)
    route_side_ref_xy = origin_xy
    base_dir_xy: Tuple[float, float] | None = None
    first_anchor_xy: Tuple[float, float] | None = None
    first_anchor_clearance_m: float | None = None
    scan_lines_xy: List[List[Tuple[float, float]]] = []
    line_sweep_items: List[Dict[str, Any]] = []
    offset_plan = _choose_line_offset_plan(
        route_axis_xy,
        route_offset_m=float(route_offset_m),
        db_sep_m=float(db_sep_m),
        aircraft_row=aircraft_row,
        fallback_origin_xy=origin_xy,
    )

    for idx, s_val in enumerate(s_values):
        guide = LineString(
            [
                planner._from_st_xy(float(s_val), float(min_t - pad_t), ux, uy, vx, vy),
                planner._from_st_xy(float(s_val), float(max_t + pad_t), ux, uy, vx, vy),
            ]
        )
        guide_center_xy = planner._from_st_xy(float(s_val), float(mid_t), ux, uy, vx, vy)
        seg = _nearest_linestring_to_xy(
            planner,
            piece_poly.intersection(guide),
            guide_center_xy,
        )
        if seg is None or len(seg.coords) < 2:
            continue
        forward_sweep_xy = [
            (float(seg.coords[0][0]), float(seg.coords[0][1])),
            (float(seg.coords[-1][0]), float(seg.coords[-1][1])),
        ]
        reverse_sweep_xy = [forward_sweep_xy[-1], forward_sweep_xy[0]]
        if base_dir_xy is None:
            base_sweep_xy = _orient_line_sweep_for_anchor(
                forward_sweep_xy,
                anchor_ref_xy=route_side_ref_xy,
                offset_m=float(route_offset_m),
                route_line_xy=route_axis_xy,
            )
            base_dir_xy = (
                float(base_sweep_xy[-1][0]) - float(base_sweep_xy[0][0]),
                float(base_sweep_xy[-1][1]) - float(base_sweep_xy[0][1]),
            )
        else:
            forward_dot = (
                (float(forward_sweep_xy[-1][0]) - float(forward_sweep_xy[0][0])) * float(base_dir_xy[0])
                + (float(forward_sweep_xy[-1][1]) - float(forward_sweep_xy[0][1])) * float(base_dir_xy[1])
            )
            reverse_dot = (
                (float(reverse_sweep_xy[-1][0]) - float(reverse_sweep_xy[0][0])) * float(base_dir_xy[0])
                + (float(reverse_sweep_xy[-1][1]) - float(reverse_sweep_xy[0][1])) * float(base_dir_xy[1])
            )
            base_sweep_xy = forward_sweep_xy if forward_dot >= reverse_dot else reverse_sweep_xy
        if idx == 0:
            offset_plan = _clamp_line_offset_plan_to_sweep(
                offset_plan,
                sweep_xy=base_sweep_xy,
                route_line_xy=route_axis_xy,
                db_sep_m=float(db_sep_m),
            )
        anchor_xy = _line_anchor_xy_from_sweep(
            base_sweep_xy,
            offset_m=float(route_offset_m),
            route_line_xy=route_axis_xy,
            reference_xy=route_side_ref_xy,
            offset_plan=offset_plan,
        )
        if anchor_xy is not None and first_anchor_xy is None:
            first_anchor_xy = anchor_xy
            first_anchor_clearance_m = _line_anchor_clearance_m(anchor_xy, base_sweep_xy)
        if anchor_xy is not None and route_side_ref_xy is None:
            route_side_ref_xy = anchor_xy
        scan_sweep_xy = list(base_sweep_xy)
        if idx % 2 == 1:
            scan_sweep_xy.reverse()
        scan_lines_xy.append(scan_sweep_xy)
        if anchor_xy is not None:
            line_sweep_items.append(
                {
                    "anchorXY": (float(anchor_xy[0]), float(anchor_xy[1])),
                    "sweepXY": [(float(x), float(y)) for x, y in scan_sweep_xy],
                }
            )

    return scan_lines_xy, line_sweep_items, first_anchor_xy, offset_plan, first_anchor_clearance_m


def _synthetic_line_overlay(
    piece: SplitPiece,
    *,
    reference_bearing_deg: float,
) -> Optional[Dict[str, Any]]:
    centerline_xy = _piece_centerline_xy(piece)
    if len(centerline_xy) < 2:
        return None
    line = LineString(centerline_xy)
    length_m = float(line.length)
    if length_m <= 1e-6:
        return None
    bearing_deg = _line_bearing_deg(centerline_xy) or float(reference_bearing_deg)
    width_m = max(1.0, float(_piece_width_m(piece)))
    half_width_m = max(0.5, width_m * 0.5)
    half_length_m = max(1.0, length_m * 0.5)
    center_point = line.interpolate(0.5, normalized=True)
    center_xy = (float(center_point.x), float(center_point.y))
    theta = math.radians(float(bearing_deg) % 360.0)
    ux = math.sin(theta)
    uy = math.cos(theta)
    vx = uy
    vy = -ux

    box_xy = [
        (center_xy[0] - (ux * half_length_m) - (vx * half_width_m), center_xy[1] - (uy * half_length_m) - (vy * half_width_m)),
        (center_xy[0] + (ux * half_length_m) - (vx * half_width_m), center_xy[1] + (uy * half_length_m) - (vy * half_width_m)),
        (center_xy[0] + (ux * half_length_m) + (vx * half_width_m), center_xy[1] + (uy * half_length_m) + (vy * half_width_m)),
        (center_xy[0] - (ux * half_length_m) + (vx * half_width_m), center_xy[1] - (uy * half_length_m) + (vy * half_width_m)),
    ]

    return {
        "pieceIndex": int(piece.piece_index or 0),
        "aircraftID": int(piece.assigned_uav or 0),
        "bearingDeg": float(bearing_deg),
        "boxXY": box_xy,
        "midLineXY": [centerline_xy[0], centerline_xy[-1]],
        "centerXY": center_xy,
        "lengthM": float(length_m),
        "widthM": float(width_m),
        "maxWidthM": float(width_m),
        "maxWidthLeftM": float(half_width_m),
        "maxWidthRightM": float(half_width_m),
        "dbCoverWidthM": float(width_m),
        "dbMaxWidthM": float(width_m),
        "midLineRequired": False,
        "synthetic": True,
    }


def _line_reference_bearing_deg(
    line_specs: List[Dict[str, Any]],
    pieces: Sequence[SplitPiece],
) -> float:
    for spec in line_specs:
        coords_xy = coords_to_xy(spec.get("coordinateList") or [])
        if len(coords_xy) >= 2:
            return _line_bearing_deg(coords_xy)
    for piece in pieces:
        coords_xy = _piece_centerline_xy(piece)
        if len(coords_xy) >= 2:
            return _line_bearing_deg(coords_xy)
    return 0.0


def _next_collab_entry_tprime_target_sep_ratio(runtime_cfg: Dict[str, Any] | None = None) -> float:
    value = float(
        get_runtime_float(
            "next_collab_entry_tprime_target_sep_ratio",
            NEXT_COLLAB_ENTRY_TPRIME_TARGET_SEP_RATIO,
            runtime_cfg,
        )
    )
    scale = float(
        get_runtime_float(
            "next_collab_entry_tprime_ratio_scale",
            NEXT_COLLAB_ENTRY_TPRIME_RATIO_SCALE,
            runtime_cfg,
        )
    )
    return max(0.05, min(1.0, float(value) * max(0.10, min(5.0, float(scale)))))


def _next_collab_entry_tprime_db_row(
    planner: _HeadlessLinePlanner,
    width_m: float,
    *,
    runtime_cfg: Dict[str, Any] | None = None,
) -> Dict[str, float] | None:
    target_m = max(0.0, float(width_m))
    if target_m <= 0.0:
        return None

    candidates = [
        dict(row)
        for row in (planner._fov_db_rows() or [])
        if float(row.get("width", 0.0) or 0.0) + 1e-6 >= target_m
    ]
    if not candidates:
        fallback_row = planner._covering_db_row(float(width_m))
        return dict(fallback_row) if isinstance(fallback_row, dict) else None

    max_sep_row = min(
        candidates,
        key=lambda row: (
            -float(row.get("sep", 0.0) or 0.0),
            float(row.get("width", 0.0) or 0.0),
            -float(row.get("fov", 0.0) or 0.0),
        ),
    )
    max_sep_m = float(max_sep_row.get("sep", 0.0) or 0.0)
    if max_sep_m <= 0.0:
        return dict(max_sep_row)

    target_sep_m = max_sep_m * _next_collab_entry_tprime_target_sep_ratio(runtime_cfg)
    lower_or_equal = [
        dict(row)
        for row in candidates
        if float(row.get("sep", 0.0) or 0.0) <= target_sep_m + 1e-6
    ]
    if lower_or_equal:
        return max(
            lower_or_equal,
            key=lambda row: (
                float(row.get("sep", 0.0) or 0.0),
                -float(row.get("fov", 0.0) or 0.0),
                -float(row.get("width", 0.0) or 0.0),
            ),
        )

    return min(
        candidates,
        key=lambda row: (
            abs(float(row.get("sep", 0.0) or 0.0) - target_sep_m),
            float(row.get("width", 0.0) or 0.0),
            -float(row.get("fov", 0.0) or 0.0),
        ),
    )


def _next_collab_entry_tprime_target_sep_m(
    planner: _HeadlessLinePlanner,
    width_m: float,
    *,
    runtime_cfg: Dict[str, Any] | None = None,
) -> float:
    db_row = _next_collab_entry_tprime_db_row(planner, width_m, runtime_cfg=runtime_cfg)
    if not isinstance(db_row, dict):
        return 0.0
    sep_m = float(db_row.get("sep", 0.0) or 0.0)
    if sep_m <= 0.0:
        return 0.0
    return float(sep_m)


def _next_collab_line_db_weights(
    runtime_cfg: Dict[str, Any] | None = None,
) -> tuple[float, float, float]:
    width_weight = max(
        0.0,
        float(
            get_runtime_float(
                "next_collab_line_db_width_weight",
                NEXT_COLLAB_LINE_DB_WIDTH_WEIGHT,
                runtime_cfg,
            )
        ),
    )
    sep_weight = max(
        0.0,
        float(
            get_runtime_float(
                "next_collab_line_db_sep_weight",
                NEXT_COLLAB_LINE_DB_SEP_WEIGHT,
                runtime_cfg,
            )
        ),
    )
    fov_weight = max(
        0.0,
        float(
            get_runtime_float(
                "next_collab_line_db_fov_weight",
                NEXT_COLLAB_LINE_DB_FOV_WEIGHT,
                runtime_cfg,
            )
        ),
    )
    total = float(width_weight + sep_weight + fov_weight)
    if total <= 1e-9:
        return (
            float(NEXT_COLLAB_LINE_DB_WIDTH_WEIGHT),
            float(NEXT_COLLAB_LINE_DB_SEP_WEIGHT),
            float(NEXT_COLLAB_LINE_DB_FOV_WEIGHT),
        )
    return (
        float(width_weight) / total,
        float(sep_weight) / total,
        float(fov_weight) / total,
    )


def _normalized_unit_score(value: float, minimum: float, maximum: float, *, prefer_low: bool) -> float:
    if maximum - minimum <= 1e-9:
        return 1.0
    ratio = (float(value) - float(minimum)) / float(maximum - minimum)
    ratio = max(0.0, min(1.0, float(ratio)))
    return 1.0 - ratio if prefer_low else ratio


def _next_collab_resolved_db_row(
    planner: _HeadlessLinePlanner,
    width_m: float,
    sep_m: float,
    *,
    runtime_cfg: Dict[str, Any] | None = None,
) -> Dict[str, float] | None:
    target_m = max(0.0, float(width_m))
    limit_sep_m = _db_sep_requirement_m(float(sep_m), runtime_cfg)
    if target_m <= 0.0:
        return None

    candidates = [
        dict(row)
        for row in (planner._fov_db_rows() or [])
        if float(row.get("width", 0.0) or 0.0) + 1e-6 >= target_m
    ]
    if not candidates:
        return None

    matching: List[Dict[str, float]] = []
    if limit_sep_m > 0.0:
        matching = [
            dict(row)
            for row in candidates
            if float(row.get("sep", 0.0) or 0.0) >= limit_sep_m
        ]
    pool = matching or candidates
    width_weight, sep_weight, fov_weight = _next_collab_line_db_weights(runtime_cfg)

    width_errors = [
        max(float(row.get("width", 0.0) or 0.0) - target_m, 0.0)
        for row in pool
    ]
    if limit_sep_m > 0.0 and matching:
        sep_errors = [
            max(float(row.get("sep", 0.0) or 0.0) - limit_sep_m, 0.0)
            for row in pool
        ]
    elif limit_sep_m > 0.0:
        sep_errors = [
            abs(float(row.get("sep", 0.0) or 0.0) - limit_sep_m)
            for row in pool
        ]
    else:
        sep_errors = [0.0 for _ in pool]
    fov_values = [float(row.get("fov", 0.0) or 0.0) for row in pool]

    min_width_error = min(width_errors) if width_errors else 0.0
    max_width_error = max(width_errors) if width_errors else 0.0
    min_sep_error = min(sep_errors) if sep_errors else 0.0
    max_sep_error = max(sep_errors) if sep_errors else 0.0
    min_fov = min(fov_values) if fov_values else 0.0
    max_fov = max(fov_values) if fov_values else 0.0

    scored_rows: List[tuple[float, Dict[str, float]]] = []
    for row, width_error, sep_error, fov_value in zip(pool, width_errors, sep_errors, fov_values):
        width_score = _normalized_unit_score(
            float(width_error),
            float(min_width_error),
            float(max_width_error),
            prefer_low=True,
        )
        sep_score = _normalized_unit_score(
            float(sep_error),
            float(min_sep_error),
            float(max_sep_error),
            prefer_low=True,
        )
        fov_score = _normalized_unit_score(
            float(fov_value),
            float(min_fov),
            float(max_fov),
            prefer_low=False,
        )
        total_score = (
            (float(width_weight) * float(width_score))
            + (float(sep_weight) * float(sep_score))
            + (float(fov_weight) * float(fov_score))
        )
        scored_rows.append((float(total_score), dict(row)))

    best_score, best_row = max(
        scored_rows,
        key=lambda item: (
            float(item[0]),
            -max(float(item[1].get("width", 0.0) or 0.0) - target_m, 0.0),
            -(
                max(float(item[1].get("sep", 0.0) or 0.0) - limit_sep_m, 0.0)
                if limit_sep_m > 0.0 and matching
                else abs(float(item[1].get("sep", 0.0) or 0.0) - limit_sep_m)
            ),
            float(item[1].get("fov", 0.0) or 0.0),
            float(item[1].get("sep", 0.0) or 0.0),
            float(item[1].get("vel", 0.0) or 0.0),
        ),
    )
    _ = best_score
    return dict(best_row)


def _line_db_row_covering_anchor_sep(
    planner: _HeadlessLinePlanner,
    *,
    width_m: float,
    required_sep_m: float,
) -> Dict[str, float] | None:
    target_width_m = max(0.0, float(width_m))
    target_sep_m = max(0.0, float(required_sep_m))
    if target_width_m <= 0.0 or target_sep_m <= 0.0:
        return None
    rows = [
        dict(row)
        for row in (planner._fov_db_rows() or [])
        if float(row.get("width", 0.0) or 0.0) + 1e-6 >= target_width_m
        and float(row.get("sep", 0.0) or 0.0) + 1e-6 >= target_sep_m
    ]
    if not rows:
        return None
    return min(
        rows,
        key=lambda row: (
            float(row.get("fov", 0.0) or 0.0),
            float(row.get("sep", 0.0) or 0.0),
            float(row.get("width", 0.0) or 0.0),
        ),
    )


def _line_entry_endpoints_xy(
    sweep_lines_xy: Sequence[Sequence[Tuple[float, float]]],
) -> List[Tuple[float, float]] | None:
    for sweep_xy in sweep_lines_xy:
        rows = [(float(x), float(y)) for x, y in sweep_xy]
        if len(rows) < 2:
            continue
        return [rows[0], rows[-1]]
    return None


def _resolve_line_entry_tprime_state(
    planner: _HeadlessLinePlanner,
    *,
    part_width_m: float,
    tangent_xy: Tuple[float, float] | None,
    entry_target_xy: Tuple[float, float] | None,
    entry_line_endpoints_xy: Sequence[Tuple[float, float]] | None,
    runtime_cfg: Dict[str, Any] | None = None,
) -> tuple[Tuple[float, float] | None, float | None, Dict[str, float] | None]:
    if tangent_xy is None or entry_target_xy is None:
        return None, None, None
    if not isinstance(entry_line_endpoints_xy, Sequence) or len(entry_line_endpoints_xy) < 2:
        return None, None, None

    ep0 = entry_line_endpoints_xy[0]
    ep1 = entry_line_endpoints_xy[1]
    tprime_sep_m = max(
        _distance_xy(tangent_xy, (float(ep0[0]), float(ep0[1]))),
        _distance_xy(tangent_xy, (float(ep1[0]), float(ep1[1]))),
    )
    target_sep_m = _next_collab_entry_tprime_target_sep_m(
        planner,
        float(part_width_m),
        runtime_cfg=runtime_cfg,
    )
    entry_t_prime_xy: Tuple[float, float] | None = None

    ingress_dx = float(entry_target_xy[0]) - float(tangent_xy[0])
    ingress_dy = float(entry_target_xy[1]) - float(tangent_xy[1])
    ingress_len_m = math.hypot(ingress_dx, ingress_dy)
    if tprime_sep_m > target_sep_m > 0.0 and ingress_len_m > 1e-6:
        ux_i = ingress_dx / ingress_len_m
        uy_i = ingress_dy / ingress_len_m
        lo = 0.0
        hi = float(ingress_len_m)
        for _ in range(40):
            mid = (lo + hi) * 0.5
            test_xy = (
                float(tangent_xy[0]) + (ux_i * mid),
                float(tangent_xy[1]) + (uy_i * mid),
            )
            test_sep_m = max(
                _distance_xy(test_xy, (float(ep0[0]), float(ep0[1]))),
                _distance_xy(test_xy, (float(ep1[0]), float(ep1[1]))),
            )
            if test_sep_m > target_sep_m:
                lo = mid
            else:
                hi = mid
        entry_t_prime_xy = (
            float(tangent_xy[0]) + (ux_i * lo),
            float(tangent_xy[1]) + (uy_i * lo),
        )
        tprime_sep_m = max(
            _distance_xy(entry_t_prime_xy, (float(ep0[0]), float(ep0[1]))),
            _distance_xy(entry_t_prime_xy, (float(ep1[0]), float(ep1[1]))),
        )

    sep_cand_m = float(tprime_sep_m)

    resolved_db_row = _next_collab_resolved_db_row(
        planner,
        float(part_width_m),
        float(sep_cand_m),
        runtime_cfg=runtime_cfg,
    )
    if not isinstance(resolved_db_row, dict):
        resolved_db_row = _next_collab_entry_tprime_db_row(
            planner,
            float(part_width_m),
            runtime_cfg=runtime_cfg,
        )
    return entry_t_prime_xy, float(sep_cand_m), dict(resolved_db_row) if isinstance(resolved_db_row, dict) else None


def _augment_line_path_row(
    planner: _HeadlessLinePlanner,
    piece: SplitPiece,
    overlay: Dict[str, Any],
    path_row: Dict[str, Any],
    *,
    shared_db_row: Dict[str, Any] | None = None,
    aircraft_row: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    out = dict(path_row)
    runtime_cfg = load_runtime_settings()
    forced_shared_row = dict(shared_db_row) if isinstance(shared_db_row, dict) and shared_db_row else None
    marker_rows = out.get("markerRows") if isinstance(out.get("markerRows"), list) else []
    centerline_xy = _piece_centerline_xy(piece)
    target_xy = _piece_target_xy(piece)
    target_face_raw = out.get("targetFaceXY")
    target_face_xy = (
        (float(target_face_raw[0]), float(target_face_raw[1]))
        if isinstance(target_face_raw, (tuple, list)) and len(target_face_raw) >= 2
        else None
    )
    tangent_raw = out.get("tangentXY") or out.get("waypointStartXY")
    tangent_xy = (
        (float(tangent_raw[0]), float(tangent_raw[1]))
        if isinstance(tangent_raw, (tuple, list)) and len(tangent_raw) >= 2
        else None
    )
    if tangent_xy is None:
        tangent_xy = _marker_xy(marker_rows, "tangent")
    waypoint_start_xy = (
        (float(out["waypointStartXY"][0]), float(out["waypointStartXY"][1]))
        if isinstance(out.get("waypointStartXY"), (tuple, list)) and len(out["waypointStartXY"]) >= 2
        else tangent_xy
    )
    waypoint_end_xy = (
        (float(out["waypointEndXY"][0]), float(out["waypointEndXY"][1]))
        if isinstance(out.get("waypointEndXY"), (tuple, list)) and len(out["waypointEndXY"]) >= 2
        else target_face_xy
    )
    reference_xy = waypoint_start_xy or tangent_xy or target_face_xy
    if len(centerline_xy) >= 2:
        centerline_xy = _orient_polyline_xy(centerline_xy, reference_xy=reference_xy)
    piece_width_m = float(_piece_width_m(piece))
    overlay_width_m = float(overlay.get("maxWidthM", 0.0) or overlay.get("widthM", 0.0) or 0.0)
    width_ref_m = float(piece_width_m if piece_width_m > 0.0 else overlay_width_m)
    if width_ref_m <= 0.0:
        width_ref_m = float(overlay_width_m)
    initial_sep_hint_m = _to_float(out.get("sepCandM")) or _to_float(out.get("dbSepM")) or 0.0
    db_row = _next_collab_resolved_db_row(
        planner,
        float(width_ref_m),
        float(initial_sep_hint_m),
        runtime_cfg=runtime_cfg,
    ) or planner._covering_db_row(width_ref_m) or {}
    db_sep_m = float(db_row.get("sep", out.get("dbSepM", 0.0)) or 0.0)
    if db_sep_m <= 0.0:
        db_sep_m = max(30.0, float(width_ref_m) * 0.5)
    manual_line_base_fov_deg = _runtime_line_manual_base_fov_deg(runtime_cfg)
    manual_line_fov_deg = _runtime_line_manual_fov_deg(runtime_cfg)
    resolved_base_fov_deg = _clamp_line_fov_deg(
        manual_line_base_fov_deg or db_row.get("fov", out.get("resolvedFovDeg", 0.0)),
        MIN_LINE_FOV_DEG,
    )
    if resolved_base_fov_deg <= 0.0:
        resolved_base_fov_deg = _clamp_line_fov_deg(db_row.get("fov", 0.0), MIN_LINE_FOV_DEG)
    resolved_fov_deg = _clamp_line_fov_deg(
        manual_line_fov_deg
        or apply_runtime_camera_adjusted_fov_deg(
            resolved_base_fov_deg,
            runtime_cfg,
            minimum_fov_deg=MIN_LINE_FOV_DEG,
            context="NEXTCOLLAB LINE RESOLVED",
        )
        or 0.0,
        resolved_base_fov_deg,
    )
    route_wp_spacing_m = _runtime_line_route_wp_spacing_m(runtime_cfg)

    route_xy = out.get("routeXY") if isinstance(out.get("routeXY"), list) else []
    route_origin_xy = (
        (float(route_xy[0][0]), float(route_xy[0][1]))
        if route_xy and isinstance(route_xy[0], (tuple, list)) and len(route_xy[0]) >= 2
        else None
    )
    aircraft_entry_xy = _first_xy_from_aircraft_row(aircraft_row)
    origin_xy = aircraft_entry_xy or route_origin_xy
    entry_t_prime_xy = (
        (float(out["entryTPrimeXY"][0]), float(out["entryTPrimeXY"][1]))
        if isinstance(out.get("entryTPrimeXY"), (tuple, list)) and len(out["entryTPrimeXY"]) >= 2
        else None
    )
    sep_cand_m = _to_float(out.get("sepCandM"))
    explicit_route_offset_sep_m = _to_float(out.get("lineRouteOffsetSepM"))
    # sepCandM is an ingress/T' clearance candidate. Keep route offset tied to
    # the sweep DB separation so one aircraft's entry geometry cannot push the
    # LINE anchors far away from the corridor.
    route_offset_sep_m = _route_offset_sep_for_fov(
        planner,
        resolved_base_fov_deg or resolved_fov_deg,
        (
            explicit_route_offset_sep_m
            or _to_float(out.get("dbSepM"))
            or db_sep_m
            or 0.0
        ),
        runtime_cfg=runtime_cfg,
    )
    corridor_sweeps_xy: List[List[Tuple[float, float]]] = []
    corridor_sweep_items: List[Dict[str, Any]] = []
    first_anchor_xy: Tuple[float, float] | None = None
    line_offset_plan: _LineOffsetPlan | None = None
    first_anchor_clearance_m: float | None = None
    active_db_row = dict(forced_shared_row) if forced_shared_row is not None else (dict(db_row) if isinstance(db_row, dict) else {})

    def _db_row_signature(row: Dict[str, Any]) -> tuple[float, float, float]:
        return (
            round(float(row.get("width", 0.0) or 0.0), 6),
            round(float(row.get("sep", 0.0) or 0.0), 6),
            round(float(row.get("fov", 0.0) or 0.0), 6),
        )

    for attempt in range(1 if forced_shared_row is not None else 3):
        active_db_sep_m = float(active_db_row.get("sep", db_sep_m) or db_sep_m or 0.0)
        if active_db_sep_m <= 0.0:
            active_db_sep_m = max(30.0, float(width_ref_m) * 0.5)
        active_base_fov_deg = _clamp_line_fov_deg(
            manual_line_base_fov_deg
            or active_db_row.get("fov", resolved_base_fov_deg)
            or resolved_base_fov_deg
            or 0.0,
            resolved_base_fov_deg,
        )
        active_fov_deg = _clamp_line_fov_deg(
            manual_line_fov_deg
            or apply_runtime_camera_adjusted_fov_deg(
                active_base_fov_deg,
                runtime_cfg,
                minimum_fov_deg=MIN_LINE_FOV_DEG,
                context="NEXTCOLLAB LINE ACTIVE_DB",
            )
            or 0.0,
            active_base_fov_deg,
        )
        if active_fov_deg <= 0.0:
            active_fov_deg = _clamp_line_fov_deg(resolved_fov_deg)
        active_raw_sweep_spacing_m = _line_sweep_spacing_m(
            separation_m=float(active_db_sep_m),
            fov_deg=float(active_fov_deg or 1.0),
            runtime_cfg=runtime_cfg,
        )
        active_route_offset_sep_m = _route_offset_sep_for_fov(
            planner,
            active_base_fov_deg or active_fov_deg,
            (
                explicit_route_offset_sep_m
                or route_offset_sep_m
                or active_db_sep_m
            ),
            runtime_cfg=runtime_cfg,
        )
        active_route_offset_m = max(
            float(active_route_offset_sep_m) * float(_runtime_line_route_offset_scale(runtime_cfg)),
            1.0,
        )

        (
            corridor_sweeps_xy,
            corridor_sweep_items,
            first_anchor_xy,
            line_offset_plan,
            first_anchor_clearance_m,
        ) = _centerline_corridor_sweep_lines_xy(
            planner,
            piece,
            sweep_spacing_m=float(active_raw_sweep_spacing_m),
            route_offset_m=float(active_route_offset_m),
            origin_xy=origin_xy,
            db_sep_m=float(active_db_sep_m),
            aircraft_row=aircraft_row,
        )
        if not corridor_sweeps_xy:
            (
                corridor_sweeps_xy,
                corridor_sweep_items,
                first_anchor_xy,
                line_offset_plan,
                first_anchor_clearance_m,
            ) = _corridor_sweep_lines_xy(
                planner,
                piece,
                overlay,
                sweep_spacing_m=float(active_raw_sweep_spacing_m),
                route_offset_m=float(active_route_offset_m),
                origin_xy=origin_xy,
                db_sep_m=float(active_db_sep_m),
                aircraft_row=aircraft_row,
            )
        entry_line_endpoints_xy = _line_entry_endpoints_xy(corridor_sweeps_xy)
        entry_target_xy = (
            (float(first_anchor_xy[0]), float(first_anchor_xy[1]))
            if isinstance(first_anchor_xy, tuple)
            else _midpoint_xy(entry_line_endpoints_xy or [])
        )
        if entry_target_xy is None:
            entry_target_xy = waypoint_end_xy or target_face_xy or target_xy

        computed_entry_t_prime_xy, computed_sep_cand_m, next_db_row = _resolve_line_entry_tprime_state(
            planner,
            part_width_m=float(width_ref_m),
            tangent_xy=tangent_xy,
            entry_target_xy=entry_target_xy,
            entry_line_endpoints_xy=entry_line_endpoints_xy,
            runtime_cfg=runtime_cfg,
        )
        if computed_entry_t_prime_xy is not None:
            entry_t_prime_xy = computed_entry_t_prime_xy
            waypoint_start_xy = entry_t_prime_xy
        if computed_sep_cand_m is not None:
            sep_cand_m = float(computed_sep_cand_m)
        if (
            first_anchor_clearance_m is not None
            and float(first_anchor_clearance_m) > float(active_db_sep_m) + 1e-6
        ):
            anchor_cover_row = _line_db_row_covering_anchor_sep(
                planner,
                width_m=float(width_ref_m),
                required_sep_m=float(first_anchor_clearance_m),
            )
            if isinstance(anchor_cover_row, dict):
                next_db_row = anchor_cover_row

        db_sep_m = float(active_db_sep_m)
        resolved_base_fov_deg = float(active_base_fov_deg)
        resolved_fov_deg = float(active_fov_deg)
        raw_sweep_spacing_m = float(active_raw_sweep_spacing_m)
        route_offset_sep_m = float(active_route_offset_sep_m)
        route_offset_m = float(active_route_offset_m)
        if line_offset_plan is not None and float(line_offset_plan.offset_m) > 0.0:
            route_offset_m = float(line_offset_plan.offset_m)

        if forced_shared_row is not None:
            break
        if not isinstance(next_db_row, dict):
            break
        if _db_row_signature(next_db_row) == _db_row_signature(active_db_row):
            active_db_row = dict(next_db_row)
            break
        if attempt >= 2:
            break
        active_db_row = dict(next_db_row)

    final_db_row = dict(forced_shared_row) if forced_shared_row is not None else (active_db_row if isinstance(active_db_row, dict) else {})
    final_base_fov_deg = _clamp_line_fov_deg(
        manual_line_base_fov_deg
        or final_db_row.get("fov", resolved_base_fov_deg)
        or resolved_base_fov_deg
        or 0.0,
        resolved_base_fov_deg,
    )
    resolved_fov_deg = _clamp_line_fov_deg(
        manual_line_fov_deg
        or apply_runtime_camera_adjusted_fov_deg(
            final_base_fov_deg,
            runtime_cfg,
            minimum_fov_deg=MIN_LINE_FOV_DEG,
            context="NEXTCOLLAB LINE FINAL",
        )
        or 0.0,
        final_base_fov_deg,
    )
    out["resolvedBaseFovDeg"] = float(final_base_fov_deg)
    out["resolvedFovDeg"] = float(resolved_fov_deg)
    out["dbSepM"] = float(final_db_row.get("sep", out.get("dbSepM", db_sep_m)) or db_sep_m or 0.0)
    out["dbWidthM"] = float(final_db_row.get("width", out.get("dbWidthM", width_ref_m)) or width_ref_m or 0.0)
    if forced_shared_row is not None:
        sep_cand_m = float(final_db_row.get("sep", sep_cand_m or 0.0) or 0.0)
    if out.get("resolvedVelMps") is None or float(out.get("resolvedVelMps", 0.0) or 0.0) <= 0.0:
        out["resolvedVelMps"] = float(
            apply_runtime_camera_adjusted_search_speed(
                float(final_db_row.get("vel", 0.0) or 0.0),
                final_base_fov_deg if final_base_fov_deg > 0.0 else resolved_fov_deg,
                runtime_cfg,
                adjusted_fov_deg=resolved_fov_deg,
            )
            * float(_runtime_line_search_speed_weight(runtime_cfg))
        )

    if corridor_sweeps_xy:
        out["sweepLineListXY"] = corridor_sweeps_xy
    elif not isinstance(out.get("sweepLineListXY"), list) or not out.get("sweepLineListXY"):
        if len(centerline_xy) >= 2:
            out["sweepLineListXY"] = [centerline_xy]
    else:
        out["sweepLineListXY"] = [
            [(float(x), float(y)) for x, y in line_xy]
            for line_xy in out.get("sweepLineListXY") or []
            if isinstance(line_xy, list) and len(line_xy) >= 2
        ]

    if target_face_xy is not None:
        out["targetFaceXY"] = target_face_xy
    if target_xy is not None:
        out["targetXY"] = (float(target_xy[0]), float(target_xy[1]))
    elif target_face_xy is not None:
        out["targetXY"] = target_face_xy
    if tangent_xy is not None:
        out["tangentXY"] = tangent_xy
    if waypoint_start_xy is not None:
        out["waypointStartXY"] = waypoint_start_xy
    if first_anchor_xy is not None:
        out["waypointEndXY"] = (float(first_anchor_xy[0]), float(first_anchor_xy[1]))
    elif waypoint_end_xy is not None:
        out["waypointEndXY"] = waypoint_end_xy
    if entry_line_endpoints_xy is not None:
        out["entryLineEndpointsXY"] = [
            (float(entry_line_endpoints_xy[0][0]), float(entry_line_endpoints_xy[0][1])),
            (float(entry_line_endpoints_xy[1][0]), float(entry_line_endpoints_xy[1][1])),
        ]

    out["source"] = str(out.get("source") or "line_make_path")
    out["partWidthM"] = float(width_ref_m)
    out["pieceWidthM"] = float(piece_width_m)
    out["overlayWidthM"] = float(overlay_width_m)
    out["dbWidthM"] = float(out.get("dbWidthM", db_row.get("width", width_ref_m)) or 0.0)
    out["dbSepM"] = float(out.get("dbSepM", db_sep_m) or 0.0)
    out["lineSweepSpacingM"] = float(raw_sweep_spacing_m)
    out["lineRouteOffsetSepM"] = float(route_offset_sep_m)
    out["lineRouteOffsetM"] = float(route_offset_m)
    out["lineRouteWpSpacingM"] = float(route_wp_spacing_m)
    if line_offset_plan is not None:
        out["lineRouteOffsetStartSide"] = float(line_offset_plan.start_side)
        out["lineRouteOffsetEndSide"] = float(line_offset_plan.end_side)
        out["lineRouteOffsetPlanScore"] = float(line_offset_plan.score)
        out["lineRouteOffsetFirstTurnSign"] = int(line_offset_plan.first_turn_sign)
        if line_offset_plan.dubins_length_m is not None:
            out["lineRouteOffsetDubinsLengthM"] = float(line_offset_plan.dubins_length_m)
        if first_anchor_clearance_m is not None:
            out["lineRouteFirstAnchorClearanceM"] = float(first_anchor_clearance_m)
    out["resolvedBaseFovDeg"] = float(final_base_fov_deg)
    out["resolvedFovDeg"] = float(resolved_fov_deg)
    if out.get("resolvedVelMps") is None:
        out["resolvedVelMps"] = float(
            apply_runtime_camera_adjusted_search_speed(
                float(db_row.get("vel", 0.0) or 0.0),
                final_base_fov_deg if final_base_fov_deg > 0.0 else resolved_fov_deg,
                runtime_cfg,
                adjusted_fov_deg=resolved_fov_deg,
            )
            * float(_runtime_line_search_speed_weight(runtime_cfg))
        )
    out["bearingDeg"] = float(out.get("bearingDeg", overlay.get("bearingDeg", 0.0)) or 0.0)
    if out.get("horizonSec") is None:
        out["horizonSec"] = 0.0
    if out.get("branch") is None:
        out["branch"] = "direct"
    if sep_cand_m is not None:
        out["sepCandM"] = float(sep_cand_m)
    if tangent_xy is not None and waypoint_start_xy is not None:
        out["entryTPrimeXY"] = (
            entry_t_prime_xy
            if entry_t_prime_xy is not None
            else ((float(waypoint_start_xy[0]), float(waypoint_start_xy[1])) if _distance_xy(tangent_xy, waypoint_start_xy) > 1e-6 else None)
        )
        if out["entryTPrimeXY"] is not None:
            out["startLabel"] = "T'"
    if len(centerline_xy) >= 2:
        out["centerLineXY"] = list(centerline_xy)
    line_sweep_items = list(corridor_sweep_items or [])
    if not line_sweep_items:
        line_sweep_items = _build_line_sweep_items(
            out.get("sweepLineListXY") or [],
            reference_xy=reference_xy,
            offset_m=float(route_offset_m),
            route_line_xy=centerline_xy,
            offset_plan=line_offset_plan,
        )
    if line_sweep_items:
        if len(centerline_xy) >= 2:
            route_points_xy = _resample_polyline_xy(
                centerline_xy,
                spacing_m=float(route_wp_spacing_m),
            )
            sweep_midpoints_xy: List[Tuple[float, float]] = []
            for item in line_sweep_items:
                sweep_xy = item.get("sweepXY") if isinstance(item, dict) else []
                midpoint_xy = _midpoint_xy(sweep_xy if isinstance(sweep_xy, list) else [])
                sweep_midpoints_xy.append(midpoint_xy or (0.0, 0.0))
            selected_indices = _select_route_sweep_indices(route_points_xy, sweep_midpoints_xy)
            if selected_indices:
                selected_items: List[Dict[str, Any]] = []
                for idx in selected_indices:
                    if idx < 0 or idx >= len(line_sweep_items):
                        continue
                    item = dict(line_sweep_items[idx])
                    item["sweepIndex"] = int(idx)
                    selected_items.append(item)
                out["lineSweepItemsXY"] = selected_items
            else:
                for idx, item in enumerate(line_sweep_items):
                    item["sweepIndex"] = int(idx)
                out["lineSweepItemsXY"] = line_sweep_items
        else:
            for idx, item in enumerate(line_sweep_items):
                item["sweepIndex"] = int(idx)
            out["lineSweepItemsXY"] = line_sweep_items
    return out


def _shared_line_db_row_from_expected_rows(
    expected_rows: Sequence[Dict[str, Any]],
    *,
    runtime_cfg: Dict[str, Any] | None = None,
) -> Dict[str, Any] | None:
    candidates: List[Dict[str, Any]] = []
    required_sep_m = 0.0
    required_width_m = 0.0
    for row in expected_rows:
        if not isinstance(row, dict):
            continue
        spacing_m = _to_float(row.get("lineSweepSpacingM"))
        fov_deg = _to_float(row.get("resolvedFovDeg"))
        sep_m = _to_float(row.get("dbSepM"))
        width_m = _to_float(row.get("dbWidthM")) or _to_float(row.get("partWidthM"))
        sep_cand_m = _to_float(row.get("sepCandM"))
        part_width_m = _to_float(row.get("partWidthM"))
        vel_mps = _to_float(row.get("resolvedVelMps"))
        if spacing_m is None or spacing_m <= 0.0:
            continue
        if fov_deg is None or fov_deg <= 0.0:
            continue
        if sep_m is None or sep_m <= 0.0:
            continue
        if sep_cand_m is not None and sep_cand_m > 0.0:
            required_sep_m = max(float(required_sep_m), _db_sep_requirement_m(float(sep_cand_m), runtime_cfg))
        if part_width_m is not None and part_width_m > 0.0:
            required_width_m = max(float(required_width_m), float(part_width_m))
        candidates.append(
            {
                "spacing": float(spacing_m),
                "fov": float(fov_deg),
                "sep": float(sep_m),
                "width": float(width_m or 0.0),
                "vel": float(vel_mps or 0.0),
            }
        )
    if not candidates:
        return None
    covering_candidates = [
        item for item in candidates
        if float(item["sep"]) + 1e-6 >= float(required_sep_m)
        and float(item["width"]) + 1e-6 >= float(required_width_m)
    ]
    pool = covering_candidates or candidates
    best = min(
        pool,
        key=lambda item: (
            float(item["spacing"]),
            float(item["sep"]),
            -float(item["fov"]),
        ),
    )
    return {
        "width": float(best["width"]),
        "sep": float(best["sep"]),
        "fov": float(best["fov"]),
        "vel": float(best["vel"]),
    }


def _bind_scaled_turn_methods(
    planner: _HeadlessLinePlanner,
    *,
    turn_radius_scale: float,
) -> None:
    scale = max(0.1, float(turn_radius_scale))
    base_radius_m = float(TURN_PREVIEW_RADIUS_M) * scale
    original_turn_prediction = planner._turn_prediction_points_xy

    def _turn_prediction_points_xy_scaled(
        self: _HeadlessLinePlanner,
        origin_xy: Tuple[float, float],
        bearing_deg: float,
        *,
        speed_mps: float = TURN_PREVIEW_SPEED_MPS,
        horizon_s: float = TURN_PREVIEW_HORIZON_S,
        radius_m: float = TURN_PREVIEW_RADIUS_M,
    ) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        effective_radius = float(radius_m)
        if abs(effective_radius - float(TURN_PREVIEW_RADIUS_M)) <= 1e-6:
            effective_radius = float(base_radius_m)
        return original_turn_prediction(
            origin_xy,
            bearing_deg,
            speed_mps=speed_mps,
            horizon_s=horizon_s,
            radius_m=effective_radius,
        )

    def _find_visibility_segment_scaled(
        self: _HeadlessLinePlanner,
        aircraft_id: int,
        origin_xy: Tuple[float, float],
        bearing_deg: float,
        target_xy: Tuple[float, float],
    ) -> Optional[Dict[str, Any]]:
        if self._line_avoids_turn_circles(
            origin_xy,
            target_xy,
            origin_xy,
            bearing_deg,
            radius_m=base_radius_m,
        ):
            return {
                "aircraftID": int(aircraft_id),
                "startXY": origin_xy,
                "endXY": target_xy,
                "horizonSec": 0.0,
                "branch": "direct",
                "turnPoints": [],
            }

        max_steps = max(
            1,
            int(
                math.ceil(
                    (2.0 * math.pi * float(base_radius_m))
                    / (float(TURN_PREVIEW_SPEED_MPS) * float(TURN_PREVIEW_HORIZON_S))
                )
            ),
        )
        preferred_branch = self._turn_branch_toward_target(origin_xy, bearing_deg, target_xy)
        for step_idx in range(1, max_steps + 1):
            horizon_s = float(step_idx) * float(TURN_PREVIEW_HORIZON_S)
            left_xy, right_xy = self._turn_prediction_points_xy(
                origin_xy,
                bearing_deg,
                horizon_s=horizon_s,
                radius_m=base_radius_m,
            )
            candidates: List[Dict[str, Any]] = []
            branch_points = (("L", left_xy), ("R", right_xy))
            # LINE ingress must not accept the opposite turn branch once the
            # target-side branch is known; that can place T/T' outside the
            # mission corridor when an aircraft is already turning away.
            if preferred_branch == "L":
                branch_points = (("L", left_xy),)
            elif preferred_branch == "R":
                branch_points = (("R", right_xy),)
            for branch, candidate_xy in branch_points:
                if not self._line_avoids_turn_circles(
                    candidate_xy,
                    target_xy,
                    origin_xy,
                    bearing_deg,
                    radius_m=base_radius_m,
                ):
                    continue
                turn_points: List[Tuple[float, float]] = []
                turn_point_horizons: List[float] = []
                for prior_idx in range(1, step_idx + 1):
                    prior_horizon_s = float(prior_idx) * float(TURN_PREVIEW_HORIZON_S)
                    prior_left_xy, prior_right_xy = self._turn_prediction_points_xy(
                        origin_xy,
                        bearing_deg,
                        horizon_s=prior_horizon_s,
                        radius_m=base_radius_m,
                    )
                    turn_points.append(prior_left_xy if branch == "L" else prior_right_xy)
                    turn_point_horizons.append(float(prior_horizon_s))
                start_xy = candidate_xy
                start_horizon_s = float(horizon_s)
                if step_idx >= 2:
                    refined = self._refine_visibility_start_xy(
                        origin_xy,
                        bearing_deg,
                        target_xy,
                        branch=branch,
                        min_horizon_s=float(step_idx - 1) * float(TURN_PREVIEW_HORIZON_S),
                        max_horizon_s=horizon_s,
                        radius_m=base_radius_m,
                    )
                    if refined is not None:
                        start_xy = refined["startXY"]
                        start_horizon_s = float(refined["horizonSec"])
                        if turn_points:
                            turn_points[-1] = start_xy
                            turn_point_horizons[-1] = start_horizon_s
                        if (
                            len(turn_points) >= 3
                            and (turn_point_horizons[-1] - turn_point_horizons[-2])
                            < (float(TURN_PREVIEW_HORIZON_S) - 1e-6)
                        ):
                            del turn_points[-2]
                            del turn_point_horizons[-2]
                candidates.append(
                    {
                        "aircraftID": int(aircraft_id),
                        "startXY": start_xy,
                        "endXY": target_xy,
                        "horizonSec": float(start_horizon_s),
                        "branch": branch,
                        "turnPoints": turn_points,
                    }
                )
            if candidates:
                return min(
                    candidates,
                    key=lambda row: (
                        float(row.get("horizonSec", 0.0) or 0.0),
                        _distance_xy(row["startXY"], target_xy),
                    ),
                )
        return None

    planner._turn_prediction_points_xy = MethodType(_turn_prediction_points_xy_scaled, planner)
    planner._find_visibility_segment = MethodType(_find_visibility_segment_scaled, planner)


@contextmanager
def _silent_message_boxes():
    original_warning = QMessageBox.warning
    original_information = QMessageBox.information
    original_critical = QMessageBox.critical
    try:
        QMessageBox.warning = staticmethod(lambda *args, **kwargs: 0)  # type: ignore[assignment]
        QMessageBox.information = staticmethod(lambda *args, **kwargs: 0)  # type: ignore[assignment]
        QMessageBox.critical = staticmethod(lambda *args, **kwargs: 0)  # type: ignore[assignment]
        yield
    finally:
        QMessageBox.warning = original_warning  # type: ignore[assignment]
        QMessageBox.information = original_information  # type: ignore[assignment]
        QMessageBox.critical = original_critical  # type: ignore[assignment]


def run_next_collab_line_plan(
    *,
    target_mission: Dict[str, Any] | None = None,
    mission_detail: Dict[str, Any] | None = None,
    aircraft_entries: List[Dict[str, Any]],
    turn_radius_scale: float | None = None,
    log: Callable[[str], None] | None = None,
) -> NextCollabLinePlanResult:
    mission_source: Dict[str, Any]
    if isinstance(target_mission, dict):
        mission_source = target_mission
    elif isinstance(mission_detail, dict):
        mission_source = {"missionDetail": mission_detail}
    else:
        raise RuntimeError("next-collab line planner requires a target mission or missionDetail payload.")

    if isinstance(mission_detail, dict) and not isinstance(mission_source.get("missionDetail"), dict):
        mission_source = dict(mission_source)
        mission_source["missionDetail"] = mission_detail

    runtime_cfg = load_runtime_settings()
    mission_payload, line_specs = _normalize_line_specs(mission_source)
    mission_detail_payload = (
        mission_payload.get("missionDetail")
        if isinstance(mission_payload.get("missionDetail"), dict)
        else {}
    )
    source_line_width_m = _resolve_source_line_width_m(mission_payload, mission_detail_payload, line_specs)
    source_coordinate_list = _resolve_source_coordinate_list(mission_detail_payload, line_specs)
    mission_center_xy = _mission_center_xy(line_specs)
    aircraft_rows = _normalize_aircraft_rows(aircraft_entries, mission_center_xy=mission_center_xy)
    _ensure(bool(aircraft_rows), "next-collab line planner requires at least one aircraft entry.")
    aircraft_row_by_id = {int(row["aircraftID"]): dict(row) for row in aircraft_rows}

    planner = _HeadlessLinePlanner()
    planner.hide()
    planner.state.mission_kind = MISSION_LINE
    planner.state.mode = MODE_MISSION_READY
    planner.state.mission_points_xy = [
        point_xy
        for spec in line_specs
        for point_xy in coords_to_xy(spec.get("coordinateList") or [])
    ]
    planner.state.line_width_m = max(float(spec.get("width", 1.0) or 1.0) for spec in line_specs)
    planner.state.uav_ids = [int(row["aircraftID"]) for row in aircraft_rows]
    planner.state.uav_positions_xy = [row["positionXY"] for row in aircraft_rows]
    planner.state.uav_heading_deg = [float(row["headingDeg"]) for row in aircraft_rows]
    planner._selected_uav_count = max(1, len(planner.state.uav_ids))
    _bind_scaled_turn_methods(planner, turn_radius_scale=float(turn_radius_scale or 1.2))

    cmpk_payload = {
        "availableAircraftList": [{"aircraftID": int(row["aircraftID"])} for row in aircraft_rows],
        "inputMissionList": [mission_payload],
    }
    mrpk_payload = {
        "takeOverInfoList": [
            {
                "aircraftID": int(row["aircraftID"]),
                "coordinate": dict(row["coordinate"]),
                "headingDeg": float(row["headingDeg"]),
            }
            for row in aircraft_rows
        ]
    }
    planner._cmpk_payload = cmpk_payload
    planner._mrpk_payload = mrpk_payload

    with _silent_message_boxes():
        split_result = run_split_pipeline(
            cmpk_payload,
            mrpk_payload,
            list(planner.state.uav_ids),
            apply_assignment=False,
            apply_scheduling=False,
        )
        _ensure(
            split_result is not None and bool(split_result.pieces),
            "next-collab line planner failed to produce split pieces.",
        )
        assign_report = planner._assign_split_result_by_prediction_distance(split_result)
        planner.state.split_result = split_result

        try:
            reference_bearing_deg, overlays, mid_lines = planner._mid_line_overlay_bundle(split_result)
        except Exception:
            reference_bearing_deg = _line_reference_bearing_deg(line_specs, split_result.pieces)
            overlays = []
            mid_lines = []
        if not isinstance(reference_bearing_deg, (int, float)):
            reference_bearing_deg = _line_reference_bearing_deg(line_specs, split_result.pieces)

        overlay_by_piece: Dict[int, Dict[str, Any]] = {
            int(row.get("pieceIndex", 0) or 0): dict(row)
            for row in overlays
            if isinstance(row, dict)
        }
        for piece in split_result.pieces:
            piece_idx = int(piece.piece_index or 0)
            if piece_idx in overlay_by_piece:
                continue
            synthetic_overlay = _synthetic_line_overlay(piece, reference_bearing_deg=float(reference_bearing_deg))
            if synthetic_overlay is None:
                continue
            overlay_by_piece[piece_idx] = synthetic_overlay
            overlays.append(synthetic_overlay)
        overlays = sorted(
            (dict(row) for row in overlays if isinstance(row, dict)),
            key=lambda row: int(row.get("pieceIndex", 0) or 0),
        )
        planner.state.mid_line_segments = overlays

        expected_rows: List[Dict[str, Any]] = []
        piece_by_index: Dict[int, SplitPiece] = {}
        for piece in sorted(split_result.pieces, key=lambda row: int(row.piece_index or 0)):
            piece_by_index[int(piece.piece_index or 0)] = piece
            overlay = overlay_by_piece.get(int(piece.piece_index or 0))
            if overlay is None:
                continue
            path_row = planner._make_path_row(piece, overlay)
            if not isinstance(path_row, dict):
                continue
            piece_aircraft_id = int(piece.assigned_uav or path_row.get("aircraftID") or 0)
            expected_rows.append(
                _augment_line_path_row(
                    planner,
                    piece,
                    overlay,
                    path_row,
                    aircraft_row=aircraft_row_by_id.get(piece_aircraft_id),
                )
            )
        if not expected_rows:
            if log is not None:
                log("[NEXTCOLLAB][LINE] retrying path rows with alternate overlay target points.")
            for piece in sorted(split_result.pieces, key=lambda row: int(row.piece_index or 0)):
                overlay = overlay_by_piece.get(int(piece.piece_index or 0))
                if overlay is None:
                    continue
                path_row = planner._make_path_row(piece, overlay, allow_alt_targets=True)
                if not isinstance(path_row, dict):
                    continue
                piece_aircraft_id = int(piece.assigned_uav or path_row.get("aircraftID") or 0)
                expected_rows.append(
                    _augment_line_path_row(
                        planner,
                        piece,
                        overlay,
                        path_row,
                        aircraft_row=aircraft_row_by_id.get(piece_aircraft_id),
                    )
                )
        _ensure(bool(expected_rows), "next-collab line planner produced no path rows.")
        shared_db_row = _shared_line_db_row_from_expected_rows(expected_rows, runtime_cfg=runtime_cfg)
        if isinstance(shared_db_row, dict) and len(expected_rows) >= 2:
            shared_expected_rows: List[Dict[str, Any]] = []
            for row in expected_rows:
                piece_idx = int(row.get("pieceIndex", 0) or 0)
                piece = piece_by_index.get(piece_idx)
                overlay = overlay_by_piece.get(piece_idx)
                if piece is None or overlay is None:
                    shared_expected_rows.append(dict(row))
                    continue
                shared_expected_rows.append(
                    _augment_line_path_row(
                        planner,
                        piece,
                        overlay,
                        row,
                        shared_db_row=shared_db_row,
                        aircraft_row=aircraft_row_by_id.get(int(row.get("aircraftID") or piece.assigned_uav or 0)),
                    )
                )
            expected_rows = shared_expected_rows
        for row in expected_rows:
            if not isinstance(row, dict):
                continue
            row["sourceLineWidthM"] = float(source_line_width_m)
            if len(source_coordinate_list) >= 2:
                row["sourceCoordinateList"] = copy.deepcopy(source_coordinate_list)
        split_result.expected_paths = list(expected_rows)
        planner.state.expected_paths = list(expected_rows)

    workflow = "line_prediction_path" if len(expected_rows) > 1 else "line_single_path"
    result_lines = [
        "[NEXTCOLLAB][LINE] split "
        f"pieces={len(split_result.pieces)} "
        f"assignment={planner._assignment_summary_text(split_result)}",
        "[NEXTCOLLAB][LINE] prediction-assign "
        f"assigned={int(assign_report.get('assignedPieces', 0))}/"
        f"{int(assign_report.get('pieceCount', 0))}",
        f"[NEXTCOLLAB][LINE] mid-line refBearing={float(reference_bearing_deg):.1f}deg overlays={len(overlays)}",
        f"[NEXTCOLLAB][LINE] path rows={len(expected_rows)}",
    ]
    shared_db_row = _shared_line_db_row_from_expected_rows(expected_rows, runtime_cfg=runtime_cfg)
    if isinstance(shared_db_row, dict):
        result_lines.append(
            "[NEXTCOLLAB][LINE] shared sweep row "
            f"FOV={float(shared_db_row.get('fov', 0.0) or 0.0):.1f} "
            f"SEP={float(shared_db_row.get('sep', 0.0) or 0.0):.1f}m"
        )
    for row in expected_rows:
        if not isinstance(row, dict):
            continue
        base_fov = row.get("resolvedBaseFovDeg")
        adjusted_fov = row.get("resolvedFovDeg")
        if base_fov is None or adjusted_fov is None:
            continue
        fov_adjust_log = format_runtime_camera_fov_adjustment_log(
            base_fov,
            adjusted_fov,
            runtime_cfg,
            context=(
                f"NEXTCOLLAB LINE UAV{int(row.get('aircraftID', 0) or 0)}"
                f"/P{int(row.get('pieceIndex', 0) or 0)}"
            ),
        )
        if fov_adjust_log:
            result_lines.append(fov_adjust_log)
    for piece_line in assign_report.get("pieceLines") or []:
        result_lines.append(str(piece_line))
    for line in result_lines:
        if log is not None:
            log(line)

    planner_result_text = "\n".join(result_lines)
    planner._set_result(planner_result_text)
    return NextCollabLinePlanResult(
        workflow=workflow,
        split_result=split_result,
        mid_line_segments=list(overlays),
        expected_paths=list(expected_rows),
        planner_result_text=str(planner._result_text_buffer or planner_result_text),
    )


__all__ = [
    "NextCollabLinePlanResult",
    "run_next_collab_line_plan",
]
