from __future__ import annotations

import copy
import math
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional

from PyQt5.QtWidgets import QMessageBox

from modules.mission_planning.runtime.next_collab_heading import (
    monitor_heading_to_planner_bearing_deg,
)
from modules.mission_planning.MissionPlanner.runtime_settings import (
    format_runtime_camera_fov_adjustment_log,
)
from modules.mission_planning.planners.next_collab_division._canvas_state import CanvasState
from modules.mission_planning.planners.next_collab_division._constants import (
    MISSION_AREA,
    MODE_MISSION_READY,
    SplitRunResult,
)
from modules.mission_planning.planners.next_collab_division._geo_utils import (
    coord_to_xy,
    coords_to_xy,
)
from modules.mission_planning.planners.next_collab_division._planner_window import (
    DivisionPlannerWindow,
)


@dataclass
class NextCollabDivisionPlanResult:
    workflow: str
    split_result: SplitRunResult
    mid_line_segments: List[Dict[str, Any]]
    expected_paths: List[Dict[str, Any]]
    planner_result_text: str


class _HeadlessDivisionPlanner:
    def __init__(self) -> None:
        self.state = CanvasState(line_width_m=300.0)
        self._turn_radius_scale = 1.4
        self._selected_uav_count = 1
        self._cmpk_payload = None
        self._mrpk_payload = None
        self._fov_db_rows_cache = None
        self._fov_db_widths_cache = None
        self._mid_line_no_split_active = False
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


for _name, _value in DivisionPlannerWindow.__dict__.items():
    if _name.startswith("__"):
        continue
    if hasattr(_HeadlessDivisionPlanner, _name):
        continue
    if callable(_value):
        setattr(_HeadlessDivisionPlanner, _name, _value)


def _bearing_deg_from_xy(
    start_xy: tuple[float, float],
    end_xy: tuple[float, float],
) -> float:
    dx = float(end_xy[0]) - float(start_xy[0])
    dy = float(end_xy[1]) - float(start_xy[1])
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return 0.0
    return float((math.degrees(math.atan2(dx, dy)) + 360.0) % 360.0)


def _mission_center_xy(mission_polygon: List[Dict[str, Any]]) -> tuple[float, float] | None:
    polygon_xy = coords_to_xy(mission_polygon)
    if not polygon_xy:
        return None
    x_vals = [float(point[0]) for point in polygon_xy]
    y_vals = [float(point[1]) for point in polygon_xy]
    return (
        sum(x_vals) / float(len(x_vals)),
        sum(y_vals) / float(len(y_vals)),
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
        aircraft_id = entry.get("aircraftID")
        try:
            aircraft_id = int(aircraft_id)
        except Exception:
            continue
        coord = entry.get("coordinate")
        position_xy = coord_to_xy(coord)
        if aircraft_id <= 0 or position_xy is None:
            continue
        heading_deg = entry.get("headingDeg")
        try:
            heading_val = (
                float(monitor_heading_to_planner_bearing_deg(heading_deg))
                if heading_deg is not None
                else None
            )
        except Exception:
            heading_val = None
        if heading_val is None and mission_center_xy is not None:
            heading_val = _bearing_deg_from_xy(position_xy, mission_center_xy)
        if heading_val is None:
            heading_val = 0.0
        rows.append(
            {
                "aircraftID": int(aircraft_id),
                "coordinate": dict(coord),
                "positionXY": (float(position_xy[0]), float(position_xy[1])),
                "headingDeg": float(heading_val),
            }
        )
    rows.sort(key=lambda item: int(item["aircraftID"]))
    return rows


def _ensure(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


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


def run_next_collab_division_plan(
    *,
    mission_polygon: List[Dict[str, Any]],
    aircraft_entries: List[Dict[str, Any]],
    turn_radius_scale: float | None = None,
    log: Callable[[str], None] | None = None,
) -> NextCollabDivisionPlanResult:
    mission_center_xy = _mission_center_xy(mission_polygon)
    aircraft_rows = _normalize_aircraft_rows(
        aircraft_entries,
        mission_center_xy=mission_center_xy,
    )
    _ensure(bool(aircraft_rows), "next-collab planner requires at least one aircraft entry.")
    mission_points_xy = coords_to_xy(mission_polygon)
    _ensure(len(mission_points_xy) >= 3, "next-collab planner requires an area polygon.")

    planner = _HeadlessDivisionPlanner()
    planner.hide()
    try:
        try:
            planner._turn_radius_scale = max(0.1, float(turn_radius_scale or 1.4))
        except Exception:
            planner._turn_radius_scale = 1.4
        planner.state.mission_kind = MISSION_AREA
        planner.state.mode = MODE_MISSION_READY
        planner.state.mission_points_xy = [
            (float(point[0]), float(point[1])) for point in mission_points_xy
        ]
        planner.state.uav_ids = [int(row["aircraftID"]) for row in aircraft_rows]
        planner.state.uav_positions_xy = [row["positionXY"] for row in aircraft_rows]
        planner.state.uav_heading_deg = [float(row["headingDeg"]) for row in aircraft_rows]
        planner._clear_plan_result()

        with _silent_message_boxes():
            planner._run_area_division()
            _ensure(
                planner.state.split_result is not None and bool(planner.state.split_result.pieces),
                "next-collab planner failed to produce split pieces.",
            )
            if log is not None:
                log(
                    "[NEXTCOLLAB] division planner area-division "
                    f"pieces={len(planner.state.split_result.pieces)}"
                )

            planner._generate_mid_lines()
            _ensure(bool(planner.state.mid_line_segments), "next-collab planner failed to build mid-lines.")

            if planner._mid_line_no_split_mode():
                planner._make_path_0()
                _ensure(bool(planner.state.expected_paths), "next-collab planner PATH0 result is empty.")
                planner._make_sweep()
                workflow = "no_split"
            else:
                planner._make_new_area()
                planner._generate_mid_lines_2()
                planner._assignment_path_1()
                _ensure(bool(planner.state.assignment_path_rows), "next-collab planner path-1 assignment is empty.")
                planner._make_waypoint()
                _ensure(
                    any(
                        isinstance(row, dict) and row.get("waypointEndXY") is not None
                        for row in planner.state.expected_paths
                    ),
                    "next-collab planner waypoint result is empty.",
                )
                planner._assignment_next_mission()
                _ensure(
                    any(
                        isinstance(row, dict) and str(row.get("source", "") or "") == "next_mission"
                        for row in planner.state.assignment_path_rows
                    ),
                    "next-collab planner next-mission assignment is empty.",
                )
                planner._make_path_2()
                _ensure(
                    any(
                        isinstance(row, dict) and str(row.get("source", "") or "") == "make_path_2"
                        for row in planner.state.expected_paths
                    ),
                    "next-collab planner PATH2 result is empty.",
                )
                planner._make_sweep_2()
                workflow = "split"

        expected_paths = [
            dict(row)
            for row in planner.state.expected_paths
            if isinstance(row, dict)
        ]
        _ensure(bool(expected_paths), "next-collab planner produced no final path rows.")
        if log is not None:
            log(
                "[NEXTCOLLAB] division planner final "
                f"workflow={workflow} paths={len(expected_paths)}"
            )
            for row in expected_paths:
                base_fov = row.get("resolvedBaseFovDeg")
                adjusted_fov = row.get("resolvedFovDeg")
                if base_fov is None or adjusted_fov is None:
                    continue
                fov_adjust_log = format_runtime_camera_fov_adjustment_log(
                    base_fov,
                    adjusted_fov,
                    context=(
                        f"NEXTCOLLAB AREA {str(row.get('source', '') or '').upper()} "
                        f"UAV{int(row.get('aircraftID', 0) or 0)}"
                        f"/P{int(row.get('pieceIndex', 0) or 0)}"
                    ),
                )
                if fov_adjust_log:
                    log(fov_adjust_log)
        result_text = str(getattr(planner, "_result_text_buffer", "") or "")
        return NextCollabDivisionPlanResult(
            workflow=str(workflow),
            split_result=copy.deepcopy(planner.state.split_result),
            mid_line_segments=copy.deepcopy(list(planner.state.mid_line_segments or [])),
            expected_paths=copy.deepcopy(expected_paths),
            planner_result_text=str(result_text or ""),
        )
    finally:
        planner.close()
