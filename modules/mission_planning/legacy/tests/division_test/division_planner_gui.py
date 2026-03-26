from __future__ import annotations

import csv
import copy
import itertools
import math
import os
import sys
import traceback
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from PyQt5.QtCore import QPoint, QPointF, QRectF, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import (
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
from shapely.geometry import LineString, MultiPolygon, Point, Polygon, box
from shapely.ops import split as geom_split, unary_union

from modules.mission_planning.MissionPlanner.planning_enhanced.algo import run_split_pipeline, review_overflow_areas
from modules.mission_planning.MissionPlanner.planning_enhanced.algo.area_review import review_assigned_areas_local
from modules.mission_planning.MissionPlanner.planning_enhanced.algo.split_runner import assign_split_result_by_takeover_distance
from modules.mission_planning.MissionPlanner.planning_enhanced.io.export_0302 import (
    build_0302_packages_from_split_with_lah,
    save_0302_packages,
)
from modules.mission_planning.MissionPlanner.planning_enhanced.io.export_0303_0304 import (
    build_0303_0304_from_0302_packages,
    save_0303_plans,
    save_0304_plans,
)
from modules.mission_planning.MissionPlanner.planning_enhanced.models import SplitPiece, SplitRunResult
from modules.mission_planning.MissionPlanner.planning_enhanced.pathing import (
    calculate_expected_velocity,
    generate_expected_paths,
)
from modules.mission_planning.MissionPlanner.planning_enhanced.scheduling import run_milp_scheduling
from modules.mission_planning.MissionPlanner.planning_enhanced.type_decider import (
    PROFILE_DEFAULT,
    apply_logic_type_decider,
)
try:
    from modules.mission_planning.MissionPlanner.runtime_settings import (
        get_runtime_area_review_max_segment_m,
        get_runtime_str,
    )
except Exception:
    from modules.mission_planning.MissionPlanner.runtime_settings import (  # type: ignore
        get_runtime_area_review_max_segment_m,
        get_runtime_str,
    )


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
_FOV_DB_PATH = _PROJECT_ROOT / "resource" / "db" / "fov_db.csv"

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
TURN_PREVIEW_SPEED_MPS = 40.0
TURN_PREVIEW_RADIUS_M = TURN_RADIUS_BY_SPEED_MPS[TURN_PREVIEW_SPEED_MPS]
TURN_PREVIEW_BANK_DEG = 30.0
TURN_PREVIEW_HORIZON_S = 5.0
ASSIGNMENT_PATH_ARC_STEP_S = 1.0
ASSIGNMENT_PATH1_ANGLE_WEIGHT = 1.0
ASSIGNMENT_PATH1_TURN_TIME_WEIGHT = 2.0
MAKE_PATH_INTERVAL_M = 1_000.0
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


def local_xy_to_llh(
    east_m: float,
    north_m: float,
    lat0: float = _ORIGIN_LAT,
    lon0: float = _ORIGIN_LON,
) -> Tuple[float, float]:
    lat0_r = math.radians(lat0)
    lat = lat0 + math.degrees(north_m / _R)
    lon = lon0 + math.degrees(east_m / (_R * math.cos(lat0_r)))
    return lat, lon


def llh_to_local_xy(
    lat: float,
    lon: float,
    lat0: float = _ORIGIN_LAT,
    lon0: float = _ORIGIN_LON,
) -> Tuple[float, float]:
    lat0_r = math.radians(lat0)
    east_m = math.radians(lon - lon0) * _R * math.cos(lat0_r)
    north_m = math.radians(lat - lat0) * _R
    return east_m, north_m


def meters_to_coord(east_m: float, north_m: float, alt_m: float = _DEFAULT_ALT) -> Dict[str, float]:
    lat, lon = local_xy_to_llh(east_m, north_m)
    return {"latitude": float(lat), "longitude": float(lon), "altitude": float(alt_m)}


def coord_to_xy(coord: Dict[str, Any]) -> Optional[Tuple[float, float]]:
    if not isinstance(coord, dict):
        return None
    if "latitude" not in coord or "longitude" not in coord:
        return None
    return llh_to_local_xy(float(coord["latitude"]), float(coord["longitude"]))


def coords_to_xy(coords: Sequence[Dict[str, Any]]) -> List[Tuple[float, float]]:
    out: List[Tuple[float, float]] = []
    for coord in coords:
        xy = coord_to_xy(coord)
        if xy is not None:
            out.append(xy)
    return out


def _distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _dedupe_points(
    points_xy: Sequence[Tuple[float, float]],
    min_dist_m: float = 1.0,
) -> List[Tuple[float, float]]:
    out: List[Tuple[float, float]] = []
    for point in points_xy:
        xy = (float(point[0]), float(point[1]))
        if out and _distance(out[-1], xy) < min_dist_m:
            continue
        out.append(xy)
    return out


def normalize_area_points(points_xy: Sequence[Tuple[float, float]]) -> Tuple[List[Tuple[float, float]], bool]:
    cleaned = _dedupe_points(points_xy)
    if len(cleaned) >= 2 and _distance(cleaned[0], cleaned[-1]) < 1.0:
        cleaned = cleaned[:-1]
    poly = Polygon(cleaned)
    corrected = False
    if not poly.is_valid:
        poly = poly.buffer(0)
        corrected = True
    if poly.is_empty:
        raise ValueError("영역이 비어 있습니다. 다시 입력해 주세요.")
    if isinstance(poly, MultiPolygon):
        poly = max((g for g in poly.geoms if isinstance(g, Polygon) and not g.is_empty), key=lambda g: g.area)
        corrected = True
    if not isinstance(poly, Polygon):
        raise ValueError("유효한 영역을 만들 수 없습니다. 교차 없이 다시 입력해 주세요.")
    result = [(float(x), float(y)) for (x, y) in list(poly.exterior.coords)[:-1]]
    if len(result) < 3:
        raise ValueError("영역 점이 3개 미만으로 정리되었습니다. 다시 입력해 주세요.")
    return result, corrected


def corridor_polygon_xy(points_xy: Sequence[Tuple[float, float]], width_m: float) -> List[Tuple[float, float]]:
    if len(points_xy) < 2 or width_m <= 0.0:
        return []
    line = LineString(points_xy)
    poly = line.buffer(float(width_m) * 0.5, cap_style=2, join_style=2, mitre_limit=10.0)
    if poly.is_empty:
        return []
    if isinstance(poly, MultiPolygon):
        poly = max((g for g in poly.geoms if isinstance(g, Polygon) and not g.is_empty), key=lambda g: g.area)
    if not isinstance(poly, Polygon):
        return []
    return [(float(x), float(y)) for (x, y) in list(poly.exterior.coords)[:-1]]


def centroid_xy(points_xy: Sequence[Tuple[float, float]]) -> Optional[Tuple[float, float]]:
    if not points_xy:
        return None
    sx = sum(p[0] for p in points_xy)
    sy = sum(p[1] for p in points_xy)
    n = float(len(points_xy))
    return sx / n, sy / n


def _qcolor(hex_color: str, alpha: int = 255) -> QColor:
    color = QColor(hex_color)
    color.setAlpha(max(0, min(255, int(alpha))))
    return color


def _uav_color(aircraft_id: int, alpha: int = 255) -> QColor:
    return _qcolor(UAV_COLORS.get(int(aircraft_id), "#64748b"), alpha)


def _format_coord(point_xy: Tuple[float, float]) -> str:
    lat, lon = local_xy_to_llh(point_xy[0], point_xy[1])
    return f"E {point_xy[0]:7.1f}m  N {point_xy[1]:7.1f}m  |  {lat:.6f}, {lon:.6f}"


def _undirected_angle_deg(
    vertex_xy: Tuple[float, float],
    point1_xy: Tuple[float, float],
    point2_xy: Tuple[float, float],
) -> Optional[float]:
    v1x = float(point1_xy[0]) - float(vertex_xy[0])
    v1y = float(point1_xy[1]) - float(vertex_xy[1])
    v2x = float(point2_xy[0]) - float(vertex_xy[0])
    v2y = float(point2_xy[1]) - float(vertex_xy[1])
    norm1 = math.hypot(v1x, v1y)
    norm2 = math.hypot(v2x, v2y)
    if norm1 <= 1e-9 or norm2 <= 1e-9:
        return None
    cos_theta = ((v1x * v2x) + (v1y * v2y)) / (norm1 * norm2)
    cos_theta = max(-1.0, min(1.0, float(cos_theta)))
    return float(math.degrees(math.acos(abs(cos_theta))))


def _bearing_deg_from_xy(
    start_xy: Tuple[float, float],
    end_xy: Tuple[float, float],
) -> Optional[float]:
    dx = float(end_xy[0]) - float(start_xy[0])
    dy = float(end_xy[1]) - float(start_xy[1])
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return None
    return float((math.degrees(math.atan2(dx, dy)) + 360.0) % 360.0)


def _prepare_legacy_missionplanner_path() -> None:
    candidate = str(_MISSION_PLANNER_DIR)
    if candidate not in sys.path:
        sys.path.insert(0, candidate)


@dataclass
class CanvasState:
    mode: str = MODE_IDLE
    mission_kind: Optional[str] = None
    draft_points_xy: List[Tuple[float, float]] = field(default_factory=list)
    mission_points_xy: List[Tuple[float, float]] = field(default_factory=list)
    line_width_m: float = 300.0
    line_width_pending: bool = False
    uav_positions_xy: List[Tuple[float, float]] = field(default_factory=list)
    uav_heading_deg: List[Optional[float]] = field(default_factory=list)
    uav_ids: List[int] = field(default_factory=list)
    split_result: Optional[SplitRunResult] = None
    expected_paths: List[Dict[str, Any]] = field(default_factory=list)
    assignment_path_rows: List[Dict[str, Any]] = field(default_factory=list)
    flight_plans_0303: List[Dict[str, Any]] = field(default_factory=list)
    flight_plans_0304: List[Dict[str, Any]] = field(default_factory=list)
    visibility_segments: List[Dict[str, Any]] = field(default_factory=list)
    mid_line_segments: List[Dict[str, Any]] = field(default_factory=list)
    tangent_checks: List[Dict[str, Any]] = field(default_factory=list)
    show_turn_overlays: bool = True
    visible_uav_ids: List[int] = field(default_factory=lambda: list(_UAV_IDS))


class PlanningCanvas(QWidget):
    worldLeftClicked = pyqtSignal(float, float)
    worldRightClicked = pyqtSignal(float, float)
    hoverTextChanged = pyqtSignal(str)

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._state = CanvasState()
        self._view_center_xy = (0.0, 0.0)
        self._view_half_span_m = _INITIAL_HALF_SPAN_M
        self._map_rect = QRectF()
        self._is_panning = False
        self._pan_last_pos = QPoint()
        self._hover_xy: Optional[Tuple[float, float]] = None
        self.setMinimumSize(860, 760)
        self.setMouseTracking(True)
        self.setFocusPolicy(Qt.WheelFocus)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_state(self, state: CanvasState) -> None:
        self._state = state
        self.update()

    def reset_view(self) -> None:
        self._view_center_xy = (0.0, 0.0)
        self._view_half_span_m = _INITIAL_HALF_SPAN_M
        self.update()

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QColor("#e9eef5"))

        full = QRectF(0.0, 0.0, float(self.width()), float(self.height()))
        inner = full.adjusted(54.0, 22.0, -20.0, -46.0)
        side = max(100.0, min(inner.width(), inner.height()))
        left = inner.left() + (inner.width() - side) * 0.5
        top = inner.top() + (inner.height() - side) * 0.5
        self._map_rect = QRectF(left, top, side, side)

        self._draw_map_frame(painter)
        self._draw_grid(painter)
        self._draw_final_mission(painter)
        self._draw_split_result(painter)
        self._draw_mid_line_segments(painter)
        self._draw_visibility_segments(painter)
        self._draw_expected_paths(painter)
        self._draw_draft_input(painter)
        self._draw_uav_heading_guide(painter)
        self._draw_uav_positions(painter)
        self._draw_0303_paths(painter)
        self._draw_status_overlay(painter)
        self._draw_hover_label(painter)

    def wheelEvent(self, event) -> None:
        if not self._map_rect.contains(QPointF(event.pos())):
            event.ignore()
            return

        before = self.screen_to_world(QPointF(event.pos()))
        if before is None:
            event.ignore()
            return

        delta = float(event.angleDelta().y())
        if abs(delta) < 1e-9:
            event.ignore()
            return

        factor = 0.85 if delta > 0 else 1.18
        new_half = min(_MAX_HALF_SPAN_M, max(_MIN_HALF_SPAN_M, self._view_half_span_m * factor))
        if abs(new_half - self._view_half_span_m) < 1e-9:
            event.ignore()
            return

        rel_x = (float(event.pos().x()) - self._map_rect.center().x()) / (self._map_rect.width() * 0.5)
        rel_y = (float(event.pos().y()) - self._map_rect.center().y()) / (self._map_rect.height() * 0.5)
        self._view_center_xy = (
            before[0] - (rel_x * new_half),
            before[1] + (rel_y * new_half),
        )
        self._view_half_span_m = new_half
        self.update()

    def mousePressEvent(self, event) -> None:
        if event.button() == Qt.MiddleButton and self._map_rect.contains(QPointF(event.pos())):
            self._is_panning = True
            self._pan_last_pos = event.pos()
            self.setCursor(Qt.ClosedHandCursor)
            event.accept()
            return

        if event.button() not in (Qt.LeftButton, Qt.RightButton):
            event.ignore()
            return

        world = self.screen_to_world(QPointF(event.pos()))
        if world is None:
            event.ignore()
            return

        if event.button() == Qt.LeftButton:
            self.worldLeftClicked.emit(world[0], world[1])
        else:
            self.worldRightClicked.emit(world[0], world[1])
        event.accept()

    def mouseMoveEvent(self, event) -> None:
        if self._is_panning:
            dx_px = float(event.pos().x() - self._pan_last_pos.x())
            dy_px = float(event.pos().y() - self._pan_last_pos.y())
            scale = self._map_rect.width() / (self._view_half_span_m * 2.0)
            self._view_center_xy = (
                self._view_center_xy[0] - (dx_px / scale),
                self._view_center_xy[1] + (dy_px / scale),
            )
            self._pan_last_pos = event.pos()
            self.update()
            return

        world = self.screen_to_world(QPointF(event.pos()))
        pending_origin: Optional[Tuple[float, float]] = None
        if world is None:
            self._hover_xy = None
            self.hoverTextChanged.emit(
                f"강원 기준 원점 LLA: {_ORIGIN_LAT:.6f}, {_ORIGIN_LON:.6f} | 중클릭 드래그: 이동 | 휠: 확대/축소"
            )
        else:
            self._hover_xy = world
            lat, lon = local_xy_to_llh(world[0], world[1])
            pending_origin = self._pending_uav_heading_origin()
            message = (
                f"E {world[0]:7.1f}m / N {world[1]:7.1f}m | "
                f"LLA {lat:.6f}, {lon:.6f} | 중클릭 드래그: 이동 | 휠: 확대/축소"
            )
            if pending_origin is not None:
                bearing = _bearing_deg_from_xy(pending_origin, world)
                if bearing is not None:
                    message += f" | Heading {bearing:.1f} deg"
            self.hoverTextChanged.emit(message)
        self.update()

    def mouseReleaseEvent(self, event) -> None:
        if event.button() == Qt.MiddleButton and self._is_panning:
            self._is_panning = False
            self.unsetCursor()
            event.accept()
            return
        event.ignore()

    def leaveEvent(self, _event) -> None:
        self._hover_xy = None
        self.hoverTextChanged.emit(f"강원 기준 원점 LLA: {_ORIGIN_LAT:.6f}, {_ORIGIN_LON:.6f}")
        self.update()

    def screen_to_world(self, pos: QPointF) -> Optional[Tuple[float, float]]:
        if self._map_rect.isNull() or (not self._map_rect.contains(pos)):
            return None
        rel_x = (pos.x() - self._map_rect.center().x()) / (self._map_rect.width() * 0.5)
        rel_y = (pos.y() - self._map_rect.center().y()) / (self._map_rect.height() * 0.5)
        east = self._view_center_xy[0] + (rel_x * self._view_half_span_m)
        north = self._view_center_xy[1] - (rel_y * self._view_half_span_m)
        return east, north

    def world_to_screen(self, east_m: float, north_m: float) -> QPointF:
        scale = self._map_rect.width() / (self._view_half_span_m * 2.0)
        sx = self._map_rect.center().x() + ((east_m - self._view_center_xy[0]) * scale)
        sy = self._map_rect.center().y() - ((north_m - self._view_center_xy[1]) * scale)
        return QPointF(sx, sy)

    def _aircraft_visible(self, aircraft_id: int) -> bool:
        aid = int(aircraft_id or 0)
        if aid <= 0:
            return True
        visible = {int(x) for x in self._state.visible_uav_ids}
        if not visible:
            return False
        return aid in visible

    def _append_unique_xy(
        self,
        points_xy: List[Tuple[float, float]],
        point_xy: Tuple[float, float],
        *,
        min_dist_m: float = 1.0,
    ) -> None:
        if points_xy and _distance(points_xy[-1], point_xy) < min_dist_m:
            return
        points_xy.append((float(point_xy[0]), float(point_xy[1])))

    def _draw_number_badge(
        self,
        painter: QPainter,
        point_xy: Tuple[float, float],
        text: str,
        color: QColor,
        *,
        diameter: float = 30.0,
    ) -> None:
        screen = self.world_to_screen(point_xy[0], point_xy[1])
        radius = diameter * 0.5
        rect = QRectF(screen.x() - radius, screen.y() - radius, diameter, diameter)
        painter.save()
        font = QFont(painter.font())
        font.setBold(True)
        font.setPointSizeF(max(10.0, diameter * 0.38))
        painter.setFont(font)
        painter.setPen(QPen(color, 2.4))
        painter.setBrush(QColor(255, 255, 255, 245))
        painter.drawEllipse(rect)
        painter.setPen(QColor("#0f172a"))
        painter.drawText(rect, Qt.AlignCenter, text)
        painter.restore()

    def _draw_path_arrows(
        self,
        painter: QPainter,
        points_xy: Sequence[Tuple[float, float]],
        color: QColor,
        *,
        min_segment_m: float = 40.0,
        max_arrows: int = 12,
    ) -> None:
        if len(points_xy) < 2:
            return
        step = max(1, (len(points_xy) - 1) // max(1, max_arrows))
        for idx in range(0, len(points_xy) - 1, step):
            if _distance(points_xy[idx], points_xy[idx + 1]) < min_segment_m:
                continue
            self._draw_arrow(painter, points_xy[idx], points_xy[idx + 1], color)

    def _pending_uav_heading_index(self) -> Optional[int]:
        if self._state.mode != MODE_SET_UAV_HEADING:
            return None
        if not self._state.uav_positions_xy:
            return None
        idx = len(self._state.uav_positions_xy) - 1
        if idx < 0:
            return None
        if idx >= len(self._state.uav_heading_deg):
            return idx
        return idx if self._state.uav_heading_deg[idx] is None else None

    def _pending_uav_heading_origin(self) -> Optional[Tuple[float, float]]:
        idx = self._pending_uav_heading_index()
        if idx is None or idx >= len(self._state.uav_positions_xy):
            return None
        return self._state.uav_positions_xy[idx]

    def _draw_map_frame(self, painter: QPainter) -> None:
        painter.setPen(QPen(QColor("#9aa8bc"), 1.2))
        painter.setBrush(QColor("#fbfdff"))
        painter.drawRoundedRect(self._map_rect, 8.0, 8.0)

    def _draw_grid(self, painter: QPainter) -> None:
        step = self._grid_step(self._view_half_span_m)
        min_e = self._view_center_xy[0] - self._view_half_span_m
        max_e = self._view_center_xy[0] + self._view_half_span_m
        min_n = self._view_center_xy[1] - self._view_half_span_m
        max_n = self._view_center_xy[1] + self._view_half_span_m

        grid_pen = QPen(QColor("#d8e0eb"), 1.0)
        axis_pen = QPen(QColor("#9fb1c8"), 1.4)
        painter.setFont(QFont("Consolas", 8))

        start_e = math.floor(min_e / step) * step
        end_e = math.ceil(max_e / step) * step
        e_val = start_e
        while e_val <= end_e + 1e-9:
            p0 = self.world_to_screen(e_val, min_n)
            p1 = self.world_to_screen(e_val, max_n)
            painter.setPen(axis_pen if abs(e_val) < 1e-9 else grid_pen)
            painter.drawLine(p0, p1)
            label_pos = self.world_to_screen(e_val, min_n)
            painter.setPen(QColor("#52637a"))
            painter.drawText(QPointF(label_pos.x() - 12.0, self._map_rect.bottom() + 16.0), f"{int(round(e_val))}")
            e_val += step

        start_n = math.floor(min_n / step) * step
        end_n = math.ceil(max_n / step) * step
        n_val = start_n
        while n_val <= end_n + 1e-9:
            p0 = self.world_to_screen(min_e, n_val)
            p1 = self.world_to_screen(max_e, n_val)
            painter.setPen(axis_pen if abs(n_val) < 1e-9 else grid_pen)
            painter.drawLine(p0, p1)
            label_pos = self.world_to_screen(min_e, n_val)
            painter.setPen(QColor("#52637a"))
            painter.drawText(QPointF(self._map_rect.left() - 48.0, label_pos.y() + 4.0), f"{int(round(n_val))}")
            n_val += step

        painter.setPen(QColor("#334155"))
        painter.drawText(QPointF(self._map_rect.left(), self._map_rect.bottom() + 34.0), "East [m]")
        painter.save()
        painter.translate(self._map_rect.left() - 40.0, self._map_rect.top() + 26.0)
        painter.rotate(-90.0)
        painter.drawText(QPointF(0.0, 0.0), "North [m]")
        painter.restore()

    def _draw_final_mission(self, painter: QPainter) -> None:
        if not self._state.mission_points_xy:
            return

        if self._state.mission_kind == MISSION_AREA:
            points = QPolygonF([self.world_to_screen(x, y) for (x, y) in self._state.mission_points_xy])
            painter.setBrush(_qcolor("#cbd5e1", 70))
            painter.setPen(QPen(QColor("#111827"), 2.0))
            painter.drawPolygon(points)
            self._draw_indexed_points(painter, self._state.mission_points_xy, QColor("#0f172a"), 4.0)
            return

        corridor = corridor_polygon_xy(self._state.mission_points_xy, self._state.line_width_m)
        if corridor:
            painter.setBrush(_qcolor("#bfdbfe", 95))
            painter.setPen(QPen(QColor("#1d4ed8"), 1.8))
            painter.drawPolygon(QPolygonF([self.world_to_screen(x, y) for (x, y) in corridor]))
        painter.setPen(QPen(QColor("#0f172a"), 2.4))
        self._draw_polyline(painter, self._state.mission_points_xy)
        self._draw_indexed_points(painter, self._state.mission_points_xy, QColor("#0f172a"), 4.0)

    def _draw_split_result(self, painter: QPainter) -> None:
        if self._state.split_result is None:
            return

        piece_lookup = self._piece_lookup(self._state.split_result)
        for piece in self._state.split_result.pieces:
            coords = coords_to_xy((piece.data or {}).get("coordinateList", []))
            if len(coords) < 3:
                continue
            aid = int(piece.assigned_uav or 0)
            if not self._aircraft_visible(aid):
                continue
            fill = _uav_color(aid, 78) if aid > 0 else _qcolor("#64748b", 70)
            edge = _uav_color(aid, 190) if aid > 0 else _qcolor("#475569", 220)
            painter.setBrush(fill)
            painter.setPen(QPen(edge, 1.6))
            painter.drawPolygon(QPolygonF([self.world_to_screen(x, y) for (x, y) in coords]))

            center = self._piece_center_xy(piece, piece_lookup)
            if center is None:
                center = centroid_xy(coords)
            if center is None:
                continue
            self._draw_centroid_marker(painter, center, edge)
            bearing_deg = self._piece_bearing_deg(piece)
            if bearing_deg is not None:
                self._draw_bearing_arrow(
                    painter,
                    center,
                    bearing_deg,
                    edge,
                )
            label = f"P{piece.piece_index}"
            if aid > 0:
                label += f" / UAV{aid}"
            screen = self.world_to_screen(center[0], center[1])
            rect = QRectF(screen.x() - 34.0, screen.y() - 32.0, 68.0, 22.0)
            painter.setPen(QColor("#0f172a"))
            painter.setBrush(QColor("#ffffff"))
            painter.drawRoundedRect(rect, 6.0, 6.0)
            painter.drawText(rect, Qt.AlignCenter, label)

    def _draw_expected_paths(self, painter: QPainter) -> None:
        if not self._state.expected_paths:
            return
        piece_lookup = self._piece_lookup(self._state.split_result)
        for row in self._state.expected_paths:
            coords: List[Tuple[float, float]] = []
            route_xy = row.get("routeXY") if isinstance(row, dict) else None
            if isinstance(route_xy, list):
                for point_xy in route_xy:
                    if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2:
                        coords.append((float(point_xy[0]), float(point_xy[1])))
            else:
                coords = coords_to_xy(row.get("coordinateList", []))
            if len(coords) < 2:
                continue
            aid = int(row.get("aircraftID", 0) or 0) if isinstance(row, dict) else 0
            if aid <= 0:
                aid = self._path_uav_id(row, piece_lookup)
            if not self._aircraft_visible(aid):
                continue

            sweep_line_list_xy = row.get("sweepLineListXY") if isinstance(row, dict) else None
            if isinstance(sweep_line_list_xy, list):
                sweep_pen = QPen(_qcolor("#0f766e", 165), 1.2)
                sweep_pen.setStyle(Qt.SolidLine)
                sweep_pen.setCapStyle(Qt.RoundCap)
                sweep_pen.setJoinStyle(Qt.RoundJoin)
                painter.save()
                painter.setPen(sweep_pen)
                for line_xy in sweep_line_list_xy:
                    if not (isinstance(line_xy, list) and len(line_xy) >= 2):
                        continue
                    line_points: List[Tuple[float, float]] = []
                    for point_xy in line_xy:
                        if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2:
                            line_points.append((float(point_xy[0]), float(point_xy[1])))
                    if len(line_points) >= 2:
                        self._draw_polyline(painter, line_points)
                painter.restore()

            pen = QPen(_uav_color(aid, 230) if aid > 0 else _qcolor("#111827", 220), 2.4)
            pen.setStyle(Qt.DashLine)
            painter.setPen(pen)
            self._draw_polyline(painter, coords)
            self._draw_arrow(painter, coords[-2], coords[-1], pen.color())

            marker_rows = row.get("markerRows") if isinstance(row, dict) else None
            if isinstance(marker_rows, list):
                painter.save()
                for marker in marker_rows:
                    if not isinstance(marker, dict):
                        continue
                    point_xy = marker.get("xy")
                    if not (isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2):
                        continue
                    label = str(marker.get("label", "") or "")
                    kind = str(marker.get("kind", "") or "")
                    if kind == "tangent":
                        color = QColor("#f59e0b")
                    elif kind == "entry":
                        color = QColor("#fb7185")
                    elif kind == "waypoint_start":
                        color = QColor("#22c55e")
                    elif kind == "waypoint_end":
                        color = QColor("#111827")
                    elif kind == "interval":
                        color = QColor("#0f766e")
                    elif kind == "last_face":
                        color = QColor("#111827")
                    else:
                        color = _uav_color(aid, 230) if aid > 0 else _qcolor("#111827", 220)
                    screen = self.world_to_screen(float(point_xy[0]), float(point_xy[1]))
                    painter.setPen(QPen(QColor("#ffffff"), 1.0))
                    painter.setBrush(color)
                    radius = 4.0 if kind == "tangent" else 3.4
                    painter.drawEllipse(screen, radius, radius)
                    if label:
                        painter.setPen(QPen(color.darker(220), 1.0))
                        rect = QRectF(screen.x() + 5.0, screen.y() - 11.0, 34.0, 18.0)
                        painter.drawText(rect, Qt.AlignLeft | Qt.AlignVCenter, label)
                painter.restore()

    def _draw_visibility_segments(self, painter: QPainter) -> None:
        if not self._state.visibility_segments:
            return
        for row in self._state.visibility_segments:
            if not isinstance(row, dict):
                continue
            start_xy = row.get("startXY")
            end_xy = row.get("endXY")
            if not (
                isinstance(start_xy, (tuple, list))
                and len(start_xy) >= 2
                and isinstance(end_xy, (tuple, list))
                and len(end_xy) >= 2
            ):
                continue
            aid = int(row.get("aircraftID", 0) or 0)
            if not self._aircraft_visible(aid):
                continue

            color = _uav_color(aid, 230) if aid > 0 else _qcolor("#111827", 220)
            pen = QPen(color, 1.2, Qt.DashLine)
            pen.setCapStyle(Qt.RoundCap)
            painter.save()
            painter.setPen(pen)
            self._draw_polyline(
                painter,
                [
                    (float(start_xy[0]), float(start_xy[1])),
                    (float(end_xy[0]), float(end_xy[1])),
                ],
            )
            painter.restore()

    def _draw_tangent_checks(self, painter: QPainter) -> None:
        if not self._state.tangent_checks:
            return
        for row in self._state.tangent_checks:
            if not isinstance(row, dict):
                continue
            aid = int(row.get("aircraftID", 0) or 0)
            if not self._aircraft_visible(aid):
                continue
            tangent_xy = row.get("tangentXY")
            target_xy = row.get("targetXY")
            circle_center_xy = row.get("circleCenterXY")
            if not (
                isinstance(tangent_xy, (tuple, list))
                and len(tangent_xy) >= 2
                and isinstance(target_xy, (tuple, list))
                and len(target_xy) >= 2
                and isinstance(circle_center_xy, (tuple, list))
                and len(circle_center_xy) >= 2
            ):
                continue

            tangent_point_xy = (float(tangent_xy[0]), float(tangent_xy[1]))
            target_point_xy = (float(target_xy[0]), float(target_xy[1]))
            circle_point_xy = (float(circle_center_xy[0]), float(circle_center_xy[1]))
            color = _uav_color(aid, 235) if aid > 0 else _qcolor("#111827", 220)

            painter.save()

            radius_pen = QPen(_uav_color(aid, 90) if aid > 0 else _qcolor("#64748b", 90), 0.9, Qt.DotLine)
            radius_pen.setCapStyle(Qt.RoundCap)
            painter.setPen(radius_pen)
            self._draw_polyline(painter, [circle_point_xy, tangent_point_xy])

            tangent_pen = QPen(color, 1.4, Qt.DashLine)
            tangent_pen.setCapStyle(Qt.RoundCap)
            painter.setPen(tangent_pen)
            self._draw_polyline(painter, [tangent_point_xy, target_point_xy])

            screen = self.world_to_screen(tangent_point_xy[0], tangent_point_xy[1])
            painter.setPen(QPen(QColor("#ffffff"), 1.0))
            painter.setBrush(color)
            painter.drawEllipse(screen, 3.8, 3.8)

            label = str(row.get("label", "") or "")
            if label:
                rect = QRectF(screen.x() + 5.0, screen.y() - 10.0, 52.0, 18.0)
                painter.setPen(QColor("#0f172a"))
                painter.setBrush(_qcolor("#ffffff", 240))
                painter.drawRoundedRect(rect, 5.0, 5.0)
                painter.drawText(rect, Qt.AlignCenter, label)

            painter.restore()

    def _draw_mid_line_segments(self, painter: QPainter) -> None:
        if not self._state.mid_line_segments:
            return
        for row in self._state.mid_line_segments:
            if not isinstance(row, dict):
                continue
            aid = int(row.get("aircraftID", 0) or 0)
            if not self._aircraft_visible(aid):
                continue
            mid_line_required = bool(row.get("midLineRequired", True))

            box_xy = row.get("boxXY")
            if isinstance(box_xy, list) and len(box_xy) >= 4:
                box_points = []
                for point_xy in box_xy:
                    if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2:
                        box_points.append(self.world_to_screen(float(point_xy[0]), float(point_xy[1])))
                if len(box_points) >= 4:
                    painter.save()
                    box_pen = QPen(_uav_color(aid, 150) if aid > 0 else _qcolor("#475569", 150), 1.0, Qt.DotLine)
                    box_pen.setCapStyle(Qt.RoundCap)
                    painter.setPen(box_pen)
                    painter.setBrush(Qt.NoBrush)
                    painter.drawPolygon(QPolygonF(box_points))
                    painter.restore()

            mid_line_xy = row.get("midLineXY")
            line_points: List[Tuple[float, float]] = []
            if isinstance(mid_line_xy, list) and len(mid_line_xy) >= 2:
                for point_xy in mid_line_xy:
                    if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2:
                        line_points.append((float(point_xy[0]), float(point_xy[1])))
            if mid_line_required and len(line_points) >= 2:
                line_pen = QPen(_uav_color(aid, 235) if aid > 0 else _qcolor("#0f172a", 220), 1.8, Qt.DashLine)
                line_pen.setCapStyle(Qt.RoundCap)
                painter.save()
                painter.setPen(line_pen)
                self._draw_polyline(painter, line_points)
                painter.restore()

                split_parts = row.get("splitParts")
                if isinstance(split_parts, list):
                    for part_idx, part in enumerate(split_parts):
                        if not isinstance(part, dict):
                            continue
                        part_color = (
                            _qcolor("#f97316", 210)
                            if part_idx == 0
                            else _qcolor("#14b8a6", 210)
                        )
                        polygon_xy = part.get("polygonXY")
                        if isinstance(polygon_xy, list) and len(polygon_xy) >= 3:
                            polygon_points = []
                            for point_xy in polygon_xy:
                                if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2:
                                    polygon_points.append(self.world_to_screen(float(point_xy[0]), float(point_xy[1])))
                            if len(polygon_points) >= 3:
                                painter.save()
                                part_pen = QPen(part_color, 1.3, Qt.DotLine)
                                part_pen.setCapStyle(Qt.RoundCap)
                                painter.setPen(part_pen)
                                painter.setBrush(Qt.NoBrush)
                                painter.drawPolygon(QPolygonF(polygon_points))
                                painter.restore()
                        part_points = part.get("pointLabels")
                        if not isinstance(part_points, list):
                            continue
                        self._draw_named_points(
                            painter,
                            part_points,
                            part_color,
                        )

            max_width_xy = row.get("maxWidthLineXY")
            width_points: List[Tuple[float, float]] = []
            if isinstance(max_width_xy, list) and len(max_width_xy) >= 2:
                for point_xy in max_width_xy:
                    if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2:
                        width_points.append((float(point_xy[0]), float(point_xy[1])))
            if len(width_points) >= 2:
                width_pen = QPen(_uav_color(aid, 245) if aid > 0 else _qcolor("#0f172a", 230), 2.2)
                width_pen.setCapStyle(Qt.RoundCap)
                painter.save()
                painter.setPen(width_pen)
                self._draw_polyline(painter, width_points)
                painter.setBrush(width_pen.color())
                for point_xy in width_points:
                    screen = self.world_to_screen(point_xy[0], point_xy[1])
                    painter.drawEllipse(screen, 3.2, 3.2)

                center_xy = row.get("maxWidthCenterXY")
                max_width_m = float(row.get("maxWidthM", 0.0) or 0.0)
                if isinstance(center_xy, (tuple, list)) and len(center_xy) >= 2 and max_width_m > 0.0:
                    screen = self.world_to_screen(float(center_xy[0]), float(center_xy[1]))
                    rect = QRectF(screen.x() - 30.0, screen.y() - 30.0, 60.0, 18.0)
                    painter.setPen(QColor("#0f172a"))
                    painter.setBrush(QColor("#ffffff"))
                    painter.drawRoundedRect(rect, 5.0, 5.0)
                    painter.drawText(rect, Qt.AlignCenter, f"W {max_width_m:.0f}")
                painter.restore()

            stage2_centers = row.get("stage2Centers")
            if isinstance(stage2_centers, list):
                painter.save()
                center_pen = QPen(_uav_color(aid, 255) if aid > 0 else _qcolor("#111827", 240), 1.2)
                painter.setPen(center_pen)
                drawn_center_points: List[Tuple[float, float]] = []
                for idx, center_row in enumerate(stage2_centers, start=1):
                    if not isinstance(center_row, dict):
                        continue
                    line_xy = center_row.get("lineXY")
                    line_points: List[Tuple[float, float]] = []
                    if isinstance(line_xy, list) and len(line_xy) >= 2:
                        for point_xy in line_xy:
                            if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2:
                                line_points.append((float(point_xy[0]), float(point_xy[1])))
                    if len(line_points) >= 2:
                        self._draw_polyline(painter, line_points)
                    center_xy = center_row.get("centerXY")
                    if not (isinstance(center_xy, (tuple, list)) and len(center_xy) >= 2):
                        continue
                    screen = self.world_to_screen(float(center_xy[0]), float(center_xy[1]))
                    overlap_idx = sum(1 for point_xy in drawn_center_points if _distance(point_xy, (float(center_xy[0]), float(center_xy[1]))) <= 12.0)
                    drawn_center_points.append((float(center_xy[0]), float(center_xy[1])))
                    fill = _qcolor("#ffffff", 240)
                    dot = _uav_color(aid, 255) if aid > 0 else _qcolor("#111827", 240)
                    painter.setPen(QPen(QColor("#ffffff"), 1.0))
                    painter.setBrush(dot)
                    painter.drawEllipse(screen, 4.4, 4.4)
                    label = str(center_row.get("label", "") or f"F{idx}")
                    rect = QRectF(screen.x() + 6.0, screen.y() - 10.0 + (float(overlap_idx) * 16.0), 44.0, 18.0)
                    painter.setPen(QColor("#0f172a"))
                    painter.setBrush(fill)
                    painter.drawRoundedRect(rect, 5.0, 5.0)
                    painter.drawText(rect, Qt.AlignCenter, label)
                painter.restore()

    def _draw_0303_paths(self, painter: QPainter) -> None:
        if not self._state.flight_plans_0303:
            return

        wp_start_offsets = self._waypoint_start_offsets_by_uav(
            self._state.flight_plans_0303,
            waypoint_key="waypointList",
        )

        for row_idx, row in enumerate(self._state.flight_plans_0303):
            if not isinstance(row, dict):
                continue
            aid = int(row.get("aircraftID", 0) or 0)
            if not self._aircraft_visible(aid):
                continue
            wps = row.get("waypointList")
            if not isinstance(wps, list):
                continue

            coords: List[Tuple[float, float]] = []
            sweep_lines: List[Tuple[List[Tuple[float, float]], int]] = []
            sweep_points: List[Tuple[float, float]] = []
            sweep_marker_points: List[Tuple[float, float]] = []
            for wp in wps:
                if not isinstance(wp, dict):
                    continue
                coord_xy = coord_to_xy(wp.get("coordinate", {}))
                if coord_xy is not None:
                    coords.append(coord_xy)
                fp = wp.get("filmingProperty")
                if isinstance(fp, dict):
                    ls = fp.get("lineSearch")
                    if isinstance(ls, dict):
                        ls_coords = coords_to_xy(ls.get("coordinateList", []))
                        if len(ls_coords) >= 2:
                            interp_points = int(ls.get("interpolationPoints", 0) or 0)
                            sweep_lines.append((ls_coords, interp_points))
                            sweep_marker_points.extend(ls_coords)
                    cor = fp.get("coordinateOrientation")
                    if isinstance(cor, dict):
                        target_xy = coord_to_xy(cor.get("coordinate", {}))
                        if target_xy is not None:
                            sweep_points.append(target_xy)
                            sweep_marker_points.append(target_xy)
            traversal_points = list(coords)

            route_color = _uav_color(aid, 245) if aid > 0 else _qcolor("#111827", 240)
            if sweep_lines:
                sweep_pen = QPen(_qcolor("#0f766e", 150), 1.0)
                sweep_pen.setStyle(Qt.SolidLine)
                sweep_pen.setCapStyle(Qt.RoundCap)
                sweep_pen.setJoinStyle(Qt.RoundJoin)
                painter.setPen(sweep_pen)
                for ls_coords, interp_points in sweep_lines:
                    chunk = max(2, int(interp_points))
                    if chunk <= 2 or len(ls_coords) <= chunk:
                        self._draw_polyline(painter, ls_coords)
                        continue
                    for base in range(0, len(ls_coords), chunk):
                        seg = ls_coords[base:base + chunk]
                        if len(seg) >= 2:
                            self._draw_polyline(painter, seg)
            elif len(sweep_points) >= 2:
                sweep_pen = QPen(_qcolor("#0f766e", 150), 1.0)
                sweep_pen.setStyle(Qt.SolidLine)
                sweep_pen.setCapStyle(Qt.RoundCap)
                sweep_pen.setJoinStyle(Qt.RoundJoin)
                painter.setPen(sweep_pen)
                self._draw_polyline(painter, sweep_points)

            if sweep_marker_points:
                self._draw_point_markers(
                    painter,
                    sweep_marker_points,
                    _qcolor("#0f766e", 170),
                    radius=2.2,
                    pen_width=0.8,
                )

            if len(traversal_points) >= 2:
                route_outline_pen = QPen(QColor(15, 23, 42, 150), 4.2)
                route_outline_pen.setStyle(Qt.SolidLine)
                route_outline_pen.setCapStyle(Qt.RoundCap)
                route_outline_pen.setJoinStyle(Qt.RoundJoin)
                painter.setPen(route_outline_pen)
                self._draw_polyline(painter, traversal_points)

                route_pen = QPen(route_color, 2.6)
                route_pen.setStyle(Qt.SolidLine)
                route_pen.setCapStyle(Qt.RoundCap)
                route_pen.setJoinStyle(Qt.RoundJoin)
                painter.setPen(route_pen)
                self._draw_polyline(painter, traversal_points)
                self._draw_path_arrows(painter, traversal_points, route_color)

            if traversal_points:
                self._draw_point_markers(
                    painter,
                    traversal_points,
                    route_color,
                    radius=3.8,
                    pen_width=1.0,
                )

            label_pool = coords
            if not label_pool:
                continue

            start_xy = label_pool[0]
            end_xy = label_pool[-1]
            start = self.world_to_screen(start_xy[0], start_xy[1])
            end = self.world_to_screen(end_xy[0], end_xy[1])
            painter.setPen(QPen(QColor("#065f46"), 1.0))
            painter.setBrush(QColor("#22c55e"))
            painter.drawEllipse(QRectF(start.x() - 4.5, start.y() - 4.5, 9.0, 9.0))
            painter.setBrush(QColor("#ef4444"))
            painter.drawEllipse(QRectF(end.x() - 4.5, end.y() - 4.5, 9.0, 9.0))

            mid = label_pool[len(label_pool) // 2]
            mp = self.world_to_screen(mid[0], mid[1])
            pid = int(row.get("pathID", 0) or 0)
            label = f"0303 U{aid}/P{pid}" if aid > 0 and pid > 0 else "0303"
            box_w = max(88.0, min(150.0, 7.0 * float(len(label)) + 16.0))
            rect = QRectF(mp.x() - (box_w * 0.5), mp.y() - 10.0, box_w, 18.0)
            painter.setPen(QPen(QColor("#0f172a"), 1.0))
            painter.setBrush(QColor(255, 255, 255, 215))
            painter.drawRoundedRect(rect, 5.0, 5.0)
            painter.drawText(rect, Qt.AlignCenter, label)

            if len(traversal_points) <= 16:
                start_offset = int(wp_start_offsets.get(int(row_idx), 0))
                for idx, point_xy in enumerate(traversal_points, start=1):
                    self._draw_number_badge(
                        painter,
                        point_xy,
                        str(int(start_offset + idx)),
                        route_color,
                        diameter=24.0,
                    )

    def _draw_draft_input(self, painter: QPainter) -> None:
        if not self._state.draft_points_xy:
            return
        draft = self._state.draft_points_xy
        if self._state.mission_kind == MISSION_AREA:
            painter.setBrush(_qcolor("#fdba74", 80))
            painter.setPen(QPen(QColor("#c2410c"), 1.8, Qt.DashLine))
            if len(draft) >= 3:
                painter.drawPolygon(QPolygonF([self.world_to_screen(x, y) for (x, y) in draft]))
            else:
                self._draw_polyline(painter, draft)
            self._draw_indexed_points(painter, draft, QColor("#c2410c"), 4.2)
            return

        if self._state.line_width_pending and self._state.line_width_m > 0.0:
            corridor = corridor_polygon_xy(draft, self._state.line_width_m)
            if corridor:
                painter.setBrush(_qcolor("#93c5fd", 95))
                painter.setPen(QPen(QColor("#2563eb"), 1.6, Qt.DashLine))
                painter.drawPolygon(QPolygonF([self.world_to_screen(x, y) for (x, y) in corridor]))

        painter.setPen(QPen(QColor("#2563eb"), 2.1, Qt.DashLine))
        self._draw_polyline(painter, draft)
        self._draw_indexed_points(painter, draft, QColor("#2563eb"), 4.2)

    def _turn_circle_centers_xy(
        self,
        origin_xy: Tuple[float, float],
        bearing_deg: float,
        *,
        radius_m: float = TURN_PREVIEW_RADIUS_M,
    ) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        theta = math.radians(float(bearing_deg) % 360.0)
        left_center = (
            float(origin_xy[0]) - (math.cos(theta) * float(radius_m)),
            float(origin_xy[1]) + (math.sin(theta) * float(radius_m)),
        )
        right_center = (
            float(origin_xy[0]) + (math.cos(theta) * float(radius_m)),
            float(origin_xy[1]) - (math.sin(theta) * float(radius_m)),
        )
        return left_center, right_center

    def _draw_turn_radius_circles(
        self,
        painter: QPainter,
        origin_xy: Tuple[float, float],
        bearing_deg: float,
        color: QColor,
        *,
        radius_m: float = TURN_PREVIEW_RADIUS_M,
    ) -> None:
        if self._map_rect.isNull() or self._view_half_span_m <= 0.0 or radius_m <= 0.0:
            return
        scale = self._map_rect.width() / (self._view_half_span_m * 2.0)
        radius_px = float(radius_m) * float(scale)
        if radius_px < 2.0:
            return

        painter.save()
        pen = QPen(color, 1.5, Qt.DashLine)
        pen.setCapStyle(Qt.RoundCap)
        painter.setPen(pen)
        fill_color = QColor(color)
        fill_color.setAlpha(max(18, min(40, int(color.alpha() * 0.3))))
        painter.setBrush(fill_color)
        for center_xy in self._turn_circle_centers_xy(origin_xy, bearing_deg, radius_m=radius_m):
            center = self.world_to_screen(center_xy[0], center_xy[1])
            painter.drawEllipse(
                QRectF(
                    center.x() - radius_px,
                    center.y() - radius_px,
                    radius_px * 2.0,
                    radius_px * 2.0,
                )
            )
        painter.restore()

    def _turn_prediction_points_xy(
        self,
        origin_xy: Tuple[float, float],
        bearing_deg: float,
        *,
        speed_mps: float = TURN_PREVIEW_SPEED_MPS,
        horizon_s: float = TURN_PREVIEW_HORIZON_S,
        radius_m: float = TURN_PREVIEW_RADIUS_M,
    ) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        if radius_m <= 0.0:
            return origin_xy, origin_xy

        def rotate_xy(vec_xy: Tuple[float, float], angle_rad: float) -> Tuple[float, float]:
            cos_a = math.cos(angle_rad)
            sin_a = math.sin(angle_rad)
            return (
                (float(vec_xy[0]) * cos_a) - (float(vec_xy[1]) * sin_a),
                (float(vec_xy[0]) * sin_a) + (float(vec_xy[1]) * cos_a),
            )

        arc_angle = (float(speed_mps) * float(horizon_s)) / float(radius_m)
        left_center_xy, right_center_xy = self._turn_circle_centers_xy(origin_xy, bearing_deg, radius_m=radius_m)
        left_radius_xy = (
            float(origin_xy[0]) - float(left_center_xy[0]),
            float(origin_xy[1]) - float(left_center_xy[1]),
        )
        right_radius_xy = (
            float(origin_xy[0]) - float(right_center_xy[0]),
            float(origin_xy[1]) - float(right_center_xy[1]),
        )

        left_rotated_xy = rotate_xy(left_radius_xy, arc_angle)
        right_rotated_xy = rotate_xy(right_radius_xy, -arc_angle)
        left_point_xy = (
            float(left_center_xy[0]) + float(left_rotated_xy[0]),
            float(left_center_xy[1]) + float(left_rotated_xy[1]),
        )
        right_point_xy = (
            float(right_center_xy[0]) + float(right_rotated_xy[0]),
            float(right_center_xy[1]) + float(right_rotated_xy[1]),
        )
        return left_point_xy, right_point_xy

    def _draw_turn_prediction_points(
        self,
        painter: QPainter,
        origin_xy: Tuple[float, float],
        bearing_deg: float,
        color: QColor,
        *,
        speed_mps: float = TURN_PREVIEW_SPEED_MPS,
        horizon_s: float = TURN_PREVIEW_HORIZON_S,
        radius_m: float = TURN_PREVIEW_RADIUS_M,
    ) -> None:
        left_point_xy, right_point_xy = self._turn_prediction_points_xy(
            origin_xy,
            bearing_deg,
            speed_mps=speed_mps,
            horizon_s=horizon_s,
            radius_m=radius_m,
        )
        painter.save()
        painter.setPen(QPen(QColor("#ffffff"), 1.6))
        painter.setBrush(color)
        for point_xy in (left_point_xy, right_point_xy):
            point = self.world_to_screen(point_xy[0], point_xy[1])
            painter.drawEllipse(point, 5.0, 5.0)
        painter.restore()

    def _draw_uav_heading_guide(self, painter: QPainter) -> None:
        anchor_xy = self._pending_uav_heading_origin()
        hover_xy = self._hover_xy
        if anchor_xy is None or hover_xy is None:
            return
        if _distance(anchor_xy, hover_xy) < 5.0:
            return

        idx = self._pending_uav_heading_index()
        aid = self._state.uav_ids[idx] if idx is not None and idx < len(self._state.uav_ids) else 0
        color = _uav_color(aid, 240) if aid > 0 else _qcolor("#475569", 230)
        bearing = _bearing_deg_from_xy(anchor_xy, hover_xy)

        if bearing is not None and self._state.show_turn_overlays:
            self._draw_turn_radius_circles(
                painter,
                anchor_xy,
                bearing,
                _uav_color(aid, 120) if aid > 0 else _qcolor("#475569", 120),
            )
            self._draw_turn_prediction_points(
                painter,
                anchor_xy,
                bearing,
                _uav_color(aid, 220) if aid > 0 else _qcolor("#475569", 220),
            )

        painter.save()
        pen = QPen(color, 2.0, Qt.DashLine)
        pen.setCapStyle(Qt.RoundCap)
        painter.setPen(pen)
        self._draw_polyline(painter, [anchor_xy, hover_xy])
        self._draw_arrow(painter, anchor_xy, hover_xy, color)
        painter.restore()

        if bearing is None:
            return
        mid_xy = (
            (float(anchor_xy[0]) + float(hover_xy[0])) * 0.5,
            (float(anchor_xy[1]) + float(hover_xy[1])) * 0.5,
        )
        mid = self.world_to_screen(mid_xy[0], mid_xy[1])
        rect = QRectF(mid.x() - 44.0, mid.y() - 14.0, 88.0, 22.0)
        painter.save()
        painter.setPen(QPen(QColor("#0f172a"), 1.0))
        painter.setBrush(QColor(255, 255, 255, 225))
        painter.drawRoundedRect(rect, 6.0, 6.0)
        painter.drawText(rect, Qt.AlignCenter, f"HDG {bearing:.1f}")
        painter.restore()

    def _draw_uav_positions(self, painter: QPainter) -> None:
        for idx, point_xy in enumerate(self._state.uav_positions_xy):
            aid = self._state.uav_ids[idx] if idx < len(self._state.uav_ids) else 0
            if not self._aircraft_visible(int(aid or 0)):
                continue
            screen = self.world_to_screen(point_xy[0], point_xy[1])
            fill = _uav_color(aid, 220) if aid > 0 else _qcolor("#475569", 220)
            heading = self._state.uav_heading_deg[idx] if idx < len(self._state.uav_heading_deg) else None
            if heading is not None and self._state.show_turn_overlays:
                self._draw_turn_radius_circles(
                    painter,
                    point_xy,
                    float(heading),
                    _uav_color(aid, 120) if aid > 0 else _qcolor("#475569", 120),
                )
                self._draw_turn_prediction_points(
                    painter,
                    point_xy,
                    float(heading),
                    _uav_color(aid, 220) if aid > 0 else _qcolor("#475569", 220),
                )
            painter.setPen(QPen(QColor("#ffffff"), 2.0))
            painter.setBrush(fill)
            painter.drawEllipse(screen, 8.0, 8.0)
            painter.setPen(QColor("#0f172a"))
            text_rect = QRectF(screen.x() + 10.0, screen.y() - 12.0, 74.0, 24.0)
            painter.drawText(text_rect, Qt.AlignLeft | Qt.AlignVCenter, f"UAV{aid or idx + 1}")
            if heading is not None:
                self._draw_bearing_arrow(painter, point_xy, float(heading), fill, length_px=34.0)

    def _draw_status_overlay(self, painter: QPainter) -> None:
        message = {
            MODE_IDLE: "좌측에서 영역 또는 Line 입력을 시작하세요.",
            MODE_DRAW_AREA: "영역 입력: 좌클릭으로 점 추가, 우클릭으로 영역 닫기",
            MODE_DRAW_LINE: "Line 입력: 좌클릭으로 점 추가, 우클릭으로 폭 입력 단계 전환",
            MODE_LINE_WIDTH_PENDING: "폭 입력 단계: 좌측 폭 값을 확인한 뒤 캔버스에서 우클릭하면 확정됩니다.",
            MODE_MISSION_READY: "임무 형상이 고정되었습니다. UAV 대수를 확정하고 위치를 입력하세요.",
            MODE_PLACE_UAV: "UAV 위치 입력: 좌클릭으로 배치 위치를 입력하세요.",
            MODE_RESULT_READY: "분할과 예상 경로가 표시되었습니다. 필요하면 다시 입력해 재실행할 수 있습니다.",
        }.get(self._state.mode, "")
        if self._state.mode == MODE_SET_UAV_HEADING:
            message = (
                f"UAV heading 설정: 지도에서 한 번 더 클릭해 방향을 확정하세요. "
                f"(40m/s, phi ±{int(TURN_PREVIEW_BANK_DEG)}°, R {int(TURN_PREVIEW_RADIUS_M)}m, "
                f"{int(TURN_PREVIEW_HORIZON_S)}초 예측점)"
            )
        if self._state.mode == MODE_RESULT_READY and self._state.split_result is not None and not self._state.expected_paths:
            message = "Area Division 결과만 표시 중입니다. 형상이나 UAV 입력을 바꾼 뒤 다시 실행할 수 있습니다."
        if not message:
            return

        rect = QRectF(
            self._map_rect.left() + 12.0,
            self._map_rect.top() + 12.0,
            min(430.0, self._map_rect.width() - 24.0),
            34.0,
        )
        painter.setPen(Qt.NoPen)
        painter.setBrush(_qcolor("#0f172a", 205))
        painter.drawRoundedRect(rect, 8.0, 8.0)
        painter.setPen(QColor("#f8fafc"))
        painter.drawText(rect.adjusted(12.0, 0.0, -12.0, 0.0), Qt.AlignVCenter | Qt.AlignLeft, message)

    def _draw_hover_label(self, painter: QPainter) -> None:
        if self._hover_xy is None:
            return
        rect = QRectF(self._map_rect.left(), self._map_rect.bottom() + 10.0, self._map_rect.width(), 24.0)
        painter.setPen(QColor("#0f172a"))
        painter.drawText(rect, Qt.AlignLeft | Qt.AlignVCenter, _format_coord(self._hover_xy))

    def _draw_polyline(self, painter: QPainter, points_xy: Sequence[Tuple[float, float]]) -> None:
        if len(points_xy) < 2:
            return
        for idx in range(len(points_xy) - 1):
            p0 = self.world_to_screen(points_xy[idx][0], points_xy[idx][1])
            p1 = self.world_to_screen(points_xy[idx + 1][0], points_xy[idx + 1][1])
            painter.drawLine(p0, p1)

    def _draw_indexed_points(
        self,
        painter: QPainter,
        points_xy: Sequence[Tuple[float, float]],
        color: QColor,
        radius: float,
    ) -> None:
        for idx, point_xy in enumerate(points_xy, start=1):
            screen = self.world_to_screen(point_xy[0], point_xy[1])
            painter.setPen(QPen(QColor("#ffffff"), 1.5))
            painter.setBrush(color)
            painter.drawEllipse(screen, radius, radius)
            painter.setPen(QColor("#0f172a"))
            painter.drawText(
                QRectF(screen.x() + 6.0, screen.y() - 10.0, 24.0, 20.0),
                Qt.AlignLeft | Qt.AlignVCenter,
                str(idx),
            )

    def _draw_point_markers(
        self,
        painter: QPainter,
        points_xy: Sequence[Tuple[float, float]],
        color: QColor,
        *,
        radius: float = 3.6,
        pen_width: float = 1.1,
    ) -> None:
        if not points_xy:
            return
        painter.save()
        painter.setPen(QPen(QColor("#ffffff"), pen_width))
        painter.setBrush(color)
        for point_xy in points_xy:
            screen = self.world_to_screen(point_xy[0], point_xy[1])
            painter.drawEllipse(screen, radius, radius)
        painter.restore()

    def _draw_named_points(
        self,
        painter: QPainter,
        labeled_points: Sequence[Dict[str, Any]],
        color: QColor,
    ) -> None:
        if not labeled_points:
            return
        painter.save()
        painter.setPen(QPen(QColor("#ffffff"), 1.1))
        painter.setBrush(color)
        for row in labeled_points:
            if not isinstance(row, dict):
                continue
            point_xy = row.get("xy")
            label = str(row.get("label", "") or "")
            if not (isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2 and label):
                continue
            screen = self.world_to_screen(float(point_xy[0]), float(point_xy[1]))
            painter.drawEllipse(screen, 2.8, 2.8)
            painter.setPen(QPen(color.darker(220), 1.0))
            rect = QRectF(screen.x() + 5.0, screen.y() - 11.0, 34.0, 18.0)
            painter.drawText(rect, Qt.AlignLeft | Qt.AlignVCenter, label)
            painter.setPen(QPen(QColor("#ffffff"), 1.1))
        painter.restore()

    def _draw_centroid_marker(
        self,
        painter: QPainter,
        point_xy: Tuple[float, float],
        color: QColor,
    ) -> None:
        screen = self.world_to_screen(point_xy[0], point_xy[1])
        painter.save()
        painter.setPen(QPen(QColor("#ffffff"), 2.0))
        painter.setBrush(color)
        painter.drawEllipse(screen, 5.0, 5.0)
        painter.setPen(QPen(QColor("#0f172a"), 1.0))
        painter.drawLine(
            QPointF(screen.x() - 8.0, screen.y()),
            QPointF(screen.x() + 8.0, screen.y()),
        )
        painter.drawLine(
            QPointF(screen.x(), screen.y() - 8.0),
            QPointF(screen.x(), screen.y() + 8.0),
        )
        painter.restore()

    def _draw_arrow(
        self,
        painter: QPainter,
        p0_xy: Tuple[float, float],
        p1_xy: Tuple[float, float],
        color: QColor,
    ) -> None:
        p0 = self.world_to_screen(p0_xy[0], p0_xy[1])
        p1 = self.world_to_screen(p1_xy[0], p1_xy[1])
        dx = p1.x() - p0.x()
        dy = p1.y() - p0.y()
        if abs(dx) < 1e-9 and abs(dy) < 1e-9:
            return
        ang = math.atan2(dy, dx)
        size = 8.0
        left = QPointF(
            p1.x() - math.cos(ang - math.pi / 6.0) * size,
            p1.y() - math.sin(ang - math.pi / 6.0) * size,
        )
        right = QPointF(
            p1.x() - math.cos(ang + math.pi / 6.0) * size,
            p1.y() - math.sin(ang + math.pi / 6.0) * size,
        )
        painter.setPen(QPen(color, 1.0))
        painter.setBrush(color)
        painter.drawPolygon(QPolygonF([p1, left, right]))

    def _draw_bearing_arrow(
        self,
        painter: QPainter,
        origin_xy: Tuple[float, float],
        bearing_deg: float,
        color: QColor,
        *,
        length_px: float = 42.0,
    ) -> None:
        origin = self.world_to_screen(origin_xy[0], origin_xy[1])
        theta = math.radians(float(bearing_deg) % 360.0)
        end = QPointF(
            origin.x() + (math.sin(theta) * float(length_px)),
            origin.y() - (math.cos(theta) * float(length_px)),
        )
        painter.save()
        pen = QPen(color, 2.2)
        pen.setCapStyle(Qt.RoundCap)
        painter.setPen(pen)
        painter.drawLine(origin, end)

        ang = math.atan2(end.y() - origin.y(), end.x() - origin.x())
        size = 8.0
        left = QPointF(
            end.x() - math.cos(ang - math.pi / 6.0) * size,
            end.y() - math.sin(ang - math.pi / 6.0) * size,
        )
        right = QPointF(
            end.x() - math.cos(ang + math.pi / 6.0) * size,
            end.y() - math.sin(ang + math.pi / 6.0) * size,
        )
        painter.setBrush(color)
        painter.drawPolygon(QPolygonF([end, left, right]))
        painter.restore()

    def _grid_step(self, half_span_m: float) -> float:
        target = half_span_m / 5.0
        steps = [25.0, 50.0, 100.0, 200.0, 250.0, 500.0, 1_000.0, 2_000.0, 5_000.0]
        for step in steps:
            if target <= step:
                return step
        return steps[-1]

    def _piece_lookup(self, split_result: Optional[SplitRunResult]) -> Dict[Tuple[int, int], SplitPiece]:
        out: Dict[Tuple[int, int], SplitPiece] = {}
        if split_result is None:
            return out
        for piece in split_result.pieces:
            out[(int(piece.parent_order), int(piece.piece_index))] = piece
        return out

    def _waypoint_start_offsets_by_uav(
        self,
        rows: Sequence[Dict[str, Any]],
        *,
        waypoint_key: str,
    ) -> Dict[int, int]:
        per_uav: Dict[int, List[Tuple[int, int, int]]] = {}
        for row_idx, row in enumerate(rows):
            if not isinstance(row, dict):
                continue
            aid = int(row.get("aircraftID", 0) or 0)
            if aid <= 0:
                continue
            wps = row.get(waypoint_key)
            if not isinstance(wps, list) or not wps:
                continue
            pid = int(row.get("pathID", 0) or 0)
            per_uav.setdefault(aid, []).append((pid, row_idx, len(wps)))

        offsets: Dict[int, int] = {}
        for aid, items in per_uav.items():
            _ = aid
            cursor = 0
            for _pid, row_idx, wp_count in sorted(items, key=lambda item: (item[0], item[1])):
                offsets[int(row_idx)] = int(cursor)
                cursor += int(wp_count)
        return offsets

    def _path_uav_id(self, row: Dict[str, Any], piece_lookup: Dict[Tuple[int, int], SplitPiece]) -> int:
        parent = int(row.get("parentOrder", 0) or 0)
        pair = row.get("pairPieces")
        if isinstance(pair, list) and pair:
            try:
                first_idx = int(pair[0])
            except Exception:
                first_idx = 0
            piece = piece_lookup.get((parent, first_idx))
            return int(piece.assigned_uav or 0) if piece is not None else 0

        try:
            piece_idx = int(row.get("index", 0) or 0)
        except Exception:
            piece_idx = 0
        piece = piece_lookup.get((parent, piece_idx))
        return int(piece.assigned_uav or 0) if piece is not None else 0

    def _piece_center_xy(
        self,
        piece: SplitPiece,
        piece_lookup: Dict[Tuple[int, int], SplitPiece],
    ) -> Optional[Tuple[float, float]]:
        _ = piece_lookup
        coords = coords_to_xy((piece.data or {}).get("coordinateList", []))
        if not coords:
            return None
        poly = Polygon(coords)
        if poly.is_empty:
            return centroid_xy(coords)
        center = poly.centroid
        return float(center.x), float(center.y)

    def _bearing_from_points_xy(
        self,
        start_xy: Tuple[float, float],
        end_xy: Tuple[float, float],
    ) -> Optional[float]:
        dx = float(end_xy[0]) - float(start_xy[0])
        dy = float(end_xy[1]) - float(start_xy[1])
        if abs(dx) < 1e-9 and abs(dy) < 1e-9:
            return None
        return float((math.degrees(math.atan2(dx, dy)) + 360.0) % 360.0)

    def _piece_bearing_deg(self, piece: SplitPiece) -> Optional[float]:
        data = piece.data if isinstance(piece.data, dict) else {}
        for key in ("phaseMoveBearing_deg", "bearing_deg", "bearingIn_deg", "bearingFromPrev"):
            value = data.get(key)
            try:
                if value is None:
                    continue
                return float(value) % 360.0
            except Exception:
                continue

        for key in ("Centerline", "coordinateList", "rawCoordinateList"):
            coords = coords_to_xy(data.get(key, []))
            if len(coords) < 2:
                continue
            bearing = self._bearing_from_points_xy(coords[0], coords[-1])
            if bearing is not None:
                return bearing
        return None


class GanttChartWidget(QWidget):
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._paths: List[Dict[str, Any]] = []
        self._visible_uav_ids: List[int] = []
        self.setObjectName("GanttChart")
        self.setMinimumHeight(170)
        self.setMaximumHeight(220)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    def set_paths(self, rows: Sequence[Dict[str, Any]], visible_uav_ids: Sequence[int]) -> None:
        self._paths = [dict(row) for row in rows if isinstance(row, dict)]
        self._visible_uav_ids = [int(aid) for aid in visible_uav_ids]
        self.update()

    def _timeline_rows(self, row: Dict[str, Any]) -> List[Dict[str, Any]]:
        timeline_rows = row.get("timelineRows")
        out: List[Dict[str, Any]] = []
        if isinstance(timeline_rows, list):
            for item in timeline_rows:
                if not isinstance(item, dict):
                    continue
                try:
                    eta_val = float(item.get("etaSec", 0.0) or 0.0)
                except Exception:
                    continue
                out.append(
                    {
                        "label": str(item.get("label", "") or ""),
                        "kind": str(item.get("kind", "") or ""),
                        "etaSec": eta_val,
                    }
                )
        out.sort(key=lambda item: float(item.get("etaSec", 0.0) or 0.0))
        return out

    def _estimated_total_sec(self, row: Dict[str, Any]) -> float:
        try:
            total_sec = float(row.get("estimatedTotalSec", 0.0) or 0.0)
        except Exception:
            total_sec = 0.0
        if total_sec > 0.0:
            return total_sec
        timeline_rows = self._timeline_rows(row)
        if timeline_rows:
            return float(max(item.get("etaSec", 0.0) or 0.0 for item in timeline_rows))
        return 0.0

    def _phase_rows(self, row: Dict[str, Any]) -> List[Dict[str, Any]]:
        phase_rows = row.get("phaseRows")
        out: List[Dict[str, Any]] = []
        if isinstance(phase_rows, list):
            for item in phase_rows:
                if not isinstance(item, dict):
                    continue
                try:
                    start_sec = float(item.get("startSec", 0.0) or 0.0)
                    end_sec = float(item.get("endSec", 0.0) or 0.0)
                except Exception:
                    continue
                if end_sec <= start_sec + 1e-6:
                    continue
                out.append(
                    {
                        "label": str(item.get("label", "") or ""),
                        "kind": str(item.get("kind", "") or ""),
                        "startSec": start_sec,
                        "endSec": end_sec,
                    }
                )
        if out:
            return out
        total_sec = self._estimated_total_sec(row)
        if total_sec > 0.0:
            return [{"label": "Route", "kind": "route", "startSec": 0.0, "endSec": total_sec}]
        return []

    def _grid_step_sec(self, max_sec: float) -> float:
        if max_sec <= 30.0:
            return 5.0
        if max_sec <= 60.0:
            return 10.0
        if max_sec <= 120.0:
            return 20.0
        if max_sec <= 240.0:
            return 30.0
        return 60.0

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QColor("#eef3f9"))

        panel = QRectF(8.0, 8.0, float(self.width()) - 16.0, float(self.height()) - 16.0)
        painter.setPen(QPen(QColor("#d3dde8"), 1.0))
        painter.setBrush(QColor("#ffffff"))
        painter.drawRoundedRect(panel, 12.0, 12.0)

        title_rect = QRectF(panel.left() + 14.0, panel.top() + 8.0, panel.width() - 28.0, 18.0)
        painter.setPen(QColor("#0f172a"))
        painter.drawText(title_rect, Qt.AlignLeft | Qt.AlignVCenter, "WP ETA / Gantt")

        rows = [
            row
            for row in self._paths
            if isinstance(row, dict) and int(row.get("aircraftID", 0) or 0) in self._visible_uav_ids
        ]
        rows.sort(key=lambda item: int(item.get("aircraftID", 0) or 0))
        max_total_sec = max((self._estimated_total_sec(row) for row in rows), default=0.0)

        if not rows or max_total_sec <= 0.0:
            empty_rect = QRectF(panel.left() + 14.0, panel.top() + 30.0, panel.width() - 28.0, panel.height() - 40.0)
            painter.setPen(QColor("#64748b"))
            painter.drawText(empty_rect, Qt.AlignCenter, "경로 ETA 데이터가 아직 없습니다.")
            return

        left_label_w = 90.0
        right_pad = 34.0
        top_axis_h = 22.0
        chart_rect = QRectF(
            panel.left() + 14.0 + left_label_w,
            panel.top() + 30.0 + top_axis_h,
            panel.width() - 28.0 - left_label_w - right_pad,
            panel.height() - 46.0 - top_axis_h,
        )
        label_rect = QRectF(panel.left() + 14.0, chart_rect.top(), left_label_w - 8.0, chart_rect.height())

        grid_step_sec = self._grid_step_sec(max_total_sec)
        tick_sec = 0.0
        while tick_sec <= max_total_sec + 1e-6:
            x = chart_rect.left() + (chart_rect.width() * (tick_sec / max_total_sec))
            painter.setPen(QPen(QColor("#dbe4ef"), 1.0))
            painter.drawLine(QPointF(x, chart_rect.top()), QPointF(x, chart_rect.bottom()))
            tick_rect = QRectF(x - 18.0, panel.top() + 30.0, 36.0, 14.0)
            painter.setPen(QColor("#475569"))
            painter.drawText(tick_rect, Qt.AlignCenter, f"{int(tick_sec)}s")
            tick_sec += grid_step_sec

        row_h = chart_rect.height() / float(max(1, len(rows)))
        for idx, row in enumerate(rows):
            aid = int(row.get("aircraftID", 0) or 0)
            target_label = str(row.get("targetLabel", "") or "")
            color = _uav_color(aid, 230)
            y0 = chart_rect.top() + (float(idx) * row_h)
            y1 = y0 + row_h
            bar_rect = QRectF(chart_rect.left(), y0 + 7.0, chart_rect.width(), max(12.0, row_h - 14.0))

            painter.setPen(QColor("#0f172a"))
            painter.drawText(
                QRectF(label_rect.left(), y0, label_rect.width(), row_h),
                Qt.AlignLeft | Qt.AlignVCenter,
                f"UAV{aid}" + (f" / {target_label}" if target_label else ""),
            )

            painter.setPen(QPen(QColor("#cbd5e1"), 1.0))
            painter.setBrush(QColor("#f8fafc"))
            painter.drawRoundedRect(bar_rect, 6.0, 6.0)

            for phase in self._phase_rows(row):
                start_sec = float(phase.get("startSec", 0.0) or 0.0)
                end_sec = float(phase.get("endSec", 0.0) or 0.0)
                x0 = chart_rect.left() + (chart_rect.width() * (start_sec / max_total_sec))
                x1 = chart_rect.left() + (chart_rect.width() * (end_sec / max_total_sec))
                phase_rect = QRectF(x0, bar_rect.top(), max(2.0, x1 - x0), bar_rect.height())
                phase_kind = str(phase.get("kind", "") or "")
                if phase_kind == "turn":
                    fill = _uav_color(aid, 170)
                elif phase_kind == "setup":
                    fill = QColor("#f59e0b")
                elif phase_kind == "waypoint":
                    fill = _uav_color(aid, 145)
                elif phase_kind in {"ingress", "straight"}:
                    fill = _uav_color(aid, 105)
                else:
                    fill = _uav_color(aid, 130)
                painter.setPen(Qt.NoPen)
                painter.setBrush(fill)
                painter.drawRoundedRect(phase_rect, 6.0, 6.0)

            for marker in self._timeline_rows(row):
                eta_sec = float(marker.get("etaSec", 0.0) or 0.0)
                x = chart_rect.left() + (chart_rect.width() * (eta_sec / max_total_sec))
                painter.setPen(QPen(color, 1.2))
                painter.drawLine(QPointF(x, bar_rect.top() - 4.0), QPointF(x, bar_rect.bottom() + 4.0))
                label = str(marker.get("label", "") or "")
                if label:
                    marker_rect = QRectF(x - 18.0, y0 - 1.0, 36.0, 14.0)
                    painter.setPen(QColor("#334155"))
                    painter.drawText(marker_rect, Qt.AlignCenter, label)

            total_sec = self._estimated_total_sec(row)
            total_rect = QRectF(chart_rect.right() + 6.0, y0, right_pad, row_h)
            painter.setPen(QColor("#334155"))
            painter.drawText(total_rect, Qt.AlignLeft | Qt.AlignVCenter, f"{total_sec:.1f}s")


class DivisionPlannerWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Division Test Mission Planner")
        self.resize(1460, 980)

        self.state = CanvasState(line_width_m=300.0)
        self._selected_uav_count = 1
        self._cmpk_payload: Optional[Dict[str, Any]] = None
        self._mrpk_payload: Optional[Dict[str, Any]] = None
        self._fov_db_rows_cache: Optional[List[Dict[str, float]]] = None
        self._fov_db_widths_cache: Optional[List[float]] = None

        self._build_ui()
        self._apply_style()
        self._sync_canvas()
        self._refresh_ui()

    def _bearing_from_points_xy(
        self,
        start_xy: Tuple[float, float],
        end_xy: Tuple[float, float],
    ) -> Optional[float]:
        dx = float(end_xy[0]) - float(start_xy[0])
        dy = float(end_xy[1]) - float(start_xy[1])
        if abs(dx) < 1e-9 and abs(dy) < 1e-9:
            return None
        return float((math.degrees(math.atan2(dx, dy)) + 360.0) % 360.0)

    def _piece_bearing_deg(self, piece: SplitPiece) -> Optional[float]:
        data = piece.data if isinstance(piece.data, dict) else {}
        for key in ("phaseMoveBearing_deg", "bearing_deg", "bearingIn_deg", "bearingFromPrev"):
            value = data.get(key)
            try:
                if value is None:
                    continue
                return float(value) % 360.0
            except Exception:
                continue

        for key in ("Centerline", "coordinateList", "rawCoordinateList"):
            coords = coords_to_xy(data.get(key, []))
            if len(coords) < 2:
                continue
            bearing = self._bearing_from_points_xy(coords[0], coords[-1])
            if bearing is not None:
                return bearing
        return None

    def _build_ui(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)
        layout = QHBoxLayout(root)
        layout.setContentsMargins(14, 14, 14, 14)
        layout.setSpacing(14)

        left_panel = QFrame()
        left_panel.setObjectName("LeftPanel")
        left_panel.setMaximumWidth(390)
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(18, 18, 18, 18)
        left_layout.setSpacing(12)
        layout.addWidget(left_panel, 0)

        title = QLabel("단일 임무 분할 / 경로 계획")
        title.setObjectName("PanelTitle")
        left_layout.addWidget(title)

        subtitle = QLabel(
            f"기준 원점 LLA: {_ORIGIN_LAT:.6f}, {_ORIGIN_LON:.6f}\n"
            "초기 가시 범위: 동/서/남/북 각 2km"
        )
        subtitle.setObjectName("MutedLabel")
        subtitle.setWordWrap(True)
        left_layout.addWidget(subtitle)

        step1 = QGroupBox("1. 임무 형상 입력")
        step1_layout = QVBoxLayout(step1)
        step1_layout.setSpacing(10)
        self.btn_start_area = QPushButton("영역 입력 시작")
        self.btn_start_area.clicked.connect(self._start_area_input)
        self.btn_start_line = QPushButton("Line 입력 시작")
        self.btn_start_line.clicked.connect(self._start_line_input)
        self.btn_undo = QPushButton("마지막 점 취소")
        self.btn_undo.clicked.connect(self._undo_last_point)
        self.btn_reset_view = QPushButton("뷰 초기화")
        self.btn_reset_view.clicked.connect(self._reset_view)
        self.btn_reset_all = QPushButton("전체 초기화")
        self.btn_reset_all.clicked.connect(self._reset_all)

        row1 = QHBoxLayout()
        row1.addWidget(self.btn_start_area)
        row1.addWidget(self.btn_start_line)
        step1_layout.addLayout(row1)

        row2 = QHBoxLayout()
        row2.addWidget(self.btn_undo)
        row2.addWidget(self.btn_reset_view)
        step1_layout.addLayout(row2)
        step1_layout.addWidget(self.btn_reset_all)

        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignLeft)
        self.spin_line_width = QDoubleSpinBox()
        self.spin_line_width.setRange(10.0, 5_000.0)
        self.spin_line_width.setDecimals(1)
        self.spin_line_width.setSingleStep(10.0)
        self.spin_line_width.setSuffix(" m")
        self.spin_line_width.setValue(self.state.line_width_m)
        self.spin_line_width.valueChanged.connect(self._on_line_width_changed)
        form.addRow("Line 폭", self.spin_line_width)
        step1_layout.addLayout(form)

        self.lbl_mission_state = QLabel("-")
        self.lbl_mission_state.setWordWrap(True)
        self.lbl_mission_state.setObjectName("StatusLabel")
        step1_layout.addWidget(self.lbl_mission_state)
        left_layout.addWidget(step1)

        step2 = QGroupBox("2. UAV 대수 선택")
        step2_layout = QVBoxLayout(step2)
        step2_layout.setSpacing(10)
        self.cmb_uav_count = QComboBox()
        self.cmb_uav_count.addItems(["1", "2", "3"])
        self.cmb_uav_count.currentIndexChanged.connect(self._on_uav_count_changed)
        self.btn_confirm_uav_count = QPushButton("다음")
        self.btn_confirm_uav_count.clicked.connect(self._confirm_uav_count)
        step2_layout.addWidget(self.cmb_uav_count)
        step2_layout.addWidget(self.btn_confirm_uav_count)
        self.lbl_uav_count = QLabel("-")
        self.lbl_uav_count.setWordWrap(True)
        self.lbl_uav_count.setObjectName("StatusLabel")
        step2_layout.addWidget(self.lbl_uav_count)
        left_layout.addWidget(step2)

        step3 = QGroupBox("3. UAV 위치 입력 / 계획 실행")
        step3_layout = QVBoxLayout(step3)
        step3_layout.setSpacing(10)
        self.btn_start_uav_input = QPushButton("UAV 위치 입력 시작")
        self.btn_start_uav_input.clicked.connect(self._start_uav_input)
        self.btn_run_area_division = QPushButton("Area Division Run")
        self.btn_run_area_division.clicked.connect(self._run_area_division)
        self.btn_mid_line_generation = QPushButton("Mid Line Generation")
        self.btn_mid_line_generation.clicked.connect(self._generate_mid_lines)
        self.btn_make_new_area = QPushButton("Make New Area")
        self.btn_make_new_area.clicked.connect(self._make_new_area)
        self.btn_mid_line_generation_2 = QPushButton("Mid Line Generation 2")
        self.btn_mid_line_generation_2.clicked.connect(self._generate_mid_lines_2)
        self.btn_assignment_path_1 = QPushButton("Assignment - Path 1")
        self.btn_assignment_path_1.clicked.connect(self._assignment_path_1)
        self.btn_make_waypoint = QPushButton("Make Waypoint")
        self.btn_make_waypoint.clicked.connect(self._make_waypoint)
        self.btn_make_path = QPushButton("Make Path")
        self.btn_make_path.clicked.connect(self._make_path)
        self.btn_check_visibility = QPushButton("Check Visibility")
        self.btn_check_visibility.clicked.connect(self._check_visibility)
        self.btn_stage2_area_division = QPushButton("Stage 2 Area Division")
        self.btn_stage2_area_division.clicked.connect(self._run_stage2_area_division)
        self.btn_run_plan = QPushButton("실제 파이프라인 실행")
        self.btn_run_plan.clicked.connect(self._run_planning)
        ratio_form = QFormLayout()
        ratio_form.setLabelAlignment(Qt.AlignLeft)
        self.stage2_ratio_spins: Dict[int, QDoubleSpinBox] = {}
        for aid in _UAV_IDS:
            spin = QDoubleSpinBox()
            spin.setRange(0.0, 100.0)
            spin.setDecimals(2)
            spin.setSingleStep(0.1)
            spin.setValue(1.0)
            self.stage2_ratio_spins[int(aid)] = spin
            ratio_form.addRow(f"UAV{aid} ratio", spin)
        step3_layout.addWidget(self.btn_start_uav_input)
        step3_layout.addWidget(self.btn_run_area_division)
        step3_layout.addWidget(self.btn_mid_line_generation)
        step3_layout.addWidget(self.btn_make_new_area)
        step3_layout.addWidget(self.btn_mid_line_generation_2)
        step3_layout.addWidget(self.btn_assignment_path_1)
        step3_layout.addWidget(self.btn_make_waypoint)
        step3_layout.addWidget(self.btn_make_path)
        self.lbl_uav_positions = QLabel("-")
        self.lbl_uav_positions.setWordWrap(True)
        self.lbl_uav_positions.setObjectName("StatusLabel")
        step3_layout.addWidget(self.lbl_uav_positions)
        left_layout.addWidget(step3)

        view_box = QGroupBox("4. UAV 시각화")
        view_layout = QVBoxLayout(view_box)
        view_layout.setSpacing(8)
        self.chk_view_uav4 = QCheckBox("UAV4")
        self.chk_view_uav5 = QCheckBox("UAV5")
        self.chk_view_uav6 = QCheckBox("UAV6")
        self.chk_view_uav4.setStyleSheet(f"color: {UAV_COLORS[4]}; font-weight: 600;")
        self.chk_view_uav5.setStyleSheet(f"color: {UAV_COLORS[5]}; font-weight: 600;")
        self.chk_view_uav6.setStyleSheet(f"color: {UAV_COLORS[6]}; font-weight: 600;")
        for chk in (self.chk_view_uav4, self.chk_view_uav5, self.chk_view_uav6):
            chk.setChecked(True)
            chk.toggled.connect(lambda _checked: self._refresh_ui())
            view_layout.addWidget(chk)
        left_layout.addWidget(view_box)

        result_box = QGroupBox("결과")
        result_layout = QVBoxLayout(result_box)
        result_layout.setSpacing(8)
        self.txt_result = QPlainTextEdit()
        self.txt_result.setReadOnly(True)
        result_layout.addWidget(self.txt_result, 1)
        left_layout.addWidget(result_box, 1)

        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(10)
        self.canvas = PlanningCanvas()
        self.canvas.worldLeftClicked.connect(self._on_canvas_left_click)
        self.canvas.worldRightClicked.connect(self._on_canvas_right_click)
        self.canvas.hoverTextChanged.connect(self.statusBar().showMessage)
        self.gantt_chart = GanttChartWidget()
        right_layout.addWidget(self.canvas, 1)
        right_layout.addWidget(self.gantt_chart, 0)
        layout.addWidget(right_panel, 1)

    def _apply_style(self) -> None:
        self.setStyleSheet(
            """
            QMainWindow { background: #eef3f9; }
            #LeftPanel {
                background: #f8fbff;
                border: 1px solid #d6dee9;
                border-radius: 12px;
            }
            #PanelTitle { font-size: 20px; font-weight: 700; color: #0f172a; }
            #MutedLabel { color: #475569; }
            #StatusLabel {
                color: #1e293b;
                background: #edf2f7;
                border-radius: 8px;
                padding: 8px 10px;
            }
            QGroupBox {
                font-weight: 700;
                color: #0f172a;
                border: 1px solid #d3dde8;
                border-radius: 10px;
                margin-top: 10px;
                background: #ffffff;
            }
            QGroupBox::title { left: 12px; padding: 0 4px; }
            QPushButton {
                min-height: 22px;
                border: 1px solid #c7d2df;
                border-radius: 7px;
                background: #f8fafc;
                color: #0f172a;
                padding: 1px 6px;
            }
            QPushButton:hover { background: #eff6ff; }
            QPushButton:disabled { color: #8a99ab; background: #e7edf4; }
            QComboBox, QDoubleSpinBox, QPlainTextEdit {
                border: 1px solid #cbd5e1;
                border-radius: 8px;
                background: #ffffff;
                padding: 6px;
            }
            #GanttChart {
                background: #eef3f9;
                border: none;
            }
            """
        )

    def _sync_canvas(self) -> None:
        self.canvas.set_state(
            CanvasState(
                mode=self.state.mode,
                mission_kind=self.state.mission_kind,
                draft_points_xy=list(self.state.draft_points_xy),
                mission_points_xy=list(self.state.mission_points_xy),
                line_width_m=float(self.state.line_width_m),
                line_width_pending=bool(self.state.line_width_pending),
                uav_positions_xy=list(self.state.uav_positions_xy),
                uav_heading_deg=list(self.state.uav_heading_deg),
                uav_ids=list(self.state.uav_ids),
                split_result=self.state.split_result,
                expected_paths=list(self.state.expected_paths),
                flight_plans_0303=list(self.state.flight_plans_0303),
                flight_plans_0304=list(self.state.flight_plans_0304),
                visibility_segments=list(self.state.visibility_segments),
                mid_line_segments=list(self.state.mid_line_segments),
                tangent_checks=list(self.state.tangent_checks),
                show_turn_overlays=bool(self.state.show_turn_overlays),
                visible_uav_ids=self._visible_uav_ids(),
            )
        )
        self.gantt_chart.set_paths(self.state.expected_paths, self._visible_uav_ids())

    def _uav_inputs_complete(self) -> bool:
        if not self.state.uav_ids:
            return False
        if len(self.state.uav_positions_xy) != len(self.state.uav_ids):
            return False
        if len(self.state.uav_heading_deg) != len(self.state.uav_positions_xy):
            return False
        return all(heading is not None for heading in self.state.uav_heading_deg)

    def _pending_uav_heading_index(self) -> Optional[int]:
        if self.state.mode != MODE_SET_UAV_HEADING:
            return None
        if not self.state.uav_positions_xy:
            return None
        idx = len(self.state.uav_positions_xy) - 1
        if idx < 0:
            return None
        if idx >= len(self.state.uav_heading_deg):
            return idx
        return idx if self.state.uav_heading_deg[idx] is None else None

    def _pop_last_uav_input(self) -> bool:
        if not self.state.uav_positions_xy:
            return False
        self.state.uav_positions_xy.pop()
        if self.state.uav_heading_deg:
            self.state.uav_heading_deg.pop()
        self.state.mode = MODE_PLACE_UAV if self.state.uav_ids else MODE_MISSION_READY
        return True

    def _refresh_ui(self) -> None:
        mission_ready = bool(self.state.mission_points_xy)
        line_mode = self.state.mission_kind == MISSION_LINE
        can_confirm_uav = mission_ready
        uav_count_locked = bool(self.state.uav_ids)
        expected_uav_count = len(self.state.uav_ids) if self.state.uav_ids else self._selected_uav_count
        uav_done = self._uav_inputs_complete() if expected_uav_count > 0 else False
        mid_line_ready = bool(self.state.split_result is not None and bool(self.state.split_result.pieces))
        make_new_area_ready = bool(self.state.mid_line_segments)
        mid_line2_ready = bool(
            self._has_make_new_area_result()
            and self.state.mid_line_segments
            and any(int(piece.assigned_uav or 0) > 0 for piece in (self.state.split_result.pieces if self.state.split_result else []))
        )
        assignment_path_1_ready = bool(
            mid_line2_ready
            and any(
                isinstance(row, dict) and isinstance(row.get("stage2Centers"), list) and row.get("stage2Centers")
                for row in self.state.mid_line_segments
            )
        )
        make_waypoint_ready = bool(self.state.assignment_path_rows)
        make_path_ready = bool(self.state.split_result is not None and self.state.mid_line_segments)
        visibility_ready = bool(
            self.state.split_result is not None
            and any(int(piece.assigned_uav or 0) > 0 for piece in self.state.split_result.pieces)
        )
        stage2_ready = bool(
            mission_ready
            and (self.state.mission_kind == MISSION_AREA)
            and self.state.split_result is not None
            and any(int(piece.assigned_uav or 0) > 0 for piece in self.state.split_result.pieces)
        )

        self.spin_line_width.setEnabled(line_mode)
        self.cmb_uav_count.setEnabled(can_confirm_uav)
        self.btn_confirm_uav_count.setEnabled(can_confirm_uav)
        self.btn_start_uav_input.setEnabled(uav_count_locked)
        self.btn_run_area_division.setEnabled(
            mission_ready and (self.state.mission_kind == MISSION_AREA) and uav_count_locked and uav_done
        )
        self.btn_mid_line_generation.setEnabled(mid_line_ready)
        self.btn_mid_line_generation.setVisible(mid_line_ready)
        self.btn_make_new_area.setEnabled(make_new_area_ready)
        self.btn_make_new_area.setVisible(make_new_area_ready)
        self.btn_mid_line_generation_2.setEnabled(mid_line2_ready)
        self.btn_mid_line_generation_2.setVisible(mid_line2_ready)
        self.btn_assignment_path_1.setEnabled(assignment_path_1_ready)
        self.btn_assignment_path_1.setVisible(assignment_path_1_ready)
        self.btn_make_waypoint.setEnabled(make_waypoint_ready)
        self.btn_make_waypoint.setVisible(make_waypoint_ready)
        self.btn_make_path.setEnabled(make_path_ready)
        self.btn_make_path.setVisible(make_path_ready)
        self.btn_stage2_area_division.setEnabled(False)
        self.btn_stage2_area_division.setVisible(False)
        self.btn_check_visibility.setEnabled(False)
        self.btn_check_visibility.setVisible(False)
        self.btn_run_plan.setEnabled(False)
        self.btn_run_plan.setVisible(False)
        self.btn_undo.setEnabled(bool(self.state.draft_points_xy) or bool(self.state.uav_positions_xy))
        for aid, spin in self.stage2_ratio_spins.items():
            spin.setEnabled(int(aid) in self.state.uav_ids)
            spin.setVisible(False)

        if not mission_ready:
            self.lbl_mission_state.setText(
                "영역: 좌클릭으로 점 입력 후 우클릭으로 닫기\n"
                "Line: 좌클릭으로 점 입력 후 우클릭 -> 폭 입력 상태 -> 다시 우클릭으로 확정"
            )
        elif self.state.mission_kind == MISSION_AREA:
            self.lbl_mission_state.setText(f"영역 입력 완료\n점 개수: {len(self.state.mission_points_xy)}개")
        else:
            self.lbl_mission_state.setText(
                f"Line 입력 완료\n점 개수: {len(self.state.mission_points_xy)}개 / 폭: {self.state.line_width_m:.1f}m"
            )

        if not mission_ready:
            self.lbl_uav_count.setText("임무 형상이 확정되면 UAV 대수를 1~3대 중 선택할 수 있습니다.")
        elif not uav_count_locked:
            self.lbl_uav_count.setText(
                f"현재 선택값: {self._selected_uav_count}대\n다음을 누르면 UAV 입력 단계로 넘어갑니다."
            )
        else:
            self.lbl_uav_count.setText(f"확정 UAV: {', '.join(f'UAV{aid}' for aid in self.state.uav_ids)}")

        if not uav_count_locked:
            self.lbl_uav_positions.setText("UAV 대수를 먼저 확정하세요.")
        elif not self.state.uav_positions_xy:
            self.lbl_uav_positions.setText(
                f"입력 대기: {len(self.state.uav_ids)}대의 위치와 heading이 필요합니다.\n"
                "지도에서 한 번 클릭해 위치를 두고, 가이드 선을 확인한 뒤 한 번 더 클릭해 heading을 확정하세요."
            )
        else:
            lines = []
            for idx, point_xy in enumerate(self.state.uav_positions_xy):
                aid = self.state.uav_ids[idx] if idx < len(self.state.uav_ids) else idx + 1
                heading = self.state.uav_heading_deg[idx] if idx < len(self.state.uav_heading_deg) else None
                heading_text = f" | HDG {float(heading):.1f} deg" if heading is not None else " | heading 대기"
                lines.append(f"UAV{aid}: {_format_coord(point_xy)}{heading_text}")
            if len(self.state.uav_positions_xy) < len(self.state.uav_ids):
                lines.append(f"남은 UAV 위치 입력: {len(self.state.uav_ids) - len(self.state.uav_positions_xy)}대")
            pending_idx = self._pending_uav_heading_index()
            if pending_idx is not None and pending_idx < len(self.state.uav_ids):
                lines.append(f"UAV{self.state.uav_ids[pending_idx]}: 지도에서 한 번 더 클릭해 heading을 확정하세요.")
            self.lbl_uav_positions.setText("\n".join(lines))

        self._sync_canvas()

    def _append_result(self, text: str) -> None:
        existing = self.txt_result.toPlainText().strip()
        self.txt_result.setPlainText(f"{existing}\n{text}".strip())
        self.txt_result.verticalScrollBar().setValue(self.txt_result.verticalScrollBar().maximum())

    def _set_result(self, text: str) -> None:
        self.txt_result.setPlainText(text)

    def _clear_plan_result(self) -> None:
        self.state.split_result = None
        self.state.expected_paths = []
        self.state.assignment_path_rows = []
        self.state.flight_plans_0303 = []
        self.state.flight_plans_0304 = []
        self.state.visibility_segments = []
        self.state.mid_line_segments = []
        self.state.tangent_checks = []
        self.state.show_turn_overlays = True
        self._cmpk_payload = None
        self._mrpk_payload = None

    def _reset_all(self) -> None:
        self.state = CanvasState(line_width_m=float(self.spin_line_width.value()))
        self._selected_uav_count = int(self.cmb_uav_count.currentText())
        self._clear_plan_result()
        self._set_result("")
        self._refresh_ui()

    def _reset_view(self) -> None:
        self.canvas.reset_view()

    def _output_root(self) -> Path:
        root = Path(__file__).resolve().parent / "output"
        root.mkdir(parents=True, exist_ok=True)
        return root

    def _area_mode(self) -> str:
        try:
            return str(get_runtime_str("area_sweep_mode", "parallel") or "parallel").strip().lower()
        except Exception:
            return "parallel"

    def _review_max_segment_m(self) -> float:
        try:
            return float(get_runtime_area_review_max_segment_m(500.0))
        except Exception:
            return 500.0

    def _uav_plan_mode(self) -> str:
        try:
            raw = str(get_runtime_str("uav_plan_mode", "normal") or "normal").strip().lower()
        except Exception:
            raw = "normal"
        return "dub_path" if raw == "dub_path" else "normal"

    def _flow_mode(self) -> str:
        raw = str(os.environ.get("DIVISION_TEST_FLOW_MODE", "initial") or "initial").strip().lower()
        return "replan" if raw == "replan" else "initial"

    def _is_replan_flow(self) -> bool:
        return self._flow_mode() == "replan"

    def _assignment_summary_text(self, split_result: SplitRunResult) -> str:
        counts: Dict[int, int] = {}
        for piece in split_result.pieces:
            aid = int(piece.assigned_uav or 0)
            if aid <= 0:
                continue
            counts[aid] = int(counts.get(aid, 0)) + 1
        if not counts:
            return "-"
        return ", ".join(f"UAV{aid}={count}" for aid, count in sorted(counts.items()))

    def _has_make_new_area_result(self) -> bool:
        if self.state.split_result is None:
            return False
        for piece in self.state.split_result.pieces:
            data = piece.data if isinstance(piece.data, dict) else {}
            review = data.get("reviewArea") if isinstance(data.get("reviewArea"), dict) else {}
            if bool(review.get("makeNewAreaApplied")):
                return True
        return False

    def _dominant_edge_bearing_deg(
        self,
        points_xy: Sequence[Tuple[float, float]],
    ) -> Optional[float]:
        if len(points_xy) < 2:
            return None
        best_bearing: Optional[float] = None
        best_len_m = 0.0
        for idx in range(len(points_xy)):
            start_xy = points_xy[idx]
            end_xy = points_xy[(idx + 1) % len(points_xy)]
            seg_len_m = _distance(start_xy, end_xy)
            if seg_len_m <= best_len_m:
                continue
            bearing_deg = self._bearing_from_points_xy(start_xy, end_xy)
            if bearing_deg is None:
                continue
            best_len_m = float(seg_len_m)
            best_bearing = float(bearing_deg)
        return best_bearing

    def _mid_line_reference_bearing_deg(
        self,
        pieces: Sequence[SplitPiece],
    ) -> Optional[float]:
        for piece in sorted(pieces, key=lambda row: int(row.piece_index or 0)):
            bearing_deg = self._piece_bearing_deg(piece)
            if bearing_deg is not None:
                return float(bearing_deg)
            coords = coords_to_xy((piece.data or {}).get("coordinateList", []))
            bearing_deg = self._dominant_edge_bearing_deg(coords)
            if bearing_deg is not None:
                return float(bearing_deg)
        return self._dominant_edge_bearing_deg(self.state.mission_points_xy)

    def _fov_db_widths_m(self) -> List[float]:
        if isinstance(self._fov_db_widths_cache, list):
            return list(self._fov_db_widths_cache)

        rows = self._fov_db_rows()
        self._fov_db_widths_cache = sorted(
            {float(row.get("width", 0.0) or 0.0) for row in rows if float(row.get("width", 0.0) or 0.0) > 0.0}
        )
        return list(self._fov_db_widths_cache)

    def _fov_db_rows(self) -> List[Dict[str, float]]:
        if isinstance(self._fov_db_rows_cache, list):
            return [dict(row) for row in self._fov_db_rows_cache]

        widths: List[float] = []
        rows: List[Dict[str, float]] = []
        try:
            with _FOV_DB_PATH.open("r", encoding="utf-8-sig", newline="") as fh:
                reader = csv.DictReader(fh)
                for row in reader:
                    try:
                        width_m = float(row.get("width", 0.0) or 0.0)
                        sep_m = float(row.get("sep", 0.0) or 0.0)
                        vel = float(row.get("vel", 0.0) or 0.0)
                        fov = float(row.get("fov", 0.0) or 0.0)
                    except Exception:
                        continue
                    if width_m > 0.0:
                        widths.append(float(width_m))
                        rows.append(
                            {
                                "width": float(width_m),
                                "sep": float(sep_m),
                                "vel": float(vel),
                                "fov": float(fov),
                            }
                        )
        except Exception:
            widths = []
            rows = []

        self._fov_db_rows_cache = sorted(rows, key=lambda item: (float(item.get("width", 0.0) or 0.0), float(item.get("sep", 0.0) or 0.0)))
        self._fov_db_widths_cache = sorted(set(widths))
        return [dict(row) for row in self._fov_db_rows_cache]

    def _covering_db_width_m(self, width_m: float) -> Optional[float]:
        target_m = float(width_m)
        if target_m <= 0.0:
            return None
        for db_width_m in self._fov_db_widths_m():
            if float(db_width_m) + 1e-6 >= target_m:
                return float(db_width_m)
        return None

    def _covering_db_row(self, width_m: float) -> Optional[Dict[str, float]]:
        rows = self._fov_db_rows()
        target_m = max(0.0, float(width_m))
        if target_m <= 0.0 or not rows:
            return None
        for row in rows:
            if float(row.get("width", 0.0) or 0.0) + 1e-6 >= target_m:
                return dict(row)
        return dict(rows[-1])

    def _largest_sep_covering_db_row(self, width_m: float) -> Optional[Dict[str, float]]:
        rows = self._fov_db_rows()
        target_m = max(0.0, float(width_m))
        if target_m <= 0.0 or not rows:
            return None

        candidates = [
            dict(row)
            for row in rows
            if float(row.get("width", 0.0) or 0.0) + 1e-6 >= target_m
        ]
        if candidates:
            return min(
                candidates,
                key=lambda row: (
                    -float(row.get("sep", 0.0) or 0.0),
                    float(row.get("width", 0.0) or 0.0),
                    -float(row.get("fov", 0.0) or 0.0),
                ),
            )

        return max(
            (dict(row) for row in rows),
            key=lambda row: (
                float(row.get("width", 0.0) or 0.0),
                float(row.get("sep", 0.0) or 0.0),
            ),
        )

    def _mid_line_overlay_bundle(
        self,
        split_result: Optional[SplitRunResult],
    ) -> Tuple[float, List[Dict[str, Any]], List[str]]:
        if split_result is None or not split_result.pieces:
            raise ValueError("Mid line generation requires split polygons.")

        pieces = [
            piece
            for piece in sorted(split_result.pieces, key=lambda row: int(row.piece_index or 0))
            if len(coords_to_xy((piece.data or {}).get("coordinateList", []))) >= 3
        ]
        if not pieces:
            raise ValueError("Mid line generation requires split polygons.")

        reference_bearing_deg = self._mid_line_reference_bearing_deg(pieces)
        if reference_bearing_deg is None:
            raise ValueError("Unable to determine a reference bearing for the split pieces.")

        overlays: List[Dict[str, Any]] = []
        lines = [f"[MID] generation refBearing={float(reference_bearing_deg):.1f}deg"]
        for piece in pieces:
            coords_xy = coords_to_xy((piece.data or {}).get("coordinateList", []))
            geometry = self._mid_line_overlay_geometry(coords_xy, float(reference_bearing_deg))
            if geometry is None:
                continue
            aid = int(piece.assigned_uav or 0)
            overlays.append(
                {
                    "pieceIndex": int(piece.piece_index or 0),
                    "aircraftID": aid,
                    "bearingDeg": float(reference_bearing_deg),
                    **geometry,
                }
            )
            max_width_m = float(geometry.get("maxWidthM", 0.0) or 0.0)
            left_width_m = float(geometry.get("maxWidthLeftM", 0.0) or 0.0)
            right_width_m = float(geometry.get("maxWidthRightM", 0.0) or 0.0)
            db_cover_width_m = float(geometry.get("dbCoverWidthM", 0.0) or 0.0)
            db_max_width_m = float(geometry.get("dbMaxWidthM", 0.0) or 0.0)
            split_required = bool(geometry.get("midLineRequired", True))
            lines.append(
                f"  P{int(piece.piece_index or 0)}"
                f"{f' / UAV{aid}' if aid > 0 else ''}: "
                f"boxWidth {float(geometry.get('widthM', 0.0)):.0f}m"
                + (
                    f" | maxWidth {max_width_m:.0f}m"
                    f" (L {left_width_m:.0f}m / R {right_width_m:.0f}m)"
                    if max_width_m > 0.0
                    else ""
                )
                + (
                    f" | no split (DB {db_cover_width_m:.0f}m covers W)"
                    if (max_width_m > 0.0 and not split_required and db_cover_width_m > 0.0)
                    else (
                        f" | split (W>{db_max_width_m:.0f}m DB max)"
                        if (max_width_m > 0.0 and split_required and db_max_width_m > 0.0)
                        else ""
                    )
                )
            )

        if not overlays:
            raise ValueError("Unable to build mid-line geometry from the split result.")
        return float(reference_bearing_deg), overlays, lines

    def _mid_line_overlay_geometry(
        self,
        coords_xy: Sequence[Tuple[float, float]],
        bearing_deg: float,
    ) -> Optional[Dict[str, Any]]:
        if len(coords_xy) < 3:
            return None

        poly = Polygon(coords_xy)
        if not poly.is_valid:
            poly = poly.buffer(0)
        if poly.is_empty:
            return None
        if isinstance(poly, MultiPolygon):
            geoms = [geom for geom in poly.geoms if isinstance(geom, Polygon) and not geom.is_empty]
            if not geoms:
                return None
            poly = max(geoms, key=lambda geom: float(geom.area))

        theta = math.radians(float(bearing_deg) % 360.0)
        ux = math.sin(theta)
        uy = math.cos(theta)
        vx = uy
        vy = -ux

        s_vals = [(float(x) * ux) + (float(y) * uy) for (x, y) in coords_xy]
        t_vals = [(float(x) * vx) + (float(y) * vy) for (x, y) in coords_xy]
        if not s_vals or not t_vals:
            return None

        min_s = float(min(s_vals))
        max_s = float(max(s_vals))
        min_t = float(min(t_vals))
        max_t = float(max(t_vals))

        def _from_st(s_val: float, t_val: float) -> Tuple[float, float]:
            return (
                (float(s_val) * ux) + (float(t_val) * vx),
                (float(s_val) * uy) + (float(t_val) * vy),
            )

        mid_t = 0.5 * (min_t + max_t)
        mid_s = 0.5 * (min_s + max_s)
        overlay = {
            "boxXY": [
                _from_st(min_s, min_t),
                _from_st(max_s, min_t),
                _from_st(max_s, max_t),
                _from_st(min_s, max_t),
            ],
            "midLineXY": [
                _from_st(min_s, mid_t),
                _from_st(max_s, mid_t),
            ],
            "centerXY": _from_st(mid_s, mid_t),
            "lengthM": max(0.0, max_s - min_s),
            "widthM": max(0.0, max_t - min_t),
        }
        length_m = max(0.0, max_s - min_s)
        sample_count = max(25, min(161, int(math.ceil(length_m / 25.0)) + 1))
        probe_pad_m = max(60.0, float(max_t - min_t) * 0.75)

        best_total_m = -1.0
        best_geometry: Optional[Dict[str, Any]] = None
        for idx in range(sample_count):
            ratio = (float(idx) / float(sample_count - 1)) if sample_count > 1 else 0.5
            s_probe = min_s + ((max_s - min_s) * ratio)
            probe_line = LineString(
                [
                    _from_st(s_probe, min_t - probe_pad_m),
                    _from_st(s_probe, max_t + probe_pad_m),
                ]
            )
            width_line = self._longest_linestring_xy(poly.intersection(probe_line))
            if width_line is None or len(width_line.coords) < 2:
                continue

            start_xy = (float(width_line.coords[0][0]), float(width_line.coords[0][1]))
            end_xy = (float(width_line.coords[-1][0]), float(width_line.coords[-1][1]))
            start_t = (float(start_xy[0]) * vx) + (float(start_xy[1]) * vy)
            end_t = (float(end_xy[0]) * vx) + (float(end_xy[1]) * vy)
            seg_min_t = min(start_t, end_t)
            seg_max_t = max(start_t, end_t)
            if (seg_min_t - 1.0) > mid_t or (seg_max_t + 1.0) < mid_t:
                continue

            left_m = max(0.0, mid_t - seg_min_t)
            right_m = max(0.0, seg_max_t - mid_t)
            total_m = left_m + right_m
            if total_m <= best_total_m:
                continue

            best_total_m = float(total_m)
            best_geometry = {
                "maxWidthLineXY": [
                    _from_st(s_probe, seg_min_t),
                    _from_st(s_probe, seg_max_t),
                ],
                "maxWidthCenterXY": _from_st(s_probe, mid_t),
                "maxWidthLeftM": float(left_m),
                "maxWidthRightM": float(right_m),
                "maxWidthM": float(total_m),
            }

        if best_geometry is not None:
            overlay.update(best_geometry)

        max_width_m = float(overlay.get("maxWidthM", 0.0) or 0.0)
        db_widths = self._fov_db_widths_m()
        db_cover_width_m = self._covering_db_width_m(max_width_m) if db_widths and max_width_m > 0.0 else None
        overlay["dbMaxWidthM"] = float(db_widths[-1]) if db_widths else 0.0
        overlay["dbCoverWidthM"] = float(db_cover_width_m) if db_cover_width_m is not None else 0.0
        overlay["midLineRequired"] = (
            False if (max_width_m > 0.0 and db_cover_width_m is not None) else True
        )

        if not bool(overlay.get("midLineRequired", True)):
            return overlay

        split_parts: List[Dict[str, Any]] = []
        try:
            split_geom = geom_split(poly, LineString(overlay["midLineXY"]))
            part_polys = [geom for geom in getattr(split_geom, "geoms", []) if isinstance(geom, Polygon) and not geom.is_empty]
        except Exception:
            part_polys = []
        if len(part_polys) >= 2:
            part_rows: List[Tuple[float, Dict[str, Any]]] = []
            for part_poly in part_polys:
                coords_part = _dedupe_points(
                    [(float(x), float(y)) for (x, y) in list(part_poly.exterior.coords)[:-1]],
                    min_dist_m=1.0,
                )
                if len(coords_part) < 3:
                    continue
                center = part_poly.centroid
                try:
                    if center.is_empty or not part_poly.buffer(1e-6).contains(center):
                        center = part_poly.representative_point()
                except Exception:
                    center = part_poly.representative_point()
                center_t = (float(center.x) * vx) + (float(center.y) * vy)
                part_rows.append(
                    (
                        float(center_t),
                        {
                            "polygonXY": coords_part,
                            "centroidXY": (float(center.x), float(center.y)),
                        },
                    )
                )
            part_rows.sort(key=lambda item: float(item[0]))
            for part_idx, (_center_t, part_row) in enumerate(part_rows):
                prefix = "A" if part_idx == 0 else "B"
                labeled_points = [
                    {"label": f"{prefix}{idx + 1}", "xy": point_xy}
                    for idx, point_xy in enumerate(part_row["polygonXY"])
                ]
                split_parts.append(
                    {
                        "name": prefix,
                        "polygonXY": part_row["polygonXY"],
                        "centroidXY": part_row["centroidXY"],
                        "pointLabels": labeled_points,
                    }
                )
        if split_parts:
            overlay["splitParts"] = split_parts
        return overlay

    def _normalize_split_part_rows(
        self,
        part_polys: Sequence[Polygon],
        bearing_deg: float,
    ) -> List[Dict[str, Any]]:
        ux, uy, vx, vy = self._mid_line_axis_vectors(bearing_deg)
        part_rows: List[Tuple[float, Dict[str, Any]]] = []
        for part_poly in part_polys:
            if not isinstance(part_poly, Polygon) or part_poly.is_empty:
                continue
            coords_part = _dedupe_points(
                [(float(x), float(y)) for (x, y) in list(part_poly.exterior.coords)[:-1]],
                min_dist_m=1.0,
            )
            if len(coords_part) < 3:
                continue
            center = part_poly.centroid
            try:
                if center.is_empty or not part_poly.buffer(1e-6).contains(center):
                    center = part_poly.representative_point()
            except Exception:
                center = part_poly.representative_point()
            center_t = (float(center.x) * vx) + (float(center.y) * vy)
            part_rows.append(
                (
                    float(center_t),
                    {
                        "polygonXY": coords_part,
                        "centroidXY": (float(center.x), float(center.y)),
                    },
                )
            )

        part_rows.sort(key=lambda item: float(item[0]))
        out: List[Dict[str, Any]] = []
        for part_idx, (_center_t, part_row) in enumerate(part_rows):
            name = chr(ord("A") + min(part_idx, 25))
            labeled_points = [
                {"label": f"{name}{idx + 1}", "xy": point_xy}
                for idx, point_xy in enumerate(part_row["polygonXY"])
            ]
            out.append(
                {
                    "name": name,
                    "polygonXY": part_row["polygonXY"],
                    "centroidXY": part_row["centroidXY"],
                    "pointLabels": labeled_points,
                }
            )
        return out

    def _resolved_split_parts_for_overlay(
        self,
        overlay: Dict[str, Any],
    ) -> List[Dict[str, Any]]:
        bearing_deg = float(overlay.get("bearingDeg", 0.0) or 0.0)
        raw_parts = overlay.get("splitParts")
        raw_polys: List[Polygon] = []
        if isinstance(raw_parts, list):
            for part in raw_parts:
                if not isinstance(part, dict):
                    continue
                polygon_xy = part.get("polygonXY")
                if not isinstance(polygon_xy, list) or len(polygon_xy) < 3:
                    continue
                try:
                    part_poly = self._largest_polygon_xy(
                        Polygon(
                            [
                                (float(point_xy[0]), float(point_xy[1]))
                                for point_xy in polygon_xy
                                if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2
                            ]
                        ).buffer(0)
                    )
                except Exception:
                    part_poly = None
                if part_poly is not None and not part_poly.is_empty:
                    raw_polys.append(part_poly)
        if len(raw_polys) >= 2:
            return self._normalize_split_part_rows(raw_polys, bearing_deg)

        piece_index = int(overlay.get("pieceIndex", 0) or 0)
        piece = next(
            (
                piece_row
                for piece_row in (self.state.split_result.pieces if self.state.split_result is not None else [])
                if int(piece_row.piece_index or 0) == piece_index
            ),
            None,
        )
        piece_poly = self._piece_polygon_xy(piece) if piece is not None else None
        mid_line_xy = overlay.get("midLineXY")
        if (
            piece_poly is None
            or piece_poly.is_empty
            or not isinstance(mid_line_xy, list)
            or len(mid_line_xy) < 2
        ):
            return self._normalize_split_part_rows(raw_polys, bearing_deg) if raw_polys else []

        line_points = [
            (float(point_xy[0]), float(point_xy[1]))
            for point_xy in mid_line_xy
            if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2
        ]
        if len(line_points) < 2:
            return self._normalize_split_part_rows(raw_polys, bearing_deg) if raw_polys else []

        dx = float(line_points[-1][0]) - float(line_points[0][0])
        dy = float(line_points[-1][1]) - float(line_points[0][1])
        line_len_m = math.hypot(dx, dy)
        if line_len_m <= 1e-6:
            return self._normalize_split_part_rows(raw_polys, bearing_deg) if raw_polys else []

        minx, miny, maxx, maxy = piece_poly.bounds
        pad_m = max(float(maxx - minx), float(maxy - miny), line_len_m) * 2.0
        ux = dx / line_len_m
        uy = dy / line_len_m
        cutter = LineString(
            [
                (float(line_points[0][0]) - (ux * pad_m), float(line_points[0][1]) - (uy * pad_m)),
                (float(line_points[-1][0]) + (ux * pad_m), float(line_points[-1][1]) + (uy * pad_m)),
            ]
        )

        try:
            split_geom = geom_split(piece_poly, cutter)
            part_polys = [
                geom
                for geom in getattr(split_geom, "geoms", [])
                if isinstance(geom, Polygon) and not geom.is_empty
            ]
        except Exception:
            part_polys = []

        if len(part_polys) >= 2:
            return self._normalize_split_part_rows(part_polys, bearing_deg)
        return self._normalize_split_part_rows(raw_polys, bearing_deg) if raw_polys else []

    def _mid_line_axis_vectors(
        self,
        bearing_deg: float,
    ) -> Tuple[float, float, float, float]:
        theta = math.radians(float(bearing_deg) % 360.0)
        ux = math.sin(theta)
        uy = math.cos(theta)
        vx = uy
        vy = -ux
        return ux, uy, vx, vy

    def _replace_piece_polygon(
        self,
        piece: SplitPiece,
        poly: Polygon,
        *,
        review_patch: Optional[Dict[str, Any]] = None,
    ) -> None:
        coords_xy = [(float(x), float(y)) for (x, y) in list(poly.exterior.coords)[:-1]]
        if len(coords_xy) < 3:
            return
        data = copy.deepcopy(piece.data if isinstance(piece.data, dict) else {})
        old_coords = data.get("coordinateList") if isinstance(data.get("coordinateList"), list) else []
        alt_m = 0.0
        if old_coords and isinstance(old_coords[0], dict):
            alt_m = float(old_coords[0].get("altitude", 0.0) or 0.0)
        coord_list = [meters_to_coord(x, y, alt_m=alt_m) for (x, y) in coords_xy]
        data["coordinateList"] = coord_list
        data["rawCoordinateList"] = copy.deepcopy(coord_list)
        review = data.get("reviewArea") if isinstance(data.get("reviewArea"), dict) else {}
        review["makeNewAreaApplied"] = True
        if isinstance(review_patch, dict):
            for key, value in review_patch.items():
                review[str(key)] = value
        data["reviewArea"] = review
        piece.data = data

    def _prefer_raw_split_preview_polygons(
        self,
        split_result: Optional[SplitRunResult],
    ) -> None:
        if split_result is None:
            return
        for piece in split_result.pieces:
            if int(piece.mission_type) not in {2, 3, 6}:
                continue
            data = piece.data if isinstance(piece.data, dict) else None
            if not isinstance(data, dict):
                continue
            raw_coords = data.get("rawCoordinateList")
            if not (isinstance(raw_coords, list) and len(coords_to_xy(raw_coords)) >= 3):
                continue
            patched = copy.deepcopy(data)
            patched["coordinateList"] = copy.deepcopy(raw_coords)
            review = patched.get("reviewArea") if isinstance(patched.get("reviewArea"), dict) else {}
            review["previewUsesRawPolygon"] = True
            patched["reviewArea"] = review
            piece.data = patched

    def _make_new_area_polygon(
        self,
        piece: SplitPiece,
        overlay: Dict[str, Any],
        mission_poly: Optional[Polygon],
    ) -> Tuple[Optional[Polygon], Dict[str, Any]]:
        poly = self._piece_polygon_xy(piece)
        if poly is None or poly.is_empty:
            return None, {"reason": "invalidPiece"}

        bearing_deg = float(overlay.get("bearingDeg", 0.0) or 0.0)
        split_parts = overlay.get("splitParts")
        part_sources: List[Dict[str, Any]] = []
        if isinstance(split_parts, list) and split_parts:
            for part_idx, part in enumerate(split_parts):
                if not isinstance(part, dict):
                    continue
                polygon_xy = part.get("polygonXY")
                if not isinstance(polygon_xy, list) or len(polygon_xy) < 3:
                    continue
                part_sources.append(
                    {
                        "name": str(part.get("name", "") or ("A" if part_idx == 0 else "B")),
                        "polygonXY": [(float(x), float(y)) for (x, y) in polygon_xy],
                    }
                )
        if not part_sources:
            coords_xy = _dedupe_points(
                [(float(x), float(y)) for (x, y) in list(poly.exterior.coords)[:-1]],
                min_dist_m=1.0,
            )
            if len(coords_xy) < 3:
                return None, {"reason": "invalidPieceCoords", "oldAreaM2": float(poly.area)}
            part_sources = [{"name": "P", "polygonXY": coords_xy}]

        part_results: List[Dict[str, Any]] = []
        boxed_part_polys: List[Polygon] = []
        changed_any = False
        for part in part_sources:
            part_name = str(part.get("name", "") or "?")
            part_coords_xy = part.get("polygonXY")
            if not isinstance(part_coords_xy, list) or len(part_coords_xy) < 3:
                continue

            part_poly = self._largest_polygon_xy(Polygon(part_coords_xy).buffer(0))
            if part_poly is None or part_poly.is_empty:
                part_results.append({"name": part_name, "reason": "invalidPart"})
                continue

            part_overlay = self._mid_line_overlay_geometry(part_coords_xy, bearing_deg)
            if part_overlay is None:
                boxed_part_polys.append(part_poly)
                part_results.append(
                    {
                        "name": part_name,
                        "reason": "noMidLine",
                        "oldAreaM2": float(part_poly.area),
                        "newAreaM2": float(part_poly.area),
                    }
                )
                continue

            box_xy = part_overlay.get("boxXY")
            if not isinstance(box_xy, list) or len(box_xy) < 4:
                boxed_part_polys.append(part_poly)
                part_results.append(
                    {
                        "name": part_name,
                        "reason": "missingBox",
                        "oldAreaM2": float(part_poly.area),
                        "newAreaM2": float(part_poly.area),
                    }
                )
                continue

            box_poly = self._largest_polygon_xy(
                Polygon([(float(x), float(y)) for (x, y) in box_xy]).buffer(0)
            )
            if box_poly is None or box_poly.is_empty:
                boxed_part_polys.append(part_poly)
                part_results.append(
                    {
                        "name": part_name,
                        "reason": "invalidBox",
                        "oldAreaM2": float(part_poly.area),
                        "newAreaM2": float(part_poly.area),
                    }
                )
                continue

            old_part_area_m2 = float(part_poly.area)
            new_part_area_m2 = float(box_poly.area)
            diff_geom = part_poly.symmetric_difference(box_poly)
            diff_area_m2 = float(diff_geom.area) if diff_geom is not None and not diff_geom.is_empty else 0.0
            boxed_part_polys.append(box_poly)
            part_results.append(
                {
                    "name": part_name,
                    "reason": "boxedPart" if diff_area_m2 > 1e-6 else "alreadyBoxed",
                    "oldAreaM2": old_part_area_m2,
                    "newAreaM2": new_part_area_m2,
                    "boxLengthM": float(part_overlay.get("lengthM", 0.0) or 0.0),
                    "boxWidthM": float(part_overlay.get("widthM", 0.0) or 0.0),
                }
            )
            if diff_area_m2 > 1e-6:
                changed_any = True

        if not boxed_part_polys:
            return None, {"reason": "noParts", "oldAreaM2": float(poly.area), "partResults": part_results}

        merged = unary_union(boxed_part_polys).buffer(0)
        reconnect_reason = ""
        disconnected_count = 0
        if isinstance(merged, MultiPolygon):
            disconnected_count = len([geom for geom in merged.geoms if isinstance(geom, Polygon) and not geom.is_empty])
            merged_box_coords_xy: List[Tuple[float, float]] = []
            for boxed_poly in boxed_part_polys:
                merged_box_coords_xy.extend(
                    _dedupe_points(
                        [(float(x), float(y)) for (x, y) in list(boxed_poly.exterior.coords)[:-1]],
                        min_dist_m=1.0,
                    )
                )
            reconnect_overlay = (
                self._mid_line_overlay_geometry(merged_box_coords_xy, bearing_deg)
                if len(merged_box_coords_xy) >= 3
                else None
            )
            reconnect_box_xy = reconnect_overlay.get("boxXY") if isinstance(reconnect_overlay, dict) else None
            if isinstance(reconnect_box_xy, list) and len(reconnect_box_xy) >= 4:
                new_poly = self._largest_polygon_xy(
                    Polygon([(float(x), float(y)) for (x, y) in reconnect_box_xy]).buffer(0)
                )
                reconnect_reason = "reconnectedBox"
            else:
                new_poly = self._largest_polygon_xy(merged.convex_hull.buffer(0))
                reconnect_reason = "reconnectedHull"
        else:
            new_poly = self._largest_polygon_xy(merged)
        old_area_m2 = float(poly.area)
        if new_poly is None or new_poly.is_empty:
            return None, {"reason": "invalidMergedBox", "oldAreaM2": old_area_m2, "partResults": part_results}

        new_area_m2 = float(new_poly.area)
        info: Dict[str, Any] = {
            "reason": "splitBoxedArea",
            "oldAreaM2": old_area_m2,
            "newAreaM2": new_area_m2,
            "bearingDeg": bearing_deg,
            "partResults": part_results,
        }
        if reconnect_reason:
            info["reconnectReason"] = reconnect_reason
            info["disconnectedPartCount"] = int(disconnected_count)
        diff_geom = poly.symmetric_difference(new_poly)
        diff_area_m2 = float(diff_geom.area) if diff_geom is not None and not diff_geom.is_empty else 0.0
        info["deltaAreaM2"] = float(new_area_m2 - old_area_m2)
        info["shapeDeltaM2"] = diff_area_m2
        if (not changed_any) or diff_area_m2 <= 1e-6:
            info["reason"] = "alreadySplitBoxed"
            return None, info
        return new_poly, info

    def _turn_prediction_points_xy(
        self,
        origin_xy: Tuple[float, float],
        bearing_deg: float,
        *,
        speed_mps: float = TURN_PREVIEW_SPEED_MPS,
        horizon_s: float = TURN_PREVIEW_HORIZON_S,
        radius_m: float = TURN_PREVIEW_RADIUS_M,
    ) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        if radius_m <= 0.0:
            return origin_xy, origin_xy

        theta = math.radians(float(bearing_deg) % 360.0)
        left_center_xy = (
            float(origin_xy[0]) - (math.cos(theta) * float(radius_m)),
            float(origin_xy[1]) + (math.sin(theta) * float(radius_m)),
        )
        right_center_xy = (
            float(origin_xy[0]) + (math.cos(theta) * float(radius_m)),
            float(origin_xy[1]) - (math.sin(theta) * float(radius_m)),
        )

        def rotate_xy(vec_xy: Tuple[float, float], angle_rad: float) -> Tuple[float, float]:
            cos_a = math.cos(angle_rad)
            sin_a = math.sin(angle_rad)
            return (
                (float(vec_xy[0]) * cos_a) - (float(vec_xy[1]) * sin_a),
                (float(vec_xy[0]) * sin_a) + (float(vec_xy[1]) * cos_a),
            )

        arc_angle = (float(speed_mps) * float(horizon_s)) / float(radius_m)
        left_radius_xy = (
            float(origin_xy[0]) - float(left_center_xy[0]),
            float(origin_xy[1]) - float(left_center_xy[1]),
        )
        right_radius_xy = (
            float(origin_xy[0]) - float(right_center_xy[0]),
            float(origin_xy[1]) - float(right_center_xy[1]),
        )
        left_rotated_xy = rotate_xy(left_radius_xy, arc_angle)
        right_rotated_xy = rotate_xy(right_radius_xy, -arc_angle)
        left_point_xy = (
            float(left_center_xy[0]) + float(left_rotated_xy[0]),
            float(left_center_xy[1]) + float(left_rotated_xy[1]),
        )
        right_point_xy = (
            float(right_center_xy[0]) + float(right_rotated_xy[0]),
            float(right_center_xy[1]) + float(right_rotated_xy[1]),
        )
        return left_point_xy, right_point_xy

    def _turn_circle_centers_xy(
        self,
        origin_xy: Tuple[float, float],
        bearing_deg: float,
        *,
        radius_m: float = TURN_PREVIEW_RADIUS_M,
    ) -> Tuple[Tuple[float, float], Tuple[float, float]]:
        theta = math.radians(float(bearing_deg) % 360.0)
        left_center_xy = (
            float(origin_xy[0]) - (math.cos(theta) * float(radius_m)),
            float(origin_xy[1]) + (math.sin(theta) * float(radius_m)),
        )
        right_center_xy = (
            float(origin_xy[0]) + (math.cos(theta) * float(radius_m)),
            float(origin_xy[1]) - (math.sin(theta) * float(radius_m)),
        )
        return left_center_xy, right_center_xy

    def _distance_point_to_segment(
        self,
        point_xy: Tuple[float, float],
        seg_start_xy: Tuple[float, float],
        seg_end_xy: Tuple[float, float],
    ) -> float:
        sx = float(seg_start_xy[0])
        sy = float(seg_start_xy[1])
        ex = float(seg_end_xy[0])
        ey = float(seg_end_xy[1])
        px = float(point_xy[0])
        py = float(point_xy[1])
        dx = ex - sx
        dy = ey - sy
        denom = (dx * dx) + (dy * dy)
        if denom <= 1e-9:
            return math.hypot(px - sx, py - sy)
        t = ((px - sx) * dx + (py - sy) * dy) / denom
        t = max(0.0, min(1.0, t))
        proj_x = sx + (t * dx)
        proj_y = sy + (t * dy)
        return math.hypot(px - proj_x, py - proj_y)

    def _line_avoids_turn_circles(
        self,
        start_xy: Tuple[float, float],
        target_xy: Tuple[float, float],
        origin_xy: Tuple[float, float],
        bearing_deg: float,
        *,
        radius_m: float = TURN_PREVIEW_RADIUS_M,
    ) -> bool:
        if _distance(start_xy, target_xy) < 1e-9:
            return True
        for center_xy in self._turn_circle_centers_xy(origin_xy, bearing_deg, radius_m=radius_m):
            if self._distance_point_to_segment(center_xy, start_xy, target_xy) < (float(radius_m) - 1e-6):
                return False
        return True

    def _turn_branch_toward_target(
        self,
        origin_xy: Tuple[float, float],
        bearing_deg: float,
        target_xy: Tuple[float, float],
    ) -> Optional[str]:
        target_bearing_deg = _bearing_deg_from_xy(origin_xy, target_xy)
        if target_bearing_deg is None:
            return None
        delta_deg = ((float(target_bearing_deg) - float(bearing_deg) + 540.0) % 360.0) - 180.0
        if abs(delta_deg) <= 1e-6:
            return None
        return "R" if delta_deg > 0.0 else "L"

    def _arc_horizon_to_point_on_branch(
        self,
        origin_xy: Tuple[float, float],
        circle_center_xy: Tuple[float, float],
        point_xy: Tuple[float, float],
        *,
        branch: str,
        radius_m: float = TURN_PREVIEW_RADIUS_M,
        speed_mps: float = TURN_PREVIEW_SPEED_MPS,
    ) -> float:
        start_angle = math.atan2(
            float(origin_xy[1]) - float(circle_center_xy[1]),
            float(origin_xy[0]) - float(circle_center_xy[0]),
        )
        point_angle = math.atan2(
            float(point_xy[1]) - float(circle_center_xy[1]),
            float(point_xy[0]) - float(circle_center_xy[0]),
        )
        if str(branch).upper() == "L":
            delta_rad = (point_angle - start_angle) % (2.0 * math.pi)
        else:
            delta_rad = (start_angle - point_angle) % (2.0 * math.pi)
        return (float(delta_rad) * float(radius_m)) / float(speed_mps)

    def _refine_visibility_start_xy(
        self,
        origin_xy: Tuple[float, float],
        bearing_deg: float,
        target_xy: Tuple[float, float],
        *,
        branch: str,
        min_horizon_s: float,
        max_horizon_s: float,
        radius_m: float = TURN_PREVIEW_RADIUS_M,
    ) -> Optional[Dict[str, Any]]:
        branch_key = str(branch).upper()
        left_center_xy, right_center_xy = self._turn_circle_centers_xy(origin_xy, bearing_deg, radius_m=radius_m)
        circle_center_xy = left_center_xy if branch_key == "L" else right_center_xy

        dx = float(target_xy[0]) - float(circle_center_xy[0])
        dy = float(target_xy[1]) - float(circle_center_xy[1])
        dist_to_target = math.hypot(dx, dy)
        if dist_to_target <= float(radius_m) + 1e-6:
            return None

        alpha = math.atan2(dy, dx)
        beta = math.acos(float(radius_m) / float(dist_to_target))
        tangent_angles = [alpha + beta, alpha - beta]

        best: Optional[Dict[str, Any]] = None
        for tangent_angle in tangent_angles:
            tangent_xy = (
                float(circle_center_xy[0]) + (float(radius_m) * math.cos(tangent_angle)),
                float(circle_center_xy[1]) + (float(radius_m) * math.sin(tangent_angle)),
            )
            horizon_s = self._arc_horizon_to_point_on_branch(
                origin_xy,
                circle_center_xy,
                tangent_xy,
                branch=branch_key,
                radius_m=radius_m,
            )
            if horizon_s < float(min_horizon_s) - 1e-6:
                continue
            if horizon_s > float(max_horizon_s) + 1e-6:
                continue
            if not self._line_avoids_turn_circles(tangent_xy, target_xy, origin_xy, bearing_deg, radius_m=radius_m):
                continue
            candidate = {
                "startXY": tangent_xy,
                "horizonSec": float(horizon_s),
                "branch": branch_key,
            }
            if best is None or float(candidate["horizonSec"]) < float(best["horizonSec"]):
                best = candidate
        return best

    def _uav_prediction_points_by_id(self) -> Dict[int, Tuple[Tuple[float, float], Tuple[float, float]]]:
        out: Dict[int, Tuple[Tuple[float, float], Tuple[float, float]]] = {}
        for idx, aid in enumerate(self.state.uav_ids):
            if idx >= len(self.state.uav_positions_xy):
                continue
            if idx >= len(self.state.uav_heading_deg):
                continue
            heading = self.state.uav_heading_deg[idx]
            if heading is None:
                continue
            out[int(aid)] = self._turn_prediction_points_xy(
                self.state.uav_positions_xy[idx],
                float(heading),
            )
        return out

    def _piece_assignment_target_xy(self, piece: SplitPiece) -> Optional[Tuple[float, float]]:
        data = piece.data if isinstance(piece.data, dict) else {}
        for key in ("coordinateList", "Centerline", "rawCoordinateList"):
            coords = coords_to_xy(data.get(key, []))
            if len(coords) >= 3:
                poly = Polygon(coords)
                if not poly.is_empty:
                    center = poly.centroid
                    return float(center.x), float(center.y)
            center_xy = centroid_xy(coords)
            if center_xy is not None:
                return center_xy
        return None

    def _overlay_st_bounds(
        self,
        overlay: Dict[str, Any],
    ) -> Optional[Tuple[float, float, float, float, float, float, float, float, float]]:
        box_xy = overlay.get("boxXY")
        if not isinstance(box_xy, list) or len(box_xy) < 4:
            return None
        bearing_deg = float(overlay.get("bearingDeg", 0.0) or 0.0)
        ux, uy, vx, vy = self._mid_line_axis_vectors(bearing_deg)
        pts: List[Tuple[float, float]] = []
        for point_xy in box_xy:
            if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2:
                pts.append((float(point_xy[0]), float(point_xy[1])))
        if len(pts) < 4:
            return None
        s_vals = [(float(x) * ux) + (float(y) * uy) for (x, y) in pts]
        t_vals = [(float(x) * vx) + (float(y) * vy) for (x, y) in pts]
        if not s_vals or not t_vals:
            return None
        return (
            float(bearing_deg),
            float(ux),
            float(uy),
            float(vx),
            float(vy),
            float(min(s_vals)),
            float(max(s_vals)),
            float(min(t_vals)),
            float(max(t_vals)),
        )

    def _from_st_xy(
        self,
        s_val: float,
        t_val: float,
        ux: float,
        uy: float,
        vx: float,
        vy: float,
    ) -> Tuple[float, float]:
        return (
            (float(s_val) * float(ux)) + (float(t_val) * float(vx)),
            (float(s_val) * float(uy)) + (float(t_val) * float(vy)),
        )

    def _make_path_face_points(
        self,
        overlay: Dict[str, Any],
        origin_xy: Tuple[float, float],
    ) -> Optional[Dict[str, Tuple[float, float]]]:
        bounds = self._overlay_st_bounds(overlay)
        if bounds is None:
            return None
        _bearing_deg, ux, uy, vx, vy, min_s, max_s, min_t, max_t = bounds
        mid_t = 0.5 * (float(min_t) + float(max_t))
        min_face_xy = self._from_st_xy(min_s, mid_t, ux, uy, vx, vy)
        max_face_xy = self._from_st_xy(max_s, mid_t, ux, uy, vx, vy)
        min_face_line_xy = [
            self._from_st_xy(min_s, min_t, ux, uy, vx, vy),
            self._from_st_xy(min_s, max_t, ux, uy, vx, vy),
        ]
        max_face_line_xy = [
            self._from_st_xy(max_s, min_t, ux, uy, vx, vy),
            self._from_st_xy(max_s, max_t, ux, uy, vx, vy),
        ]
        far_is_min = _distance(origin_xy, min_face_xy) >= _distance(origin_xy, max_face_xy)
        target_face_xy = min_face_xy if far_is_min else max_face_xy
        target_face_line_xy = min_face_line_xy if far_is_min else max_face_line_xy
        return {
            "targetFaceXY": target_face_xy,
            "targetFaceLineXY": target_face_line_xy,
            "nearFaceXY": max_face_xy if far_is_min else min_face_xy,
        }

    def _bearing_centerline_target_points(
        self,
        coords_xy: Sequence[Tuple[float, float]],
        bearing_deg: float,
        origin_xy: Tuple[float, float],
    ) -> Optional[Dict[str, Any]]:
        if len(coords_xy) < 3:
            return None
        poly = Polygon(coords_xy)
        if not poly.is_valid:
            poly = poly.buffer(0)
        if poly.is_empty:
            return None
        if isinstance(poly, MultiPolygon):
            geoms = [geom for geom in poly.geoms if isinstance(geom, Polygon) and not geom.is_empty]
            if not geoms:
                return None
            poly = max(geoms, key=lambda geom: float(geom.area))

        ux, uy, vx, vy = self._mid_line_axis_vectors(bearing_deg)
        pts = [(float(x), float(y)) for (x, y) in coords_xy]
        s_vals = [(float(x) * ux) + (float(y) * uy) for (x, y) in pts]
        t_vals = [(float(x) * vx) + (float(y) * vy) for (x, y) in pts]
        if not s_vals or not t_vals:
            return None

        min_s = float(min(s_vals))
        max_s = float(max(s_vals))
        min_t = float(min(t_vals))
        max_t = float(max(t_vals))
        mid_s = 0.5 * (min_s + max_s)
        mid_t = 0.5 * (min_t + max_t)

        center_point = poly.centroid
        try:
            if center_point.is_empty or not poly.buffer(1e-6).contains(center_point):
                center_point = poly.representative_point()
        except Exception:
            center_point = poly.representative_point()

        pad_m = max(80.0, float(max_s - min_s) * 0.25)
        guide = LineString(
            [
                self._from_st_xy(min_s - pad_m, mid_t, ux, uy, vx, vy),
                self._from_st_xy(max_s + pad_m, mid_t, ux, uy, vx, vy),
            ]
        )
        guide_center_xy = self._from_st_xy(mid_s, mid_t, ux, uy, vx, vy)
        guide_center_point = Point(float(guide_center_xy[0]), float(guide_center_xy[1]))
        center_segments = self._linestring_segments_xy(poly.intersection(guide))
        if not center_segments:
            return None

        def _segment_rank(seg: LineString) -> Tuple[float, float, float]:
            try:
                dist_to_center = float(seg.distance(guide_center_point))
            except Exception:
                dist_to_center = float("inf")
            try:
                mid_point = seg.interpolate(0.5, normalized=True)
                mid_dist = float(mid_point.distance(guide_center_point))
            except Exception:
                mid_dist = dist_to_center
            return (dist_to_center, mid_dist, -float(seg.length))

        centerline = min(center_segments, key=_segment_rank)
        if centerline is None or len(centerline.coords) < 2:
            return None

        start_xy = (float(centerline.coords[0][0]), float(centerline.coords[0][1]))
        end_xy = (float(centerline.coords[-1][0]), float(centerline.coords[-1][1]))
        far_is_start = _distance(origin_xy, start_xy) >= _distance(origin_xy, end_xy)
        target_xy = start_xy if far_is_start else end_xy
        near_xy = end_xy if far_is_start else start_xy
        return {
            "targetXY": target_xy,
            "nearXY": near_xy,
            "centerLineXY": [start_xy, end_xy],
            "shapeCenterXY": (float(center_point.x), float(center_point.y)),
        }

    def _circle_tangent_points_xy(
        self,
        circle_center_xy: Tuple[float, float],
        target_xy: Tuple[float, float],
        *,
        radius_m: float = TURN_PREVIEW_RADIUS_M,
    ) -> List[Tuple[float, float]]:
        dx = float(target_xy[0]) - float(circle_center_xy[0])
        dy = float(target_xy[1]) - float(circle_center_xy[1])
        dist_m = math.hypot(dx, dy)
        if dist_m <= float(radius_m) + 1e-6:
            return []

        alpha = math.atan2(dy, dx)
        beta = math.acos(float(radius_m) / float(dist_m))
        out: List[Tuple[float, float]] = []
        for tangent_angle in (alpha + beta, alpha - beta):
            out.append(
                (
                    float(circle_center_xy[0]) + (float(radius_m) * math.cos(tangent_angle)),
                    float(circle_center_xy[1]) + (float(radius_m) * math.sin(tangent_angle)),
                )
            )
        return out

    def _branch_forward_tangent(
        self,
        circle_center_xy: Tuple[float, float],
        tangent_xy: Tuple[float, float],
        target_xy: Tuple[float, float],
        *,
        branch: str,
    ) -> bool:
        rx = float(tangent_xy[0]) - float(circle_center_xy[0])
        ry = float(tangent_xy[1]) - float(circle_center_xy[1])
        if str(branch).upper() == "L":
            tangent_dir_xy = (-ry, rx)
        else:
            tangent_dir_xy = (ry, -rx)
        target_dir_xy = (
            float(target_xy[0]) - float(tangent_xy[0]),
            float(target_xy[1]) - float(tangent_xy[1]),
        )
        return ((float(tangent_dir_xy[0]) * float(target_dir_xy[0])) + (float(tangent_dir_xy[1]) * float(target_dir_xy[1]))) > 1e-6

    def _segment_interval_points_xy(
        self,
        start_xy: Tuple[float, float],
        end_xy: Tuple[float, float],
        *,
        interval_m: float,
    ) -> List[Tuple[float, float]]:
        total_m = _distance(start_xy, end_xy)
        if total_m <= float(interval_m) + 1e-6 or interval_m <= 0.0:
            return []
        out: List[Tuple[float, float]] = []
        dx = float(end_xy[0]) - float(start_xy[0])
        dy = float(end_xy[1]) - float(start_xy[1])
        step_count = int(math.floor(total_m / float(interval_m)))
        for idx in range(1, step_count + 1):
            dist_m = float(idx) * float(interval_m)
            if dist_m >= total_m - 1e-6:
                break
            ratio = dist_m / total_m
            out.append(
                (
                    float(start_xy[0]) + (dx * ratio),
                    float(start_xy[1]) + (dy * ratio),
                )
            )
        return out

    def _make_path_sweep_lines_xy(
        self,
        piece: SplitPiece,
        overlay: Dict[str, Any],
        *,
        sep_m: float,
    ) -> List[List[Tuple[float, float]]]:
        if sep_m <= 0.0:
            return []
        piece_poly = self._piece_polygon_xy(piece)
        bounds = self._overlay_st_bounds(overlay)
        if piece_poly is None or piece_poly.is_empty or bounds is None:
            return []
        return self._sweep_lines_for_polygon_xy(
            [(float(x), float(y)) for (x, y) in list(piece_poly.exterior.coords)[:-1]],
            float(overlay.get("bearingDeg", 0.0) or 0.0),
            sep_m=sep_m,
        )

    def _sweep_lines_for_polygon_xy(
        self,
        polygon_xy: Sequence[Tuple[float, float]],
        bearing_deg: float,
        *,
        sep_m: float,
    ) -> List[List[Tuple[float, float]]]:
        if sep_m <= 0.0 or len(polygon_xy) < 3:
            return []
        piece_poly = self._largest_polygon_xy(Polygon(polygon_xy).buffer(0))
        if piece_poly is None or piece_poly.is_empty:
            return []
        overlay = self._mid_line_overlay_geometry(polygon_xy, bearing_deg)
        bounds = self._overlay_st_bounds(
            {
                "bearingDeg": float(bearing_deg),
                **(overlay or {}),
            }
        )
        if bounds is None:
            return []
        _bearing_deg, ux, uy, vx, vy, min_s, max_s, min_t, max_t = bounds
        mid_t = 0.5 * (float(min_t) + float(max_t))
        pad_m = max(80.0, float(max_s - min_s) * 0.25)

        t_values: List[float] = [mid_t]
        step_idx = 1
        while True:
            added = False
            left_t = mid_t - (float(step_idx) * float(sep_m))
            right_t = mid_t + (float(step_idx) * float(sep_m))
            if left_t >= float(min_t) - 1e-6:
                t_values.append(float(left_t))
                added = True
            if right_t <= float(max_t) + 1e-6:
                t_values.append(float(right_t))
                added = True
            if not added:
                break
            step_idx += 1

        lines_xy: List[List[Tuple[float, float]]] = []
        for t_val in t_values:
            guide = LineString(
                [
                    self._from_st_xy(min_s - pad_m, t_val, ux, uy, vx, vy),
                    self._from_st_xy(max_s + pad_m, t_val, ux, uy, vx, vy),
                ]
            )
            seg = self._longest_linestring_xy(piece_poly.intersection(guide))
            if seg is None or len(seg.coords) < 2:
                continue
            lines_xy.append(
                [
                    (float(seg.coords[0][0]), float(seg.coords[0][1])),
                    (float(seg.coords[-1][0]), float(seg.coords[-1][1])),
                ]
            )
        return lines_xy

    def _make_path_row(
        self,
        piece: SplitPiece,
        overlay: Dict[str, Any],
    ) -> Optional[Dict[str, Any]]:
        aid = int(piece.assigned_uav or 0)
        if aid <= 0:
            return None
        uav_state = self._uav_state_for_aircraft(aid)
        if uav_state is None:
            return None
        origin_xy, heading_deg = uav_state
        target_face_xy = None
        cached_target_face_xy = overlay.get("targetFaceXY")
        if isinstance(cached_target_face_xy, (tuple, list)) and len(cached_target_face_xy) >= 2:
            target_face_xy = (float(cached_target_face_xy[0]), float(cached_target_face_xy[1]))
        else:
            face_points = self._make_path_face_points(overlay, origin_xy)
            if face_points is None:
                return None
            target_face_xy = face_points["targetFaceXY"]

        segment = self._find_visibility_segment(aid, origin_xy, heading_deg, target_face_xy)
        if segment is None:
            return None

        route_xy: List[Tuple[float, float]] = [origin_xy]
        marker_rows: List[Dict[str, Any]] = []
        timeline_rows: List[Dict[str, Any]] = []
        tangent_eta_sec = float(segment.get("horizonSec", 0.0) or 0.0)
        turn_points = segment.get("turnPoints")
        if isinstance(turn_points, list):
            for idx, point_xy in enumerate(turn_points, start=1):
                if not (isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2):
                    continue
                point_val = (float(point_xy[0]), float(point_xy[1]))
                route_xy.append(point_val)
                marker_rows.append(
                    {
                        "xy": point_val,
                        "label": f"{int(idx * TURN_PREVIEW_HORIZON_S)}s",
                        "kind": "turn",
                        "etaSec": float(idx * TURN_PREVIEW_HORIZON_S),
                    }
                )
                timeline_rows.append(
                    {
                        "label": f"{int(idx * TURN_PREVIEW_HORIZON_S)}s",
                        "kind": "turn",
                        "etaSec": float(idx * TURN_PREVIEW_HORIZON_S),
                    }
                )

        start_xy = segment.get("startXY")
        if not (isinstance(start_xy, (tuple, list)) and len(start_xy) >= 2):
            return None
        start_xy_val = (float(start_xy[0]), float(start_xy[1]))
        if _distance(route_xy[-1], start_xy_val) > 1e-6:
            route_xy.append(start_xy_val)
        if marker_rows and _distance(marker_rows[-1]["xy"], start_xy_val) <= 1e-6:
            marker_rows[-1]["label"] = "T"
            marker_rows[-1]["kind"] = "tangent"
            marker_rows[-1]["etaSec"] = tangent_eta_sec
            if timeline_rows:
                timeline_rows[-1]["label"] = "T"
                timeline_rows[-1]["kind"] = "tangent"
                timeline_rows[-1]["etaSec"] = tangent_eta_sec
        else:
            marker_rows.append({"xy": start_xy_val, "label": "T", "kind": "tangent", "etaSec": tangent_eta_sec})
            timeline_rows.append({"label": "T", "kind": "tangent", "etaSec": tangent_eta_sec})

        interval_points = self._segment_interval_points_xy(
            start_xy_val,
            target_face_xy,
            interval_m=float(MAKE_PATH_INTERVAL_M),
        )
        for idx, point_xy in enumerate(interval_points, start=1):
            route_xy.append(point_xy)
            eta_sec = tangent_eta_sec + (_distance(start_xy_val, point_xy) / float(TURN_PREVIEW_SPEED_MPS))
            marker_rows.append(
                {
                    "xy": point_xy,
                    "label": f"{int(idx)}k",
                    "kind": "interval",
                    "etaSec": eta_sec,
                }
            )
            timeline_rows.append({"label": f"{int(idx)}k", "kind": "interval", "etaSec": eta_sec})

        if _distance(route_xy[-1], target_face_xy) > 1e-6:
            route_xy.append(target_face_xy)
        end_eta_sec = tangent_eta_sec + (_distance(start_xy_val, target_face_xy) / float(TURN_PREVIEW_SPEED_MPS))
        marker_rows.append({"xy": target_face_xy, "label": "END", "kind": "last_face", "etaSec": end_eta_sec})
        timeline_rows.append({"label": "END", "kind": "last_face", "etaSec": end_eta_sec})

        width_ref_m = float(overlay.get("maxWidthM", 0.0) or overlay.get("widthM", 0.0) or 0.0)
        db_row = self._covering_db_row(width_ref_m)
        sep_m = float(db_row.get("sep", 0.0) or 0.0) if isinstance(db_row, dict) else 0.0
        sweep_lines_xy = self._make_path_sweep_lines_xy(piece, overlay, sep_m=sep_m)
        phase_rows: List[Dict[str, Any]] = []
        if tangent_eta_sec > 1e-6:
            phase_rows.append({"label": "Turn", "kind": "turn", "startSec": 0.0, "endSec": tangent_eta_sec})
        if end_eta_sec > tangent_eta_sec + 1e-6:
            phase_rows.append({"label": "Ingress", "kind": "ingress", "startSec": tangent_eta_sec, "endSec": end_eta_sec})

        return {
            "source": "make_path",
            "aircraftID": int(aid),
            "pieceIndex": int(piece.piece_index or 0),
            "routeXY": route_xy,
            "markerRows": marker_rows,
            "timelineRows": timeline_rows,
            "phaseRows": phase_rows,
            "estimatedTotalSec": float(end_eta_sec),
            "sweepLineListXY": sweep_lines_xy,
            "targetFaceXY": target_face_xy,
            "horizonSec": float(tangent_eta_sec),
            "branch": str(segment.get("branch", "") or ""),
            "dbWidthM": float(db_row.get("width", 0.0) or 0.0) if isinstance(db_row, dict) else 0.0,
            "dbSepM": float(sep_m),
        }

    def _build_turn_prefix_rows(
        self,
        origin_xy: Tuple[float, float],
        heading_deg: float,
        *,
        branch: str,
        horizon_s: float,
        tangent_point_xy: Tuple[float, float],
    ) -> Tuple[List[Tuple[float, float]], List[Dict[str, Any]], List[Dict[str, Any]]]:
        route_xy: List[Tuple[float, float]] = [origin_xy]
        marker_rows: List[Dict[str, Any]] = []
        timeline_rows: List[Dict[str, Any]] = []

        if horizon_s > 1e-6 and branch in {"L", "R"}:
            sample_step_s = min(float(TURN_PREVIEW_HORIZON_S), float(ASSIGNMENT_PATH_ARC_STEP_S))
            sample_count = int(math.floor(horizon_s / float(sample_step_s)))
            for idx in range(1, sample_count + 1):
                sample_horizon_s = float(idx) * float(sample_step_s)
                if sample_horizon_s >= horizon_s - 1e-6:
                    break
                left_xy, right_xy = self._turn_prediction_points_xy(
                    origin_xy,
                    heading_deg,
                    horizon_s=sample_horizon_s,
                )
                point_xy = left_xy if branch == "L" else right_xy
                if _distance(route_xy[-1], point_xy) > 1e-6:
                    route_xy.append(point_xy)
                marker_multiple = sample_horizon_s / float(TURN_PREVIEW_HORIZON_S)
                if abs(marker_multiple - round(marker_multiple)) <= 1e-6:
                    marker_rows.append(
                        {
                            "xy": point_xy,
                            "label": f"{int(sample_horizon_s)}s",
                            "kind": "turn",
                            "etaSec": sample_horizon_s,
                        }
                    )
                    timeline_rows.append({"label": f"{int(sample_horizon_s)}s", "kind": "turn", "etaSec": sample_horizon_s})

        if _distance(route_xy[-1], tangent_point_xy) > 1e-6:
            route_xy.append(tangent_point_xy)
        marker_rows.append({"xy": tangent_point_xy, "label": "T", "kind": "tangent", "etaSec": horizon_s})
        timeline_rows.append({"label": "T", "kind": "tangent", "etaSec": horizon_s})
        return route_xy, marker_rows, timeline_rows

    def _build_assignment_path_1_row(
        self,
        candidate: Dict[str, Any],
    ) -> Optional[Dict[str, Any]]:
        aid = int(candidate.get("aircraftID", 0) or 0)
        if aid <= 0:
            return None
        uav_state = self._uav_state_for_aircraft(aid)
        if uav_state is None:
            return None
        origin_xy, heading_deg = uav_state
        tangent_xy = candidate.get("tangentXY")
        target_xy = candidate.get("targetXY")
        if not (
            isinstance(tangent_xy, (tuple, list))
            and len(tangent_xy) >= 2
            and isinstance(target_xy, (tuple, list))
            and len(target_xy) >= 2
        ):
            return None

        tangent_point_xy = (float(tangent_xy[0]), float(tangent_xy[1]))
        target_point_xy = (float(target_xy[0]), float(target_xy[1]))
        branch = str(candidate.get("branch", "") or "")
        horizon_s = float(candidate.get("horizonSec", 0.0) or 0.0)

        route_xy, marker_rows, timeline_rows = self._build_turn_prefix_rows(
            origin_xy,
            heading_deg,
            branch=branch,
            horizon_s=horizon_s,
            tangent_point_xy=tangent_point_xy,
        )

        if _distance(route_xy[-1], target_point_xy) > 1e-6:
            route_xy.append(target_point_xy)
        target_eta_sec = horizon_s + (_distance(tangent_point_xy, target_point_xy) / float(TURN_PREVIEW_SPEED_MPS))
        marker_rows.append(
            {
                "xy": target_point_xy,
                "label": str(candidate.get("targetLabel", "") or "END1"),
                "kind": "last_face",
                "etaSec": target_eta_sec,
            }
        )
        timeline_rows.append(
            {
                "label": str(candidate.get("targetLabel", "") or "END1"),
                "kind": "last_face",
                "etaSec": target_eta_sec,
            }
        )
        phase_rows: List[Dict[str, Any]] = []
        if horizon_s > 1e-6:
            phase_rows.append({"label": "Turn", "kind": "turn", "startSec": 0.0, "endSec": horizon_s})
        if target_eta_sec > horizon_s + 1e-6:
            phase_rows.append({"label": "Ingress", "kind": "ingress", "startSec": horizon_s, "endSec": target_eta_sec})

        center_line_xy = candidate.get("centerLineXY")
        part_polygon_xy = candidate.get("partPolygonXY")
        return {
            "source": "assignment_path_1",
            "aircraftID": int(aid),
            "pieceIndex": int(candidate.get("pieceIndex", 0) or 0),
            "targetLabel": str(candidate.get("targetLabel", "") or ""),
            "routeXY": route_xy,
            "markerRows": marker_rows,
            "timelineRows": timeline_rows,
            "phaseRows": phase_rows,
            "estimatedTotalSec": float(target_eta_sec),
            "targetFaceXY": target_point_xy,
            "targetXY": target_point_xy,
            "tangentXY": tangent_point_xy,
            "horizonSec": float(horizon_s),
            "branch": branch,
            "entryAngleDeg": float(candidate.get("entryAngleDeg", 0.0) or 0.0),
            "segmentLenM": float(candidate.get("segmentLenM", 0.0) or 0.0),
            "selectionScore": float(candidate.get("selectionScore", 0.0) or 0.0),
            "centerLineXY": [
                (float(point_xy[0]), float(point_xy[1]))
                for point_xy in center_line_xy
                if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2
            ] if isinstance(center_line_xy, list) else [],
            "partPolygonXY": [
                (float(point_xy[0]), float(point_xy[1]))
                for point_xy in part_polygon_xy
                if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2
            ] if isinstance(part_polygon_xy, list) else [],
            "partWidthM": float(candidate.get("partWidthM", 0.0) or 0.0),
            "midLineLengthM": float(candidate.get("midLineLengthM", 0.0) or 0.0),
            "bearingDeg": float(candidate.get("bearingDeg", 0.0) or 0.0),
        }

    def _build_make_waypoint_row(
        self,
        base_row: Dict[str, Any],
    ) -> Optional[Dict[str, Any]]:
        aid = int(base_row.get("aircraftID", 0) or 0)
        if aid <= 0:
            return None
        uav_state = self._uav_state_for_aircraft(aid)
        if uav_state is None:
            return None
        origin_xy, heading_deg = uav_state

        tangent_xy = base_row.get("tangentXY")
        target_xy = base_row.get("targetXY")
        part_polygon_xy_raw = base_row.get("partPolygonXY")
        if not (
            isinstance(tangent_xy, (tuple, list))
            and len(tangent_xy) >= 2
            and isinstance(target_xy, (tuple, list))
            and len(target_xy) >= 2
            and isinstance(part_polygon_xy_raw, list)
            and len(part_polygon_xy_raw) >= 3
        ):
            return None

        tangent_point_xy = (float(tangent_xy[0]), float(tangent_xy[1]))
        target_point_xy = (float(target_xy[0]), float(target_xy[1]))
        part_polygon_xy = [
            (float(point_xy[0]), float(point_xy[1]))
            for point_xy in part_polygon_xy_raw
            if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2
        ]
        if len(part_polygon_xy) < 3:
            return None

        branch = str(base_row.get("branch", "") or "")
        horizon_s = float(base_row.get("horizonSec", 0.0) or 0.0)
        route_xy, marker_rows, timeline_rows = self._build_turn_prefix_rows(
            origin_xy,
            heading_deg,
            branch=branch,
            horizon_s=horizon_s,
            tangent_point_xy=tangent_point_xy,
        )

        width_ref_m = float(base_row.get("partWidthM", 0.0) or 0.0)
        db_row = self._largest_sep_covering_db_row(width_ref_m)
        sep_m = float(db_row.get("sep", 0.0) or 0.0) if isinstance(db_row, dict) else 0.0

        mid_line_length_m = float(base_row.get("midLineLengthM", 0.0) or 0.0)
        if mid_line_length_m <= 1e-6:
            center_line_xy = base_row.get("centerLineXY")
            if isinstance(center_line_xy, list) and len(center_line_xy) >= 2:
                mid_line_length_m = _distance(center_line_xy[0], center_line_xy[-1])
        if mid_line_length_m <= 1e-6:
            mid_line_length_m = _distance(tangent_point_xy, target_point_xy)

        bearing_deg = float(base_row.get("bearingDeg", 0.0) or 0.0)
        sweep_lines_xy = self._sweep_lines_for_polygon_xy(
            part_polygon_xy,
            bearing_deg,
            sep_m=sep_m,
        )
        first_segment_row = self._first_sweep_segment_point_xy(
            sweep_lines_xy,
            tangent_point_xy,
            target_point_xy,
        )
        first_segment_point_xy = first_segment_row[0] if first_segment_row is not None else None
        first_segment_distance_m = float(first_segment_row[1]) if first_segment_row is not None else 0.0
        if first_segment_point_xy is None:
            first_segment_point_xy = self._first_polygon_entry_point_xy(
                part_polygon_xy,
                tangent_point_xy,
                target_point_xy,
            )
            if first_segment_point_xy is not None:
                first_segment_distance_m = _distance(tangent_point_xy, first_segment_point_xy)
        if first_segment_point_xy is None:
            first_segment_point_xy = target_point_xy
            first_segment_distance_m = _distance(tangent_point_xy, first_segment_point_xy)

        start_waypoint_xy = tangent_point_xy
        start_label = "T"
        if sep_m > 1e-6 and first_segment_distance_m > sep_m + 1e-6:
            start_waypoint_xy = self._project_along_xy(first_segment_point_xy, tangent_point_xy, sep_m)
            start_label = "T'"

        end_waypoint_xy = self._project_along_xy(
            start_waypoint_xy,
            target_point_xy,
            mid_line_length_m,
        )

        start_eta_sec = horizon_s + (_distance(tangent_point_xy, start_waypoint_xy) / float(TURN_PREVIEW_SPEED_MPS))
        entry_eta_sec = horizon_s + (_distance(tangent_point_xy, first_segment_point_xy) / float(TURN_PREVIEW_SPEED_MPS))
        end_eta_sec = horizon_s + (_distance(tangent_point_xy, end_waypoint_xy) / float(TURN_PREVIEW_SPEED_MPS))

        if _distance(route_xy[-1], start_waypoint_xy) > 1e-6:
            route_xy.append(start_waypoint_xy)
        if (
            _distance(tangent_point_xy, end_waypoint_xy) > first_segment_distance_m + 1e-6
            and
            _distance(first_segment_point_xy, start_waypoint_xy) > 1e-6
            and _distance(first_segment_point_xy, end_waypoint_xy) > 1e-6
        ):
            route_xy.append(first_segment_point_xy)
            marker_rows.append({"xy": first_segment_point_xy, "label": "S1", "kind": "entry", "etaSec": entry_eta_sec})
            timeline_rows.append({"label": "S1", "kind": "entry", "etaSec": entry_eta_sec})
        if _distance(route_xy[-1], end_waypoint_xy) > 1e-6:
            route_xy.append(end_waypoint_xy)

        if start_label == "T'":
            marker_rows.append({"xy": start_waypoint_xy, "label": "T'", "kind": "waypoint_start", "etaSec": start_eta_sec})
            timeline_rows.append({"label": "T'", "kind": "waypoint_start", "etaSec": start_eta_sec})
        marker_rows.append({"xy": end_waypoint_xy, "label": "WP", "kind": "waypoint_end", "etaSec": end_eta_sec})
        timeline_rows.append({"label": "WP", "kind": "waypoint_end", "etaSec": end_eta_sec})

        phase_rows: List[Dict[str, Any]] = []
        if horizon_s > 1e-6:
            phase_rows.append({"label": "Turn", "kind": "turn", "startSec": 0.0, "endSec": horizon_s})
        if start_eta_sec > horizon_s + 1e-6:
            phase_rows.append({"label": "Setup", "kind": "setup", "startSec": horizon_s, "endSec": start_eta_sec})
        if end_eta_sec > start_eta_sec + 1e-6:
            phase_rows.append({"label": "Waypoint", "kind": "waypoint", "startSec": start_eta_sec, "endSec": end_eta_sec})

        waypoint_list = [
            {"label": "WP_START", "coordinate": meters_to_coord(start_waypoint_xy[0], start_waypoint_xy[1])},
            {"label": "WP_END", "coordinate": meters_to_coord(end_waypoint_xy[0], end_waypoint_xy[1])},
        ]

        return {
            "source": "make_waypoint",
            "aircraftID": int(aid),
            "pieceIndex": int(base_row.get("pieceIndex", 0) or 0),
            "targetLabel": str(base_row.get("targetLabel", "") or ""),
            "routeXY": route_xy,
            "markerRows": marker_rows,
            "timelineRows": timeline_rows,
            "phaseRows": phase_rows,
            "estimatedTotalSec": float(end_eta_sec),
            "waypointList": waypoint_list,
            "tangentXY": tangent_point_xy,
            "targetXY": target_point_xy,
            "entryPointXY": first_segment_point_xy,
            "waypointStartXY": start_waypoint_xy,
            "waypointEndXY": end_waypoint_xy,
            "targetFaceXY": end_waypoint_xy,
            "horizonSec": float(horizon_s),
            "branch": branch,
            "entryAngleDeg": float(base_row.get("entryAngleDeg", 0.0) or 0.0),
            "selectionScore": float(base_row.get("selectionScore", 0.0) or 0.0),
            "dbWidthM": float(db_row.get("width", 0.0) or 0.0) if isinstance(db_row, dict) else 0.0,
            "dbSepM": float(sep_m),
            "shapeWidthM": float(width_ref_m),
            "midLineLengthM": float(mid_line_length_m),
            "entryDistanceM": float(first_segment_distance_m),
            "startLabel": start_label,
            "sweepLineListXY": sweep_lines_xy,
            "centerLineXY": [
                (float(point_xy[0]), float(point_xy[1]))
                for point_xy in base_row.get("centerLineXY", [])
                if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2
            ] if isinstance(base_row.get("centerLineXY"), list) else [],
            "partPolygonXY": part_polygon_xy,
            "bearingDeg": float(bearing_deg),
        }

    def _assign_split_result_by_prediction_distance(self, split_result: SplitRunResult) -> Dict[str, Any]:
        prediction_map = self._uav_prediction_points_by_id()
        assigned_pieces = 0
        uav_summary: Dict[int, int] = {}
        piece_lines: List[str] = []

        for piece in split_result.pieces:
            piece.assigned_uav = None

        if not prediction_map:
            return {
                "pieceCount": int(len(split_result.pieces)),
                "assignedPieces": 0,
                "uavSummary": {},
                "pieceLines": piece_lines,
            }

        target_items: List[Tuple[SplitPiece, Tuple[float, float]]] = []
        for piece in split_result.pieces:
            target_xy = self._piece_assignment_target_xy(piece)
            if target_xy is not None:
                target_items.append((piece, target_xy))

        usable_uavs = sorted(int(aid) for aid in prediction_map.keys())
        if not target_items or not usable_uavs:
            return {
                "pieceCount": int(len(split_result.pieces)),
                "assignedPieces": 0,
                "uavSummary": {},
                "pieceLines": piece_lines,
            }

        def _best_candidate_for_target(
            target_xy: Tuple[float, float],
            aid: int,
        ) -> Tuple[float, str]:
            left_xy, right_xy = prediction_map[int(aid)]
            uav_state = self._uav_state_for_aircraft(int(aid))
            preferred_branch: Optional[str] = None
            if uav_state is not None:
                preferred_branch = self._turn_branch_toward_target(
                    uav_state[0],
                    float(uav_state[1]),
                    target_xy,
                )
            left_dist = _distance(target_xy, left_xy)
            right_dist = _distance(target_xy, right_xy)
            if preferred_branch == "L":
                return left_dist, "L"
            if preferred_branch == "R":
                return right_dist, "R"
            if left_dist <= right_dist:
                return left_dist, "L"
            return right_dist, "R"

        assigned_records: List[Tuple[SplitPiece, int, str, float]] = []

        if len(target_items) <= len(usable_uavs):
            best_total = float("inf")
            best_records: List[Tuple[SplitPiece, int, str, float]] = []
            for aid_perm in itertools.permutations(usable_uavs, len(target_items)):
                total = 0.0
                candidate_records: List[Tuple[SplitPiece, int, str, float]] = []
                for (piece, target_xy), aid in zip(target_items, aid_perm):
                    dist_m, branch = _best_candidate_for_target(target_xy, aid)
                    total += float(dist_m)
                    candidate_records.append((piece, int(aid), branch, float(dist_m)))
                if total < best_total:
                    best_total = total
                    best_records = candidate_records
            assigned_records = best_records
        else:
            used_uavs: set[int] = set()
            for piece, target_xy in target_items:
                candidate_uavs = [aid for aid in usable_uavs if aid not in used_uavs]
                if not candidate_uavs:
                    candidate_uavs = list(usable_uavs)

                best_aid = 0
                best_branch = ""
                best_dist = float("inf")
                for aid in candidate_uavs:
                    dist_m, branch = _best_candidate_for_target(target_xy, aid)
                    if dist_m < best_dist:
                        best_dist = float(dist_m)
                        best_aid = int(aid)
                        best_branch = branch
                if best_aid <= 0:
                    continue
                assigned_records.append((piece, best_aid, best_branch, best_dist))
                used_uavs.add(best_aid)

        for piece, best_aid, best_branch, best_dist in assigned_records:
            piece.assigned_uav = int(best_aid)
            assigned_pieces += 1
            uav_summary[best_aid] = int(uav_summary.get(best_aid, 0)) + 1
            piece_lines.append(f"  P{piece.piece_index}: UAV{best_aid} ({best_branch}, {best_dist:.1f}m)")

        return {
            "pieceCount": int(len(split_result.pieces)),
            "assignedPieces": int(assigned_pieces),
            "uavSummary": {int(k): int(v) for k, v in sorted(uav_summary.items())},
            "pieceLines": piece_lines,
        }

    def _largest_polygon_xy(self, geom: Any) -> Optional[Polygon]:
        if geom is None:
            return None
        candidate = geom
        if getattr(candidate, "is_empty", True):
            return None
        if isinstance(candidate, Polygon):
            poly = candidate
        else:
            gt = str(getattr(candidate, "geom_type", ""))
            if gt not in {"MultiPolygon", "GeometryCollection"}:
                return None
            polys = [g for g in getattr(candidate, "geoms", []) if isinstance(g, Polygon) and not g.is_empty]
            if not polys:
                return None
            poly = max(polys, key=lambda g: g.area)
        if not poly.is_valid:
            fixed = poly.buffer(0)
            if isinstance(fixed, Polygon) and not fixed.is_empty:
                poly = fixed
            else:
                polys = [g for g in getattr(fixed, "geoms", []) if isinstance(g, Polygon) and not g.is_empty]
                if not polys:
                    return None
                poly = max(polys, key=lambda g: g.area)
        return poly if not poly.is_empty and poly.area > 1e-6 else None

    def _mission_polygon_xy(self) -> Optional[Polygon]:
        if self.state.mission_kind != MISSION_AREA or len(self.state.mission_points_xy) < 3:
            return None
        return self._largest_polygon_xy(Polygon(self.state.mission_points_xy))

    def _stage2_grid_size_m(self, mission_poly: Polygon) -> float:
        minx, miny, maxx, maxy = mission_poly.bounds
        width_m = max(0.0, float(maxx - minx))
        height_m = max(0.0, float(maxy - miny))
        if width_m <= float(STAGE2_GRID_BOUND_SMALL_M) and height_m <= float(STAGE2_GRID_BOUND_SMALL_M):
            return float(STAGE2_GRID_SIZE_SMALL_M)
        if width_m <= float(STAGE2_GRID_BOUND_MEDIUM_M) and height_m <= float(STAGE2_GRID_BOUND_MEDIUM_M):
            return float(STAGE2_GRID_SIZE_MEDIUM_M)
        return float(STAGE2_GRID_SIZE_LARGE_M)

    def _piece_polygon_xy(self, piece: SplitPiece) -> Optional[Polygon]:
        data = piece.data if isinstance(piece.data, dict) else {}
        coords = coords_to_xy(data.get("coordinateList", []))
        if len(coords) < 3:
            return None
        return self._largest_polygon_xy(Polygon(coords))

    def _visibility_segment_by_aircraft_id(self, aircraft_id: int) -> Optional[Dict[str, Any]]:
        aid = int(aircraft_id)
        for row in self.state.visibility_segments:
            if not isinstance(row, dict):
                continue
            if int(row.get("aircraftID", 0) or 0) == aid:
                return row
        return None

    def _stage2_ratio_map(self, active_uav_ids: Sequence[int]) -> Dict[int, float]:
        out: Dict[int, float] = {}
        for aid in active_uav_ids:
            spin = self.stage2_ratio_spins.get(int(aid))
            value = float(spin.value()) if spin is not None else 0.0
            out[int(aid)] = max(0.0, value)
        return out

    def _stage2_piece_segment(self, aircraft_id: int, piece: SplitPiece) -> Optional[Dict[str, Any]]:
        cached = self._visibility_segment_by_aircraft_id(int(aircraft_id))
        if cached is not None:
            return cached
        uav_state = self._uav_state_for_aircraft(int(aircraft_id))
        target_xy = self._piece_assignment_target_xy(piece)
        if uav_state is None or target_xy is None:
            return None
        return self._find_visibility_segment(
            int(aircraft_id),
            uav_state[0],
            float(uav_state[1]),
            target_xy,
        )

    def _stage2_seed_xy(self, aircraft_id: int, piece: SplitPiece) -> Tuple[float, float]:
        piece_poly = self._piece_polygon_xy(piece)
        centroid_xy_val = self._piece_assignment_target_xy(piece)
        if centroid_xy_val is None and piece_poly is not None:
            centroid_xy_val = (float(piece_poly.centroid.x), float(piece_poly.centroid.y))
        if centroid_xy_val is None:
            uav_state = self._uav_state_for_aircraft(int(aircraft_id))
            if uav_state is not None:
                return uav_state[0]
            return 0.0, 0.0

        anchor_xy = centroid_xy_val
        seg = self._stage2_piece_segment(int(aircraft_id), piece)
        if isinstance(seg, dict):
            start_xy = seg.get("startXY")
            if isinstance(start_xy, (tuple, list)) and len(start_xy) >= 2:
                anchor_xy = (float(start_xy[0]), float(start_xy[1]))
        else:
            uav_state = self._uav_state_for_aircraft(int(aircraft_id))
            if uav_state is not None:
                anchor_xy = uav_state[0]

        blend = float(STAGE2_ANCHOR_BLEND)
        return (
            ((1.0 - blend) * float(centroid_xy_val[0])) + (blend * float(anchor_xy[0])),
            ((1.0 - blend) * float(centroid_xy_val[1])) + (blend * float(anchor_xy[1])),
        )

    def _stage2_entry_time_sec(self, aircraft_id: int, piece: SplitPiece) -> float:
        target_xy = self._piece_assignment_target_xy(piece)
        if target_xy is None:
            return 0.0

        seg = self._stage2_piece_segment(int(aircraft_id), piece)
        if isinstance(seg, dict):
            start_xy = seg.get("startXY")
            if isinstance(start_xy, (tuple, list)) and len(start_xy) >= 2:
                start = (float(start_xy[0]), float(start_xy[1]))
                horizon_s = float(seg.get("horizonSec", 0.0) or 0.0)
                return horizon_s + (_distance(start, target_xy) / float(TURN_PREVIEW_SPEED_MPS))

        uav_state = self._uav_state_for_aircraft(int(aircraft_id))
        if uav_state is None:
            return 0.0
        return _distance(uav_state[0], target_xy) / float(TURN_PREVIEW_SPEED_MPS)

    def _longest_linestring_xy(self, geom: Any) -> Optional[LineString]:
        if geom is None or getattr(geom, "is_empty", True):
            return None
        if isinstance(geom, LineString):
            return geom if float(geom.length) > 1e-6 else None
        gt = str(getattr(geom, "geom_type", ""))
        if gt not in {"MultiLineString", "GeometryCollection"}:
            return None
        lines = [g for g in getattr(geom, "geoms", []) if isinstance(g, LineString) and float(g.length) > 1e-6]
        if not lines:
            return None
        return max(lines, key=lambda g: float(g.length))

    def _linestring_segments_xy(self, geom: Any) -> List[LineString]:
        if geom is None or getattr(geom, "is_empty", True):
            return []
        if isinstance(geom, LineString):
            return [geom] if float(geom.length) > 1e-6 else []
        gt = str(getattr(geom, "geom_type", ""))
        if gt not in {"MultiLineString", "GeometryCollection"}:
            return []
        return [g for g in getattr(geom, "geoms", []) if isinstance(g, LineString) and float(g.length) > 1e-6]

    def _geometry_point_candidates_xy(self, geom: Any) -> List[Tuple[float, float]]:
        if geom is None or getattr(geom, "is_empty", True):
            return []
        if isinstance(geom, Point):
            return [(float(geom.x), float(geom.y))]
        if isinstance(geom, LineString):
            coords = list(geom.coords)
            if not coords:
                return []
            if len(coords) == 1:
                return [(float(coords[0][0]), float(coords[0][1]))]
            return [
                (float(coords[0][0]), float(coords[0][1])),
                (float(coords[-1][0]), float(coords[-1][1])),
            ]
        gt = str(getattr(geom, "geom_type", ""))
        if gt not in {"MultiPoint", "MultiLineString", "GeometryCollection"}:
            return []
        out: List[Tuple[float, float]] = []
        for item in getattr(geom, "geoms", []):
            out.extend(self._geometry_point_candidates_xy(item))
        return _dedupe_points(out, min_dist_m=1.0)

    def _first_polygon_entry_point_xy(
        self,
        polygon_xy: Sequence[Tuple[float, float]],
        start_xy: Tuple[float, float],
        toward_xy: Tuple[float, float],
    ) -> Optional[Tuple[float, float]]:
        if len(polygon_xy) < 3 or _distance(start_xy, toward_xy) <= 1e-6:
            return None

        poly = Polygon(polygon_xy)
        if not poly.is_valid:
            poly = poly.buffer(0)
        if poly.is_empty:
            return None
        if isinstance(poly, MultiPolygon):
            geoms = [geom for geom in poly.geoms if isinstance(geom, Polygon) and not geom.is_empty]
            if not geoms:
                return None
            poly = max(geoms, key=lambda geom: float(geom.area))

        dx = float(toward_xy[0]) - float(start_xy[0])
        dy = float(toward_xy[1]) - float(start_xy[1])
        total_len_m = math.hypot(dx, dy)
        if total_len_m <= 1e-6:
            return None
        ux = dx / total_len_m
        uy = dy / total_len_m
        pad_m = max(80.0, total_len_m * 0.25)
        probe_end_xy = (
            float(toward_xy[0]) + (ux * pad_m),
            float(toward_xy[1]) + (uy * pad_m),
        )
        probe_line = LineString([start_xy, probe_end_xy])
        candidates = self._geometry_point_candidates_xy(poly.boundary.intersection(probe_line))
        if not candidates:
            return None

        best_xy: Optional[Tuple[float, float]] = None
        best_proj_m: Optional[float] = None
        for point_xy in candidates:
            px = float(point_xy[0]) - float(start_xy[0])
            py = float(point_xy[1]) - float(start_xy[1])
            proj_m = (px * ux) + (py * uy)
            cross_m = abs((px * uy) - (py * ux))
            if proj_m <= 1e-6 or cross_m > 2.0:
                continue
            if best_proj_m is None or proj_m < best_proj_m:
                best_proj_m = float(proj_m)
                best_xy = (float(point_xy[0]), float(point_xy[1]))
        return best_xy

    def _first_sweep_segment_point_xy(
        self,
        sweep_lines_xy: Sequence[Sequence[Tuple[float, float]]],
        start_xy: Tuple[float, float],
        toward_xy: Tuple[float, float],
    ) -> Optional[Tuple[Tuple[float, float], float]]:
        if not sweep_lines_xy or _distance(start_xy, toward_xy) <= 1e-6:
            return None

        dx = float(toward_xy[0]) - float(start_xy[0])
        dy = float(toward_xy[1]) - float(start_xy[1])
        path_len_m = math.hypot(dx, dy)
        if path_len_m <= 1e-6:
            return None
        ux = dx / path_len_m
        uy = dy / path_len_m
        pad_m = max(80.0, path_len_m * 0.5)
        probe_line = LineString(
            [
                start_xy,
                (float(toward_xy[0]) + (ux * pad_m), float(toward_xy[1]) + (uy * pad_m)),
            ]
        )

        best_xy: Optional[Tuple[float, float]] = None
        best_proj_m: Optional[float] = None
        for line_xy in sweep_lines_xy:
            if not isinstance(line_xy, (list, tuple)) or len(line_xy) < 2:
                continue
            seg_points = [
                (float(point_xy[0]), float(point_xy[1]))
                for point_xy in line_xy
                if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2
            ]
            if len(seg_points) < 2:
                continue
            intersection = LineString(seg_points).intersection(probe_line)
            candidates = self._geometry_point_candidates_xy(intersection)
            if not candidates:
                continue
            for point_xy in candidates:
                px = float(point_xy[0]) - float(start_xy[0])
                py = float(point_xy[1]) - float(start_xy[1])
                proj_m = (px * ux) + (py * uy)
                cross_m = abs((px * uy) - (py * ux))
                if proj_m <= 1e-6 or cross_m > 2.0:
                    continue
                if best_proj_m is None or proj_m < best_proj_m:
                    best_proj_m = float(proj_m)
                    best_xy = (float(point_xy[0]), float(point_xy[1]))
        if best_xy is None or best_proj_m is None:
            return None
        return best_xy, float(best_proj_m)

    def _project_along_xy(
        self,
        start_xy: Tuple[float, float],
        toward_xy: Tuple[float, float],
        distance_m: float,
    ) -> Tuple[float, float]:
        dx = float(toward_xy[0]) - float(start_xy[0])
        dy = float(toward_xy[1]) - float(start_xy[1])
        length_m = math.hypot(dx, dy)
        if length_m <= 1e-6:
            return (float(start_xy[0]), float(start_xy[1]))
        scale = float(distance_m) / float(length_m)
        return (
            float(start_xy[0]) + (dx * scale),
            float(start_xy[1]) + (dy * scale),
        )

    def _stage2_axis_guide_line(
        self,
        mission_poly: Polygon,
        center_xy: Tuple[float, float],
        bearing_deg: float,
    ) -> Optional[LineString]:
        minx, miny, maxx, maxy = mission_poly.bounds
        extent_m = max(float(maxx - minx), float(maxy - miny), 1.0) * 3.0
        theta = math.radians(float(bearing_deg) % 360.0)
        ux = math.sin(theta)
        uy = math.cos(theta)
        axis_line = LineString(
            [
                (float(center_xy[0]) - (ux * extent_m), float(center_xy[1]) - (uy * extent_m)),
                (float(center_xy[0]) + (ux * extent_m), float(center_xy[1]) + (uy * extent_m)),
            ]
        )
        clipped = mission_poly.intersection(axis_line)
        return self._longest_linestring_xy(clipped)

    def _stage2_guide_geom(
        self,
        mission_poly: Polygon,
        aircraft_id: int,
        piece: SplitPiece,
    ) -> Optional[Any]:
        target_xy = self._piece_assignment_target_xy(piece)
        geoms: List[Any] = []
        if target_xy is None:
            return None

        seg = self._stage2_piece_segment(int(aircraft_id), piece)
        path_points: List[Tuple[float, float]] = []
        uav_state = self._uav_state_for_aircraft(int(aircraft_id))
        if uav_state is not None:
            path_points.append((float(uav_state[0][0]), float(uav_state[0][1])))
        if isinstance(seg, dict):
            turn_points = seg.get("turnPoints")
            if isinstance(turn_points, list):
                for point_xy in turn_points:
                    if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2:
                        path_points.append((float(point_xy[0]), float(point_xy[1])))
            start_xy = seg.get("startXY")
            if isinstance(start_xy, (tuple, list)) and len(start_xy) >= 2:
                path_points.append((float(start_xy[0]), float(start_xy[1])))
        path_points.append((float(target_xy[0]), float(target_xy[1])))
        path_points = _dedupe_points(path_points, min_dist_m=5.0)
        if len(path_points) >= 2:
            geoms.append(LineString(path_points))

        bearing_deg = self._piece_bearing_deg(piece)
        if bearing_deg is None and len(path_points) >= 2:
            bearing_deg = _bearing_deg_from_xy(path_points[-2], path_points[-1])
        if bearing_deg is not None:
            axis_line = self._stage2_axis_guide_line(mission_poly, target_xy, float(bearing_deg))
            if axis_line is not None:
                geoms.append(axis_line)

        if not geoms:
            return None
        if len(geoms) == 1:
            return geoms[0]
        return unary_union(geoms)

    def _stage2_guide_distance_m(
        self,
        guide_geom: Optional[Any],
        point_xy: Tuple[float, float],
    ) -> float:
        if guide_geom is None or getattr(guide_geom, "is_empty", True):
            return float("inf")
        return float(guide_geom.distance(Point(float(point_xy[0]), float(point_xy[1]))))

    def _stage2_smooth_polygon_xy(
        self,
        poly: Polygon,
        mission_poly: Polygon,
        *,
        grid_size_m: float,
    ) -> Polygon:
        if poly.is_empty:
            return poly
        smooth_buffer_m = max(8.0, float(grid_size_m) * float(STAGE2_SMOOTH_BUFFER_RATIO))
        simplify_m = max(float(STAGE2_SIMPLIFY_MIN_M), float(grid_size_m) * float(STAGE2_SIMPLIFY_RATIO))

        # Close small staircase teeth from the grid partition first.
        smoothed = poly.buffer(
            float(smooth_buffer_m),
            join_style=2,
            quad_segs=1,
        ).buffer(
            -float(smooth_buffer_m),
            join_style=2,
            quad_segs=1,
        )
        candidate = self._largest_polygon_xy(smoothed)
        if candidate is None:
            candidate = poly

        # Then aggressively collapse tiny jagged edges into a cleaner polygonal boundary.
        simplified = candidate.simplify(float(simplify_m), preserve_topology=False)
        candidate = self._largest_polygon_xy(simplified) or candidate

        clipped = mission_poly.intersection(candidate).buffer(0)
        return self._largest_polygon_xy(clipped) or poly

    def _stage2_relax_polygon_xy(
        self,
        poly: Polygon,
        clip_poly: Polygon,
        *,
        grid_size_m: float,
    ) -> Polygon:
        if poly.is_empty:
            return poly
        relax_buffer_m = max(4.0, float(grid_size_m) * float(STAGE2_PAIR_RELAX_BUFFER_RATIO))
        simplify_m = max(8.0, float(grid_size_m) * float(STAGE2_PAIR_SIMPLIFY_RATIO))
        relaxed = poly.buffer(
            float(relax_buffer_m),
            join_style=2,
            quad_segs=1,
        )
        candidate = self._largest_polygon_xy(relaxed) or poly
        simplified = candidate.simplify(float(simplify_m), preserve_topology=True)
        candidate = self._largest_polygon_xy(simplified) or candidate
        clipped = clip_poly.intersection(candidate).buffer(0)
        return self._largest_polygon_xy(clipped) or poly

    def _stage2_expand_overlap_polygon_xy(
        self,
        poly: Polygon,
        mission_poly: Polygon,
        *,
        grid_size_m: float,
    ) -> Polygon:
        if poly.is_empty:
            return poly
        overlap_buffer_m = max(3.0, float(grid_size_m) * float(STAGE2_OVERLAP_BUFFER_RATIO))
        expanded = poly.buffer(
            float(overlap_buffer_m),
            join_style=2,
            quad_segs=1,
        )
        candidate = self._largest_polygon_xy(expanded) or poly
        clipped = mission_poly.intersection(candidate).buffer(0)
        return self._largest_polygon_xy(clipped) or poly

    def _stage2_line_parts(self, geom: Any) -> List[LineString]:
        if geom is None or getattr(geom, "is_empty", True):
            return []
        if isinstance(geom, LineString):
            return [geom] if float(geom.length) > 1e-6 else []
        gt = str(getattr(geom, "geom_type", ""))
        if gt not in {"MultiLineString", "GeometryCollection"}:
            return []
        out: List[LineString] = []
        for item in getattr(geom, "geoms", []):
            out.extend(self._stage2_line_parts(item))
        return out

    def _stage2_fit_line_direction_xy(
        self,
        points_xy: Sequence[Tuple[float, float]],
    ) -> Optional[Tuple[float, float]]:
        if len(points_xy) < 2:
            return None
        mx = sum(float(p[0]) for p in points_xy) / float(len(points_xy))
        my = sum(float(p[1]) for p in points_xy) / float(len(points_xy))
        sxx = sum((float(p[0]) - mx) ** 2 for p in points_xy)
        syy = sum((float(p[1]) - my) ** 2 for p in points_xy)
        sxy = sum((float(p[0]) - mx) * (float(p[1]) - my) for p in points_xy)
        if abs(sxx) < 1e-9 and abs(syy) < 1e-9:
            return None
        angle_rad = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
        vx = math.cos(angle_rad)
        vy = math.sin(angle_rad)
        norm = math.hypot(vx, vy)
        if norm <= 1e-9:
            return None
        return float(vx / norm), float(vy / norm)

    def _stage2_regularize_pair_boundary(
        self,
        poly_a: Polygon,
        poly_b: Polygon,
        *,
        seed_a_xy: Tuple[float, float],
        seed_b_xy: Tuple[float, float],
        grid_size_m: float,
    ) -> Tuple[Polygon, Polygon]:
        shared_geom = poly_a.boundary.intersection(poly_b.boundary)
        shared_lines = [
            line
            for line in self._stage2_line_parts(shared_geom)
            if float(line.length) >= max(6.0, float(grid_size_m) * 0.35)
        ]
        if not shared_lines:
            return poly_a, poly_b

        shared_len = sum(float(line.length) for line in shared_lines)
        if shared_len < max(20.0, float(grid_size_m) * 1.2):
            return poly_a, poly_b

        sample_points_xy: List[Tuple[float, float]] = []
        cx = 0.0
        cy = 0.0
        total_w = 0.0
        for line in shared_lines:
            coords = [(float(x), float(y)) for (x, y) in list(line.coords)]
            sample_points_xy.extend(coords)
            mid = line.interpolate(0.5, normalized=True)
            cx += float(mid.x) * float(line.length)
            cy += float(mid.y) * float(line.length)
            total_w += float(line.length)
        if total_w <= 1e-9:
            return poly_a, poly_b

        dir_xy = self._stage2_fit_line_direction_xy(sample_points_xy)
        if dir_xy is None:
            longest = max(shared_lines, key=lambda line: float(line.length))
            coords = list(longest.coords)
            if len(coords) < 2:
                return poly_a, poly_b
            dx = float(coords[-1][0]) - float(coords[0][0])
            dy = float(coords[-1][1]) - float(coords[0][1])
            norm = math.hypot(dx, dy)
            if norm <= 1e-9:
                return poly_a, poly_b
            dir_xy = (float(dx / norm), float(dy / norm))

        union_poly = self._largest_polygon_xy(poly_a.union(poly_b))
        if union_poly is None:
            return poly_a, poly_b

        center_xy = (float(cx / total_w), float(cy / total_w))
        minx, miny, maxx, maxy = union_poly.bounds
        extent_m = max(float(maxx - minx), float(maxy - miny), 1.0) * 4.0
        cut_line = LineString(
            [
                (
                    float(center_xy[0]) - (float(dir_xy[0]) * extent_m),
                    float(center_xy[1]) - (float(dir_xy[1]) * extent_m),
                ),
                (
                    float(center_xy[0]) + (float(dir_xy[0]) * extent_m),
                    float(center_xy[1]) + (float(dir_xy[1]) * extent_m),
                ),
            ]
        )

        try:
            split_out = geom_split(union_poly, cut_line)
        except Exception:
            return poly_a, poly_b

        parts = [
            geom
            for geom in getattr(split_out, "geoms", [])
            if isinstance(geom, Polygon) and not geom.is_empty and float(geom.area) > 1e-6
        ]
        if len(parts) < 2:
            return poly_a, poly_b

        point_a = Point(float(seed_a_xy[0]), float(seed_a_xy[1]))
        point_b = Point(float(seed_b_xy[0]), float(seed_b_xy[1]))
        parts_a: List[Polygon] = []
        parts_b: List[Polygon] = []
        for part in parts:
            overlap_a = float(part.intersection(poly_a).area)
            overlap_b = float(part.intersection(poly_b).area)
            if abs(overlap_a - overlap_b) <= 1e-6:
                rep = part.representative_point()
                if float(rep.distance(point_a)) <= float(rep.distance(point_b)):
                    parts_a.append(part)
                else:
                    parts_b.append(part)
            elif overlap_a >= overlap_b:
                parts_a.append(part)
            else:
                parts_b.append(part)

        if not parts_a or not parts_b:
            return poly_a, poly_b

        new_a = self._largest_polygon_xy(unary_union(parts_a))
        new_b = self._largest_polygon_xy(unary_union(parts_b))
        if new_a is None or new_b is None:
            return poly_a, poly_b
        new_a = self._stage2_relax_polygon_xy(
            new_a,
            union_poly,
            grid_size_m=float(grid_size_m),
        )
        new_b = self._stage2_relax_polygon_xy(
            new_b,
            union_poly,
            grid_size_m=float(grid_size_m),
        )
        return new_a, new_b

    def _stage2_regularize_shared_boundaries(
        self,
        polygon_by_aid: Dict[int, Polygon],
        *,
        seed_xy_by_aid: Dict[int, Tuple[float, float]],
        grid_size_m: float,
    ) -> Dict[int, Polygon]:
        aids = sorted(int(aid) for aid in polygon_by_aid.keys())
        out: Dict[int, Polygon] = {int(aid): poly for aid, poly in polygon_by_aid.items()}
        for _ in range(1):
            changed = False
            for left_idx in range(len(aids)):
                for right_idx in range(left_idx + 1, len(aids)):
                    aid_a = int(aids[left_idx])
                    aid_b = int(aids[right_idx])
                    poly_a = out.get(int(aid_a))
                    poly_b = out.get(int(aid_b))
                    if poly_a is None or poly_b is None:
                        continue
                    new_a, new_b = self._stage2_regularize_pair_boundary(
                        poly_a,
                        poly_b,
                        seed_a_xy=seed_xy_by_aid[int(aid_a)],
                        seed_b_xy=seed_xy_by_aid[int(aid_b)],
                        grid_size_m=float(grid_size_m),
                    )
                    if (
                        float(new_a.symmetric_difference(poly_a).area) > 1e-6
                        or float(new_b.symmetric_difference(poly_b).area) > 1e-6
                    ):
                        out[int(aid_a)] = new_a
                        out[int(aid_b)] = new_b
                        changed = True
            if not changed:
                break
        return out

    def _stage2_polygon_parts(self, geom: Any) -> List[Polygon]:
        if geom is None or getattr(geom, "is_empty", True):
            return []
        if isinstance(geom, Polygon):
            return [geom] if float(geom.area) > 1e-6 else []
        gt = str(getattr(geom, "geom_type", ""))
        if gt not in {"MultiPolygon", "GeometryCollection"}:
            return []
        out: List[Polygon] = []
        for item in getattr(geom, "geoms", []):
            out.extend(self._stage2_polygon_parts(item))
        return out

    def _stage2_fill_uncovered_gaps(
        self,
        mission_poly: Polygon,
        polygon_by_aid: Dict[int, Polygon],
        *,
        seed_xy_by_aid: Dict[int, Tuple[float, float]],
        grid_size_m: float,
    ) -> Dict[int, Polygon]:
        out: Dict[int, Polygon] = {int(aid): poly for aid, poly in polygon_by_aid.items()}
        contact_tol_m = max(1.0, float(grid_size_m) * 0.35)

        for _ in range(3):
            coverage = unary_union([poly for poly in out.values() if poly is not None and not poly.is_empty])
            uncovered = mission_poly.difference(coverage)
            gap_parts = [
                gap
                for gap in self._stage2_polygon_parts(uncovered)
                if float(gap.area) > 1e-3
            ]
            if not gap_parts:
                break

            changed = False
            for gap in sorted(gap_parts, key=lambda poly: float(poly.area), reverse=True):
                rep = gap.representative_point()
                rep_xy = (float(rep.x), float(rep.y))
                candidate_aids = [
                    int(aid)
                    for aid, poly in out.items()
                    if poly is not None and not poly.is_empty and float(poly.distance(gap)) <= float(contact_tol_m)
                ]
                if not candidate_aids:
                    candidate_aids = [int(aid) for aid in out.keys()]
                if not candidate_aids:
                    continue

                best_aid = min(
                    candidate_aids,
                    key=lambda aid: (
                        -float(out[int(aid)].boundary.intersection(gap.boundary).length),
                        _distance(rep_xy, seed_xy_by_aid.get(int(aid), rep_xy)),
                        float(out[int(aid)].distance(rep)),
                        int(aid),
                    ),
                )
                merged = out[int(best_aid)].union(gap).buffer(0)
                new_poly = self._largest_polygon_xy(merged)
                if new_poly is None:
                    continue
                out[int(best_aid)] = new_poly
                changed = True

            if not changed:
                break

        return out

    def _stage2_target_area_map(
        self,
        split_result: SplitRunResult,
        piece_by_aid: Dict[int, SplitPiece],
        ratio_map: Dict[int, float],
        *,
        grid_size_m: float,
    ) -> Tuple[Dict[int, float], Dict[int, Dict[str, float]]]:
        calculate_expected_velocity(
            split_result,
            expected_paths=self.state.expected_paths,
        )

        stats_by_aid: Dict[int, Dict[str, float]] = {}
        total_area_m2 = 0.0
        total_time_sec = 0.0
        total_valid_area_m2 = 0.0
        total_valid_scan_sec = 0.0

        for aid, piece in piece_by_aid.items():
            poly = self._piece_polygon_xy(piece)
            area_m2 = float(poly.area) if poly is not None else 0.0
            total_area_m2 += area_m2
            data = piece.data if isinstance(piece.data, dict) else {}
            exp_vel = data.get("expVel") if isinstance(data.get("expVel"), dict) else {}
            scan_sec_raw = exp_vel.get("timeSelectedSec")
            scan_sec = float(scan_sec_raw) if isinstance(scan_sec_raw, (int, float)) and float(scan_sec_raw) > 1e-6 else None
            if scan_sec is not None and area_m2 > 1e-6:
                total_valid_area_m2 += float(area_m2)
                total_valid_scan_sec += float(scan_sec)
            entry_sec = float(self._stage2_entry_time_sec(int(aid), piece))
            stats_by_aid[int(aid)] = {
                "currentAreaM2": float(area_m2),
                "entrySec": float(entry_sec),
                "scanSec": float(scan_sec) if scan_sec is not None else -1.0,
                "areaRateM2ps": -1.0,
            }

        common_area_rate_m2ps = (
            float(total_valid_area_m2) / float(total_valid_scan_sec)
            if total_valid_area_m2 > 1e-6 and total_valid_scan_sec > 1e-6
            else float(STAGE2_DEFAULT_AREA_RATE_M2PS)
        )
        ratio_sum = sum(max(0.0, float(v)) for v in ratio_map.values())
        if ratio_sum <= 1e-9:
            raise ValueError("Stage 2 ratio sum must be greater than 0.")

        raw_target_area_map: Dict[int, float] = {}
        min_target_area_m2 = float(grid_size_m * grid_size_m * STAGE2_MIN_CELL_AREA_RATIO)
        for aid, meta in stats_by_aid.items():
            scan_sec = float(meta["scanSec"]) if float(meta["scanSec"]) > 0.0 else (
                float(meta["currentAreaM2"]) / max(float(common_area_rate_m2ps), 1e-6)
            )
            current_total_sec = float(meta["entrySec"]) + float(scan_sec)
            total_time_sec += float(current_total_sec)
            meta["scanSec"] = float(scan_sec)
            meta["areaRateM2ps"] = float(common_area_rate_m2ps)
            meta["areaRateMode"] = "shared"
            meta["currentTotalSec"] = float(current_total_sec)

        for aid, meta in stats_by_aid.items():
            desired_total_sec = float(total_time_sec) * (float(ratio_map.get(int(aid), 0.0)) / float(ratio_sum))
            desired_scan_sec = max(1.0, float(desired_total_sec) - float(meta["entrySec"]))
            raw_target_area_m2 = max(min_target_area_m2, desired_scan_sec * float(common_area_rate_m2ps))
            raw_target_area_map[int(aid)] = float(raw_target_area_m2)
            meta["desiredTotalSec"] = float(desired_total_sec)
            meta["desiredScanSec"] = float(desired_scan_sec)
            meta["sharedAreaRateM2ps"] = float(common_area_rate_m2ps)

        raw_sum_m2 = sum(float(v) for v in raw_target_area_map.values())
        scale = (float(total_area_m2) / float(raw_sum_m2)) if raw_sum_m2 > 1e-6 else 1.0
        target_area_map: Dict[int, float] = {}
        for aid, raw_area_m2 in raw_target_area_map.items():
            target_area_map[int(aid)] = float(raw_area_m2) * float(scale)
            stats_by_aid[int(aid)]["targetAreaM2"] = float(target_area_map[int(aid)])

        return target_area_map, stats_by_aid

    def _stage2_build_cells(
        self,
        mission_poly: Polygon,
        *,
        cell_size_m: float,
    ) -> Tuple[List[Dict[str, Any]], List[List[int]]]:
        minx, miny, maxx, maxy = mission_poly.bounds
        cell_size = max(5.0, float(cell_size_m))
        start_x = math.floor(minx / cell_size) * cell_size
        start_y = math.floor(miny / cell_size) * cell_size
        nx = max(1, int(math.ceil((maxx - start_x) / cell_size)))
        ny = max(1, int(math.ceil((maxy - start_y) / cell_size)))
        min_area_m2 = float(cell_size * cell_size * STAGE2_MIN_CELL_AREA_RATIO)

        cells: List[Dict[str, Any]] = []
        rc_to_idx: Dict[Tuple[int, int], int] = {}
        for row in range(ny):
            for col in range(nx):
                x0 = start_x + (float(col) * cell_size)
                y0 = start_y + (float(row) * cell_size)
                clipped = mission_poly.intersection(box(x0, y0, x0 + cell_size, y0 + cell_size))
                poly = self._largest_polygon_xy(clipped)
                if poly is None or float(poly.area) < float(min_area_m2):
                    continue
                point = poly.representative_point()
                idx = len(cells)
                cells.append(
                    {
                        "geom": poly,
                        "centerXY": (float(point.x), float(point.y)),
                        "areaM2": float(poly.area),
                        "rc": (int(col), int(row)),
                    }
                )
                rc_to_idx[(int(col), int(row))] = int(idx)

        neighbors: List[List[int]] = [[] for _ in cells]
        for idx, cell in enumerate(cells):
            col, row = cell["rc"]
            for dc, dr in (
                (1, 0),
                (-1, 0),
                (0, 1),
                (0, -1),
                (1, 1),
                (1, -1),
                (-1, 1),
                (-1, -1),
            ):
                near_idx = rc_to_idx.get((int(col) + int(dc), int(row) + int(dr)))
                if near_idx is not None:
                    neighbors[idx].append(int(near_idx))

        return cells, neighbors

    def _stage2_pick_seed_cell_idx(
        self,
        cells: Sequence[Dict[str, Any]],
        seed_xy: Tuple[float, float],
        *,
        piece_poly: Optional[Polygon] = None,
        used_indices: Optional[set[int]] = None,
    ) -> Optional[int]:
        used = used_indices if used_indices is not None else set()
        best_inside: Optional[Tuple[float, int]] = None
        best_any: Optional[Tuple[float, int]] = None
        for idx, cell in enumerate(cells):
            if idx in used:
                continue
            center_xy = cell["centerXY"]
            dist_m = _distance(center_xy, seed_xy)
            point = Point(float(center_xy[0]), float(center_xy[1]))
            if piece_poly is not None and float(piece_poly.distance(point)) <= 1e-6:
                if best_inside is None or float(dist_m) < float(best_inside[0]):
                    best_inside = (float(dist_m), int(idx))
            if best_any is None or float(dist_m) < float(best_any[0]):
                best_any = (float(dist_m), int(idx))
        if best_inside is not None:
            return int(best_inside[1])
        if best_any is not None:
            return int(best_any[1])
        return None

    def _stage2_growth_score(
        self,
        cell_center_xy: Tuple[float, float],
        *,
        seed_xy: Tuple[float, float],
        anchor_xy: Tuple[float, float],
        guide_geom: Optional[Any],
        current_area_m2: float,
        target_area_m2: float,
    ) -> float:
        guide_dist_m = self._stage2_guide_distance_m(guide_geom, cell_center_xy)
        if not math.isfinite(guide_dist_m):
            guide_dist_m = (
                ((1.0 - float(STAGE2_ANCHOR_BLEND)) * _distance(cell_center_xy, seed_xy))
                + (float(STAGE2_ANCHOR_BLEND) * _distance(cell_center_xy, anchor_xy))
            )
        seed_dist_m = _distance(cell_center_xy, seed_xy)
        anchor_dist_m = _distance(cell_center_xy, anchor_xy)
        half_swath_m = float(STAGE2_MAX_SWATH_WIDTH_M) * 0.5
        over_width_m = max(0.0, float(guide_dist_m) - float(half_swath_m))
        width_penalty = (float(over_width_m) * 10.0) + ((float(over_width_m) ** 2) * 0.06)
        base_score = (
            (float(guide_dist_m) * 1.9)
            + (float(seed_dist_m) * 0.08)
            + (float(anchor_dist_m) * 0.04)
            + float(width_penalty)
        )
        overload_ratio = max(0.0, float(current_area_m2) - float(target_area_m2)) / max(float(target_area_m2), 1.0)
        return float(base_score) + (800.0 * float(overload_ratio))

    def _stage2_rebalance_area_polygons(
        self,
        mission_poly: Polygon,
        piece_by_aid: Dict[int, SplitPiece],
        target_area_map: Dict[int, float],
        *,
        grid_size_m: float,
    ) -> Tuple[Dict[int, Polygon], Dict[int, float]]:
        cells, neighbors = self._stage2_build_cells(mission_poly, cell_size_m=float(grid_size_m))
        if len(cells) < len(piece_by_aid):
            raise ValueError("Stage 2 review needs at least one grid cell per assigned UAV.")

        aids = sorted(int(aid) for aid in piece_by_aid.keys())
        piece_poly_by_aid = {int(aid): self._piece_polygon_xy(piece) for aid, piece in piece_by_aid.items()}
        seed_xy_by_aid = {int(aid): self._stage2_seed_xy(int(aid), piece) for aid, piece in piece_by_aid.items()}
        guide_geom_by_aid = {
            int(aid): self._stage2_guide_geom(mission_poly, int(aid), piece)
            for aid, piece in piece_by_aid.items()
        }
        anchor_xy_by_aid: Dict[int, Tuple[float, float]] = {}
        for aid, piece in piece_by_aid.items():
            anchor_xy = seed_xy_by_aid[int(aid)]
            seg = self._stage2_piece_segment(int(aid), piece)
            if isinstance(seg, dict):
                start_xy = seg.get("startXY")
                if isinstance(start_xy, (tuple, list)) and len(start_xy) >= 2:
                    anchor_xy = (float(start_xy[0]), float(start_xy[1]))
            else:
                uav_state = self._uav_state_for_aircraft(int(aid))
                if uav_state is not None:
                    anchor_xy = uav_state[0]
            anchor_xy_by_aid[int(aid)] = anchor_xy

        owner_by_idx: List[int] = [0 for _ in cells]
        frontier_by_aid: Dict[int, set[int]] = {int(aid): set() for aid in aids}
        current_area_by_aid: Dict[int, float] = {int(aid): 0.0 for aid in aids}
        used_seed_idx: set[int] = set()

        for aid in aids:
            seed_idx = self._stage2_pick_seed_cell_idx(
                cells,
                seed_xy_by_aid[int(aid)],
                piece_poly=piece_poly_by_aid.get(int(aid)),
                used_indices=used_seed_idx,
            )
            if seed_idx is None:
                raise ValueError(f"Stage 2 seed cell selection failed for UAV{aid}.")
            owner_by_idx[int(seed_idx)] = int(aid)
            current_area_by_aid[int(aid)] += float(cells[int(seed_idx)]["areaM2"])
            used_seed_idx.add(int(seed_idx))

        for aid in aids:
            for idx, owner in enumerate(owner_by_idx):
                if int(owner) != int(aid):
                    continue
                for near_idx in neighbors[idx]:
                    if owner_by_idx[int(near_idx)] <= 0:
                        frontier_by_aid[int(aid)].add(int(near_idx))

        unassigned_n = sum(1 for owner in owner_by_idx if int(owner) <= 0)
        guard = 0
        while unassigned_n > 0 and guard < (len(cells) * 12):
            guard += 1
            candidate_aid: Optional[int] = None
            candidate_idx: Optional[int] = None

            underfilled = [
                int(aid)
                for aid in aids
                if frontier_by_aid[int(aid)] and (float(current_area_by_aid[int(aid)]) + 1e-6) < float(target_area_map[int(aid)])
            ]
            if underfilled:
                candidate_aid = max(
                    underfilled,
                    key=lambda aid: (
                        (float(target_area_map[int(aid)]) - float(current_area_by_aid[int(aid)]))
                        / max(float(target_area_map[int(aid)]), 1.0)
                    ),
                )
                candidate_idx = min(
                    frontier_by_aid[int(candidate_aid)],
                    key=lambda idx: self._stage2_growth_score(
                        cells[int(idx)]["centerXY"],
                        seed_xy=seed_xy_by_aid[int(candidate_aid)],
                        anchor_xy=anchor_xy_by_aid[int(candidate_aid)],
                        guide_geom=guide_geom_by_aid.get(int(candidate_aid)),
                        current_area_m2=float(current_area_by_aid[int(candidate_aid)]),
                        target_area_m2=float(target_area_map[int(candidate_aid)]),
                    ),
                )
            else:
                best_row: Optional[Tuple[float, int, int]] = None
                for aid in aids:
                    for idx in frontier_by_aid[int(aid)]:
                        score = self._stage2_growth_score(
                            cells[int(idx)]["centerXY"],
                            seed_xy=seed_xy_by_aid[int(aid)],
                            anchor_xy=anchor_xy_by_aid[int(aid)],
                            guide_geom=guide_geom_by_aid.get(int(aid)),
                            current_area_m2=float(current_area_by_aid[int(aid)]),
                            target_area_m2=float(target_area_map[int(aid)]),
                        )
                        row = (float(score), int(aid), int(idx))
                        if best_row is None or row < best_row:
                            best_row = row
                if best_row is not None:
                    candidate_aid = int(best_row[1])
                    candidate_idx = int(best_row[2])

            if candidate_aid is None or candidate_idx is None:
                unassigned_indices = [idx for idx, owner in enumerate(owner_by_idx) if int(owner) <= 0]
                if not unassigned_indices:
                    break
                best_row = min(
                    (
                        (
                            self._stage2_growth_score(
                                cells[int(idx)]["centerXY"],
                                seed_xy=seed_xy_by_aid[int(aid)],
                                anchor_xy=anchor_xy_by_aid[int(aid)],
                                guide_geom=guide_geom_by_aid.get(int(aid)),
                                current_area_m2=float(current_area_by_aid[int(aid)]),
                                target_area_m2=float(target_area_map[int(aid)]),
                            ),
                            int(aid),
                            int(idx),
                        )
                        for aid in aids
                        for idx in unassigned_indices
                    ),
                    key=lambda row: (float(row[0]), int(row[1]), int(row[2])),
                )
                candidate_aid = int(best_row[1])
                candidate_idx = int(best_row[2])

            owner_by_idx[int(candidate_idx)] = int(candidate_aid)
            current_area_by_aid[int(candidate_aid)] += float(cells[int(candidate_idx)]["areaM2"])
            for aid in aids:
                frontier_by_aid[int(aid)].discard(int(candidate_idx))
            for near_idx in neighbors[int(candidate_idx)]:
                if owner_by_idx[int(near_idx)] <= 0:
                    frontier_by_aid[int(candidate_aid)].add(int(near_idx))
            unassigned_n -= 1

        polygon_by_aid: Dict[int, Polygon] = {}
        for aid in aids:
            geoms = [cells[idx]["geom"] for idx, owner in enumerate(owner_by_idx) if int(owner) == int(aid)]
            if not geoms:
                raise ValueError(f"Stage 2 produced an empty region for UAV{aid}.")
            merged = unary_union(geoms)
            clipped = mission_poly.intersection(merged).buffer(0)
            poly = self._largest_polygon_xy(clipped)
            if poly is None:
                raise ValueError(f"Stage 2 polygon generation failed for UAV{aid}.")
            poly = self._stage2_smooth_polygon_xy(
                poly,
                mission_poly,
                grid_size_m=float(grid_size_m),
            )
            polygon_by_aid[int(aid)] = poly

        polygon_by_aid = self._stage2_regularize_shared_boundaries(
            polygon_by_aid,
            seed_xy_by_aid=seed_xy_by_aid,
            grid_size_m=float(grid_size_m),
        )
        polygon_by_aid = self._stage2_fill_uncovered_gaps(
            mission_poly,
            polygon_by_aid,
            seed_xy_by_aid=seed_xy_by_aid,
            grid_size_m=float(grid_size_m),
        )
        polygon_by_aid = self._stage2_regularize_shared_boundaries(
            polygon_by_aid,
            seed_xy_by_aid=seed_xy_by_aid,
            grid_size_m=float(grid_size_m),
        )
        for aid in aids:
            polygon_by_aid[int(aid)] = self._stage2_expand_overlap_polygon_xy(
                polygon_by_aid[int(aid)],
                mission_poly,
                grid_size_m=float(grid_size_m),
            )

        achieved_area_map: Dict[int, float] = {}
        for aid in aids:
            achieved_area_map[int(aid)] = float(polygon_by_aid[int(aid)].area)

        return polygon_by_aid, achieved_area_map

    def _stage2_apply_review_polygons(
        self,
        piece_by_aid: Dict[int, SplitPiece],
        polygon_by_aid: Dict[int, Polygon],
        ratio_map: Dict[int, float],
        target_area_map: Dict[int, float],
        *,
        grid_size_m: float,
    ) -> None:
        for aid, piece in piece_by_aid.items():
            poly = polygon_by_aid.get(int(aid))
            if poly is None:
                continue
            coords_xy = [(float(x), float(y)) for (x, y) in list(poly.exterior.coords)[:-1]]
            if len(coords_xy) < 3:
                continue
            data = copy.deepcopy(piece.data if isinstance(piece.data, dict) else {})
            alt_m = 0.0
            old_coords = data.get("coordinateList") if isinstance(data.get("coordinateList"), list) else []
            if old_coords and isinstance(old_coords[0], dict):
                alt_m = float(old_coords[0].get("altitude", 0.0) or 0.0)
            coord_list = [meters_to_coord(x, y, alt_m=alt_m) for (x, y) in coords_xy]
            data["coordinateList"] = coord_list
            data["rawCoordinateList"] = copy.deepcopy(coord_list)
            review = data.get("reviewArea") if isinstance(data.get("reviewArea"), dict) else {}
            review.update(
                {
                    "stage2Rebalanced": True,
                    "targetRatio": float(ratio_map.get(int(aid), 0.0)),
                    "targetAreaM2": float(target_area_map.get(int(aid), 0.0)),
                    "reviewedAreaM2": float(poly.area),
                    "gridSizeM": float(grid_size_m),
                    "maxSwathWidthM": float(STAGE2_MAX_SWATH_WIDTH_M),
                }
            )
            data["reviewArea"] = review
            piece.data = data
            piece.assigned_uav = int(aid)

    def _has_stage2_review(self, split_result: Optional[SplitRunResult]) -> bool:
        if split_result is None:
            return False
        for piece in split_result.pieces:
            data = piece.data if isinstance(piece.data, dict) else {}
            review = data.get("reviewArea")
            if isinstance(review, dict) and bool(review.get("stage2Rebalanced", False)):
                return True
        return False

    def _uav_state_for_aircraft(self, aircraft_id: int) -> Optional[Tuple[Tuple[float, float], float]]:
        aid = int(aircraft_id)
        for idx, current_aid in enumerate(self.state.uav_ids):
            if int(current_aid) != aid:
                continue
            if idx >= len(self.state.uav_positions_xy) or idx >= len(self.state.uav_heading_deg):
                return None
            heading = self.state.uav_heading_deg[idx]
            if heading is None:
                return None
            return self.state.uav_positions_xy[idx], float(heading)
        return None

    def _find_visibility_segment(
        self,
        aircraft_id: int,
        origin_xy: Tuple[float, float],
        bearing_deg: float,
        target_xy: Tuple[float, float],
    ) -> Optional[Dict[str, Any]]:
        if self._line_avoids_turn_circles(origin_xy, target_xy, origin_xy, bearing_deg):
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
                    (2.0 * math.pi * TURN_PREVIEW_RADIUS_M)
                    / (TURN_PREVIEW_SPEED_MPS * TURN_PREVIEW_HORIZON_S)
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
            )
            candidates: List[Dict[str, Any]] = []
            branch_points = (("L", left_xy), ("R", right_xy))
            if preferred_branch == "L":
                branch_points = (("L", left_xy),)
            elif preferred_branch == "R":
                branch_points = (("R", right_xy),)
            for branch, candidate_xy in branch_points:
                if not self._line_avoids_turn_circles(candidate_xy, target_xy, origin_xy, bearing_deg):
                    continue
                turn_points: List[Tuple[float, float]] = []
                turn_point_horizons: List[float] = []
                max_marker_idx = step_idx
                for prior_idx in range(1, max_marker_idx + 1):
                    prior_horizon_s = float(prior_idx) * float(TURN_PREVIEW_HORIZON_S)
                    prior_left_xy, prior_right_xy = self._turn_prediction_points_xy(
                        origin_xy,
                        bearing_deg,
                        horizon_s=prior_horizon_s,
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
                    )
                    if refined is not None:
                        start_xy = refined["startXY"]
                        start_horizon_s = float(refined["horizonSec"])
                        if turn_points:
                            turn_points[-1] = start_xy
                            turn_point_horizons[-1] = float(start_horizon_s)
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
                best = min(
                    candidates,
                    key=lambda row: (
                        float(row.get("horizonSec", 0.0) or 0.0),
                        _distance(row["startXY"], target_xy),
                    ),
                )
                return best
        return None

    def _ensure_ready_for_division(self, *, area_only: bool = False) -> bool:
        if not self.state.mission_points_xy or self.state.mission_kind is None:
            QMessageBox.warning(self, "임무 없음", "임무 형상을 먼저 만들어 주세요.")
            return False
        if area_only and self.state.mission_kind != MISSION_AREA:
            QMessageBox.warning(self, "영역 분할", "Area Division Run은 영역 임무에서만 사용할 수 있습니다.")
            return False
        if not self.state.uav_ids:
            QMessageBox.warning(self, "UAV 없음", "UAV 대수를 먼저 확정해 주세요.")
            return False
        if not self._uav_inputs_complete():
            QMessageBox.warning(self, "UAV 입력 부족", "모든 UAV의 위치와 heading을 입력해 주세요.")
            return False
        return True

    def _direction_debug_lines(self, split_result: SplitRunResult) -> List[str]:
        lines: List[str] = []
        for direction in split_result.directions:
            if direction.bearing_move_deg is None and direction.bearing_split_deg is None:
                continue
            area_tag = ""
            if direction.source_area_index is not None:
                area_tag = f" A{int(direction.source_area_index)}"
            text = (
                f"[DIR] M{direction.parent_order}{area_tag} ID={direction.mission_id} "
                f"move={float(direction.bearing_move_deg or 0.0):.2f} "
                f"split={float(direction.bearing_split_deg or 0.0):.2f}"
            )
            if direction.bearing_in_deg is not None:
                text += f" in={float(direction.bearing_in_deg):.2f}"
            if direction.bearing_out_deg is not None:
                text += f" out={float(direction.bearing_out_deg):.2f}"
            lines.append(text)
        return lines

    def _run_split_stage(self) -> Tuple[SplitRunResult, List[str]]:
        self._cmpk_payload = self._build_cmpk_payload()
        self._mrpk_payload = self._build_mrpk_payload()
        use_replan_flow = self._is_replan_flow()
        split_result = run_split_pipeline(
            self._cmpk_payload,
            self._mrpk_payload,
            list(self.state.uav_ids),
            apply_assignment=use_replan_flow,
            apply_scheduling=False,
        )
        stage_lines = [f"[1] flow={self._flow_mode()} split pieces={len(split_result.pieces)}"]
        if use_replan_flow:
            stage_lines.append(f"[1a] preassign {self._assignment_summary_text(split_result)}")
        stage_lines.extend(self._direction_debug_lines(split_result))
        return split_result, stage_lines

    def _visible_uav_ids(self) -> List[int]:
        out: List[int] = []
        mapping = (
            (4, getattr(self, "chk_view_uav4", None)),
            (5, getattr(self, "chk_view_uav5", None)),
            (6, getattr(self, "chk_view_uav6", None)),
        )
        for aid, widget in mapping:
            if widget is None or widget.isChecked():
                out.append(int(aid))
        return out

    def _start_area_input(self) -> None:
        self.state.mode = MODE_DRAW_AREA
        self.state.mission_kind = MISSION_AREA
        self.state.draft_points_xy = []
        self.state.mission_points_xy = []
        self.state.line_width_pending = False
        self.state.uav_ids = []
        self.state.uav_positions_xy = []
        self.state.uav_heading_deg = []
        self._clear_plan_result()
        self._set_result("영역 입력을 시작했습니다.")
        self._refresh_ui()

    def _start_line_input(self) -> None:
        self.state.mode = MODE_DRAW_LINE
        self.state.mission_kind = MISSION_LINE
        self.state.draft_points_xy = []
        self.state.mission_points_xy = []
        self.state.line_width_m = float(self.spin_line_width.value())
        self.state.line_width_pending = False
        self.state.uav_ids = []
        self.state.uav_positions_xy = []
        self.state.uav_heading_deg = []
        self._clear_plan_result()
        self._set_result("Line 입력을 시작했습니다.")
        self._refresh_ui()

    def _undo_last_point(self) -> None:
        if self.state.mode in (MODE_DRAW_AREA, MODE_DRAW_LINE, MODE_LINE_WIDTH_PENDING):
            if self.state.draft_points_xy:
                self.state.draft_points_xy.pop()
                if self.state.mode == MODE_LINE_WIDTH_PENDING and len(self.state.draft_points_xy) >= 2:
                    self.state.mode = MODE_DRAW_LINE
                    self.state.line_width_pending = False
                self._clear_plan_result()
                self._refresh_ui()
                return

        if self.state.mode in (MODE_PLACE_UAV, MODE_SET_UAV_HEADING, MODE_RESULT_READY, MODE_MISSION_READY):
            if self._pop_last_uav_input():
                self._clear_plan_result()
                self._refresh_ui()

    def _on_line_width_changed(self, value: float) -> None:
        self.state.line_width_m = float(value)
        if self.state.mission_kind == MISSION_LINE and self.state.mission_points_xy:
            self._clear_plan_result()
        self._refresh_ui()

    def _on_uav_count_changed(self, _index: int) -> None:
        self._selected_uav_count = int(self.cmb_uav_count.currentText())
        if self.state.uav_ids and len(self.state.uav_ids) != self._selected_uav_count:
            self.state.uav_ids = []
            self.state.uav_positions_xy = []
            self.state.uav_heading_deg = []
            self.state.mode = MODE_MISSION_READY if self.state.mission_points_xy else self.state.mode
            self._clear_plan_result()
        self._refresh_ui()

    def _confirm_uav_count(self) -> None:
        if not self.state.mission_points_xy:
            QMessageBox.warning(self, "임무 없음", "임무 형상을 먼저 입력해 주세요.")
            return

        self.state.uav_ids = list(_UAV_IDS[: self._selected_uav_count])
        self.state.uav_positions_xy = []
        self.state.uav_heading_deg = []
        self.state.mode = MODE_MISSION_READY
        self._clear_plan_result()
        self._append_result(f"UAV 대수 확정: {', '.join(f'UAV{aid}' for aid in self.state.uav_ids)}")
        self._refresh_ui()

    def _start_uav_input(self) -> None:
        if not self.state.uav_ids:
            QMessageBox.warning(self, "UAV 대수", "UAV 대수를 먼저 확정해 주세요.")
            return
        self.state.uav_positions_xy = []
        self.state.uav_heading_deg = []
        self.state.mode = MODE_PLACE_UAV
        self._clear_plan_result()
        self._append_result("UAV 입력 시작: 한 번 클릭해 위치를 두고, 다시 클릭해 heading을 확정합니다.")
        self._refresh_ui()

    def _on_canvas_left_click(self, east_m: float, north_m: float) -> None:
        point_xy = (float(east_m), float(north_m))
        if self.state.mode in (MODE_DRAW_AREA, MODE_DRAW_LINE):
            if self.state.draft_points_xy and _distance(self.state.draft_points_xy[-1], point_xy) < 1.0:
                return
            self.state.draft_points_xy.append(point_xy)
            self._clear_plan_result()
            self._refresh_ui()
            return

        if self.state.mode == MODE_PLACE_UAV:
            if len(self.state.uav_positions_xy) >= len(self.state.uav_ids):
                return
            if self.state.uav_positions_xy and _distance(self.state.uav_positions_xy[-1], point_xy) < 1.0:
                return
            self.state.uav_positions_xy.append(point_xy)
            self.state.uav_heading_deg.append(None)
            aid = self.state.uav_ids[len(self.state.uav_positions_xy) - 1]
            self.state.mode = MODE_SET_UAV_HEADING
            self._clear_plan_result()
            self._append_result(f"UAV{aid} 위치 설정 완료. 다시 클릭해 heading을 정하세요.")
            self._refresh_ui()
            return

        if self.state.mode == MODE_SET_UAV_HEADING:
            idx = self._pending_uav_heading_index()
            if idx is None or idx >= len(self.state.uav_positions_xy):
                return
            anchor_xy = self.state.uav_positions_xy[idx]
            if _distance(anchor_xy, point_xy) < 5.0:
                return
            heading = _bearing_deg_from_xy(anchor_xy, point_xy)
            if heading is None:
                return
            while len(self.state.uav_heading_deg) <= idx:
                self.state.uav_heading_deg.append(None)
            self.state.uav_heading_deg[idx] = float(heading)
            aid = self.state.uav_ids[idx] if idx < len(self.state.uav_ids) else idx + 1
            self._clear_plan_result()
            self._append_result(f"UAV{aid} heading 설정: {float(heading):.1f} deg")
            if self._uav_inputs_complete():
                self.state.mode = MODE_MISSION_READY
                self._append_result("모든 UAV 위치와 heading 입력이 완료되었습니다. 계획을 실행할 수 있습니다.")
            else:
                self.state.mode = MODE_PLACE_UAV
            self._refresh_ui()
            return

    def _on_canvas_right_click(self, _east_m: float, _north_m: float) -> None:
        if self.state.mode == MODE_DRAW_AREA:
            self._finalize_area()
            return
        if self.state.mode == MODE_DRAW_LINE:
            self._enter_line_width_pending()
            return
        if self.state.mode == MODE_LINE_WIDTH_PENDING:
            self._finalize_line()
            return
        if self.state.mode == MODE_SET_UAV_HEADING:
            self._undo_last_point()

    def _finalize_area(self) -> None:
        if len(self.state.draft_points_xy) < 3:
            QMessageBox.warning(self, "영역 점 부족", "영역은 최소 3개 점이 필요합니다.")
            return
        try:
            points_xy, corrected = normalize_area_points(self.state.draft_points_xy)
        except Exception as exc:
            QMessageBox.critical(self, "영역 생성 실패", str(exc))
            return

        self.state.mission_points_xy = points_xy
        self.state.draft_points_xy = []
        self.state.mode = MODE_MISSION_READY
        self.state.line_width_pending = False
        self._clear_plan_result()

        msg = f"영역 입력 완료: {len(points_xy)}개 점"
        if corrected:
            msg += " (자가교차/중복점 자동 보정)"
        self._append_result(msg)
        self._refresh_ui()

    def _enter_line_width_pending(self) -> None:
        if len(self.state.draft_points_xy) < 2:
            QMessageBox.warning(self, "Line 점 부족", "Line은 최소 2개 점이 필요합니다.")
            return
        self.state.mode = MODE_LINE_WIDTH_PENDING
        self.state.line_width_pending = True
        self.state.line_width_m = float(self.spin_line_width.value())
        self._refresh_ui()

    def _finalize_line(self) -> None:
        points_xy = _dedupe_points(self.state.draft_points_xy)
        if len(points_xy) < 2:
            QMessageBox.warning(self, "Line 점 부족", "Line은 최소 2개 점이 필요합니다.")
            return
        if self.state.line_width_m <= 0.0:
            QMessageBox.warning(self, "폭 오류", "폭은 0보다 커야 합니다.")
            return

        self.state.mission_points_xy = points_xy
        self.state.draft_points_xy = []
        self.state.mode = MODE_MISSION_READY
        self.state.line_width_pending = False
        self._clear_plan_result()
        self._append_result(f"Line 입력 완료: {len(points_xy)}개 점 / 폭 {self.state.line_width_m:.1f}m")
        self._refresh_ui()

    def _run_area_division(self) -> None:
        if not self._ensure_ready_for_division(area_only=True):
            return

        try:
            self._cmpk_payload = self._build_cmpk_payload()
            self._mrpk_payload = self._build_mrpk_payload()
            split_result = run_split_pipeline(
                self._cmpk_payload,
                self._mrpk_payload,
                list(self.state.uav_ids),
                apply_assignment=False,
                apply_scheduling=False,
            )
            self._prefer_raw_split_preview_polygons(split_result)
            stage_lines = [f"[1] flow={self._flow_mode()} split pieces={len(split_result.pieces)}"]
            stage_lines.extend(self._direction_debug_lines(split_result))
            assign_report = self._assign_split_result_by_prediction_distance(split_result)
            stage_lines.append(
                "[1a] prediction-assign "
                f"speed={TURN_PREVIEW_SPEED_MPS:.0f}m/s "
                f"horizon={TURN_PREVIEW_HORIZON_S:.0f}s "
                f"radius={TURN_PREVIEW_RADIUS_M:.0f}m "
                f"assigned={int(assign_report.get('assignedPieces', 0))}/"
                f"{int(assign_report.get('pieceCount', 0))} "
                f"{self._assignment_summary_text(split_result)}"
            )
            self.state.split_result = split_result
            self.state.expected_paths = []
            self.state.assignment_path_rows = []
            self.state.flight_plans_0303 = []
            self.state.flight_plans_0304 = []
            self.state.show_turn_overlays = True
            self.state.mid_line_segments = []
            self.state.mode = MODE_RESULT_READY

            lines = [
                f"Mission: {self.state.mission_kind}",
                f"UAV: {', '.join(f'UAV{aid}' for aid in self.state.uav_ids)}",
                *stage_lines,
                "",
                f"Area division pieces: {len(split_result.pieces)}",
            ]
            piece_lines = assign_report.get("pieceLines")
            if isinstance(piece_lines, list) and piece_lines:
                lines.extend(str(text) for text in piece_lines)
            else:
                for piece in split_result.pieces:
                    lines.append(
                        f"  P{piece.piece_index}: type {piece.mission_type} / "
                        f"vertices {len((piece.data or {}).get('coordinateList', []))}"
                    )
            self._set_result("\n".join(lines))
            self._refresh_ui()
        except Exception as exc:
            self._clear_plan_result()
            self.state.mode = MODE_MISSION_READY
            self._set_result(traceback.format_exc())
            QMessageBox.critical(self, "영역 분할 실패", str(exc))
            self._refresh_ui()

    def _generate_mid_lines(self) -> None:
        if self.state.split_result is None or not self.state.split_result.pieces:
            QMessageBox.warning(self, "Mid Line Generation", "癒쇱? Area Division Run???ㅽ뻾??二쇱꽭??")
            return

        try:
            _reference_bearing_deg, overlays, lines = self._mid_line_overlay_bundle(self.state.split_result)
            self.state.mid_line_segments = overlays
            self.state.tangent_checks = []
            self._append_result("\n".join(lines))
            self._refresh_ui()
        except Exception as exc:
            self._append_result(traceback.format_exc())
            QMessageBox.critical(self, "Mid Line Generation", str(exc))
            self._refresh_ui()

    def _generate_mid_lines_2(self) -> None:
        if self.state.split_result is None or not self.state.split_result.pieces:
            QMessageBox.warning(self, "Mid Line Generation 2", "먼저 Area Division Run을 실행해 주세요.")
            return
        if not self.state.mid_line_segments:
            QMessageBox.warning(self, "Mid Line Generation 2", "먼저 Mid Line Generation을 실행해 주세요.")
            return
        if not self._has_make_new_area_result():
            QMessageBox.warning(self, "Mid Line Generation 2", "먼저 Make New Area를 실행해 주세요.")
            return

        try:
            overlay_rows: List[Dict[str, Any]] = []
            lines = ["[MID2] split-part far-face generation"]
            for row in self.state.mid_line_segments:
                if not isinstance(row, dict):
                    continue
                overlay = dict(row)
                aid = int(overlay.get("aircraftID", 0) or 0)
                piece_index = int(overlay.get("pieceIndex", 0) or 0)
                uav_state = self._uav_state_for_aircraft(aid)
                origin_xy = uav_state[0] if uav_state is not None else None
                bearing_deg = float(overlay.get("bearingDeg", 0.0) or 0.0)
                split_parts = self._resolved_split_parts_for_overlay(overlay)
                if split_parts:
                    overlay["splitParts"] = split_parts
                centers: List[Dict[str, Any]] = []
                if isinstance(split_parts, list) and split_parts:
                    for idx, part in enumerate(split_parts, start=1):
                        if not isinstance(part, dict):
                            continue
                        name = str(part.get("name", "") or chr(ord("A") + idx - 1))
                        polygon_xy = part.get("polygonXY")
                        center_xy = None
                        line_xy = None
                        shape_center_xy = None
                        part_width_m = 0.0
                        mid_line_length_m = 0.0
                        if isinstance(polygon_xy, list) and len(polygon_xy) >= 3:
                            try:
                                part_coords_xy = [
                                    (float(point_xy[0]), float(point_xy[1]))
                                    for point_xy in polygon_xy
                                    if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2
                                ]
                                if len(part_coords_xy) >= 3:
                                    part_overlay = self._mid_line_overlay_geometry(part_coords_xy, bearing_deg)
                                    if isinstance(part_overlay, dict):
                                        part_width_m = float(
                                            part_overlay.get("maxWidthM", 0.0)
                                            or part_overlay.get("widthM", 0.0)
                                            or 0.0
                                        )
                                    if origin_xy is not None:
                                        centerline_points = self._bearing_centerline_target_points(
                                            part_coords_xy,
                                            bearing_deg,
                                            origin_xy,
                                        )
                                        if isinstance(centerline_points, dict):
                                            raw_shape_center_xy = centerline_points.get("shapeCenterXY")
                                            if isinstance(raw_shape_center_xy, (tuple, list)) and len(raw_shape_center_xy) >= 2:
                                                shape_center_xy = (
                                                    float(raw_shape_center_xy[0]),
                                                    float(raw_shape_center_xy[1]),
                                                )
                                            target_xy = centerline_points.get("targetXY")
                                            if isinstance(target_xy, (tuple, list)) and len(target_xy) >= 2:
                                                center_xy = (
                                                    float(target_xy[0]),
                                                    float(target_xy[1]),
                                                )
                                            raw_line_xy = centerline_points.get("centerLineXY")
                                            if isinstance(raw_line_xy, list) and len(raw_line_xy) >= 2:
                                                line_xy = [
                                                    (float(line_point[0]), float(line_point[1]))
                                                    for line_point in raw_line_xy
                                                    if isinstance(line_point, (tuple, list)) and len(line_point) >= 2
                                                ]
                                                if len(line_xy) >= 2:
                                                    mid_line_length_m = _distance(line_xy[0], line_xy[-1])
                                    if center_xy is None:
                                        if part_overlay is not None and origin_xy is not None:
                                            face_points = self._make_path_face_points(part_overlay, origin_xy)
                                            if isinstance(face_points, dict):
                                                target_face_xy = face_points.get("targetFaceXY")
                                                if isinstance(target_face_xy, (tuple, list)) and len(target_face_xy) >= 2:
                                                    center_xy = (
                                                        float(target_face_xy[0]),
                                                        float(target_face_xy[1]),
                                                    )
                                                target_face_line_xy = face_points.get("targetFaceLineXY")
                                                if isinstance(target_face_line_xy, list) and len(target_face_line_xy) >= 2:
                                                    line_xy = [
                                                        (float(line_point[0]), float(line_point[1]))
                                                        for line_point in target_face_line_xy
                                                        if isinstance(line_point, (tuple, list)) and len(line_point) >= 2
                                                    ]
                                                    if len(line_xy) >= 2:
                                                        mid_line_length_m = _distance(line_xy[0], line_xy[-1])
                            except Exception:
                                center_xy = None
                                line_xy = None
                        if not (isinstance(center_xy, tuple) and len(center_xy) >= 2):
                            raw_center_xy = part.get("centroidXY")
                            if isinstance(raw_center_xy, (tuple, list)) and len(raw_center_xy) >= 2:
                                shape_center_xy = (
                                    float(raw_center_xy[0]),
                                    float(raw_center_xy[1]),
                                )
                                center_xy = (float(raw_center_xy[0]), float(raw_center_xy[1]))
                        if not (isinstance(center_xy, tuple) and len(center_xy) >= 2):
                            if isinstance(polygon_xy, list) and len(polygon_xy) >= 3:
                                try:
                                    part_poly = Polygon(
                                        [
                                            (float(point_xy[0]), float(point_xy[1]))
                                            for point_xy in polygon_xy
                                            if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2
                                        ]
                                    ).buffer(0)
                                    if isinstance(part_poly, MultiPolygon):
                                        geoms = [
                                            geom
                                            for geom in part_poly.geoms
                                            if isinstance(geom, Polygon) and not geom.is_empty
                                        ]
                                        part_poly = max(geoms, key=lambda geom: float(geom.area)) if geoms else part_poly
                                    if not part_poly.is_empty:
                                        point = part_poly.representative_point()
                                        shape_center_xy = (float(point.x), float(point.y))
                                        center_xy = (float(point.x), float(point.y))
                                except Exception:
                                    center_xy = None
                        if shape_center_xy is None:
                            raw_center_xy = part.get("centroidXY")
                            if isinstance(raw_center_xy, (tuple, list)) and len(raw_center_xy) >= 2:
                                shape_center_xy = (
                                    float(raw_center_xy[0]),
                                    float(raw_center_xy[1]),
                                )
                        if shape_center_xy is None and isinstance(center_xy, tuple) and len(center_xy) >= 2:
                            shape_center_xy = center_xy
                        if not (isinstance(center_xy, tuple) and len(center_xy) >= 2):
                            continue
                        centers.append(
                            {
                                "name": name,
                                "centerXY": (float(center_xy[0]), float(center_xy[1])),
                                "label": f"F{name}",
                                "lineXY": line_xy,
                                "shapeCenterXY": shape_center_xy,
                                "polygonXY": [
                                    (float(point_xy[0]), float(point_xy[1]))
                                    for point_xy in polygon_xy
                                    if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2
                                ] if isinstance(polygon_xy, list) else [],
                                "widthM": float(part_width_m),
                                "midLineLengthM": float(mid_line_length_m),
                                "bearingDeg": float(bearing_deg),
                            }
                        )
                    existing_names = {
                        str(center.get("name", "") or "")
                        for center in centers
                        if isinstance(center, dict)
                    }
                    for fallback_part in split_parts:
                        if not isinstance(fallback_part, dict):
                            continue
                        fallback_name = str(fallback_part.get("name", "") or "")
                        if fallback_name in existing_names:
                            continue
                        polygon_xy = fallback_part.get("polygonXY")
                        if not isinstance(polygon_xy, list) or len(polygon_xy) < 3:
                            continue
                        fallback_center_xy = None
                        raw_center_xy = fallback_part.get("centroidXY")
                        if isinstance(raw_center_xy, (tuple, list)) and len(raw_center_xy) >= 2:
                            fallback_center_xy = (float(raw_center_xy[0]), float(raw_center_xy[1]))
                        if fallback_center_xy is None:
                            try:
                                fallback_poly = self._largest_polygon_xy(
                                    Polygon(
                                        [
                                            (float(point_xy[0]), float(point_xy[1]))
                                            for point_xy in polygon_xy
                                            if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2
                                        ]
                                    ).buffer(0)
                                )
                                if fallback_poly is not None and not fallback_poly.is_empty:
                                    point = fallback_poly.representative_point()
                                    fallback_center_xy = (float(point.x), float(point.y))
                            except Exception:
                                fallback_center_xy = None
                        if fallback_center_xy is None:
                            continue
                        centers.append(
                            {
                                "name": fallback_name,
                                "centerXY": fallback_center_xy,
                                "label": f"F{fallback_name}",
                                "lineXY": None,
                                "shapeCenterXY": fallback_center_xy,
                                "polygonXY": [
                                    (float(point_xy[0]), float(point_xy[1]))
                                    for point_xy in polygon_xy
                                    if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2
                                ],
                                "widthM": 0.0,
                                "midLineLengthM": 0.0,
                                "bearingDeg": float(bearing_deg),
                            }
                        )
                else:
                    center_xy = None
                    line_xy = None
                    shape_center_xy = None
                    piece_width_m = 0.0
                    mid_line_length_m = 0.0
                    piece_coords_xy: List[Tuple[float, float]] = []
                    if origin_xy is not None:
                        piece = next(
                            (
                                piece_row
                                for piece_row in (self.state.split_result.pieces if self.state.split_result is not None else [])
                                if int(piece_row.piece_index or 0) == piece_index
                            ),
                            None,
                        )
                        piece_poly = self._piece_polygon_xy(piece) if piece is not None else None
                        if piece_poly is not None and not piece_poly.is_empty:
                            point = piece_poly.representative_point()
                            shape_center_xy = (float(point.x), float(point.y))
                            piece_coords_xy = [
                                (float(x), float(y))
                                for (x, y) in list(piece_poly.exterior.coords)[:-1]
                            ]
                            piece_overlay = self._mid_line_overlay_geometry(piece_coords_xy, bearing_deg)
                            if isinstance(piece_overlay, dict):
                                piece_width_m = float(
                                    piece_overlay.get("maxWidthM", 0.0)
                                    or piece_overlay.get("widthM", 0.0)
                                    or 0.0
                                )
                            centerline_points = self._bearing_centerline_target_points(
                                piece_coords_xy,
                                bearing_deg,
                                origin_xy,
                            )
                            if isinstance(centerline_points, dict):
                                raw_shape_center_xy = centerline_points.get("shapeCenterXY")
                                if isinstance(raw_shape_center_xy, (tuple, list)) and len(raw_shape_center_xy) >= 2:
                                    shape_center_xy = (
                                        float(raw_shape_center_xy[0]),
                                        float(raw_shape_center_xy[1]),
                                    )
                                target_xy = centerline_points.get("targetXY")
                                if isinstance(target_xy, (tuple, list)) and len(target_xy) >= 2:
                                    center_xy = (
                                        float(target_xy[0]),
                                        float(target_xy[1]),
                                    )
                                raw_line_xy = centerline_points.get("centerLineXY")
                                if isinstance(raw_line_xy, list) and len(raw_line_xy) >= 2:
                                    line_xy = [
                                        (float(line_point[0]), float(line_point[1]))
                                        for line_point in raw_line_xy
                                        if isinstance(line_point, (tuple, list)) and len(line_point) >= 2
                                    ]
                                    if len(line_xy) >= 2:
                                        mid_line_length_m = _distance(line_xy[0], line_xy[-1])
                        if center_xy is None:
                            face_points = self._make_path_face_points(overlay, origin_xy)
                            if isinstance(face_points, dict):
                                target_face_xy = face_points.get("targetFaceXY")
                                if isinstance(target_face_xy, (tuple, list)) and len(target_face_xy) >= 2:
                                    center_xy = (
                                        float(target_face_xy[0]),
                                        float(target_face_xy[1]),
                                    )
                                target_face_line_xy = face_points.get("targetFaceLineXY")
                                if isinstance(target_face_line_xy, list) and len(target_face_line_xy) >= 2:
                                    line_xy = [
                                        (float(point_xy[0]), float(point_xy[1]))
                                    for point_xy in target_face_line_xy
                                    if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2
                                ]
                                    if len(line_xy) >= 2:
                                        mid_line_length_m = _distance(line_xy[0], line_xy[-1])
                    if center_xy is None:
                        raw_center_xy = overlay.get("centerXY")
                        if isinstance(raw_center_xy, (tuple, list)) and len(raw_center_xy) >= 2:
                            center_xy = (float(raw_center_xy[0]), float(raw_center_xy[1]))
                    if shape_center_xy is None:
                        raw_center_xy = overlay.get("centerXY")
                        if isinstance(raw_center_xy, (tuple, list)) and len(raw_center_xy) >= 2:
                            shape_center_xy = (float(raw_center_xy[0]), float(raw_center_xy[1]))
                    if isinstance(center_xy, tuple) and len(center_xy) >= 2:
                        centers.append(
                            {
                                "name": "P",
                                "centerXY": (float(center_xy[0]), float(center_xy[1])),
                                "label": "FP",
                                "lineXY": line_xy,
                                "shapeCenterXY": shape_center_xy,
                                "polygonXY": piece_coords_xy,
                                "widthM": float(piece_width_m),
                                "midLineLengthM": float(mid_line_length_m),
                                "bearingDeg": float(bearing_deg),
                            }
                        )

                centers.sort(key=lambda center: str(center.get("name", "") or str(center.get("label", "") or "")))
                overlay["stage2Centers"] = centers
                overlay["midLineStage2Applied"] = True
                if centers:
                    label_text = ", ".join(
                        f"{str(center.get('label', '?'))}=({float(center['centerXY'][0]):.1f}, {float(center['centerXY'][1]):.1f})"
                        for center in centers
                        if isinstance(center, dict) and isinstance(center.get("centerXY"), (tuple, list)) and len(center.get("centerXY")) >= 2
                    )
                    lines.append(
                        f"  P{piece_index} / UAV{aid}: {label_text}"
                    )
                else:
                    lines.append(f"  P{piece_index} / UAV{aid}: 작은 도형 반대면 중심 없음")
                overlay_rows.append(overlay)

            self.state.mid_line_segments = overlay_rows
            self.state.expected_paths = []
            self.state.assignment_path_rows = []
            self.state.tangent_checks = []
            self._append_result("\n".join(lines))
            self._refresh_ui()
        except Exception as exc:
            self._append_result(traceback.format_exc())
            QMessageBox.critical(self, "Mid Line Generation 2", str(exc))
            self._refresh_ui()

    def _build_tangent_check_rows(self) -> Tuple[List[Dict[str, Any]], List[str]]:
        tangent_rows: List[Dict[str, Any]] = []
        lines = [
            "[TAN] check tangent points",
            "rule: Mid Line Generation 2 target x all UAV turn circles (L/R) x two tangent points",
        ]

        for row in self.state.mid_line_segments:
            if not isinstance(row, dict):
                continue
            piece_index = int(row.get("pieceIndex", 0) or 0)
            stage2_centers = row.get("stage2Centers")
            if not isinstance(stage2_centers, list) or not stage2_centers:
                continue

            for center_row in stage2_centers:
                if not isinstance(center_row, dict):
                    continue
                target_xy = center_row.get("centerXY")
                if not (isinstance(target_xy, (tuple, list)) and len(target_xy) >= 2):
                    continue
                target_point_xy = (float(target_xy[0]), float(target_xy[1]))
                target_label = str(center_row.get("label", "") or f"P{piece_index}")
                shape_center_xy = center_row.get("shapeCenterXY")
                shape_center_val = None
                if isinstance(shape_center_xy, (tuple, list)) and len(shape_center_xy) >= 2:
                    shape_center_val = (float(shape_center_xy[0]), float(shape_center_xy[1]))
                center_line_xy_raw = center_row.get("lineXY")
                center_line_xy: List[Tuple[float, float]] = []
                if isinstance(center_line_xy_raw, list):
                    for point_xy in center_line_xy_raw:
                        if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2:
                            center_line_xy.append((float(point_xy[0]), float(point_xy[1])))
                polygon_xy_raw = center_row.get("polygonXY")
                part_polygon_xy: List[Tuple[float, float]] = []
                if isinstance(polygon_xy_raw, list):
                    for point_xy in polygon_xy_raw:
                        if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2:
                            part_polygon_xy.append((float(point_xy[0]), float(point_xy[1])))
                try:
                    part_width_m = float(center_row.get("widthM", 0.0) or 0.0)
                except Exception:
                    part_width_m = 0.0
                try:
                    mid_line_length_m = float(center_row.get("midLineLengthM", 0.0) or 0.0)
                except Exception:
                    mid_line_length_m = 0.0
                try:
                    center_bearing_deg = float(center_row.get("bearingDeg", row.get("bearingDeg", 0.0) or 0.0) or 0.0)
                except Exception:
                    center_bearing_deg = 0.0
                target_tokens: List[str] = []

                for aid in self.state.uav_ids:
                    uav_state = self._uav_state_for_aircraft(int(aid))
                    if uav_state is None:
                        target_tokens.append(f"UAV{int(aid)}:missing")
                        continue
                    origin_xy, heading_deg = uav_state
                    left_center_xy, right_center_xy = self._turn_circle_centers_xy(origin_xy, heading_deg)
                    for branch, circle_center_xy in (("L", left_center_xy), ("R", right_center_xy)):
                        tangent_points = self._circle_tangent_points_xy(
                            circle_center_xy,
                            target_point_xy,
                        )
                        valid_tangent_rows: List[Tuple[Tuple[float, float], float]] = []
                        for tangent_xy in tangent_points:
                            tangent_point_xy = (float(tangent_xy[0]), float(tangent_xy[1]))
                            if not self._branch_forward_tangent(
                                circle_center_xy,
                                tangent_point_xy,
                                target_point_xy,
                                branch=branch,
                            ):
                                continue
                            horizon_s = self._arc_horizon_to_point_on_branch(
                                origin_xy,
                                circle_center_xy,
                                tangent_point_xy,
                                branch=branch,
                            )
                            valid_tangent_rows.append((tangent_point_xy, float(horizon_s)))
                        if not valid_tangent_rows:
                            target_tokens.append(f"UAV{int(aid)}{branch}:0")
                            continue
                        target_tokens.append(f"UAV{int(aid)}{branch}:{len(valid_tangent_rows)}")
                        for tangent_idx, (tangent_xy, horizon_s) in enumerate(valid_tangent_rows, start=1):
                            tangent_rows.append(
                                {
                                    "pieceIndex": int(piece_index),
                                    "targetLabel": target_label,
                                    "targetXY": target_point_xy,
                                    "shapeCenterXY": shape_center_val,
                                    "centerLineXY": center_line_xy,
                                    "partPolygonXY": part_polygon_xy,
                                    "partWidthM": float(part_width_m),
                                    "midLineLengthM": float(mid_line_length_m),
                                    "bearingDeg": float(center_bearing_deg),
                                    "aircraftID": int(aid),
                                    "branch": branch,
                                    "circleCenterXY": (
                                        float(circle_center_xy[0]),
                                        float(circle_center_xy[1]),
                                    ),
                                    "tangentIndex": int(tangent_idx),
                                    "tangentXY": (
                                        float(tangent_xy[0]),
                                        float(tangent_xy[1]),
                                    ),
                                    "horizonSec": float(horizon_s),
                                    "label": f"{int(aid)}{branch}{int(tangent_idx)}",
                                }
                            )

                if target_tokens:
                    lines.append(f"  P{piece_index} / {target_label}: " + ", ".join(target_tokens))

        if not tangent_rows:
            lines.append("  tangent point 없음")
        return tangent_rows, lines

    def _assignment_path_1(self) -> None:
        if self.state.split_result is None or not self.state.split_result.pieces:
            QMessageBox.warning(self, "Assignment - Path 1", "먼저 Area Division Run을 실행해 주세요.")
            return
        if not self.state.mid_line_segments:
            QMessageBox.warning(self, "Assignment - Path 1", "먼저 Mid Line Generation을 실행해 주세요.")
            return

        try:
            tangent_rows, tangent_lines = self._build_tangent_check_rows()
            self.state.tangent_checks = tangent_rows
            candidate_by_uav_target: Dict[Tuple[int, str], Dict[str, Any]] = {}
            active_uavs: List[int] = []
            target_keys: List[str] = []
            lines = [
                "[AP1] assign first target by tangent-entry angle",
                "rule: weighted score = angle*1 + turnTime*2",
            ]
            if tangent_lines:
                lines.extend(tangent_lines)

            for aid in self.state.uav_ids:
                if self._uav_state_for_aircraft(int(aid)) is not None:
                    active_uavs.append(int(aid))

            seen_target_keys: set[str] = set()
            for row in tangent_rows:
                if not isinstance(row, dict):
                    continue
                aid = int(row.get("aircraftID", 0) or 0)
                target_xy = row.get("targetXY")
                tangent_xy = row.get("tangentXY")
                shape_center_xy = row.get("shapeCenterXY")
                if not (
                    isinstance(target_xy, (tuple, list))
                    and len(target_xy) >= 2
                    and isinstance(tangent_xy, (tuple, list))
                    and len(tangent_xy) >= 2
                    and isinstance(shape_center_xy, (tuple, list))
                    and len(shape_center_xy) >= 2
                ):
                    continue

                target_point_xy = (float(target_xy[0]), float(target_xy[1]))
                tangent_point_xy = (float(tangent_xy[0]), float(tangent_xy[1]))
                shape_center_point_xy = (float(shape_center_xy[0]), float(shape_center_xy[1]))
                angle_deg = _undirected_angle_deg(
                    target_point_xy,
                    shape_center_point_xy,
                    tangent_point_xy,
                )
                if angle_deg is None:
                    continue

                piece_index = int(row.get("pieceIndex", 0) or 0)
                target_label = str(row.get("targetLabel", "") or f"P{piece_index}")
                target_key = f"P{piece_index}:{target_label}"
                candidate = dict(row)
                candidate["targetKey"] = target_key
                candidate["entryAngleDeg"] = float(angle_deg)
                candidate["segmentLenM"] = _distance(tangent_point_xy, target_point_xy)
                candidate["selectionScore"] = (
                    (float(candidate["entryAngleDeg"]) * float(ASSIGNMENT_PATH1_ANGLE_WEIGHT))
                    + (float(candidate.get("horizonSec", 0.0) or 0.0) * float(ASSIGNMENT_PATH1_TURN_TIME_WEIGHT))
                )

                score_key = (
                    float(candidate["selectionScore"]),
                    float(candidate.get("horizonSec", 0.0) or 0.0),
                    float(candidate["segmentLenM"]),
                )
                existing = candidate_by_uav_target.get((aid, target_key))
                if existing is None:
                    candidate_by_uav_target[(aid, target_key)] = candidate
                else:
                    existing_key = (
                        float(existing.get("selectionScore", 0.0) or 0.0),
                        float(existing.get("horizonSec", 0.0) or 0.0),
                        float(existing.get("segmentLenM", 0.0) or 0.0),
                    )
                    if score_key < existing_key:
                        candidate_by_uav_target[(aid, target_key)] = candidate
                if target_key not in seen_target_keys:
                    seen_target_keys.add(target_key)
                    target_keys.append(target_key)

            if not candidate_by_uav_target or not active_uavs or not target_keys:
                lines.append("  usable tangent candidate 없음")
                self.state.assignment_path_rows = []
                self._append_result("\n".join(lines))
                self._refresh_ui()
                return

            assign_count = min(len(active_uavs), len(target_keys))
            best_records: List[Dict[str, Any]] = []
            best_score: Optional[Tuple[float, float, float]] = None

            for uav_subset in itertools.combinations(active_uavs, assign_count):
                for target_perm in itertools.permutations(target_keys, assign_count):
                    candidate_rows: List[Dict[str, Any]] = []
                    weighted_sum = 0.0
                    horizon_sum = 0.0
                    segment_sum = 0.0
                    valid = True
                    for aid, target_key in zip(uav_subset, target_perm):
                        candidate = candidate_by_uav_target.get((int(aid), str(target_key)))
                        if candidate is None:
                            valid = False
                            break
                        candidate_rows.append(candidate)
                        weighted_sum += float(candidate.get("selectionScore", 0.0) or 0.0)
                        horizon_sum += float(candidate.get("horizonSec", 0.0) or 0.0)
                        segment_sum += float(candidate.get("segmentLenM", 0.0) or 0.0)
                    if not valid:
                        continue
                    score = (float(weighted_sum), float(horizon_sum), float(segment_sum))
                    if best_score is None or score < best_score:
                        best_score = score
                        best_records = candidate_rows

            if not best_records:
                lines.append("  중복 없는 배정 조합을 만들지 못함")
                self.state.assignment_path_rows = []
                self._append_result("\n".join(lines))
                self._refresh_ui()
                return

            path_rows: List[Dict[str, Any]] = []
            assigned_uavs = {int(row.get("aircraftID", 0) or 0) for row in best_records}
            for candidate in sorted(best_records, key=lambda row: int(row.get("aircraftID", 0) or 0)):
                path_row = self._build_assignment_path_1_row(candidate)
                if path_row is not None:
                    path_rows.append(path_row)
                eta_total_sec = float(path_row.get("estimatedTotalSec", 0.0) or 0.0) if isinstance(path_row, dict) else 0.0
                eta_tangent_sec = float(candidate.get("horizonSec", 0.0) or 0.0)
                lines.append(
                    f"  UAV{int(candidate.get('aircraftID', 0) or 0)}"
                    f" -> P{int(candidate.get('pieceIndex', 0) or 0)} / {str(candidate.get('targetLabel', '') or '?')}"
                    f" | {str(candidate.get('branch', '') or '')}{int(candidate.get('tangentIndex', 0) or 0)}"
                    f" | angle {float(candidate.get('entryAngleDeg', 0.0) or 0.0):.1f}deg"
                    f" | +{float(candidate.get('horizonSec', 0.0) or 0.0):.1f}s"
                    f" | ETA T {eta_tangent_sec:.1f}s"
                    f" | ETA WP {eta_total_sec:.1f}s"
                    f" | score {float(candidate.get('selectionScore', 0.0) or 0.0):.1f}"
                )

            for aid in active_uavs:
                if int(aid) not in assigned_uavs:
                    lines.append(f"  UAV{int(aid)} -> 미배정")

            self.state.expected_paths = path_rows
            self.state.assignment_path_rows = [dict(row) for row in path_rows]
            self.state.visibility_segments = []
            self.state.show_turn_overlays = True
            self._append_result("\n".join(lines))
            self._refresh_ui()
        except Exception as exc:
            self._append_result(traceback.format_exc())
            QMessageBox.critical(self, "Assignment - Path 1", str(exc))
            self._refresh_ui()

    def _make_new_area(self) -> None:
        if self.state.split_result is None or not self.state.split_result.pieces:
            QMessageBox.warning(self, "Make New Area", "먼저 Area Division Run을 실행해 주세요.")
            return
        if not self.state.mid_line_segments:
            QMessageBox.warning(self, "Make New Area", "먼저 Mid Line Generation을 실행해 주세요.")
            return

        try:
            overlay_by_piece = {
                int(row.get("pieceIndex", 0) or 0): row
                for row in self.state.mid_line_segments
                if isinstance(row, dict)
            }
            changed_count = 0
            lines = ["[NEW AREA] split parts -> oriented boxes"]

            for piece in sorted(self.state.split_result.pieces, key=lambda row: int(row.piece_index or 0)):
                overlay = overlay_by_piece.get(int(piece.piece_index or 0))
                if overlay is None:
                    aid = int(piece.assigned_uav or 0)
                    lines.append(
                        f"  P{int(piece.piece_index or 0)}"
                        f"{f' / UAV{aid}' if aid > 0 else ''}: no mid-line box"
                    )
                    continue
                aid = int(piece.assigned_uav or 0)
                new_poly, info = self._make_new_area_polygon(piece, overlay, None)
                part_results = info.get("partResults")
                part_summary = ""
                if isinstance(part_results, list) and part_results:
                    tokens: List[str] = []
                    for row in part_results:
                        if not isinstance(row, dict):
                            continue
                        name = str(row.get("name", "") or "?")
                        reason = str(row.get("reason", "") or "")
                        old_part_area = float(row.get("oldAreaM2", 0.0) or 0.0)
                        new_part_area = float(row.get("newAreaM2", old_part_area) or old_part_area)
                        if new_part_area > 0.0 or old_part_area > 0.0:
                            tokens.append(f"{name}:{reason}({old_part_area:.0f}->{new_part_area:.0f})")
                        else:
                            tokens.append(f"{name}:{reason}")
                    if tokens:
                        part_summary = " [" + ", ".join(tokens) + "]"
                area_text = (
                    f"(area {float(info.get('oldAreaM2', 0.0)):.0f}"
                    f"->{float(info.get('newAreaM2', info.get('oldAreaM2', 0.0))):.0f}m2)"
                )
                if new_poly is None:
                    lines.append(
                        f"  P{int(piece.piece_index or 0)}"
                        f"{f' / UAV{aid}' if aid > 0 else ''}: keep "
                        f"{str(info.get('reason', 'unchanged'))} {area_text}{part_summary}"
                    )
                    continue
                self._replace_piece_polygon(piece, new_poly, review_patch=info)
                changed_count += 1
                lines.append(
                    f"  P{int(piece.piece_index or 0)}"
                    f"{f' / UAV{aid}' if aid > 0 else ''}: boxed {area_text}{part_summary}"
                )

            if changed_count <= 0:
                lines.append("  no area changed")

            _reference_bearing_deg, overlays, _mid_lines = self._mid_line_overlay_bundle(self.state.split_result)
            self.state.mid_line_segments = overlays
            self.state.expected_paths = []
            self.state.assignment_path_rows = []
            self.state.flight_plans_0303 = []
            self.state.flight_plans_0304 = []
            self.state.visibility_segments = []
            self.state.tangent_checks = []
            self.state.show_turn_overlays = True
            self.state.mode = MODE_RESULT_READY
            self._append_result("\n".join(lines))
            self._refresh_ui()
        except Exception as exc:
            self._append_result(traceback.format_exc())
            QMessageBox.critical(self, "Make New Area", str(exc))
            self._refresh_ui()

    def _make_waypoint(self) -> None:
        if not self.state.assignment_path_rows:
            QMessageBox.warning(self, "Make Waypoint", "먼저 Assignment - Path 1을 실행해 주세요.")
            return

        try:
            expected_rows: List[Dict[str, Any]] = []
            lines = ["[WP] make waypoint from tangent + sep + mid-line-2 length"]
            for base_row in sorted(
                self.state.assignment_path_rows,
                key=lambda row: (
                    int(row.get("aircraftID", 0) or 0),
                    int(row.get("pieceIndex", 0) or 0),
                ),
            ):
                if not isinstance(base_row, dict):
                    continue
                waypoint_row = self._build_make_waypoint_row(base_row)
                if waypoint_row is None:
                    lines.append(
                        f"  UAV{int(base_row.get('aircraftID', 0) or 0)}"
                        f" -> P{int(base_row.get('pieceIndex', 0) or 0)}: waypoint build failed"
                    )
                    continue

                expected_rows.append(waypoint_row)
                lines.append(
                    f"  UAV{int(waypoint_row.get('aircraftID', 0) or 0)}"
                    f" -> P{int(waypoint_row.get('pieceIndex', 0) or 0)} / {str(waypoint_row.get('targetLabel', '') or '?')}"
                    f" | width {float(waypoint_row.get('shapeWidthM', 0.0) or 0.0):.0f}m"
                    f" | sep {float(waypoint_row.get('dbSepM', 0.0) or 0.0):.0f}m"
                    f" | entry {float(waypoint_row.get('entryDistanceM', 0.0) or 0.0):.0f}m"
                    f" | start {str(waypoint_row.get('startLabel', 'T') or 'T')}"
                    f" | mid {float(waypoint_row.get('midLineLengthM', 0.0) or 0.0):.0f}m"
                    f" | ETA WP {float(waypoint_row.get('estimatedTotalSec', 0.0) or 0.0):.1f}s"
                )

            if not expected_rows:
                lines.append("  usable waypoint row 없음")
                self._append_result("\n".join(lines))
                self._refresh_ui()
                return

            self.state.expected_paths = expected_rows
            self.state.visibility_segments = []
            self.state.tangent_checks = []
            self.state.show_turn_overlays = True
            self.state.mode = MODE_RESULT_READY
            self._append_result("\n".join(lines))
            self._refresh_ui()
        except Exception as exc:
            self._append_result(traceback.format_exc())
            QMessageBox.critical(self, "Make Waypoint", str(exc))
            self._refresh_ui()

    def _make_path(self) -> None:
        if self.state.split_result is None or not self.state.split_result.pieces:
            QMessageBox.warning(self, "Make Path", "먼저 Area Division Run을 실행해 주세요.")
            return
        if not self.state.mid_line_segments:
            QMessageBox.warning(self, "Make Path", "먼저 Mid Line Generation을 실행해 주세요.")
            return

        try:
            overlay_by_piece = {
                int(row.get("pieceIndex", 0) or 0): row
                for row in self.state.mid_line_segments
                if isinstance(row, dict)
            }
            expected_rows: List[Dict[str, Any]] = []
            lines = ["[PATH] make path from tangent + face midpoint + sweep"]
            for piece in sorted(self.state.split_result.pieces, key=lambda row: int(row.piece_index or 0)):
                aid = int(piece.assigned_uav or 0)
                overlay = overlay_by_piece.get(int(piece.piece_index or 0))
                if overlay is None:
                    lines.append(
                        f"  P{int(piece.piece_index or 0)}"
                        f"{f' / UAV{aid}' if aid > 0 else ''}: no mid-line overlay"
                    )
                    continue
                path_row = self._make_path_row(piece, overlay)
                if path_row is None:
                    lines.append(
                        f"  P{int(piece.piece_index or 0)}"
                        f"{f' / UAV{aid}' if aid > 0 else ''}: path build failed"
                    )
                    continue
                expected_rows.append(path_row)
                lines.append(
                    f"  P{int(piece.piece_index or 0)}"
                    f"{f' / UAV{aid}' if aid > 0 else ''}: "
                    f"{str(path_row.get('branch', ''))} "
                    f"+{float(path_row.get('horizonSec', 0.0) or 0.0):.1f}s "
                    f"| sweep={len(path_row.get('sweepLineListXY', []))} "
                    f"| DB width {float(path_row.get('dbWidthM', 0.0) or 0.0):.0f}m "
                    f"sep {float(path_row.get('dbSepM', 0.0) or 0.0):.0f}m"
                )

            self.state.expected_paths = expected_rows
            self.state.visibility_segments = []
            self.state.tangent_checks = []
            self.state.flight_plans_0303 = []
            self.state.flight_plans_0304 = []
            self.state.show_turn_overlays = False
            self.state.mode = MODE_RESULT_READY
            self._append_result("\n".join(lines))
            self._refresh_ui()
        except Exception as exc:
            self._append_result(traceback.format_exc())
            QMessageBox.critical(self, "Make Path", str(exc))
            self._refresh_ui()

    def _run_stage2_area_division(self) -> None:
        if not self._ensure_ready_for_division(area_only=True):
            return
        if self.state.split_result is None or not self.state.split_result.pieces:
            QMessageBox.warning(self, "Stage 2 Area Division", "먼저 Area Division Run을 실행해 주세요.")
            return

        try:
            mission_poly = self._mission_polygon_xy()
            if mission_poly is None:
                raise ValueError("Stage 2 review requires a valid area mission polygon.")
            grid_size_m = float(self._stage2_grid_size_m(mission_poly))

            area_piece_lists: Dict[int, List[SplitPiece]] = {}
            for piece in self.state.split_result.pieces:
                if int(piece.mission_type) not in {2, 3, 6}:
                    continue
                aid = int(piece.assigned_uav or 0)
                if aid <= 0:
                    continue
                area_piece_lists.setdefault(int(aid), []).append(piece)

            if not area_piece_lists:
                raise ValueError("Stage 2 review requires assigned area pieces.")

            multi_piece_uavs = [aid for aid, rows in sorted(area_piece_lists.items()) if len(rows) != 1]
            if multi_piece_uavs:
                raise ValueError(
                    "Stage 2 review currently supports one assigned area per UAV. "
                    f"Multiple pieces found for: {', '.join(f'UAV{aid}' for aid in multi_piece_uavs)}"
                )

            active_uav_ids = sorted(int(aid) for aid in area_piece_lists.keys())
            piece_by_aid = {int(aid): rows[0] for aid, rows in area_piece_lists.items()}
            ratio_map = self._stage2_ratio_map(active_uav_ids)
            if sum(float(v) for v in ratio_map.values()) <= 1e-9:
                raise ValueError("At least one Stage 2 ratio must be greater than 0.")

            target_area_map, stats_by_aid = self._stage2_target_area_map(
                self.state.split_result,
                piece_by_aid,
                ratio_map,
                grid_size_m=grid_size_m,
            )
            polygon_by_aid, achieved_area_map = self._stage2_rebalance_area_polygons(
                mission_poly,
                piece_by_aid,
                target_area_map,
                grid_size_m=grid_size_m,
            )
            self._stage2_apply_review_polygons(
                piece_by_aid,
                polygon_by_aid,
                ratio_map,
                target_area_map,
                grid_size_m=grid_size_m,
            )

            self.state.visibility_segments = []
            self.state.expected_paths = []
            self.state.assignment_path_rows = []
            self.state.flight_plans_0303 = []
            self.state.flight_plans_0304 = []
            self.state.show_turn_overlays = True
            self.state.mid_line_segments = []
            self.state.tangent_checks = []
            self.state.mode = MODE_RESULT_READY

            total_area_m2 = sum(float(v) for v in achieved_area_map.values())
            ratio_text = ", ".join(f"UAV{aid}={float(ratio_map.get(aid, 0.0)):.2f}" for aid in active_uav_ids)
            lines = [
                "[1b] stage2-area-review "
                f"grid={grid_size_m:.0f}m "
                f"ratios=({ratio_text}) "
                f"assignment={self._assignment_summary_text(self.state.split_result)}"
            ]
            for aid in active_uav_ids:
                meta = stats_by_aid[int(aid)]
                achieved_area_m2 = float(achieved_area_map.get(int(aid), 0.0))
                area_rate_m2ps = max(float(meta.get("areaRateM2ps", 0.0) or 0.0), 1e-6)
                est_total_sec = float(meta.get("entrySec", 0.0) or 0.0) + (achieved_area_m2 / area_rate_m2ps)
                share_pct = (100.0 * achieved_area_m2 / total_area_m2) if total_area_m2 > 1e-6 else 0.0
                lines.append(
                    f"  UAV{aid}: "
                    f"time {float(meta.get('currentTotalSec', 0.0)):.1f}s -> "
                    f"target {float(meta.get('desiredTotalSec', 0.0)):.1f}s -> "
                    f"est {est_total_sec:.1f}s | "
                    f"area {float(meta.get('currentAreaM2', 0.0)):.0f} -> {achieved_area_m2:.0f} m2 "
                    f"(target {float(target_area_map.get(int(aid), 0.0)):.0f}, share {share_pct:.1f}%)"
                )
            self._append_result("\n".join(lines))
            self._refresh_ui()
        except Exception as exc:
            self._append_result(traceback.format_exc())
            QMessageBox.critical(self, "Stage 2 Area Division", str(exc))
            self._refresh_ui()

    def _check_visibility(self) -> None:
        if self.state.split_result is None or not self.state.split_result.pieces:
            QMessageBox.warning(self, "Visibility", "먼저 Area Division Run을 실행해 주세요.")
            return

        max_steps = max(
            1,
            int(
                math.ceil(
                    (2.0 * math.pi * TURN_PREVIEW_RADIUS_M)
                    / (TURN_PREVIEW_SPEED_MPS * TURN_PREVIEW_HORIZON_S)
                )
            ),
        )
        blocked_limit_s = int(max_steps * TURN_PREVIEW_HORIZON_S)

        visibility_segments: List[Dict[str, Any]] = []
        lines = [
            "[VIS] check visibility",
            f"rule: direct -> +{int(TURN_PREVIEW_HORIZON_S)}s -> +{int(TURN_PREVIEW_HORIZON_S * 2)}s ...",
        ]

        for piece in self.state.split_result.pieces:
            aid = int(piece.assigned_uav or 0)
            if aid <= 0:
                lines.append(f"  P{piece.piece_index}: assigned UAV 없음")
                continue

            uav_state = self._uav_state_for_aircraft(aid)
            if uav_state is None:
                lines.append(f"  P{piece.piece_index} / UAV{aid}: UAV 위치 또는 heading 없음")
                continue

            target_xy = self._piece_assignment_target_xy(piece)
            if target_xy is None:
                lines.append(f"  P{piece.piece_index} / UAV{aid}: centroid 계산 실패")
                continue

            origin_xy, heading_deg = uav_state
            segment = self._find_visibility_segment(aid, origin_xy, heading_deg, target_xy)
            if segment is None:
                lines.append(f"  P{piece.piece_index} / UAV{aid}: +{blocked_limit_s}s까지 직선 연결 불가")
                continue

            visibility_segments.append(segment)
            horizon_s = float(segment.get("horizonSec", 0.0) or 0.0)
            branch = str(segment.get("branch", "") or "")
            if horizon_s <= 0.0:
                lines.append(f"  P{piece.piece_index} / UAV{aid}: direct")
            else:
                lines.append(f"  P{piece.piece_index} / UAV{aid}: +{horizon_s:.1f}s {branch}")

        self.state.visibility_segments = visibility_segments
        self.state.show_turn_overlays = False
        self._append_result("\n".join(lines))
        self._refresh_ui()

    def _run_planning(self) -> None:
        if self.state.uav_ids and not self._uav_inputs_complete():
            QMessageBox.warning(self, "UAV 입력 부족", "모든 UAV의 위치와 heading을 입력해 주세요.")
            return
        if not self.state.mission_points_xy or self.state.mission_kind is None:
            QMessageBox.warning(self, "임무 없음", "임무 형상을 먼저 입력해 주세요.")
            return
        if not self.state.uav_ids:
            QMessageBox.warning(self, "UAV 없음", "UAV 대수를 먼저 확정해 주세요.")
            return
        if len(self.state.uav_positions_xy) != len(self.state.uav_ids):
            QMessageBox.warning(self, "UAV 위치 부족", "모든 UAV 위치를 입력해 주세요.")
            return

        try:
            use_replan_flow = self._is_replan_flow()
            reuse_stage2_review = self.state.mission_kind == MISSION_AREA and self._has_stage2_review(self.state.split_result)
            if reuse_stage2_review:
                self._cmpk_payload = self._build_cmpk_payload()
                self._mrpk_payload = self._build_mrpk_payload()
                split_result = copy.deepcopy(self.state.split_result)
                stage_lines = [
                    f"[1] flow={self._flow_mode()} split pieces={len(split_result.pieces)}",
                    f"[1b] stage2-reviewed split reused pieces={len(split_result.pieces)}",
                ]
            else:
                split_result, stage_lines = self._run_split_stage()

            type_report = apply_logic_type_decider(split_result, self._cmpk_payload, profile_code=PROFILE_DEFAULT)
            stage_lines.append(
                "[2] type-decider "
                f"changed={int(type_report.get('changedPieces', 0))}/{int(type_report.get('pieceCount', 0))}"
            )

            expected_paths = generate_expected_paths(split_result, self._mrpk_payload)
            split_result.expected_paths = list(expected_paths)
            stage_lines.append(f"[3] expected-path count={len(expected_paths)}")

            review_report: Optional[Dict[str, Any]] = None
            if reuse_stage2_review:
                split_result.expected_paths = [
                    row
                    for row in expected_paths
                    if isinstance(row, dict) and str(row.get("source", "")).startswith("line_center_offset_dir")
                ]
                stage_lines.append("[4] area-review skipped (stage2 reviewed)")
            elif self._area_mode() not in {"nadir", "directdown", "bf_nadir"}:
                if use_replan_flow:
                    assign_report = assign_split_result_by_takeover_distance(
                        split_result,
                        self._mrpk_payload,
                        list(self.state.uav_ids),
                    )
                    stage_lines.append(
                        "[3a] replan-preassign "
                        f"assigned={int(assign_report.get('assignedPieces', 0))}/"
                        f"{int(assign_report.get('pieceCount', 0))} "
                        f"{self._assignment_summary_text(split_result)}"
                    )
                    review_report = review_assigned_areas_local(
                        split_result,
                        self._mrpk_payload,
                        max_segment_m=self._review_max_segment_m(),
                    )
                else:
                    review_report = review_overflow_areas(
                        split_result,
                        expected_paths,
                        max_segment_m=self._review_max_segment_m(),
                    )
                line_paths = [
                    row
                    for row in expected_paths
                    if isinstance(row, dict) and str(row.get("source", "")).startswith("line_center_offset_dir")
                ]
                split_result.expected_paths = line_paths
                if use_replan_flow:
                    stage_lines.append(
                        "[4] replan-review "
                        f"localized={int(review_report.get('localized', 0))} "
                        f"targets={int(review_report.get('targets', 0))} "
                        f"pieces={int(review_report.get('oldPieceCount', len(split_result.pieces)))}->"
                        f"{int(review_report.get('newPieceCount', len(split_result.pieces)))}"
                    )
                else:
                    stage_lines.append(
                        "[4] area-review "
                        f"overflow={int(review_report.get('overflowRows', 0))} "
                        f"targets={int(review_report.get('targets', 0))} "
                        f"pieces={int(review_report.get('oldPieceCount', len(split_result.pieces)))}->"
                        f"{int(review_report.get('newPieceCount', len(split_result.pieces)))}"
                    )
            else:
                split_result.expected_paths = [
                    row
                    for row in expected_paths
                    if isinstance(row, dict) and str(row.get("source", "")).startswith("line_center_offset_dir")
                ]
                stage_lines.append("[4] area-review skipped (nadir mode)")

            vel_report = calculate_expected_velocity(
                split_result,
                expected_paths=split_result.expected_paths,
            )
            stage_lines.append(
                "[5] expected-velocity "
                f"pieces={int(vel_report.get('pieceCount', 0))} dbRows={int(vel_report.get('dbRowCount', 0))}"
            )

            sched_report = run_milp_scheduling(
                split_result,
                mrpk=self._mrpk_payload,
                uav_ids_override=list(self.state.uav_ids),
                respect_piece_assignment=use_replan_flow,
            )
            split_result.schedule_result = sched_report
            stage_lines.append(
                "[6] scheduling "
                f"status={str(sched_report.get('status', ''))} "
                f"gap={float(sched_report.get('balanceGapSec', 0.0) or 0.0):.1f}s "
                f"inserted={int(sched_report.get('insertedMissionCount', 0))}"
            )

            packages_0302 = build_0302_packages_from_split_with_lah(split_result, cmpk=self._cmpk_payload)
            _prepare_legacy_missionplanner_path()
            uav_plan_mode = self._uav_plan_mode()
            fp_0303, fp_0304 = build_0303_0304_from_0302_packages(
                packages_0302,
                mrpk=self._mrpk_payload,
                uav_plan_mode=uav_plan_mode,
            )

            out_root = self._output_root()
            out_0302 = out_root / "auto_0302"
            out_0303 = out_root / "auto_0303"
            out_0304 = out_root / "auto_0304"
            paths_0302 = save_0302_packages(packages_0302, out_0302)
            paths_0303 = save_0303_plans(fp_0303, out_0303)
            paths_0304 = save_0304_plans(fp_0304, out_0304)
            stage_lines.append(
                f"[7] 0302 files={len(paths_0302)} | 0303 files={len(paths_0303)} | 0304 files={len(paths_0304)}"
            )

            self.state.split_result = split_result
            self.state.expected_paths = list(split_result.expected_paths)
            self.state.flight_plans_0303 = list(fp_0303)
            self.state.flight_plans_0304 = list(fp_0304)
            self.state.mode = MODE_RESULT_READY

            lines = [f"Mission: {self.state.mission_kind}", f"UAV: {', '.join(f'UAV{aid}' for aid in self.state.uav_ids)}"]
            lines.extend(stage_lines)
            lines.extend(
                [
                    "",
                    f"Final split pieces: {len(split_result.pieces)}",
                    f"Cached expected paths: {len(self.state.expected_paths)}",
                    f"Actual 0303 paths: {len(fp_0303)}",
                    f"UAV plan mode: {uav_plan_mode}",
                ]
            )
            for piece in split_result.pieces:
                lines.append(
                    f"  P{piece.piece_index}: UAV{int(piece.assigned_uav or 0)} / "
                    f"type {piece.mission_type} / vertices {len((piece.data or {}).get('coordinateList', []))}"
                )
            lines.extend(
                [
                    "",
                    f"0302 output: {out_0302}",
                    f"0303 output: {out_0303}",
                    f"0304 output: {out_0304}",
                ]
            )
            self._set_result("\n".join(lines))
            self._refresh_ui()
        except Exception as exc:
            self._clear_plan_result()
            self.state.mode = MODE_MISSION_READY
            self._set_result(traceback.format_exc())
            QMessageBox.critical(self, "계획 실행 실패", str(exc))
            self._refresh_ui()

    def _build_cmpk_payload(self) -> Dict[str, Any]:
        if self.state.mission_kind == MISSION_AREA:
            mission = {
                "inputMissionID": 1,
                "inputMissionType": 2,
                "missionDetail": {
                    "areaList": [
                        {
                            "isHole": False,
                            "coordinateList": [meters_to_coord(x, y) for (x, y) in self.state.mission_points_xy],
                        }
                    ]
                },
            }
        else:
            line_coords = [meters_to_coord(x, y) for (x, y) in self.state.mission_points_xy]
            mission = {
                "inputMissionID": 1,
                "inputMissionType": 1,
                "missionDetail": {
                    "coordinateList": list(line_coords),
                    "lineList": [
                        {
                            "width": float(self.state.line_width_m),
                            "coordinateList": list(line_coords),
                        }
                    ],
                },
            }

        return {
            "availableAircraftList": [{"aircraftID": int(aid)} for aid in self.state.uav_ids],
            "inputMissionList": [mission],
        }

    def _build_mrpk_payload(self) -> Dict[str, Any]:
        take_over_info_list: List[Dict[str, Any]] = []
        for idx, (aid, point_xy) in enumerate(zip(self.state.uav_ids, self.state.uav_positions_xy)):
            row = {
                "aircraftID": int(aid),
                "coordinate": meters_to_coord(point_xy[0], point_xy[1]),
            }
            heading = self.state.uav_heading_deg[idx] if idx < len(self.state.uav_heading_deg) else None
            if heading is not None:
                row["headingDeg"] = float(heading)
            take_over_info_list.append(row)
        return {
            "takeOverInfoList": take_over_info_list
        }


def main() -> int:
    app = QApplication.instance() or QApplication(sys.argv)
    win = DivisionPlannerWindow()
    win.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
