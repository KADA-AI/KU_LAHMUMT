from __future__ import annotations

import math
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
from shapely.geometry import LineString, MultiPolygon, Polygon

from modules.mission_planning.MissionPlanner.planning_enhanced.algo import run_split_pipeline, review_overflow_areas
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
    from modules.mission_planning.MissionPlanner.runtime_settings import get_runtime_float, get_runtime_str
except Exception:
    from modules.mission_planning.MissionPlanner.runtime_settings import get_runtime_float, get_runtime_str  # type: ignore


_R = 6_378_137.0
_ORIGIN_LAT = 37.8535
_ORIGIN_LON = 127.4465
_DEFAULT_ALT = 0.0
_INITIAL_HALF_SPAN_M = 2_000.0
_MIN_HALF_SPAN_M = 125.0
_MAX_HALF_SPAN_M = 20_000.0
_UAV_IDS = [4, 5, 6]
_MISSION_PLANNER_DIR = Path(__file__).resolve().parents[2] / "MissionPlanner"

MODE_IDLE = "idle"
MODE_DRAW_AREA = "draw_area"
MODE_DRAW_LINE = "draw_line"
MODE_LINE_WIDTH_PENDING = "line_width_pending"
MODE_MISSION_READY = "mission_ready"
MODE_PLACE_UAV = "place_uav"
MODE_RESULT_READY = "result_ready"

MISSION_AREA = "area"
MISSION_LINE = "line"

UAV_COLORS: Dict[int, str] = {
    4: "#e53935",
    5: "#1d4ed8",
    6: "#0f9d58",
}


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
    uav_ids: List[int] = field(default_factory=list)
    split_result: Optional[SplitRunResult] = None
    expected_paths: List[Dict[str, Any]] = field(default_factory=list)
    flight_plans_0303: List[Dict[str, Any]] = field(default_factory=list)
    flight_plans_0304: List[Dict[str, Any]] = field(default_factory=list)
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
        self._draw_expected_paths(painter)
        self._draw_draft_input(painter)
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
        if world is None:
            self._hover_xy = None
            self.hoverTextChanged.emit(
                f"강원 기준 원점 LLA: {_ORIGIN_LAT:.6f}, {_ORIGIN_LON:.6f} | 중클릭 드래그: 이동 | 휠: 확대/축소"
            )
        else:
            self._hover_xy = world
            lat, lon = local_xy_to_llh(world[0], world[1])
            self.hoverTextChanged.emit(
                f"E {world[0]:7.1f}m / N {world[1]:7.1f}m | LLA {lat:.6f}, {lon:.6f} | 중클릭 드래그: 이동 | 휠: 확대/축소"
            )
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
            label = f"P{piece.piece_index}"
            if aid > 0:
                label += f" / UAV{aid}"
            screen = self.world_to_screen(center[0], center[1])
            rect = QRectF(screen.x() - 34.0, screen.y() - 12.0, 68.0, 22.0)
            painter.setPen(QColor("#0f172a"))
            painter.setBrush(QColor("#ffffff"))
            painter.drawRoundedRect(rect, 6.0, 6.0)
            painter.drawText(rect, Qt.AlignCenter, label)

    def _draw_expected_paths(self, painter: QPainter) -> None:
        if not self._state.expected_paths:
            return
        piece_lookup = self._piece_lookup(self._state.split_result)
        for row in self._state.expected_paths:
            coords = coords_to_xy(row.get("coordinateList", []))
            if len(coords) < 2:
                continue
            aid = self._path_uav_id(row, piece_lookup)
            if not self._aircraft_visible(aid):
                continue
            pen = QPen(_uav_color(aid, 230) if aid > 0 else _qcolor("#111827", 220), 2.4)
            pen.setStyle(Qt.DashLine)
            painter.setPen(pen)
            self._draw_polyline(painter, coords)
            self._draw_arrow(painter, coords[-2], coords[-1], pen.color())

    def _draw_0303_paths(self, painter: QPainter) -> None:
        if not self._state.flight_plans_0303:
            return

        for row in self._state.flight_plans_0303:
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
                    cor = fp.get("coordinateOrientation")
                    if isinstance(cor, dict):
                        target_xy = coord_to_xy(cor.get("coordinate", {}))
                        if target_xy is not None:
                            sweep_points.append(target_xy)
            traversal_points = list(coords)

            route_color = _uav_color(aid, 245) if aid > 0 else _qcolor("#111827", 240)
            if sweep_lines:
                sweep_pen = QPen(_qcolor("#0f766e", 165), 1.6)
                sweep_pen.setStyle(Qt.SolidLine)
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
                sweep_pen = QPen(_qcolor("#0f766e", 165), 1.6)
                sweep_pen.setStyle(Qt.SolidLine)
                painter.setPen(sweep_pen)
                self._draw_polyline(painter, sweep_points)

            if len(traversal_points) >= 2:
                route_outline_pen = QPen(QColor(15, 23, 42, 180), 6.4)
                route_outline_pen.setStyle(Qt.SolidLine)
                route_outline_pen.setCapStyle(Qt.RoundCap)
                route_outline_pen.setJoinStyle(Qt.RoundJoin)
                painter.setPen(route_outline_pen)
                self._draw_polyline(painter, traversal_points)

                route_pen = QPen(route_color, 4.2)
                route_pen.setStyle(Qt.SolidLine)
                route_pen.setCapStyle(Qt.RoundCap)
                route_pen.setJoinStyle(Qt.RoundJoin)
                painter.setPen(route_pen)
                self._draw_polyline(painter, traversal_points)
                self._draw_path_arrows(painter, traversal_points, route_color)

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

            for idx, point_xy in enumerate(traversal_points, start=1):
                self._draw_number_badge(painter, point_xy, str(idx), route_color, diameter=32.0)

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

    def _draw_uav_positions(self, painter: QPainter) -> None:
        for idx, point_xy in enumerate(self._state.uav_positions_xy):
            aid = self._state.uav_ids[idx] if idx < len(self._state.uav_ids) else 0
            if not self._aircraft_visible(int(aid or 0)):
                continue
            screen = self.world_to_screen(point_xy[0], point_xy[1])
            fill = _uav_color(aid, 220) if aid > 0 else _qcolor("#475569", 220)
            painter.setPen(QPen(QColor("#ffffff"), 2.0))
            painter.setBrush(fill)
            painter.drawEllipse(screen, 8.0, 8.0)
            painter.setPen(QColor("#0f172a"))
            text_rect = QRectF(screen.x() + 10.0, screen.y() - 12.0, 74.0, 24.0)
            painter.drawText(text_rect, Qt.AlignLeft | Qt.AlignVCenter, f"UAV{aid or idx + 1}")

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


class DivisionPlannerWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Division Test Mission Planner")
        self.resize(1460, 980)

        self.state = CanvasState(line_width_m=300.0)
        self._selected_uav_count = 1
        self._cmpk_payload: Optional[Dict[str, Any]] = None
        self._mrpk_payload: Optional[Dict[str, Any]] = None

        self._build_ui()
        self._apply_style()
        self._sync_canvas()
        self._refresh_ui()

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
        self.btn_run_plan = QPushButton("실제 파이프라인 실행")
        self.btn_run_plan.clicked.connect(self._run_planning)
        step3_layout.addWidget(self.btn_start_uav_input)
        step3_layout.addWidget(self.btn_run_plan)
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

        self.canvas = PlanningCanvas()
        self.canvas.worldLeftClicked.connect(self._on_canvas_left_click)
        self.canvas.worldRightClicked.connect(self._on_canvas_right_click)
        self.canvas.hoverTextChanged.connect(self.statusBar().showMessage)
        layout.addWidget(self.canvas, 1)

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
                padding: 10px 12px;
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
                min-height: 36px;
                border: 1px solid #c7d2df;
                border-radius: 8px;
                background: #f8fafc;
                color: #0f172a;
                padding: 6px 10px;
            }
            QPushButton:hover { background: #eff6ff; }
            QPushButton:disabled { color: #8a99ab; background: #e7edf4; }
            QComboBox, QDoubleSpinBox, QPlainTextEdit {
                border: 1px solid #cbd5e1;
                border-radius: 8px;
                background: #ffffff;
                padding: 6px;
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
                uav_ids=list(self.state.uav_ids),
                split_result=self.state.split_result,
                expected_paths=list(self.state.expected_paths),
                flight_plans_0303=list(self.state.flight_plans_0303),
                flight_plans_0304=list(self.state.flight_plans_0304),
                visible_uav_ids=self._visible_uav_ids(),
            )
        )

    def _refresh_ui(self) -> None:
        mission_ready = bool(self.state.mission_points_xy)
        line_mode = self.state.mission_kind == MISSION_LINE
        can_confirm_uav = mission_ready
        uav_count_locked = bool(self.state.uav_ids)
        expected_uav_count = len(self.state.uav_ids) if self.state.uav_ids else self._selected_uav_count
        uav_done = len(self.state.uav_positions_xy) == expected_uav_count if expected_uav_count > 0 else False

        self.spin_line_width.setEnabled(line_mode)
        self.cmb_uav_count.setEnabled(can_confirm_uav)
        self.btn_confirm_uav_count.setEnabled(can_confirm_uav)
        self.btn_start_uav_input.setEnabled(uav_count_locked)
        self.btn_run_plan.setEnabled(mission_ready and uav_count_locked and uav_done)
        self.btn_undo.setEnabled(bool(self.state.draft_points_xy) or bool(self.state.uav_positions_xy))

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
                f"입력 대기: {len(self.state.uav_ids)}개 위치가 필요합니다.\n캔버스에서 좌클릭으로 배치하세요."
            )
        else:
            lines = []
            for idx, point_xy in enumerate(self.state.uav_positions_xy):
                aid = self.state.uav_ids[idx] if idx < len(self.state.uav_ids) else idx + 1
                lines.append(f"UAV{aid}: {_format_coord(point_xy)}")
            if len(self.state.uav_positions_xy) < len(self.state.uav_ids):
                lines.append(f"추가 입력 필요: {len(self.state.uav_ids) - len(self.state.uav_positions_xy)}개")
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
        self.state.flight_plans_0303 = []
        self.state.flight_plans_0304 = []
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
            return float(get_runtime_float("enhanced_area_review_max_segment_m", 550.0))
        except Exception:
            return 550.0

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

        if self.state.mode in (MODE_PLACE_UAV, MODE_RESULT_READY, MODE_MISSION_READY) and self.state.uav_positions_xy:
            self.state.uav_positions_xy.pop()
            if self.state.mode == MODE_RESULT_READY:
                self.state.mode = MODE_PLACE_UAV
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
            self.state.mode = MODE_MISSION_READY if self.state.mission_points_xy else self.state.mode
            self._clear_plan_result()
        self._refresh_ui()

    def _confirm_uav_count(self) -> None:
        if not self.state.mission_points_xy:
            QMessageBox.warning(self, "임무 없음", "임무 형상을 먼저 입력해 주세요.")
            return

        self.state.uav_ids = list(_UAV_IDS[: self._selected_uav_count])
        self.state.uav_positions_xy = []
        self.state.mode = MODE_MISSION_READY
        self._clear_plan_result()
        self._append_result(f"UAV 대수 확정: {', '.join(f'UAV{aid}' for aid in self.state.uav_ids)}")
        self._refresh_ui()

    def _start_uav_input(self) -> None:
        if not self.state.uav_ids:
            QMessageBox.warning(self, "UAV 대수", "UAV 대수를 먼저 확정해 주세요.")
            return
        self.state.uav_positions_xy = []
        self.state.mode = MODE_PLACE_UAV
        self._clear_plan_result()
        self._append_result("UAV 위치 입력 모드로 전환했습니다.")
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
            self._clear_plan_result()
            if len(self.state.uav_positions_xy) == len(self.state.uav_ids):
                self.state.mode = MODE_MISSION_READY
                self._append_result("UAV 위치 입력 완료. 분할 + 경로 계획 실행이 가능합니다.")
            self._refresh_ui()

    def _on_canvas_right_click(self, _east_m: float, _north_m: float) -> None:
        if self.state.mode == MODE_DRAW_AREA:
            self._finalize_area()
            return
        if self.state.mode == MODE_DRAW_LINE:
            self._enter_line_width_pending()
            return
        if self.state.mode == MODE_LINE_WIDTH_PENDING:
            self._finalize_line()

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

    def _run_planning(self) -> None:
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
            self._cmpk_payload = self._build_cmpk_payload()
            self._mrpk_payload = self._build_mrpk_payload()
            split_result = run_split_pipeline(
                self._cmpk_payload,
                self._mrpk_payload,
                list(self.state.uav_ids),
                apply_assignment=False,
                apply_scheduling=False,
            )
            stage_lines = [f"[1] split pieces={len(split_result.pieces)}"]

            type_report = apply_logic_type_decider(split_result, self._cmpk_payload, profile_code=PROFILE_DEFAULT)
            stage_lines.append(
                "[2] type-decider "
                f"changed={int(type_report.get('changedPieces', 0))}/{int(type_report.get('pieceCount', 0))}"
            )

            expected_paths = generate_expected_paths(split_result, self._mrpk_payload)
            split_result.expected_paths = list(expected_paths)
            stage_lines.append(f"[3] expected-path count={len(expected_paths)}")

            review_report: Optional[Dict[str, Any]] = None
            if self._area_mode() not in {"nadir", "directdown", "bf_nadir"}:
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
            fp_0303, fp_0304 = build_0303_0304_from_0302_packages(
                packages_0302,
                mrpk=self._mrpk_payload,
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
        return {
            "takeOverInfoList": [
                {
                    "aircraftID": int(aid),
                    "coordinate": meters_to_coord(point_xy[0], point_xy[1]),
                }
                for aid, point_xy in zip(self.state.uav_ids, self.state.uav_positions_xy)
            ]
        }


def main() -> int:
    app = QApplication.instance() or QApplication(sys.argv)
    win = DivisionPlannerWindow()
    win.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
