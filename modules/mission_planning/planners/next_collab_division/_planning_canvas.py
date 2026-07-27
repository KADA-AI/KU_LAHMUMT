"""PlanningCanvas widget -- map rendering and interaction."""
from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Sequence, Tuple

from PyQt5.QtCore import QPoint, QPointF, QRectF, Qt, pyqtSignal
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import QSizePolicy, QWidget
from shapely.geometry import LineString, Polygon

from modules.common.turn_dynamics import turn_rate_for_radius_rad_s

from ._constants import (
    MODE_SET_UAV_HEADING, MODE_IDLE, MODE_DRAW_AREA, MODE_DRAW_LINE,
    MODE_LINE_WIDTH_PENDING, MODE_MISSION_READY, MODE_PLACE_UAV, MODE_RESULT_READY,
    MISSION_AREA, MISSION_LINE,
    _ORIGIN_LAT, _ORIGIN_LON,
    _INITIAL_HALF_SPAN_M, _MIN_HALF_SPAN_M, _MAX_HALF_SPAN_M,
    _UAV_IDS,
    TURN_PREVIEW_SPEED_MPS, TURN_PREVIEW_RADIUS_M, TURN_PREVIEW_HORIZON_S,
    TURN_PREVIEW_BANK_DEG, TURN_RADIUS_BY_SPEED_MPS,
    SplitPiece, SplitRunResult,
)
from ._geo_utils import (
    _distance, _dedupe_points, _qcolor, _uav_color,
    _bearing_deg_from_xy, _format_coord,
    local_xy_to_llh, coords_to_xy, coord_to_xy,
    corridor_polygon_xy, centroid_xy,
)
from ._canvas_state import CanvasState


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
        self._draw_next_mission_circles(painter)
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
        assigned_stage2_targets = self._assigned_stage2_target_map()
        assignment_rows_by_piece = self._assignment_rows_by_piece()
        for piece in self._state.split_result.pieces:
            visual_polygon_rows = self._piece_visual_polygon_rows(piece)
            visual_polygons_xy = [list(row.get("polygonXY", [])) for row in visual_polygon_rows]
            if not visual_polygon_rows:
                continue
            piece_index = int(piece.piece_index or 0)
            aid = int(piece.assigned_uav or 0)
            if not self._aircraft_visible(aid):
                continue
            use_assignment_fill = bool(assigned_stage2_targets) and len(visual_polygon_rows) >= 2
            for polygon_row in visual_polygon_rows:
                coords = polygon_row.get("polygonXY")
                if not isinstance(coords, list) or len(coords) < 3:
                    continue
                if use_assignment_fill:
                    part_name = str(polygon_row.get("name", "") or "").strip().upper()
                    selected_aid = self._assigned_stage2_target_aid(piece_index, part_name, assigned_stage2_targets)
                    if selected_aid > 0:
                        fill = _uav_color(selected_aid, 90)
                        edge = _uav_color(selected_aid, 210)
                    else:
                        fill = _qcolor("#cbd5e1", 56)
                        edge = _qcolor("#94a3b8", 190)
                elif aid > 0:
                    fill = _uav_color(aid, 78)
                    edge = _uav_color(aid, 190)
                else:
                    fill = _qcolor("#64748b", 70)
                    edge = _qcolor("#475569", 220)
                painter.setBrush(fill)
                painter.setPen(QPen(edge, 1.6))
                if len(coords) < 3:
                    continue
                painter.drawPolygon(QPolygonF([self.world_to_screen(x, y) for (x, y) in coords]))

            center = self._piece_center_xy(piece, piece_lookup)
            if center is None:
                flat_coords = [point_xy for coords in visual_polygons_xy for point_xy in coords]
                center = centroid_xy(flat_coords)
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
            label = f"P{piece_index}"
            if aid > 0:
                label += f" / UAV{aid}"
            screen = self.world_to_screen(center[0], center[1])
            rect = QRectF(screen.x() - 34.0, screen.y() - 32.0, 68.0, 22.0)
            painter.setPen(QColor("#0f172a"))
            painter.setBrush(QColor("#ffffff"))
            painter.drawRoundedRect(rect, 6.0, 6.0)
            painter.drawText(rect, Qt.AlignCenter, label)

            assignment_rows = assignment_rows_by_piece.get(piece_index, [])
            if assignment_rows:
                painter.save()
                drawn_label_centers: List[Tuple[float, float]] = []
                for row in assignment_rows:
                    if not isinstance(row, dict):
                        continue
                    part_polygon_xy = row.get("partPolygonXY")
                    coords: List[Tuple[float, float]] = []
                    if isinstance(part_polygon_xy, list):
                        coords = [
                            (float(point_xy[0]), float(point_xy[1]))
                            for point_xy in part_polygon_xy
                            if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2
                        ]
                    if len(coords) < 3:
                        continue

                    part_center_xy = centroid_xy(coords)
                    if part_center_xy is None:
                        continue

                    part_width_m = float(row.get("partWidthM", 0.0) or 0.0)
                    selected_sep_m = float(row.get("dbSepM", 0.0) or row.get("sepCandM", 0.0) or 0.0)
                    if selected_sep_m <= 0.0 and part_width_m > 0.0:
                        db_row = _largest_sep_covering_db_row_for_width(part_width_m)
                        if isinstance(db_row, dict):
                            selected_sep_m = float(db_row.get("sep", 0.0) or 0.0)

                    overlap_idx = sum(
                        1
                        for point_xy in drawn_label_centers
                        if _distance(point_xy, part_center_xy) <= 28.0
                    )
                    drawn_label_centers.append(part_center_xy)
                    label_text = (
                        f"W {part_width_m:.0f} / SEP {selected_sep_m:.0f}"
                        if selected_sep_m > 0.0
                        else f"W {part_width_m:.0f} / SEP -"
                    )
                    label_screen = self.world_to_screen(float(part_center_xy[0]), float(part_center_xy[1]))
                    label_rect = QRectF(
                        label_screen.x() - 68.0,
                        label_screen.y() + 16.0 + (float(overlap_idx) * 18.0),
                        136.0,
                        18.0,
                    )
                    painter.setPen(QColor("#0f172a"))
                    painter.setBrush(QColor("#ffffff"))
                    painter.drawRoundedRect(label_rect, 5.0, 5.0)
                    painter.drawText(label_rect, Qt.AlignCenter, label_text)
                painter.restore()

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
                        text_w = max(34.0, 10.0 + (float(len(label)) * 7.0))
                        rect = QRectF(screen.x() + 5.0, screen.y() - 11.0, text_w, 18.0)
                        painter.drawText(rect, Qt.AlignLeft | Qt.AlignVCenter, label)
                painter.restore()

            # Draw lines from T (tangent point) to both endpoints of the Near line, with distances
            entry_endpoints_raw = row.get("entryLineEndpointsXY") if isinstance(row, dict) else None
            tangent_raw = row.get("tangentXY") if isinstance(row, dict) else None
            if (
                isinstance(entry_endpoints_raw, list) and len(entry_endpoints_raw) >= 2
                and isinstance(tangent_raw, (tuple, list)) and len(tangent_raw) >= 2
                and self._aircraft_visible(aid)
                and not self._state.show_next_mission_circles
            ):
                tangent_xy = (float(tangent_raw[0]), float(tangent_raw[1]))
                near_color = QColor("#a855f7")
                near_pen = QPen(near_color, 2.0, Qt.SolidLine)
                near_pen.setCapStyle(Qt.RoundCap)
                painter.save()
                for ep_raw in entry_endpoints_raw:
                    if not (isinstance(ep_raw, (tuple, list)) and len(ep_raw) >= 2):
                        continue
                    ep_xy = (float(ep_raw[0]), float(ep_raw[1]))
                    dist_m = _distance(tangent_xy, ep_xy)
                    painter.setPen(near_pen)
                    self._draw_polyline(painter, [tangent_xy, ep_xy])
                    ep_screen = self.world_to_screen(ep_xy[0], ep_xy[1])
                    painter.setPen(QPen(near_color, 1.2))
                    painter.setBrush(QColor("#ffffff"))
                    painter.drawEllipse(ep_screen, 3.5, 3.5)
                    mid_x = (tangent_xy[0] + ep_xy[0]) * 0.5
                    mid_y = (tangent_xy[1] + ep_xy[1]) * 0.5
                    mid_screen = self.world_to_screen(mid_x, mid_y)
                    dist_rect = QRectF(mid_screen.x() + 4.0, mid_screen.y() - 10.0, 72.0, 18.0)
                    painter.setPen(QColor("#0f172a"))
                    painter.setBrush(_qcolor("#ffffff", 232))
                    painter.drawRoundedRect(dist_rect, 5.0, 5.0)
                    painter.setPen(near_color.darker(140))
                    painter.drawText(dist_rect, Qt.AlignCenter, f"{dist_m:.0f} m")
                painter.restore()

            # Draw Entry T' marker if needed
            entry_t_prime_raw = row.get("entryTPrimeXY") if isinstance(row, dict) else None
            if (
                isinstance(entry_t_prime_raw, (tuple, list)) and len(entry_t_prime_raw) >= 2
                and self._aircraft_visible(aid)
            ):
                t_prime_xy = (float(entry_t_prime_raw[0]), float(entry_t_prime_raw[1]))
                t_prime_screen = self.world_to_screen(t_prime_xy[0], t_prime_xy[1])
                painter.save()
                t_prime_color = QColor("#e11d48")
                painter.setPen(QPen(QColor("#ffffff"), 1.0))
                painter.setBrush(t_prime_color)
                painter.drawEllipse(t_prime_screen, 4.5, 4.5)
                t_prime_rect = QRectF(t_prime_screen.x() + 6.0, t_prime_screen.y() - 11.0, 24.0, 18.0)
                painter.setPen(QPen(t_prime_color.darker(140), 1.0))
                painter.drawText(t_prime_rect, Qt.AlignLeft | Qt.AlignVCenter, "T'")
                painter.restore()

            # Draw Sep_cand and FOV label
            sep_cand_val = float(row.get("sepCandM", 0.0) or 0.0) if isinstance(row, dict) else 0.0
            resolved_fov = row.get("resolvedFovDeg") if isinstance(row, dict) else None
            resolved_vel = row.get("resolvedVelMps") if isinstance(row, dict) else None
            if sep_cand_val > 0.0 and self._aircraft_visible(aid):
                # Find a suitable position for the label (near the first near face endpoint)
                _ep_raw = row.get("entryLineEndpointsXY") if isinstance(row, dict) else None
                if isinstance(_ep_raw, list) and _ep_raw:
                    first_ep = _ep_raw[0]
                    if isinstance(first_ep, (tuple, list)) and len(first_ep) >= 2:
                        fov_label_screen = self.world_to_screen(float(first_ep[0]), float(first_ep[1]))
                        fov_text = f"SEP {sep_cand_val:.0f}"
                        if resolved_fov is not None:
                            fov_text += f" / FOV {resolved_fov:.1f}"
                        else:
                            fov_text += " / FOV -"
                        if resolved_vel is not None:
                            fov_text += f" / VEL {resolved_vel:.0f}"
                        painter.save()
                        fov_rect = QRectF(fov_label_screen.x() - 100.0, fov_label_screen.y() + 8.0, 200.0, 18.0)
                        painter.setPen(QColor("#0f172a"))
                        painter.setBrush(_qcolor("#ffffff", 240))
                        painter.drawRoundedRect(fov_rect, 5.0, 5.0)
                        painter.setPen(QColor("#7c3aed"))
                        painter.drawText(fov_rect, Qt.AlignCenter, fov_text)
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
            piece_index = int(row.get("pieceIndex", 0) or 0)
            aid = int(row.get("aircraftID", 0) or 0)
            if not self._aircraft_visible(aid):
                continue
            mid_line_required = bool(row.get("midLineRequired", True))
            base_line_color = _uav_color(aid, 190) if aid > 0 else _qcolor("#475569", 190)
            base_width_color = _uav_color(aid, 245) if aid > 0 else _qcolor("#0f172a", 230)

            box_xy = row.get("boxXY")
            if isinstance(box_xy, list) and len(box_xy) >= 4:
                box_points = []
                for point_xy in box_xy:
                    if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2:
                        box_points.append(self.world_to_screen(float(point_xy[0]), float(point_xy[1])))
                if len(box_points) >= 4:
                    painter.save()
                    box_pen = QPen(base_line_color, 1.0, Qt.DotLine)
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
                line_pen = QPen(base_line_color, 1.8, Qt.DashLine)
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

            t0_route_xy = row.get("t0RouteXY")
            t0_points: List[Tuple[float, float]] = []
            if isinstance(t0_route_xy, list):
                for point_xy in t0_route_xy:
                    if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2:
                        t0_points.append((float(point_xy[0]), float(point_xy[1])))
            if len(t0_points) >= 2:
                painter.save()
                t0_pen = QPen(_uav_color(aid, 225) if aid > 0 else _qcolor("#0f172a", 220), 2.0, Qt.DashLine)
                t0_pen.setCapStyle(Qt.RoundCap)
                t0_pen.setJoinStyle(Qt.RoundJoin)
                painter.setPen(t0_pen)
                self._draw_polyline(painter, t0_points)
                self._draw_arrow(painter, t0_points[-2], t0_points[-1], t0_pen.color())
                painter.restore()

            t0_marker_rows = row.get("t0MarkerRows")
            if isinstance(t0_marker_rows, list):
                painter.save()
                for marker in t0_marker_rows:
                    if not isinstance(marker, dict):
                        continue
                    point_xy = marker.get("xy")
                    if not (isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2):
                        continue
                    label = str(marker.get("label", "") or "")
                    kind = str(marker.get("kind", "") or "")
                    if kind == "tangent":
                        color = QColor("#f59e0b")
                        radius = 4.0
                    elif kind == "turn":
                        color = _uav_color(aid, 230) if aid > 0 else _qcolor("#0f172a", 220)
                        radius = 3.4
                    else:
                        color = _uav_color(aid, 210) if aid > 0 else _qcolor("#0f172a", 220)
                        radius = 3.4
                    screen = self.world_to_screen(float(point_xy[0]), float(point_xy[1]))
                    painter.setPen(QPen(QColor("#ffffff"), 1.0))
                    painter.setBrush(color)
                    painter.drawEllipse(screen, radius, radius)
                    if label:
                        painter.setPen(QPen(color.darker(220), 1.0))
                        text_w = max(34.0, 10.0 + (float(len(label)) * 7.0))
                        rect = QRectF(screen.x() + 5.0, screen.y() - 11.0, text_w, 18.0)
                        painter.drawText(rect, Qt.AlignLeft | Qt.AlignVCenter, label)
                painter.restore()

            t0_tangent_raw = row.get("t0TangentXY")
            t0_shape_point_raw = row.get("t0ShapePointXY")
            if (
                isinstance(t0_tangent_raw, (tuple, list)) and len(t0_tangent_raw) >= 2
                and isinstance(t0_shape_point_raw, (tuple, list)) and len(t0_shape_point_raw) >= 2
            ):
                t0_tangent_xy = (float(t0_tangent_raw[0]), float(t0_tangent_raw[1]))
                t0_shape_point_xy = (float(t0_shape_point_raw[0]), float(t0_shape_point_raw[1]))
                t0_shape_point_dist_m = float(row.get("t0ShapePointDistM", 0.0) or 0.0)
                if t0_shape_point_dist_m <= 1e-6:
                    t0_shape_point_dist_m = _distance(t0_tangent_xy, t0_shape_point_xy)
                near_color = QColor("#a855f7")
                near_pen = QPen(near_color, 2.0, Qt.SolidLine)
                near_pen.setCapStyle(Qt.RoundCap)
                painter.save()
                painter.setPen(near_pen)
                self._draw_polyline(painter, [t0_tangent_xy, t0_shape_point_xy])
                mid_x = (t0_tangent_xy[0] + t0_shape_point_xy[0]) * 0.5
                mid_y = (t0_tangent_xy[1] + t0_shape_point_xy[1]) * 0.5
                mid_screen = self.world_to_screen(mid_x, mid_y)
                dist_rect = QRectF(mid_screen.x() + 4.0, mid_screen.y() - 10.0, 72.0, 18.0)
                painter.setPen(QColor("#0f172a"))
                painter.setBrush(_qcolor("#ffffff", 232))
                painter.drawRoundedRect(dist_rect, 5.0, 5.0)
                painter.setPen(near_color.darker(140))
                painter.drawText(dist_rect, Qt.AlignCenter, f"{t0_shape_point_dist_m:.0f} m")
                painter.restore()

            max_width_xy = row.get("maxWidthLineXY")
            width_points: List[Tuple[float, float]] = []
            if isinstance(max_width_xy, list) and len(max_width_xy) >= 2:
                for point_xy in max_width_xy:
                    if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2:
                        width_points.append((float(point_xy[0]), float(point_xy[1])))
            if len(width_points) >= 2:
                width_pen = QPen(base_width_color, 2.2)
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
                center_pen = QPen(base_line_color, 1.2)
                painter.setPen(center_pen)
                drawn_stage2_points: List[Tuple[float, float]] = []
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
                        center_label = str(center_row.get("label", "") or f"F{idx}").strip().upper()
                        if "A" in center_label and "B" not in center_label:
                            center_line_color = _qcolor("#f97316", 235)
                        elif "B" in center_label:
                            center_line_color = _qcolor("#14b8a6", 235)
                        else:
                            center_line_color = base_line_color
                        painter.setPen(QPen(center_line_color, 1.8, Qt.SolidLine))
                        self._draw_polyline(painter, line_points)
                    near_xy = center_row.get("nearXY")
                    if isinstance(near_xy, (tuple, list)) and len(near_xy) >= 2:
                        near_point_xy = (float(near_xy[0]), float(near_xy[1]))
                        near_screen = self.world_to_screen(near_point_xy[0], near_point_xy[1])
                        near_overlap_idx = sum(
                            1
                            for point_xy in drawn_stage2_points
                            if _distance(point_xy, near_point_xy) <= 12.0
                        )
                        drawn_stage2_points.append(near_point_xy)
                        near_fill = _qcolor("#ffffff", 235)
                        center_label = str(center_row.get("label", "") or f"F{idx}").strip().upper()
                        if "A" in center_label and "B" not in center_label:
                            near_dot = _qcolor("#f97316", 235)
                        elif "B" in center_label:
                            near_dot = _qcolor("#14b8a6", 235)
                        else:
                            near_dot = _uav_color(aid, 185) if aid > 0 else _qcolor("#94a3b8", 210)
                        painter.setPen(QPen(near_dot, 1.2))
                        painter.setBrush(near_fill)
                        painter.drawEllipse(near_screen, 3.6, 3.6)
                        near_label = str(center_row.get("nearLabel", "") or f"N{idx}")
                        near_rect = QRectF(
                            near_screen.x() + 6.0,
                            near_screen.y() - 10.0 + (float(near_overlap_idx) * 16.0),
                            44.0,
                            18.0,
                        )
                        painter.setPen(QColor("#0f172a"))
                        painter.setBrush(_qcolor("#ffffff", 232))
                        painter.drawRoundedRect(near_rect, 5.0, 5.0)
                        painter.drawText(near_rect, Qt.AlignCenter, near_label)
                    center_xy = center_row.get("centerXY")
                    if not (isinstance(center_xy, (tuple, list)) and len(center_xy) >= 2):
                        continue
                    center_point_xy = (float(center_xy[0]), float(center_xy[1]))
                    screen = self.world_to_screen(center_point_xy[0], center_point_xy[1])
                    overlap_idx = sum(
                        1
                        for point_xy in drawn_stage2_points
                        if _distance(point_xy, center_point_xy) <= 12.0
                    )
                    drawn_stage2_points.append(center_point_xy)
                    fill = _qcolor("#ffffff", 240)
                    center_label = str(center_row.get("label", "") or f"F{idx}").strip().upper()
                    if "A" in center_label and "B" not in center_label:
                        dot = _qcolor("#f97316", 245)
                    elif "B" in center_label:
                        dot = _qcolor("#14b8a6", 245)
                    else:
                        dot = _uav_color(aid, 255) if aid > 0 else _qcolor("#94a3b8", 240)
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

        arc_angle = float(turn_rate_for_radius_rad_s(speed_mps, radius_m) or 0.0) * float(horizon_s)
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
            if heading is not None and self._state.show_turn_overlays and not self._state.show_next_mission_circles:
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

    def _draw_next_mission_circles(self, painter: QPainter) -> None:
        if not self._state.show_next_mission_circles or not self._state.next_mission_rows:
            return
        for row in self._state.next_mission_rows:
            if not isinstance(row, dict):
                continue
            mp_end_raw = row.get("mpEndXY")
            bearing_raw = row.get("bearingDeg")
            aid = int(row.get("aircraftID", 0) or 0)
            if mp_end_raw is None or bearing_raw is None:
                continue
            if not self._aircraft_visible(aid):
                continue
            mp_end_xy = (float(mp_end_raw[0]), float(mp_end_raw[1]))
            bearing_deg = float(bearing_raw)
            turn_radius_m = float(row.get("turnRadiusM", TURN_PREVIEW_RADIUS_M) or TURN_PREVIEW_RADIUS_M)
            vel_mps = float(row.get("velMps", 0.0) or 0.0)
            color = _uav_color(aid, 160) if aid > 0 else _qcolor("#475569", 160)
            self._draw_turn_radius_circles(painter, mp_end_xy, bearing_deg, color, radius_m=turn_radius_m)
            # MP_E 점 표시
            screen = self.world_to_screen(mp_end_xy[0], mp_end_xy[1])
            dot_color = _uav_color(aid, 255) if aid > 0 else _qcolor("#475569", 255)
            painter.save()
            painter.setPen(QPen(QColor("#ffffff"), 2.0))
            painter.setBrush(dot_color)
            painter.drawEllipse(screen, 6.0, 6.0)
            # heading 화살표
            self._draw_bearing_arrow(painter, mp_end_xy, bearing_deg, dot_color, length_px=28.0)
            # 레이블 (속도 + 반경 포함)
            painter.setPen(QColor("#0f172a"))
            vel_label = f"  {vel_mps:.0f}m/s R{turn_radius_m:.0f}" if vel_mps > 0 else ""
            label_rect = QRectF(screen.x() + 9.0, screen.y() - 12.0, 160.0, 24.0)
            painter.drawText(label_rect, Qt.AlignLeft | Qt.AlignVCenter, f"UAV{aid} MP_E{vel_label}")
            painter.restore()

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

    def _assigned_stage2_target_map(self) -> Dict[Tuple[int, str], int]:
        out: Dict[Tuple[int, str], int] = {}
        rows = self._state.assignment_path_rows
        if not isinstance(rows, list):
            return out
        for row in rows:
            if not isinstance(row, dict):
                continue
            piece_index = int(row.get("pieceIndex", 0) or 0)
            aid = int(row.get("aircraftID", 0) or 0)
            if piece_index <= 0:
                continue
            target_label = str(row.get("targetLabel", "") or "").strip().upper()
            if not target_label:
                continue
            out[(int(piece_index), target_label)] = int(aid)
        return out

    def _assigned_stage2_target_aid(
        self,
        piece_index: int,
        part_name: str,
        assigned_stage2_targets: Dict[Tuple[int, str], int],
    ) -> int:
        normalized = str(part_name or "").strip().upper()
        if not normalized:
            return 0
        for candidate_label in (normalized, f"F{normalized}", f"N{normalized}"):
            aid = int(assigned_stage2_targets.get((int(piece_index), candidate_label), 0) or 0)
            if aid > 0:
                return aid
        for (mapped_piece_index, mapped_label), aid in assigned_stage2_targets.items():
            if int(mapped_piece_index) != int(piece_index):
                continue
            label_text = str(mapped_label or "").strip().upper()
            if label_text.endswith(normalized):
                aid_val = int(aid or 0)
                if aid_val > 0:
                    return aid_val
        return 0

    def _assignment_rows_by_piece(self) -> Dict[int, List[Dict[str, Any]]]:
        out: Dict[int, List[Dict[str, Any]]] = {}
        rows = self._state.assignment_path_rows
        if not isinstance(rows, list) or not rows:
            rows = [
                dict(row)
                for row in self._state.expected_paths
                if isinstance(row, dict) and str(row.get("source", "") or "") == "assignment_path_1"
            ]
        for row in rows:
            if not isinstance(row, dict):
                continue
            piece_index = int(row.get("pieceIndex", 0) or 0)
            if piece_index <= 0:
                continue
            out.setdefault(int(piece_index), []).append(row)
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
        polygons_xy = self._piece_visual_polygon_groups_xy(piece)
        if not polygons_xy:
            return None
        if len(polygons_xy) == 1:
            coords = polygons_xy[0]
            poly = Polygon(coords)
            if poly.is_empty:
                return centroid_xy(coords)
            center = poly.centroid
            return float(center.x), float(center.y)

        area_sum = 0.0
        cx_sum = 0.0
        cy_sum = 0.0
        flat_coords: List[Tuple[float, float]] = []
        for coords in polygons_xy:
            flat_coords.extend(coords)
            try:
                poly = Polygon(coords)
            except Exception:
                poly = None
            if poly is None or poly.is_empty:
                continue
            area_val = float(abs(poly.area))
            if area_val <= 1e-9:
                continue
            center = poly.centroid
            area_sum += area_val
            cx_sum += float(center.x) * area_val
            cy_sum += float(center.y) * area_val
        if area_sum > 1e-9:
            return (cx_sum / area_sum, cy_sum / area_sum)
        return centroid_xy(flat_coords)

    def _piece_visual_polygon_groups_xy(
        self,
        piece: SplitPiece,
    ) -> List[List[Tuple[float, float]]]:
        return [
            list(row.get("polygonXY", []))
            for row in self._piece_visual_polygon_rows(piece)
            if isinstance(row, dict)
        ]

    def _piece_visual_polygon_rows(
        self,
        piece: SplitPiece,
    ) -> List[Dict[str, Any]]:
        data = piece.data if isinstance(piece.data, dict) else {}
        review = data.get("reviewArea") if isinstance(data.get("reviewArea"), dict) else {}

        visual_groups = review.get("boxedPartPolygonsXY")
        out: List[Dict[str, Any]] = []
        if isinstance(visual_groups, list):
            for idx, group in enumerate(visual_groups):
                polygon_xy = group.get("polygonXY") if isinstance(group, dict) else group
                if not isinstance(polygon_xy, list) or len(polygon_xy) < 3:
                    continue
                coords = [
                    (float(point_xy[0]), float(point_xy[1]))
                    for point_xy in polygon_xy
                    if isinstance(point_xy, (tuple, list)) and len(point_xy) >= 2
                ]
                if len(coords) >= 3:
                    name = (
                        str(group.get("name", "") or chr(ord("A") + min(int(idx), 25))).strip().upper()
                        if isinstance(group, dict)
                        else chr(ord("A") + min(int(idx), 25))
                    )
                    out.append({"name": name, "polygonXY": coords})
        if out:
            return out

        coords = coords_to_xy(data.get("coordinateList", []))
        if len(coords) >= 3:
            return [{"name": "P", "polygonXY": coords}]
        return []

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
