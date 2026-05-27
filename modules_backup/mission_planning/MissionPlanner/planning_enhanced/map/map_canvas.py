from __future__ import annotations

import math
from typing import Any, Dict, Iterable, List, Optional, Tuple

from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PyQt5.QtWidgets import QWidget

from ..models import DirectionDebug, SplitRunResult


LINE_TYPES = {1, 4, 5, 7}
AREA_TYPES = {2, 3, 6}

COLORS = [
    "#e53935",
    "#1e88e5",
    "#43a047",
    "#fb8c00",
    "#8e24aa",
    "#00897b",
    "#6d4c41",
    "#3949ab",
    "#c2185b",
    "#7cb342",
]


def _pick_color(idx: int) -> str:
    return COLORS[idx % len(COLORS)]


def _to_int(value: Any, default: int = 0) -> int:
    try:
        return int(value)
    except Exception:
        return default


def _piece_is_area_like(piece: Any) -> bool:
    try:
        mtype = int(getattr(piece, "mission_type", 0) or 0)
    except Exception:
        mtype = 0
    return mtype in AREA_TYPES


def _coord_to_latlon(coord: Dict[str, Any]) -> Optional[Tuple[float, float]]:
    if not isinstance(coord, dict):
        return None
    if "latitude" not in coord or "longitude" not in coord:
        return None
    return float(coord["latitude"]), float(coord["longitude"])


def _coords_to_latlon(coords: Iterable[Dict[str, Any]]) -> List[Tuple[float, float]]:
    out: List[Tuple[float, float]] = []
    for c in coords:
        ll = _coord_to_latlon(c)
        if ll is not None:
            out.append(ll)
    return out


def _centroid(coords: List[Tuple[float, float]]) -> Tuple[float, float]:
    lat = sum(c[0] for c in coords) / len(coords)
    lon = sum(c[1] for c in coords) / len(coords)
    return lat, lon


def _corridor_polygon(path_ll: List[Tuple[float, float]], width_m: float) -> List[Tuple[float, float]]:
    half = float(width_m) / 2.0
    poly_left: List[Tuple[float, float]] = []
    poly_right: List[Tuple[float, float]] = []

    for i in range(len(path_ll) - 1):
        lat1, lon1 = path_ll[i]
        lat2, lon2 = path_ll[i + 1]
        dx = (lon2 - lon1) * 111_000.0 * math.cos(math.radians((lat1 + lat2) / 2.0))
        dy = (lat2 - lat1) * 111_000.0
        length = math.hypot(dx, dy)
        if length == 0:
            continue
        ux, uy = dx / length, dy / length
        px, py = uy, -ux

        dlat = (py * half) / 111_000.0
        dlon = (px * half) / (111_000.0 * math.cos(math.radians(lat1)))

        left_start = (lat1 + dlat, lon1 + dlon)
        right_start = (lat1 - dlat, lon1 - dlon)
        left_end = (lat2 + dlat, lon2 + dlon)
        right_end = (lat2 - dlat, lon2 - dlon)

        if i == 0:
            poly_left.append(left_start)
            poly_right.append(right_start)
        poly_left.append(left_end)
        poly_right.append(right_end)

    return poly_left + poly_right[::-1]


def _offset_latlon(lat: float, lon: float, north_m: float = 0.0, east_m: float = 0.0) -> Tuple[float, float]:
    k = 111_132.92
    dlat = north_m / k
    dlon = east_m / (k * math.cos(math.radians(lat)))
    return lat + dlat, lon + dlon


def _bearing_to_vec(bearing_deg: float, length_m: float) -> Tuple[float, float]:
    th = math.radians(float(bearing_deg))
    east = math.sin(th) * length_m
    north = math.cos(th) * length_m
    return north, east


def _segment_len_m(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    lat1, lon1 = a
    lat2, lon2 = b
    dx = (lon2 - lon1) * 111_000.0 * math.cos(math.radians((lat1 + lat2) / 2.0))
    dy = (lat2 - lat1) * 111_000.0
    return math.hypot(dx, dy)


class MissionMapCanvas(QWidget):
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._cmpk: Optional[Dict[str, Any]] = None
        self._mrpk: Optional[Dict[str, Any]] = None
        self._split_result: Optional[SplitRunResult] = None
        self._flight_plans_0303: List[Dict[str, Any]] = []
        self._flight_plans_0304: List[Dict[str, Any]] = []
        self._layer_visibility: Dict[str, bool] = {
            "show_0201": True,
            "show_0203": True,
            "show_split": True,
            "show_direction": True,
            "show_expected_paths": True,
            "show_0303_route": True,
            "show_0303_sweep": True,
            "show_0304": True,
            "show_path_unmanned": True,
            "show_path_manned": True,
        }
        self._base_geo_bounds: Optional[Tuple[float, float, float, float]] = None
        self._geo_bounds: Optional[Tuple[float, float, float, float]] = None
        self._manual_geo_bounds: Optional[Tuple[float, float, float, float]] = None
        self._view_rect: Optional[QRectF] = None
        self._zoom_min_ratio = 0.02
        self._zoom_max_ratio = 4.0
        self._is_panning = False
        self._pan_last_pos: Optional[QPointF] = None
        self.setMinimumSize(920, 620)
        # Ensure wheel events are routed to this canvas when cursor is over it.
        self.setFocusPolicy(Qt.WheelFocus)
        self.setMouseTracking(True)

    def set_data(
        self,
        cmpk: Optional[Dict[str, Any]],
        mrpk: Optional[Dict[str, Any]],
        split_result: Optional[SplitRunResult],
        *,
        flight_plans_0303: Optional[List[Dict[str, Any]]] = None,
        flight_plans_0304: Optional[List[Dict[str, Any]]] = None,
        layer_visibility: Optional[Dict[str, bool]] = None,
    ) -> None:
        # Reset zoom only when the base scenario changes.
        if (cmpk is not self._cmpk) or (mrpk is not self._mrpk):
            self._manual_geo_bounds = None
        self._cmpk = cmpk
        self._mrpk = mrpk
        self._split_result = split_result
        self._flight_plans_0303 = list(flight_plans_0303) if isinstance(flight_plans_0303, list) else []
        self._flight_plans_0304 = list(flight_plans_0304) if isinstance(flight_plans_0304, list) else []
        if isinstance(layer_visibility, dict):
            for key, value in layer_visibility.items():
                self._layer_visibility[str(key)] = bool(value)
        self.update()

    def _layer_on(self, key: str, default: bool = True) -> bool:
        if key not in self._layer_visibility:
            return bool(default)
        return bool(self._layer_visibility.get(key, default))

    def _path_aircraft_visible(self, aircraft_id: int) -> bool:
        aid = int(aircraft_id)
        # Convention in this project:
        # 1~3: manned(LAH), 4~6: unmanned(UAV)
        if 1 <= aid <= 3:
            return self._layer_on("show_path_manned", True)
        return self._layer_on("show_path_unmanned", True)

    def paintEvent(self, _event) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QColor("#f8fafc"))

        if self.width() < 40 or self.height() < 40:
            return

        padding = 32.0
        self._view_rect = QRectF(
            padding,
            padding,
            max(1.0, self.width() - (padding * 2.0)),
            max(1.0, self.height() - (padding * 2.0)),
        )
        self._base_geo_bounds = self._compute_bounds()
        if self._base_geo_bounds is None:
            self._geo_bounds = None
            self._manual_geo_bounds = None
        elif self._manual_geo_bounds is None:
            self._geo_bounds = self._base_geo_bounds
        else:
            self._geo_bounds = self._normalize_bounds(self._manual_geo_bounds, self._base_geo_bounds)

        self._draw_grid(painter)
        if self._geo_bounds is None:
            self._draw_empty_message(painter)
            return

        if self._layer_on("show_0201", True):
            self._draw_original_missions(painter)
        if self._layer_on("show_0203", True):
            self._draw_0203_points(painter)
        if self._layer_on("show_split", True):
            self._draw_split_result(painter)
        if self._layer_on("show_expected_paths", True):
            self._draw_expected_paths(painter)
        if self._layer_on("show_direction", True):
            self._draw_direction_debugs(painter)
        if self._layer_on("show_0303_route", True) or self._layer_on("show_0303_sweep", True):
            self._draw_0303_paths(painter)
        if self._layer_on("show_0304", True):
            self._draw_0304_paths(painter)
        self._draw_legend(painter)

    def wheelEvent(self, event) -> None:
        if self._view_rect is None or self._geo_bounds is None or self._base_geo_bounds is None:
            event.ignore()
            return
        pos = event.posF() if hasattr(event, "posF") else QPointF(float(event.pos().x()), float(event.pos().y()))
        sx = float(pos.x())
        sy = float(pos.y())
        if not self._view_rect.contains(QPointF(sx, sy)):
            event.ignore()
            return

        angle_delta = float(event.angleDelta().y())
        pixel_delta = float(event.pixelDelta().y())
        if abs(angle_delta) > 1e-9:
            steps = angle_delta / 120.0
        elif abs(pixel_delta) > 1e-9:
            # Trackpad fallback: smaller step scale than wheel notch.
            steps = pixel_delta / 60.0
        else:
            event.ignore()
            return

        scale = 0.85 ** steps  # wheel up: zoom in

        bounds = self._geo_bounds
        lat_min, lat_max, lon_min, lon_max = bounds
        lat_range = max(1e-9, lat_max - lat_min)
        lon_range = max(1e-9, lon_max - lon_min)

        anchor = self._screen_to_geo(sx, sy, bounds)
        if anchor is None:
            event.ignore()
            return
        a_lat, a_lon = anchor

        base = self._base_geo_bounds
        b_lat_range = max(1e-9, base[1] - base[0])
        b_lon_range = max(1e-9, base[3] - base[2])
        min_lat_range = b_lat_range * self._zoom_min_ratio
        max_lat_range = b_lat_range * self._zoom_max_ratio
        min_lon_range = b_lon_range * self._zoom_min_ratio
        max_lon_range = b_lon_range * self._zoom_max_ratio

        new_lat_range = min(max(lat_range * scale, min_lat_range), max_lat_range)
        new_lon_range = min(max(lon_range * scale, min_lon_range), max_lon_range)

        nx = (a_lon - lon_min) / lon_range
        ny = (lat_max - a_lat) / lat_range
        new_lon_min = a_lon - nx * new_lon_range
        new_lon_max = new_lon_min + new_lon_range
        new_lat_max = a_lat + ny * new_lat_range
        new_lat_min = new_lat_max - new_lat_range

        self._manual_geo_bounds = (new_lat_min, new_lat_max, new_lon_min, new_lon_max)
        self._geo_bounds = self._normalize_bounds(self._manual_geo_bounds, base)
        self.update()
        event.accept()

    def mousePressEvent(self, event) -> None:
        if event.button() != Qt.RightButton:
            super().mousePressEvent(event)
            return
        if self._view_rect is None or self._geo_bounds is None:
            event.ignore()
            return
        pos = event.localPos() if hasattr(event, "localPos") else QPointF(float(event.pos().x()), float(event.pos().y()))
        if not self._view_rect.contains(pos):
            event.ignore()
            return
        self._is_panning = True
        self._pan_last_pos = QPointF(float(pos.x()), float(pos.y()))
        self.setCursor(Qt.ClosedHandCursor)
        event.accept()

    def mouseMoveEvent(self, event) -> None:
        if not self._is_panning:
            super().mouseMoveEvent(event)
            return
        if (event.buttons() & Qt.RightButton) == 0:
            self._is_panning = False
            self._pan_last_pos = None
            self.unsetCursor()
            super().mouseMoveEvent(event)
            return
        if self._view_rect is None or self._geo_bounds is None or self._base_geo_bounds is None:
            event.ignore()
            return

        pos = event.localPos() if hasattr(event, "localPos") else QPointF(float(event.pos().x()), float(event.pos().y()))
        last = self._pan_last_pos
        if last is None:
            self._pan_last_pos = QPointF(float(pos.x()), float(pos.y()))
            event.accept()
            return

        dx = float(pos.x() - last.x())
        dy = float(pos.y() - last.y())
        if abs(dx) < 1e-9 and abs(dy) < 1e-9:
            event.accept()
            return

        lat_min, lat_max, lon_min, lon_max = self._geo_bounds
        lat_range = max(1e-9, lat_max - lat_min)
        lon_range = max(1e-9, lon_max - lon_min)
        px_w = max(1e-9, self._view_rect.width())
        px_h = max(1e-9, self._view_rect.height())

        dlon = -(dx * lon_range / px_w)
        dlat = dy * lat_range / px_h

        self._manual_geo_bounds = (
            lat_min + dlat,
            lat_max + dlat,
            lon_min + dlon,
            lon_max + dlon,
        )
        self._geo_bounds = self._normalize_bounds(self._manual_geo_bounds, self._base_geo_bounds)
        self._pan_last_pos = QPointF(float(pos.x()), float(pos.y()))
        self.update()
        event.accept()

    def mouseReleaseEvent(self, event) -> None:
        if event.button() != Qt.RightButton:
            super().mouseReleaseEvent(event)
            return
        self._is_panning = False
        self._pan_last_pos = None
        self.unsetCursor()
        event.accept()

    def _compute_bounds(self) -> Optional[Tuple[float, float, float, float]]:
        points: List[Tuple[float, float]] = []

        if self._layer_on("show_0203", True) and isinstance(self._mrpk, dict):
            for key in ("takeOverInfoList", "handOverInfoList"):
                infos = self._mrpk.get(key)
                if not isinstance(infos, list):
                    continue
                for item in infos:
                    if not isinstance(item, dict):
                        continue
                    ll = _coord_to_latlon(item.get("coordinate", {}))
                    if ll is not None:
                        points.append(ll)

        if self._layer_on("show_0201", True) and isinstance(self._cmpk, dict):
            missions = self._cmpk.get("inputMissionList")
            if isinstance(missions, list):
                for mission in missions:
                    if not isinstance(mission, dict):
                        continue
                    detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else {}
                    mtype = int(mission.get("inputMissionType", 0) or 0)
                    if mtype in LINE_TYPES:
                        lines = detail.get("lineList") if isinstance(detail, dict) else []
                        if isinstance(lines, list):
                            for line in lines:
                                if not isinstance(line, dict):
                                    continue
                                points.extend(_coords_to_latlon(line.get("coordinateList", [])))
                    elif mtype in AREA_TYPES:
                        areas = detail.get("areaList") if isinstance(detail, dict) else []
                        if isinstance(areas, list):
                            for area in areas:
                                if not isinstance(area, dict):
                                    continue
                                points.extend(_coords_to_latlon(area.get("coordinateList", [])))

        if self._split_result is not None:
            if self._layer_on("show_split", True):
                for piece in self._split_result.pieces:
                    points.extend(_coords_to_latlon(piece.data.get("coordinateList", [])))
                    points.extend(_coords_to_latlon(piece.data.get("rawCoordinateList", [])))
            if self._layer_on("show_direction", True):
                for dbg in self._split_result.directions:
                    for c in (dbg.prev_point, dbg.center_point, dbg.next_point, dbg.line_start, dbg.line_end):
                        ll = _coord_to_latlon(c) if isinstance(c, dict) else None
                        if ll is not None:
                            points.append(ll)
            if self._layer_on("show_expected_paths", True):
                for path in self._split_result.expected_paths:
                    if isinstance(path, dict):
                        points.extend(_coords_to_latlon(path.get("coordinateList", [])))

        if self._layer_on("show_0303", True):
            for row in self._flight_plans_0303:
                if not isinstance(row, dict):
                    continue
                aid = _to_int(row.get("aircraftID"), 0)
                if not self._path_aircraft_visible(aid):
                    continue
                wps = row.get("waypointList")
                if not isinstance(wps, list):
                    continue
                for wp in wps:
                    if not isinstance(wp, dict):
                        continue
                    ll = _coord_to_latlon(wp.get("coordinate", {}))
                    if ll is not None:
                        points.append(ll)

        if self._layer_on("show_0304", True):
            for row in self._flight_plans_0304:
                if not isinstance(row, dict):
                    continue
                aid = _to_int(row.get("aircraftID"), 0)
                if not self._path_aircraft_visible(aid):
                    continue
                wps = row.get("lahWaypointList")
                if not isinstance(wps, list):
                    continue
                for wp in wps:
                    if not isinstance(wp, dict):
                        continue
                    ll = _coord_to_latlon(wp.get("coordinate", {}))
                    if ll is not None:
                        points.append(ll)

        if not points:
            return None

        lats = [p[0] for p in points]
        lons = [p[1] for p in points]
        lat_min, lat_max = min(lats), max(lats)
        lon_min, lon_max = min(lons), max(lons)

        lat_range = max(1e-6, lat_max - lat_min)
        lon_range = max(1e-6, lon_max - lon_min)

        lat_margin = lat_range * 0.12
        lon_margin = lon_range * 0.12

        return (
            lat_min - lat_margin,
            lat_max + lat_margin,
            lon_min - lon_margin,
            lon_max + lon_margin,
        )

    def _normalize_bounds(
        self,
        bounds: Tuple[float, float, float, float],
        fallback: Tuple[float, float, float, float],
    ) -> Tuple[float, float, float, float]:
        lat_min, lat_max, lon_min, lon_max = bounds
        if not all(math.isfinite(v) for v in bounds):
            return fallback
        if lat_max - lat_min < 1e-9 or lon_max - lon_min < 1e-9:
            return fallback
        return (lat_min, lat_max, lon_min, lon_max)

    def _screen_to_geo(
        self,
        sx: float,
        sy: float,
        bounds: Tuple[float, float, float, float],
    ) -> Optional[Tuple[float, float]]:
        if self._view_rect is None:
            return None
        lat_min, lat_max, lon_min, lon_max = bounds
        xr = max(1e-9, lon_max - lon_min)
        yr = max(1e-9, lat_max - lat_min)
        nx = (sx - self._view_rect.left()) / max(1e-9, self._view_rect.width())
        ny = (sy - self._view_rect.top()) / max(1e-9, self._view_rect.height())
        lon = lon_min + nx * xr
        lat = lat_max - ny * yr
        return (lat, lon)

    def _to_screen(self, lat: float, lon: float) -> QPointF:
        if self._geo_bounds is None or self._view_rect is None:
            return QPointF(0.0, 0.0)
        lat_min, lat_max, lon_min, lon_max = self._geo_bounds
        xr = max(1e-9, lon_max - lon_min)
        yr = max(1e-9, lat_max - lat_min)
        nx = (lon - lon_min) / xr
        ny = (lat_max - lat) / yr
        x = self._view_rect.left() + nx * self._view_rect.width()
        y = self._view_rect.top() + ny * self._view_rect.height()
        return QPointF(x, y)

    def _points_to_polygon(self, points: List[Tuple[float, float]]) -> QPolygonF:
        poly = QPolygonF()
        for lat, lon in points:
            poly.append(self._to_screen(lat, lon))
        return poly

    def _draw_grid(self, painter: QPainter) -> None:
        if self._view_rect is None:
            return
        painter.setPen(QPen(QColor("#d6deea"), 1))
        painter.setBrush(Qt.NoBrush)
        painter.drawRoundedRect(self._view_rect, 8, 8)

        step_count = 10
        for i in range(1, step_count):
            t = i / step_count
            x = self._view_rect.left() + t * self._view_rect.width()
            y = self._view_rect.top() + t * self._view_rect.height()
            painter.drawLine(QPointF(x, self._view_rect.top()), QPointF(x, self._view_rect.bottom()))
            painter.drawLine(QPointF(self._view_rect.left(), y), QPointF(self._view_rect.right(), y))

    def _draw_empty_message(self, painter: QPainter) -> None:
        painter.setPen(QPen(QColor("#334155"), 1))
        font = QFont()
        font.setPointSize(12)
        painter.setFont(font)
        painter.drawText(
            self.rect(),
            Qt.AlignCenter,
            "Load 0201/0203 JSON to render missions and split result.",
        )

    def _draw_original_missions(self, painter: QPainter) -> None:
        if not isinstance(self._cmpk, dict):
            return
        missions = self._cmpk.get("inputMissionList")
        if not isinstance(missions, list):
            return

        for order, mission in enumerate(missions, start=1):
            if not isinstance(mission, dict):
                continue
            color = QColor(_pick_color(order - 1))
            mission_id = mission.get("inputMissionID", order)
            mtype = int(mission.get("inputMissionType", 0) or 0)
            detail = mission.get("missionDetail") if isinstance(mission.get("missionDetail"), dict) else {}
            label_point: Optional[Tuple[float, float]] = None

            if mtype in LINE_TYPES:
                lines = detail.get("lineList") if isinstance(detail, dict) else []
                if isinstance(lines, list):
                    for line in lines:
                        if not isinstance(line, dict):
                            continue
                        coords = _coords_to_latlon(line.get("coordinateList", []))
                        if len(coords) < 2:
                            continue
                        label_point = label_point or coords[0]

                        try:
                            width = float(line.get("width", 0.0))
                        except (TypeError, ValueError):
                            width = 0.0
                        if width > 0:
                            corridor = _corridor_polygon(coords, width)
                            if len(corridor) >= 3:
                                poly = self._points_to_polygon(corridor)
                                fill = QColor(color)
                                fill.setAlpha(28)
                                painter.setPen(QPen(color, 1))
                                painter.setBrush(fill)
                                painter.drawPolygon(poly)

                        painter.setPen(QPen(color, 3))
                        painter.setBrush(Qt.NoBrush)
                        for i in range(len(coords) - 1):
                            p1 = self._to_screen(coords[i][0], coords[i][1])
                            p2 = self._to_screen(coords[i + 1][0], coords[i + 1][1])
                            painter.drawLine(p1, p2)

            elif mtype in AREA_TYPES:
                areas = detail.get("areaList") if isinstance(detail, dict) else []
                if isinstance(areas, list):
                    for area in areas:
                        if not isinstance(area, dict):
                            continue
                        coords = _coords_to_latlon(area.get("coordinateList", []))
                        if len(coords) < 3:
                            continue
                        label_point = label_point or _centroid(coords)
                        poly = self._points_to_polygon(coords)
                        fill = QColor(color)
                        fill.setAlpha(42)
                        painter.setPen(QPen(color, 2))
                        painter.setBrush(fill)
                        painter.drawPolygon(poly)

            if label_point is not None:
                self._draw_mission_label(
                    painter,
                    label_point,
                    f"M{order}",
                    color,
                    f"ID={mission_id} T={mtype}",
                )

    def _draw_mission_label(
        self,
        painter: QPainter,
        ll: Tuple[float, float],
        label: str,
        color: QColor,
        sub: str,
    ) -> None:
        p = self._to_screen(ll[0], ll[1])
        box = QRectF(p.x() - 22.0, p.y() - 14.0, 44.0, 24.0)
        painter.setPen(QPen(color, 1))
        painter.setBrush(QColor(255, 255, 255, 220))
        painter.drawRoundedRect(box, 8, 8)

        font = QFont()
        font.setBold(True)
        font.setPointSize(9)
        painter.setFont(font)
        painter.setPen(QPen(color, 1))
        painter.drawText(box, Qt.AlignCenter, label)

        painter.setPen(QPen(QColor("#334155"), 1))
        info_box = QRectF(p.x() + 10.0, p.y() - 28.0, 90.0, 16.0)
        painter.drawText(info_box, Qt.AlignLeft | Qt.AlignVCenter, sub)

    def _draw_0203_points(self, painter: QPainter) -> None:
        if not isinstance(self._mrpk, dict):
            return

        self._draw_info_points(
            painter,
            self._mrpk.get("takeOverInfoList"),
            QColor("#0d47a1"),
            "TO",
        )
        self._draw_info_points(
            painter,
            self._mrpk.get("handOverInfoList"),
            QColor("#1b5e20"),
            "HO",
        )

    def _draw_info_points(
        self,
        painter: QPainter,
        points: Any,
        color: QColor,
        prefix: str,
    ) -> None:
        if not isinstance(points, list):
            return
        font = QFont()
        font.setPointSize(8)
        painter.setFont(font)
        for idx, item in enumerate(points, start=1):
            if not isinstance(item, dict):
                continue
            ll = _coord_to_latlon(item.get("coordinate", {}))
            if ll is None:
                continue
            p = self._to_screen(ll[0], ll[1])
            painter.setPen(QPen(color, 1))
            painter.setBrush(color)
            painter.drawEllipse(p, 4.5, 4.5)
            painter.drawText(QRectF(p.x() + 6.0, p.y() - 8.0, 40.0, 16.0), Qt.AlignLeft | Qt.AlignVCenter, f"{prefix}{idx}")

    def _draw_split_result(self, painter: QPainter) -> None:
        if self._split_result is None:
            return

        for piece in self._split_result.pieces:
            color = QColor(_pick_color(piece.parent_order - 1))
            raw_coords = _coords_to_latlon(piece.data.get("rawCoordinateList", []))
            if len(raw_coords) >= 3:
                raw_poly = self._points_to_polygon(raw_coords)
                faint_fill = QColor(color)
                faint_fill.setAlpha(14)
                faint_pen = QPen(QColor(color), 1)
                faint_pen.setStyle(Qt.DotLine)
                painter.setPen(faint_pen)
                painter.setBrush(faint_fill)
                painter.drawPolygon(raw_poly)

            coords = _coords_to_latlon(piece.data.get("coordinateList", []))
            if len(coords) < 3:
                continue
            poly = self._points_to_polygon(coords)
            fill = QColor(color)
            fill.setAlpha(82)
            painter.setPen(QPen(QColor("#111827"), 1))
            painter.setBrush(fill)
            painter.drawPolygon(poly)

            c = _centroid(coords)
            cp = self._to_screen(c[0], c[1])
            if self._layer_on("show_direction", True):
                self._draw_piece_local_direction(painter, piece, cp)
            u_label = str(piece.assigned_uav) if (piece.assigned_uav is not None and piece.assigned_uav > 0) else "-"
            txt = f"{piece.parent_order}-{piece.piece_index}/U{u_label}"
            stage = piece.data.get("splitStage")
            if isinstance(stage, int) and stage >= 1:
                txt += f" S{stage}"
            painter.setPen(QPen(QColor("#0f172a"), 1))
            painter.setBrush(QColor(255, 255, 255, 205))
            painter.drawRoundedRect(QRectF(cp.x() - 42.0, cp.y() - 10.0, 84.0, 18.0), 5, 5)
            painter.drawText(QRectF(cp.x() - 42.0, cp.y() - 10.0, 84.0, 18.0), Qt.AlignCenter, txt)

            # Show type-decider result when available.
            im_type = piece.data.get("individualMissionType")
            pat_type = piece.data.get("patternType")
            try:
                im_val = int(im_type)
            except Exception:
                im_val = 0
            try:
                pat_val = int(pat_type)
            except Exception:
                pat_val = 0
            if im_val > 0 or pat_val > 0:
                tp_txt = f"T{im_val}/P{pat_val}"
                rect = QRectF(cp.x() - 34.0, cp.y() + 10.0, 68.0, 14.0)
                painter.setPen(QPen(QColor("#1f2937"), 1))
                painter.setBrush(QColor(255, 255, 255, 210))
                painter.drawRoundedRect(rect, 4, 4)
                tp_font = QFont()
                tp_font.setPointSize(7)
                painter.setFont(tp_font)
                painter.drawText(rect, Qt.AlignCenter, tp_txt)

            # Show expected velocity/time range when available.
            exp_vel = piece.data.get("expVel") if isinstance(piece.data, dict) else None
            if isinstance(exp_vel, dict):
                vmin = float(exp_vel.get("velMinKmh", 0.0) or 0.0)
                vmax = float(exp_vel.get("velMaxKmh", 0.0) or 0.0)
                wref = float(exp_vel.get("widthRefM", 0.0) or 0.0)
                v_approx = bool(exp_vel.get("velApprox", False))
                tmin_s = exp_vel.get("timeMinSec")
                tmax_s = exp_vel.get("timeMaxSec")
                if tmin_s is None or tmax_s is None:
                    tmin_m = exp_vel.get("timeMinMin")
                    tmax_m = exp_vel.get("timeMaxMin")
                    if tmin_m is not None and tmax_m is not None:
                        tmin_s = float(tmin_m) * 60.0
                        tmax_s = float(tmax_m) * 60.0
                if vmin > 0.0 and vmax > 0.0:
                    prefix = "V~" if v_approx else "V"
                    vel_txt = f"{prefix}{vmin:.0f}~{vmax:.0f} W{wref:.0f}m"
                else:
                    vel_txt = f"V- W{wref:.0f}m"
                time_txt = ""
                g_leader = bool(exp_vel.get("areaTimeGroupLeader", True))
                g_count = int(exp_vel.get("areaTimeGroupCount", 1) or 1)
                g_tmin_s = exp_vel.get("groupTimeMinSec")
                g_tmax_s = exp_vel.get("groupTimeMaxSec")
                if _piece_is_area_like(piece) and g_count > 1:
                    if g_leader and g_tmin_s is not None and g_tmax_s is not None:
                        time_txt = f"TΣ{float(g_tmin_s):.0f}~{float(g_tmax_s):.0f}s"
                    else:
                        time_txt = ""
                elif tmin_s is not None and tmax_s is not None:
                    time_txt = f"T{float(tmin_s):.0f}~{float(tmax_s):.0f}s"

                v_rect = QRectF(cp.x() - 52.0, cp.y() + 26.0, 104.0, 14.0)
                painter.setPen(QPen(QColor("#1f2937"), 1))
                painter.setBrush(QColor(255, 255, 255, 214))
                painter.drawRoundedRect(v_rect, 4, 4)
                vt_font = QFont()
                vt_font.setPointSize(7)
                painter.setFont(vt_font)
                painter.drawText(v_rect, Qt.AlignCenter, vel_txt)

                if time_txt:
                    t_rect = QRectF(cp.x() - 52.0, cp.y() + 40.0, 104.0, 14.0)
                    painter.setPen(QPen(QColor("#1f2937"), 1))
                    painter.setBrush(QColor(255, 255, 255, 214))
                    painter.drawRoundedRect(t_rect, 4, 4)
                    painter.drawText(t_rect, Qt.AlignCenter, time_txt)

    def _piece_bearing_deg(self, piece: Any, *keys: str) -> Optional[float]:
        data = getattr(piece, "data", None)
        if not isinstance(data, dict):
            return None
        for key in keys:
            if key not in data:
                continue
            try:
                return float(data.get(key))
            except Exception:
                continue
        return None

    def _draw_piece_local_direction(self, painter: QPainter, piece: Any, center_pt: QPointF) -> None:
        move_deg = self._piece_bearing_deg(piece, "phaseMoveBearing_deg", "bearing_deg", "bearingFromPrev")
        split_deg = self._piece_bearing_deg(piece, "boundaryAxisBearing_deg")
        if move_deg is None and split_deg is None:
            return

        def _draw_arrow(bearing_deg: float, length_m: float, color: str, width: float) -> None:
            dn, de = _bearing_to_vec(float(bearing_deg), float(length_m))
            tip = QPointF(center_pt.x() + de, center_pt.y() - dn)
            painter.setPen(QPen(QColor(color), width))
            painter.drawLine(center_pt, tip)
            head_l = QPointF(
                tip.x() - (tip.x() - center_pt.x()) * 0.28 - (tip.y() - center_pt.y()) * 0.16,
                tip.y() - (tip.y() - center_pt.y()) * 0.28 + (tip.x() - center_pt.x()) * 0.16,
            )
            head_r = QPointF(
                tip.x() - (tip.x() - center_pt.x()) * 0.28 + (tip.y() - center_pt.y()) * 0.16,
                tip.y() - (tip.y() - center_pt.y()) * 0.28 - (tip.x() - center_pt.x()) * 0.16,
            )
            painter.drawLine(tip, head_l)
            painter.drawLine(tip, head_r)

        if split_deg is not None:
            _draw_arrow(float(split_deg), 12.0, "#f59e0b", 1.4)
        if move_deg is not None:
            _draw_arrow(float(move_deg), 16.0, "#2563eb", 1.8)

    def _draw_expected_paths(self, painter: QPainter) -> None:
        if self._split_result is None:
            return
        for row in self._split_result.expected_paths:
            if not isinstance(row, dict):
                continue
            coords = _coords_to_latlon(row.get("coordinateList", []))
            if len(coords) < 2:
                continue

            pen = QPen(QColor("#1f2937"), 2)
            pen.setStyle(Qt.DashDotLine)
            painter.setPen(pen)
            painter.setBrush(Qt.NoBrush)
            for i in range(len(coords) - 1):
                p1 = self._to_screen(coords[i][0], coords[i][1])
                p2 = self._to_screen(coords[i + 1][0], coords[i + 1][1])
                painter.drawLine(p1, p2)

            mid = coords[len(coords) // 2]
            mp = self._to_screen(mid[0], mid[1])
            parent = int(row.get("parentOrder", 0) or 0)
            idx = int(row.get("index", 0) or 0)
            set_name = str(row.get("setName", f"E{parent}-{idx}"))
            lb_color = QColor("#111827")
            painter.setPen(QPen(lb_color, 1))
            painter.setBrush(QColor(255, 255, 255, 210))
            box_w = max(56.0, min(120.0, 8.0 * float(len(set_name)) + 18.0))
            painter.drawRoundedRect(QRectF(mp.x() - (box_w * 0.5), mp.y() - 10.0, box_w, 18.0), 5, 5)
            painter.drawText(
                QRectF(mp.x() - (box_w * 0.5), mp.y() - 10.0, box_w, 18.0),
                Qt.AlignCenter,
                set_name,
            )

            point_list = row.get("pointList")
            if not isinstance(point_list, list):
                continue
            pt_font = QFont()
            pt_font.setPointSize(7)
            painter.setFont(pt_font)
            for item in point_list:
                if not isinstance(item, dict):
                    continue
                ll = _coord_to_latlon(item.get("coordinate", {}))
                if ll is None:
                    continue
                pp = self._to_screen(ll[0], ll[1])
                role = str(item.get("role", ""))
                if role == "start":
                    fill = QColor("#ef4444")
                elif role == "mid":
                    fill = QColor("#2563eb")
                else:
                    fill = QColor("#10b981")
                painter.setPen(QPen(QColor("#0f172a"), 1))
                painter.setBrush(fill)
                painter.drawEllipse(QRectF(pp.x() - 3.2, pp.y() - 3.2, 6.4, 6.4))

                name = str(item.get("name", ""))
                if not name:
                    continue
                w = max(52.0, min(120.0, 6.6 * float(len(name)) + 10.0))
                rect = QRectF(pp.x() + 4.0, pp.y() - 12.0, w, 14.0)
                painter.setPen(QPen(QColor("#111827"), 1))
                painter.setBrush(QColor(255, 255, 255, 215))
                painter.drawRoundedRect(rect, 4, 4)
                painter.drawText(rect, Qt.AlignCenter, name)

    def _draw_direction_debugs(self, painter: QPainter) -> None:
        if self._split_result is None:
            return
        for dbg in self._split_result.directions:
            self._draw_direction_debug(painter, dbg)

    def _draw_direction_debug(self, painter: QPainter, dbg: DirectionDebug) -> None:
        if dbg.prev_point and dbg.center_point:
            p = _coord_to_latlon(dbg.prev_point)
            c = _coord_to_latlon(dbg.center_point)
            if p is not None and c is not None:
                self._draw_arrow_line(
                    painter,
                    self._to_screen(p[0], p[1]),
                    self._to_screen(c[0], c[1]),
                    QColor("#ff1744"),
                    width=2,
                )

        if dbg.center_point and dbg.next_point:
            c = _coord_to_latlon(dbg.center_point)
            n = _coord_to_latlon(dbg.next_point)
            if c is not None and n is not None:
                self._draw_arrow_line(
                    painter,
                    self._to_screen(c[0], c[1]),
                    self._to_screen(n[0], n[1]),
                    QColor("#ff8f00"),
                    width=2,
                )

        if dbg.center_point and dbg.bearing_split_deg is not None:
            c = _coord_to_latlon(dbg.center_point)
            if c is not None:
                n1, e1 = _bearing_to_vec(dbg.bearing_split_deg, 2200.0)
                n2, e2 = _bearing_to_vec((dbg.bearing_split_deg + 180.0) % 360.0, 2200.0)
                p1_ll = _offset_latlon(c[0], c[1], north_m=n1, east_m=e1)
                p2_ll = _offset_latlon(c[0], c[1], north_m=n2, east_m=e2)
                p1 = self._to_screen(p1_ll[0], p1_ll[1])
                p2 = self._to_screen(p2_ll[0], p2_ll[1])
                pen = QPen(QColor("#6a1b9a"), 2)
                pen.setStyle(Qt.DashLine)
                painter.setPen(pen)
                painter.setBrush(Qt.NoBrush)
                painter.drawLine(p1, p2)

        if dbg.line_start and dbg.line_end:
            s = _coord_to_latlon(dbg.line_start)
            e = _coord_to_latlon(dbg.line_end)
            if s is not None and e is not None:
                self._draw_arrow_line(
                    painter,
                    self._to_screen(s[0], s[1]),
                    self._to_screen(e[0], e[1]),
                    QColor("#00acc1"),
                    width=2,
                )

    def _draw_0303_paths(self, painter: QPainter) -> None:
        if not isinstance(self._flight_plans_0303, list):
            return
        show_route = self._layer_on("show_0303_route", True)
        show_sweep = self._layer_on("show_0303_sweep", True)
        if not show_route and not show_sweep:
            return
        font = QFont()
        font.setPointSize(7)
        painter.setFont(font)
        for row in self._flight_plans_0303:
            if not isinstance(row, dict):
                continue
            aid = _to_int(row.get("aircraftID"), 0)
            if not self._path_aircraft_visible(aid):
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
                ll = _coord_to_latlon(wp.get("coordinate", {}))
                if ll is not None:
                    coords.append(ll)
                fp = wp.get("filmingProperty")
                if isinstance(fp, dict):
                    ls = fp.get("lineSearch")
                    if isinstance(ls, dict):
                        ls_coords = _coords_to_latlon(ls.get("coordinateList", []))
                        if len(ls_coords) >= 2:
                            interp_points = _to_int(ls.get("interpolationPoints"), 0)
                            sweep_lines.append((ls_coords, int(interp_points)))
                    # Nadir-point style sweep target (coordinateOrientation) for P3/P4.
                    cor = fp.get("coordinateOrientation")
                    if isinstance(cor, dict):
                        tgt = _coord_to_latlon(cor.get("coordinate", {}))
                        if tgt is not None:
                            sweep_points.append(tgt)
            if len(coords) < 2:
                continue

            # Route line (waypoint-to-waypoint) as a light context line.
            if show_route:
                route_pen = QPen(QColor("#64748b"), 1)
                route_pen.setStyle(Qt.DashLine)
                painter.setPen(route_pen)
                painter.setBrush(Qt.NoBrush)
                for i in range(len(coords) - 1):
                    a = coords[i]
                    b = coords[i + 1]
                    # Sweep-carrying missions often keep only anchor WPs, which makes
                    # route-only rendering look like tiny detached dashes. Hide those
                    # very short anchor links in route layer; sweep layer still shows
                    # the real scan geometry.
                    if (sweep_lines or sweep_points) and _segment_len_m(a, b) < 150.0:
                        continue
                    p1 = self._to_screen(a[0], a[1])
                    p2 = self._to_screen(b[0], b[1])
                    painter.drawLine(p1, p2)

            # Sweep lines from lineSearch are the real scan pattern.
            if show_sweep and sweep_lines:
                sweep_pen = QPen(QColor("#0f766e"), 2)
                sweep_pen.setStyle(Qt.SolidLine)
                painter.setPen(sweep_pen)
                for ls_coords, interp_points in sweep_lines:
                    chunk = max(2, int(interp_points))
                    # lineSearch may store many sweep lines concatenated as
                    # [line1_points..., line2_points..., ...].
                    # Draw each chunk independently so we do not connect
                    # end(line_k) -> start(line_k+1) with a fake bridge.
                    if chunk <= 2 or len(ls_coords) <= chunk:
                        for i in range(len(ls_coords) - 1):
                            p1 = self._to_screen(ls_coords[i][0], ls_coords[i][1])
                            p2 = self._to_screen(ls_coords[i + 1][0], ls_coords[i + 1][1])
                            painter.drawLine(p1, p2)
                    else:
                        for base in range(0, len(ls_coords), chunk):
                            seg = ls_coords[base:base + chunk]
                            if len(seg) < 2:
                                continue
                            for i in range(len(seg) - 1):
                                p1 = self._to_screen(seg[i][0], seg[i][1])
                                p2 = self._to_screen(seg[i + 1][0], seg[i + 1][1])
                                painter.drawLine(p1, p2)
            elif show_sweep and len(sweep_points) >= 2:
                # For nadir P3/P4: show sweep(target) separately from flight route.
                sweep_pen = QPen(QColor("#0f766e"), 2)
                sweep_pen.setStyle(Qt.SolidLine)
                painter.setPen(sweep_pen)
                for i in range(len(sweep_points) - 1):
                    p1 = self._to_screen(sweep_points[i][0], sweep_points[i][1])
                    p2 = self._to_screen(sweep_points[i + 1][0], sweep_points[i + 1][1])
                    painter.drawLine(p1, p2)
            elif show_sweep:
                # Fallback for non-lineSearch paths (e.g., nadir point mode only).
                pen = QPen(QColor("#0f766e"), 2)
                pen.setStyle(Qt.SolidLine)
                painter.setPen(pen)
                for i in range(len(coords) - 1):
                    p1 = self._to_screen(coords[i][0], coords[i][1])
                    p2 = self._to_screen(coords[i + 1][0], coords[i + 1][1])
                    painter.drawLine(p1, p2)

            s = self._to_screen(coords[0][0], coords[0][1])
            e = self._to_screen(coords[-1][0], coords[-1][1])
            painter.setPen(QPen(QColor("#065f46"), 1))
            painter.setBrush(QColor("#22c55e"))
            painter.drawEllipse(QRectF(s.x() - 3.0, s.y() - 3.0, 6.0, 6.0))
            painter.setBrush(QColor("#ef4444"))
            painter.drawEllipse(QRectF(e.x() - 3.0, e.y() - 3.0, 6.0, 6.0))

            label_pool = coords
            if sweep_lines:
                label_pool = []
                for ls_coords, _interp_points in sweep_lines:
                    label_pool.extend(ls_coords)
            elif sweep_points:
                label_pool = sweep_points
            mid = label_pool[len(label_pool) // 2]
            mp = self._to_screen(mid[0], mid[1])
            aid = _to_int(row.get("aircraftID"), 0)
            pid = _to_int(row.get("pathID"), 0)
            label = f"0303 A{aid}/P{pid}" if aid > 0 and pid > 0 else "0303"
            box_w = max(84.0, min(132.0, 7.0 * float(len(label)) + 14.0))
            rect = QRectF(mp.x() - (box_w * 0.5), mp.y() - 10.0, box_w, 18.0)
            painter.setPen(QPen(QColor("#0f172a"), 1))
            painter.setBrush(QColor(255, 255, 255, 210))
            painter.drawRoundedRect(rect, 5, 5)
            painter.drawText(rect, Qt.AlignCenter, label)

    def _draw_0304_paths(self, painter: QPainter) -> None:
        if not isinstance(self._flight_plans_0304, list):
            return
        font = QFont()
        font.setPointSize(7)
        painter.setFont(font)
        for row in self._flight_plans_0304:
            if not isinstance(row, dict):
                continue
            aid = _to_int(row.get("aircraftID"), 0)
            if not self._path_aircraft_visible(aid):
                continue
            wps = row.get("lahWaypointList")
            if not isinstance(wps, list):
                continue
            coords: List[Tuple[float, float]] = []
            for wp in wps:
                if not isinstance(wp, dict):
                    continue
                ll = _coord_to_latlon(wp.get("coordinate", {}))
                if ll is not None:
                    coords.append(ll)
            if len(coords) < 2:
                continue

            pen = QPen(QColor("#a16207"), 2)
            pen.setStyle(Qt.DashLine)
            painter.setPen(pen)
            painter.setBrush(Qt.NoBrush)
            for i in range(len(coords) - 1):
                p1 = self._to_screen(coords[i][0], coords[i][1])
                p2 = self._to_screen(coords[i + 1][0], coords[i + 1][1])
                painter.drawLine(p1, p2)

            mid = coords[len(coords) // 2]
            mp = self._to_screen(mid[0], mid[1])
            aid = _to_int(row.get("aircraftID"), 0)
            pid = _to_int(row.get("pathID"), 0)
            label = f"0304 A{aid}/P{pid}" if aid > 0 and pid > 0 else "0304"
            box_w = max(84.0, min(132.0, 7.0 * float(len(label)) + 14.0))
            rect = QRectF(mp.x() - (box_w * 0.5), mp.y() - 10.0, box_w, 18.0)
            painter.setPen(QPen(QColor("#78350f"), 1))
            painter.setBrush(QColor(255, 255, 255, 205))
            painter.drawRoundedRect(rect, 5, 5)
            painter.drawText(rect, Qt.AlignCenter, label)

    def _draw_arrow_line(
        self,
        painter: QPainter,
        p1: QPointF,
        p2: QPointF,
        color: QColor,
        width: int = 2,
    ) -> None:
        painter.setPen(QPen(color, width))
        painter.setBrush(Qt.NoBrush)
        painter.drawLine(p1, p2)

        dx = p2.x() - p1.x()
        dy = p2.y() - p1.y()
        length = math.hypot(dx, dy)
        if length < 1e-6:
            return
        ux = dx / length
        uy = dy / length
        head = 11.0
        wing = 5.0
        bx = p2.x() - ux * head
        by = p2.y() - uy * head
        wx = -uy * wing
        wy = ux * wing

        left = QPointF(bx + wx, by + wy)
        right = QPointF(bx - wx, by - wy)
        poly = QPolygonF([p2, left, right])
        painter.setPen(QPen(color, 1))
        painter.setBrush(color)
        painter.drawPolygon(poly)

    def _draw_legend(self, painter: QPainter) -> None:
        pieces = len(self._split_result.pieces) if self._split_result is not None else 0
        uavs = self._split_result.uav_count if self._split_result is not None else 0
        exp_paths = len(self._split_result.expected_paths) if self._split_result is not None else 0
        fp_0303 = len(self._flight_plans_0303) if isinstance(self._flight_plans_0303, list) else 0
        fp_0304 = len(self._flight_plans_0304) if isinstance(self._flight_plans_0304, list) else 0

        x, y, w, h = 14.0, 14.0, 390.0, 166.0
        painter.setPen(QPen(QColor("#334155"), 1))
        painter.setBrush(QColor(255, 255, 255, 230))
        painter.drawRoundedRect(QRectF(x, y, w, h), 7, 7)

        painter.setPen(QPen(QColor("#0f172a"), 1))
        title_font = QFont()
        title_font.setPointSize(9)
        title_font.setBold(True)
        painter.setFont(title_font)
        painter.drawText(QRectF(x + 10.0, y + 6.0, w - 20.0, 18.0), Qt.AlignLeft | Qt.AlignVCenter, "Mission Split Tester")

        txt_font = QFont()
        txt_font.setPointSize(8)
        painter.setFont(txt_font)
        painter.drawText(QRectF(x + 10.0, y + 25.0, w - 20.0, 16.0), Qt.AlignLeft | Qt.AlignVCenter, "0201: mission order color-coded")
        painter.drawText(QRectF(x + 10.0, y + 42.0, w - 20.0, 16.0), Qt.AlignLeft | Qt.AlignVCenter, "Split before/after: faint dotted(raw) + dark filled(processed)")
        painter.drawText(QRectF(x + 10.0, y + 59.0, w - 20.0, 16.0), Qt.AlignLeft | Qt.AlignVCenter, "Direction: red(entry), orange(exit), purple(boundary), cyan(line)")
        painter.drawText(QRectF(x + 10.0, y + 76.0, w - 20.0, 16.0), Qt.AlignLeft | Qt.AlignVCenter, "Area split: two-stage (entry-side + beyond-boundary side)")
        painter.drawText(QRectF(x + 10.0, y + 93.0, w - 20.0, 16.0), Qt.AlignLeft | Qt.AlignVCenter, "Expected path + velocity range: dash-dot path, Vmin~max, Tmin~max")
        painter.drawText(
            QRectF(x + 10.0, y + 110.0, w - 20.0, 16.0),
            Qt.AlignLeft | Qt.AlignVCenter,
            "0303: route=light dashed, sweep(lineSearch/point-target)=solid teal",
        )
        painter.drawText(
            QRectF(x + 10.0, y + 127.0, w - 20.0, 16.0),
            Qt.AlignLeft | Qt.AlignVCenter,
            f"Pieces={pieces}, UAV={uavs}, ExpPath={exp_paths}, 0303={fp_0303}, 0304={fp_0304}",
        )
        layer_state = (
            f"L(0201:{int(self._layer_on('show_0201'))},0203:{int(self._layer_on('show_0203'))},"
            f"S:{int(self._layer_on('show_split'))},D:{int(self._layer_on('show_direction'))},"
            f"E:{int(self._layer_on('show_expected_paths'))},R3:{int(self._layer_on('show_0303_route'))},"
            f"SW3:{int(self._layer_on('show_0303_sweep'))},"
            f"F4:{int(self._layer_on('show_0304'))})"
        )
        painter.drawText(QRectF(x + 10.0, y + 144.0, w - 20.0, 16.0), Qt.AlignLeft | Qt.AlignVCenter, layer_state)
