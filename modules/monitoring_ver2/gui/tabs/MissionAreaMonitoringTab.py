# -*- coding: utf-8 -*-
from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Iterable, Optional

from PyQt5.QtCore import Qt, QPointF, QRectF
from PyQt5.QtGui import (
    QPixmap,
    QPainter,
    QImage,
    QColor,
    QPen,
    QBrush,
    QPolygonF,
    QPainterPath,
)
from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QLabel,
    QGroupBox,
    QFormLayout,
    QSplitter,
    QHBoxLayout,
    QFrame,
    QSizePolicy,
    QGraphicsView,
    QGraphicsScene,
    QGraphicsPixmapItem,
)

from modules.common import db_paths

PROJECT_ROOT = Path(__file__).resolve().parents[4]

MISSION_TYPE_STYLES = {
    3: {"stroke": "#ff7043", "fill": "#ffd8c2", "width": 2.2, "label": "협업기저임무"},
    6: {"stroke": "#1565c0", "fill": None, "width": 2.0, "label": "Sweep Line"},
    7: {"stroke": "#2e7d32", "fill": "#b9e4c9", "width": 1.6, "label": "Sweep Area"},
    9: {"stroke": "#c2185b", "fill": "#f8bbd0", "width": 1.4, "label": "WP"},
}
DEFAULT_MISSION_STYLE = {"stroke": "#5e35b1", "fill": "#d1c4e9", "width": 1.2, "label": "Mission"}


class MissionAreaMonitoringTab(QWidget):
    """Mission-area monitor that shows system context plus a zoomable map preview."""

    def __init__(self, manager, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.manager = manager

        self._plan_label = QLabel("-")
        self._mode_label = QLabel("-")
        self._active_input_label = QLabel("-")
        self._aircraft_label = QLabel("-")
        self._missions_label = QLabel("-")
        self._waypoints_label = QLabel("-")
        self._updated_label = QLabel("-")
        self._image_status_label = QLabel("-")

        self._image_view = _ZoomableImageView()
        self._image_loaded = False
        self._pixmap_cache: Optional[QPixmap] = None
        self._image_bytes: Optional[bytes] = None
        self._dem_bounds: Optional[tuple[float, float, float, float]] = None
        self._dem_size: Optional[tuple[int, int]] = None
        self._dem_transform = None
        self._dem_inv_transform = None
        self._dem_crs = None
        self._mission_file_cache: Dict[int, list] = {}
        self._mission_output_index: Optional[Dict[int, Path]] = None
        self._last_plan_id: Optional[int] = None

        self._init_ui()
        self._load_resource_image()
        self.refresh_display()

    # ------------------------------------------------------------------ UI
    def _init_ui(self) -> None:

        def _value_label() -> QLabel:
            lbl = QLabel("-")
            lbl.setAlignment(Qt.AlignLeft)
            lbl.setStyleSheet("font-weight:600;")
            return lbl

        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 20, 24, 20)
        layout.setSpacing(18)

        title = QLabel("Stack 기반 모니터링")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size:17px; font-weight:600;")

        subtitle = QLabel("상단에서는 운용 모드 및 계획 정보를, 하단에서는 지형 이미지를 확인할 수 있습니다.")
        subtitle.setAlignment(Qt.AlignCenter)
        subtitle.setWordWrap(True)
        subtitle.setStyleSheet("color:#666;")

        info_box = QGroupBox("시스템 · 임무 요약")
        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignRight)
        form.setFormAlignment(Qt.AlignTop)
        form.addRow("시스템 운용 모드:", self._mode_label)
        form.addRow("MissionPlan ID:", self._plan_label)
        form.addRow("활성 Input ID:", self._active_input_label)
        form.addRow("기체 수:", self._aircraft_label)
        form.addRow("개별 임무 수:", self._missions_label)
        form.addRow("웨이포인트 수:", self._waypoints_label)
        form.addRow("최근 갱신:", self._updated_label)
        info_box.setLayout(form)

        splitter = QSplitter(Qt.Horizontal)
        splitter.setChildrenCollapsible(False)

        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(6)

        map_header = QHBoxLayout()
        map_title = QLabel("지형 미리보기 (비활성화)")
        map_title.setStyleSheet("font-weight:600;")
        map_header.addWidget(map_title)
        map_header.addStretch(1)
        self._image_status_label.setStyleSheet("color:#666;")
        map_header.addWidget(self._image_status_label)

        view_hint = QLabel("휠 : 확대/축소 · 드래그 : 이동")
        view_hint.setStyleSheet("color:#777; font-size:12px;")

        left_layout.addLayout(map_header)
        left_layout.addWidget(self._image_view, stretch=1)
        left_layout.addWidget(view_hint)

        right_placeholder = QFrame()
        right_layout = QVBoxLayout(right_placeholder)
        right_layout.setContentsMargins(12, 12, 12, 12)
        right_layout.addStretch(1)
        empty_label = QLabel("향후 임무 영역 분석 패널이 추가될 예정입니다.")
        empty_label.setWordWrap(True)
        empty_label.setAlignment(Qt.AlignCenter)
        empty_label.setStyleSheet("color:#888;")
        right_layout.addWidget(empty_label)
        right_layout.addStretch(2)

        splitter.addWidget(left_panel)
        splitter.addWidget(right_placeholder)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

        layout.addWidget(title)
        layout.addWidget(subtitle)
        layout.addWidget(info_box)
        layout.addWidget(splitter, stretch=1)

    # ------------------------------------------------------------------ Data refresh
    def refresh_display(self, update_info: Iterable[Any] | None = None, payload: object | None = None) -> None:
        context = self._current_plan_context()

        plan_id = self._to_int(context.get("missionPlanID"))
        active_input = context.get("activeInputMissionID")
        if active_input is None:
            active_input = self.manager.logic_store.get_data("active_input_mission_id")
        active_input = self._to_int(active_input)

        aircraft_data = context.get("aircraft") or {}
        aircraft_count = len(aircraft_data)
        mission_count = 0
        waypoint_count = 0
        for aircraft_payload in aircraft_data.values():
            missions = aircraft_payload.get("missions") or []
            mission_count += len(missions)
            for mission in missions:
                waypoint_count += len(mission.get("waypoints") or [])

        self._plan_label.setText(self._fmt(plan_id))
        self._active_input_label.setText(self._fmt(active_input))
        self._aircraft_label.setText(self._fmt(aircraft_count))
        self._missions_label.setText(self._fmt(mission_count))
        self._waypoints_label.setText(self._fmt(waypoint_count))
        self._updated_label.setText(self._format_update_source(update_info))
        self._mode_label.setText(self._current_mode_text())
        self._update_mission_layers(plan_id, aircraft_data)

    # ------------------------------------------------------------------ Helpers
    def _current_plan_context(self) -> Dict[str, Any]:
        context = self.manager.logic_store.get_data("current_mission_plan") or {}
        return context if isinstance(context, dict) else {}

    def _current_mode_text(self) -> str:
        mode_value = self.manager.logic_store.get_data("SystemMode")
        try:
            mode_int = int(mode_value)
        except (TypeError, ValueError):
            return "-"
        mode_map = {
            0: "OFF/초기화",
            1: "대기",
            2: "초기 임무 계획",
            3: "임무 수행",
            4: "특수 모드",
            5: "전원 OFF",
        }
        return f"{mode_int} ({mode_map.get(mode_int, '미정')})"

    def _update_mission_layers(self, plan_id: Optional[int], aircraft_data: Dict[str, Any]) -> None:
        if not self._pixmap_cache or not self._dem_bounds:
            self._image_view.clear_overlays()
            return
        if plan_id is not None and plan_id != self._last_plan_id:
            self._mission_file_cache.clear()
        self._last_plan_id = plan_id

        geometries = self._collect_mission_geometries(aircraft_data)
        self._image_view.clear_overlays()
        if not geometries:
            return

        for geom in geometries:
            points = geom.get("points") or []
            if not points:
                continue
            style = geom.get("style") or DEFAULT_MISSION_STYLE
            stroke = style.get("stroke", DEFAULT_MISSION_STYLE["stroke"])
            width = style.get("width", DEFAULT_MISSION_STYLE["width"])
            fill_color = style.get("fill")
            opacity = geom.get("fill_opacity", 0.35)
            pen = self._make_pen(stroke, width, geom.get("dashed", False))
            brush = self._make_brush(fill_color if geom.get("fill", True) else None, opacity)
            if geom.get("type") == "point":
                radius = style.get("radius", 4.5)
                self._image_view.add_point_item(
                    points[0],
                    radius,
                    pen.color(),
                    brush.color() if brush.style() != Qt.NoBrush else pen.color(),
                )
                continue

            path = QPainterPath()
            path.moveTo(points[0])
            for pt in points[1:]:
                path.lineTo(pt)
            if geom.get("close"):
                path.closeSubpath()
            self._image_view.add_path_item(path, pen, brush)

    def _collect_mission_geometries(self, aircraft_data: Dict[str, Any]) -> list[Dict[str, Any]]:
        geometries: list[Dict[str, Any]] = []
        for aircraft_id, payload in (aircraft_data or {}).items():
            package_id = payload.get("individualMissionPackageID")
            missions = payload.get("missions") or []
            plan_entries = self._load_individual_plan_entries(package_id)
            if not plan_entries:
                continue

            for idx, mission in enumerate(missions):
                entry = plan_entries[idx] if idx < len(plan_entries) else None
                if not entry:
                    continue
                info = entry.get("individualMissionInfo") or {}
                mission_type = self._to_int(info.get("individualMissionType"))
                style = dict(DEFAULT_MISSION_STYLE)
                if mission_type in MISSION_TYPE_STYLES:
                    style.update(MISSION_TYPE_STYLES[mission_type])
                geometries.extend(
                    self._build_shapes_from_info(
                        info=info,
                        mission_type=mission_type,
                        style=style,
                    )
                )
        return geometries

    def _build_shapes_from_info(
        self,
        *,
        info: Dict[str, Any],
        mission_type: Optional[int],
        style: Dict[str, Any],
    ) -> list[Dict[str, Any]]:
        shapes: list[Dict[str, Any]] = []
        prefer_closed = mission_type in (3, 7)

        coordinate_list = info.get("coordinateList") or []
        shapes.extend(
            self._shape_from_coords(
                coordinate_list,
                style,
                close=prefer_closed,
                allow_fill=prefer_closed,
            )
        )

        for line in info.get("lineList") or []:
            coords = line.get("coordinateList") or []
            shapes.extend(
                self._shape_from_coords(
                    coords,
                    style,
                    close=False,
                    allow_fill=False,
                )
            )

        for area in info.get("areaList") or []:
            coords = area.get("coordinateList") or []
            shapes.extend(
                self._shape_from_coords(
                    coords,
                    style,
                    close=True,
                    allow_fill=True,
                )
            )

        return shapes

    def _shape_from_coords(
        self,
        coords: Iterable[Any],
        style: Dict[str, Any],
        *,
        close: bool,
        allow_fill: bool,
    ) -> list[Dict[str, Any]]:
        points = self._project_coordinates(coords)
        if not points:
            return []
        if len(points) == 1:
            return [
                {
                    "type": "point",
                    "points": points,
                    "style": {**style, "fill": style.get("fill")},
                    "fill": True,
                }
            ]
        return [
            {
                "type": "polygon" if close and allow_fill else "polyline",
                "points": points,
                "close": close and allow_fill,
                "fill": allow_fill,
                "style": style,
            }
        ]

    def _project_coordinates(self, coords: Iterable[Any]) -> list[QPointF]:
        if not self._pixmap_cache:
            return []

        width = max(1, self._pixmap_cache.width())
        height = max(1, self._pixmap_cache.height())
        scale_x = scale_y = 1.0
        if self._dem_size:
            scale_x = width / max(1, self._dem_size[0])
            scale_y = height / max(1, self._dem_size[1])

        points: list[QPointF] = []
        for coord in coords:
            if isinstance(coord, dict):
                lon = coord.get("longitude")
                lat = coord.get("latitude")
            elif isinstance(coord, (list, tuple)) and len(coord) >= 2:
                lon, lat = coord[0], coord[1]
            else:
                continue
            if lon is None or lat is None:
                continue
            try:
                lon_f = float(lon)
                lat_f = float(lat)
            except (TypeError, ValueError):
                continue

            if self._dem_inv_transform is not None:
                col, row = self._dem_inv_transform * (lon_f, lat_f)
                x = col * scale_x
                y = row * scale_y
            elif self._dem_bounds:
                left, bottom, right, top = self._dem_bounds
                lon_span = right - left or 1e-9
                lat_span = top - bottom or 1e-9
                x = (lon_f - left) / lon_span * width
                y = (top - lat_f) / lat_span * height
            else:
                continue
            points.append(QPointF(x, y))
        return points

    def _make_pen(self, color_hex: Optional[str], width: float, dashed: bool = False) -> QPen:
        color = QColor(color_hex or DEFAULT_MISSION_STYLE["stroke"])
        pen = QPen(color)
        pen.setWidthF(max(width, 0.8))
        pen.setCosmetic(True)
        pen.setCapStyle(Qt.RoundCap)
        pen.setJoinStyle(Qt.RoundJoin)
        if dashed:
            pen.setStyle(Qt.DashLine)
        return pen

    def _make_brush(self, color_hex: Optional[str], opacity: float) -> QBrush:
        if not color_hex:
            return QBrush(Qt.NoBrush)
        color = QColor(color_hex)
        color.setAlphaF(max(0.0, min(1.0, opacity)))
        brush = QBrush(color)
        brush.setStyle(Qt.SolidPattern)
        return brush

    def _load_individual_plan_entries(self, package_id: Any) -> Optional[list]:
        if package_id is None:
            return None
        try:
            package_id = int(package_id)
        except (TypeError, ValueError):
            return None
        cached = self._mission_file_cache.get(package_id)
        if cached is not None:
            return cached
        path = self._locate_individual_plan_file(package_id)
        if path is None or not path.exists():
            return None
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            return None
        missions = data.get("individualMissionList")
        if not isinstance(missions, list):
            return None
        self._mission_file_cache[package_id] = missions
        return missions

    def _locate_individual_plan_file(self, package_id: int) -> Optional[Path]:
        try:
            primary = db_paths.get_db_subpath(
                "IndividualMissionPlan", f"{package_id}.json"
            )
        except Exception:
            primary = PROJECT_ROOT / "database" / "IndividualMissionPlan" / f"{package_id}.json"
        if primary.exists():
            return primary
        self._ensure_mission_output_index()
        if self._mission_output_index:
            return self._mission_output_index.get(package_id)
        return None

    def _ensure_mission_output_index(self) -> None:
        if self._mission_output_index is not None:
            return
        index: Dict[int, Path] = {}
        base = PROJECT_ROOT / "database" / "mission_output"
        if not base.exists():
            self._mission_output_index = index
            return
        for path in base.glob("IndividualMissionPlan_*.json"):
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
                pkg = data.get("individualMissionPackageID")
                if pkg is not None:
                    index[int(pkg)] = path
            except Exception:
                continue
        self._mission_output_index = index

    def _load_resource_image(self) -> None:
        if self._image_loaded:
            return
        width, height = 640, 360
        placeholder = QPixmap(width, height)
        placeholder.fill(QColor(32, 32, 48))

        painter = QPainter(placeholder)
        painter.setPen(QPen(QColor('#bbbbbb')))
        painter.drawText(
            placeholder.rect(),
            Qt.AlignCenter,
            '지형 미리보기 비활성화',
        )
        painter.end()

        self._image_view.set_pixmap(placeholder)
        self._pixmap_cache = placeholder
        self._dem_bounds = None
        self._dem_size = (width, height)
        self._image_loaded = True
        self._image_status_label.setText('지도 미표시')


    @staticmethod
    def _to_int(value: Any) -> Optional[int]:
        try:
            return int(value)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _fmt(value: Any) -> str:
        return "-" if value is None else str(value)

    @staticmethod
    def _format_update_source(update_info: Iterable[Any] | None) -> str:
        if isinstance(update_info, tuple) and len(update_info) == 2:
            update_type, key = update_info
            if update_type or key:
                return f"{update_type or '-'} / {key or '-'}"
        return "-"


class _ZoomableImageView(QGraphicsView):
    """QGraphicsView helper that supports wheel zoom and drag panning."""

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._scene = QGraphicsScene(self)
        self._pixmap_item: Optional[QGraphicsPixmapItem] = None
        self._overlay_items: list = []
        self.setScene(self._scene)

        self.setFrameShape(QFrame.StyledPanel)
        self.setBackgroundBrush(Qt.black)
        self.setRenderHint(QPainter.SmoothPixmapTransform, True)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorUnderMouse)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.viewport().setCursor(Qt.OpenHandCursor)

    def set_pixmap(self, pixmap: QPixmap) -> None:
        self._scene.clear()
        self._overlay_items.clear()
        self._pixmap_item = self._scene.addPixmap(pixmap)
        self._scene.setSceneRect(self._pixmap_item.boundingRect())
        self.setScene(self._scene)
        self.resetTransform()

    def clear_overlays(self) -> None:
        for item in list(self._overlay_items):
            try:
                self._scene.removeItem(item)
            except Exception:
                pass
        self._overlay_items.clear()

    def add_path_item(self, path: QPainterPath, pen: QPen, brush: QBrush):
        item = self._scene.addPath(path, pen, brush)
        item.setZValue(5)
        self._overlay_items.append(item)
        return item

    def add_point_item(
        self,
        point: QPointF,
        radius: float,
        stroke: QColor,
        fill: QColor,
    ):
        rect = QRectF(
            point.x() - radius,
            point.y() - radius,
            radius * 2,
            radius * 2,
        )
        pen = QPen(stroke)
        pen.setWidthF(max(1.0, radius / 2.0))
        brush = QBrush(fill)
        item = self._scene.addEllipse(rect, pen, brush)
        item.setZValue(6)
        self._overlay_items.append(item)
        return item

    def wheelEvent(self, event) -> None:  # type: ignore[override]
        if self._pixmap_item is None:
            return
        zoom_in_factor = 1.25
        zoom_out_factor = 0.8
        factor = zoom_in_factor if event.angleDelta().y() > 0 else zoom_out_factor
        self.scale(factor, factor)

    def mousePressEvent(self, event) -> None:  # type: ignore[override]
        if event.button() == Qt.LeftButton:
            self.viewport().setCursor(Qt.ClosedHandCursor)
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event) -> None:  # type: ignore[override]
        if event.button() == Qt.LeftButton:
            self.viewport().setCursor(Qt.OpenHandCursor)
        super().mouseReleaseEvent(event)

