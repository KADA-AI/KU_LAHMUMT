# -*- coding: utf-8 -*-

"""Stack 기반 모니터링 탭 – 시스템 모드 컨트롤 + 듀얼 미션 뷰어."""



from __future__ import annotations



import math

from typing import Iterable, Optional, Sequence, Tuple



from PyQt5.QtCore import Qt, QPoint, QPointF, QRectF

from PyQt5.QtGui import QColor, QPen, QBrush, QPainterPath, QFont, QPainter

from PyQt5.QtWidgets import (

    QWidget,
    QVBoxLayout,
    QGroupBox,
    QFormLayout,
    QLabel,
    QCheckBox,
    QHBoxLayout,
    QSplitter,
    QFrame,
    QSizePolicy,
    QGraphicsView,
    QGraphicsScene,

    QGraphicsSimpleTextItem,

    QGraphicsItem,

)



from modules.monitoring_ver2.config import SYSTEM_MODE_OPTIONS

from modules.monitoring_ver2.gui.mission_area_presenter import MissionAreaPresenter





class MissionAreaMonitoringTab(QWidget):

    """Provides the redesigned Stack 기반 모니터링 layout with dual map canvases."""



    LAT_RANGE = (36.5, 39.0)

    LON_RANGE = (127.0, 128.0)



    def __init__(self, manager, parent: QWidget | None = None) -> None:

        super().__init__(parent)

        self.manager = manager

        self.current_mode_label = QLabel("-")
        self.display_options = {
            "collab": True,
            "individual": True,
            "routes": True,
            "filming": True,
        }
        self._option_checkboxes: dict[str, QCheckBox] = {}
        toggle_bar = self._build_display_controls()

        self.left_panel = _MapSection("임무 누적상태", self.LAT_RANGE, self.LON_RANGE, controls=toggle_bar)
        self.right_panel = _MapSection("현재 임무 상태", self.LAT_RANGE, self.LON_RANGE)

        self._init_ui()
        self.presenter = MissionAreaPresenter(
            manager=self.manager,
            cumulative_view=self.left_panel.map_view,
            current_view=None,
            display_options=self.display_options,
        )
        self._apply_placeholder_paths()
        self.refresh_display(("logic", "SystemMode"))



    # ------------------------------------------------------------------ UI

    def _init_ui(self) -> None:

        layout = QVBoxLayout(self)

        layout.setContentsMargins(16, 16, 16, 16)

        layout.setSpacing(12)



        layout.addWidget(self._build_system_mode_box())
        layout.addWidget(self._build_map_panel())

    def _build_display_controls(self) -> QWidget:
        container = QWidget()
        layout = QHBoxLayout(container)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(12)

        options = [
            ("collab", "협업기저임무"),
            ("individual", "개별임무"),
            ("routes", "경로"),
            ("filming", "촬영계획"),
        ]
        for key, label in options:
            checkbox = QCheckBox(label)
            checkbox.blockSignals(True)
            checkbox.setChecked(True)
            checkbox.blockSignals(False)
            checkbox.stateChanged.connect(lambda _, opt=key: self._on_display_option_changed(opt))
            layout.addWidget(checkbox)
            self._option_checkboxes[key] = checkbox
        layout.addStretch(1)
        return container


    def _build_system_mode_box(self) -> QGroupBox:

        group = QGroupBox("시스템 운용 모드")

        form = QFormLayout()

        form.setLabelAlignment(Qt.AlignLeft)

        form.setFormAlignment(Qt.AlignLeft)



        self.current_mode_label.setStyleSheet("font-weight:600;")

        form.addRow("현재 모드:", self.current_mode_label)



        group.setLayout(form)

        return group



    def _build_map_panel(self) -> QFrame:
        frame = QFrame()
        frame.setObjectName("missionAreaBody")
        frame.setStyleSheet(
            "#missionAreaBody {"
            "background-color: transparent;"
            "border:1px solid #c8c8c8;"
            "border-radius:4px;"
            "}"
        )
        frame_layout = QHBoxLayout(frame)
        frame_layout.setContentsMargins(12, 12, 12, 12)
        frame_layout.setSpacing(12)
        frame_layout.addWidget(self.left_panel, stretch=1)
        frame_layout.addWidget(self.right_panel, stretch=1)
        return frame


    # ------------------------------------------------------------------ Refresh

    def refresh_display(

        self,

        update_info: Iterable[object] | None = None,

        data_object: object | None = None,

    ) -> None:

        update_type, key = self._unpack_update(update_info)

        if (update_type, key) in {("logic", "SystemMode"), ("receive", "0101"), (None, None)}:
            candidate = None
            if data_object is not None and hasattr(data_object, "systemMode"):
                candidate = getattr(data_object, "systemMode", None)
            if candidate is None:
                candidate = self._safe_get_logic("SystemMode")
            self._sync_system_mode(candidate)

        if hasattr(self, "presenter"):
            self.presenter.handle_update(update_type, key, data_object)


    # ------------------------------------------------------------------ Helpers

    def _sync_system_mode(self, mode_value: object | None) -> None:

        if mode_value is None:

            self.current_mode_label.setText("-")

            return

        try:

            mode_int = int(mode_value)

        except (TypeError, ValueError):

            return



        label = next((text for value, text in SYSTEM_MODE_OPTIONS if value == mode_int), f"모드 {mode_int}")
        self.current_mode_label.setText(label)
        if hasattr(self, "presenter"):
            self.presenter.on_system_mode(mode_int)


    def _safe_get_logic(self, key: str) -> object | None:
        getter = getattr(self.manager, "get_logic_result", None)
        if callable(getter):
            try:
                return getter(key)
            except Exception:
                return None
        return None

    def _on_display_option_changed(self, option: str) -> None:
        checkbox = self._option_checkboxes.get(option)
        if checkbox is None:
            return
        self.display_options[option] = checkbox.isChecked()
        if hasattr(self, "presenter"):
            self.presenter.update_display_options(self.display_options)


    @staticmethod

    def _unpack_update(update_info: Iterable[object] | None) -> Tuple[Optional[str], Optional[str]]:

        if isinstance(update_info, (list, tuple)):

            first = str(update_info[0]) if len(update_info) > 0 and update_info[0] is not None else None

            second = str(update_info[1]) if len(update_info) > 1 and update_info[1] is not None else None

            return first, second

        return None, None



    def _apply_placeholder_paths(self) -> None:

        """Seed simple overlay lines so the empty canvases still show context."""

        left_paths = [

            [

                (37.05, 127.08),

                (37.25, 127.25),

                (37.42, 127.4),

                (37.65, 127.6),

            ],

            [

                (37.15, 127.7),

                (37.5, 127.55),

                (37.82, 127.85),

            ],

        ]

        right_paths = [

            [

                (37.1, 127.15),

                (37.3, 127.45),

                (37.6, 127.35),

            ],

        ]



        palette = ["#1f78ff", "#ff6f00", "#19a974"]

        self.left_panel.map_view.clear_overlays()

        for idx, coords in enumerate(left_paths):

            self.left_panel.map_view.add_polyline(coords, color=palette[idx % len(palette)], width=3.0)



        self.right_panel.map_view.clear_overlays()

        for idx, coords in enumerate(right_paths):

            self.right_panel.map_view.add_polyline(coords, color=palette[idx % len(palette)], width=3.0)





class _MapSection(QFrame):
    """Wraps a map view with a title to mimic the provided mock-up."""

    def __init__(
        self,
        title: str,
        lat_range: Tuple[float, float],
        lon_range: Tuple[float, float],
        parent: QWidget | None = None,
        controls: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.setFrameShape(QFrame.NoFrame)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)

        title_label = QLabel(title)
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("color:#111111; font-size:15px; font-weight:600;")
        layout.addWidget(title_label)

        if controls is not None:
            layout.addWidget(controls)

        canvas_frame = QFrame()
        canvas_frame.setObjectName("mapCanvas")
        canvas_frame.setStyleSheet(
            "#mapCanvas { background-color:#ffffff; border:2px solid #c8c8c8; border-radius:3px; }"
        )
        canvas_layout = QVBoxLayout(canvas_frame)
        canvas_layout.setContentsMargins(0, 0, 0, 0)
        canvas_layout.setSpacing(0)

        self.map_view = MissionMapView(lat_range, lon_range)
        canvas_layout.addWidget(self.map_view)

        layout.addWidget(canvas_frame)




class MissionMapView(QGraphicsView):

    """Custom QGraphicsView supporting wheel zoom + right-click drag pan in lat/lon space."""



    SCALE_PER_DEGREE = 900.0

    def __init__(self, lat_range: Tuple[float, float], lon_range: Tuple[float, float], parent: QWidget | None = None) -> None:

        super().__init__(parent)

        self.lat_range = lat_range

        self.lon_range = lon_range

        self._scene = QGraphicsScene(self)

        self._scene.setItemIndexMethod(QGraphicsScene.NoIndex)

        self.setScene(self._scene)



        self._overlay_items: list = []

        self._grid_items: list = []

        self._panning = False

        self._pan_start = QPoint(0, 0)



        self._setup_view()

        self._refresh_grid()



    # ------------------------------------------------------------------ Public API

    def clear_overlays(self) -> None:

        for item in list(self._overlay_items):

            try:

                self._scene.removeItem(item)

            except Exception:

                pass

        self._overlay_items.clear()



    def add_polyline(

        self,

        latlon_points: Sequence[Tuple[float, float]],

        color: str = "#1f78ff",

        width: float = 2.0,

        z_value: float = 5,

        dash_pattern: Optional[Sequence[int]] = None,

    ):

        if len(latlon_points) < 2:

            return None



        pen = QPen(QColor(color))

        pen.setWidthF(width)

        pen.setCosmetic(True)

        pen.setJoinStyle(Qt.RoundJoin)

        pen.setCapStyle(Qt.RoundCap)

        if dash_pattern:

            pen.setDashPattern(list(dash_pattern))



        start_point = self._latlon_to_point(*latlon_points[0])

        path = QPainterPath(start_point)

        for lat, lon in latlon_points[1:]:

            path.lineTo(self._latlon_to_point(lat, lon))



        item = self._scene.addPath(path, pen)

        item.setZValue(z_value)

        return self._register_overlay(item)



    def add_polygon(

        self,

        latlon_points: Sequence[Tuple[float, float]],

        stroke: str = "#f97316",

        fill: str = "#fed7aa",

        width: float = 2.0,

        opacity: float = 0.4,

        z_value: float = 4,

    ):

        if len(latlon_points) < 3:

            return None



        start_point = self._latlon_to_point(*latlon_points[0])

        path = QPainterPath(start_point)

        for lat, lon in latlon_points[1:]:

            path.lineTo(self._latlon_to_point(lat, lon))

        path.closeSubpath()



        pen = QPen(QColor(stroke))

        pen.setWidthF(width)

        pen.setCosmetic(True)



        brush_color = QColor(fill)

        brush_color.setAlphaF(max(0.0, min(1.0, opacity)))

        brush = QBrush(brush_color)



        item = self._scene.addPath(path, pen, brush)

        item.setZValue(z_value)

        return self._register_overlay(item)



    def add_point(
        self,
        lat: float,
        lon: float,
        radius: float = 4.0,
        stroke: str = "#111111",
        fill: str = "#ffffff",
        z_value: float = 6,
        stroke_width: float | None = None,
    ):
        center = self._latlon_to_point(lat, lon)
        rect = QRectF(center.x() - radius, center.y() - radius, radius * 2, radius * 2)
        pen = QPen(QColor(stroke))
        pen.setWidthF(stroke_width if stroke_width is not None else 0)
        pen.setCosmetic(True)
        brush = QBrush(QColor(fill))
        item = self._scene.addEllipse(rect, pen, brush)
        item.setZValue(z_value)
        return self._register_overlay(item)



    def add_label(
        self,
        lat: float,
        lon: float,
        text: str,
        *,
        color: str = "#111111",
        font_size: int = 10,
        z_value: float = 7,
        offset_x: float = 0.0,
        offset_y: float = 0.0,
    ):
        item = QGraphicsSimpleTextItem(text)
        font = QFont()
        font.setPointSize(font_size)
        font.setBold(True)
        item.setFont(font)
        item.setBrush(QColor(color))
        item.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)
        scene_pos = self._latlon_to_point(lat, lon)
        rect = item.boundingRect()
        item.setPos(
            scene_pos.x() - rect.width() / 2 + offset_x,
            scene_pos.y() - rect.height() / 2 + offset_y,
        )
        item.setZValue(z_value)
        self._scene.addItem(item)
        return self._register_overlay(item)



    def _register_overlay(self, item):

        if item is not None:

            self._overlay_items.append(item)

        return item



    # ------------------------------------------------------------------ Internal drawing

    def _setup_view(self) -> None:

        self.setRenderHints(self.renderHints() | QPainter.Antialiasing | QPainter.SmoothPixmapTransform)

        self.setBackgroundBrush(QColor("#ffffff"))

        self.setFrameShape(QFrame.NoFrame)

        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)

        self.setResizeAnchor(QGraphicsView.AnchorUnderMouse)

        self.setDragMode(QGraphicsView.NoDrag)

        self.setContextMenuPolicy(Qt.NoContextMenu)



        lon_span = self.lon_range[1] - self.lon_range[0]

        lat_span = self.lat_range[1] - self.lat_range[0]

        self._map_rect = QRectF(

            0.0,

            0.0,

            max(1.0, lon_span * self.SCALE_PER_DEGREE),

            max(1.0, lat_span * self.SCALE_PER_DEGREE),

        )

        base_rect = self._scene.addRect(self._map_rect, QPen(QColor("#d0d0d0")), QBrush(QColor("#ffffff")))

        base_rect.setZValue(0)



        pad_x = self._map_rect.width() * 8

        pad_y = self._map_rect.height() * 8

        self._scene.setSceneRect(

            self._map_rect.left() - pad_x,

            self._map_rect.top() - pad_y,

            self._map_rect.width() + pad_x * 2,

            self._map_rect.height() + pad_y * 2,

        )

        self.fitInView(self._map_rect, Qt.KeepAspectRatio)



    def _refresh_grid(self) -> None:

        if self._scene is None:

            return



        for item in list(self._grid_items):

            try:

                self._scene.removeItem(item)

            except Exception:

                pass

        self._grid_items.clear()



        view_rect = self.mapToScene(self.viewport().rect()).boundingRect()

        if view_rect.isNull():

            view_rect = self._scene.sceneRect()



        lat_top = self._scene_to_lat(view_rect.top())

        lat_bottom = self._scene_to_lat(view_rect.bottom())

        lat_min, lat_max = sorted((lat_top, lat_bottom))

        lon_left = self._scene_to_lon(view_rect.left())

        lon_right = self._scene_to_lon(view_rect.right())

        lon_min, lon_max = sorted((lon_left, lon_right))



        lat_step = max(self._pick_step(lat_max - lat_min), 0.01)

        lon_step = max(self._pick_step(lon_max - lon_min), 0.01)



        grid_pen = QPen(QColor("#d4d4d4"))

        grid_pen.setCosmetic(True)



        lat_value = math.floor(lat_min / lat_step) * lat_step

        while lat_value <= lat_max + 1e-6:

            y = self._lat_to_scene(lat_value)

            line = self._scene.addLine(

                view_rect.left(),

                y,

                view_rect.right(),

                y,

                grid_pen,

            )

            line.setZValue(1)

            self._grid_items.append(line)

            label = self._add_grid_label(f"{lat_value:.2f}°N", QPointF(view_rect.left() + 6, y - 12))

            self._grid_items.append(label)

            lat_value += lat_step



        lon_value = math.floor(lon_min / lon_step) * lon_step

        while lon_value <= lon_max + 1e-6:

            x = self._lon_to_scene(lon_value)

            line = self._scene.addLine(

                x,

                view_rect.top(),

                x,

                view_rect.bottom(),

                grid_pen,

            )

            line.setZValue(1)

            self._grid_items.append(line)

            label = self._add_grid_label(f"{lon_value:.2f}°E", QPointF(x - 22, view_rect.top() + 8))

            self._grid_items.append(label)

            lon_value += lon_step



    def _add_grid_label(self, text: str, pos: QPointF) -> QGraphicsSimpleTextItem:

        label = QGraphicsSimpleTextItem(text)

        font = QFont()

        font.setPointSize(8)

        label.setFont(font)

        label.setBrush(QColor("#595959"))

        label.setPos(pos)

        label.setZValue(2)

        label.setFlag(QGraphicsItem.ItemIgnoresTransformations, True)

        self._scene.addItem(label)

        return label



    # ------------------------------------------------------------------ Interaction overrides

    def wheelEvent(self, event) -> None:  # type: ignore[override]

        zoom_factor = 1.2 if event.angleDelta().y() > 0 else 0.8

        self.scale(zoom_factor, zoom_factor)

        self._refresh_grid()

        event.accept()



    def mousePressEvent(self, event) -> None:  # type: ignore[override]

        if event.button() == Qt.RightButton:

            self._panning = True

            self._pan_start = event.pos()

            self.setCursor(Qt.ClosedHandCursor)

            event.accept()

            return

        super().mousePressEvent(event)



    def mouseMoveEvent(self, event) -> None:  # type: ignore[override]

        if self._panning:

            delta = event.pos() - self._pan_start

            self._pan_start = event.pos()

            self.horizontalScrollBar().setValue(self.horizontalScrollBar().value() - delta.x())

            self.verticalScrollBar().setValue(self.verticalScrollBar().value() - delta.y())

            self._refresh_grid()

            event.accept()

            return

        super().mouseMoveEvent(event)



    def mouseReleaseEvent(self, event) -> None:  # type: ignore[override]

        if event.button() == Qt.RightButton and self._panning:

            self._panning = False

            self.setCursor(Qt.ArrowCursor)

            self._refresh_grid()

            event.accept()

            return

        super().mouseReleaseEvent(event)



    def resizeEvent(self, event) -> None:  # type: ignore[override]

        super().resizeEvent(event)

        self._refresh_grid()



    def scrollContentsBy(self, dx: int, dy: int) -> None:  # type: ignore[override]

        super().scrollContentsBy(dx, dy)

        if self._panning:

            return

        self._refresh_grid()



    # ------------------------------------------------------------------ Converters

    def _latlon_to_point(self, lat: float, lon: float) -> QPointF:

        return QPointF(self._lon_to_scene(lon), self._lat_to_scene(lat))



    def _lat_to_scene(self, lat: float) -> float:

        lat_span = self.lat_range[1] - self.lat_range[0]

        if lat_span == 0:

            return self._map_rect.top()

        normalized = (self.lat_range[1] - lat) / lat_span

        return self._map_rect.top() + normalized * self._map_rect.height()



    def _lon_to_scene(self, lon: float) -> float:

        lon_span = self.lon_range[1] - self.lon_range[0]

        if lon_span == 0:

            return self._map_rect.left()

        normalized = (lon - self.lon_range[0]) / lon_span

        return self._map_rect.left() + normalized * self._map_rect.width()



    def _scene_to_lat(self, y: float) -> float:

        lat_span = self.lat_range[1] - self.lat_range[0]

        if lat_span == 0 or self._map_rect.height() == 0:

            return self.lat_range[0]

        normalized = (y - self._map_rect.top()) / self._map_rect.height()

        return self.lat_range[1] - normalized * lat_span



    def _scene_to_lon(self, x: float) -> float:

        lon_span = self.lon_range[1] - self.lon_range[0]

        if lon_span == 0 or self._map_rect.width() == 0:

            return self.lon_range[0]

        normalized = (x - self._map_rect.left()) / self._map_rect.width()

        return self.lon_range[0] + normalized * lon_span



    @staticmethod

    def _pick_step(span: float) -> float:

        if span <= 0:

            return 0.1

        steps = [0.01, 0.02, 0.05, 0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0]

        for step in steps:

            if span / step <= 12:

                return step

        return 200.0
