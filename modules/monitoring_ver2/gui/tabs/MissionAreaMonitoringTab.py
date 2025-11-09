# -*- coding: utf-8 -*-
from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, Iterable, Optional

import numpy as np

try:
    import rasterio  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    rasterio = None

try:
    from PIL import Image, ImageOps  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    Image = None
    ImageOps = None

try:
    from matplotlib import cm  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    cm = None

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPixmap, QPainter, QImage
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

PROJECT_ROOT = Path(__file__).resolve().parents[4]
RESOURCE_PATH = PROJECT_ROOT / "resource" / "n38_e127_1arc_v3.tif"


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

        title = QLabel("임무영역 모니터링")
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
        map_title = QLabel("지형 미리보기 (resource/n38_e127_1arc_v3.tif)")
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

    def _load_resource_image(self) -> None:
        if self._image_loaded:
            return
        if not RESOURCE_PATH.exists():
            self._image_status_label.setText(f"지도 파일 없음: {RESOURCE_PATH.name}")
            return

        arr = None
        try:
            if rasterio is not None:
                with rasterio.open(RESOURCE_PATH) as ds:
                    data = ds.read(1, masked=True)
                    arr = data.astype(np.float32)
                    mask = np.ma.getmaskarray(data)
                    if mask is np.ma.nomask:
                        mask = np.zeros_like(arr, dtype=bool)
                    arr = np.where(mask, np.nan, arr)
            elif Image is not None:
                with Image.open(RESOURCE_PATH) as img:
                    arr = np.array(img, dtype=np.float32)
            else:
                self._image_status_label.setText("rasterio/Pillow가 없어 미리보기를 만들 수 없습니다.")
                return
        except Exception as exc:  # pragma: no cover - defensive
            self._image_status_label.setText(f"DEM 읽기 실패: {exc}")
            return

        valid = ~np.isnan(arr)
        if not valid.any():
            self._image_status_label.setText("유효한 픽셀이 없습니다.")
            return

        clipped = arr[valid]
        v_min = np.percentile(clipped, 1.0)
        v_max = np.percentile(clipped, 99.0)
        if v_max <= v_min:
            v_max = v_min + 1.0
        norm = np.clip((arr - v_min) / (v_max - v_min), 0.0, 1.0)
        norm[~valid] = np.nan

        try:
            if cm is not None:
                cmap = cm.get_cmap("terrain")
                rgba = cmap(np.nan_to_num(norm, nan=0.0))
                rgba[..., 3] = np.where(valid, 1.0, 0.0)
                rgb_arr = (rgba[..., :3] * 255).astype(np.uint8)
            elif ImageOps is not None and Image is not None:
                norm_img = Image.fromarray(
                    np.nan_to_num(norm, nan=0.0, posinf=0.0, neginf=0.0) * 255
                .astype(np.uint8),
                    mode="L",
                )
                norm_img = ImageOps.autocontrast(norm_img)
                colored = ImageOps.colorize(norm_img, "#29434E", "#FFE7A9")
                rgb_arr = np.array(colored, dtype=np.uint8)
            else:
                base = np.nan_to_num(norm, nan=0.0)
                rgb_arr = np.repeat((base * 255).astype(np.uint8)[..., None], 3, axis=2)
        except Exception as exc:  # pragma: no cover - defensive
            self._image_status_label.setText(f"색상 변환 실패: {exc}")
            return

        height, width, _ = rgb_arr.shape
        bytes_per_line = width * 3
        self._image_bytes = rgb_arr.tobytes()
        qimage = QImage(self._image_bytes, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimage)
        self._image_view.set_pixmap(pixmap)
        self._pixmap_cache = pixmap
        self._image_loaded = True
        self._image_status_label.setText(RESOURCE_PATH.name)

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
        self._pixmap_item = self._scene.addPixmap(pixmap)
        self._scene.setSceneRect(self._pixmap_item.boundingRect())
        self.setScene(self._scene)
        self.resetTransform()

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

