# 파일: /mnt/data/operation_flow_panel.py
# -*- coding: utf-8 -*-
from pathlib import Path

from PyQt5.QtWidgets import (
    QDialog,
    QGridLayout,
    QLabel,
    QPushButton,
    QShortcut,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
    QGraphicsView,
    QGraphicsScene,
)
from PyQt5.QtGui import QKeySequence, QPixmap, QPainter
from PyQt5.QtCore import Qt, pyqtSignal, QRectF, QTimer

from .cards import Card


class OperationButton(QPushButton):
    """Button that raises a signal when right-clicked."""

    rightClicked = pyqtSignal(str)

    def __init__(self, code: str, label: str, parent=None):
        super().__init__(f"{code} {label}", parent)
        self._code = code

    def mousePressEvent(self, event):  # noqa: N802
        if event.button() == Qt.RightButton:
            self.rightClicked.emit(self._code)
            event.accept()
            return
        super().mousePressEvent(event)


# ─────────────────────────────────────────────
# 확대/축소 전용 GraphicsView (초기 1:1 표시)
# ─────────────────────────────────────────────
class _ZoomableGraphicsView(QGraphicsView):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setRenderHints(QPainter.Antialiasing | QPainter.SmoothPixmapTransform)
        self.setDragMode(QGraphicsView.ScrollHandDrag)  # 마우스로 패닝
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)

        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)
        self._pixmap_item = self._scene.addPixmap(QPixmap())

        self._fit_scale = 1.0
        self._scale = 1.0
        self._min_scale = 0.1
        self._max_scale = 12.0
        self._fit_pending = False

    def set_image(self, pm: QPixmap):
        self._pixmap_item.setPixmap(pm)
        self._scene.setSceneRect(QRectF(pm.rect()))
        # ★ 초기 배율: 1.0 (원본 크기 그대로)
        self.resetTransform()
        self._fit_scale = 1.0
        self._scale = 1.0
        self._fit_pending = True
        QTimer.singleShot(0, self._apply_fit_scale)

    def _compute_fit_scale(self) -> float:
        pm = self._pixmap_item.pixmap()
        if pm.isNull():
            return 1.0
        viewport = self.viewport().size()
        if viewport.width() <= 0 or viewport.height() <= 0:
            return 1.0
        scale_x = viewport.width() / pm.width()
        scale_y = viewport.height() / pm.height()
        fit_scale = min(scale_x, scale_y)
        if fit_scale >= 1.0:
            return 1.0
        return max(self._min_scale, min(self._max_scale, fit_scale))

    def _apply_fit_scale(self):
        if self._pixmap_item.pixmap().isNull():
            return
        fit_scale = self._compute_fit_scale()
        self._fit_scale = fit_scale
        self._scale = fit_scale
        self._apply_scale()
        self._fit_pending = False

    def _apply_scale(self):
        self.resetTransform()
        s = max(self._min_scale, min(self._max_scale, self._scale))
        self.scale(s, s)

    # 단축키용
    def zoom_in(self):
        self._scale *= 1.2
        self._apply_scale()

    def zoom_out(self):
        self._scale /= 1.2
        self._apply_scale()

    def reset_zoom(self):
        self._scale = self._fit_scale
        self._apply_scale()

    # Ctrl + 휠로만 확대/축소
    def wheelEvent(self, event):
        if event.modifiers() & Qt.ControlModifier:
            delta = event.angleDelta().y()
            factor = 1.2 if delta > 0 else (1 / 1.2)
            self._scale *= factor
            self._apply_scale()
            event.accept()
            return
        super().wheelEvent(event)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self._pixmap_item.pixmap().isNull():
            return
        fit_scale = self._compute_fit_scale()
        if self._fit_pending or abs(self._scale - self._fit_scale) < 1e-3:
            self._fit_scale = fit_scale
            self._scale = fit_scale
            self._apply_scale()
            self._fit_pending = False
        else:
            self._fit_scale = fit_scale


class OperationFlowPanel(Card):
    """Card that lists operation-state shortcuts and previews."""

    stateTriggered = pyqtSignal(str)
    IMAGE_ROOT = Path(__file__).resolve().parents[1] / "resources" / "operation_process"

    def __init__(self, parent=None):
        super().__init__("운용 단계 / 상태 모니터링", parent)
        self._image_dialogs = []

        hint = QLabel("※ 상태 버튼은 추후 기능과 연동될 예정입니다.", self)
        hint.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        hint.setObjectName("OperationHint")
        self.body_layout.addWidget(hint, 0, Qt.AlignLeft)

        container = QWidget(self)
        grid = QGridLayout(container)
        grid.setContentsMargins(0, 8, 0, 0)
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(8)

        button_specs = [
            ("S100", "초기화"),
            ("S110", "초기임무계획"),
            ("S120", "임무시작"),
            ("S210", "정상수행"),
            ("S211", "협업기저임무 전환"),
            ("S212", "협업기저임무 재수행"),
            ("S213", "연료량 경고"),
            ("S220", "협업기저임무 편집"),
            ("S221", "선행임무 입력"),
            ("S230", "선행임무 취소"),
            ("S233", "임무대기"),
            ("S234", "임무복귀"),
            ("S235", "강제귀환"),
            ("S240", "시스템 판단 재계획"),
            ("S241", "시스템 판단 재계획 - Option 미생성"),
            ("S250", "자동 줌인 수행"),
            ("S251", "자동 줌인 중단"),
        ]

        columns = 5
        buttons = []
        for idx, (code, label) in enumerate(button_specs):
            row = idx // columns
            col = idx % columns
            btn = OperationButton(code, label, container)
            btn.setObjectName(f"btn_state_{code.lower()}")
            btn.setMinimumHeight(34)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            btn.clicked.connect(lambda checked=False, c=code: self.stateTriggered.emit(c))
            btn.rightClicked.connect(self._show_operation_image)
            grid.addWidget(btn, row, col)
            buttons.append(btn)

        if buttons:
            target_width = max(btn.sizeHint().width() for btn in buttons)
            for btn in buttons:
                btn.setMinimumWidth(target_width)
            for col in range(columns):
                grid.setColumnStretch(col, 1)

        self.body_layout.addWidget(container, 1)
        self.body_layout.addStretch(1)

    # ------------------------------------------------------------------
    # Image helpers
    # ------------------------------------------------------------------
    def _show_operation_image(self, code: str) -> None:
        """Display the flow image corresponding to the given code."""
        image_path = self._resolve_image_path(code)

        dialog = QDialog(self)
        dialog.setWindowTitle(f"{code} 단계")
        dialog.setAttribute(Qt.WA_DeleteOnClose, True)

        vbox = QVBoxLayout(dialog)
        vbox.setContentsMargins(12, 12, 12, 12)
        vbox.setSpacing(8)

        view = _ZoomableGraphicsView(dialog)
        vbox.addWidget(view, 1)

        info_label = QLabel("ESC 닫기 · Ctrl+휠 확대/축소 · Ctrl+=/Ctrl+- · Ctrl+0(리셋)", dialog)
        info_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        info_label.setObjectName("OperationImageHint")
        vbox.addWidget(info_label)

        QShortcut(QKeySequence(Qt.Key_Escape), dialog, activated=dialog.close)
        QShortcut(QKeySequence.ZoomIn, dialog, activated=view.zoom_in)    # Ctrl+=
        QShortcut(QKeySequence.ZoomOut, dialog, activated=view.zoom_out)  # Ctrl+-
        QShortcut(QKeySequence("Ctrl+0"), dialog, activated=view.reset_zoom)

        pm = QPixmap(str(image_path)) if image_path else QPixmap()
        if not pm.isNull():
            view.set_image(pm)  # ★ 초기 1:1
            # ★ 창 크기 = 이미지 크기 + 여백 (스크린 가용 영역 90% 이내로만 제한)
            screen_geo = dialog.screen().availableGeometry()
            pad_w, pad_h = 48, 120
            want_w = pm.width() + pad_w
            want_h = pm.height() + pad_h
            max_w = int(screen_geo.width() * 0.9)
            max_h = int(screen_geo.height() * 0.9)
            dialog.resize(max(360, min(want_w, max_w)), max(240, min(want_h, max_h)))
        else:
            placeholder = QLabel("이미지를 찾을 수 없습니다.", dialog)
            placeholder.setStyleSheet("color:#b22;")
            placeholder.setAlignment(Qt.AlignCenter)
            vbox.insertWidget(0, placeholder, 1)
            dialog.resize(360, 180)

        self._track_dialog(dialog)
        dialog.show()

    def _scaled_pixmap(self, pixmap: QPixmap) -> QPixmap:
        # (호환 유지용 — 현재는 GraphicsView 사용)
        max_width = 1180
        max_height = 820
        if pixmap.width() <= max_width and pixmap.height() <= max_height:
            return pixmap
        return pixmap.scaled(max_width, max_height, Qt.KeepAspectRatio, Qt.SmoothTransformation)

    def _resolve_image_path(self, code: str) -> Path | None:
        if not code:
            return None
        candidates = [
            self.IMAGE_ROOT / f"{code}.png",
            self.IMAGE_ROOT / f"{code}.PNG",
            self.IMAGE_ROOT / f"{code}.jpg",
            self.IMAGE_ROOT / f"{code}.JPG",
        ]
        for path in candidates:
            if path.exists():
                return path
        return None

    def _track_dialog(self, dialog: QDialog) -> None:
        self._image_dialogs.append(dialog)

        def _cleanup(_result: int):
            try:
                self._image_dialogs.remove(dialog)
            except ValueError:
                pass

        dialog.finished.connect(_cleanup)
