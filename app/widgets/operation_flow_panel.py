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
)
from PyQt5.QtGui import QKeySequence, QPixmap
from PyQt5.QtCore import Qt, pyqtSignal

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

        columns = 9
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

        label = QLabel(dialog)
        label.setAlignment(Qt.AlignCenter)
        vbox.addWidget(label)

        info_label = QLabel("ESC 키를 누르면 창이 닫힙니다.", dialog)
        info_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        info_label.setObjectName("OperationImageHint")
        vbox.addWidget(info_label)

        QShortcut(QKeySequence(Qt.Key_Escape), dialog, activated=dialog.close)

        pixmap = QPixmap(str(image_path)) if image_path else QPixmap()
        if not pixmap.isNull():
            scaled = self._scaled_pixmap(pixmap)
            label.setPixmap(scaled)
            dialog.resize(scaled.width() + 40, scaled.height() + 60)
        else:
            label.setText("이미지를 찾을 수 없습니다.")
            label.setStyleSheet("color:#b22;")
            dialog.resize(360, 180)

        self._track_dialog(dialog)
        dialog.show()

    def _scaled_pixmap(self, pixmap: QPixmap) -> QPixmap:
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
