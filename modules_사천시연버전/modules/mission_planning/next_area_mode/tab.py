from __future__ import annotations

import os
from typing import Optional

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QComboBox,
    QFrame,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from .config import FLOW_MODE_ENV_KEY, WINDOW_TITLE


class NextAreaModeLauncherTab(QWidget):
    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._planner_window = None
        self._build_ui()
        self._refresh_status()

    def _build_ui(self) -> None:
        layout = QVBoxLayout(self)
        layout.setContentsMargins(18, 18, 18, 18)
        layout.setSpacing(12)

        title = QLabel(WINDOW_TITLE)
        title.setStyleSheet("font-size: 18px; font-weight: 700; color: #0f172a;")
        layout.addWidget(title)

        desc = QLabel(
            "기존 임무계획 파이프라인과 분리된 전용 모드입니다.\n"
            "다음 영역 임무수행 전용 division planner 로직을 독립 창으로 실행합니다."
        )
        desc.setWordWrap(True)
        desc.setStyleSheet("color: #334155;")
        layout.addWidget(desc)

        card = QFrame()
        card.setStyleSheet(
            "QFrame { background: #f8fbff; border: 1px solid #d6dee9; border-radius: 10px; }"
        )
        card_layout = QVBoxLayout(card)
        card_layout.setContentsMargins(14, 14, 14, 14)
        card_layout.setSpacing(10)

        flow_row = QHBoxLayout()
        flow_row.addWidget(QLabel("Flow:"))
        self.cmb_flow = QComboBox()
        self.cmb_flow.addItem("Initial", "initial")
        self.cmb_flow.addItem("Replan", "replan")
        self.cmb_flow.currentIndexChanged.connect(self._apply_flow_mode)
        flow_row.addWidget(self.cmb_flow)
        flow_row.addStretch(1)
        card_layout.addLayout(flow_row)

        button_row = QHBoxLayout()
        self.btn_open = QPushButton("모드 열기")
        self.btn_open.clicked.connect(self._open_or_focus_window)
        self.btn_new = QPushButton("새 창 다시 열기")
        self.btn_new.clicked.connect(self._reopen_window)
        button_row.addWidget(self.btn_open)
        button_row.addWidget(self.btn_new)
        button_row.addStretch(1)
        card_layout.addLayout(button_row)

        self.lbl_status = QLabel("-")
        self.lbl_status.setWordWrap(True)
        self.lbl_status.setStyleSheet(
            "background: #edf2f7; color: #0f172a; border-radius: 8px; padding: 8px 10px;"
        )
        card_layout.addWidget(self.lbl_status)

        layout.addWidget(card)
        layout.addStretch(1)
        self._sync_flow_combo_from_env()

    def _sync_flow_combo_from_env(self) -> None:
        env_value = str(os.environ.get(FLOW_MODE_ENV_KEY, "initial") or "initial").strip().lower()
        idx = 1 if env_value == "replan" else 0
        self.cmb_flow.blockSignals(True)
        self.cmb_flow.setCurrentIndex(idx)
        self.cmb_flow.blockSignals(False)
        self._apply_flow_mode()

    def _apply_flow_mode(self) -> None:
        flow_value = str(self.cmb_flow.currentData() or "initial").strip().lower()
        os.environ[FLOW_MODE_ENV_KEY] = "replan" if flow_value == "replan" else "initial"
        self._refresh_status()

    def _planner_alive(self) -> bool:
        win = self._planner_window
        if win is None:
            return False
        try:
            win.isVisible()
        except Exception:
            return False
        return True

    def _refresh_status(self) -> None:
        flow_value = str(os.environ.get(FLOW_MODE_ENV_KEY, "initial") or "initial").strip().lower()
        state = "열림" if self._planner_alive() else "닫힘"
        self.lbl_status.setText(
            f"현재 flow: {flow_value}\n"
            f"창 상태: {state}\n"
            "이 탭은 런처만 담당하고, 실제 로직과 파라미터는 next_area_mode 패키지에서 관리합니다."
        )

    def _open_or_focus_window(self) -> None:
        self._apply_flow_mode()
        if self._planner_alive():
            self._planner_window.show()
            self._planner_window.raise_()
            self._planner_window.activateWindow()
            self._refresh_status()
            return
        self._create_window()

    def _reopen_window(self) -> None:
        self._apply_flow_mode()
        if self._planner_alive():
            try:
                self._planner_window.close()
            except Exception:
                pass
        self._create_window()

    def _create_window(self) -> None:
        from .planner_window import NextAreaPlanningWindow

        self._planner_window = NextAreaPlanningWindow()
        self._planner_window.setAttribute(Qt.WA_DeleteOnClose, True)
        self._planner_window.destroyed.connect(self._on_window_destroyed)
        self._planner_window.show()
        self._planner_window.raise_()
        self._planner_window.activateWindow()
        self._refresh_status()

    def _on_window_destroyed(self, *_args) -> None:
        self._planner_window = None
        self._refresh_status()


__all__ = ["NextAreaModeLauncherTab"]
