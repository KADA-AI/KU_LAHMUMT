from pathlib import Path

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QLabel,
    QGroupBox,
    QFormLayout,
    QComboBox,
    QTextEdit,
    QPushButton,
    QHBoxLayout,
    QSizePolicy,
)

from modules.monitoring_ver2.config import SYSTEM_MODE_OPTIONS


class ReplanRulesInfoTab(QWidget):
    """Provides system mode control and a free-form note area for replan rules."""

    def __init__(self, manager, parent=None):
        super().__init__(parent)
        self.manager = manager
        self._notes_path = Path(__file__).resolve().parents[3] / "data" / "replan_rules_notes.txt"

        main_layout = QVBoxLayout(self)

        # --- System mode control (shared with 재계획 판단 탭) ---
        mode_groupbox = QGroupBox("시스템 운용 모드")
        mode_layout = QFormLayout()

        self.system_mode_combo = QComboBox()
        for value, label in SYSTEM_MODE_OPTIONS:
            self.system_mode_combo.addItem(label, value)
        self.system_mode_combo.currentIndexChanged.connect(self.on_system_mode_changed)

        mode_layout.addRow(QLabel("현재 모드:"), self.system_mode_combo)
        mode_groupbox.setLayout(mode_layout)
        main_layout.addWidget(mode_groupbox)

        # --- Notes area for manual entries ---
        notes_groupbox = QGroupBox("재계획 규칙 메모")
        notes_layout = QVBoxLayout()

        self.notes_edit = QTextEdit()
        self.notes_edit.setPlaceholderText("재계획 규칙 관련 메모를 입력하세요.")
        self.notes_edit.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        notes_layout.addWidget(self.notes_edit, stretch=1)

        controls_layout = QHBoxLayout()
        controls_layout.addStretch(1)
        self.save_status_label = QLabel("")
        self.save_status_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.save_button = QPushButton("저장")
        self.save_button.clicked.connect(self._save_notes)

        controls_layout.addWidget(self.save_status_label)
        controls_layout.addWidget(self.save_button)
        notes_layout.addLayout(controls_layout)

        notes_groupbox.setLayout(notes_layout)
        main_layout.addWidget(notes_groupbox, stretch=1)

        self._load_notes()
        self.refresh_display(("logic", "SystemMode"))

    def _load_notes(self) -> None:
        """Populate the note editor with existing content, if available."""
        try:
            self._notes_path.parent.mkdir(parents=True, exist_ok=True)
            if self._notes_path.exists():
                self.notes_edit.setPlainText(self._notes_path.read_text(encoding="utf-8"))
        except Exception as exc:
            self.save_status_label.setText(f"로드 실패: {exc}")

    def _save_notes(self) -> None:
        """Persist current notes to disk."""
        try:
            self._notes_path.write_text(self.notes_edit.toPlainText(), encoding="utf-8")
        except Exception as exc:
            self.save_status_label.setText(f"저장 실패: {exc}")
        else:
            self.save_status_label.setText("저장 완료")

    def refresh_display(self, update_info, data_object=None):
        """Update the system mode combo box based on manager notifications."""
        source, key = update_info

        if source == "logic" and key == "SystemMode":
            new_mode = self.manager.get_logic_result("SystemMode")
            self._sync_mode_combo(new_mode)
        elif source == "receive" and key == "0101":
            if data_object and hasattr(data_object, "systemMode"):
                self._sync_mode_combo(getattr(data_object, "systemMode", None))

    def _sync_mode_combo(self, mode_value) -> None:
        """Align combo box with the supplied mode value."""
        try:
            mode_int = int(mode_value)
        except (TypeError, ValueError):
            return
        index = self.system_mode_combo.findData(mode_int)
        if index >= 0:
            self.system_mode_combo.blockSignals(True)
            self.system_mode_combo.setCurrentIndex(index)
            self.system_mode_combo.blockSignals(False)

    def on_system_mode_changed(self, index) -> None:
        """Handle manual system mode selection."""
        mode_value = self.system_mode_combo.itemData(index)
        if mode_value is None:
            return
        self.manager.set_system_mode(int(mode_value))
