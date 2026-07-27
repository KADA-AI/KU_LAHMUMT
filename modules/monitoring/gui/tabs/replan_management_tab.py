from __future__ import annotations

import copy
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable

from PyQt5.QtCore import Qt, QSignalBlocker, QTimer
from PyQt5.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QSpinBox,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from modules.monitoring.logic.replan_runtime_settings import (
    default_replan_settings,
    defaults_path as replan_defaults_path,
    load_recommended_defaults,
    load_replan_settings,
    normalize_replan_settings,
    save_recommended_defaults,
    save_replan_settings,
    settings_path as replan_settings_path,
)


@dataclass(frozen=True)
class FieldSpec:
    group: str
    field: str
    label: str
    kind: str = "float"
    minimum: float | int | None = None
    maximum: float | int | None = None
    step: float | int | None = None
    decimals: int = 2
    options: tuple[tuple[str, Any], ...] = ()
    placeholder: str = ""


def _deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    merged = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _deep_merge(dict(merged[key]), value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def _as_int_list(value: Any) -> list[int]:
    if isinstance(value, str):
        items = value.replace(";", ",").split(",")
    elif isinstance(value, (list, tuple)):
        items = list(value)
    else:
        items = []
    out: list[int] = []
    seen: set[int] = set()
    for item in items:
        try:
            parsed = int(float(str(item).strip()))
        except Exception:
            continue
        if parsed <= 0 or parsed in seen:
            continue
        seen.add(parsed)
        out.append(parsed)
    return out


def _format_int_list(value: Any) -> str:
    if isinstance(value, str):
        return value
    if not isinstance(value, (list, tuple)):
        return ""
    return ", ".join(str(int(v)) for v in value if str(v).strip())


def _parse_json_text(text: str, fallback: Any) -> Any:
    raw = text.strip()
    if not raw:
        return fallback
    try:
        return json.loads(raw)
    except Exception:
        return fallback


class ToggleCard(QFrame):
    def __init__(self, title: str, description: str, enabled: bool = False, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("replanToggleCard")
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self._callback: Callable[[bool], None] | None = None

        root = QVBoxLayout(self)
        root.setContentsMargins(12, 10, 12, 10)
        root.setSpacing(6)

        top = QHBoxLayout()
        self.title_label = QLabel(title)
        self.title_label.setObjectName("replanToggleTitle")
        self.badge = QLabel()
        self.badge.setObjectName("replanToggleBadge")
        self.badge.setAlignment(Qt.AlignCenter)
        self.badge.setFixedSize(46, 18)
        top.addWidget(self.title_label, 1)
        top.addWidget(self.badge, 0, Qt.AlignRight)

        self.desc_label = QLabel(description)
        self.desc_label.setObjectName("replanToggleDescription")
        self.desc_label.setWordWrap(True)

        bottom = QHBoxLayout()
        self.button = QToolButton()
        self.button.setObjectName("replanToggleButton")
        self.button.setCheckable(True)
        self.button.setCursor(Qt.PointingHandCursor)
        self.button.setFixedSize(60, 28)
        self.button.clicked.connect(self._clicked)
        bottom.addWidget(self.button, 0, Qt.AlignLeft)
        bottom.addStretch(1)

        root.addLayout(top)
        root.addWidget(self.desc_label)
        root.addLayout(bottom)
        self.set_state(enabled, emit=False)

    def set_callback(self, callback: Callable[[bool], None] | None) -> None:
        self._callback = callback

    def state(self) -> bool:
        return bool(self.button.isChecked())

    def set_state(self, enabled: bool, *, emit: bool = False) -> None:
        with QSignalBlocker(self.button):
            self.button.setChecked(bool(enabled))
        self._sync(bool(enabled))
        if emit and self._callback is not None:
            self._callback(bool(enabled))

    def _clicked(self, checked: bool) -> None:
        self._sync(bool(checked))
        if self._callback is not None:
            self._callback(bool(checked))

    def _sync(self, enabled: bool) -> None:
        text = "ON" if enabled else "OFF"
        self.button.setText(text)
        self.badge.setText(text)
        self.badge.setProperty("enabledState", enabled)
        self.style().unpolish(self.badge)
        self.style().polish(self.badge)
        self.badge.update()


class _FocusWheelComboBox(QComboBox):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setFocusPolicy(Qt.StrongFocus)

    def wheelEvent(self, event) -> None:  # type: ignore[override]
        if self.hasFocus():
            super().wheelEvent(event)
            return
        event.ignore()

    def mousePressEvent(self, event) -> None:  # type: ignore[override]
        self.setFocus(Qt.MouseFocusReason)
        super().mousePressEvent(event)


class _FocusWheelSpinBox(QSpinBox):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setFocusPolicy(Qt.StrongFocus)

    def wheelEvent(self, event) -> None:  # type: ignore[override]
        if self.hasFocus():
            super().wheelEvent(event)
            return
        event.ignore()

    def mousePressEvent(self, event) -> None:  # type: ignore[override]
        self.setFocus(Qt.MouseFocusReason)
        super().mousePressEvent(event)


class _FocusWheelDoubleSpinBox(QDoubleSpinBox):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setFocusPolicy(Qt.StrongFocus)

    def wheelEvent(self, event) -> None:  # type: ignore[override]
        if self.hasFocus():
            super().wheelEvent(event)
            return
        event.ignore()

    def mousePressEvent(self, event) -> None:  # type: ignore[override]
        self.setFocus(Qt.MouseFocusReason)
        super().mousePressEvent(event)


class ReplanManagementTab(QWidget):
    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setObjectName("replanManagementTab")
        self._toggle_callbacks: dict[str, Callable[[bool, dict[str, Any]], None]] = {}
        self._toggle_cards: dict[str, ToggleCard] = {}
        self._field_specs: dict[tuple[str, str], FieldSpec] = {}
        self._field_widgets: dict[tuple[str, str], QWidget] = {}
        self._state: dict[str, Any] = {}
        self._status_callback: Callable[[str, bool], None] | None = None
        self._autosave_timer = QTimer(self)
        self._autosave_timer.setSingleShot(True)
        self._autosave_timer.setInterval(250)
        self._autosave_timer.timeout.connect(self.save_now)

        self._build_ui()
        self.set_all_states(load_replan_settings(), emit=False)
        self._ensure_recommended_defaults()
        self._update_status("임무 재계획 관리 탭 준비 완료", False)

    def set_log_callback(self, callback: Callable[[str, bool], None] | None) -> None:
        self._status_callback = callback

    def set_toggle_callback(self, key: str, callback: Callable[[bool, dict[str, Any]], None] | None) -> None:
        if callback is None:
            self._toggle_callbacks.pop(str(key), None)
        else:
            self._toggle_callbacks[str(key)] = callback

    def set_all_toggle_callbacks(self, callbacks: dict[str, Callable[[bool, dict[str, Any]], None]] | None) -> None:
        self._toggle_callbacks = dict(callbacks or {})

    def set_toggle_state(self, key: str, enabled: bool, *, emit: bool = False) -> None:
        key = str(key)
        self._state.setdefault("toggles", {})[key] = bool(enabled)
        card = self._toggle_cards.get(key)
        if card is not None:
            card.set_state(bool(enabled), emit=False)
        if emit:
            self._emit_toggle_callback(key, bool(enabled))

    def set_all_toggle_states(self, states: dict[str, Any] | None, *, emit: bool = False) -> None:
        for key, value in dict(states or {}).items():
            self.set_toggle_state(key, bool(value), emit=emit)

    def set_all_states(self, payload: dict[str, Any] | None, *, emit: bool = False) -> None:
        merged = _deep_merge(default_replan_settings(), payload if isinstance(payload, dict) else {})
        self._state = normalize_replan_settings(merged, use_disk_defaults=False)
        self._apply_state_to_ui(self._state, emit=emit)

    def get_state(self) -> dict[str, Any]:
        return copy.deepcopy(normalize_replan_settings(self._capture_state_from_ui(), use_disk_defaults=False))

    def reload_from_disk(self) -> dict[str, Any]:
        self.set_all_states(load_replan_settings(), emit=True)
        self._update_status("현재값을 다시 불러왔습니다", False)
        return self.get_state()

    def restore_recommended(self) -> dict[str, Any]:
        self.set_all_states(load_recommended_defaults(), emit=True)
        self._update_status("권장값을 복원했습니다", False)
        return self.save_now()

    def save_now(self) -> dict[str, Any]:
        state = normalize_replan_settings(self._capture_state_from_ui(), use_disk_defaults=False)
        self._state = copy.deepcopy(state)
        save_replan_settings(state)
        self._update_status("현재값을 저장했습니다", False)
        return copy.deepcopy(state)

    def save_recommended_now(self) -> dict[str, Any]:
        state = normalize_replan_settings(self._capture_state_from_ui(), use_disk_defaults=False)
        save_recommended_defaults(state)
        self._update_status("권장값을 저장했습니다", False)
        return copy.deepcopy(state)

    def _build_ui(self) -> None:
        assets_dir = Path(__file__).resolve().parent / "assets"
        spin_up_icon = (assets_dir / "spin_up.svg").as_posix()
        spin_down_icon = (assets_dir / "spin_down.svg").as_posix()

        stylesheet = """
            QWidget#replanManagementTab { background: #f4f7fb; color: #1b2840; }
            QScrollArea { background: transparent; border: none; }
            QFrame#replanSectionCard { background: #ffffff; border: 1px solid #d8e3ef; border-radius: 18px; }
            QFrame#replanToggleGroup { background: #fbfdff; border: 1px solid #d9e5f2; border-radius: 14px; }
            QFrame#replanToggleCard { background: #ffffff; border: 1px solid #d9e4ef; border-radius: 14px; }
            QFrame#replanParamCard { background: #fcfdff; border: 1px solid #d3dfed; border-radius: 18px; }
            QWidget#replanFieldRow { background: transparent; }
            QLabel#replanTitle { color: #10203b; font-size: 24px; font-weight: 700; }
            QLabel#replanSubtitle { color: #6c7a8f; font-size: 11px; }
            QLabel#replanBanner { background: #eef4ff; color: #2d4f9b; border: 1px solid #d2defa; border-radius: 10px; padding: 7px 10px; }
            QLabel#replanSectionTitle { color: #10203b; font-size: 15px; font-weight: 700; }
            QLabel#replanSectionNote { color: #66758a; font-size: 11px; }
            QLabel#replanSubgroupTitle { color: #17325c; font-size: 13px; font-weight: 700; }
            QLabel#replanSubgroupNote { color: #62748b; font-size: 11px; }
            QLabel#replanToggleTitle { color: #10203b; font-size: 13px; font-weight: 700; }
            QLabel#replanToggleDescription { color: #66758a; font-size: 10px; }
            QLabel#replanToggleBadge { background: #edf2fb; color: #3056a2; border-radius: 8px; font-weight: 700; font-size: 10px; }
            QLabel#replanToggleBadge[enabledState="true"] { background: #e6f4eb; color: #1f7a46; }
            QToolButton#replanToggleButton { background: #eff3f8; color: #1d2d47; border: 1px solid #cad6e4; border-radius: 12px; padding: 4px 10px; font-weight: 700; font-size: 11px; }
            QToolButton#replanToggleButton:checked { background: #2f6df1; color: #ffffff; border-color: #2f6df1; }
            QLabel#replanParamTitle { color: #10203b; font-size: 17px; font-weight: 700; }
            QLabel#replanParamNote { color: #607086; font-size: 11px; }
            QLabel#replanFieldLabel { color: #203047; font-size: 13px; font-weight: 600; }
            QLineEdit, QPlainTextEdit { background: #ffffff; border: 1px solid #cfd9e7; border-radius: 11px; padding: 7px 11px; color: #1a2740; font-size: 13px; selection-background-color: #2f6df1; }
            QComboBox, QSpinBox, QDoubleSpinBox { background: #ffffff; color: #172438; border: 1px solid #c8d4e3; border-radius: 11px; min-height: 36px; padding: 4px 38px 4px 10px; font-size: 13px; selection-background-color: #2f6df1; }
            QComboBox:hover, QSpinBox:hover, QDoubleSpinBox:hover { background: #fbfdff; border-color: #afbfd3; }
            QLineEdit:focus, QPlainTextEdit:focus, QComboBox:focus, QSpinBox:focus, QDoubleSpinBox:focus { background: #ffffff; border-color: #4a7df2; }
            QComboBox::drop-down { subcontrol-origin: padding; subcontrol-position: top right; width: 28px; background: #eef4ff; border-left: 1px solid #c9d7ea; border-top-right-radius: 10px; border-bottom-right-radius: 10px; }
            QComboBox QAbstractItemView { background: #ffffff; color: #172438; border: 1px solid #c8d4e3; selection-background-color: #eaf2ff; selection-color: #17325c; }
            QComboBox::down-arrow { image: url("__SPIN_DOWN_ICON__"); width: 10px; height: 10px; }
            QSpinBox::up-button, QDoubleSpinBox::up-button { subcontrol-origin: border; subcontrol-position: top right; width: 28px; background: #eef4ff; border-left: 1px solid #c9d7ea; border-top-right-radius: 10px; }
            QSpinBox::down-button, QDoubleSpinBox::down-button { subcontrol-origin: border; subcontrol-position: bottom right; width: 28px; background: #eef4ff; border-left: 1px solid #c9d7ea; border-top: 1px solid #d7e2f1; border-bottom-right-radius: 10px; }
            QSpinBox::up-arrow, QDoubleSpinBox::up-arrow { image: url("__SPIN_UP_ICON__"); width: 10px; height: 10px; }
            QSpinBox::down-arrow, QDoubleSpinBox::down-arrow { image: url("__SPIN_DOWN_ICON__"); width: 10px; height: 10px; }
            QSpinBox::up-button:hover, QDoubleSpinBox::up-button:hover, QSpinBox::down-button:hover, QDoubleSpinBox::down-button:hover, QComboBox::drop-down:hover { background: #dfeaff; }
            QCheckBox { color: #203047; font-size: 13px; font-weight: 600; spacing: 8px; }
            QCheckBox::indicator { width: 18px; height: 18px; border: 1px solid #bfd0e4; border-radius: 5px; background: #ffffff; }
            QCheckBox::indicator:checked { background: #2f6df1; border-color: #2f6df1; }
            QPushButton#replanActionButton { background: #2f6df1; color: #ffffff; border: none; border-radius: 12px; padding: 9px 15px; font-weight: 700; }
            QPushButton#replanActionButton:hover { background: #245fe0; }
            QPushButton#replanGhostButton { background: #ffffff; color: #21314a; border: 1px solid #cfd9e7; border-radius: 12px; padding: 9px 15px; font-weight: 700; }
            QPushButton#replanGhostButton:hover { background: #f7faff; }
            """
        stylesheet = stylesheet.replace("__SPIN_UP_ICON__", spin_up_icon)
        stylesheet = stylesheet.replace("__SPIN_DOWN_ICON__", spin_down_icon)
        self.setStyleSheet(stylesheet)
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)
        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setContentsMargins(18, 18, 18, 18)
        layout.setSpacing(14)
        layout.addWidget(self._build_header())
        layout.addWidget(self._build_toggle_board())
        layout.addWidget(self._build_parameter_board())
        layout.addWidget(self._build_footer())
        layout.addStretch(1)
        scroll.setWidget(content)
        root.addWidget(scroll)

    def _build_header(self) -> QFrame:
        frame = QFrame()
        frame.setObjectName("replanSectionCard")
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(20, 18, 20, 18)
        layout.setSpacing(8)
        title = QLabel("임무 재계획 관리")
        title.setObjectName("replanTitle")
        subtitle = QLabel(f"현재값: {replan_settings_path().name} / 권장값: {replan_defaults_path().name}")
        subtitle.setObjectName("replanSubtitle")
        banner = QLabel("상단 스위치는 기능 ON/OFF, 아래 카드는 재계획 세부 기준값입니다. 전체 화면을 스크롤하면서 순서대로 조정하면 됩니다.")
        banner.setObjectName("replanBanner")
        banner.setWordWrap(True)
        layout.addWidget(title)
        layout.addWidget(subtitle)
        layout.addWidget(banner)
        return frame

    def _build_toggle_board(self) -> QFrame:
        frame = QFrame()
        frame.setObjectName("replanSectionCard")
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(18, 16, 18, 16)
        layout.setSpacing(14)
        head = QHBoxLayout()
        title = QLabel("기능 ON / OFF")
        title.setObjectName("replanSectionTitle")
        note = QLabel("모니터링 표시 제어와 재계획 판단 제어를 분리했습니다.")
        note.setObjectName("replanSectionNote")
        note.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        note.setWordWrap(True)
        head.addWidget(title, 1)
        head.addWidget(note, 1)
        layout.addLayout(head)
        for title_text, note_text, columns, items in self._toggle_groups():
            layout.addWidget(self._build_toggle_group(title_text, note_text, items, columns=columns))
        return frame

    def _build_toggle_group(self, title: str, note: str, items: list[tuple[str, str, str]], *, columns: int) -> QFrame:
        frame = QFrame()
        frame.setObjectName("replanToggleGroup")
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(14, 13, 14, 13)
        layout.setSpacing(10)
        title_label = QLabel(title)
        title_label.setObjectName("replanSubgroupTitle")
        note_label = QLabel(note)
        note_label.setObjectName("replanSubgroupNote")
        note_label.setWordWrap(True)
        grid = QGridLayout()
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(12)
        for index, (key, title_text, desc) in enumerate(items):
            card = self._toggle_cards.get(key)
            if card is None:
                card = ToggleCard(title_text, desc, bool(self._state.get("toggles", {}).get(key, False)))
                card.set_callback(lambda enabled, k=key: self._on_toggle_changed(k, enabled))
                self._toggle_cards[key] = card
            grid.addWidget(card, index // columns, index % columns)
        layout.addWidget(title_label)
        layout.addWidget(note_label)
        layout.addLayout(grid)
        return frame

    def _build_parameter_board(self) -> QFrame:
        frame = QFrame()
        frame.setObjectName("replanSectionCard")
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(18, 16, 18, 18)
        layout.setSpacing(14)
        head = QHBoxLayout()
        title = QLabel("세부 파라미터")
        title.setObjectName("replanSectionTitle")
        note = QLabel("한 화면에 억지로 맞추지 않고 카드별로 나눴습니다. 전체 탭 스크롤을 사용하면 됩니다.")
        note.setObjectName("replanSectionNote")
        note.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        note.setWordWrap(True)
        head.addWidget(title, 1)
        head.addWidget(note, 1)
        layout.addLayout(head)
        for title_text, note_text, specs in self._parameter_sections():
            layout.addWidget(self._make_section_card(title_text, note_text, specs))
        return frame

    def _build_footer(self) -> QFrame:
        frame = QFrame()
        frame.setObjectName("replanSectionCard")
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(18, 14, 18, 14)
        layout.setSpacing(10)
        self.status_label = QLabel("준비됨")
        self.status_label.setObjectName("replanSectionNote")
        self.status_label.setWordWrap(True)
        row = QHBoxLayout()
        row.setSpacing(10)
        self.save_button = QPushButton("현재값 저장")
        self.save_button.setObjectName("replanActionButton")
        self.save_button.clicked.connect(self.save_now)
        self.reload_button = QPushButton("현재값 다시 불러오기")
        self.reload_button.setObjectName("replanGhostButton")
        self.reload_button.clicked.connect(self.reload_from_disk)
        self.restore_button = QPushButton("권장값 복원")
        self.restore_button.setObjectName("replanGhostButton")
        self.restore_button.clicked.connect(self.restore_recommended)
        self.save_recommended_button = QPushButton("권장값 저장")
        self.save_recommended_button.setObjectName("replanGhostButton")
        self.save_recommended_button.clicked.connect(self.save_recommended_now)
        row.addWidget(self.save_button)
        row.addWidget(self.reload_button)
        row.addWidget(self.restore_button)
        row.addWidget(self.save_recommended_button)
        row.addStretch(1)
        layout.addWidget(self.status_label)
        layout.addLayout(row)
        return frame

    def _make_section_card(self, title: str, note: str, fields: list[FieldSpec]) -> QFrame:
        frame = QFrame()
        frame.setObjectName("replanParamCard")
        frame.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Maximum)
        layout = QVBoxLayout(frame)
        layout.setContentsMargins(22, 20, 22, 20)
        layout.setSpacing(12)
        title_label = QLabel(title)
        title_label.setObjectName("replanParamTitle")
        note_label = QLabel(note)
        note_label.setObjectName("replanParamNote")
        note_label.setWordWrap(True)
        layout.addWidget(title_label)
        layout.addWidget(note_label)
        for spec in fields:
            self._field_specs[(spec.group, spec.field)] = spec
            widget = self._create_editor(spec)
            self._field_widgets[(spec.group, spec.field)] = widget
            layout.addWidget(self._make_field_row(spec, widget))
        return frame

    def _make_field_row(self, spec: FieldSpec, editor: QWidget) -> QWidget:
        row = QWidget()
        row.setObjectName("replanFieldRow")
        layout = QHBoxLayout(row)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(18)
        label = QLabel(spec.label)
        label.setObjectName("replanFieldLabel")
        label.setWordWrap(True)
        label.setMinimumWidth(250)
        label.setMaximumWidth(320)
        layout.addWidget(label, 0)
        layout.addWidget(editor, 1)
        return row

    def _create_editor(self, spec: FieldSpec) -> QWidget:
        if spec.kind == "bool":
            widget = QCheckBox("사용")
            widget.stateChanged.connect(self._queue_save)
            return widget
        if spec.kind == "choice":
            widget = _FocusWheelComboBox()
            for text, data in spec.options:
                widget.addItem(text, data)
            widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            widget.currentIndexChanged.connect(self._queue_save)
            return widget
        if spec.kind == "int":
            widget = _FocusWheelSpinBox()
            if spec.minimum is not None:
                widget.setMinimum(int(spec.minimum))
            if spec.maximum is not None:
                widget.setMaximum(int(spec.maximum))
            if spec.step is not None:
                widget.setSingleStep(int(spec.step))
            widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            widget.valueChanged.connect(self._queue_save)
            return widget
        if spec.kind == "list":
            widget = QLineEdit()
            widget.setPlaceholderText(spec.placeholder)
            widget.setMinimumHeight(36)
            widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            widget.editingFinished.connect(self._queue_save)
            return widget
        if spec.kind == "json":
            widget = QPlainTextEdit()
            widget.setPlaceholderText(spec.placeholder)
            widget.setMinimumHeight(140)
            widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
            widget.textChanged.connect(self._queue_save)
            return widget
        widget = _FocusWheelDoubleSpinBox()
        widget.setDecimals(spec.decimals)
        if spec.minimum is not None:
            widget.setMinimum(float(spec.minimum))
        if spec.maximum is not None:
            widget.setMaximum(float(spec.maximum))
        if spec.step is not None:
            widget.setSingleStep(float(spec.step))
        widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        widget.valueChanged.connect(self._queue_save)
        return widget

    def _queue_save(self, *args: Any) -> None:
        self._autosave_timer.start()

    def _on_toggle_changed(self, key: str, enabled: bool) -> None:
        self._state.setdefault("toggles", {})[str(key)] = bool(enabled)
        self._emit_toggle_callback(str(key), bool(enabled))
        self._queue_save()

    def _capture_state_from_ui(self) -> dict[str, Any]:
        state = copy.deepcopy(self._state) if isinstance(self._state, dict) else default_replan_settings()
        toggles = state.setdefault("toggles", {})
        for key, card in self._toggle_cards.items():
            toggles[key] = bool(card.state())
        for (group, field), spec in self._field_specs.items():
            widget = self._field_widgets[(group, field)]
            fallback = (state.get(group) or {}).get(field)
            state.setdefault(group, {})[field] = self._read_widget_value(spec, widget, fallback)
        return state

    def _apply_state_to_ui(self, state: dict[str, Any], *, emit: bool = False) -> None:
        toggles = state.get("toggles") if isinstance(state.get("toggles"), dict) else {}
        for key, card in self._toggle_cards.items():
            enabled = bool(toggles.get(key, False))
            card.set_state(enabled, emit=False)
            if emit:
                self._emit_toggle_callback(key, enabled)
        for (group, field), spec in self._field_specs.items():
            self._write_widget_value(spec, self._field_widgets[(group, field)], (state.get(group) or {}).get(field))

    def _read_widget_value(self, spec: FieldSpec, widget: QWidget, fallback: Any) -> Any:
        if spec.kind == "bool":
            return bool(widget.isChecked())
        if spec.kind == "choice":
            data = widget.currentData()  # type: ignore[attr-defined]
            return fallback if data is None else data
        if spec.kind == "int":
            return int(widget.value())  # type: ignore[attr-defined]
        if spec.kind == "list":
            text = widget.text().strip()  # type: ignore[attr-defined]
            return _as_int_list(text) if text else []
        if spec.kind == "json":
            return _parse_json_text(widget.toPlainText(), fallback)  # type: ignore[attr-defined]
        return float(widget.value())  # type: ignore[attr-defined]

    def _write_widget_value(self, spec: FieldSpec, widget: QWidget, value: Any) -> None:
        if spec.kind == "bool":
            with QSignalBlocker(widget):
                widget.setChecked(bool(value))  # type: ignore[attr-defined]
            return
        if spec.kind == "choice":
            index = 0
            for i in range(widget.count()):  # type: ignore[attr-defined]
                if widget.itemData(i) == value:  # type: ignore[attr-defined]
                    index = i
                    break
            with QSignalBlocker(widget):
                widget.setCurrentIndex(index)  # type: ignore[attr-defined]
            return
        if spec.kind == "int":
            with QSignalBlocker(widget):
                widget.setValue(int(value) if value is not None else 0)  # type: ignore[attr-defined]
            return
        if spec.kind == "list":
            with QSignalBlocker(widget):
                widget.setText(_format_int_list(value))  # type: ignore[attr-defined]
            return
        if spec.kind == "json":
            with QSignalBlocker(widget):
                widget.setPlainText(json.dumps(value if value is not None else [], ensure_ascii=False, indent=2))  # type: ignore[attr-defined]
            return
        with QSignalBlocker(widget):
            widget.setValue(float(value) if value is not None else 0.0)  # type: ignore[attr-defined]

    def _emit_toggle_callback(self, key: str, enabled: bool) -> None:
        callback = self._toggle_callbacks.get(str(key))
        if callback is None:
            return
        try:
            callback(bool(enabled), self.get_state())
        except Exception as exc:
            self._update_status(f"{key} 토글 콜백 오류: {exc}", True)

    def _ensure_recommended_defaults(self) -> None:
        if not replan_defaults_path().exists():
            save_recommended_defaults(default_replan_settings())

    def _update_status(self, message: str, is_error: bool) -> None:
        if hasattr(self, "status_label"):
            self.status_label.setText(message)
            self.status_label.setStyleSheet("color: #b42318;" if is_error else "color: #677589;")
        if self._status_callback is not None:
            try:
                self._status_callback(message, is_error)
            except TypeError:
                try:
                    prefix = "[REPLANCFG][ERR]" if is_error else "[REPLANCFG]"
                    self._status_callback(f"{prefix} {message}")
                except Exception:
                    pass
            except Exception:
                pass

    def _toggle_groups_legacy(self) -> list[tuple[str, str, int, list[tuple[str, str, str]]]]:
        return [
            (
                "모니터링 / 표시",
                "재계획 판단이 아니라 화면 표시와 모니터링 갱신 여부를 제어합니다.",
                1,
                [
                    ("quality_monitor", "촬영품질 모니터", "품질 상태 카드와 품질 표시 갱신"),
                ],
            ),
            (
                "재계획 판단",
                "조건 충족 시 0902 재계획 요청까지 이어지는 판단 로직 스위치입니다.",
                3,
                [
                    ("path_deviation", "경로추종", "0401 선회·Orbit 기반 재계획"),
                    ("quality_speed", "품질속도", "속도 재계획 판단"),
                    ("forced_command", "강제대기", "0802 강제대기 재계획"),
                    ("input_refresh", "입력갱신", "중복·재실행 처리"),
                    ("prior_mission", "선행임무", "선행임무 위험"),
                    ("dl_risk", "DL 위험", "학습 위험 기준"),
                    ("imaging_schedule", "촬영계획", "촬영 일정 기반"),
                    ("next_collab", "다음협업", "다음 임무 진입"),
                    ("rtb", "RTB", "복귀 판단"),
                    ("target_detection", "표적탐지", "탐지 후속 판정"),
                    ("post_attack_rejoin", "공격후복귀", "공격 성공 후 복귀 UAV 재합류 판단"),
                    ("fuel_threshold", "연료", "연료 경계값"),
                ],
            ),
        ]

    def _parameter_sections(self) -> list[tuple[str, str, list[FieldSpec]]]:
        return [
            (
                "경로추종 / 선회 시작",
                "0401 heading 변화와 수신 상태를 기반으로 선회 시작 여부를 판단합니다.",
                [
                    FieldSpec("path_deviation", "turn_rate_threshold_dps", "선회율 임계값 (deg/s)", "float", 0.1, 10.0, 0.05, 2),
                    FieldSpec("path_deviation", "turn_window_s", "선회 계산 창 (s)", "float", 2.0, 30.0, 0.5, 1),
                    FieldSpec("path_deviation", "stale_timeout_s", "수신 지연 timeout (s)", "float", 0.5, 20.0, 0.5, 1),
                    FieldSpec("path_deviation", "heading_move_min_m", "좌표 heading 최소 이동거리 (m)", "float", 0.0, 100.0, 0.5, 1),
                    FieldSpec("path_deviation", "turn_gap_reset_s", "선회 추적 reset 간격 (s)", "float", 0.1, 10.0, 0.1, 1),
                ],
            ),
            (
                "경로추종 / Orbit 누적",
                "현재 WP 주변을 반복 선회하는지 누적해서 watch·warning 전이를 판단합니다.",
                [
                    FieldSpec("path_deviation", "spiral_window_s", "누적 감시 창 (s)", "float", 5.0, 180.0, 1.0, 1),
                    FieldSpec("path_deviation", "spiral_min_points", "최소 샘플 수", "int", 3, 50, 1),
                    FieldSpec("path_deviation", "center_ignore_radius_m", "WP 중심 무시 반경 (m)", "float", 0.0, 500.0, 5.0, 1),
                    FieldSpec("path_deviation", "accumulation_step_max_deg", "누적 step 최대각 (deg)", "float", 10.0, 360.0, 5.0, 1),
                    FieldSpec("path_deviation", "watch_angle_deg", "주의 각도 (deg)", "float", 10.0, 360.0, 5.0, 1),
                    FieldSpec("path_deviation", "warning_angle_deg", "경고 각도 (deg)", "float", 20.0, 360.0, 5.0, 1),
                    FieldSpec("path_deviation", "hold_s", "경고 유지 시간 (s)", "float", 0.0, 20.0, 0.5, 1),
                ],
            ),
            (
                "경로추종 / 반경 허용",
                "실제 선회 반경이 기준 반경과 얼마나 비슷해야 경로추종 경고로 인정할지 정합니다.",
                [
                    FieldSpec("path_deviation", "near_threshold_min_m", "WP 근접 최소거리 (m)", "float", 1.0, 2000.0, 5.0, 1),
                    FieldSpec("path_deviation", "near_threshold_radius_factor", "WP 근접 반경배수", "float", 1.0, 10.0, 0.05, 2),
                    FieldSpec("path_deviation", "radius_error_min_m", "평균반경 허용 최소값 (m)", "float", 1.0, 2000.0, 5.0, 1),
                    FieldSpec("path_deviation", "radius_error_factor", "평균반경 허용 배수", "float", 0.05, 5.0, 0.05, 2),
                    FieldSpec("path_deviation", "latest_radius_min_m", "최종반경 허용 최소값 (m)", "float", 1.0, 2000.0, 5.0, 1),
                    FieldSpec("path_deviation", "latest_radius_factor", "최종반경 허용 배수", "float", 0.01, 5.0, 0.01, 2),
                    FieldSpec("path_deviation", "release_min_distance_m", "해제 최소거리 (m)", "float", 1.0, 5000.0, 10.0, 1),
                    FieldSpec("path_deviation", "release_factor", "해제 반경배수", "float", 1.0, 10.0, 0.05, 2),
                    FieldSpec("path_deviation", "adaptive_enabled", "Adaptive correction", "bool"),
                    FieldSpec("path_deviation", "adaptive_sample_min_interval_s", "Adaptive sample interval (s)", "float", 0.2, 10.0, 0.05, 2),
                    FieldSpec("path_deviation", "adaptive_ema_alpha", "Adaptive EMA alpha", "float", 0.01, 1.0, 0.01, 2),
                    FieldSpec("path_deviation", "adaptive_warmup_samples", "Adaptive warmup samples", "int", 1, 100, 1),
                    FieldSpec("path_deviation", "adaptive_min_scale", "Adaptive min scale", "float", 0.1, 5.0, 0.01, 2),
                    FieldSpec("path_deviation", "adaptive_max_scale", "Adaptive max scale", "float", 0.1, 5.0, 0.01, 2),
                    FieldSpec("path_deviation", "adaptive_threshold_gain", "Adaptive threshold gain", "float", 0.0, 1.0, 0.01, 2),
                    FieldSpec("path_deviation", "adaptive_save_interval_s", "Adaptive save interval (s)", "float", 1.0, 120.0, 1.0, 1),
                ],
            ),
            (
                "경로추종 / 대체 WP 계산",
                "warning 상태가 유지될 때 synthetic 대체 WP와 다음 임무 진입점을 계산하는 값입니다.",
                [
                    FieldSpec("path_deviation", "alt_waypoint_trigger_s", "대체 WP 트리거 지연 (s)", "float", 0.0, 20.0, 0.5, 1),
                    FieldSpec("path_deviation", "alt_waypoint_lead_time_s", "대체 WP 선행시간 (s)", "float", 0.0, 30.0, 0.5, 1),
                    FieldSpec("path_deviation", "next_mission_entry_lead_time_s", "다음 임무 진입 선행시간 (s)", "float", 0.0, 30.0, 0.5, 1),
                    FieldSpec("path_deviation", "turn_radius_30_m", "30m/s 기준 반경 (m)", "float", 1.0, 5000.0, 10.0, 1),
                    FieldSpec("path_deviation", "turn_radius_40_m", "40m/s 기준 반경 (m)", "float", 1.0, 5000.0, 10.0, 1),
                    FieldSpec("path_deviation", "turn_radius_50_m", "50m/s 기준 반경 (m)", "float", 1.0, 5000.0, 10.0, 1),
                    FieldSpec("path_deviation", "coord_heading_fallback_min_delta_deg", "좌표 heading fallback 최소차 (deg)", "float", 0.0, 180.0, 1.0, 1),
                    FieldSpec("path_deviation", "coord_heading_fallback_max_delta_deg", "좌표 heading fallback 최대차 (deg)", "float", 0.0, 180.0, 1.0, 1),
                    FieldSpec("path_deviation", "coord_heading_fallback_max_turn_dps", "좌표 heading fallback 최대 선회율 (deg/s)", "float", 0.0, 90.0, 0.5, 1),
                ],
            ),
            (
                "촬영품질 / 속도",
                "촬영품질 기반 속도 재계획에서 쓰는 샘플 수와 비율 기준입니다.",
                [
                    FieldSpec("quality_speed", "lower_band_ratio", "하한 비율", "float", 0.1, 1.0, 0.01, 2),
                    FieldSpec("quality_speed", "search_speed_up_scale", "속도 상향 배수", "float", 0.1, 5.0, 0.01, 2),
                    FieldSpec("quality_speed", "search_speed_down_scale", "속도 하향 배수", "float", 0.1, 5.0, 0.01, 2),
                    FieldSpec("quality_speed", "min_sample_count", "최소 샘플 수", "int", 1, 100, 1),
                    FieldSpec("quality_speed", "max_sample_count", "최대 샘플 수", "int", 1, 500, 1),
                    FieldSpec("quality_speed", "startup_grace_sec", "시작 유예시간 (s)", "float", 0.0, 60.0, 0.5, 1),
                    FieldSpec("quality_speed", "disabled_flight_mode", "비활성 flight mode", "int", 0, 100, 1),
                ],
            ),
            (
                "입력갱신 / 선행임무 / DL 위험",
                "중복 입력, 선행임무, DL 평균 위험 판정에 쓰는 기준입니다.",
                [
                    FieldSpec("input_refresh", "duplicate_window_ms", "명령 중복 방지 창 (ms)", "int", 0, 5000, 50),
                    FieldSpec("input_refresh", "block_when_reexecute_active", "재실행 활성 중 중복 차단", "bool"),
                    FieldSpec("prior_mission", "dl_risk_threshold", "선행임무 DL 위험 임계값", "float", 0.0, 1.0, 0.01, 2),
                    FieldSpec("dl_risk", "mean_risk_threshold", "DL 평균 risk 임계값", "float", 0.0, 1.0, 0.01, 2),
                    FieldSpec("dl_risk", "cooldown_sec", "DL 위험 cooldown (s)", "float", 0.0, 300.0, 1.0, 1),
                ],
            ),
            (
                "강제대기 / RTB",
                "강제대기 재계획과 RTB 비정상 판단 기준입니다.",
                [
                    FieldSpec("forced_command", "hold_delay_seconds", "강제대기 hold (s)", "float", 0.0, 120.0, 0.5, 1),
                    FieldSpec("forced_command", "signature_dedup_seconds", "명령 중복 방지 (s)", "float", 0.0, 10.0, 0.1, 2),
                    FieldSpec("rtb", "unexpected_rtb_flight_mode", "예상 밖 RTB flight mode", "int", 0, 100, 1),
                    FieldSpec("rtb", "abnormal_health_value", "비정상 health 값", "int", 0, 100, 1),
                    FieldSpec("rtb", "fuel_warning_replan_level", "연료 경고 level", "int", 0, 10, 1),
                    FieldSpec("rtb", "signal_loss_grace_ms", "신호 손실 유예 (ms)", "int", 0, 60000, 100),
                    FieldSpec("rtb", "replan_hold_ms", "RTB hold (ms)", "int", 0, 60000, 100),
                    FieldSpec("rtb", "fault_unavailable_hold_ms", "고장/통신/장비 비가용 hold (ms)", "int", 0, 120000, 1000),
                ],
            ),
            (
                "촬영계획",
                "촬영 일정 기반 재계획 트리거 조건을 조정합니다.",
                [
                    FieldSpec("imaging_schedule", "trigger_probability", "촬영 트리거 확률", "float", 0.0, 1.0, 0.01, 2),
                    FieldSpec("imaging_schedule", "imaging_operation_modes", "허용 operation mode 목록", "list", placeholder="1, 2, 3"),
                    FieldSpec("imaging_schedule", "imaging_pattern_types", "허용 pattern type 목록", "list", placeholder="3, 4, 5"),
                ],
            ),
            (
                "연료 임계값",
                "연료 경고와 재계획 판단에 쓰는 기본 임계값을 조정합니다.",
                [
                    FieldSpec("fuel_threshold", "capacity_liters", "연료 총량 (L)", "float", 0.1, 1000.0, 0.1, 1),
                    FieldSpec("fuel_threshold", "yellow_ratio", "연료 Yellow 비율", "float", 0.0, 1.0, 0.01, 2),
                    FieldSpec("fuel_threshold", "red_ratio", "연료 Red 비율", "float", 0.0, 1.0, 0.01, 2),
                ],
            ),
            (
                "표적탐지 고급",
                "표적탐지 후속 재계획과 대상 option preset을 직접 조정합니다.",
                [
                    FieldSpec("target_detection", "cooldown_ms", "탐지 cooldown (ms)", "int", 0, 600000, 100),
                    FieldSpec("target_detection", "watcher_uav_ids", "watcher UAV IDs", "list", placeholder="4, 5, 6"),
                    FieldSpec("target_detection", "attack_manned_ids", "attack manned IDs", "list", placeholder="2, 3"),
                    FieldSpec(
                        "target_detection",
                        "target_type_priority",
                        "target type priority",
                        "list",
                        placeholder="1, 2, 3, 4, 5, 6",
                    ),
                    FieldSpec(
                        "target_detection",
                        "option_presets",
                        "option presets (JSON)",
                        "json",
                        placeholder='[{"option_id": 2, "option_name": "공격 활성"}]',
                    ),
                ],
            ),
            (
                "공격 후 복귀 재계획",
                "0402에서 표적 파괴가 확정되면 복귀 UAV의 재합류가 이득인지 판단합니다. "
                "남은 임무 시간, 복귀 ETA, 선회 반경 보정을 함께 사용합니다.",
                [
                    FieldSpec("post_attack_rejoin", "closure_cooldown_ms", "종료 트리거 중복 방지 (ms)", "int", 0, 300000, 100),
                    FieldSpec("post_attack_rejoin", "min_remaining_eta_s", "재계획 최소 잔여 ETA (s)", "int", 0, 3600, 5),
                    FieldSpec("post_attack_rejoin", "rejoin_margin_s", "복귀 여유시간 (s)", "int", 0, 1800, 5),
                    FieldSpec("post_attack_rejoin", "turn_radius_m", "복귀 선회 반경 (m)", "float", 1.0, 5000.0, 5.0, 1),
                    FieldSpec("post_attack_rejoin", "default_cruise_speed_mps", "기본 복귀 속도 (m/s)", "float", 1.0, 200.0, 0.5, 1),
                    FieldSpec("post_attack_rejoin", "active_progress_skip_percent", "active UAV 평균 progress 스킵 (%)", "int", 0, 100, 1),
                ],
            ),
            (
                "Replan Queue",
                "한 번에 하나의 재계획만 active로 유지하고 나머지는 queue에서 순차 처리합니다.",
                [
                    FieldSpec("replan_queue", "active_timeout_ms", "active timeout (ms)", "int", 1000, 300000, 500),
                    FieldSpec("replan_queue", "history_limit", "history limit", "int", 5, 200, 1),
                    FieldSpec("replan_queue", "target_dispatch_delay_ms", "target burst delay (ms)", "int", 0, 10000, 100),
                    FieldSpec("replan_queue", "release_on_option_info", "release on 0701", "bool"),
                    FieldSpec(
                        "replan_queue",
                        "suppress_active_target_options_on_new_detection",
                        "옵션 전 새 표적 시 중단",
                        "bool",
                    ),
                ],
            ),
        ]
    def _toggle_groups(self) -> list[tuple[str, str, int, list[tuple[str, str, str]]]]:
        return [
            (
                "모니터링 / 표시",
                "재계획 트리거 자체가 아니라 모니터링 화면과 상태 갱신을 제어하는 항목입니다.",
                1,
                [
                    ("quality_monitor", "촬영품질 모니터", "품질 상태 카드와 품질 추세를 갱신"),
                ],
            ),
            (
                "재계획 트리거",
                "조건 충족 시 0902 재계획 요청까지 이어지는 자동 판단 로직 스위치입니다.",
                3,
                [
                    ("path_deviation", "경로추종", "0401 선회/Orbit 기반 재계획"),
                    ("quality_speed", "품질속도", "속도 재계획 판단"),
                    ("forced_command", "강제대기", "0802 강제대기 재계획"),
                    ("input_refresh", "입력갱신", "중복/재실행 입력 처리"),
                    ("prior_mission", "선행임무", "선행임무 위험 재계획"),
                    ("dl_risk", "DL 위험", "학습 기반 위험 판단"),
                    ("imaging_schedule", "촬영계획", "촬영 일정 기반 재계획"),
                    ("next_collab", "다음협업", "다음 임무 진입"),
                    ("rtb", "RTB", "복귀 판단"),
                    ("target_detection", "표적탐지", "0402 탐지 후속 재계획"),
                    ("post_attack_rejoin", "공격후복귀", "공격 성공 후 추적 UAV 복귀 판단"),
                    ("fuel_threshold", "연료", "연료 임계치 자동판단"),
                ],
            ),
        ]

    def _parameter_sections_legacy(self) -> list[tuple[str, str, list[FieldSpec]]]:
        return [
            (
                "경로추종 / 선회 시작",
                "0401 heading 변화와 수신 상태를 바탕으로 초기 선회 징후를 판단합니다.",
                [
                    FieldSpec("path_deviation", "turn_rate_threshold_dps", "선회율 임계값 (deg/s)", "float", 0.1, 10.0, 0.05, 2),
                    FieldSpec("path_deviation", "turn_window_s", "선회 계산 창 (s)", "float", 2.0, 30.0, 0.5, 1),
                    FieldSpec("path_deviation", "stale_timeout_s", "수신 지연 timeout (s)", "float", 0.5, 20.0, 0.5, 1),
                    FieldSpec("path_deviation", "heading_move_min_m", "좌표 heading 최소 이동거리 (m)", "float", 0.0, 100.0, 0.5, 1),
                    FieldSpec("path_deviation", "turn_gap_reset_s", "선회 추적 reset 간격 (s)", "float", 0.1, 10.0, 0.1, 1),
                ],
            ),
            (
                "경로추종 / Orbit 추적",
                "현재 WP 주변을 반복 선회하는지 누적 관찰해 watch/warning 상태를 판단합니다.",
                [
                    FieldSpec("path_deviation", "spiral_window_s", "누적 감시 창 (s)", "float", 5.0, 180.0, 1.0, 1),
                    FieldSpec("path_deviation", "spiral_min_points", "최소 샘플 수", "int", 3, 50, 1),
                    FieldSpec("path_deviation", "center_ignore_radius_m", "WP 중심 무시 반경 (m)", "float", 0.0, 500.0, 5.0, 1),
                    FieldSpec("path_deviation", "accumulation_step_max_deg", "누적 step 최대각 (deg)", "float", 10.0, 360.0, 5.0, 1),
                    FieldSpec("path_deviation", "watch_angle_deg", "주의 각도 (deg)", "float", 10.0, 360.0, 5.0, 1),
                    FieldSpec("path_deviation", "warning_angle_deg", "경고 각도 (deg)", "float", 20.0, 360.0, 5.0, 1),
                    FieldSpec("path_deviation", "hold_s", "경고 유지 시간 (s)", "float", 0.0, 20.0, 0.5, 1),
                ],
            ),
            (
                "경로추종 / 반경 허용",
                "실제 선회 반경과 기준 반경의 차이를 어느 정도까지 허용할지 조정합니다.",
                [
                    FieldSpec("path_deviation", "near_threshold_min_m", "WP 근접 최소거리 (m)", "float", 1.0, 2000.0, 5.0, 1),
                    FieldSpec("path_deviation", "near_threshold_radius_factor", "WP 근접 반경배수", "float", 1.0, 10.0, 0.05, 2),
                    FieldSpec("path_deviation", "radius_error_min_m", "평균반경 허용 최소값 (m)", "float", 1.0, 2000.0, 5.0, 1),
                    FieldSpec("path_deviation", "radius_error_factor", "평균반경 허용 배수", "float", 0.05, 5.0, 0.05, 2),
                    FieldSpec("path_deviation", "latest_radius_min_m", "최종반경 허용 최소값 (m)", "float", 1.0, 2000.0, 5.0, 1),
                    FieldSpec("path_deviation", "latest_radius_factor", "최종반경 허용 배수", "float", 0.01, 5.0, 0.01, 2),
                    FieldSpec("path_deviation", "release_min_distance_m", "해제 최소거리 (m)", "float", 1.0, 5000.0, 10.0, 1),
                    FieldSpec("path_deviation", "release_factor", "해제 반경배수", "float", 1.0, 10.0, 0.05, 2),
                    FieldSpec("path_deviation", "adaptive_enabled", "Adaptive correction", "bool"),
                    FieldSpec("path_deviation", "adaptive_sample_min_interval_s", "Adaptive sample interval (s)", "float", 0.2, 10.0, 0.05, 2),
                    FieldSpec("path_deviation", "adaptive_ema_alpha", "Adaptive EMA alpha", "float", 0.01, 1.0, 0.01, 2),
                    FieldSpec("path_deviation", "adaptive_warmup_samples", "Adaptive warmup samples", "int", 1, 100, 1),
                    FieldSpec("path_deviation", "adaptive_min_scale", "Adaptive min scale", "float", 0.1, 5.0, 0.01, 2),
                    FieldSpec("path_deviation", "adaptive_max_scale", "Adaptive max scale", "float", 0.1, 5.0, 0.01, 2),
                    FieldSpec("path_deviation", "adaptive_threshold_gain", "Adaptive threshold gain", "float", 0.0, 1.0, 0.01, 2),
                    FieldSpec("path_deviation", "adaptive_save_interval_s", "Adaptive save interval (s)", "float", 1.0, 120.0, 1.0, 1),
                ],
            ),
            (
                "경로추종 / 대체 WP 계산",
                "warning 상태에서 synthetic 대체 WP와 다음 임무 진입점을 계산하는 기준값입니다.",
                [
                    FieldSpec("path_deviation", "alt_waypoint_trigger_s", "대체 WP 트리거 지연 (s)", "float", 0.0, 20.0, 0.5, 1),
                    FieldSpec("path_deviation", "alt_waypoint_lead_time_s", "대체 WP 선행시간 (s)", "float", 0.0, 30.0, 0.5, 1),
                    FieldSpec("path_deviation", "next_mission_entry_lead_time_s", "다음 임무 진입 선행시간 (s)", "float", 0.0, 30.0, 0.5, 1),
                    FieldSpec("path_deviation", "turn_radius_30_m", "30m/s 기준 반경 (m)", "float", 1.0, 5000.0, 10.0, 1),
                    FieldSpec("path_deviation", "turn_radius_40_m", "40m/s 기준 반경 (m)", "float", 1.0, 5000.0, 10.0, 1),
                    FieldSpec("path_deviation", "turn_radius_50_m", "50m/s 기준 반경 (m)", "float", 1.0, 5000.0, 10.0, 1),
                    FieldSpec("path_deviation", "coord_heading_fallback_min_delta_deg", "좌표 heading fallback 최소차 (deg)", "float", 0.0, 180.0, 1.0, 1),
                    FieldSpec("path_deviation", "coord_heading_fallback_max_delta_deg", "좌표 heading fallback 최대차 (deg)", "float", 0.0, 180.0, 1.0, 1),
                    FieldSpec("path_deviation", "coord_heading_fallback_max_turn_dps", "좌표 heading fallback 최대 선회율 (deg/s)", "float", 0.0, 90.0, 0.5, 1),
                ],
            ),
            (
                "촬영품질 / 속도",
                "촬영품질 기반 속도 재계획에서 사용하는 표본 수와 비율 기준입니다.",
                [
                    FieldSpec("quality_speed", "lower_band_ratio", "하한 비율", "float", 0.1, 1.0, 0.01, 2),
                    FieldSpec("quality_speed", "search_speed_up_scale", "속도 상향 배수", "float", 0.1, 5.0, 0.01, 2),
                    FieldSpec("quality_speed", "search_speed_down_scale", "속도 하향 배수", "float", 0.1, 5.0, 0.01, 2),
                    FieldSpec("quality_speed", "min_sample_count", "최소 샘플 수", "int", 1, 100, 1),
                    FieldSpec("quality_speed", "max_sample_count", "최대 샘플 수", "int", 1, 500, 1),
                    FieldSpec("quality_speed", "startup_grace_sec", "시작 유예시간 (s)", "float", 0.0, 60.0, 0.5, 1),
                    FieldSpec("quality_speed", "disabled_flight_mode", "비활성 flight mode", "int", 0, 100, 1),
                ],
            ),
            (
                "입력갱신 / 선행임무 / DL 위험",
                "중복 입력, 선행임무, DL 평균 위험 판단에 공통으로 쓰는 기준값입니다.",
                [
                    FieldSpec("input_refresh", "duplicate_window_ms", "명령 중복 방지 창 (ms)", "int", 0, 5000, 50),
                    FieldSpec("input_refresh", "block_when_reexecute_active", "재실행 활성 중 중복 차단", "bool"),
                    FieldSpec("prior_mission", "dl_risk_threshold", "선행임무 DL 위험 임계값", "float", 0.0, 1.0, 0.01, 2),
                    FieldSpec("dl_risk", "mean_risk_threshold", "DL 평균 risk 임계값", "float", 0.0, 1.0, 0.01, 2),
                    FieldSpec("dl_risk", "cooldown_sec", "DL 위험 cooldown (s)", "float", 0.0, 300.0, 1.0, 1),
                ],
            ),
            (
                "강제대기 / RTB",
                "강제대기 재계획과 RTB 비정상 판단 기준값입니다.",
                [
                    FieldSpec("forced_command", "hold_delay_seconds", "강제대기 hold (s)", "float", 0.0, 120.0, 0.5, 1),
                    FieldSpec("forced_command", "signature_dedup_seconds", "명령 중복 방지 (s)", "float", 0.0, 10.0, 0.1, 2),
                    FieldSpec("rtb", "unexpected_rtb_flight_mode", "예상 밖 RTB flight mode", "int", 0, 100, 1),
                    FieldSpec("rtb", "abnormal_health_value", "비정상 health 값", "int", 0, 100, 1),
                    FieldSpec("rtb", "fuel_warning_replan_level", "연료 경고 level", "int", 0, 10, 1),
                    FieldSpec("rtb", "signal_loss_grace_ms", "신호 손실 유예 (ms)", "int", 0, 60000, 100),
                    FieldSpec("rtb", "replan_hold_ms", "RTB hold (ms)", "int", 0, 60000, 100),
                    FieldSpec("rtb", "fault_unavailable_hold_ms", "고장/통신/장비 비가용 hold (ms)", "int", 0, 120000, 1000),
                ],
            ),
            (
                "촬영계획",
                "촬영 일정 기반 재계획의 트리거 조건을 조정합니다.",
                [
                    FieldSpec("imaging_schedule", "trigger_probability", "촬영 트리거 확률", "float", 0.0, 1.0, 0.01, 2),
                    FieldSpec("imaging_schedule", "imaging_operation_modes", "허용 operation mode 목록", "list", placeholder="1, 2, 3"),
                    FieldSpec("imaging_schedule", "imaging_pattern_types", "허용 pattern type 목록", "list", placeholder="3, 4, 5"),
                ],
            ),
            (
                "연료 임계값",
                "연료 경고와 재계획 판단에 쓰는 기본 임계값입니다.",
                [
                    FieldSpec("fuel_threshold", "capacity_liters", "연료 총량 (L)", "float", 0.1, 1000.0, 0.1, 1),
                    FieldSpec("fuel_threshold", "yellow_ratio", "연료 Yellow 비율", "float", 0.0, 1.0, 0.01, 2),
                    FieldSpec("fuel_threshold", "red_ratio", "연료 Red 비율", "float", 0.0, 1.0, 0.01, 2),
                ],
            ),
            (
                "표적탐지 고급",
                "표적탐지 후속 재계획과 공격 option preset을 직접 조정합니다.",
                [
                    FieldSpec("target_detection", "cooldown_ms", "탐지 cooldown (ms)", "int", 0, 600000, 100),
                    FieldSpec("target_detection", "watcher_uav_ids", "watcher UAV IDs", "list", placeholder="4, 5, 6"),
                    FieldSpec("target_detection", "attack_manned_ids", "attack manned IDs", "list", placeholder="2, 3"),
                    FieldSpec(
                        "target_detection",
                        "target_type_priority",
                        "target type priority",
                        "list",
                        placeholder="1, 2, 3, 4, 5, 6",
                    ),
                    FieldSpec(
                        "target_detection",
                        "option_presets",
                        "option presets (JSON)",
                        "json",
                        placeholder='[{"option_id": 2, "option_name": "공격 특화"}]',
                    ),
                ],
            ),
            (
                "공격 후 복귀 재계획",
                "0402에서 isDestroyed=true가 들어온 뒤, 복귀 UAV를 다시 협업 임무에 투입할지 판단하는 기준값입니다.",
                [
                    FieldSpec("post_attack_rejoin", "closure_cooldown_ms", "종결 트리거 중복방지 (ms)", "int", 0, 600000, 100),
                    FieldSpec("post_attack_rejoin", "min_remaining_eta_s", "최소 남은 임무시간 (s)", "int", 0, 7200, 10),
                    FieldSpec("post_attack_rejoin", "rejoin_margin_s", "복귀 여유시간 (s)", "int", 0, 3600, 5),
                    FieldSpec("post_attack_rejoin", "turn_radius_m", "복귀 선회 반경 (m)", "float", 1.0, 5000.0, 10.0, 1),
                    FieldSpec("post_attack_rejoin", "default_cruise_speed_mps", "기본 복귀 속도 (m/s)", "float", 1.0, 150.0, 0.5, 1),
                    FieldSpec("post_attack_rejoin", "active_progress_skip_percent", "active UAV 평균 progress 스킵 (%)", "int", 0, 100, 1),
                ],
            ),
            (
                "Replan Queue",
                "한 번에 하나의 재계획만 active로 유지하고 나머지는 queue에서 순차 처리합니다.",
                [
                    FieldSpec("replan_queue", "active_timeout_ms", "active timeout (ms)", "int", 1000, 300000, 500),
                    FieldSpec("replan_queue", "history_limit", "history limit", "int", 5, 200, 1),
                    FieldSpec("replan_queue", "target_dispatch_delay_ms", "target burst delay (ms)", "int", 0, 10000, 100),
                    FieldSpec("replan_queue", "release_on_option_info", "release on 0701", "bool"),
                    FieldSpec(
                        "replan_queue",
                        "suppress_active_target_options_on_new_detection",
                        "옵션 전 새 표적 시 중단",
                        "bool",
                    ),
                ],
            ),
        ]


__all__ = ["ReplanManagementTab", "ToggleCard"]
