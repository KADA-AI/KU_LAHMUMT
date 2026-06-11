from __future__ import annotations

import copy
import json
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFontDatabase
from PyQt5.QtWidgets import (
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFrame,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QVBoxLayout,
    QWidget,
)


@dataclass(frozen=True)
class MessageFieldSpec:
    name: str
    type_name: str


_FIELD_PATTERN = re.compile(
    r"^(?P<type>List<[^>]+>|[A-Za-z_][A-Za-z0-9_.<>]*)\s+"
    r"(?P<name>[A-Za-z_][A-Za-z0-9_]*)\s*:?\s*$"
)

_ENUM_OPTIONS: dict[str, dict[str, list[tuple[str, Any]]]] = {
    "0102": {
        "status": [("0 | Unknown", 0), ("1 | Normal", 1), ("2 | Fault", 2)],
    },
    "0305": {
        "missionPlanningStatus": [
            ("0 | Unknown", 0),
            ("1 | Idle", 1),
            ("2 | Running", 2),
            ("3 | Failed", 3),
        ],
    },
}

_MULTILINE_FIELDS = {"contents", "replanReason"}


def _spec_dir_for(msg_id: str) -> Path:
    return Path(__file__).resolve().parent / "nFusion_MessageLIbrary" / f"msg_{str(msg_id).zfill(4)}"


def load_message_field_specs(msg_id: str) -> list[MessageFieldSpec]:
    spec_dir = _spec_dir_for(msg_id)
    if not spec_dir.exists():
        return []

    try:
        spec_path = next(iter(sorted(spec_dir.glob("*.nfpsh"))))
    except StopIteration:
        return []

    specs: list[MessageFieldSpec] = []
    try:
        lines = spec_path.read_text(encoding="utf-8", errors="ignore").splitlines()
    except Exception:
        return specs

    for raw_line in lines:
        line = raw_line.strip()
        if not line or line.startswith("#") or line.startswith("-"):
            continue
        match = _FIELD_PATTERN.match(line)
        if not match:
            continue
        specs.append(MessageFieldSpec(name=match.group("name"), type_name=match.group("type")))
    return specs


def _ordered_specs(msg_id: str, payload: dict[str, Any]) -> list[MessageFieldSpec]:
    ordered = load_message_field_specs(msg_id)
    known = {spec.name for spec in ordered}
    for key, value in (payload or {}).items():
        if key in known:
            continue
        ordered.append(MessageFieldSpec(name=str(key), type_name=_guess_type_name(value)))
    return ordered


def _guess_type_name(value: Any) -> str:
    if isinstance(value, bool):
        return "bool"
    if isinstance(value, int):
        return "int"
    if isinstance(value, float):
        return "float"
    if isinstance(value, list):
        return "List<object>"
    if isinstance(value, dict):
        return "object"
    return "string"


def _json_text(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, indent=2)


class MessagePayloadDialog(QDialog):
    def __init__(
        self,
        msg_id: str,
        message_name: str,
        initial_payload: dict[str, Any] | None,
        *,
        periodic_rate_hz: float | None = None,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self._msg_id = str(msg_id).zfill(4)
        self._message_name = str(message_name or self._msg_id)
        self._initial_payload = copy.deepcopy(initial_payload or {})
        self._payload: dict[str, Any] | None = None
        self._error_label: QLabel | None = None
        self._editors: dict[str, tuple[MessageFieldSpec, QWidget]] = {}

        self.setModal(True)
        self.setWindowTitle(f"{self._msg_id} 발신 값 확인")
        self.resize(680, 560)

        root = QVBoxLayout(self)
        root.setContentsMargins(18, 18, 18, 18)
        root.setSpacing(12)

        title = QLabel(f"{self._msg_id} | {self._message_name}")
        title.setObjectName("SectionLabel")
        root.addWidget(title)

        desc = QLabel("ICD 기준 필드를 임시로 채운 상태입니다. 값 수정 후 발신할 수 있습니다.")
        desc.setWordWrap(True)
        desc.setObjectName("HintLabel")
        root.addWidget(desc)

        if periodic_rate_hz:
            note = QLabel(f"이 값으로 {periodic_rate_hz:g}Hz 주기 송신을 시작합니다.")
            note.setObjectName("InfoBadge")
            root.addWidget(note)

        scroll = QScrollArea(self)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        container = QWidget(scroll)
        form = QFormLayout(container)
        form.setContentsMargins(0, 0, 0, 0)
        form.setHorizontalSpacing(16)
        form.setVerticalSpacing(12)
        form.setFieldGrowthPolicy(QFormLayout.AllNonFixedFieldsGrow)

        for spec in _ordered_specs(self._msg_id, self._initial_payload):
            label = QLabel(f"{spec.name} ({spec.type_name})")
            label.setObjectName("FieldCaption")
            editor = self._make_editor(spec, self._initial_payload.get(spec.name))
            self._editors[spec.name] = (spec, editor)
            form.addRow(label, editor)

        scroll.setWidget(container)
        root.addWidget(scroll, 1)

        self._error_label = QLabel("")
        self._error_label.setStyleSheet("color:#b42318; font-weight:600;")
        self._error_label.setWordWrap(True)
        self._error_label.hide()
        root.addWidget(self._error_label)

        footer = QHBoxLayout()
        footer.setContentsMargins(0, 0, 0, 0)
        footer.setSpacing(8)

        btn_reset = QPushButton("기본값 복원", self)
        btn_reset.setObjectName("SecondaryButton")
        btn_reset.clicked.connect(self._restore_defaults)
        footer.addWidget(btn_reset)
        footer.addStretch(1)

        btns = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, parent=self)
        ok_btn = btns.button(QDialogButtonBox.Ok)
        cancel_btn = btns.button(QDialogButtonBox.Cancel)
        if ok_btn is not None:
            ok_btn.setText("발신")
        if cancel_btn is not None:
            cancel_btn.setText("취소")
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)
        footer.addWidget(btns)
        root.addLayout(footer)

    @property
    def payload(self) -> dict[str, Any] | None:
        return copy.deepcopy(self._payload)

    def _make_editor(self, spec: MessageFieldSpec, value: Any) -> QWidget:
        enum_items = _ENUM_OPTIONS.get(self._msg_id, {}).get(spec.name)
        if enum_items:
            combo = QComboBox(self)
            combo.setEditable(False)
            combo.setMinimumHeight(34)
            for label, item_value in enum_items:
                combo.addItem(label, item_value)
            current = value if value is not None else enum_items[0][1]
            for index in range(combo.count()):
                if combo.itemData(index) == current:
                    combo.setCurrentIndex(index)
                    break
            return combo

        if self._is_json_field(spec, value):
            editor = QPlainTextEdit(self)
            editor.setMinimumHeight(140 if spec.name in _MULTILINE_FIELDS else 180)
            try:
                editor.setFont(QFontDatabase.systemFont(QFontDatabase.FixedFont))
            except Exception:
                pass
            if value in (None, ""):
                if spec.type_name.startswith("List<"):
                    editor.setPlainText("[]")
                elif spec.type_name == "object":
                    editor.setPlainText("{}")
                else:
                    editor.setPlainText("")
            else:
                editor.setPlainText(_json_text(value) if isinstance(value, (dict, list)) else str(value))
            return editor

        if spec.type_name == "bool":
            combo = QComboBox(self)
            combo.setEditable(False)
            combo.setMinimumHeight(34)
            combo.addItem("false", False)
            combo.addItem("true", True)
            combo.setCurrentIndex(1 if bool(value) else 0)
            return combo

        if spec.name in _MULTILINE_FIELDS:
            editor = QPlainTextEdit(self)
            editor.setMinimumHeight(110)
            editor.setPlainText("" if value is None else str(value))
            return editor

        editor = QLineEdit(self)
        editor.setMinimumHeight(34)
        if value is not None:
            editor.setText(str(value))
        return editor

    @staticmethod
    def _is_json_field(spec: MessageFieldSpec, value: Any) -> bool:
        if isinstance(value, (dict, list)):
            return True
        type_name = str(spec.type_name or "")
        return type_name.startswith("List<") or type_name not in {
            "ulong",
            "uint",
            "uint32",
            "uint64",
            "int",
            "int32",
            "int64",
            "float",
            "float32",
            "float64",
            "double",
            "bool",
            "string",
        }

    def _restore_defaults(self) -> None:
        for spec in _ordered_specs(self._msg_id, self._initial_payload):
            _, editor = self._editors[spec.name]
            value = self._initial_payload.get(spec.name)
            if isinstance(editor, QComboBox):
                matched = False
                for index in range(editor.count()):
                    if editor.itemData(index) == value:
                        editor.setCurrentIndex(index)
                        matched = True
                        break
                if not matched and editor.count():
                    editor.setCurrentIndex(0)
                continue
            if isinstance(editor, QPlainTextEdit):
                if value in (None, ""):
                    if spec.type_name.startswith("List<"):
                        editor.setPlainText("[]")
                    elif spec.type_name == "object":
                        editor.setPlainText("{}")
                    else:
                        editor.setPlainText("")
                else:
                    editor.setPlainText(_json_text(value) if isinstance(value, (dict, list)) else str(value))
                continue
            if isinstance(editor, QLineEdit):
                editor.setText("" if value is None else str(value))

        if self._error_label is not None:
            self._error_label.hide()
            self._error_label.clear()

    def _collect_payload(self) -> dict[str, Any]:
        payload: dict[str, Any] = {}
        for name, (spec, editor) in self._editors.items():
            payload[name] = self._parse_editor_value(spec, editor)
        return payload

    def _parse_editor_value(self, spec: MessageFieldSpec, editor: QWidget) -> Any:
        if isinstance(editor, QComboBox):
            return editor.currentData()

        if isinstance(editor, QPlainTextEdit):
            text = editor.toPlainText().strip()
            if self._is_json_field(spec, self._initial_payload.get(spec.name)):
                if not text:
                    return [] if spec.type_name.startswith("List<") else {}
                try:
                    return json.loads(text)
                except json.JSONDecodeError as exc:
                    raise ValueError(f"{spec.name}: JSON 형식이 올바르지 않습니다. ({exc.msg})") from exc
            return text

        if not isinstance(editor, QLineEdit):
            return None

        text = editor.text().strip()
        type_name = str(spec.type_name or "").lower()

        if type_name in {"ulong", "uint", "uint32", "uint64", "int", "int32", "int64"}:
            if not text:
                return 0
            try:
                return int(text, 10)
            except Exception as exc:
                raise ValueError(f"{spec.name}: 정수 값을 입력해야 합니다.") from exc

        if type_name in {"float", "float32", "float64", "double"}:
            if not text:
                return 0.0
            try:
                return float(text)
            except Exception as exc:
                raise ValueError(f"{spec.name}: 실수 값을 입력해야 합니다.") from exc

        if type_name == "bool":
            return text.lower() in {"1", "true", "yes", "y", "on"}

        return text

    def accept(self) -> None:
        try:
            self._payload = self._collect_payload()
        except ValueError as exc:
            if self._error_label is not None:
                self._error_label.setText(str(exc))
                self._error_label.show()
            return
        super().accept()


class JsonPayloadBatchDialog(QDialog):
    def __init__(
        self,
        msg_id: str,
        message_name: str,
        payloads: list[dict[str, Any]],
        *,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self._payloads: list[dict[str, Any]] | None = None
        self.setModal(True)
        self.setWindowTitle(f"{str(msg_id).zfill(4)} 일괄 발신 값 확인")
        self.resize(760, 560)

        root = QVBoxLayout(self)
        root.setContentsMargins(18, 18, 18, 18)
        root.setSpacing(12)

        title = QLabel(f"{str(msg_id).zfill(4)} | {message_name}")
        title.setObjectName("SectionLabel")
        root.addWidget(title)

        desc = QLabel("배열의 각 항목이 실제 송신 1건입니다. 순서대로 순차 발신합니다.")
        desc.setWordWrap(True)
        desc.setObjectName("HintLabel")
        root.addWidget(desc)

        self._editor = QPlainTextEdit(self)
        self._editor.setPlainText(_json_text(payloads))
        try:
            self._editor.setFont(QFontDatabase.systemFont(QFontDatabase.FixedFont))
        except Exception:
            pass
        root.addWidget(self._editor, 1)

        self._error_label = QLabel("")
        self._error_label.setStyleSheet("color:#b42318; font-weight:600;")
        self._error_label.setWordWrap(True)
        self._error_label.hide()
        root.addWidget(self._error_label)

        footer = QHBoxLayout()
        footer.setContentsMargins(0, 0, 0, 0)
        footer.setSpacing(8)

        btn_reset = QPushButton("기본값 복원", self)
        btn_reset.setObjectName("SecondaryButton")
        btn_reset.clicked.connect(lambda: self._editor.setPlainText(_json_text(payloads)))
        footer.addWidget(btn_reset)
        footer.addStretch(1)

        btns = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel, parent=self)
        ok_btn = btns.button(QDialogButtonBox.Ok)
        cancel_btn = btns.button(QDialogButtonBox.Cancel)
        if ok_btn is not None:
            ok_btn.setText("발신")
        if cancel_btn is not None:
            cancel_btn.setText("취소")
        btns.accepted.connect(self.accept)
        btns.rejected.connect(self.reject)
        footer.addWidget(btns)
        root.addLayout(footer)

    @property
    def payloads(self) -> list[dict[str, Any]] | None:
        return copy.deepcopy(self._payloads)

    def accept(self) -> None:
        try:
            parsed = json.loads(self._editor.toPlainText().strip() or "[]")
        except json.JSONDecodeError as exc:
            self._error_label.setText(f"JSON 형식이 올바르지 않습니다. ({exc.msg})")
            self._error_label.show()
            return

        if not isinstance(parsed, list) or any(not isinstance(item, dict) for item in parsed):
            self._error_label.setText("최상위 값은 객체 목록(JSON 배열)이어야 합니다.")
            self._error_label.show()
            return

        self._payloads = parsed
        super().accept()
