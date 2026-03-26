from __future__ import annotations

import copy
import importlib
import json
import re
from pathlib import Path
from typing import Any, Iterable

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QDialog,
    QDialogButtonBox,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QPlainTextEdit,
    QSplitter,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from modules.common import db_paths

PROJECT_ROOT = Path(__file__).resolve().parents[2]
NFUSION_LIBRARY_ROOT = PROJECT_ROOT / "modules" / "common" / "nFusion_MessageLIbrary"

DB_MESSAGE_DIRS = {
    "0201": "InputMissionPlan",
    "0203": "MissionReferenceInfo",
    "0301": "MissionPlan",
    "0302": "IndividualMissionPlan",
    "0303": "FlightPath",
    "0304": "FlightPath",
    "0701": "MissionPlanOptionInfo",
}

SCALAR_TIME_KEYS = {
    "timestamp",
    "requesttime",
    "operatorreplanrequesttime",
}


def clone_payload(payload: Any) -> Any:
    try:
        return copy.deepcopy(payload)
    except Exception:
        try:
            return json.loads(json.dumps(payload, ensure_ascii=False))
        except Exception:
            return payload


def refresh_payload_timestamps(payload: Any, *, now_ms: int) -> Any:
    refreshed = clone_payload(payload)

    def _refresh_dict(node: dict[str, Any]) -> None:
        lowered = {str(key).lower(): key for key in list(node.keys())}
        for target in SCALAR_TIME_KEYS:
            real_key = lowered.get(target)
            if real_key is not None:
                node[real_key] = int(now_ms)
        nested_time_key = lowered.get("replanrequesttime")
        if nested_time_key is not None and isinstance(node.get(nested_time_key), dict):
            nested = node[nested_time_key]
            nested_lowered = {str(key).lower(): key for key in list(nested.keys())}
            nested_real = nested_lowered.get("replanrequesttimestamp")
            if nested_real is not None:
                nested[nested_real] = int(now_ms)

    def _walk(node: Any) -> None:
        if isinstance(node, dict):
            _refresh_dict(node)
            for value in node.values():
                _walk(value)
        elif isinstance(node, list):
            for value in node:
                _walk(value)

    _walk(refreshed)
    return refreshed


def _numeric_stem(path: Path) -> tuple[int, str]:
    stem = path.stem
    if stem.isdigit():
        return (int(stem), stem)
    return (10**18, stem)


def _read_json_dict(path: Path) -> dict[str, Any] | None:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except Exception:
        return None
    return payload if isinstance(payload, dict) else None


def _flight_path_matches_message(msg_id: str, payload: dict[str, Any]) -> bool:
    if msg_id == "0303":
        return isinstance(payload.get("waypointList"), list)
    if msg_id == "0304":
        return isinstance(payload.get("lahWaypointList"), list)
    return True


def load_db_payload_candidates(msg_id: str, *, payload_ids: Iterable[int] | None = None) -> list[dict[str, Any]]:
    dir_name = DB_MESSAGE_DIRS.get(str(msg_id).strip())
    if not dir_name:
        return []

    db_dir = db_paths.ensure_db_payload(dir_name)
    if not db_dir.exists():
        return []

    selected_files: list[Path] = []
    if payload_ids is not None:
        for raw_id in payload_ids:
            try:
                payload_id = int(raw_id)
            except Exception:
                continue
            path = db_dir / f"{payload_id}.json"
            if path.exists():
                selected_files.append(path)
    else:
        selected_files = sorted(db_dir.glob("*.json"), key=_numeric_stem)

    payloads: list[dict[str, Any]] = []
    for path in selected_files:
        payload = _read_json_dict(path)
        if not payload:
            continue
        if dir_name == "FlightPath" and not _flight_path_matches_message(str(msg_id).strip(), payload):
            continue
        payloads.append(payload)
    return payloads


def generate_default_payload(msg_id: str, *, source_code: str) -> dict[str, Any] | None:
    msg_id = str(msg_id).strip().zfill(4)
    try:
        module = importlib.import_module(f"modules.common.generator.message{msg_id}_generator")
        factory = getattr(module, f"make_msg{msg_id}_body")
    except Exception:
        return None

    try:
        payload = factory(source=source_code)
    except TypeError:
        try:
            payload = factory()
        except Exception:
            return None
    except Exception:
        return None

    return payload if isinstance(payload, dict) else None


def _schema_file_for_message(msg_id: str) -> Path | None:
    msg_dir = NFUSION_LIBRARY_ROOT / f"msg_{str(msg_id).strip().zfill(4)}"
    if not msg_dir.exists():
        return None
    for path in sorted(msg_dir.glob("*.nfpsh")):
        return path
    for path in sorted(msg_dir.glob("*.nftype")):
        return path
    return None


_FIELD_PATTERN = re.compile(r"^(?P<type>[A-Za-z0-9_.<>]+)\s+(?P<name>[A-Za-z0-9_]+):?$")


def _parse_schema_fields(path: Path) -> list[dict[str, Any]]:
    fields: list[dict[str, Any]] = []
    current: dict[str, Any] | None = None
    try:
        lines = path.read_text(encoding="utf-8", errors="ignore").splitlines()
    except Exception:
        return fields

    for raw_line in lines:
        stripped = raw_line.strip()
        if not stripped or stripped.startswith("#"):
            continue
        if stripped.startswith("-"):
            if current is not None:
                current.setdefault("notes", []).append(stripped[1:].strip())
            continue

        match = _FIELD_PATTERN.match(stripped)
        if not match:
            continue
        if current is not None:
            fields.append(current)
        current = {
            "type": match.group("type"),
            "name": match.group("name"),
            "notes": [],
        }

    if current is not None:
        fields.append(current)
    return fields


def _nested_schema_path(msg_id: str, field_type: str) -> Path | None:
    match = re.search(r"(?:List<)?(?:CommonType\.)?(?P<name>[A-Za-z0-9_]+)>?$", str(field_type).strip())
    if not match:
        return None
    type_name = match.group("name")
    if not type_name:
        return None

    msg_dir = NFUSION_LIBRARY_ROOT / f"msg_{str(msg_id).strip().zfill(4)}" / f"{type_name}.nftype"
    if msg_dir.exists():
        return msg_dir
    common_type = NFUSION_LIBRARY_ROOT / "CommonType" / f"{type_name}.nftype"
    if common_type.exists():
        return common_type
    return None


def build_icd_summary_text(msg_id: str) -> str:
    schema_path = _schema_file_for_message(msg_id)
    if schema_path is None:
        return "ICD 요약 파일을 찾지 못했습니다."

    fields = _parse_schema_fields(schema_path)
    if not fields:
        return "ICD 필드 정보를 읽지 못했습니다."

    lines = [f"스키마: {schema_path.name}"]
    for field in fields:
        notes = [str(item) for item in field.get("notes") or [] if str(item).strip()]
        nested_path = _nested_schema_path(msg_id, str(field.get("type") or ""))
        if nested_path is not None:
            nested_fields = _parse_schema_fields(nested_path)
            nested_names = [str(item.get("name") or "").strip() for item in nested_fields if str(item.get("name") or "").strip()]
            if nested_names:
                preview = ", ".join(nested_names[:8])
                if len(nested_names) > 8:
                    preview += ", ..."
                notes.append(f"하위필드: {preview}")
        note_text = " | ".join(notes) if notes else "설명 없음"
        lines.append(f"- {field.get('name')} ({field.get('type')}): {note_text}")
    return "\n".join(lines)


class TxComposeDialog(QDialog):
    def __init__(
        self,
        *,
        parent: QWidget | None,
        msg_id: str,
        msg_name: str,
        payload: dict[str, Any] | list[dict[str, Any]],
        icd_summary: str,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle(f"{msg_id} 발신 데이터 확인")
        self.resize(980, 760)
        self._original_payload = clone_payload(payload)

        root = QVBoxLayout(self)
        root.setContentsMargins(14, 14, 14, 14)
        root.setSpacing(10)

        header = QLabel(f"{msg_id}  {msg_name}")
        header.setStyleSheet("font-size:16px; font-weight:700; color:#0f172a;")
        root.addWidget(header)

        hint = QLabel("발신 전에 payload를 수정할 수 있습니다. dict 1건 또는 list[dict] 여러 건 JSON을 그대로 편집하면 됩니다.")
        hint.setWordWrap(True)
        hint.setStyleSheet("color:#475467;")
        root.addWidget(hint)

        splitter = QSplitter(Qt.Vertical, self)
        splitter.setChildrenCollapsible(False)

        info_wrap = QWidget(self)
        info_layout = QVBoxLayout(info_wrap)
        info_layout.setContentsMargins(0, 0, 0, 0)
        info_layout.setSpacing(6)
        info_label = QLabel("ICD 요약")
        info_label.setStyleSheet("font-weight:700; color:#344054;")
        info_layout.addWidget(info_label)
        self._icd_view = QTextEdit(info_wrap)
        self._icd_view.setReadOnly(True)
        self._icd_view.setPlainText(icd_summary)
        self._icd_view.setMinimumHeight(180)
        info_layout.addWidget(self._icd_view, 1)
        splitter.addWidget(info_wrap)

        editor_wrap = QWidget(self)
        editor_layout = QVBoxLayout(editor_wrap)
        editor_layout.setContentsMargins(0, 0, 0, 0)
        editor_layout.setSpacing(6)
        editor_label = QLabel("발신 payload")
        editor_label.setStyleSheet("font-weight:700; color:#344054;")
        editor_layout.addWidget(editor_label)
        self._editor = QPlainTextEdit(editor_wrap)
        editor_font = QFont("Consolas")
        editor_font.setStyleHint(QFont.Monospace)
        editor_font.setPointSize(10)
        self._editor.setFont(editor_font)
        self._editor.setPlainText(json.dumps(payload, ensure_ascii=False, indent=2))
        editor_layout.addWidget(self._editor, 1)
        splitter.addWidget(editor_wrap)

        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        root.addWidget(splitter, 1)

        button_row = QHBoxLayout()
        button_row.setContentsMargins(0, 0, 0, 0)
        button_row.setSpacing(8)
        self._btn_reset = QPushButton("기본값 복원", self)
        self._btn_reset.clicked.connect(self._restore_original_payload)
        button_row.addWidget(self._btn_reset)
        button_row.addStretch(1)

        buttons = QDialogButtonBox(QDialogButtonBox.Cancel | QDialogButtonBox.Ok, parent=self)
        ok_button = buttons.button(QDialogButtonBox.Ok)
        if ok_button is not None:
            ok_button.setText("발신")
        cancel_button = buttons.button(QDialogButtonBox.Cancel)
        if cancel_button is not None:
            cancel_button.setText("취소")
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        button_row.addWidget(buttons)
        root.addLayout(button_row)

    def _restore_original_payload(self) -> None:
        self._editor.setPlainText(json.dumps(self._original_payload, ensure_ascii=False, indent=2))

    def edited_payload(self) -> dict[str, Any] | list[dict[str, Any]] | None:
        raw_text = self._editor.toPlainText().strip()
        if not raw_text:
            QMessageBox.warning(self, "발신 데이터", "payload JSON이 비어 있습니다.")
            return None
        try:
            payload = json.loads(raw_text)
        except Exception as exc:
            QMessageBox.warning(self, "발신 데이터", f"JSON 파싱에 실패했습니다.\n{exc}")
            return None

        if isinstance(payload, dict):
            return payload
        if isinstance(payload, list) and all(isinstance(item, dict) for item in payload):
            return payload

        QMessageBox.warning(self, "발신 데이터", "payload는 dict 또는 list[dict] 형식이어야 합니다.")
        return None
