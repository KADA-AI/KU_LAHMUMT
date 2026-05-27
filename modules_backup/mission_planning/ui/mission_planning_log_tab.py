from __future__ import annotations

import json
from datetime import datetime
from typing import Any, Dict, List, Optional

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QLabel,
    QListWidget,
    QListWidgetItem,
    QPlainTextEdit,
    QSplitter,
    QVBoxLayout,
    QWidget,
)


def _format_timestamp(ts: Optional[float]) -> str:
    if ts is None:
        return "-"
    try:
        return datetime.fromtimestamp(ts).strftime("%H:%M:%S")
    except Exception:
        return str(ts)


class MissionPlanningLogTab(QWidget):
    """Simple viewer that tracks mission-planning pipeline runs and their logs."""

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        self._sessions: Dict[str, Dict[str, Any]] = {}
        self._items: Dict[str, QListWidgetItem] = {}
        self._current_session_id: Optional[str] = None

        header = QLabel("임무계획 파이프라인 실행 로그")
        header.setStyleSheet("font-weight:600; padding:4px 0;")

        self.session_list = QListWidget()
        self.session_list.currentItemChanged.connect(self._on_session_selected)
        self.session_list.setSelectionMode(QListWidget.SingleSelection)

        self.detail_view = QPlainTextEdit()
        self.detail_view.setReadOnly(True)
        self.detail_view.setLineWrapMode(QPlainTextEdit.NoWrap)

        splitter = QSplitter(Qt.Vertical)
        splitter.addWidget(self.session_list)
        splitter.addWidget(self.detail_view)
        splitter.setStretchFactor(1, 2)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.addWidget(header)
        layout.addWidget(splitter)

    # Session management -------------------------------------------------
    def start_session(self, session_id: str, meta: Dict[str, Any]) -> None:
        data = {
            "meta": dict(meta or {}),
            "lines": [],  # type: List[Dict[str, Any]]
            "status": "running",
            "summary": None,
        }
        self._sessions[session_id] = data
        label = self._format_item_label(session_id, data)
        item = QListWidgetItem(label)
        item.setData(Qt.UserRole, session_id)
        self.session_list.insertItem(0, item)
        self.session_list.setCurrentItem(item)
        self._items[session_id] = item

    def append_event(
        self,
        session_id: str,
        level: str,
        message: str,
        *,
        detail: Optional[Dict[str, Any]] = None,
        timestamp: Optional[float] = None,
    ) -> None:
        session = self._sessions.get(session_id)
        if not session:
            return
        session["lines"].append(
            {
                "level": level,
                "message": message,
                "detail": detail,
                "timestamp": timestamp,
            }
        )
        if self._current_session_id == session_id:
            self._render_detail(session_id)

    def finish_session(
        self,
        session_id: str,
        status: str,
        *,
        summary: Optional[Dict[str, Any]] = None,
    ) -> None:
        session = self._sessions.get(session_id)
        if not session:
            return
        session["status"] = status
        session["summary"] = summary
        item = self._items.get(session_id)
        if item:
            item.setText(self._format_item_label(session_id, session))
        if self._current_session_id == session_id:
            self._render_detail(session_id)

    # Internal helpers ---------------------------------------------------
    def _format_item_label(self, session_id: str, session: Dict[str, Any]) -> str:
        meta = session.get("meta", {})
        reason = meta.get("reason") or "-"
        plan_ids = meta.get("plan_ids") or []
        status = session.get("status", "running")
        ts = meta.get("timestamp")
        ts_text = _format_timestamp(ts)
        pid_text = ", ".join(str(pid) for pid in plan_ids) or "-"
        return f"[{ts_text}] {status.upper()}  planIds={pid_text}  reason={reason}"

    def _on_session_selected(
        self,
        current: Optional[QListWidgetItem],
        previous: Optional[QListWidgetItem],
    ) -> None:
        session_id = current.data(Qt.UserRole) if current else None
        self._current_session_id = session_id
        self._render_detail(session_id)

    def _render_detail(self, session_id: Optional[str]) -> None:
        if not session_id or session_id not in self._sessions:
            self.detail_view.setPlainText("")
            return
        session = self._sessions[session_id]
        meta = session.get("meta", {})
        summary = session.get("summary")
        lines = session.get("lines", [])

        parts: List[str] = []
        parts.append("=== Context ===")
        parts.append(json.dumps(meta, ensure_ascii=False, indent=2))
        parts.append("")
        parts.append(f"=== Status: {session.get('status', '-')} ===")
        if summary:
            parts.append(json.dumps(summary, ensure_ascii=False, indent=2))
            parts.append("")
        parts.append("=== Events ===")
        if not lines:
            parts.append("(no events)")
        else:
            for entry in lines:
                prefix = f"[{_format_timestamp(entry.get('timestamp'))}] {entry.get('level','INFO').upper()}"
                message = entry.get("message", "")
                parts.append(f"{prefix}  {message}")
                detail = entry.get("detail")
                if detail:
                    parts.append(json.dumps(detail, ensure_ascii=False, indent=2))
        self.detail_view.setPlainText("\n".join(parts))
