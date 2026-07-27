from __future__ import annotations

from pathlib import Path
from typing import Optional

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QCursor
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QApplication,
    QHeaderView,
    QTabWidget,
    QTableWidget,
)


def _project_root(start: Optional[Path] = None) -> Path:
    current = (start or Path(__file__).resolve()).resolve()
    for candidate in (current.parent, *current.parents):
        if (candidate / "run.py").exists():
            return candidate
    return current.parent


def load_shared_stylesheet(app: QApplication, project_root: Optional[Path] = None) -> Optional[Path]:
    root = _project_root(project_root)
    candidates = (
        root / "app" / "resources" / "style.qss",
        root / "resources" / "style.qss",
    )
    for path in candidates:
        if not path.exists():
            continue
        try:
            app.setStyleSheet(path.read_text(encoding="utf-8"))
            return path
        except Exception:
            continue
    return None


def position_window_from_env(app: QApplication, win, *, base_margin_x: int = 72, base_margin_y: int = 72) -> None:
    import os

    raw = (os.environ.get("KU_WINDOW_OFFSET") or "").strip()
    dx = 0
    dy = 0
    if raw:
        try:
            sx, sy = raw.split(",", 1)
            dx = int(sx.strip())
            dy = int(sy.strip())
        except Exception:
            dx = 0
            dy = 0

    try:
        cursor_pos = QCursor.pos()
    except Exception:
        cursor_pos = None

    screen = None
    if cursor_pos is not None:
        try:
            screen = app.screenAt(cursor_pos)
        except Exception:
            screen = None
    if screen is None:
        try:
            screen = app.primaryScreen()
        except Exception:
            screen = None
    if screen is None:
        return

    try:
        screen_geo = screen.availableGeometry()
    except Exception:
        try:
            screen_geo = screen.geometry()
        except Exception:
            return

    frame_geo = win.frameGeometry()
    frame_w = frame_geo.width() or win.width()
    frame_h = frame_geo.height() or win.height()

    target_x = screen_geo.x() + int(base_margin_x) + int(dx)
    target_y = screen_geo.y() + int(base_margin_y) + int(dy)

    if frame_w > 0:
        target_x = max(screen_geo.x(), min(target_x, screen_geo.x() + screen_geo.width() - frame_w))
    if frame_h > 0:
        target_y = max(screen_geo.y(), min(target_y, screen_geo.y() + screen_geo.height() - frame_h))

    try:
        win.move(target_x, target_y)
    except Exception:
        pass


def polish_tabs(tabs: QTabWidget) -> None:
    try:
        tabs.setDocumentMode(True)
    except Exception:
        pass
    try:
        tabs.setElideMode(Qt.ElideRight)
    except Exception:
        pass
    try:
        bar = tabs.tabBar()
        if bar is not None:
            bar.setExpanding(False)
            bar.setUsesScrollButtons(True)
        tabs.setMovable(False)
    except Exception:
        pass


def polish_message_table(table: QTableWidget) -> None:
    if table is None:
        return

    object_name = (table.objectName() or "").strip()
    if not object_name:
        object_name = "MessageTable"
        table.setObjectName(object_name)
    is_plain_grid = object_name == "PlainGrid"

    try:
        table.setShowGrid(False)
        table.setAlternatingRowColors(True)
        table.setWordWrap(False)
        table.setSelectionBehavior(QAbstractItemView.SelectRows)
        table.setSelectionMode(QAbstractItemView.NoSelection)
        table.setTextElideMode(Qt.ElideRight)
        table.setFocusPolicy(table.focusPolicy())
        table.setCornerButtonEnabled(False)
        table.verticalHeader().setVisible(False)
        table.verticalHeader().setDefaultSectionSize(34 if is_plain_grid else 40)
        table.verticalHeader().setMinimumSectionSize(30 if is_plain_grid else 34)
        header = table.horizontalHeader()
        header.setHighlightSections(False)
        header.setStretchLastSection(False)
        header.setDefaultAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        header.setMinimumSectionSize(74)
    except Exception:
        pass

    header = table.horizontalHeader()
    cols = int(table.columnCount())
    if cols == 5:
        _set_mode(header, 0, QHeaderView.Fixed, 90)
        _set_mode(header, 1, QHeaderView.Stretch)
        _set_mode(header, 2, QHeaderView.ResizeToContents)
        _set_mode(header, 3, QHeaderView.Fixed, 82)
        _set_mode(header, 4, QHeaderView.Fixed, 78)
    elif cols == 4:
        _set_mode(header, 0, QHeaderView.Fixed, 90)
        _set_mode(header, 1, QHeaderView.Stretch)
        _set_mode(header, 2, QHeaderView.ResizeToContents)
        _set_mode(header, 3, QHeaderView.Fixed, 78)
    elif cols == 3:
        _set_mode(header, 0, QHeaderView.Fixed, 90)
        _set_mode(header, 1, QHeaderView.Stretch)
        _set_mode(header, 2, QHeaderView.ResizeToContents)
    elif cols == 2:
        _set_mode(header, 0, QHeaderView.Fixed, 84 if is_plain_grid else 96)
        _set_mode(header, 1, QHeaderView.Stretch)


def _set_mode(header: QHeaderView, column: int, mode: QHeaderView.ResizeMode, width: Optional[int] = None) -> None:
    try:
        header.setSectionResizeMode(column, mode)
        if width is not None:
            header.resizeSection(column, width)
    except Exception:
        pass
