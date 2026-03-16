# -*- coding: utf-8 -*-
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QAbstractItemView,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSizePolicy,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
)

from modules.common.gui_style import polish_message_table

from .cards import Card


class ModuleWithLog(Card):
    """
    Compact dashboard module card.

    The legacy bottom log area was removed on purpose to keep the dashboard
    lighter and avoid retaining noisy UI text buffers.
    """

    def __init__(self, title: str, parent=None):
        super().__init__(title, parent, dense=False)

        self.rx_counts = {}
        self.tx_counts = {}
        self.rx_row = {}
        self.tx_row = {}
        self._last_log_text = ""

        root = QVBoxLayout()
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(12)

        self.mode_line = QLineEdit(self)
        self.mode_line.setObjectName("ModeTitleLine")
        self.mode_line.setPlaceholderText("Mode")
        self.mode_line.setMinimumHeight(36)
        self.mode_line.setAlignment(Qt.AlignCenter)
        root.addWidget(self.mode_line)

        row_ctrl = QHBoxLayout()
        row_ctrl.setContentsMargins(0, 0, 0, 0)
        row_ctrl.setSpacing(12)

        self.btn_run = QPushButton("GUI 실행", self)
        self.btn_run.setObjectName("PlainBtn")
        self.btn_run.setMinimumHeight(38)
        self.btn_run.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        row_ctrl.addWidget(self.btn_run, 1)

        row_ctrl.addStretch(1)

        self.auto_status = QLabel("Auto: OFF", self)
        self.auto_status.setObjectName("AutoStatus")
        self.auto_status.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.auto_status.setMinimumHeight(32)
        row_ctrl.addWidget(self.auto_status, 0, alignment=Qt.AlignRight)

        root.addLayout(row_ctrl)

        row_titles = QHBoxLayout()
        row_titles.setContentsMargins(0, 0, 0, 0)
        row_titles.setSpacing(16)

        lbl_rx = QLabel("수신 목록", self)
        lbl_tx = QLabel("발신 목록", self)
        lbl_rx.setObjectName("TableTitle")
        lbl_tx.setObjectName("TableTitle")
        row_titles.addWidget(lbl_rx, 1, alignment=Qt.AlignLeft)
        row_titles.addWidget(lbl_tx, 1, alignment=Qt.AlignLeft)
        root.addLayout(row_titles)

        row_tables = QHBoxLayout()
        row_tables.setContentsMargins(0, 0, 0, 0)
        row_tables.setSpacing(16)

        self.table_rx = self._make_table_2cols()
        self.table_tx = self._make_table_2cols()

        for table in (self.table_rx, self.table_tx):
            table.setMinimumHeight(220)
            table.setMaximumHeight(320)
            table.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        row_tables.addWidget(self.table_rx, 1)
        row_tables.addWidget(self.table_tx, 1)
        root.addLayout(row_tables)

        self.body_layout.addLayout(root, 1)

        self._auto_enabled = False
        self._update_auto_status(False)

        for code in ("0301", "0302", "0303", "0304", "0305"):
            self._ensure_row(self.table_rx, code, self.rx_row, self.rx_counts)
            self._ensure_row(self.table_tx, code, self.tx_row, self.tx_counts)

    def _make_table_2cols(self) -> QTableWidget:
        table = QTableWidget(0, 2, self)
        table.setObjectName("PlainGrid")
        table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        table.setSelectionMode(QAbstractItemView.NoSelection)
        table.setVerticalScrollMode(QAbstractItemView.ScrollPerPixel)
        table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        table.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        table.horizontalHeader().setVisible(False)
        table.verticalHeader().setVisible(False)
        polish_message_table(table)
        return table

    def _update_auto_status(self, enabled: bool):
        self._auto_enabled = bool(enabled)
        state_txt = "ON" if self._auto_enabled else "OFF"
        self.auto_status.setText(f"Auto: {state_txt}")
        self.auto_status.setProperty("active", self._auto_enabled)
        self.auto_status.style().unpolish(self.auto_status)
        self.auto_status.style().polish(self.auto_status)

    def set_auto_enabled(self, enabled: bool):
        self._update_auto_status(enabled)

    def _ensure_row(self, table: QTableWidget, code: str, row_map: dict, cnt_map: dict):
        if code in row_map:
            return
        row = table.rowCount()
        table.insertRow(row)
        item_code = QTableWidgetItem(code)
        item_cnt = QTableWidgetItem("0")
        item_code.setTextAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        item_cnt.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
        table.setItem(row, 0, item_code)
        table.setItem(row, 1, item_cnt)
        row_map[code] = row
        cnt_map[code] = 0

    def _set_count(self, table: QTableWidget, code: str, count: int, row_map: dict, cnt_map: dict):
        self._ensure_row(table, code, row_map, cnt_map)
        cnt_map[code] = int(count)
        row = row_map[code]
        table.item(row, 1).setText(str(cnt_map[code]))

    def bump_rx(self, code: str):
        self._set_count(self.table_rx, code, self.rx_counts.get(code, 0) + 1, self.rx_row, self.rx_counts)

    def bump_tx(self, code: str):
        self._set_count(self.table_tx, code, self.tx_counts.get(code, 0) + 1, self.tx_row, self.tx_counts)

    def set_rx_rows(self, rows):
        self.table_rx.setRowCount(0)
        self.rx_counts.clear()
        self.rx_row.clear()
        for code, cnt in rows:
            self._set_count(self.table_rx, str(code), int(cnt), self.rx_row, self.rx_counts)

    def set_tx_rows(self, rows):
        self.table_tx.setRowCount(0)
        self.tx_counts.clear()
        self.tx_row.clear()
        for code, cnt in rows:
            self._set_count(self.table_tx, str(code), int(cnt), self.tx_row, self.tx_counts)

    def set_log_max_lines(self, limit: int) -> None:
        _ = limit

    def append_log(self, text: str):
        self._last_log_text = str(text)

    def set_mode_text(self, text: str):
        try:
            self.mode_line.setText(str(text))
        except Exception:
            pass
