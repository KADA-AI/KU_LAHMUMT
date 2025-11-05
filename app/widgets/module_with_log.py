# -*- coding: utf-8 -*-
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QTextEdit,
    QLineEdit, QPushButton, QTableWidget, QTableWidgetItem, QSizePolicy,
    QAbstractItemView, QHeaderView
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QTextCursor
from .cards import Card

_DEFAULT_MAX_LOG_LINES = 10

class ModuleWithLog(Card):
    """
    모듈 컨테이너(위: 모듈 UI, 아래: 각진 검정 로그 박스).
    구성:
      1) 상단: QLineEdit("Mode 모니터링 txt")
      2) 그 아래: [GUI 실행] 버튼 (좌), [Auto 토글 + 상태점] (우)
      3) 중간: "수신 목록" / "송신 목록" 라벨 + 2열 표(코드, 카운트) — 내부 스크롤
      4) 하단: 검정 로그 박스(#LogBox, 각진)
    외부 API:
      - bump_rx(code:str), bump_tx(code:str) → 해당 코드의 카운트를 +1 (표 즉시 갱신)
      - set_rx_rows([(code,count), ...]), set_tx_rows([...]) → 전체 교체
    """
    def __init__(self, title: str, parent=None):
        super().__init__(title, parent, dense=False)

        # 누적 카운트 저장소
        self.rx_counts = {}   # code -> int
        self.tx_counts = {}
        self.rx_row = {}      # code -> row index
        self.tx_row = {}

        root = QVBoxLayout()
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(12)

        # (1) 모드 텍스트
        self.mode_line = QLineEdit(self)
        self.mode_line.setObjectName("ModeTitleLine")
        self.mode_line.setPlaceholderText("Mode 모니터링 txt")
        self.mode_line.setMinimumHeight(32)
        self.mode_line.setAlignment(Qt.AlignCenter)
        root.addWidget(self.mode_line)

        # (2) control row: run button + auto indicator
        row_ctrl = QHBoxLayout()
        row_ctrl.setContentsMargins(0, 0, 0, 0)
        row_ctrl.setSpacing(16)

        self.btn_run = QPushButton("GUI 실행", self)
        self.btn_run.setObjectName("PlainBtn")
        self.btn_run.setMinimumHeight(36)
        self.btn_run.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        row_ctrl.addWidget(self.btn_run, 1)

        row_ctrl.addStretch(1)

        self.auto_status = QLabel("Auto: OFF", self)
        self.auto_status.setObjectName("AutoStatus")
        self.auto_status.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        self.auto_status.setMinimumHeight(32)
        row_ctrl.addWidget(self.auto_status, 0, alignment=Qt.AlignRight)

        root.addLayout(row_ctrl)

        # (3) 수신/송신 표 타이틀
        row_titles = QHBoxLayout()
        row_titles.setContentsMargins(0, 0, 0, 0)
        row_titles.setSpacing(24)
        lbl_rx = QLabel("수신 목록", self)
        lbl_tx = QLabel("발신 목록", self)
        lbl_rx.setObjectName("TableTitle")
        lbl_tx.setObjectName("TableTitle")
        row_titles.addWidget(lbl_rx, 1, alignment=Qt.AlignLeft)
        row_titles.addWidget(lbl_tx, 1, alignment=Qt.AlignLeft)
        root.addLayout(row_titles)

        # (3-1) 2열 표(코드, 카운트) — 내부 스크롤
        row_tables = QHBoxLayout()
        row_tables.setContentsMargins(0, 0, 0, 0)
        row_tables.setSpacing(24)

        self.table_rx = self._make_table_2cols()
        self.table_tx = self._make_table_2cols()

        # 표 높이를 지정해 내부 스크롤이 발생하도록 고정
        for t in (self.table_rx, self.table_tx):
            t.setMinimumHeight(180)
            t.setMaximumHeight(260)
            t.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        row_tables.addWidget(self.table_rx, 1)
        row_tables.addWidget(self.table_tx, 1)
        root.addLayout(row_tables)

        # (4) 로그 박스(각진 검정)
        self.log = QTextEdit(self)
        self.log.setObjectName("LogBox")
        self.log.setReadOnly(True)
        self.log.setMinimumHeight(140)
        self._log_max_blocks = _DEFAULT_MAX_LOG_LINES
        self._configure_log_limits()
        root.addWidget(self.log, 1)

        self.body_layout.addLayout(root, 1)

        # Auto 상태 표시 초기화
        self._auto_enabled = False
        self._update_auto_status(False)

        # 예시 데이터(표 형태가 확실히 보이도록 0301~0305 사전 등록)
        for code in ["0301", "0302", "0303", "0304", "0305"]:
            self._ensure_row(self.table_rx, code, self.rx_row, self.rx_counts)
            self._ensure_row(self.table_tx, code, self.tx_row, self.tx_counts)

    # ----------------- helpers -----------------
    def _make_table_2cols(self) -> QTableWidget:
        """
        헤더 숨김/2열(코드, 카운트)/그리드 표시/내부 스크롤 활성 표 생성
        """
        tbl = QTableWidget(0, 2, self)
        tbl.setObjectName("PlainGrid")
        tbl.setShowGrid(True)
        tbl.setAlternatingRowColors(False)
        tbl.setEditTriggers(QAbstractItemView.NoEditTriggers)
        tbl.setSelectionMode(QAbstractItemView.NoSelection)
        tbl.setVerticalScrollMode(QAbstractItemView.ScrollPerPixel)
        tbl.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        tbl.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        tbl.horizontalHeader().setVisible(False)
        tbl.verticalHeader().setVisible(False)
        tbl.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)       # 코드
        tbl.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)  # 카운트
        return tbl

    def _update_auto_status(self, enabled: bool):
        self._auto_enabled = bool(enabled)
        if hasattr(self, 'auto_status') and self.auto_status is not None:
            state_txt = 'ON' if self._auto_enabled else 'OFF'
            self.auto_status.setText(f"Auto: {state_txt}")
            self.auto_status.setProperty('active', self._auto_enabled)
            self.auto_status.style().unpolish(self.auto_status)
            self.auto_status.style().polish(self.auto_status)

    def set_auto_enabled(self, enabled: bool):
        """Update auto indicator text/state."""
        self._update_auto_status(enabled)

    def _ensure_row(self, tbl: QTableWidget, code: str, row_map: dict, cnt_map: dict):
        """코드용 행이 없으면 생성(카운트 0으로)"""
        if code in row_map:
            return
        r = tbl.rowCount()
        tbl.insertRow(r)
        item_code = QTableWidgetItem(code)
        item_cnt  = QTableWidgetItem("0")
        # 정렬
        item_code.setTextAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        item_cnt.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
        tbl.setItem(r, 0, item_code)
        tbl.setItem(r, 1, item_cnt)
        row_map[code] = r
        cnt_map[code] = 0

    def _set_count(self, tbl: QTableWidget, code: str, count: int, row_map: dict, cnt_map: dict):
        """카운트 값을 갱신하고 표 셀 반영"""
        self._ensure_row(tbl, code, row_map, cnt_map)
        cnt_map[code] = int(count)
        r = row_map[code]
        tbl.item(r, 1).setText(str(cnt_map[code]))

    # -------- 외부 연동용 API --------
    def bump_rx(self, code: str):
        """수신 목록에서 해당 코드의 카운트를 +1"""
        cur = self.rx_counts.get(code, 0) + 1
        self._set_count(self.table_rx, code, cur, self.rx_row, self.rx_counts)

    def bump_tx(self, code: str):
        """송신 목록에서 해당 코드의 카운트를 +1"""
        cur = self.tx_counts.get(code, 0) + 1
        self._set_count(self.table_tx, code, cur, self.tx_row, self.tx_counts)

    def set_rx_rows(self, rows):
        """수신 목록 전체 교체: rows=[(code, count), ...]"""
        self.table_rx.setRowCount(0)
        self.rx_counts.clear()
        self.rx_row.clear()
        for code, cnt in rows:
            self._set_count(self.table_rx, str(code), int(cnt), self.rx_row, self.rx_counts)

    def set_tx_rows(self, rows):
        """송신 목록 전체 교체"""
        self.table_tx.setRowCount(0)
        self.tx_counts.clear()
        self.tx_row.clear()
        for code, cnt in rows:
            self._set_count(self.table_tx, str(code), int(cnt), self.tx_row, self.tx_counts)

    def set_log_max_lines(self, limit: int) -> None:
        """외부에서 로그 최대 라인 수를 조정할 때 사용."""
        try:
            limit_val = max(0, int(limit))
        except (TypeError, ValueError):
            limit_val = _DEFAULT_MAX_LOG_LINES
        self._log_max_blocks = limit_val
        self._configure_log_limits()

    def append_log(self, text: str):
        """
        같은 텍스트가 짧은 시간(0.8s) 안에 연속으로 들어오면 중복 로깅을 막는다.
        (대시보드에서 동일 문구 브로드캐스트가 겹쳐도 카드에는 1회만 표시)
        """
        try:
            import time
            now = time.monotonic()
            last_txt = getattr(self, "_last_log_text", None)
            last_ts  = getattr(self, "_last_log_ts", 0.0)
            if text == last_txt and (now - last_ts) < 0.8:
                return
            self._last_log_text = text
            self._last_log_ts = now
        except Exception:
            pass

        self.log.append(text)
        self._truncate_log_history()

    def _configure_log_limits(self) -> None:
        """Apply max block count to the log widget and trim existing entries."""
        if self.log is None:
            return
        try:
            doc = self.log.document()
            doc.setMaximumBlockCount(self._log_max_blocks)
        except Exception:
            pass
        self._truncate_log_history()

    def _truncate_log_history(self) -> None:
        """Drop old log blocks beyond the configured maximum."""
        if self.log is None:
            return
        if self._log_max_blocks <= 0:
            return
        try:
            doc = self.log.document()
            while doc.blockCount() > self._log_max_blocks:
                cursor = QTextCursor(doc)
                cursor.movePosition(QTextCursor.Start)
                cursor.select(QTextCursor.BlockUnderCursor)
                cursor.removeSelectedText()
                cursor.deleteChar()
        except Exception:
            pass

    def set_mode_text(self, text: str):
        """
        대시보드에서 'Mode 모니터링 txt'에 표시할 텍스트를 설정한다.
        """
        try:
            self.mode_line.setText(str(text))
        except Exception:
            pass
