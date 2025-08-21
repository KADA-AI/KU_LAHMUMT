# -*- coding: utf-8 -*-
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QGridLayout, QPushButton, QLabel, QLineEdit, QFileDialog
)
from PyQt5.QtCore import Qt
from .zones import GRID_ROWS, GRID_COLS, ZONES
from ..widgets.cards import Card
from ..widgets.module_with_log import ModuleWithLog
from ..widgets.mode_buttons_panel import ModeButtonsPanel
        # 제목 없는 카드
from ..widgets.flow_visualizer import FlowVisualizer
from ..widgets.operation_flow_panel import OperationFlowPanel

class MainWindow(QMainWindow):
    """메인 화면: 35x50 가상 그리드에 구역 배치"""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("건국대 의사결정 지원 모듈 통합 관리 프로그램")
        self.resize(1800, 900)

        self._db_path_line: QLineEdit = None
        self._build_ui()

    # ---------- UI 조립 ----------
    def _build_ui(self):
        root = QWidget(self)
        grid = QGridLayout(root)
        # 가로 여백 축소(요청), 세로는 기존 느낌 유지
        grid.setContentsMargins(0, 8, 0, 0)    # 좌우 0으로 하단 검정바가 빈틈 없이 붙도록
        grid.setHorizontalSpacing(4)           # 가로 간격 축소
        grid.setVerticalSpacing(12)

        # 그리드 비율 (동일 가중치)
        for r in range(GRID_ROWS):
            grid.setRowStretch(r, 1)
        for c in range(GRID_COLS):
            grid.setColumnStretch(c, 1)

        # 1) 타이틀(카드 없이 큰/굵은 라벨)
        title_lbl = QLabel("건국대 의사결정 지원 모듈 통합 관리 SW", self)
        title_lbl.setObjectName("MainTitle")
        title_lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self._add_zone(grid, title_lbl, "TITLE")

        # 2) 경로 지정 버튼(= 찾아보기) — 카드 제거, 버튼만 배치
        btn_browse = QPushButton("찾아보기", self)
        btn_browse.setMinimumHeight(28)
        btn_browse.clicked.connect(self._browse_db)
        self._add_zone(grid, btn_browse, "ROUTE_BUTTON")

        # 3) DB 경로 텍스트 창(버튼 없음)
        self._db_path_line = QLineEdit(self)
        self._db_path_line.setObjectName("DbPathLine")
        self._db_path_line.setPlaceholderText("DB 폴더 경로")
        # 읽기 전용으로 유지(경로는 '찾아보기'로만 설정)
        self._db_path_line.setReadOnly(True)
        self._add_zone(grid, self._db_path_line, "DB_PATH")

        # 4~6 + 7~9 통합: 모듈+로그 결합 카드
        self.module_mission  = ModuleWithLog("임무 할당 및 계획")
        self._add_zone(grid, self.module_mission, "MODULE_MISSION_COMBO")

        self.module_monitor  = ModuleWithLog("모니터링 및 판단 모듈")
        self._add_zone(grid, self.module_monitor, "MODULE_MONITOR_COMBO")

        self.module_decision = ModuleWithLog("의사결정 지원 모듈")
        self._add_zone(grid, self.module_decision, "MODULE_DECISION_COMBO")

        # 10) 데이터 흐름 시각화 (제목 없음)
        self._add_zone(grid, FlowVisualizer(), "FLOW_VIS")

        # 11) 모드 버튼부 (제목 없음)
        self._add_zone(grid, ModeButtonsPanel(), "MODE_BUTTONS")

        # 12) 운용흐름 모니터링 (제목 없음)
        self._add_zone(grid, OperationFlowPanel(), "OPS_FLOW")

        # 13) 하단 검정 바(흰 글자) — 빈 곳 없이 전체 폭
        footer = QLabel("건국대 의사결정 지원 모듈 통합 관리 프로그램", self)
        footer.setObjectName("FooterFull")
        footer.setAlignment(Qt.AlignCenter)
        self._add_zone(grid, footer, "FOOTER")

        self.setCentralWidget(root)

    def _add_zone(self, grid: QGridLayout, w: QWidget, key: str):
        """ZONES의 (r0,c0,rs,cs)로 그리드 배치"""
        z = ZONES[key]
        grid.addWidget(w, z["r0"], z["c0"], z["rs"], z["cs"])

    # ---------- 동작 ----------
    def _browse_db(self):
        path = QFileDialog.getExistingDirectory(self, "DB 폴더 선택")
        if path:
            self._db_path_line.setText(path)
            # 필요 시: 모듈 로그에 기록
            self.module_mission.append_log(f"[PATH] {path}")
            self.module_monitor.append_log(f"[PATH] {path}")
            self.module_decision.append_log(f"[PATH] {path}")
