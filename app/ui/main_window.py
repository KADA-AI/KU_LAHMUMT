# -*- coding: utf-8 -*-
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QGridLayout, QPushButton, QLabel, QLineEdit, QFileDialog, QShortcut
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QKeySequence
from .zones import GRID_ROWS, GRID_COLS, ZONES
from ..widgets.cards import Card
from ..widgets.module_with_log import ModuleWithLog
from ..widgets.mode_buttons_panel import ModeButtonsPanel
        # 제목 없는 카드
from ..widgets.flow_visualizer import FlowVisualizer
from ..widgets.operation_flow_panel import OperationFlowPanel
import os, subprocess

class MainWindow(QMainWindow):
    """메인 화면: 35x50 가상 그리드에 구역 배치"""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("건국대 의사결정 지원 모듈 통합 관리 프로그램")
        self.resize(1800, 900)

        self._db_path_line: QLineEdit = None
        self._build_ui()

    def _build_ui(self):
        root = QWidget(self)
        grid = QGridLayout(root)
        grid.setContentsMargins(0, 8, 0, 0)
        grid.setHorizontalSpacing(4)
        grid.setVerticalSpacing(12)

        for r in range(GRID_ROWS):
            grid.setRowStretch(r, 1)
        for c in range(GRID_COLS):
            grid.setColumnStretch(c, 1)

        # 타이틀
        title_lbl = QLabel("건국대 의사결정 지원 모듈 통합 관리 SW", self)
        title_lbl.setObjectName("MainTitle")
        title_lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self._add_zone(grid, title_lbl, "TITLE")

        # 찾아보기 버튼
        btn_browse = QPushButton("찾아보기")
        btn_browse.setMinimumHeight(28)
        btn_browse.clicked.connect(self._browse_db)
        self._add_zone(grid, btn_browse, "ROUTE_BUTTON")

        # DB 경로(읽기전용)
        self._db_path_line = QLineEdit(self)
        self._db_path_line.setObjectName("DbPathLine")
        self._db_path_line.setPlaceholderText("DB 폴더 경로")
        self._db_path_line.setReadOnly(True)
        self._add_zone(grid, self._db_path_line, "DB_PATH")

        # 모듈 카드들
        self.module_mission  = ModuleWithLog("임무 할당 및 계획")
        self._add_zone(grid, self.module_mission, "MODULE_MISSION_COMBO")
        self.module_monitor  = ModuleWithLog("모니터링 및 판단 모듈")
        self._add_zone(grid, self.module_monitor, "MODULE_MONITOR_COMBO")
        self.module_decision = ModuleWithLog("의사결정 지원 모듈")
        self._add_zone(grid, self.module_decision, "MODULE_DECISION_COMBO")

        # 데이터 흐름 다이어그램(외부 카드 없이)
        self.flow = FlowVisualizer()          # ← 참조 보관
        self._add_zone(grid, self.flow, "FLOW_VIS")

        # 좌측 모드 버튼
        self._add_zone(grid, ModeButtonsPanel(), "MODE_BUTTONS")

        # 운용 흐름
        self._add_zone(grid, OperationFlowPanel(), "OPS_FLOW")

        # 하단 검정 바
        footer = QLabel("건국대 의사결정 지원 모듈 통합 관리 프로그램", self)
        footer.setObjectName("FooterFull")
        footer.setAlignment(Qt.AlignCenter)
        self._add_zone(grid, footer, "FOOTER")

        self.setCentralWidget(root)

        # ✅ 테스트 단축키/데모 설치
        self._install_flow_test_shortcuts()

        # 의사결정 지원 GUI
        self.module_decision.btn_run.clicked.connect(
            lambda: self._launch_gui(
                r"C:\Users\LAHMUMT_2\anaconda3\envs\LAHMUMT\python.exe",
                r"C:\Users\LAHMUMT_2\Desktop\KU_LAHMUMT\app\modules\decision_support\decision_support_gui.py"
            )
        )

        # 임무 할당·계획수립 GUI (AssignmentPlanningTab 전용)
        self.module_mission.btn_run.clicked.connect(
            lambda: self._launch_gui(
                r"C:\Users\LAHMUMT_2\anaconda3\envs\LAHMUMT\python.exe",
                r"C:\Users\LAHMUMT_2\Desktop\KU_LAHMUMT\app\modules\mission_planning\mission_planning_gui.py"
            )
        )

        # 임무 모니터링·판단 GUI
        self.module_monitor.btn_run.clicked.connect(
            lambda: self._launch_gui(
                r"C:\Users\LAHMUMT_2\anaconda3\envs\LAHMUMT\python.exe",
                r"C:\Users\LAHMUMT_2\Desktop\KU_LAHMUMT\app\modules\monitoring\monitoring_gui.py"
            )
        )

    def _launch_gui(self, py_exe: str, script_path: str):
        try:
            subprocess.Popen(
                [py_exe, script_path],
                cwd=os.path.dirname(script_path),
                shell=False,
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0)  # 콘솔 숨김(Windows)
            )
        except Exception as e:
            # 필요 시 모듈 로그에 기록하고 싶으면 주석 해제
            # self.module_decision.append_log(f"[RUN ERR] {e}")
            print(e)

    def _install_flow_test_shortcuts(self):
        """데이터 흐름 애니메이션 테스트용 단축키 설치"""
        # 1/2: 모니터링 in/out
        QShortcut(QKeySequence("1"), self, activated=lambda: self._pulse("monitor", "in"))
        QShortcut(QKeySequence("2"), self, activated=lambda: self._pulse("monitor", "out"))
        # 3/4: 임무 할당 in/out
        QShortcut(QKeySequence("3"), self, activated=lambda: self._pulse("mission", "in"))
        QShortcut(QKeySequence("4"), self, activated=lambda: self._pulse("mission", "out"))
        # 5/6: 의사결정 in/out
        QShortcut(QKeySequence("5"), self, activated=lambda: self._pulse("decision", "in"))
        QShortcut(QKeySequence("6"), self, activated=lambda: self._pulse("decision", "out"))

        # D: 데모 토글
        QShortcut(QKeySequence("D"), self, activated=self._toggle_demo_flow)

        # 데모 타이머 준비
        self._demo_timer = QTimer(self)
        self._demo_timer.setInterval(100)  # 0.6s 간격으로 다음 이벤트
        self._demo_timer.timeout.connect(self._demo_step)
        self._demo_seq = [
            ("monitor", "in"), ("monitor", "out"),
            ("mission", "in"), ("mission", "out"),
            ("decision", "in"), ("decision", "out"),
        ]
        self._demo_idx = 0

    def _pulse(self, module: str, direction: str):
        """단축키에서 호출되는 단발 트리거"""
        if hasattr(self, "flow") and self.flow:
            self.flow.trigger(module, direction)

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

    def _toggle_demo_flow(self):
        """D 키로 데모 on/off"""
        if self._demo_timer.isActive():
            self._demo_timer.stop()
            # 로그에 남기고 싶으면 주석 해제
            # self.module_monitor.append_log("[DEMO] stop")
        else:
            self._demo_idx = 0
            self._demo_timer.start()
            # self.module_monitor.append_log("[DEMO] start")

    def _demo_step(self):
        """데모 시퀀스 한 스텝"""
        mod, direc = self._demo_seq[self._demo_idx]
        self._pulse(mod, direc)
        self._demo_idx = (self._demo_idx + 1) % len(self._demo_seq)
