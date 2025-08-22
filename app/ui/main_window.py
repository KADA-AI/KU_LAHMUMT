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
from pathlib import Path

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

        self._bind_module_buttons()
        self._init_msg_monitor()

    def _launch_gui(self, script_name: str):
        """
        modules/decision_support 아래의 단일 GUI 스크립트를
        프로젝트 루트 기준 절대경로로 찾아 실행한다.
        """
        import sys

        # main_window.py 위치: <root>/app/ui/main_window.py
        root = Path(__file__).resolve().parents[2]   # 프로젝트 루트
        ds_dir = root / "modules" / "decision_support"
        script = ds_dir / script_name

        if not script.exists():
            # 공용 로그 박스가 있으면 의사결정 카드 로그에 남김
            try:
                self.module_decision.append_log(f"[RUN ERR] not found: {script}")
            except Exception:
                pass
            return

        try:
            subprocess.Popen(
                [sys.executable, str(script)],
                cwd=str(root),                         # 항상 루트에서 실행
                shell=False,
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
            )
        except Exception as e:
            try:
                self.module_decision.append_log(f"[RUN ERR] {e}")
            except Exception:
                pass

    def _bind_module_buttons(self):
        for btn in (getattr(self.module_decision, "btn_run", None),
                    getattr(self.module_mission,  "btn_run", None),
                    getattr(self.module_monitor,  "btn_run", None)):
            try: btn.clicked.disconnect()
            except Exception: pass

        self.module_decision.btn_run.clicked.connect(lambda: self._launch_role("decision"))
        self.module_mission.btn_run.clicked.connect( lambda: self._launch_role("mission"))
        self.module_monitor.btn_run.clicked.connect( lambda: self._launch_role("monitor"))


    def mark_received(self, msg_id: str, raw: bytes | None = None):
        mid = str(msg_id)

        def handle(module_key: str, kind: str):
            # kind: "tx" → out, "rx" → in
            mod = {"mission": self.module_mission,
                "monitor": self.module_monitor,
                "decision": self.module_decision}[module_key]
            if kind == "tx":
                if hasattr(mod, "bump_tx"): mod.bump_tx(mid)
                if hasattr(self, "flow"):   self.flow.trigger(module_key, "out")
                if hasattr(mod, "append_log"): mod.append_log(f"[{mid}] PUSH 완료")
            else:
                if hasattr(mod, "bump_rx"): mod.bump_rx(mid)
                if hasattr(self, "flow"):   self.flow.trigger(module_key, "in")
                if hasattr(mod, "append_log"): mod.append_log(f"[{mid}] RX 수신")

        maps = getattr(self, "_msg_maps", {})
        for key in ("mission", "monitor", "decision"):
            m = maps.get(key, {})
            if mid in m.get("tx", set()):
                handle(key, "tx")
            if mid in m.get("rx", set()):
                handle(key, "rx")

    def _init_msg_monitor(self):
        from importlib import import_module
        from receive_center import register_listener  # GUI 스레드 안전 큐잉으로 호출됨

        # 탭 정의에서 (msg_id 리스트) 가져오기
        mods = {
            "mission":  ("Tabs.assignment_planning_tab", "AssignmentPlanningTab"),
            "monitor":  ("Tabs.mission_monitoring_tab", "MissionMonitoringTab"),
            "decision": ("Tabs.decision_support_tab",   "DecisionSupportTab"),
        }

        self._msg_maps = {}
        all_ids = set()

        for key, (mod_name, cls_name) in mods.items():
            mod = import_module(mod_name)
            cls = getattr(mod, cls_name)
            tx = set(mid for mid, _ in getattr(cls, "PUSH_MESSAGES", []))
            rx = set(mid for mid, _ in getattr(cls, "RECEIVE_MESSAGES", []))
            self._msg_maps[key] = {"tx": tx, "rx": rx}
            all_ids |= tx | rx

        # 모든 msg_id를 메인 윈도우(self) 리스너로 등록
        for mid in sorted(all_ids):
            register_listener(mid, self)

    def _launch_role(self, role: str):
        import sys, subprocess
        from pathlib import Path

        root = Path(__file__).resolve().parents[2]

        if role == "decision":
            candidates = [
                root / "modules" / "decision_support" / "decision_support_gui.py",
                root / "app"     / "modules" / "decision_support" / "decision_support_gui.py",
            ]
            target_log = self.module_decision

        elif role == "mission":
            candidates = [
                root / "modules" / "mission_planning" / "mission_planning_gui.py",
                root / "app"     / "modules" / "mission_planning" / "mission_planning_gui.py",
                # fallback (옛 파일명)
                root / "modules" / "decision_support" / "assignment_planning_gui.py",
                root / "app"     / "modules" / "decision_support" / "assignment_planning_gui.py",
            ]
            target_log = self.module_mission

        elif role == "monitor":
            candidates = [
                root / "modules" / "monitoring" / "monitoring_gui.py",
                root / "app"     / "modules" / "monitoring" / "monitoring_gui.py",
                # fallback (DS 폴더에 둘 경우)
                root / "modules" / "decision_support" / "monitoring_gui.py",
                root / "modules" / "decision_support" / "monitoritng_gui.py",
                root / "app"     / "modules" / "decision_support" / "monitoring_gui.py",
                root / "app"     / "modules" / "decision_support" / "monitoritng_gui.py",
            ]
            target_log = self.module_monitor

        else:
            return

        script = next((p for p in candidates if p.exists()), None)
        if not script:
            try:
                target_log.append_log("[RUN ERR] not found:\n" + "\n".join(str(p) for p in candidates))
            except Exception:
                pass
            return

        try:
            subprocess.Popen([sys.executable, str(script)], cwd=str(root),
                            shell=False,
                            creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        except Exception as e:
            try:
                target_log.append_log(f"[RUN ERR] {e}")
            except Exception:
                pass


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
