# /mnt/data/main_window.py
# -*- coding: utf-8 -*-
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QGridLayout, QPushButton, QLabel, QLineEdit, QFileDialog, QShortcut,
    QHBoxLayout, QVBoxLayout, QSizePolicy, QMessageBox, QPlainTextEdit
)
from PyQt5.QtCore import Qt, QTimer, QUrl
from PyQt5.QtGui import QKeySequence, QDesktopServices
from typing import Optional
from .zones import GRID_ROWS, GRID_COLS, ZONES
from ..widgets.cards import Card
from ..widgets.module_with_log import ModuleWithLog
from ..widgets.operation_flow_panel import OperationFlowPanel
import os, sys, subprocess, json, socket, shutil
from pathlib import Path
from modules.common import db_paths

APP_TITLE = "KU Mission Decision Support Dashboard (v1.2.0)"
REFERENCE_PDF_PATH = db_paths.PROJECT_ROOT / "ref" / "04. 모듈 간 인터페이스 설계-v7-20260116_175548.pdf"
if not REFERENCE_PDF_PATH.exists():
    REFERENCE_PDF_PATH = db_paths.PROJECT_ROOT / "ref" / "04. 모듈 간 인터페이스 설계-v7-20250917_133206.pdf"

class MainWindow(QMainWindow):
    """Main dashboard window arranged on a 35x50 grid."""
    def __init__(self):
        super().__init__()
        self.setWindowTitle(APP_TITLE)
        self.resize(980, 700)

        self._db_path_line: QLineEdit = None
        self._scenario_root_line: QLineEdit = None
        self._current_db_root: str = ""
        self._scenario_status_dot: Optional[QLabel] = None
        self._scenario_status_label: Optional[QLabel] = None
        self._version_notes: Optional[QPlainTextEdit] = None
        self._version_notes_path: Path = db_paths.PROJECT_ROOT / "change_log.md"

        # Middleware widget references
        self._mw_name: QLineEdit = None
        self._mw_addr: QLineEdit = None
        self._mw_local: QLineEdit = None
        self._mw_external: QLineEdit = None

        self.module_mission = None
        self.module_monitor = None
        self.module_decision = None
        self.flow = None

        self.btn_auto_boot = None
        self.btn_module_shutdown = None
        self.btn_integration_module = None
        self.btn_simulation_run = None
        self.btn_overwrite_020x = None
        self.btn_reference_pdf = None
        self.btn_decision_reset = None
        self._role_processes = {}

        self._build_ui()

    def _build_ui(self):
        root = QWidget(self)
        grid = QGridLayout(root)
        grid.setContentsMargins(20, 16, 20, 16)
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(12)

        for r in range(GRID_ROWS):
            grid.setRowStretch(r, 1)
        for c in range(GRID_COLS):
            grid.setColumnStretch(c, 1)
        footer_zone = ZONES.get("FOOTER")
        if footer_zone:
            grid.setRowStretch(footer_zone["r0"], 2)

        # Title label
        title_wrap = QWidget(self)
        title_layout = QVBoxLayout(title_wrap)
        title_layout.setContentsMargins(0, 0, 0, 0)
        title_layout.setSpacing(2)
        title_lbl = QLabel(APP_TITLE, title_wrap)
        title_lbl.setObjectName("MainTitle")
        title_lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        title_layout.addWidget(title_lbl)
        subtitle_lbl = QLabel("최근 업데이트 날짜 : 26-03-10", title_wrap)
        subtitle_lbl.setObjectName("MainSubtitle")
        subtitle_lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        title_layout.addWidget(subtitle_lbl)
        self._add_zone(grid, title_wrap, "TITLE")

        btn_browse = QPushButton("DB 폴더 선택")
        btn_browse.setObjectName("SecondaryButton")
        btn_browse.setMinimumHeight(28)
        btn_browse.setFixedWidth(140)
        btn_browse.clicked.connect(self._browse_db)
        btn_wrap = QWidget(self)
        btn_layout = QHBoxLayout(btn_wrap)
        btn_layout.setContentsMargins(0, 0, 0, 0)
        btn_layout.addStretch(1)
        btn_layout.addWidget(btn_browse, 0, Qt.AlignRight | Qt.AlignVCenter)
        self._add_zone(grid, btn_wrap, "ROUTE_BUTTON")

        # Database path entry
        self._db_path_line = QLineEdit(self)
        self._db_path_line.setObjectName("DbPathLine")
        self._db_path_line.setPlaceholderText("Database directory")
        self._db_path_line.setReadOnly(True)
        self._scenario_root_line = QLineEdit(self)
        self._scenario_root_line.setObjectName("ScenarioRootLine")
        self._scenario_root_line.setPlaceholderText("Scenario base (optional)")
        self._scenario_root_line.setReadOnly(True)
        self._scenario_root_line.setVisible(False)

        # Apply default path via shared db path manager
        info = db_paths.get_info()
        default_db_path = info.get("db_root") or db_paths.get_active_db_root_str()
        self._current_db_root = str(default_db_path)
        self._db_path_line.setText(self._current_db_root)
        self.update_scenario_root(info.get("base_root"))

        path_container = QWidget(self)
        path_container.setObjectName("Card")
        path_container.setAttribute(Qt.WA_StyledBackground, True)
        path_layout = QVBoxLayout(path_container)
        path_layout.setContentsMargins(12, 10, 12, 10)
        path_layout.setSpacing(6)
        indicator_row = QHBoxLayout()
        indicator_row.setContentsMargins(0, 0, 0, 0)
        indicator_row.setSpacing(6)
        self._scenario_status_dot = QLabel("●", self)
        self._scenario_status_dot.setObjectName("ScenarioDot")
        self._scenario_status_dot.setProperty("active", False)
        self._scenario_status_dot.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        indicator_row.addWidget(self._scenario_status_dot, 0, Qt.AlignLeft)
        self._scenario_status_label = QLabel("신규 DB 설정 여부 (유지)", self)
        self._scenario_status_label.setObjectName("ScenarioStatusLabel")
        self._scenario_status_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        indicator_row.addWidget(self._scenario_status_label, 0, Qt.AlignLeft)
        indicator_row.addStretch(1)
        path_layout.addLayout(indicator_row)
        path_layout.addWidget(self._db_path_line)
        self._add_zone(grid, path_container, "DB_PATH")
        self.update_scenario_status_indicator(False, "대기 모드에서 새로운 폴더가 생성되면 초록색으로 바뀝니다.")

        # Middleware configuration row
        mw_row = self._make_middleware_row()
        self._add_zone(grid, mw_row, "MIDDLEWARE")

        # Remove left-hand controls but keep layout slots
        self._add_left_placeholder(grid)

        notes_card = self._build_version_notes()
        self._add_zone(grid, notes_card, "FOOTER")

        self.operation_panel = None

        self.setCentralWidget(root)

        # Install flow test shortcuts
        self._install_flow_test_shortcuts()

        self._bind_module_buttons()
        self._init_msg_monitor()
        self._apply_middleware()

    def _load_version_notes_text(self) -> str:
        path = self._version_notes_path
        try:
            return path.read_text(encoding="utf-8")
        except FileNotFoundError:
            return f"[change log missing] {path}"
        except Exception as exc:
            return f"[change log read failed] {path}: {exc}"

    def _build_version_notes(self) -> QWidget:
        card = Card("Change Log", self, dense=True)
        body = getattr(card, "body_layout", None)
        self._version_notes = QPlainTextEdit(self)
        self._version_notes.setObjectName("VersionNotes")
        self._version_notes.setReadOnly(True)
        self._version_notes.setMinimumHeight(120)
        self._version_notes.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._version_notes.setPlainText(self._load_version_notes_text())
        if body is not None:
            body.addWidget(self._version_notes)
        return card

    # ---------- Middleware helpers ----------
    def _make_middleware_row(self) -> QWidget:
        """Build the inline middleware configuration row."""
        w = QWidget(self)
        w.setObjectName("Card")
        w.setAttribute(Qt.WA_StyledBackground, True)
        lay = QHBoxLayout(w)
        lay.setContentsMargins(12, 10, 12, 10)
        lay.setSpacing(10)

        lbl = QLabel("Middleware", self)
        lbl.setStyleSheet("font-weight:600;")
        lay.addWidget(lbl)

        def _mk_line(ph: str, width: int = 120, default: str = "") -> QLineEdit:
            le = QLineEdit(self)
            le.setPlaceholderText(ph)
            if default:
                le.setText(default)
            le.setMinimumWidth(width)
            return le

        self._mw_name     = _mk_line("Name", 120, "AVS1")
        self._mw_addr     = _mk_line("NetworkAddress (e.g. 203.)", 140, "192.")
        self._mw_local    = _mk_line("LocalDomain", 100, "10")
        self._mw_external = _mk_line("ExternalDomain", 110, "100")

        lay.addWidget(QLabel("Name:", self));           lay.addWidget(self._mw_name)
        lay.addWidget(QLabel("Network:", self));        lay.addWidget(self._mw_addr)
        lay.addWidget(QLabel("Local:", self));          lay.addWidget(self._mw_local)
        lay.addWidget(QLabel("External:", self));       lay.addWidget(self._mw_external)

        btn_apply = QPushButton("Apply", self)
        btn_apply.setObjectName("SecondaryButton")
        btn_apply.setMinimumWidth(80)
        btn_apply.clicked.connect(self._apply_middleware)
        lay.addWidget(btn_apply, 0, Qt.AlignRight)

        self._load_middleware_config()

        return w


    def _apply_middleware(self) -> None:
        """Persist middleware settings to every nFusionSettings.json in the project."""
        name = (self._mw_name.text() or "").strip() or "AVS1"
        net = (self._mw_addr.text() or "").strip() or "192."
        if not net.endswith('.'):
            net += '.'
        try:
            local = int((self._mw_local.text() or "10").strip())
        except Exception:
            local = 10
        try:
            ext = int((self._mw_external.text() or "100").strip())
        except Exception:
            ext = 100

        cfg = {
            "Middleware": {
                "Name": name,
                "NetworkAddress": net,
                "LocalDomain": local,
                "ExternalDomain": ext,
            }
        }

        proj_root = self._find_project_root()
        cfg_json = json.dumps(cfg, ensure_ascii=False, separators=(",", ":"))

        updated_paths = []
        errors = []
        for cfg_path in proj_root.rglob('nFusionSettings.json'):
            try:
                cfg_path.write_text(cfg_json, encoding='utf-8')
                updated_paths.append(cfg_path)
            except Exception as exc:
                errors.append((cfg_path, exc))

        ip_prefix = net.split('.', 1)[0] if '.' in net else net
        msg = (f"[CFG] nFusionSettings updated ({len(updated_paths)} files) | "
               f"Name={name}, Net={net}, Prefix={ip_prefix}, Local={local}, External={ext}")
        if errors:
            err_txt = '; '.join(f"{p.name}:{e}" for p, e in errors[:3])
            msg = f"[CFG WARN] middleware update partial: {err_txt}"

        for mod in (getattr(self, 'module_mission', None),
                    getattr(self, 'module_monitor', None),
                    getattr(self, 'module_decision', None)):
            try:
                mod.append_log(msg)
            except Exception:
                pass

        self._last_middleware_prefix = ip_prefix

    def _load_middleware_config(self) -> None:
        """Load existing middleware configuration if available."""
        proj_root = self._find_project_root()
        cfg_path = proj_root / "nFusionSettings.json"
        if not cfg_path.exists():
            return

        try:
            data = json.loads(cfg_path.read_text(encoding="utf-8"))
        except Exception:
            return

        mw = data.get("Middleware") if isinstance(data, dict) else None
        if not isinstance(mw, dict):
            return

        if self._mw_name is not None and mw.get("Name") is not None:
            self._mw_name.setText(str(mw["Name"]))
        if self._mw_addr is not None and mw.get("NetworkAddress") is not None:
            self._mw_addr.setText(str(mw["NetworkAddress"]))
        if self._mw_local is not None and mw.get("LocalDomain") is not None:
            self._mw_local.setText(str(mw["LocalDomain"]))
        if self._mw_external is not None and mw.get("ExternalDomain") is not None:
            self._mw_external.setText(str(mw["ExternalDomain"]))


    def _find_project_root(self) -> Path:
        """Locate the project root by searching for run.py upward."""
        here = Path(__file__).resolve().parent
        for candidate in [here, *here.parents]:
            if (candidate / "run.py").exists():
                return candidate
        return here


    # ---------- Existing behaviour ----------
    def _launch_gui(self, script_name: str):
        """Launch a GUI script located under the decision_support module."""
        import sys

        # main_window.py location: <root>/app/ui/main_window.py
        root = Path(__file__).resolve().parents[2]
        ds_dir = root / "modules" / "decision_support"
        script = ds_dir / script_name

        if not script.exists():
            try:
                self.module_decision.append_log(f"[RUN ERR] not found: {script}")
            except Exception:
                pass
            return

        try:
            subprocess.Popen(
                [sys.executable, str(script), *extra_args],
                cwd=str(root),
                shell=False,
                creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0),
            )
        except Exception as e:
            try:
                self.module_decision.append_log(f"[RUN ERR] {e}")
            except Exception:
                pass


    def _bind_module_buttons(self):
        buttons = {
            "decision": getattr(getattr(self, "module_decision", None), "btn_run", None),
            "mission": getattr(getattr(self, "module_mission", None), "btn_run", None),
            "monitor": getattr(getattr(self, "module_monitor", None), "btn_run", None),
        }

        if not any(buttons.values()):
            return

        for btn in buttons.values():
            if btn is None:
                continue
            try:
                btn.clicked.disconnect()
            except Exception:
                pass

        if buttons["decision"]:
            buttons["decision"].clicked.connect(lambda _checked=False: self._launch_role("decision"))
        if buttons["mission"]:
            buttons["mission"].clicked.connect(lambda _checked=False: self._launch_role("mission"))
        if buttons["monitor"]:
            buttons["monitor"].clicked.connect(lambda _checked=False: self._launch_role("monitor"))

    def mark_received(self, msg_id: str, raw: bytes | None = None):
        mid = str(msg_id)

        def handle(module_key: str, kind: str):
            # kind: "tx" means out, "rx" means in
            mod = {"mission": self.module_mission,
                "monitor": self.module_monitor,
                "decision": self.module_decision}[module_key]
            if kind == "tx":
                if hasattr(mod, "bump_tx"): mod.bump_tx(mid)
                if hasattr(self, "flow"):   self.flow.trigger(module_key, "out")
                if hasattr(mod, "append_log"): mod.append_log(f"[{mid}] PUSH sent")
            else:
                if hasattr(mod, "bump_rx"): mod.bump_rx(mid)
                if hasattr(self, "flow"):   self.flow.trigger(module_key, "in")
                if hasattr(mod, "append_log"): mod.append_log(f"[{mid}] RX received")

        maps = getattr(self, "_msg_maps", {})
        for key in ("mission", "monitor", "decision"):
            m = maps.get(key, {})
            if mid in m.get("tx", set()):
                handle(key, "tx")
            if mid in m.get("rx", set()):
                handle(key, "rx")

    def _init_msg_monitor(self):
        from importlib import import_module
        from receive_center import register_listener  # relay incoming messages to GUI

        # Map of message definitions
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

        # Optional: register_listener(mid, self) for all IDs if needed

    def _launch_role(self, role: str, *, schedule_powerup: bool = True):
        import sys, subprocess
        from pathlib import Path

        self._debug_log(f'_launch_role called role={role}')
        root = Path(__file__).resolve().parents[2]

        target_log = None
        extra_args = []
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
                root / "modules" / "decision_support" / "assignment_planning_gui.py",
                root / "app"     / "modules" / "decision_support" / "assignment_planning_gui.py",
            ]
            target_log = self.module_mission

        elif role == "monitor":
            candidates = [
                root / "modules" / "monitoring_ver2" / "monitoring_gui.py",
                root / "app"     / "modules" / "monitoring_ver2" / "monitoring_gui.py",
            ]
            target_log = self.module_monitor

        elif role == "info":
            candidates = [
                root / "modules" / "info_manage" / "info_manage.py",
                root / "app"     / "modules" / "info_manage" / "info_manage.py",
            ]
        elif role == "integration":
            candidates = [
                root / "modules" / "integration_module" / "integration_gui.py",
                root / "app"     / "modules" / "integration_module" / "integration_gui.py",
            ]
        else:
            return

        existing = self._role_processes.get(role)
        if existing and existing.poll() is not None:
            self._role_processes.pop(role, None)
            existing = None
        if existing and existing.poll() is None:
            try:
                if target_log is not None:
                    target_log.append_log("[RUN] already running")
            except Exception:
                pass
            return

        script = next((p for p in candidates if p.exists()), None)
        if not script:
            self._debug_log(f'_launch_role script not found role={role}')
            try:
                if target_log is not None:
                    target_log.append_log("[RUN ERR] not found:\n" + "\n".join(str(p) for p in candidates))
            except Exception:
                pass
            return

        try:
            offset_map = {
                "mission": "40,40",
                "monitor": "130,90",
                "decision": "220,140",
                "info": "310,190",
                "integration": "400,240",
            }
            env = os.environ.copy()
            env["KU_WINDOW_OFFSET"] = offset_map.get(role, "40,40")
            self._debug_log(f'_launch_role resolved script={script}')
            proc = subprocess.Popen([sys.executable, str(script), *extra_args], cwd=str(root),
                                    shell=False,
                                    env=env,
                                    creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
            self._role_processes[role] = proc
            try:
                if target_log is not None:
                    target_log.append_log(f"[RUN] launched {script.name}")
            except Exception:
                pass
            if schedule_powerup and role in ("mission", "monitor", "decision"):
                self._schedule_module_powerup(role)
        except Exception as e:
            self._debug_log(f'_launch_role error role={role} err={e}')
            try:
                if target_log is not None:
                    target_log.append_log(f"[RUN ERR] {e}")
            except Exception:
                pass

    
    def _launch_simulation(self):
        root = Path(__file__).resolve().parents[2]
        script = root / "sim_main.py"
        if not script.exists():
            self._log_to_modules(f"[RUN ERR] not found: {script}")
            return

        existing = self._role_processes.get("sim")
        if existing and existing.poll() is None:
            self._log_to_modules("[RUN] Simulation already running")
            return

        try:
            proc = subprocess.Popen([sys.executable, str(script)], cwd=str(root))
            try:
                import webbrowser
                try:
                    from modules.sim.config import SERVER_HOST, SERVER_PORT
                    host = str(SERVER_HOST)
                    port = int(SERVER_PORT)
                    if host in ("0.0.0.0", "::"):
                        host = "127.0.0.1"
                    url = f"http://{host}:{port}/"
                except Exception:
                    url = "http://127.0.0.1:8000/"
                webbrowser.open(url, new=2)
            except Exception:
                pass
            self._role_processes["sim"] = proc
            self._log_to_modules("[RUN] Simulation launched")
        except Exception as exc:
            self._log_to_modules(f"[RUN ERR] Simulation launch failed: {exc}")

    def _install_flow_test_shortcuts(self):
        """Setup demo shortcuts for flow visualizer."""
        # 1/2: Monitoring in/out
        QShortcut(QKeySequence("1"), self, activated=lambda: self._pulse("monitor", "in"))
        QShortcut(QKeySequence("2"), self, activated=lambda: self._pulse("monitor", "out"))
        # 3/4: Assignment in/out
        QShortcut(QKeySequence("3"), self, activated=lambda: self._pulse("mission", "in"))
        QShortcut(QKeySequence("4"), self, activated=lambda: self._pulse("mission", "out"))
        # 5/6: Decision in/out
        QShortcut(QKeySequence("5"), self, activated=lambda: self._pulse("decision", "in"))
        QShortcut(QKeySequence("6"), self, activated=lambda: self._pulse("decision", "out"))

        # D: Toggle demo flow
        QShortcut(QKeySequence("D"), self, activated=self._toggle_demo_flow)

        # Demo timer configuration
        self._demo_timer = QTimer(self)
        self._demo_timer.setInterval(100)  # 0.6 s interval
        self._demo_timer.timeout.connect(self._demo_step)
        self._demo_seq = [
            ("monitor", "in"), ("monitor", "out"),
            ("mission", "in"), ("mission", "out"),
            ("decision", "in"), ("decision", "out"),
        ]
        self._demo_idx = 0

    def _pulse(self, module: str, direction: str):
        """Trigger flow animation helpers."""
        if hasattr(self, "flow") and self.flow:
            self.flow.trigger(module, direction)

    def _add_left_placeholder(self, grid: QGridLayout) -> None:
        mode_zone = ZONES.get("MODE_BUTTONS")
        flow_zone = ZONES.get("FLOW_VIS")

        if not mode_zone or not flow_zone:
            for key in ("MODE_BUTTONS", "FLOW_VIS"):
                self._add_placeholder(grid, key)
            return

        row0 = min(mode_zone["r0"], flow_zone["r0"])
        row_end = max(mode_zone["r0"] + mode_zone["rs"],
                       flow_zone["r0"] + flow_zone["rs"])
        col0 = min(mode_zone["c0"], flow_zone["c0"])
        col_end = max(mode_zone["c0"] + mode_zone["cs"],
                       flow_zone["c0"] + flow_zone["cs"])

        placeholder = Card("", self)
        placeholder.setObjectName("LEFT_PLACEHOLDER")
        placeholder.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        body = getattr(placeholder, 'body_layout', None)
        if body is not None:
            body.setSpacing(16)
            body.setContentsMargins(18, 16, 18, 16)

            self.btn_module_shutdown = QPushButton("모듈 종료", placeholder)
            self.btn_module_shutdown.setObjectName("BtnModuleShutdown")
            self.btn_module_shutdown.setMinimumHeight(32)
            self.btn_module_shutdown.clicked.connect(self._handle_module_shutdown)
            body.addWidget(self.btn_module_shutdown)

            self.btn_integration_module = QPushButton("통합모듈 실행", placeholder)
            self.btn_integration_module.setObjectName("BtnIntegrationModule")
            self.btn_integration_module.setMinimumHeight(32)
            self.btn_integration_module.clicked.connect(lambda: self._launch_role("integration"))
            body.addWidget(self.btn_integration_module)

            self.btn_simulation_run = QPushButton("Simulation \uc2e4\ud589", placeholder)
            self.btn_simulation_run.setObjectName("BtnSimulationRun")
            self.btn_simulation_run.setMinimumHeight(32)
            self.btn_simulation_run.clicked.connect(self._launch_simulation)
            body.addWidget(self.btn_simulation_run)

            self.btn_overwrite_020x = QPushButton("0201/0203 덮어쓰기", placeholder)
            self.btn_overwrite_020x.setObjectName("BtnOverwrite020x")
            self.btn_overwrite_020x.setMinimumHeight(32)
            self.btn_overwrite_020x.clicked.connect(self._handle_overwrite_020x)
            body.addWidget(self.btn_overwrite_020x)


            self.btn_decision_reset = QPushButton("의사결정 SW 초기화", placeholder)
            self.btn_decision_reset.setObjectName("BtnDecisionReset")
            self.btn_decision_reset.setMinimumHeight(32)
            body.addWidget(self.btn_decision_reset)

            body.addStretch(1)

        grid.addWidget(placeholder, row0, col0, row_end - row0, col_end - col0)

    def _debug_log(self, message: str) -> None:
        # Debug logging disabled to avoid creating log files
        return

    def _module_widget(self, role: str):
        return {
            'mission': getattr(self, 'module_mission', None),
            'monitor': getattr(self, 'module_monitor', None),
            'decision': getattr(self, 'module_decision', None),
        }.get(role)

    def _set_all_module_modes(self, text: str) -> None:
        target = str(text)
        for role in ('mission', 'monitor', 'decision'):
            mod = self._module_widget(role)
            if not mod:
                continue
            setter = getattr(mod, 'set_mode_text', None)
            if callable(setter):
                try:
                    setter(target)
                except Exception:
                    pass

    def _log_to_modules(self, message: str) -> None:
        for role in ('mission', 'monitor', 'decision'):
            self._log_to_module(role, message)

    def _log_to_module(self, role: str, message: str) -> None:
        mod = self._module_widget(role)
        if mod and hasattr(mod, 'append_log'):
            try:
                mod.append_log(message)
            except Exception:
                pass

    def _broadcast_ctrl(self, payload: dict) -> None:
        self._debug_log(f'_broadcast_ctrl payload={payload}')
        data = json.dumps(payload).encode('utf-8')
        targets = [
            ('mission', 45981),
            ('monitoring', 45982),
            ('decision', 45983),
            ('info', 45984),
        ]
        for _role, port in targets:
            sock = None
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.sendto(data, ('127.0.0.1', port))
            except Exception as exc:
                self._debug_log(f'_broadcast_ctrl failed target={_role} err={exc}')
            finally:
                if sock is not None:
                    try:
                        sock.close()
                    except Exception:
                        pass

    def _send_ctrl_single(self, target: str, payload: dict) -> bool:
        port_map = {'mission': 45981, 'monitor': 45982, 'decision': 45983}
        port = port_map.get(target)
        if port is None:
            self._debug_log(f'_send_ctrl_single unknown target={target}')
            return False

        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            data = json.dumps(payload).encode('utf-8')
            sock.sendto(data, ('127.0.0.1', port))
            self._debug_log(f'_send_ctrl_single ok target={target} payload={payload}')
            return True
        except Exception as exc:
            self._debug_log(f'_send_ctrl_single failed target={target} err={exc}')
            return False
        finally:
            if sock is not None:
                try:
                    sock.close()
                except Exception:
                    pass

    def _schedule_module_powerup(self, role: str) -> None:
        self._debug_log(f'_schedule_module_powerup role={role}')

        def send_mode_on():
            ok = self._send_ctrl_single(role, {'cmd': 'mode', 'text': '초기화 모드'})
            if ok:
                self._log_to_module(role, '[AUTO] 초기화 모드 broadcast ("초기화 모드")')
            else:
                self._log_to_module(role, '[AUTO WARN] 초기화 모드 send failed')

        def send_self_check():
            ok = self._send_ctrl_single(role, {'cmd': 'self_check', 'status': 1})
            if ok:
                self._log_to_module(role, '[AUTO] self-check requested (0102)')
            else:
                self._log_to_module(role, '[AUTO WARN] self-check send failed')

        QTimer.singleShot(1000, send_mode_on)
        QTimer.singleShot(2000, send_self_check)

    def _handle_auto_boot(self) -> None:
        self._debug_log('auto boot triggered')
        self._log_to_modules('[AUTO] boot sequence started')

        for role in ("mission", "monitor", "decision", "info"):
            self._log_to_module(role, '[AUTO] module launch requested')
            try:
                self._debug_log(f'launching role={role}')
                self._launch_role(role, schedule_powerup=False)
            except Exception as exc:
                self._log_to_module(role, '[AUTO WARN] launch failed')
                self._debug_log(f'launch failed role={role} err={exc}')

        def _set_init_mode():
            self._log_to_modules('[AUTO] 초기화 모드 broadcast')
            self._set_all_module_modes("초기화 모드")
            self._broadcast_ctrl({'cmd': 'mode', 'text': '초기화 모드'})

        QTimer.singleShot(1000, _set_init_mode)

    def _handle_module_shutdown(self) -> None:
        for role, proc in list(self._role_processes.items()):
            if not proc:
                self._role_processes.pop(role, None)
                continue
            if proc.poll() is None:
                try:
                    proc.terminate()
                    proc.wait(timeout=3)
                except Exception:
                    try:
                        proc.kill()
                    except Exception:
                        pass
            self._role_processes.pop(role, None)
        for mod in (self.module_mission, self.module_monitor, self.module_decision):
            if mod and hasattr(mod, 'append_log'):
                try:
                    mod.append_log('[RUN] module shutdown requested')
                except Exception:
                    pass

    def _handle_overwrite_020x(self) -> None:
        # Step 1: standby + system mode(0101) request through INF
        self._set_all_module_modes("대기모드")
        self._broadcast_ctrl({"cmd": "mode", "text": "standby"})
        self._broadcast_ctrl({"cmd": "system_mode", "mode": 1})
        self._log_to_modules("[RUN] overwrite sequence: step1 standby (system_mode=1)")

        def _step2_overwrite():
            try:
                src_root = db_paths.PROJECT_ROOT / "Logs"
                tasks = [
                    ("0201", src_root / "InputMissionPlan", db_paths.get_db_subpath("InputMissionPlan")),
                    ("0203", src_root / "MissionReferenceInfo", db_paths.get_db_subpath("MissionReferenceInfo")),
                ]
                messages = []
                total = 0
                for code, src_dir, dest_dir in tasks:
                    src_dir = Path(src_dir)
                    dest_dir = Path(dest_dir)
                    if not src_dir.exists():
                        messages.append(f"{code}: source missing ({src_dir})")
                        continue
                    count = 0
                    for path in src_dir.rglob("*"):
                        if not path.is_file():
                            continue
                        target = dest_dir / path.relative_to(src_dir)
                        target.parent.mkdir(parents=True, exist_ok=True)
                        shutil.copy2(path, target)
                        count += 1
                    dest_dir.mkdir(parents=True, exist_ok=True)
                    messages.append(f"{code}: copied {count} files -> {dest_dir}")
                    total += count

                info = db_paths.get_info()
                scenario = info.get("scenario_dir") or "(scenario unknown)"
                summary = "\n".join(messages) if messages else "No files copied."

                if total > 0:
                    self._log_to_modules(f"[RUN] 0201/0203 overwrite done ({total} files) -> {scenario}")
                    self._debug_log(f"overwrite 020x copied={total} scenario={scenario}")
                    QMessageBox.information(self, "0201/0203 overwrite", f"{summary}\n\nscenario: {scenario}")
                else:
                    self._log_to_modules("[RUN] 0201/0203 overwrite: no source files copied.")
                    self._debug_log(f"overwrite 020x no files scenario={scenario}")
                    QMessageBox.warning(self, "0201/0203 overwrite", f"No files copied.\n{summary}\n\nscenario: {scenario}")

                # Step 3 after 1 second
                QTimer.singleShot(1000, _step3_initial_plan)
            except Exception as exc:
                self._log_to_modules(f"[RUN] 0201/0203 overwrite failed: {exc}")
                self._debug_log(f"overwrite 020x error={exc}")
                QMessageBox.critical(self, "0201/0203 overwrite", f"Copy failed.\n{exc}")

        def _step3_initial_plan():
            self._set_all_module_modes("초기임무계획")
            self._broadcast_ctrl({"cmd": "mode", "text": "initplan"})
            self._broadcast_ctrl({"cmd": "system_mode", "mode": 2})
            self._log_to_modules("[RUN] overwrite sequence: step3 initial-plan mode (system_mode=2)")

        # Step 2 after 1 second
        QTimer.singleShot(1000, _step2_overwrite)

    def _add_placeholder(self, grid: QGridLayout, zone_key: str) -> None:
        placeholder = Card("", self)
        placeholder.setObjectName(f"{zone_key}_placeholder")
        placeholder.setAttribute(Qt.WA_TransparentForMouseEvents)
        placeholder.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        body = getattr(placeholder, 'body_layout', None)
        if body is not None:
            body.addStretch(1)
        self._add_zone(grid, placeholder, zone_key)

    def _normalize_module_columns(self, grid: QGridLayout) -> None:
        zone = ZONES.get("MODULE_CENTER")
        if not zone:
            return
        for col in range(zone["c0"], zone["c0"] + zone["cs"]):
            grid.setColumnStretch(col, 1)

    def _add_zone(self, grid: QGridLayout, w: QWidget, key: str):
        """Add widget to the grid using ZONES metadata."""
        z = ZONES[key]
        grid.addWidget(w, z["r0"], z["c0"], z["rs"], z["cs"])

    def closeEvent(self, event):
        """Shutdown child modules before closing."""
        try:
            self._handle_module_shutdown()
        except Exception:
            pass
        super().closeEvent(event)

    # ---------- Actions ----------
    def _browse_db(self):
        path = QFileDialog.getExistingDirectory(self, "Select database directory")
        if path:
            selected = Path(path)
            looks_like_db = (
                selected.name.lower() == "database"
                or (selected / "mission_plan_seq.txt").exists()
                or (selected / "InputMissionPlan").exists()
                or (selected / "MissionPlan").exists()
            )
            if looks_like_db:
                info = db_paths.set_manual_db_root(path, source="manual-browse")
                self._current_db_root = info.get("db_root") or path
                self._db_path_line.setText(self._current_db_root)
                self.update_scenario_root(info.get("base_root"))
                self.update_scenario_status_indicator(False, f"수동 DB 선택: {self._current_db_root}")
                # Record path selection in module logs when modules are available
                for attr in ("module_mission", "module_monitor", "module_decision"):
                    mod = getattr(self, attr, None)
                    if mod and hasattr(mod, "append_log"):
                        try:
                            mod.append_log(f"[PATH] {self._current_db_root}")
                        except Exception:
                            pass
            else:
                info = db_paths.set_scenario_base_root(path)
                base_root = info.get("base_root")
                self.update_scenario_root(base_root)
                display = base_root or ""
                self.update_scenario_status_indicator(False, f"시나리오 베이스 지정: {display}")
                for attr in ("module_mission", "module_monitor", "module_decision"):
                    mod = getattr(self, attr, None)
                    if mod and hasattr(mod, "append_log"):
                        try:
                            mod.append_log(f"[SCENARIO ROOT] {display}")
                        except Exception:
                            pass

    def _open_reference_pdf(self) -> None:
        target = Path(REFERENCE_PDF_PATH)
        if not target.exists():
            QMessageBox.warning(
                self,
                "파일 없음",
                f"지정한 문서를 찾을 수 없습니다.\n{target}",
            )
            return
        try:
            import sys
            if sys.platform.startswith("win"):
                os.startfile(str(target))  # type: ignore[attr-defined]
            else:
                QDesktopServices.openUrl(QUrl.fromLocalFile(str(target)))
        except Exception as exc:
            QMessageBox.warning(
                self,
                "열기 실패",
                f"문서를 여는 중 오류가 발생했습니다.\n{exc}",
            )

    
    def update_scenario_status_indicator(self, changed: bool, tooltip: Optional[str] | None = None) -> None:
        if self._scenario_status_dot is None:
            return
        active = bool(changed)
        self._scenario_status_dot.setProperty("active", active)
        label_text = "신규 DB 설정 여부 (신규)" if active else "신규 DB 설정 여부 (유지)"
        if self._scenario_status_label is not None:
            self._scenario_status_label.setText(label_text)
            self._scenario_status_label.setProperty("active", active)
        if tooltip is not None:
            tip_text = tooltip or ""
            self._scenario_status_dot.setToolTip(tip_text)
            if self._scenario_status_label is not None:
                self._scenario_status_label.setToolTip(tip_text)
        self._scenario_status_dot.style().unpolish(self._scenario_status_dot)
        self._scenario_status_dot.style().polish(self._scenario_status_dot)
        self._scenario_status_dot.update()
        if self._scenario_status_label is not None:
            self._scenario_status_label.style().unpolish(self._scenario_status_label)
            self._scenario_status_label.style().polish(self._scenario_status_label)
            self._scenario_status_label.update()


    def update_db_root(self, path: str | Path) -> None:
        self._current_db_root = str(path)
        if self._db_path_line is not None:
            self._db_path_line.setText(self._current_db_root)

    def update_scenario_root(self, path: str | Path | None) -> None:
        text = str(path) if path else ""
        if self._scenario_root_line is not None:
            self._scenario_root_line.setText(text)
            self._scenario_root_line.setToolTip(text or "Scenario base (optional)")
        if self._db_path_line is not None:
            if text:
                self._db_path_line.setToolTip(f"Scenario base: {text}")
            else:
                self._db_path_line.setToolTip(self._current_db_root or "")

    def _toggle_demo_flow(self):
        """Toggle demo animation with the D shortcut."""
        if self._demo_timer.isActive():
            self._demo_timer.stop()
        else:
            self._demo_idx = 0
            self._demo_timer.start()

    def _demo_step(self):
        """Advance demo animation step."""
        mod, direc = self._demo_seq[self._demo_idx]
        self._pulse(mod, direc)
        self._demo_idx = (self._demo_idx + 1) % len(self._demo_seq)

