# /mnt/data/main_window.py
# -*- coding: utf-8 -*-
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QGridLayout, QPushButton, QLabel, QLineEdit, QFileDialog, QShortcut,
    QHBoxLayout
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QKeySequence
from .zones import GRID_ROWS, GRID_COLS, ZONES
from ..widgets.cards import Card
from ..widgets.module_with_log import ModuleWithLog
from ..widgets.mode_buttons_panel import ModeButtonsPanel
from ..widgets.flow_visualizer import FlowVisualizer
from ..widgets.operation_flow_panel import OperationFlowPanel
import os, subprocess, json
from pathlib import Path

class MainWindow(QMainWindow):
    """Main dashboard window arranged on a 35x50 grid."""
    def __init__(self):
        super().__init__()
        self.setWindowTitle("KU Mission Decision Support Dashboard")
        self.resize(1800, 900)

        self._db_path_line: QLineEdit = None

        # Middleware widget references
        self._mw_name: QLineEdit = None
        self._mw_addr: QLineEdit = None
        self._mw_local: QLineEdit = None
        self._mw_external: QLineEdit = None

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

        # Title label
        title_lbl = QLabel("KU Mission Decision Support Dashboard", self)
        title_lbl.setObjectName("MainTitle")
        title_lbl.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self._add_zone(grid, title_lbl, "TITLE")

        btn_browse = QPushButton("Browse...")
        btn_browse.setMinimumHeight(28)
        btn_browse.clicked.connect(self._browse_db)
        self._add_zone(grid, btn_browse, "ROUTE_BUTTON")

        # Database path entry
        self._db_path_line = QLineEdit(self)
        self._db_path_line.setObjectName("DbPathLine")
        self._db_path_line.setPlaceholderText("Database directory")
        self._db_path_line.setReadOnly(True)

        # Apply default path and sync KU_MISSION_DB_ROOT
        default_db_path = self._find_project_root() / "database"
        try:
            default_db_path.mkdir(parents=True, exist_ok=True)
        except Exception:
            pass

        default_db_path_str = str(default_db_path)
        self._db_path_line.setText(default_db_path_str)
        os.environ["KU_MISSION_DB_ROOT"] = default_db_path_str

        self._add_zone(grid, self._db_path_line, "DB_PATH")

        # Middleware configuration row
        mw_row = self._make_middleware_row()
        self._add_zone(grid, mw_row, "MIDDLEWARE")

        # Module cards
        self.module_mission  = ModuleWithLog("Assignment Planning Module")
        self._add_zone(grid, self.module_mission, "MODULE_MISSION_COMBO")
        self.module_monitor  = ModuleWithLog("Mission Monitoring Module")
        self._add_zone(grid, self.module_monitor, "MODULE_MONITOR_COMBO")
        self.module_decision = ModuleWithLog("Decision Support Module")
        self._add_zone(grid, self.module_decision, "MODULE_DECISION_COMBO")

        # Flow visualizer card
        self.flow = FlowVisualizer()
        self._add_zone(grid, self.flow, "FLOW_VIS")

        # Mode buttons panel
        self._add_zone(grid, ModeButtonsPanel(), "MODE_BUTTONS")

        # Operation flow panel
        self.operation_panel = OperationFlowPanel()
        self._add_zone(grid, self.operation_panel, "OPS_FLOW")
        # Operation flow panel
        footer = QLabel("KU Mission Decision Support Dashboard", self)
        footer.setObjectName("FooterFull")
        footer.setAlignment(Qt.AlignCenter)
        self._add_zone(grid, footer, "FOOTER")

        self.setCentralWidget(root)

        # Install flow test shortcuts
        self._install_flow_test_shortcuts()

        self._bind_module_buttons()
        self._init_msg_monitor()
        self._apply_middleware()

    # ---------- Middleware helpers ----------
    def _make_middleware_row(self) -> QWidget:
        """Build the inline middleware configuration row."""
        w = QWidget(self)
        lay = QHBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(8)

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
        btn_apply.setMinimumWidth(80)
        btn_apply.clicked.connect(self._apply_middleware)
        lay.addWidget(btn_apply, 0, Qt.AlignRight)

        self._load_middleware_config()

        return w


    def _apply_middleware(self) -> None:
        """Persist middleware settings to nFusionSettings.json."""
        name = (self._mw_name.text() or "").strip() or "AVS1"
        net = (self._mw_addr.text() or "").strip() or "192."
        if not net.endswith("."):
            net += "."
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
        target = proj_root / "nFusionSettings.json"

        data = json.dumps(cfg, ensure_ascii=False, separators=(",", ":"))
        try:
            target.write_text(data, encoding="utf-8")
            msg = (f"[CFG] nFusionSettings updated {target} | "
                   f"Name={name}, Net={net}, Local={local}, External={ext}")
        except Exception as e:
            msg = f"[CFG ERR] {target.name}: {e}"

        for mod in (getattr(self, "module_mission", None),
                    getattr(self, "module_monitor", None),
                    getattr(self, "module_decision", None)):
            try:
                mod.append_log(msg)
            except Exception:
                pass


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
                [sys.executable, str(script)],
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
                # fallback (legacy)
                root / "modules" / "decision_support" / "assignment_planning_gui.py",
                root / "app"     / "modules" / "decision_support" / "assignment_planning_gui.py",
            ]
            target_log = self.module_mission

        elif role == "monitor":
            candidates = [
                root / "modules" / "monitoring" / "monitoring_gui.py",
                root / "app"     / "modules" / "monitoring" / "monitoring_gui.py",
                # fallback (decision support directory)
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

    def _add_zone(self, grid: QGridLayout, w: QWidget, key: str):
        """Add widget to the grid using ZONES metadata."""
        z = ZONES[key]
        grid.addWidget(w, z["r0"], z["c0"], z["rs"], z["cs"])

    # ---------- Actions ----------
    def _browse_db(self):
        path = QFileDialog.getExistingDirectory(self, "Select database directory")
        if path:
            self._db_path_line.setText(path)
            os.environ["KU_MISSION_DB_ROOT"] = path
            # Record path selection in module logs
            self.module_mission.append_log(f"[PATH] {path}")
            self.module_monitor.append_log(f"[PATH] {path}")
            self.module_decision.append_log(f"[PATH] {path}")

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

