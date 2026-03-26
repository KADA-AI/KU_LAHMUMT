# /mnt/data/main_window.py
# -*- coding: utf-8 -*-
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QGridLayout, QPushButton, QLabel, QLineEdit, QFileDialog, QShortcut,
    QHBoxLayout, QVBoxLayout, QSizePolicy, QMessageBox, QPlainTextEdit
)
from PyQt5.QtCore import Qt, QTimer, QUrl
from PyQt5.QtGui import QKeySequence, QDesktopServices
from typing import Optional
from datetime import datetime
from .zones import GRID_ROWS, GRID_COLS, ZONES
from ..widgets.cards import Card
from ..widgets.module_with_log import ModuleWithLog
from ..widgets.operation_flow_panel import OperationFlowPanel
import os, sys, subprocess, json, socket, shutil, re
from pathlib import Path
from modules.common import db_paths
from modules.common.process_console import (
    creationflags_for_subprocess,
    preferred_console_python,
    should_show_module_consoles,
)

CHANGE_LOG_PATH = db_paths.PROJECT_ROOT / "change_log.md"
_VERSION_LINE_RE = re.compile(r"^\d{4}-\d{2}-\d{2}\s+(v[0-9.]+)\s+-")


def _load_app_version() -> str:
    try:
        for line in CHANGE_LOG_PATH.read_text(encoding="utf-8").splitlines():
            match = _VERSION_LINE_RE.match(line.strip())
            if match:
                return match.group(1)
    except Exception:
        pass
    return "v1.1.31"


APP_VERSION = _load_app_version()
APP_TITLE = f"KU Mission Decision Support Dashboard ({APP_VERSION})"
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
        self._path_card: Optional[QWidget] = None
        self._path_validation_label: Optional[QLabel] = None
        self._scenario_status_dot: Optional[QLabel] = None
        self._scenario_status_label: Optional[QLabel] = None
        self._version_notes: Optional[QPlainTextEdit] = None
        self._version_notes_path: Path = CHANGE_LOG_PATH

        # Middleware widget references
        self._middleware_card: Optional[QWidget] = None
        self._middleware_status_label: Optional[QLabel] = None
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
        shell = QVBoxLayout(root)
        shell.setContentsMargins(18, 16, 18, 16)
        shell.setSpacing(12)

        self._db_path_line = QLineEdit(self)
        self._db_path_line.setObjectName("DbPathLine")
        self._db_path_line.setPlaceholderText("Database directory")
        self._db_path_line.setReadOnly(True)
        self._db_path_line.setMinimumHeight(36)

        self._scenario_root_line = QLineEdit(self)
        self._scenario_root_line.setObjectName("ScenarioRootLine")
        self._scenario_root_line.setPlaceholderText("Scenario base")
        self._scenario_root_line.setReadOnly(True)
        self._scenario_root_line.setMinimumHeight(36)

        info = db_paths.get_info()
        default_db_path = info.get("db_root") or db_paths.get_active_db_root_str()
        self._current_db_root = str(default_db_path)
        self._db_path_line.setText(self._current_db_root)
        self.update_scenario_root(info.get("base_root"))

        shell.addWidget(self._build_header_bar(), 0)

        content_row = QHBoxLayout()
        content_row.setSpacing(12)

        left_col = QVBoxLayout()
        left_col.setSpacing(14)
        left_col.addWidget(self._build_scenario_card(), 0)
        left_col.addWidget(self._make_middleware_row(), 0)
        left_col.addStretch(1)
        content_row.addLayout(left_col, 7)

        action_panel = self._build_action_panel()
        action_panel.setMinimumWidth(280)
        action_panel.setMaximumWidth(320)
        content_row.addWidget(action_panel, 3)

        shell.addLayout(content_row, 0)
        shell.addWidget(self._build_version_notes(), 1)

        self.operation_panel = None
        self.setCentralWidget(root)
        self._install_flow_test_shortcuts()
        self._bind_module_buttons()
        self._init_msg_monitor()
        self._apply_middleware()
        self.validate_launch_prerequisites(show_message=False)
        return

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
        subtitle_lbl = QLabel("최근 업데이트 날짜 : 26-03-09", title_wrap)
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
        self._path_card = path_container
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
        self._path_validation_label = QLabel(self)
        self._path_validation_label.setObjectName("PathValidationLabel")
        self._path_validation_label.setWordWrap(True)
        path_layout.addWidget(self._path_validation_label)
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

    def _build_header_bar(self) -> QWidget:
        card = Card("", self, dense=True)
        card.setObjectName("HeaderBar")
        body = getattr(card, "body_layout", None)
        if body is None:
            return card
        body.setContentsMargins(20, 18, 20, 18)
        body.setSpacing(12)

        top = QHBoxLayout()
        top.setContentsMargins(0, 0, 0, 0)
        top.setSpacing(18)

        title_col = QVBoxLayout()
        title_col.setContentsMargins(0, 0, 0, 0)
        title_col.setSpacing(6)

        title = QLabel("KU Mission Decision Support", card)
        title.setObjectName("HeroTitle")
        title_col.addWidget(title, 0, Qt.AlignLeft)

        meta_row = QHBoxLayout()
        meta_row.setContentsMargins(0, 4, 0, 0)
        meta_row.setSpacing(8)

        version_badge = QLabel(APP_VERSION, card)
        version_badge.setObjectName("HeaderBadge")
        meta_row.addWidget(version_badge, 0, Qt.AlignLeft)

        try:
            updated_at = datetime.fromtimestamp(self._version_notes_path.stat().st_mtime).strftime("%Y-%m-%d")
        except Exception:
            updated_at = datetime.now().strftime("%Y-%m-%d")
        updated_badge = QLabel(f"Updated {updated_at}", card)
        updated_badge.setObjectName("HeaderBadge")
        meta_row.addWidget(updated_badge, 0, Qt.AlignLeft)
        meta_row.addStretch(1)
        title_col.addLayout(meta_row)

        top.addLayout(title_col, 1)

        actions = QVBoxLayout()
        actions.setContentsMargins(0, 0, 0, 0)
        actions.setSpacing(8)

        self.btn_reference_pdf = QPushButton("참고 문서", card)
        self.btn_reference_pdf.setObjectName("PlainBtn")
        self.btn_reference_pdf.setFixedHeight(34)
        self.btn_reference_pdf.clicked.connect(self._open_reference_pdf)
        actions.addWidget(self.btn_reference_pdf)

        top.addLayout(actions, 0)
        body.addLayout(top)
        return card

    def _build_scenario_card(self) -> QWidget:
        card = Card("", self, dense=True)
        card.setObjectName("ScenarioCard")
        body = getattr(card, "body_layout", None)
        if body is None:
            return card
        body.setContentsMargins(18, 16, 18, 16)
        body.setSpacing(12)

        header = QHBoxLayout()
        header.setContentsMargins(0, 0, 0, 0)
        header.setSpacing(10)

        title_col = QVBoxLayout()
        title_col.setContentsMargins(0, 0, 0, 0)
        title_col.setSpacing(0)

        section = QLabel("Scenario & Paths", card)
        section.setObjectName("SectionLabel")
        title_col.addWidget(section, 0, Qt.AlignLeft)
        header.addLayout(title_col, 1)

        db_btn = QPushButton("DB 폴더 선택", card)
        db_btn.setObjectName("SecondaryButton")
        db_btn.setFixedHeight(34)
        db_btn.setMinimumWidth(118)
        db_btn.clicked.connect(self._browse_db)
        header.addWidget(db_btn, 0, Qt.AlignVCenter)

        status_wrap = QWidget(card)
        status_wrap.setObjectName("StatusCluster")
        status_wrap.setFixedHeight(34)
        status_lay = QHBoxLayout(status_wrap)
        status_lay.setContentsMargins(12, 0, 12, 0)
        status_lay.setSpacing(6)
        self._scenario_status_dot = QLabel("●", status_wrap)
        self._scenario_status_dot.setObjectName("ScenarioDot")
        status_lay.addWidget(self._scenario_status_dot, 0, Qt.AlignLeft | Qt.AlignVCenter)
        self._scenario_status_label = QLabel("현재 경로 사용 중", status_wrap)
        self._scenario_status_label.setObjectName("ScenarioStatusLabel")
        status_lay.addWidget(self._scenario_status_label, 0, Qt.AlignLeft | Qt.AlignVCenter)
        header.addWidget(status_wrap, 0, Qt.AlignVCenter)

        body.addLayout(header)

        def _field_block(title_text: str, widget: QWidget) -> QWidget:
            wrap = QWidget(card)
            wrap.setObjectName("FieldBlock")
            wrap.setAttribute(Qt.WA_StyledBackground, True)
            lay = QVBoxLayout(wrap)
            lay.setContentsMargins(0, 0, 0, 0)
            lay.setSpacing(6)
            caption = QLabel(title_text, wrap)
            caption.setObjectName("FieldCaption")
            lay.addWidget(caption, 0, Qt.AlignLeft)
            lay.addWidget(widget)
            return wrap

        self._scenario_root_line.setVisible(True)
        body.addWidget(_field_block("Active DB", self._db_path_line))
        body.addWidget(_field_block("Scenario Base", self._scenario_root_line))

        self._path_validation_label = QLabel(card)
        self._path_validation_label.setObjectName("PathValidationLabel")
        self._path_validation_label.setWordWrap(True)
        body.addWidget(self._path_validation_label)

        self.update_scenario_status_indicator(False, "현재 선택된 경로를 사용 중입니다.")
        return card

    def _build_action_panel(self) -> QWidget:
        card = Card("", self, dense=True)
        card.setObjectName("ActionPanel")
        card.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Expanding)
        body = getattr(card, "body_layout", None)
        if body is None:
            return card
        body.setContentsMargins(16, 16, 16, 16)
        body.setSpacing(10)

        title = QLabel("Quick Actions", card)
        title.setObjectName("SectionLabel")
        body.addWidget(title, 0, Qt.AlignLeft)

        button_stack = QVBoxLayout()
        button_stack.setContentsMargins(0, 2, 0, 0)
        button_stack.setSpacing(10)

        self.btn_simulation_run = QPushButton("Simulation 실행", card)
        self.btn_simulation_run.setObjectName("BtnSimulationRun")
        self.btn_simulation_run.setFixedHeight(40)
        self.btn_simulation_run.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.btn_simulation_run.clicked.connect(self._launch_simulation)
        button_stack.addWidget(self.btn_simulation_run)

        self.btn_overwrite_020x = QPushButton("0201/0203 덮어쓰기", card)
        self.btn_overwrite_020x.setObjectName("BtnOverwrite020x")
        self.btn_overwrite_020x.setFixedHeight(40)
        self.btn_overwrite_020x.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.btn_overwrite_020x.clicked.connect(self._handle_overwrite_020x)
        button_stack.addWidget(self.btn_overwrite_020x)

        self.btn_module_shutdown = QPushButton("모듈 종료", card)
        self.btn_module_shutdown.setObjectName("BtnModuleShutdown")
        self.btn_module_shutdown.setFixedHeight(40)
        self.btn_module_shutdown.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.btn_module_shutdown.clicked.connect(self._handle_module_shutdown)
        button_stack.addWidget(self.btn_module_shutdown)

        self.btn_decision_reset = QPushButton("의사결정 SW 초기화", card)
        self.btn_decision_reset.setObjectName("BtnDecisionReset")
        self.btn_decision_reset.setFixedHeight(40)
        self.btn_decision_reset.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        button_stack.addWidget(self.btn_decision_reset)

        body.addLayout(button_stack)
        body.addStretch(1)
        return card

    def _build_version_notes(self) -> QWidget:
        card = Card("", self, dense=True)
        card.setObjectName("VersionCard")
        body = getattr(card, "body_layout", None)
        if body is None:
            return card
        body.setContentsMargins(18, 16, 18, 16)
        body.setSpacing(10)

        header = QHBoxLayout()
        header.setContentsMargins(0, 0, 0, 0)
        header.setSpacing(8)

        title = QLabel("Change Log", card)
        title.setObjectName("SectionLabel")
        header.addWidget(title, 0, Qt.AlignLeft)
        header.addStretch(1)
        body.addLayout(header)

        self._version_notes = QPlainTextEdit(self)
        self._version_notes.setObjectName("VersionNotes")
        self._version_notes.setReadOnly(True)
        self._version_notes.setMinimumHeight(150)
        self._version_notes.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._version_notes.setPlainText(self._load_version_notes_text())
        body.addWidget(self._version_notes)
        return card

    # ---------- Middleware helpers ----------
    def _make_middleware_row(self) -> QWidget:
        card = Card("", self, dense=True)
        self._middleware_card = card
        card.setObjectName("MiddlewareCard")
        body = getattr(card, "body_layout", None)
        if body is None:
            return card
        body.setContentsMargins(18, 16, 18, 16)
        body.setSpacing(12)

        header = QHBoxLayout()
        header.setContentsMargins(0, 0, 0, 0)
        header.setSpacing(10)

        title_col = QVBoxLayout()
        title_col.setContentsMargins(0, 0, 0, 0)
        title_col.setSpacing(0)
        title = QLabel("Middleware", card)
        title.setObjectName("SectionLabel")
        title_col.addWidget(title, 0, Qt.AlignLeft)
        header.addLayout(title_col, 1)

        btn_apply = QPushButton("Apply", card)
        btn_apply.setObjectName("SecondaryButton")
        btn_apply.setFixedHeight(34)
        btn_apply.setMinimumWidth(88)
        btn_apply.clicked.connect(self._apply_middleware)
        header.addWidget(btn_apply, 0, Qt.AlignTop)
        body.addLayout(header)

        def _mk_line(ph: str, width: int, default: str) -> QLineEdit:
            le = QLineEdit(card)
            le.setPlaceholderText(ph)
            le.setMinimumWidth(width)
            le.setText(default)
            le.setFixedHeight(36)
            return le

        self._mw_name = _mk_line("AVS1", 110, "AVS1")
        self._mw_addr = _mk_line("203", 110, "192")
        self._mw_local = _mk_line("10", 72, "10")
        self._mw_external = _mk_line("100", 72, "100")

        form = QGridLayout()
        form.setContentsMargins(0, 0, 0, 0)
        form.setHorizontalSpacing(10)
        form.setVerticalSpacing(8)

        fields = [
            ("Name", self._mw_name),
            ("Network", self._mw_addr),
            ("Local", self._mw_local),
            ("External", self._mw_external),
        ]
        for col, (label_text, widget) in enumerate(fields):
            caption = QLabel(label_text, card)
            caption.setObjectName("FieldCaption")
            form.addWidget(caption, 0, col)
            form.addWidget(widget, 1, col)
            form.setColumnStretch(col, 1)

        body.addLayout(form)

        self._middleware_status_label = QLabel(card)
        self._middleware_status_label.setObjectName("MiddlewareStatusLabel")
        self._middleware_status_label.setWordWrap(True)
        body.addWidget(self._middleware_status_label)

        self._load_middleware_config()
        return card

    def _current_middleware_settings(self) -> dict:
        name = (self._mw_name.text() or "").strip() or "AVS1"
        network = (self._mw_addr.text() or "").strip() or "192"
        try:
            local = int((self._mw_local.text() or "10").strip())
        except Exception:
            local = 10
        try:
            external = int((self._mw_external.text() or "100").strip())
        except Exception:
            external = 100
        return {
            "Name": name,
            "NetworkAddress": network,
            "LocalDomain": local,
            "ExternalDomain": external,
        }

    def _apply_middleware(self) -> dict:
        """Persist middleware settings to every nFusionSettings.json in the project."""
        settings = self._current_middleware_settings()
        name = settings["Name"]
        net = settings["NetworkAddress"]
        local = settings["LocalDomain"]
        ext = settings["ExternalDomain"]

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
        self.validate_launch_prerequisites(show_message=False)
        return settings

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
            raw_network = str(mw["NetworkAddress"])
            normalized_network = self._normalize_network_prefix(raw_network)
            self._mw_addr.setText(normalized_network or raw_network.rstrip("."))
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

    def _nearest_existing_parent(self, path: Path) -> Optional[Path]:
        current = Path(path)
        while True:
            if current.exists():
                return current
            parent = current.parent
            if parent == current:
                return None
            current = parent

    def _can_prepare_directory(self, path: Path) -> tuple[bool, str]:
        try:
            target = Path(path)
            if target.exists():
                if not target.is_dir():
                    return False, f"폴더가 아님: {target}"
                if not os.access(target, os.W_OK):
                    return False, f"쓰기 불가: {target}"
                return True, "ok"
            parent = self._nearest_existing_parent(target)
            if parent is None:
                return False, f"상위 경로 없음: {target}"
            if not parent.is_dir():
                return False, f"상위 경로가 폴더가 아님: {parent}"
            if not os.access(parent, os.W_OK):
                return False, f"상위 경로 쓰기 불가: {parent}"
            return True, "ok"
        except Exception as exc:
            return False, str(exc)

    def _is_subpath(self, child: Path | str, parent: Path | str) -> bool:
        try:
            Path(child).resolve().relative_to(Path(parent).resolve())
            return True
        except Exception:
            return False

    def _set_line_edit_state(self, widget: Optional[QLineEdit], *, ok: bool) -> None:
        if widget is None:
            return
        if ok:
            widget.setStyleSheet(
                "QLineEdit { border: 1px solid #91d2ad; background: #f5fbf7; color: #14532d; }"
            )
        else:
            widget.setStyleSheet(
                "QLineEdit { border: 1px solid #f0b4ad; background: #fff5f4; color: #912018; }"
            )

    def _set_status_label(self, label: Optional[QLabel], *, ok: bool, text: str) -> None:
        if label is None:
            return
        if ok:
            label.setStyleSheet(
                "QLabel { color: #166534; background: #eef9f1; border: 1px solid #b7dfc6; "
                "border-radius: 12px; padding: 6px 10px; font-weight: 600; }"
            )
        else:
            label.setStyleSheet(
                "QLabel { color: #b42318; background: #fef3f2; border: 1px solid #f0b4ad; "
                "border-radius: 12px; padding: 6px 10px; font-weight: 600; }"
            )
        label.setText(text)
        label.setToolTip(text)

    def _collect_local_ipv4_addresses(self) -> list[str]:
        found: set[str] = {"127.0.0.1"}
        try:
            hostname = socket.gethostname()
            _, _, addrs = socket.gethostbyname_ex(hostname)
            for addr in addrs:
                if addr:
                    found.add(str(addr))
        except Exception:
            pass
        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.connect(("8.8.8.8", 80))
            local_ip = sock.getsockname()[0]
            if local_ip:
                found.add(str(local_ip))
        except Exception:
            pass
        finally:
            if sock is not None:
                try:
                    sock.close()
                except Exception:
                    pass
        return sorted(found)

    def _normalize_network_prefix(self, value: str) -> str:
        text = str(value or "").strip()
        while text.endswith("."):
            text = text[:-1]
        if not text:
            return ""
        parts = text.split(".")
        if len(parts) > 4:
            return ""
        normalized: list[str] = []
        for part in parts:
            if not part.isdigit():
                return ""
            try:
                number = int(part)
            except Exception:
                return ""
            if number < 0 or number > 255:
                return ""
            normalized.append(str(number))
        return ".".join(normalized)

    def _validate_path_settings(self) -> dict:
        info = db_paths.get_info()
        info_path = db_paths.INFO_PATH
        problems: list[str] = []

        db_root_text = str(info.get("db_root") or self._current_db_root or "").strip()
        base_root_text = str(info.get("base_root") or "").strip()
        scenario_dir_text = str(info.get("scenario_dir") or "").strip()
        source = str(info.get("source") or "").strip()

        if not info_path.exists():
            problems.append(f"current_scenario.json 없음: {info_path}")

        if not db_root_text:
            problems.append("DB 경로가 비어 있습니다.")
        else:
            ok, detail = self._can_prepare_directory(Path(db_root_text))
            if not ok:
                problems.append(f"DB 경로 확인 필요: {detail}")

        if not base_root_text:
            problems.append("Scenario base 경로가 비어 있습니다.")
        else:
            ok, detail = self._can_prepare_directory(Path(base_root_text))
            if not ok:
                problems.append(f"Scenario base 확인 필요: {detail}")

        if source == "scenario":
            if not scenario_dir_text:
                problems.append("current_scenario.json에 scenario_dir가 없습니다.")
            elif base_root_text and not self._is_subpath(scenario_dir_text, base_root_text):
                problems.append("scenario_dir가 base_root 밖에 있습니다.")
            if scenario_dir_text and db_root_text and not self._is_subpath(db_root_text, scenario_dir_text):
                problems.append("db_root가 scenario_dir 밖에 있습니다.")

        ok = not problems
        if ok:
            detail = f"경로 확인 완료: {db_root_text}"
        else:
            detail = "경로 확인 필요: " + " / ".join(problems[:3])

        self._set_line_edit_state(self._db_path_line, ok=ok)
        self._set_status_label(self._path_validation_label, ok=ok, text=detail)
        return {
            "ok": ok,
            "message": detail,
            "details": problems,
            "db_root": db_root_text,
            "base_root": base_root_text,
        }

    def _validate_middleware_settings(self) -> dict:
        problems: list[str] = []
        name = (self._mw_name.text() if self._mw_name is not None else "") or ""
        raw_prefix = (self._mw_addr.text() if self._mw_addr is not None else "") or ""
        prefix = self._normalize_network_prefix(raw_prefix)
        local_text = (self._mw_local.text() if self._mw_local is not None else "") or ""
        external_text = (self._mw_external.text() if self._mw_external is not None else "") or ""

        if not name.strip():
            problems.append("Middleware Name이 비어 있습니다.")
        if not prefix:
            problems.append("NetworkAddress 형식이 올바르지 않습니다. 예: 192.168.100")
        try:
            int(local_text.strip())
        except Exception:
            problems.append("LocalDomain이 숫자가 아닙니다.")
        try:
            int(external_text.strip())
        except Exception:
            problems.append("ExternalDomain이 숫자가 아닙니다.")

        local_ips = self._collect_local_ipv4_addresses()
        non_loopback = [ip for ip in local_ips if not ip.startswith("127.")]
        candidate_ips = non_loopback or local_ips
        if prefix and candidate_ips and not any(ip.startswith(prefix) for ip in candidate_ips):
            shown = ", ".join(candidate_ips[:3])
            problems.append(f"현재 PC IPv4({shown})와 NetworkAddress({prefix})가 맞지 않습니다.")

        ok = not problems
        if ok:
            detail = f"IP 확인 완료: {prefix} / local {', '.join(candidate_ips[:2])}"
        else:
            detail = "IP 확인 필요: " + " / ".join(problems[:3])

        for field in (self._mw_name, self._mw_addr, self._mw_local, self._mw_external):
            self._set_line_edit_state(field, ok=ok)
        self._set_status_label(self._middleware_status_label, ok=ok, text=detail)
        return {
            "ok": ok,
            "message": detail,
            "details": problems,
            "prefix": prefix or raw_prefix,
            "local_ips": candidate_ips,
        }

    def validate_launch_prerequisites(self, *, show_message: bool = False, context: str = "run.py 실행") -> dict:
        path_state = self._validate_path_settings()
        middleware_state = self._validate_middleware_settings()
        ok = bool(path_state.get("ok")) and bool(middleware_state.get("ok"))

        if ok:
            summary = "경로/IP 확인 완료"
        else:
            summary = "경로/IP 확인 필요"

        if show_message and not ok:
            lines = [
                f"{context} 전에 설정 확인이 필요합니다.",
                "",
                f"- 경로: {path_state.get('message')}",
                f"- IP: {middleware_state.get('message')}",
                "",
                "색이 바뀐 항목을 확인한 뒤 다시 실행하세요.",
            ]
            QMessageBox.warning(self, "실행 전 확인 필요", "\n".join(lines))

        return {
            "ok": ok,
            "summary": summary,
            "path": path_state,
            "middleware": middleware_state,
        }


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
            show_console = should_show_module_consoles()
            env = os.environ.copy()
            env["PYTHONUNBUFFERED"] = "1"
            env["KU_CONSOLE_TITLE"] = f"KU {script.stem} Console"
            subprocess.Popen(
                [preferred_console_python(sys.executable), str(script), *extra_args],
                cwd=str(root),
                shell=False,
                env=env,
                creationflags=creationflags_for_subprocess(show_console=show_console),
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

        validation = self.validate_launch_prerequisites(show_message=True, context="모듈 실행")
        if not validation.get("ok"):
            self._log_to_modules("[RUN BLOCKED] 경로/IP 확인 필요")
            return

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
            env["PYTHONUNBUFFERED"] = "1"
            title_map = {
                "mission": "KU Mission Planning Console",
                "monitor": "KU Monitoring Console",
                "decision": "KU Decision Support Console",
                "info": "KU Info Manage Console",
                "integration": "KU Integration Console",
            }
            env["KU_CONSOLE_TITLE"] = title_map.get(role, f"KU {role} Console")
            self._debug_log(f'_launch_role resolved script={script}')
            show_console = should_show_module_consoles()
            proc = subprocess.Popen([preferred_console_python(sys.executable), str(script), *extra_args], cwd=str(root),
                                    shell=False,
                                    env=env,
                                    creationflags=creationflags_for_subprocess(show_console=show_console))
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
            self._apply_middleware()
            show_console = should_show_module_consoles()
            env = os.environ.copy()
            env["PYTHONUNBUFFERED"] = "1"
            env["KU_CONSOLE_TITLE"] = "KU Simulation Console"
            try:
                from modules.sim.config import resolve_server_binding

                host, port = resolve_server_binding()
            except Exception:
                host, port = "0.0.0.0", 8000
            env["SIM_SERVER_HOST"] = str(host)
            env["SIM_SERVER_PORT"] = str(port)
            proc = subprocess.Popen(
                [preferred_console_python(sys.executable), str(script)],
                cwd=str(root),
                env=env,
                creationflags=creationflags_for_subprocess(show_console=show_console),
            )
            try:
                import webbrowser
                url = f"http://127.0.0.1:{int(port)}/"
                webbrowser.open(url, new=2)
            except Exception:
                pass
            self._role_processes["sim"] = proc
            self._log_to_modules(f"[RUN] Simulation launched ({host}:{port})")
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
            body.setSpacing(14)
            body.setContentsMargins(18, 16, 18, 16)

            self.btn_module_shutdown = QPushButton("모듈 종료", placeholder)
            self.btn_module_shutdown.setObjectName("BtnModuleShutdown")
            self.btn_module_shutdown.setFixedHeight(40)
            self.btn_module_shutdown.clicked.connect(self._handle_module_shutdown)
            body.addWidget(self.btn_module_shutdown)

            self.btn_integration_module = QPushButton("통합모듈 실행", placeholder)
            self.btn_integration_module.setObjectName("BtnIntegrationModule")
            self.btn_integration_module.setFixedHeight(40)
            self.btn_integration_module.clicked.connect(lambda: self._launch_role("integration"))
            body.addWidget(self.btn_integration_module)
            self.btn_integration_module.hide()

            self.btn_simulation_run = QPushButton("Simulation \uc2e4\ud589", placeholder)
            self.btn_simulation_run.setObjectName("BtnSimulationRun")
            self.btn_simulation_run.setFixedHeight(40)
            self.btn_simulation_run.clicked.connect(self._launch_simulation)
            body.addWidget(self.btn_simulation_run)

            self.btn_overwrite_020x = QPushButton("0201/0203 덮어쓰기", placeholder)
            self.btn_overwrite_020x.setObjectName("BtnOverwrite020x")
            self.btn_overwrite_020x.setFixedHeight(40)
            self.btn_overwrite_020x.clicked.connect(self._handle_overwrite_020x)
            body.addWidget(self.btn_overwrite_020x)


            self.btn_decision_reset = QPushButton("의사결정 SW 초기화", placeholder)
            self.btn_decision_reset.setObjectName("BtnDecisionReset")
            self.btn_decision_reset.setFixedHeight(40)
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
        validation = self.validate_launch_prerequisites(show_message=True, context="전체 실행")
        if not validation.get("ok"):
            self._log_to_modules('[AUTO BLOCKED] 경로/IP 확인 필요')
            return
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
        self.validate_launch_prerequisites(show_message=False)

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
        self.validate_launch_prerequisites(show_message=False)

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

