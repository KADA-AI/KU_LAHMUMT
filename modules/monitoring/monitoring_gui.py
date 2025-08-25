# -*- coding: utf-8 -*- 
# monitoring_gui.py – 임무 모니터링·판단 전용 GUI
from __future__ import annotations

import sys, os, threading
os.environ["KU_ROLE"] = "monitoring"
from pathlib import Path

from PyQt5.QtCore import qInstallMessageHandler, QtMsgType, pyqtSignal, QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QTabWidget, QShortcut, QVBoxLayout, QPushButton
from PyQt5.QtGui import QKeySequence

# ───────── Qt 경고 필터 ─────────
def _qt_silent_handler(mode: QtMsgType, context, message: str):
    if "Cannot queue arguments of type" in message:
        return
    sys.stderr.write(message + "\n")

qInstallMessageHandler(_qt_silent_handler)

# ───────── 경로 부트스트랩 ─────────
def _bootstrap_paths():
    here = Path(__file__).resolve()
    modules_dir = here.parents[1]                # .../modules
    root = modules_dir.parent                    # .../<project root>
    common_dir = modules_dir / "common"
    # Add monitoring module path to allow relative imports
    monitoring_dir = modules_dir / "monitoring"
    for p in (monitoring_dir, common_dir, root):
        p_str = str(p)
        if p.exists() and p_str not in sys.path:
            sys.path.insert(0, p_str)
    try: os.chdir(root)
    except Exception: pass
    return root, common_dir

PROJECT_ROOT, COMMON_DIR = _bootstrap_paths()

# ───────── 설정/라이선스 정규화 ─────────
def _ensure_fusion_configs():
    cands = [
        PROJECT_ROOT / "nFusionSettings.json",
        COMMON_DIR   / "nFusionSettings.json",
        PROJECT_ROOT / "FusionSettings.json",
        COMMON_DIR   / "FusionSettings.json",
        PROJECT_ROOT / "nFusion" / "FusionSettings.json",
    ]
    src = next((p for p in cands if p.exists()), None)
    if src is None:
        raise FileNotFoundError("nFusionSettings.json/FusionSettings.json 이 없습니다.")
    dst = PROJECT_ROOT / "nFusionSettings.json"
    if src != dst:
        dst.write_text(src.read_text(encoding="utf-8"), encoding="utf-8")

    lcands = [
        PROJECT_ROOT / "nFusionLicense.lic",
        COMMON_DIR   / "nFusionLicense.lic",
        PROJECT_ROOT / "nFusion" / "nFusionLicense.lic",
    ]
    lsrc = next((p for p in lcands if p.exists()), None)
    if lsrc:
        ldst = PROJECT_ROOT / "nFusionLicense.lic"
        if lsrc != ldst:
            ldst.write_text(lsrc.read_text(encoding="utf-8"), encoding="utf-8")
    return str(dst)

# ───────── nFusion DLL/어셈블리 로드 ─────────
from dll_files.nFusionImports import *  # FusionNodeIoc, NodeMessenger, clr

def _load_msglib_and_deps():
    _clr = globals().get("clr", None)
    if _clr is None:
        try:
            from dll_files.nFusionImports import clr as _clr  # type: ignore
        except Exception:
            import clr as _clr  # type: ignore
    msg_dir = COMMON_DIR / "msg_files"
    stem = msg_dir / "MessageLibrary"
    try: _clr.AddReference(str(stem))
    except Exception: _clr.AddReference(str(stem.with_suffix(".dll")))

_settings_path = _ensure_fusion_configs()
_ = _load_msglib_and_deps()

from modules.common.Tabs.csc_tab_base import CSCTabBase
from modules.monitoring.monitoring_config import PUSH_MESSAGES, RECEIVE_MESSAGES
from modules.monitoring.monitoring_manager import MonitoringManager

# ───────── 모니터링 탭 ─────────
class MonitoringTab(CSCTabBase):
    TITLE = "임무 모니터링·판단 CSC"
    PUSH_MESSAGES = PUSH_MESSAGES
    RECEIVE_MESSAGES = RECEIVE_MESSAGES

    def __init__(self, *, messenger, parent=None):
        super().__init__(messenger=messenger, parent=parent)
        self.manager = MonitoringManager(node_messenger=self.messenger, log_callback=self.log_from_manager)

        # Add a button to trigger the logic
        self.btn_trigger_logic = QPushButton("Run Monitoring Logic")
        self.btn_trigger_logic.clicked.connect(self.manager.trigger_logic)

        # Add the button to the layout
        # A bit of a hack to add it to the right side layout
        right_layout = self.layout().itemAt(1).layout().itemAt(1).widget().layout()
        right_layout.addWidget(self.btn_trigger_logic)

    def log_from_manager(self, tag, log_type, message, raw_data):
        log_message = f"[{tag}] [{log_type}] {message}"
        self._write_log(self.log_rx, "MANAGER", "LOG", log_message.encode('utf-8'))

# ───────── 메인 윈도우 ─────────
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("임무 모니터링·판단 GUI")
        self.resize(1100, 700)

        tabs = QTabWidget()
        self._tab = MonitoringTab(messenger=NodeMessenger)
        tabs.addTab(self._tab, "임무 모니터링·판단 CSC")
        self.setCentralWidget(tabs)

        threading.Thread(target=self._rx_setup, daemon=True).start()

    def _rx_setup(self):
        FusionNodeIoc.Configure()
        NodeMessenger.Initialize("MultiTopicReceiveNode")
        NodeMessenger.RegistAllConsumerFromFusionNodeIoc()
        NodeMessenger.InitAllSubscriberFromAssembly()
        NodeMessenger.RegistAllProviderFromFusionNodeIoc()

# ───────── 엔트리 ─────────
if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow(); win.show()
    sys.exit(app.exec_())