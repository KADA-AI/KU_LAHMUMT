# -*- coding: utf-8 -*-
# main_Nex1.py – Nex1용 GUI (modules/decision_support 전용)
from __future__ import annotations

import sys, os, json, re
from pathlib import Path
from PyQt5.QtCore import qInstallMessageHandler, QtMsgType
from PyQt5.QtWidgets import QApplication, QMainWindow, QTabWidget

def _qt_silent_handler(mode: QtMsgType, context, message: str):
    if "Cannot queue arguments of type" in message:
        return
    sys.stderr.write(message + "\n")

qInstallMessageHandler(_qt_silent_handler)

def _bootstrap_paths():
    ds_dir = Path(__file__).resolve().parent
    modules_dir = ds_dir.parent
    root = modules_dir.parent
    common_dir = modules_dir / "common"

    for p in (ds_dir, common_dir, root):
        p_str = str(p)
        if p.exists() and p_str not in sys.path:
            sys.path.insert(0, p_str)
    try:
        os.chdir(root)
    except Exception:
        pass
    return root, ds_dir, common_dir

PROJECT_ROOT, DS_DIR, COMMON_DIR = _bootstrap_paths()
ROOT = str(PROJECT_ROOT)

def _ensure_fusion_configs():
    """
    설정/라이선스 파일을 루트로 정규화만 수행 (IP 형식 검증 없음).
    """
    from pathlib import Path

    settings_candidates = [
        PROJECT_ROOT / "nFusionSettings.json",
        DS_DIR / "nFusionSettings.json",
        COMMON_DIR / "nFusionSettings.json",
        PROJECT_ROOT / "FusionSettings.json",
        DS_DIR / "FusionSettings.json",
        COMMON_DIR / "FusionSettings.json",
        PROJECT_ROOT / "nFusion" / "FusionSettings.json",
    ]
    settings_src = next((p for p in settings_candidates if p.exists()), None)
    if settings_src is None:
        raise FileNotFoundError("nFusionSettings.json/FusionSettings.json을 찾을 수 없습니다.")

    settings_dst = PROJECT_ROOT / "nFusionSettings.json"
    if settings_src != settings_dst:
        settings_dst.write_text(settings_src.read_text(encoding="utf-8"), encoding="utf-8")

    lic_candidates = [
        PROJECT_ROOT / "nFusionLicense.lic",
        DS_DIR / "nFusionLicense.lic",
        COMMON_DIR / "nFusionLicense.lic",
        PROJECT_ROOT / "nFusion" / "nFusionLicense.lic",
    ]
    lic_src = next((p for p in lic_candidates if p.exists()), None)
    if lic_src is not None:
        lic_dst = PROJECT_ROOT / "nFusionLicense.lic"
        if lic_src != lic_dst:
            lic_dst.write_text(lic_src.read_text(encoding="utf-8"), encoding="utf-8")

    return str(settings_dst)


from dll_files.nFusionImports import *  # FusionNodeIoc, NodeMessenger, clr

def _load_msglib_and_deps():
    _clr = globals().get("clr", None)
    if _clr is None:
        try:
            from dll_files.nFusionImports import clr as _clr
        except Exception:
            import clr as _clr
    msg_dir = COMMON_DIR / "msg_files"
    stem = msg_dir / "MessageLibrary"
    try:
        _clr.AddReference(str(stem))
    except Exception:
        _clr.AddReference(str(stem.with_suffix(".dll")))
    for s in ["K4586Model", "K4586Model.Assist", "MiscUtil"]:
        dll = msg_dir / f"{s}.dll"
        if dll.exists():
            try:
                _clr.AddReference(str(dll.with_suffix("")))
            except Exception:
                try:
                    _clr.AddReference(str(dll))
                except Exception:
                    pass

_settings_path = _ensure_fusion_configs()
_load_msglib_and_deps()

from receive import *  # modules/common/receive

FusionNodeIoc.Configure()
NodeMessenger.Initialize("CommonChannel")

# Nex1는 ManageInfo 단일 탭이라 가정(공용 Tabs 사용)
from Tabs.manage_info_tab import ManageInfo

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Nex1 Emulator")
        self.resize(1000, 700)

        tabs = QTabWidget()
        tabs.addTab(ManageInfo(messenger=NodeMessenger), "정보관리 CSC")
        self.setCentralWidget(tabs)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
