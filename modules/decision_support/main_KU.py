# -*- coding: utf-8 -*-
# main_KU.py – KU용 GUI (modules/decision_support 전용)
from __future__ import annotations

import sys, os, threading, json, re
from pathlib import Path
from PyQt5.QtCore import qInstallMessageHandler, QtMsgType
from PyQt5.QtWidgets import QApplication, QMainWindow, QTabWidget

# ───────── Qt 경고 필터 ─────────
def _qt_silent_handler(mode: QtMsgType, context, message: str):
    if "Cannot queue arguments of type" in message:
        return
    sys.stderr.write(message + "\n")

qInstallMessageHandler(_qt_silent_handler)

# ───────── 경로 부트스트랩 ─────────
def _bootstrap_paths():
    ds_dir = Path(__file__).resolve().parent               # .../modules/decision_support
    modules_dir = ds_dir.parent                            # .../modules
    root = modules_dir.parent                              # .../ <project root>
    common_dir = modules_dir / "common"

    for p in (ds_dir, common_dir, root):
        p_str = str(p)
        if p.exists() and p_str not in sys.path:
            sys.path.insert(0, p_str)

    # CWD를 루트로 고정(설정/라이선스/어셈블리 탐색 안정화)
    try:
        os.chdir(root)
    except Exception:
        pass

    return root, ds_dir, common_dir

PROJECT_ROOT, DS_DIR, COMMON_DIR = _bootstrap_paths()
ROOT = str(PROJECT_ROOT)  # 호환용

# ───────── 설정/라이선스 정규화 + 검증 ─────────
def _ensure_fusion_configs():
    """
    설정/라이선스 파일을 루트로 정규화만 수행 (IP 형식 검증 없음).
    """
    from pathlib import Path

    # settings
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

    # license
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


# ───────── nFusion DLL/어셈블리 로드 ─────────
from dll_files.nFusionImports import *  # FusionNodeIoc, NodeMessenger, clr 등 내보냄

def _load_msglib_and_deps():
    """
    modules/common/msg_files/MessageLibrary(.dll) 및 의존 DLL을 절대경로로 로드
    """
    # clr 확보
    _clr = globals().get("clr", None)
    if _clr is None:
        try:
            from dll_files.nFusionImports import clr as _clr  # type: ignore
        except Exception:
            import clr as _clr  # type: ignore

    msg_dir = COMMON_DIR / "msg_files"
    asm_stem = msg_dir / "MessageLibrary"
    try:
        _clr.AddReference(str(asm_stem))  # 확장자 없이 우선
    except Exception:
        _clr.AddReference(str(asm_stem.with_suffix(".dll")))

    # 의존 DLL 사전 로드(환경에 따라 필요)
    for stem in ["K4586Model", "K4586Model.Assist", "MiscUtil"]:
        dll = msg_dir / f"{stem}.dll"
        if dll.exists():
            try:
                _clr.AddReference(str(dll.with_suffix("")))
            except Exception:
                try:
                    _clr.AddReference(str(dll))
                except Exception:
                    pass

# ───────── 설정 보정 → 어셈블리 로드 → Receiver 등록 ─────────
_settings_path = _ensure_fusion_configs()
_load_msglib_and_deps()

from receive import *  # modules/common/receive 에서 import

FusionNodeIoc.Configure()
NodeMessenger.Initialize("CommonChannel")
# 필요시 아래 주석 해제
# NodeMessenger.RegistAllConsumerFromFusionNodeIoc()
# NodeMessenger.InitAllSubscriberFromAssembly()

# ───────── 탭 임포트(공용 Tabs) ─────────
from Tabs.assignment_planning_tab import AssignmentPlanningTab
from Tabs.mission_monitoring_tab import MissionMonitoringTab
from Tabs.decision_support_tab import DecisionSupportTab

# ───────── 메인 윈도우 ─────────
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("건국대 Emulator (KU)")
        self.resize(1280, 720)

        tabs = QTabWidget()
        nm = NodeMessenger

        tabs.addTab(MissionMonitoringTab(messenger=nm), "임무 모니터링·판단 CSC")
        tabs.addTab(AssignmentPlanningTab(messenger=nm), "업무 할당·계획수립 CSC")
        tabs.addTab(DecisionSupportTab(messenger=nm), "의사결정 지원 CSC")
        self.setCentralWidget(tabs)

        threading.Thread(target=self._rx_setup, daemon=True).start()

    def _rx_setup(self):
        NodeMessenger.Initialize("MultiTopicReceiveNode")
        NodeMessenger.RegistAllConsumerFromFusionNodeIoc()
        NodeMessenger.InitAllSubscriberFromAssembly()
        NodeMessenger.RegistAllProviderFromFusionNodeIoc()

# ───────── 엔트리 ─────────
if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
