# -*- coding: utf-8 -*-
# main_KU.py – KU용 GUI
# ─────────────────────────────────────────────────────────────
from __future__ import annotations

import sys, os, threading
from pathlib import Path

from PyQt5.QtCore import qInstallMessageHandler, QtMsgType
from PyQt5.QtWidgets import QApplication, QMainWindow, QTabWidget


# ─────────────────────────────────────────────────────────────
# Qt 경고 필터 (기존 유지)
def _qt_silent_handler(mode: QtMsgType, context, message: str):
    """
    Qt가 내부에서 뿌리는 “Cannot queue arguments of type ...” 경고를 무시합니다.
    그 외 QtDebug, QtWarning 등은 그대로 stderr로 출력합니다.
    """
    if "Cannot queue arguments of type" in message:
        return
    # 나머지 메시지는 원래대로 출력
    sys.stderr.write(message + "\n")

qInstallMessageHandler(_qt_silent_handler)


# ─────────────────────────────────────────────────────────────
# 경로 부트스트랩: DS 폴더를 sys.path 최우선, CWD는 프로젝트 루트로 고정
def _bootstrap_paths():
    root = Path(__file__).resolve().parent
    ds_dir = root / "modules" / "decision_support"

    # DS 폴더와 루트를 sys.path 맨 앞에 삽입(존재하지 않으면 건너뜀)
    for p in (ds_dir, root):
        p_str = str(p)
        if p.exists() and p_str not in sys.path:
            sys.path.insert(0, p_str)

    # 라이선스/세팅/어셈블리 탐색을 위해 CWD를 루트로 고정
    try:
        os.chdir(root)
    except Exception:
        pass

    return root, ds_dir

PROJECT_ROOT, DS_DIR = _bootstrap_paths()
# 기존 코드 호환을 위해 ROOT 문자열도 유지
ROOT = str(PROJECT_ROOT)


# ─────────────────────────────────────────────────────────────
# 1) nFusion DLL 로드
from dll_files.nFusionImports import *  # noqa: F401,F403  (FusionNodeIoc, NodeMessenger, clr 등을 노출)


# MessageLibrary(.dll) 절대경로 로드
def _load_msglib():
    """
    msg_files/MessageLibrary(.dll)을 절대경로로 안전하게 로드합니다.
    dll_files.nFusionImports에서 clr을 노출하지 않으면 pythonnet의 clr로 폴백합니다.
    """
    # clr 확보
    _clr = None
    try:
        _clr = globals().get("clr", None)
        if _clr is None:
            from dll_files.nFusionImports import clr as _clr  # type: ignore
    except Exception:
        pass
    if _clr is None:
        import clr as _clr  # type: ignore

    msg_dir = PROJECT_ROOT / "msg_files"
    asm_stem = msg_dir / "MessageLibrary"
    # 경로 두 가지 시도(확장자 유무)
    try:
        _clr.AddReference(str(asm_stem))
    except Exception:
        dll_path = asm_stem.with_suffix(".dll")
        _clr.AddReference(str(dll_path))

# 어셈블리 선 로드(Receiver/Tabs import 전에)
_load_msglib()


# ─────────────────────────────────────────────────────────────
# 2) Receive용 Receiver 클래스들을 import + IoC 등록
from receive import *  # noqa: F401,F403

# FusionNodeIoc.AddConsumerFromAssemblyContainsType(SystemOperationModeReceiver)
FusionNodeIoc.Configure()
NodeMessenger.Initialize("CommonChannel")
# NodeMessenger.RegistAllConsumerFromFusionNodeIoc()
# NodeMessenger.InitAllSubscriberFromAssembly()


# ── 탭 위젯들 ---------------------------------------------------------------
# DS 폴더가 sys.path 최우선이므로, 동일한 모듈명이 루트에도 있어도 DS 쪽이 우선 로드됩니다.
from Tabs.manage_info_tab import ManageInfo

# ─────────────────────────────────────────────────────────────
# 메인 윈도우
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Nex1 Emulator")
        self.resize(1280, 720)

        tabs = QTabWidget()
        nm = NodeMessenger  # 단축

        # MissionMonitoringTab이 내부적으로 CSCManager를 사용하도록 수정되었으므로,
        # main_window에서는 MissionMonitoringTab을 그대로 사용합니다.
        tabs.addTab(ManageInfo(messenger=nm), "정보관리 CSC")
        self.setCentralWidget(tabs)

        # RX 폴링 스레드 (기존 로직 유지)
        threading.Thread(target=self._rx_setup, daemon=True).start()

    def _rx_setup(self):
        NodeMessenger.Initialize("MultiTopicReceiveNode")
        NodeMessenger.RegistAllConsumerFromFusionNodeIoc()
        NodeMessenger.InitAllSubscriberFromAssembly()
        NodeMessenger.RegistAllProviderFromFusionNodeIoc()


# ─────────────────────────────────────────────────────────────
# 엔트리 포인트
if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()

    # NodeMessenger RX 스레드 시작 (main_window에서 관리)
    # _rx_setup 메서드 내에서 NodeMessenger.Initialize("MultiTopicReceiveNode")가 호출됩니다.
    # 이 스레드가 메시지를 수신하면, FusionNodeIoc에 등록된 Consumer를 통해
    # 각 탭의 mark_received 메서드가 호출될 것입니다.
    # (CSCTabBase의 mark_received는 NodeMessenger Consumer와 연결되어야 함)
    sys.exit(app.exec_())



