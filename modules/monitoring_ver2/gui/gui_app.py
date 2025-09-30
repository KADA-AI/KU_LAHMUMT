# gui/gui_app.py: GUI 애플리케이션 실행 로직

import sys
import os

# --- 파이썬 모듈 임포트 ---
# 경로가 변경되었으므로 상위 폴더(..)에서 임포트합니다.
from PyQt5.QtWidgets import QApplication
from config import RECEIVE_MESSAGES

# from manager import MonitoringManager # MonitoringManager는 인자로 받음
from gui.monitoring_gui import MainWindow
from receive.receive_center import SignalEmitter, set_global_signal_emitter


def start_gui(manager_instance):
    """
    모니터링 모듈의 메인 실행 함수.
    GUI, 데이터/로직, 통신 컴포넌트를 초기화하고 애플리케이션을 실행합니다.
    """
    # 0. Qt 애플리케이션 생성
    app = QApplication(sys.argv)

    # 1. 전역 시그널 이미터 설정
    global_emitter = SignalEmitter()
    set_global_signal_emitter(global_emitter)

    # 2. 시그널을 매니저의 핸들러에 연결
    global_emitter.message_received.connect(manager_instance.handle_message_reception)

    # 3. GUI 초기화 및 실행
    main_window = MainWindow(manager_instance)
    manager_instance.log_callback = main_window.add_log_message
    manager_instance.gui_update_callback = main_window.update_view
    print("INFO: GUI 초기화 및 콜백 연결 완료.")

    main_window.show()
    print("INFO: 애플리케이션 시작.")

    sys.exit(app.exec_())

