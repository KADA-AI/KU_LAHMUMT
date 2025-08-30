# c:/Users/HJW/Documents/Dev/MUMT/KU_LAHMUMT/modules/monitoring/__init__.py

import sys
import os
import clr  # pythonnet 라이브러리

dll_folder_path = ""  # 에러 메시지 출력을 위해 미리 선언
try:
    # 현재 스크립트(__file__)의 위치를 기준으로 프로젝트 루트 폴더 경로를 계산합니다.
    # test_monitoring.py -> monitoring -> modules -> KU_LAHMUMT (프로젝트 루트)
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__)))

    # C# DLL 파일이 포함된 실제 폴더 경로를 지정합니다.
    msg_dll_folder_path = os.path.join(project_root, "msg_files")
    if msg_dll_folder_path not in sys.path:
        sys.path.append(msg_dll_folder_path)

    # !!! 중요: nFusion 프레임워크 DLL이 있는 경로 추가 !!!
    framework_dll_folder_path = os.path.join(
        project_root, "dll_files"
    )  # 실제 경로가 다르면 수정 필요
    if framework_dll_folder_path not in sys.path:
        sys.path.append(framework_dll_folder_path)

    # 종속성(Dependency)을 먼저 로드합니다.
    clr.AddReference("nFusion.Interface.Contracts")
    print(f"성공: 종속성 'nFusion.Interface.Contracts.dll'을 로드했습니다.")

    # 사용할 C# 어셈블리(DLL)를 로드합니다. (.dll 확장자는 생략)
    # 이 작업이 성공해야 하위 모듈에서 C# 타입을 import 할 수 있습니다.
    clr.AddReference("MessageLibrary")
    print(
        f"성공: '{os.path.join(msg_dll_folder_path, 'MessageLibrary.dll')}' 어셈블리를 로드했습니다."
    )

except Exception as e:
    print(f"치명적 오류: .NET 어셈블리 로딩에 실패했습니다.")
    print(f"에러: {e}")
    print(
        f"확인 사항: DLL 폴더 경로('{dll_folder_path}')가 올바른지, 'pythonnet'이 설치되었는지 확인하세요."
    )
    sys.exit(1)  # 에러 발생 시 프로그램 즉시 종료

# GUI 라이브러리는 PyQt5라고 가정하겠습니다. 실제 사용하는 라이브러리에 따라 변경될 수 있습니다.
from PyQt5.QtWidgets import QApplication

from config import RECEIVE_MESSAGES
from manager import MonitoringManager
from gui.monitoring_gui import MainWindow


def main():
    """
    모니터링 모듈의 메인 실행 함수.
    GUI, 데이터/로직, 통신 컴포넌트를 초기화하고 애플리케이션을 실행합니다.
    """
    # 0. Qt 애플리케이션 생성 (GUI)
    app = QApplication(sys.argv)

    # 1. 데이터 저장소 및 로직 핸들러 초기화
    # 프로그램 실행 초기에 싱글톤 인스턴스들이 생성되도록 명시적으로 import 합니다.

    print("INFO: 데이터 저장소 및 로직 핸들러 초기화 완료.")

    # 2. 통신 컴포넌트(nFusion) 초기화
    # monitoring_manager가 외부에서 messenger 객체를 주입받으므로, 여기서 생성해서 전달합니다.
    node_messenger = None
    

    # 3. 중앙 관리자(MonitoringManager) 생성
    # 모든 컴포넌트를 연결하는 중앙 관리자를 생성합니다.
    manager = MonitoringManager(
        node_messenger=node_messenger, receive_messages_config=RECEIVE_MESSAGES
    )
    print("INFO: 중앙 관리자(MonitoringManager) 생성 완료.")

    # 4. GUI 초기화 및 실행
    # GUI 메인 윈도우를 생성하고, manager와 연결합니다.
    # MonitoringWindow 클래스 이름은 monitoring_gui.py의 실제 클래스 이름으로 가정합니다.

    main_window = MainWindow(manager)

    # manager의 콜백들을 GUI의 메소드와 연결합니다.
    # (GUI 클래스에 add_log_message와 update_view 메소드가 있다고 가정)
    manager.log_callback = main_window.add_log_message
    manager.gui_update_callback = main_window.update_view  # 데이터 변경 알림 콜백 연결
    print("INFO: GUI 초기화 및 콜백 연결 완료.")

    main_window.show()
    print("INFO: 애플리케이션 시작.")
    sys.exit(app.exec_())


# 이 모듈이 메인으로 실행될 경우(예: python -m modules.monitoring) main 함수를 호출합니다.
if __name__ == "__main__":
    main()
