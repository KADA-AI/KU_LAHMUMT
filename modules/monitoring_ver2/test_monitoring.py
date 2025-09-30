# test_monitoring.py: 모니터링 모듈의 메인 실행 파일 및 테스트 진입점

import sys
import os
import threading
import time

# --- Import nFusionImports to handle CLR and DLL loading ---
from modules.common.dll_files.nFusionImports import *

# --- C# 어셈블리 로드 (handled by nFusionImports) ---
dll_folder_path = ""  # 에러 메시지 출력을 위해 미리 선언
try:
    # 현재 스크립트(__file__)의 위치를 기준으로 프로젝트 루트 폴더 경로를 계산합니다.
    script_dir = os.path.abspath(os.path.dirname(__file__))
    repo_root = os.path.abspath(os.path.join(script_dir, "..", ".."))

    for candidate in (script_dir, repo_root):
        if candidate not in sys.path:
            sys.path.append(candidate)

    project_root = script_dir
    common_dir = os.path.abspath(os.path.join(project_root, "..", "common"))

    # C# DLL 파일이 포함된 폴더들의 경로를 sys.path에 추가합니다.
    msg_dll_folder_path = os.path.join(common_dir, "msg_files")
    if msg_dll_folder_path not in sys.path:
        sys.path.append(msg_dll_folder_path)

    framework_dll_folder_path = os.path.join(common_dir, "dll_files")
    if framework_dll_folder_path not in sys.path:
        sys.path.append(framework_dll_folder_path)

    # dll_files 폴더에 있는 모든 종속성 DLL은 nFusionImports.py에서 처리됩니다.
    # dlls_to_load = [ ... ]
    # for dll in dlls_to_load: clr.AddReference(dll)

    print(f"성공: C# 어셈블리 로딩 완료.")

except Exception as e:
    print(f"치명적 오류: .NET 어셈블리 로딩에 실패했습니다: {e}")
    print(f"에러: {e}")
    print(
        f"확인 사항: DLL 폴더 경로가 올바른지, 'pythonnet'이 설치되었는지 확인하세요."
    )
    sys.exit(1)  # 에러 발생 시 프로그램 즉시 종료


# --- 파이썬 모듈 임포트 ---
from nFusion.Nodes.Core import NodeMessenger
from nFusion.Nodes.Core.Ioc import FusionNodeIoc

from manager import (
    MonitoringManager,
)  # 추가: MonitoringManager 임포트
from config import RECEIVE_MESSAGES  # 추가: RECEIVE_MESSAGES 임포트

# GUI 실행 로직은 gui/gui_app.py에 있습니다.
from gui.gui_app import start_gui as run_gui_application

from modules.common.receive import *


# --- nFusion 초기화 및 NodeMessenger 반환 함수 ---
def initialize_nfusion_and_get_messenger(init_completed_event: threading.Event):
    try:

        # nFusion 프레임워크 초기화
        print("INFO: nFusion 통신 컴포넌트 초기화를 시작합니다...")
        FusionNodeIoc.Configure()
        NodeMessenger.Initialize("CommonChannel")
        NodeMessenger.RegistAllConsumerFromFusionNodeIoc()
        NodeMessenger.InitAllSubscriberFromAssembly()
        # NodeMessenger.RegistAllProviderFromFusionNodeIoc()
        print("INFO: nFusion 통신 컴포넌트 초기화 완료.")
        # 초기화 완료 신호 전송
        init_completed_event.set()

    except Exception as e:
        print(f"치명적 오류: nFusion 초기화에 실패했습니다: {e}")
        sys.exit(1)


# --- 메인 실행 함수 ---
def main():
    print("--- 모니터링 모듈 메인 실행 ---" )

    # 1. nFusion 통신 컴포넌트 초기화
    nfusion_init_completed = threading.Event()
    init_thread = threading.Thread(
        target=initialize_nfusion_and_get_messenger, 
        args=(nfusion_init_completed,),
        daemon=True
    )
    init_thread.start()

    # nFusion 초기화가 완료될 때까지 최대 10초 대기
    print("INFO: nFusion 초기화를 기다리는 중...")
    initialized = nfusion_init_completed.wait(timeout=10.0)
    if not initialized:
        print("치명적 오류: nFusion 초기화가 시간 내에 완료되지 않았습니다.")
        sys.exit(1)

    # 2. 중앙 관리자(MonitoringManager) 생성
    manager = MonitoringManager(
        node_messenger=NodeMessenger, receive_messages_config=RECEIVE_MESSAGES
    )

    # 3. GUI 실행 여부 판단
    if "--gui" in sys.argv:
        print("INFO: GUI 모드로 애플리케이션을 시작합니다.")
        # GUI 실행 함수 호출
        run_gui_application(manager)  # manager 인스턴스를 GUI에 전달
    else:
        print("INFO: 콘솔 모드로 애플리케이션을 시작합니다. (GUI 실행 안 함)")
        print("INFO: GUI를 실행하려면 'python test_monitoring.py --gui' 로 실행하세요.")
        # 무한 루프를 돌며 Ctrl+C 입력을 기다립니다.
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nINFO: 어플리케이션 종료 중...")
            manager.shutdown()
            print("INFO: 모든 리소스가 정리되었습니다.")


if __name__ == "__main__":
    main()
