# dummy_message_generator.py
# 더미 메시지 생성을 위한 스크립트입니다.

import os, time
import sys
import random
from datetime import datetime, timezone

try:
    from pythonnet import load

    load("coreclr")
    import clr
except ImportError:
    print(
        "오류: 'pythonnet' 라이브러리가 설치되지 않았습니다. `pip install pythonnet`으로 설치하세요."
    )
    sys.exit(1)

# --- DLL 파일 경로 설정 및 C# 라이브러리 로드 ---
try:
    current_dir = os.path.dirname(__file__)
    modules_dir = os.path.abspath(os.path.join(current_dir, ".."))
    common_dir = os.path.join(modules_dir, "common")
    dll_path = os.path.abspath(os.path.join(common_dir, "dll_files"))
    msg_lib_path = os.path.abspath(
        os.path.join(common_dir, "msg_files", "MessageLibrary.dll")
    )

    if not os.path.isdir(dll_path):
        raise FileNotFoundError(f"dll_files 디렉토리를 찾을 수 없습니다: {dll_path}")
    if not os.path.exists(msg_lib_path):
        raise FileNotFoundError(
            f"MessageLibrary.dll을 찾을 수 없습니다: {msg_lib_path}"
        )

    clr.AddReference(os.path.join(dll_path, "nFusion.Interface.Contracts"))
    clr.AddReference(os.path.join(dll_path, "nFusion.Nodes.Core"))
    clr.AddReference(msg_lib_path)

    from nFusion.Nodes.Core import NodeMessenger
    from nFusion.Nodes.Core.Ioc import FusionNodeIoc
    from System import String, UInt64, UInt32

    # 필요한 메시지 모델 임포트 (data/message_models.py 참조)
    from nFusion.Model.msg_0101 import SystemOperationMode  # 예시
    from nFusion.Model.msg_0801 import InitialPlanCommand  # 재계획
    from nFusion.Model.msg_0902 import ReplanRequest  # 재계획
    from nFusion.Model.msg_0502 import EndMissionRequest  # 임무 복귀/종료
    from nFusion.Model.msg_0401 import (
        AgentState,
    )  # 추적 (AgentState 내에 AutoTrackingModel이 있을 수 있음)
    from nFusion.Model.msg_0202 import PriorMissionInfo  # 추적 (선행임무)

except Exception as e:
    print(f"nFusion 라이브러리 로드 중 오류 발생: {e}")
    sys.exit(1)


# --- 헬퍼 함수 ---
def get_current_timestamp():
    return int(time.time() * 1000)


# --- 메시지 전송 함수 ---


def send_tracking_message(uav_id: int, is_tracking: bool):
    """
    무인기 추적 상태 메시지를 보냅니다.
    AgentState 메시지 내에 AutoTrackingModel을 포함하는 방식으로 구현될 수 있습니다.
    """
    print(
        f"UAV {uav_id} 추적 상태 메시지 전송: {'추적 중' if is_tracking else '추적 종료'}"
    )
    # TODO: AutoTrackingModel을 포함하는 AgentState 메시지 생성 및 전송 로직 구현
    # AgentState 메시지 구조를 확인하여 AutoTrackingModel을 어떻게 포함시키는지 파악해야 합니다.
    # 현재는 더미 출력만 합니다.
    pass


def send_return_message(uav_id: int, mission_id: int):
    """
    무인기 임무 복귀 메시지를 보냅니다 (EndMissionRequest 사용).
    """
    print(f"UAV {uav_id} 임무 {mission_id} 복귀 메시지 전송")
    try:
        msg_obj = EndMissionRequest()
        msg_obj.timestamp = get_current_timestamp()
        msg_obj.source = f"UAV_{uav_id}"
        msg_obj.missionID = UInt32(mission_id)  # missionID는 UInt32로 가정

        NodeMessenger.Push[EndMissionRequest](msg_obj)
        print(f"[0502] PUSH 완료: UAV {uav_id} 임무 {mission_id} 복귀")
    except Exception as e:
        print(f"EndMissionRequest 메시지 전송 오류: {e}")


def send_replan_message(reason: str, level: int):
    """
    전체 임무 재계획 메시지를 보냅니다 (ReplanRequest 사용).
    """
    print(f"전체 임무 재계획 메시지 전송: 사유='{reason}', 레벨={level}")
    try:
        msg_obj = ReplanRequest()
        msg_obj.timestamp = get_current_timestamp()
        msg_obj.source = "SYSTEM"
        # ReplanRequestTimeModel은 ReplanRequest 내부에 포함될 수 있습니다.
        # 여기서는 간단히 timestamp를 사용합니다.
        msg_obj.replanRequestTime = (
            get_current_timestamp()
        )  # ReplanRequestTimeModel 대신 직접 timestamp 사용
        msg_obj.replanLevel = UInt32(level)
        msg_obj.replanReason = String(reason)

        NodeMessenger.Push[ReplanRequest](msg_obj)
        print(f"[0902] PUSH 완료: 사유='{reason}', 레벨={level}")
    except Exception as e:
        print(f"ReplanRequest 메시지 전송 오류: {e}")


# --- 메인 실행 로직 ---
print("--- 더미 메시지 생성 스크립트 시작 ---")

MODULE_NAME = "NF.KU_LAHMUMT_MODULE.MONITORING"

try:
    # 1. nFusion 프레임워크 초기화
    FusionNodeIoc.Configure()
    NodeMessenger.Initialize("CommonChannel")
    NodeMessenger.RegistAllConsumerFromFusionNodeIoc()
    NodeMessenger.InitAllSubscriberFromAssembly()
    NodeMessenger.RegistAllProviderFromFusionNodeIoc()

    print(f"NodeMessenger 생성 완료")
    time.sleep(1)

    messages_to_send = [
        # 무인기 1대 추적 전환 (UAV ID, 추적 여부)
        (send_tracking_message, 1, True),
        # 무인기 1대 선행 임무에서 복귀 (UAV ID, 임무 ID)
        (send_return_message, 1, 101),
        # 무인기 전체 임무 재계획 (사유, 레벨)
        (send_replan_message, "운용자 명령", 1),
        (send_tracking_message, 2, True),
        (send_return_message, 2, 102),
        (send_replan_message, "비상 상황 발생", 2),
        (send_tracking_message, 3, False),
        (send_return_message, 3, 103),
    ]

    for i, (func, *args) in enumerate(messages_to_send):
        print(f"\n--- 메시지 {i+1}/8 전송 중 ---")
        func(*args)
        time.sleep(1)  # 메시지 간 1초 대기

except Exception as e:
    print(f"\n오류 발생: {e}")
    sys.exit(1)

print("\n스크립트 실행 완료.")
