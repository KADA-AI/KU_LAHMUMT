# send_message_0101.py
# nFusion 메시지 0101 전송을 위한 단일 파일 스크립트입니다.

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
    dll_path = os.path.abspath(os.path.join(current_dir, "dll_files"))
    msg_lib_path = os.path.abspath(
        os.path.join(current_dir, "msg_files", "MessageLibrary.dll")
    )

    if not os.path.isdir(dll_path):
        raise FileNotFoundError(f"dll_files 디렉토리를 찾을 수 없습니다: {dll_path}")
    if not os.path.exists(msg_lib_path):
        raise FileNotFoundError(
            f"MessageLibrary.dll을 찾을 수 없습니다: {msg_lib_path}"
        )

    # !!! 중요: FusionNodeIoc.Configure()가 필요로 하는 추가 DLL들을 여기에 AddReference 해야 합니다. !!!
    # 예: clr.AddReference("K4586Model"), clr.AddReference("MiscUtil") 등

    clr.AddReference(os.path.join(dll_path, "nFusion.Interface.Contracts"))
    clr.AddReference(os.path.join(dll_path, "nFusion.Nodes.Core"))
    clr.AddReference(msg_lib_path)

    from nFusion.Nodes.Core import NodeMessenger
    from nFusion.Nodes.Core.Ioc import FusionNodeIoc
    from nFusion.Model.msg_0101 import SystemOperationMode
    from System import String, UInt64, UInt32  # ulong은 UInt64, uint는 UInt32에 해당

except Exception as e:
    print(f"nFusion 라이브러리 로드 중 오류 발생: {e}")
    sys.exit(1)

# --- 메인 실행 로직 ---
print("--- 0101 메시지 단일 파일 전송 스크립트 ---")

MODULE_NAME = "NF.KU_LAHMUMT_MODULE.MONITORING"
MSG_ID = "0101"

try:
    # 1. nFusion 프레임워크 초기화
    # 이 과정에서 K4586Model.dll, MiscUtil.dll 등 추가 라이브러리를 로드하므로
    # 해당 파일들이 dll_files 폴더에 있는지, 위에 clr.AddReference가 되었는지 확인해야 합니다.
    FusionNodeIoc.Configure()
    NodeMessenger.Initialize("CommonChannel")
    NodeMessenger.RegistAllConsumerFromFusionNodeIoc()
    NodeMessenger.InitAllSubscriberFromAssembly()
    NodeMessenger.RegistAllProviderFromFusionNodeIoc()

    print(f"NodeMessenger 생성 완료")

    # 3. 메시지 객체 생성 및 속성 설정

    while True:
        # C#의 ulong 타입에 맞추기 위해 틱(ticks)을 사용하거나,
        # Unix 에포크 밀리초를 UInt64로 캐스팅하는 것이 일반적입니다.
        # 여기서는 time.time() * 1000을 사용하고 UInt64로 변환합니다.
        # current_timestamp = int(time.time() * 1000)
        # source_id = "DUMMY"
        # system_mode = random.choice([0, 3])

        msg_obj = SystemOperationMode()
        msg_obj.timestamp = int(time.time() * 1000)
        msg_obj.source = "source_id"
        msg_obj.systemMode = random.randint(0, 3)

        print("\n생성된 메시지 객체:")
        print(f"  - Timestamp: {msg_obj.timestamp}")
        print(f"  - Source: {msg_obj.source}")
        print(f"  - SystemMode: {msg_obj.systemMode}")

        # 4. 메시지 전송
        print("\n메시지 전송 시도...")
        NodeMessenger.Push[SystemOperationMode](msg_obj)

        print(f"\n[{MSG_ID}] PUSH 완료")
        time.sleep(3)


except Exception as e:
    print(f"\n오류 발생: {e}")
    sys.exit(1)

print("\n스크립트 실행 완료.")
