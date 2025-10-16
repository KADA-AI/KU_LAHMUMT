import traceback

# C# 연동 관련 클래스
from dll_files.nFusionImports import IFusionReceive, IsLocal, IsSingletone

# C# SystemOperationMode 타입을 실제 네임스페이스에서 임포트합니다.
from nFusion.Model.msg_0101 import SystemOperationMode

# 로컬 이벤트 버스
from gui.gui_app import notify_to_manager

# 데이터 저장소 및 Python 데이터 모델
from data.receive_storage import ReceiveStorage
from data.message_models import SystemOperationModeModel

# 대/소문자 안전 접근
_get = lambda obj, *names: next(
    (getattr(obj, n) for n in names if hasattr(obj, n)), None
)


class SystemOperationModeReceiver_0101(
    IFusionReceive[SystemOperationMode], IsLocal, IsSingletone
):
    """0101 SystemOperationMode 메시지 수신 리시버"""

    __namespace__ = "SystemOperationModeReceiver_0101"

    def Receive(self, data, src):
        try:
            # .NET 객체를 Python 데이터 모델 객체로 변환
            timestamp_val = data.timestamp
            # source_val = data.Source
            systemMode_val = data.systemMode

            python_data = SystemOperationModeModel(
                timestamp=timestamp_val,
                systemMode=systemMode_val,
            )

            # 데이터를 중앙 저장소에 저장
            ReceiveStorage().set_data("0101", python_data)

            # Manager 및 다른 모듈에 데이터 수신 알림 (객체 그대로 전달)
            notify_to_manager("0101", python_data)

        except Exception as e:
            # 에러 발생 시 로그 기록
            # self.manager._log(...) 와 같은 형태로 로거를 사용할 수 있다면 더 좋습니다.
            print("[ERROR][Receive-0101] traceback ↓↓↓")
            traceback.print_exc()
            print(f"[ERROR][Receive-0101] Exception: {e}")
