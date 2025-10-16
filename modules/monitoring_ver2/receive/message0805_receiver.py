# modules/monitoring/receive/message0805_receiver.py
import traceback

# C# 연동 관련 클래스
from dll_files.nFusionImports import IFusionReceive, IsLocal, IsSingletone

# C# SystemEvent 타입을 실제 네임스페이스에서 임포트합니다.
from nFusion.Model.msg_0805 import SystemEvent

# 로컬 이벤트 버스
from .receive_center import notify_to_manager

# 데이터 저장소 및 Python 데이터 모델
from data.receive_storage import ReceiveStorage
from data.message_models import OperationEventModel

# 대/소문자 안전 접근
_get = lambda obj, *names: next(
    (getattr(obj, n) for n in names if hasattr(obj, n)), None
)


class SystemEventReceiver_0805(IFusionReceive[SystemEvent], IsLocal, IsSingletone):
    """0805 SystemEvent 메시지 수신 리시버"""

    __namespace__ = "SystemEventReceiver_0805"

    def Receive(self, data: SystemEvent, src):
        try:
            # .NET 객체를 Python 데이터 모델 객체로 변환
            python_data = OperationEventModel(
                timestamp=_get(data, "timestamp", "Timestamp"),
                source=_get(data, "source", "Source", "source", "source"),
                eventType=_get(data, "eventType", "EventType"),
            )

            # 데이터를 중앙 저장소에 저장
            ReceiveStorage().set_data("0805", python_data)

            # Manager 및 다른 모듈에 데이터 수신 알림
            notify_to_manager("0805", python_data)

        except Exception as e:
            print(f"[ERROR][Receive-0805] traceback ↓↓↓")
            traceback.print_exc()
            print(f"[ERROR][Receive-0805] Exception: {e}")