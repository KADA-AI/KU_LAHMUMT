# modules/monitoring/receive/message0903_receiver.py
import traceback

# C# 연동 관련 클래스
from dll_files.nFusionImports import IFusionReceive, IsLocal, IsSingletone

# C# RequestRenewMission 타입을 실제 네임스페이스에서 임포트합니다.
from nFusion.Model.msg_0903 import RequestRenewMission

# 로컬 이벤트 버스
from .receive_center import notify_to_manager

# 데이터 저장소 및 Python 데이터 모델
from data.receive_storage import ReceiveStorage
from data.message_models import PerformanceMissionUpdateCommandModel

# 대/소문자 안전 접근
_get = lambda obj, *names: next(
    (getattr(obj, n) for n in names if hasattr(obj, n)), None
)


class RequestRenewMissionReceiver_0903(IFusionReceive[RequestRenewMission], IsLocal, IsSingletone):
    """0903 RequestRenewMission 메시지 수신 리시버"""

    __namespace__ = "RequestRenewMissionReceiver_0903"

    def Receive(self, data: RequestRenewMission, src):
        try:
            # .NET 객체를 Python 데이터 모델 객체로 변환
            python_data = PerformanceMissionUpdateCommandModel(
                timestamp=_get(data, "timestamp", "Timestamp"),
                source=_get(data, "source", "Source", "sourceModuleName", "SourceModuleName"),
                missionPlanID=_get(data, "missionPlanID", "MissionPlanID"),
            )

            # 데이터를 중앙 저장소에 저장
            ReceiveStorage().set_data("0903", python_data)

            # Manager 및 다른 모듈에 데이터 수신 알림
            notify_to_manager("0903", python_data)

        except Exception as e:
            print(f"[ERROR][Receive-0903] traceback ↓↓↓")
            traceback.print_exc()
            print(f"[ERROR][Receive-0903] Exception: {e}")