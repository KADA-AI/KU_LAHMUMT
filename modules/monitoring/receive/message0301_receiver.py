# modules/monitoring/receive/message0301_receiver.py
import traceback
from typing import List
from System.Collections.Generic import List as CSharpList

# C# 연동 관련 클래스
from dll_files.nFusionImports import IFusionReceive, IsLocal, IsSingletone

# C# 타입을 실제 네임스페이스에서 임포트합니다.
from nFusion.Model.msg_0301 import MissionPlan, Aircraft

# 로컬 이벤트 버스
from gui.gui_app import notify_to_manager

# 데이터 저장소 및 Python 데이터 모델
from data.receive_storage import ReceiveStorage
from data.message_models import (
    MissionPlanModel,
    AircraftModel,
)

# 대/소문자 안전 접근
_get = lambda obj, *names: next(
    (getattr(obj, n) for n in names if hasattr(obj, n)), None
)

# --- Nested Object Conversion Helpers ---

def _to_aircraft_model_list(cs_list: CSharpList) -> List[AircraftModel]:
    if not cs_list:
        return []
    return [
        AircraftModel(
            aircraftID=_get(item, "aircraftID", "AircraftID"),
            individualMissionPackageID=_get(
                item, "individualMissionPackageID", "IndividualMissionPackageID"
            ),
        )
        for item in cs_list
    ]


class MissionPlanReceiver_0301(IFusionReceive[MissionPlan], IsLocal, IsSingletone):
    """0301 MissionPlan 메시지 수신 리시버"""

    __namespace__ = "MissionPlanReceiver_0301"

    def Receive(self, data: MissionPlan, src):
        try:
            # .NET 객체를 Python 데이터 모델 객체로 변환
            python_data = MissionPlanModel(
                timestamp=_get(data, "timestamp", "Timestamp"),
                missionPlanID=_get(data, "missionPlanID", "MissionPlanID"),
                missionPlanTimestamp=_get(
                    data, "missionPlanTimestamp", "MissionPlanTimestamp"
                ),
                planningTime=_get(data, "planningTime", "PlanningTime"),
                plannerID=_get(data, "plannerID", "PlannerID"),
                inputMissionPackageID=_get(
                    data, "inputMissionPackageID", "InputMissionPackageID"
                ),
                missionReferencePackageID=_get(
                    data, "missionReferencePackageID", "MissionReferencePackageID"
                ),
                aircraftList=_to_aircraft_model_list(
                    _get(data, "aircraftList", "AircraftList")
                ),
            )

            # 데이터를 중앙 저장소에 저장
            ReceiveStorage().set_data("0301", python_data)

            # Manager 및 다른 모듈에 데이터 수신 알림
            notify_to_manager("0301", python_data)

        except Exception as e:
            print(f"[ERROR][Receive-0301] traceback ↓↓↓")
            traceback.print_exc()
            print(f"[ERROR][Receive-0301] Exception: {e}")
