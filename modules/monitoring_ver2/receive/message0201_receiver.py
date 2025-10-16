# modules/monitoring/receive/message0201_receiver.py
import traceback
from typing import List
from System.Collections.Generic import List as CSharpList

# C# 연동 관련 클래스
from dll_files.nFusionImports import IFusionReceive, IsLocal, IsSingletone

# C# 타입을 실제 네임스페이스에서 임포트합니다.
from nFusion.Model.msg_0201 import (
    InputMissionPlan,
    AvailableAircraft,
    InputMission,
    MissionDetail,
)

# from nFusion.Model.CommonType import Coordinate, Line, Area

# 로컬 이벤트 버스
from .receive_center import notify_to_manager

# 데이터 저장소 및 Python 데이터 모델
from data.receive_storage import ReceiveStorage
from data.message_models import (
    InputMissionPlanModel,
    AvailableAircraftModel,
    InputMissionModel,
    MissionDetailModel,
    LineModel,
    AreaModel,
    CoordinateModel,
)

# 대/소문자 안전 접근
_get = lambda obj, *names: next(
    (getattr(obj, n) for n in names if hasattr(obj, n)), None
)

# --- Nested Object Conversion Helpers ---


def _to_coordinate_model_list(cs_list: CSharpList) -> List[CoordinateModel]:
    if not cs_list:
        return []
    return [
        CoordinateModel(
            latitude=_get(item, "latitude", "Latitude"),
            longitude=_get(item, "longitude", "Longitude"),
            altitude=_get(item, "altitude", "Altitude"),
        )
        for item in cs_list
    ]


def _to_line_model_list(cs_list: CSharpList) -> List[LineModel]:
    if not cs_list:
        return []
    return [
        LineModel(
            width=_get(item, "width", "Width"),
            coordinateList=_to_coordinate_model_list(
                _get(item, "coordinateList", "CoordinateList")
            ),
        )
        for item in cs_list
    ]


def _to_area_model_list(cs_list: CSharpList) -> List[AreaModel]:
    if not cs_list:
        return []
    return [
        AreaModel(
            isHole=_get(item, "isHole", "IsHole"),
            coordinateList=_to_coordinate_model_list(
                _get(item, "coordinateList", "CoordinateList")
            ),
        )
        for item in cs_list
    ]


def _to_mission_detail_model(cs_obj: MissionDetail) -> MissionDetailModel:
    if not cs_obj:
        return None
    return MissionDetailModel(
        coordinateList=_to_coordinate_model_list(
            _get(cs_obj, "coordinateList", "CoordinateList")
        ),
        lineList=_to_line_model_list(_get(cs_obj, "lineList", "LineList")),
        areaList=_to_area_model_list(_get(cs_obj, "areaList", "AreaList")),
    )


def _to_input_mission_model_list(cs_list: CSharpList) -> List[InputMissionModel]:
    if not cs_list:
        return []
    return [
        InputMissionModel(
            inputMissionID=_get(item, "inputMissionID", "InputMissionID"),
            inputMissionType=_get(item, "inputMissionType", "InputMissionType"),
            isDone=_get(item, "isDone", "IsDone"),
            missionDetail=_to_mission_detail_model(
                _get(item, "missionDetail", "MissionDetail")
            ),
        )
        for item in cs_list
    ]


def _to_available_aircraft_model_list(
    cs_list: CSharpList,
) -> List[AvailableAircraftModel]:
    if not cs_list:
        return []
    return [
        AvailableAircraftModel(aircraftID=_get(item, "aircraftID", "AircraftID"))
        for item in cs_list
    ]


class InputMissionPlanReceiver_0201(
    IFusionReceive[InputMissionPlan], IsLocal, IsSingletone
):
    """0201 InputMissionPlan 메시지 수신 리시버"""

    __namespace__ = "InputMissionPlanReceiver_0201"

    def Receive(self, data: InputMissionPlan, src):
        try:
            # .NET 객체를 Python 데이터 모델 객체로 변환
            python_data = InputMissionPlanModel(
                timestamp=_get(data, "timestamp", "Timestamp"),
                inputMissionPackageID=_get(
                    data, "inputMissionPackageID", "InputMissionPackageID"
                ),
                inputMissionPackageType=_get(
                    data, "inputMissionPackageType", "InputMissionPackageType"
                ),
                mainSensor=_get(data, "mainSensor", "MainSensor"),
                availableAircraftList=_to_available_aircraft_model_list(
                    _get(data, "availableAircraftList", "AvailableAircraftList")
                ),
                inputMissionList=_to_input_mission_model_list(
                    _get(data, "inputMissionList", "InputMissionList")
                ),
            )

            # 데이터를 중앙 저장소에 저장
            ReceiveStorage().set_data("0201", python_data)

            # Manager 및 다른 모듈에 데이터 수신 알림
            notify_to_manager("0201", python_data)

        except Exception as e:
            print(f"[ERROR][Receive-0201] traceback ↓↓↓")
            traceback.print_exc()
            print(f"[ERROR][Receive-0201] Exception: {e}")
