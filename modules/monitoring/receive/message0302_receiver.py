# modules/monitoring/receive/message0302_receiver.py
import traceback
from typing import List
from System.Collections.Generic import List as CSharpList

# C# 연동 관련 클래스
from dll_files.nFusionImports import IFusionReceive, IsLocal, IsSingletone

# C# 타입을 실제 네임스페이스에서 임포트합니다.
from nFusion.Model.msg_0302 import (
    IndividualMissionPlan,
    IndividualMission,
    RelatedMission,
    IndividualMissionInfo,
)
from nFusion.Model.CommonType import Coordinate, Line, Area

# 로컬 이벤트 버스
from .receive_center import notify_to_manager

# 데이터 저장소 및 Python 데이터 모델
from data.receive_storage import ReceiveStorage
from data.message_models import (
    IndividualMissionPlanModel,
    IndividualMissionModel,
    RelatedMissionModel,
    IndividualMissionInfoModel,
    CoordinateModel,
    LineModel,
    AreaModel,
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

def _to_related_mission_model(cs_obj: RelatedMission) -> RelatedMissionModel:
    if not cs_obj:
        return None
    return RelatedMissionModel(
        relatedMissionType=_get(cs_obj, "relatedMissionType", "RelatedMissionType"),
        inputMissionID=_get(cs_obj, "inputMissionID", "InputMissionID"),
        priorMissionID=_get(cs_obj, "priorMissionID", "PriorMissionID"),
    )

def _to_individual_mission_info_model(
    cs_obj: IndividualMissionInfo,
) -> IndividualMissionInfoModel:
    if not cs_obj:
        return None
    return IndividualMissionInfoModel(
        individualMissionType=_get(
            cs_obj, "individualMissionType", "IndividualMissionType"
        ),
        patternType=_get(cs_obj, "patternType", "PatternType"),
        autoZoomIn=_get(cs_obj, "autoZoomIn", "AutoZoomIn"),
        coordinateList=_to_coordinate_model_list(
            _get(cs_obj, "coordinateList", "CoordinateList")
        ),
        lineList=_to_line_model_list(_get(cs_obj, "lineList", "LineList")),
        areaList=_to_area_model_list(_get(cs_obj, "areaList", "AreaList")),
        targetID=_get(cs_obj, "targetID", "TargetID"),
    )

def _to_individual_mission_model_list(cs_list: CSharpList) -> List[IndividualMissionModel]:
    if not cs_list:
        return []
    return [
        IndividualMissionModel(
            individualMissionID=_get(item, "individualMissionID", "IndividualMissionID"),
            isDone=_get(item, "isDone", "IsDone"),
            relatedMission=_to_related_mission_model(
                _get(item, "relatedMission", "RelatedMission")
            ),
            individualMissionInfo=_to_individual_mission_info_model(
                _get(item, "individualMissionInfo", "IndividualMissionInfo")
            ),
            pathID=_get(item, "pathID", "PathID"),
        )
        for item in cs_list
    ]


class IndividualMissionPlanReceiver_0302(
    IFusionReceive[IndividualMissionPlan], IsLocal, IsSingletone
):
    """0302 IndividualMissionPlan 메시지 수신 리시버"""

    __namespace__ = "IndividualMissionPlanReceiver_0302"

    def Receive(self, data: IndividualMissionPlan, src):
        try:
            # .NET 객체를 Python 데이터 모델 객체로 변환
            python_data = IndividualMissionPlanModel(
                timestamp=_get(data, "timestamp", "Timestamp"),
                individualMissionPackageID=_get(
                    data, "individualMissionPackageID", "IndividualMissionPackageID"
                ),
                aircraftID=_get(data, "aircraftID", "AircraftID"),
                individualMissionList=_to_individual_mission_model_list(
                    _get(data, "individualMissionList", "IndividualMissionList")
                ),
            )

            # 데이터를 중앙 저장소에 저장
            ReceiveStorage().set_data("0302", python_data)

            # Manager 및 다른 모듈에 데이터 수신 알림
            notify_to_manager("0302", python_data)

        except Exception as e:
            print(f"[ERROR][Receive-0302] traceback ↓↓↓")
            traceback.print_exc()
            print(f"[ERROR][Receive-0302] Exception: {e}")