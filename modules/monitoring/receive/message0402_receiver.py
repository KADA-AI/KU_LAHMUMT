# modules/monitoring/receive/message0402_receiver.py
import traceback
from typing import List
from System.Collections.Generic import List as CSharpList

# C# 연동 관련 클래스
from dll_files.nFusionImports import IFusionReceive, IsLocal, IsSingletone

# C# 타입을 실제 네임스페이스에서 임포트합니다.
from nFusion.Model.msg_0402 import (
    SituationAwarenessInfo,
    ROIInfo,
    SituationAwarenessInfo,
)
from nFusion.Model.CommonType import Coordinate

# 로컬 이벤트 버스
from .receive_center import notify

# 데이터 저장소 및 Python 데이터 모델
from data.receive_storage import ReceiveStorage
from data.message_models import (
    BattlefieldSituationAwarenessInfoModel,
    ROIInfoModel,
    SituationAwarenessInfoModel,
    CoordinateModel,
)

# 대/소문자 안전 접근
_get = lambda obj, *names: next(
    (getattr(obj, n) for n in names if hasattr(obj, n)), None
)

# --- Nested Object Conversion Helpers ---


def _to_coordinate_model(cs_obj: Coordinate) -> CoordinateModel:
    if not cs_obj:
        return None
    return CoordinateModel(
        latitude=_get(cs_obj, "latitude", "Latitude"),
        longitude=_get(cs_obj, "longitude", "Longitude"),
        altitude=_get(cs_obj, "altitude", "Altitude"),
    )


def _to_roi_info_model_list(cs_list: CSharpList) -> List[ROIInfoModel]:
    if not cs_list:
        return []
    return [
        ROIInfoModel(
            aircraftID=_get(item, "aircraftID", "AircraftID"),
            coordinate=_to_coordinate_model(_get(item, "coordinate", "Coordinate")),
            fov=_get(item, "fov", "Fov"),
        )
        for item in cs_list
    ]


def _to_situation_awareness_info_model_list(
    cs_list: CSharpList,
) -> List[SituationAwarenessInfoModel]:
    if not cs_list:
        return []
    return [
        SituationAwarenessInfoModel(
            aircraftID=_get(item, "aircraftID", "AircraftID"),
            coordinate=_to_coordinate_model(_get(item, "coordinate", "Coordinate")),
            fov=_get(item, "fov", "Fov"),
        )
        for item in cs_list
    ]


class SituationAwarenessInfoReceiver_0402(
    IFusionReceive[SituationAwarenessInfo], IsLocal, IsSingletone
):
    """0402 BattlefieldSituationAwarenessInfo 메시지 수신 리시버"""

    __namespace__ = "BattlefieldSituationAwarenessInfoReceiver_0402"

    def Receive(self, data: SituationAwarenessInfo, src):
        try:
            # .NET 객체를 Python 데이터 모델 객체로 변환
            python_data = BattlefieldSituationAwarenessInfoModel(
                timestamp=_get(data, "timestamp", "Timestamp"),
                source=_get(data, "source", "Source"),
                roiInfoList=_to_roi_info_model_list(
                    _get(data, "roiInfoList", "RoiInfoList")
                ),
                situationAwarenessInfoList=_to_situation_awareness_info_model_list(
                    _get(
                        data,
                        "situationAwarenessInfoList",
                        "SituationAwarenessInfoList",
                    )
                ),
            )

            # 데이터를 중앙 저장소에 저장
            ReceiveStorage().set_data("0402", python_data)

            # Manager 및 다른 모듈에 데이터 수신 알림
            notify("0402", python_data)

        except Exception as e:
            print(f"[ERROR][Receive-0402] traceback ↓↓↓")
            traceback.print_exc()
            print(f"[ERROR][Receive-0402] Exception: {e}")
