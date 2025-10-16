# modules/monitoring/receive/message0203_receiver.py
import traceback
from typing import List
from System.Collections.Generic import List as CSharpList

# C# 연동 관련 클래스
from dll_files.nFusionImports import IFusionReceive, IsLocal, IsSingletone

# C# 타입을 실제 네임스페이스에서 임포트합니다.
from nFusion.Model.msg_0203 import (
    FlightReferenceInfo,
    TakeOverInfo,
    HandOverInfo,
    RTBCoordinate,
    FlightArea,
    ProhibitedArea,
    AreaLatLon,
    AltitudeLimits,
)
from nFusion.Model.CommonType import Coordinate

# 로컬 이벤트 버스
from .receive_center import notify_to_manager

# 데이터 저장소 및 Python 데이터 모델
from data.receive_storage import ReceiveStorage
from data.message_models import (
    FlightReferenceInfoModel,
    TakeOverInfoModel,
    HandOverInfoModel,
    RTBCoordinateModel,
    FlightAreaModel,
    ProhibitedAreaModel,
    AreaLatLonModel,
    AltitudeLimitsModel,
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

def _to_take_over_info_model_list(cs_list: CSharpList) -> List[TakeOverInfoModel]:
    if not cs_list:
        return []
    return [
        TakeOverInfoModel(
            aircraftID=_get(item, "aircraftID", "AircraftID"),
            coordinate=_to_coordinate_model(_get(item, "coordinate", "Coordinate")),
        )
        for item in cs_list
    ]

def _to_hand_over_info_model_list(cs_list: CSharpList) -> List[HandOverInfoModel]:
    if not cs_list:
        return []
    return [
        HandOverInfoModel(
            aircraftID=_get(item, "aircraftID", "AircraftID"),
            coordinate=_to_coordinate_model(_get(item, "coordinate", "Coordinate")),
        )
        for item in cs_list
    ]

def _to_rtb_coordinate_model_list(cs_list: CSharpList) -> List[RTBCoordinateModel]:
    if not cs_list:
        return []
    return [
        RTBCoordinateModel(
            latitude=_get(item, "latitude", "Latitude"),
            longitude=_get(item, "longitude", "Longitude"),
            altitude=_get(item, "altitude", "Altitude"),
        )
        for item in cs_list
    ]

def _to_area_lat_lon_model_list(cs_list: CSharpList) -> List[AreaLatLonModel]:
    if not cs_list:
        return []
    return [
        AreaLatLonModel(
            latitude=_get(item, "latitude", "Latitude"),
            longitude=_get(item, "longitude", "Longitude"),
        )
        for item in cs_list
    ]

def _to_altitude_limits_model(cs_obj: AltitudeLimits) -> AltitudeLimitsModel:
    if not cs_obj:
        return None
    return AltitudeLimitsModel(
        lowerLimit=_get(cs_obj, "lowerLimit", "LowerLimit"),
        upperLimit=_get(cs_obj, "upperLimit", "UpperLimit"),
    )

def _to_flight_area_model_list(cs_list: CSharpList) -> List[FlightAreaModel]:
    if not cs_list:
        return []
    return [
        FlightAreaModel(
            flightAreaID=_get(item, "flightAreaID", "FlightAreaID"),
            areaLatLonList=_to_area_lat_lon_model_list(
                _get(item, "areaLatLonList", "AreaLatLonList")
            ),
            altitudeLimits=_to_altitude_limits_model(
                _get(item, "altitudeLimits", "AltitudeLimits")
            ),
        )
        for item in cs_list
    ]

def _to_prohibited_area_model_list(cs_list: CSharpList) -> List[ProhibitedAreaModel]:
    if not cs_list:
        return []
    return [
        ProhibitedAreaModel(
            prohibitedAreaID=_get(item, "prohibitedAreaID", "ProhibitedAreaID"),
            areaLatLonList=_to_area_lat_lon_model_list(
                _get(item, "areaLatLonList", "AreaLatLonList")
            ),
            altitudeLimits=_to_altitude_limits_model(
                _get(item, "altitudeLimits", "AltitudeLimits")
            ),
        )
        for item in cs_list
    ]


class FlightReferenceInfoReceiver_0203(
    IFusionReceive[FlightReferenceInfo], IsLocal, IsSingletone
):
    """0203 FlightReferenceInfo 메시지 수신 리시버"""

    __namespace__ = "FlightReferenceInfoReceiver_0203"

    def Receive(self, data: FlightReferenceInfo, src):
        try:
            # .NET 객체를 Python 데이터 모델 객체로 변환
            python_data = FlightReferenceInfoModel(
                timestamp=_get(data, "timestamp", "Timestamp"),
                missionReferencePackageID=_get(
                    data, "missionReferencePackageID", "MissionReferencePackageID"
                ),
                inputTimestamp=_get(data, "inputTimestamp", "InputTimestamp"),
                takeOverInfoList=_to_take_over_info_model_list(
                    _get(data, "takeOverInfoList", "TakeOverInfoList")
                ),
                handOverInfoList=_to_hand_over_info_model_list(
                    _get(data, "handOverInfoList", "HandOverInfoList")
                ),
                rtbCoordinateList=_to_rtb_coordinate_model_list(
                    _get(data, "rtbCoordinateList", "RtbCoordinateList")
                ),
                flightAreaList=_to_flight_area_model_list(
                    _get(data, "flightAreaList", "FlightAreaList")
                ),
                prohibitedAreaList=_to_prohibited_area_model_list(
                    _get(data, "prohibitedAreaList", "ProhibitedAreaList")
                ),
            )

            # 데이터를 중앙 저장소에 저장
            ReceiveStorage().set_data("0203", python_data)

            # Manager 및 다른 모듈에 데이터 수신 알림
            notify_to_manager("0203", python_data)

        except Exception as e:
            print(f"[ERROR][Receive-0203] traceback ↓↓↓")
            traceback.print_exc()
            print(f"[ERROR][Receive-0203] Exception: {e}")