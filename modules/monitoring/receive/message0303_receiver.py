# modules/monitoring/receive/message0303_receiver.py
import traceback
from typing import List
from System.Collections.Generic import List as CSharpList

# C# 연동 관련 클래스
from dll_files.nFusionImports import IFusionReceive, IsLocal, IsSingletone

# C# 타입을 실제 네임스페이스에서 임포트합니다.
from nFusion.Model.msg_0303 import (
    UAVFlightPlan,
    Waypoint,
    FilmingProperty,
    LoiterProperty,
)
from nFusion.Model.CommonType import (
    Coordinate,
    CoordinateOrientation,
    AutoScan,
    GimbalYawLimits,
    AircraftFixed,
    AutoTracking,
    LineSearch,
    FormationInfo,
    Formation,
)

# 로컬 이벤트 버스
from .receive_center import notify_to_manager

# 데이터 저장소 및 Python 데이터 모델
from data.receive_storage import ReceiveStorage
from data.message_models import (
    UAVFlightPlanModel,
    WaypointModel,
    FilmingPropertyModel,
    AutoScanModel,
    GimbalYawLimitsModel,
    AircraftFixedModel,
    AutoTrackingModel,
    LineSearchModel,
    LoiterPropertyModel,
    FormationInfoModel,
    FormationModel,
    CoordinateModel,
    CoordinateOrientationModel,
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

def _to_coordinate_model_list(cs_list: CSharpList) -> List[CoordinateModel]:
    if not cs_list:
        return []
    return [_to_coordinate_model(item) for item in cs_list]

def _to_formation_model(cs_obj: Formation) -> FormationModel:
    if not cs_obj:
        return None
    return FormationModel(
        dX=_get(cs_obj, "dX", "DX"),
        dY=_get(cs_obj, "dY", "DY"),
        dZ=_get(cs_obj, "dZ", "DZ"),
    )

def _to_formation_info_model(cs_obj: FormationInfo) -> FormationInfoModel:
    if not cs_obj:
        return None
    return FormationInfoModel(
        leaderAircraftID=_get(cs_obj, "leaderAircraftID", "LeaderAircraftID"),
        formation=_to_formation_model(_get(cs_obj, "formation", "Formation")),
    )

def _to_loiter_property_model(cs_obj: LoiterProperty) -> LoiterPropertyModel:
    if not cs_obj:
        return None
    return LoiterPropertyModel(
        radius=_get(cs_obj, "radius", "Radius"),
        direction=_get(cs_obj, "direction", "Direction"),
        time=_get(cs_obj, "time", "Time"),
        speed=_get(cs_obj, "speed", "Speed"),
    )

def _to_coordinate_orientation_model(
    cs_obj: CoordinateOrientation,
) -> CoordinateOrientationModel:
    if not cs_obj:
        return None
    return CoordinateOrientationModel(
        coordinate=_to_coordinate_model(_get(cs_obj, "coordinate", "Coordinate"))
    )

def _to_line_search_model(cs_obj: LineSearch) -> LineSearchModel:
    if not cs_obj:
        return None
    return LineSearchModel(
        coordinateList=_to_coordinate_model_list(
            _get(cs_obj, "coordinateList", "CoordinateList")
        ),
        searchSpeed=_get(cs_obj, "searchSpeed", "SearchSpeed"),
    )

def _to_auto_tracking_model(cs_obj: AutoTracking) -> AutoTrackingModel:
    if not cs_obj:
        return None
    return AutoTrackingModel(targetID=_get(cs_obj, "targetID", "TargetID"))

def _to_aircraft_fixed_model(cs_obj: AircraftFixed) -> AircraftFixedModel:
    if not cs_obj:
        return None
    return AircraftFixedModel(
        gimbalPitch=_get(cs_obj, "gimbalPitch", "GimbalPitch"),
        gimbalYaw=_get(cs_obj, "gimbalYaw", "GimbalYaw"),
    )

def _to_gimbal_yaw_limits_model(cs_obj: GimbalYawLimits) -> GimbalYawLimitsModel:
    if not cs_obj:
        return None
    return GimbalYawLimitsModel(
        leftLimit=_get(cs_obj, "leftLimit", "LeftLimit"),
        rightLimit=_get(cs_obj, "rightLimit", "RightLimit"),
    )

def _to_auto_scan_model(cs_obj: AutoScan) -> AutoScanModel:
    if not cs_obj:
        return None
    return AutoScanModel(
        gimbalPitch=_get(cs_obj, "gimbalPitch", "GimbalPitch"),
        gimbalYawLimits=_to_gimbal_yaw_limits_model(
            _get(cs_obj, "gimbalYawLimits", "GimbalYawLimits")
        ),
        gimbalYawAngularSpeed=_get(
            cs_obj, "gimbalYawAngularSpeed", "GimbalYawAngularSpeed"
        ),
    )

def _to_filming_property_model(cs_obj: FilmingProperty) -> FilmingPropertyModel:
    if not cs_obj:
        return None
    return FilmingPropertyModel(
        fieldOfView=_get(cs_obj, "fieldOfView", "FieldOfView"),
        sensorType=_get(cs_obj, "sensorType", "SensorType"),
        operationMode=_get(cs_obj, "operationMode", "OperationMode"),
        coordinateOrientation=_to_coordinate_orientation_model(
            _get(cs_obj, "coordinateOrientation", "CoordinateOrientation")
        ),
        lineSearch=_to_line_search_model(_get(cs_obj, "lineSearch", "LineSearch")),
        autoTracking=_to_auto_tracking_model(
            _get(cs_obj, "autoTracking", "AutoTracking")
        ),
        aircraftFixed=_to_aircraft_fixed_model(
            _get(cs_obj, "aircraftFixed", "AircraftFixed")
        ),
        autoScan=_to_auto_scan_model(_get(cs_obj, "autoScan", "AutoScan")),
    )

def _to_waypoint_model_list(cs_list: CSharpList) -> List[WaypointModel]:
    if not cs_list:
        return []
    return [
        WaypointModel(
            waypointID=_get(item, "waypointID", "WaypointID"),
            coordinate=_to_coordinate_model(_get(item, "coordinate", "Coordinate")),
            speed=_get(item, "speed", "Speed"),
            eta=_get(item, "eta", "Eta"),
            ecf=_get(item, "ecf", "Ecf"),
            nextWaypointID=_get(item, "nextWaypointID", "NextWaypointID"),
            waypointPassType=_get(item, "waypointPassType", "WaypointPassType"),
            loiterProperty=_to_loiter_property_model(
                _get(item, "loiterProperty", "LoiterProperty")
            ),
            filmingProperty=_to_filming_property_model(
                _get(item, "filmingProperty", "FilmingProperty")
            ),
        )
        for item in cs_list
    ]


class UAVFlightPlanReceiver_0303(IFusionReceive[UAVFlightPlan], IsLocal, IsSingletone):
    """0303 UAVFlightPlan 메시지 수신 리시버"""

    __namespace__ = "UAVFlightPlanReceiver_0303"

    def Receive(self, data: UAVFlightPlan, src):
        try:
            # .NET 객체를 Python 데이터 모델 객체로 변환
            python_data = UAVFlightPlanModel(
                timestamp=_get(data, "timestamp", "Timestamp"),
                pathID=_get(data, "pathID", "PathID"),
                aircraftID=_get(data, "aircraftID", "AircraftID"),
                isFormationFlight=_get(data, "isFormationFlight", "IsFormationFlight"),
                formationInfo=_to_formation_info_model(
                    _get(data, "formationInfo", "FormationInfo")
                ),
                waypointList=_to_waypoint_model_list(
                    _get(data, "waypointList", "WaypointList")
                ),
            )

            # 데이터를 중앙 저장소에 저장
            ReceiveStorage().set_data("0303", python_data)

            # Manager 및 다른 모듈에 데이터 수신 알림
            notify_to_manager("0303", python_data)

        except Exception as e:
            print(f"[ERROR][Receive-0303] traceback ↓↓↓")
            traceback.print_exc()
            print(f"[ERROR][Receive-0303] Exception: {e}")