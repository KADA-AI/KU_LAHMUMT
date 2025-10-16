# modules/monitoring/receive/message0401_receiver.py
import traceback
from typing import List
from System.Collections.Generic import List as CSharpList

# C# 연동 관련 클래스
from dll_files.nFusionImports import IFusionReceive, IsLocal, IsSingletone

# C# 타입을 실제 네임스페이스에서 임포트합니다.
from nFusion.Model.msg_0401 import (
    AgentStatus,
    MannedInfo,
    UnmannedInfo,
    Weapons,
    DatalinkStatus,
    CurrentWaypointID,
    LoiterCoordinate,
    TargetFollowing,
    SensorInfo,
    CenterCoordinate,
    Velocity,
)
from nFusion.Model.CommonType import Coordinate, LeaderAircraftID

# 로컬 이벤트 버스
from gui.gui_app import notify_to_manager

# 데이터 저장소 및 Python 데이터 모델
from data.receive_storage import ReceiveStorage
from data.message_models import (
    AgentStatusModel,
    AgentStateModel,
    MannedInfoModel,
    UnmannedInfoModel,
    WeaponsModel,
    DatalinkStatusModel,
    CurrentWaypointIDModel,
    LoiterCoordinateModel,
    TargetFollowingModel,
    LeaderAircraftIDModel,
    SensorInfoModel,
    CenterCoordinateModel,
    CoordinateModel,
    VelocityModel,
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


def _to_velocity_model(cs_obj: Velocity) -> VelocityModel:
    if not cs_obj:
        return None
    return VelocityModel(
        speed=_get(cs_obj, "speed", "Speed"),
        heading=_get(cs_obj, "heading", "Heading"),
    )


def _to_weapons_model(cs_obj: Weapons) -> WeaponsModel:
    if not cs_obj:
        return None
    return WeaponsModel(
        type1=_get(cs_obj, "type1", "Type1"),
        type2=_get(cs_obj, "type2", "Type2"),
        type3=_get(cs_obj, "type3", "Type3"),
    )


def _to_datalink_status_model(cs_obj: DatalinkStatus) -> DatalinkStatusModel:
    if not cs_obj:
        return None
    return DatalinkStatusModel(
        isConnectedToUAV1=_get(cs_obj, "isConnectedToUAV1", "IsConnectedToUAV1"),
        isConnectedToUAV2=_get(cs_obj, "isConnectedToUAV2", "IsConnectedToUAV2"),
        isConnectedToUAV3=_get(cs_obj, "isConnectedToUAV3", "IsConnectedToUAV3"),
    )


def _to_manned_info_model(cs_obj: MannedInfo) -> MannedInfoModel:
    if not cs_obj:
        return None
    return MannedInfoModel(
        weapons=_to_weapons_model(_get(cs_obj, "weapons", "Weapons")),
        datalinkStatus=_to_datalink_status_model(
            _get(cs_obj, "datalinkStatus", "DatalinkStatus")
        ),
    )


def _to_current_waypoint_id_model(cs_obj: CurrentWaypointID) -> CurrentWaypointIDModel:
    if not cs_obj:
        return None
    return CurrentWaypointIDModel(waypointID=_get(cs_obj, "waypointID", "WaypointID"))


def _to_loiter_coordinate_model(cs_obj: LoiterCoordinate) -> LoiterCoordinateModel:
    if not cs_obj:
        return None
    return LoiterCoordinateModel(
        latitude=_get(cs_obj, "latitude", "Latitude"),
        longitude=_get(cs_obj, "longitude", "Longitude"),
        altitude=_get(cs_obj, "altitude", "Altitude"),
    )


def _to_target_following_model(cs_obj: TargetFollowing) -> TargetFollowingModel:
    if not cs_obj:
        return None
    return TargetFollowingModel(targetID=_get(cs_obj, "targetID", "TargetID"))


def _to_leader_aircraft_id_model(cs_obj: LeaderAircraftID) -> LeaderAircraftIDModel:
    if not cs_obj:
        return None
    return LeaderAircraftIDModel(aircraftID=_get(cs_obj, "aircraftID", "AircraftID"))


def _to_center_coordinate_model(cs_obj: CenterCoordinate) -> CenterCoordinateModel:
    if not cs_obj:
        return None
    return CenterCoordinateModel(
        latitude=_get(cs_obj, "latitude", "Latitude"),
        longitude=_get(cs_obj, "longitude", "Longitude"),
        altitude=_get(cs_obj, "altitude", "Altitude"),
    )


def _to_sensor_info_model(cs_obj: SensorInfo) -> SensorInfoModel:
    if not cs_obj:
        return None
    return SensorInfoModel(
        operationalMode=_get(cs_obj, "operationalMode", "OperationalMode"),
        sensorType=_get(cs_obj, "sensorType", "SensorType"),
        fov=_get(cs_obj, "fov", "Fov"),
        centerCoordinate=_to_center_coordinate_model(
            _get(cs_obj, "centerCoordinate", "CenterCoordinate")
        ),
    )


def _to_unmanned_info_model(cs_obj: UnmannedInfo) -> UnmannedInfoModel:
    if not cs_obj:
        return None
    return UnmannedInfoModel(
        currentWaypointID=_to_current_waypoint_id_model(
            _get(cs_obj, "currentWaypointID", "CurrentWaypointID")
        ),
        flightMode=_get(cs_obj, "flightMode", "FlightMode"),
        loiterCoordinate=_to_loiter_coordinate_model(
            _get(cs_obj, "loiterCoordinate", "LoiterCoordinate")
        ),
        targetFollowing=_to_target_following_model(
            _get(cs_obj, "targetFollowing", "TargetFollowing")
        ),
        leaderAircraftID=_to_leader_aircraft_id_model(
            _get(cs_obj, "leaderAircraftID", "LeaderAircraftID")
        ),
        sensorInfo=_to_sensor_info_model(_get(cs_obj, "sensorInfo", "SensorInfo")),
        payloadHealth=_get(cs_obj, "payloadHealth", "PayloadHealth"),
        fuelWarning=_get(cs_obj, "fuelWarning", "FuelWarning"),
    )


def _to_agent_state_model_list(cs_list: CSharpList) -> List[AgentStateModel]:
    if not cs_list:
        return []
    return [
        AgentStateModel(
            aircraftID=_get(item, "aircraftID", "AircraftID"),
            isUnmanned=_get(item, "isUnmanned", "IsUnmanned"),
            coordinate=_to_coordinate_model(_get(item, "coordinate", "Coordinate")),
            velocity=_to_velocity_model(_get(item, "velocity", "Velocity")),
            fuel=_get(item, "fuel", "Fuel"),
            health=_get(item, "health", "Health"),
            mannedInfo=_to_manned_info_model(_get(item, "mannedInfo", "MannedInfo")),
            unmannedInfo=_to_unmanned_info_model(
                _get(item, "unmannedInfo", "UnmannedInfo")
            ),
        )
        for item in cs_list
    ]


class AgentStatusReceiver_0401(IFusionReceive[AgentStatus], IsLocal, IsSingletone):
    """0401 AgentStatus 메시지 수신 리시버"""

    __namespace__ = "AgentStatusReceiver_0401"

    def Receive(self, data: AgentStatus, src):
        try:
            # .NET 객체를 Python 데이터 모델 객체로 변환
            python_data = AgentStatusModel(
                timestamp=_get(data, "timestamp", "Timestamp"),
                source=_get(data, "source", "Source"),
                agentStateList=_to_agent_state_model_list(
                    _get(data, "agentStateList", "AgentStateList")
                ),
            )

            # 데이터를 중앙 저장소에 저장
            ReceiveStorage().set_data("0401", python_data)

            # Manager 및 다른 모듈에 데이터 수신 알림
            notify_to_manager("0401", python_data)

        except Exception as e:
            print(f"[ERROR][Receive-0401] traceback ↓↓↓")
            traceback.print_exc()
            print(f"[ERROR][Receive-0401] Exception: {e}")
