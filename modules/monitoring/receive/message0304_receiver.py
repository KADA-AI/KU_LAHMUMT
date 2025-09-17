# modules/monitoring/receive/message0304_receiver.py
import traceback
from typing import List
from System.Collections.Generic import List as CSharpList

# C# 연동 관련 클래스
from dll_files.nFusionImports import IFusionReceive, IsLocal, IsSingletone

# C# 타입을 실제 네임스페이스에서 임포트합니다.
from nFusion.Model.msg_0304 import (
    LAHFlightPlan,
    LAHWaypoint,
    Hovering,
    Loiter,
    Attack,
)
from nFusion.Model.CommonType import Coordinate

# 로컬 이벤트 버스
from .receive_center import notify_to_manager

# 데이터 저장소 및 Python 데이터 모델
from data.receive_storage import ReceiveStorage
from data.message_models import (
    LAHFlightPlanModel,
    LAHWaypointModel,
    HoveringModel,
    LoiterModel,
    AttackModel,
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

def _to_hovering_model(cs_obj: Hovering) -> HoveringModel:
    if not cs_obj:
        return None
    return HoveringModel(time=_get(cs_obj, "time", "Time"))

def _to_loiter_model(cs_obj: Loiter) -> LoiterModel:
    if not cs_obj:
        return None
    return LoiterModel(
        radius=_get(cs_obj, "radius", "Radius"),
        direction=_get(cs_obj, "direction", "Direction"),
        time=_get(cs_obj, "time", "Time"),
        speed=_get(cs_obj, "speed", "Speed"),
    )

def _to_attack_model(cs_obj: Attack) -> AttackModel:
    if not cs_obj:
        return None
    return AttackModel(
        targetID=_get(cs_obj, "targetID", "TargetID"),
        weaponType=_get(cs_obj, "weaponType", "WeaponType"),
    )

def _to_lah_waypoint_model_list(cs_list: CSharpList) -> List[LAHWaypointModel]:
    if not cs_list:
        return []
    return [
        LAHWaypointModel(
            waypointID=_get(item, "waypointID", "WaypointID"),
            coordinate=_to_coordinate_model(_get(item, "coordinate", "Coordinate")),
            speed=_get(item, "speed", "Speed"),
            eta=_get(item, "eta", "Eta"),
            ecf=_get(item, "ecf", "Ecf"),
            nextWaypointID=_get(item, "nextWaypointID", "NextWaypointID"),
            hovering=_to_hovering_model(_get(item, "hovering", "Hovering")),
            loiter=_to_loiter_model(_get(item, "loiter", "Loiter")),
            attack=_to_attack_model(_get(item, "attack", "Attack")),
        )
        for item in cs_list
    ]


class LAHFlightPlanReceiver_0304(IFusionReceive[LAHFlightPlan], IsLocal, IsSingletone):
    """0304 LAHFlightPlan 메시지 수신 리시버"""

    __namespace__ = "LAHFlightPlanReceiver_0304"

    def Receive(self, data: LAHFlightPlan, src):
        try:
            # .NET 객체를 Python 데이터 모델 객체로 변환
            python_data = LAHFlightPlanModel(
                timestamp=_get(data, "timestamp", "Timestamp"),
                pathID=_get(data, "pathID", "PathID"),
                aircraftID=_get(data, "aircraftID", "AircraftID"),
                lahWaypointList=_to_lah_waypoint_model_list(
                    _get(data, "lahWaypointList", "LahWaypointList")
                ),
            )

            # 데이터를 중앙 저장소에 저장
            ReceiveStorage().set_data("0304", python_data)

            # Manager 및 다른 모듈에 데이터 수신 알림
            notify_to_manager("0304", python_data)

        except Exception as e:
            print(f"[ERROR][Receive-0304] traceback ↓↓↓")
            traceback.print_exc()
            print(f"[ERROR][Receive-0304] Exception: {e}")