# modules/monitoring/receive/message0402_receiver.py
import traceback
from typing import Iterable, List, Optional
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
from .receive_center import notify_to_manager

# 데이터 저장소 및 Python 데이터 모델
from data.receive_storage import ReceiveStorage
from data.message_models import (
    BattlefieldSituationAwarenessInfoModel,
    ROIInfoModel,
    SituationAwarenessInfoModel,
    CoordinateModel,
    TargetInfoModel,
    TargetWatcherModel,
)

# 대/소문자 안전 접근
_get = lambda obj, *names: next(
    (getattr(obj, n) for n in names if hasattr(obj, n)), None
)

# --- Nested Object Conversion Helpers ---


def _coerce_int(value) -> Optional[int]:
    if value is None:
        return None
    if isinstance(value, bool):
        return int(value)
    try:
        return int(value)
    except (TypeError, ValueError):
        try:
            return int(float(str(value).strip()))
        except (TypeError, ValueError):
            return None


def _coerce_float(value) -> Optional[float]:
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        try:
            return float(str(value).strip())
        except (TypeError, ValueError):
            return None


def _coerce_bool(value) -> Optional[bool]:
    if isinstance(value, bool):
        return value
    if value is None:
        return None
    if isinstance(value, (int, float)):
        return bool(value)
    try:
        lowered = str(value).strip().lower()
    except Exception:
        return None
    if lowered in {"1", "true", "t", "y", "yes", "on"}:
        return True
    if lowered in {"0", "false", "f", "n", "no", "off"}:
        return False
    return None


def _to_coordinate_model(cs_obj: Coordinate) -> Optional[CoordinateModel]:
    if not cs_obj:
        return None
    return CoordinateModel(
        latitude=_get(cs_obj, "latitude", "Latitude"),
        longitude=_get(cs_obj, "longitude", "Longitude"),
        altitude=_get(cs_obj, "altitude", "Altitude"),
    )


def _to_roi_info_model(cs_obj) -> Optional[ROIInfoModel]:
    if not cs_obj:
        return None
    coordinate = _to_coordinate_model(_get(cs_obj, "coordinate", "Coordinate")) or _to_coordinate_model(cs_obj)
    if coordinate is None:
        return None
    aircraft_id = _coerce_int(_get(cs_obj, "aircraftID", "AircraftID"))
    fov = _coerce_float(_get(cs_obj, "fov", "Fov"))
    return ROIInfoModel(
        aircraftID=aircraft_id if aircraft_id is not None else 0,
        coordinate=coordinate,
        fov=fov if fov is not None else 0.0,
    )


def _iter_safe(seq) -> Iterable:
    if not seq:
        return []
    try:
        return list(seq)
    except TypeError:
        return [seq]


def _to_roi_info_model_list(cs_list) -> List[ROIInfoModel]:
    if not cs_list:
        return []
    result: List[ROIInfoModel] = []
    for item in _iter_safe(cs_list):
        roi = _to_roi_info_model(item)
        if roi:
            result.append(roi)
    return result


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


def _to_target_info_model_list(cs_list) -> List[TargetInfoModel]:
    if not cs_list:
        return []
    result: List[TargetInfoModel] = []
    for item in _iter_safe(cs_list):
        if not item:
            continue
        coordinate = _to_coordinate_model(_get(item, "coordinate", "Coordinate")) or _to_coordinate_model(item)
        watcher_obj = _get(item, "watcher", "Watcher")
        watcher = None
        if watcher_obj:
            watcher_id = _coerce_int(_get(watcher_obj, "aircraftID", "AircraftID"))
            watcher = TargetWatcherModel(aircraftID=watcher_id)
        target = TargetInfoModel(
            targetID=_coerce_int(_get(item, "targetID", "TargetID", "targetId")),
            targetType=_coerce_int(_get(item, "targetType", "TargetType")),
            coordinate=coordinate,
            watcher=watcher,
            targetInFrame=_coerce_bool(_get(item, "targetInFrame", "TargetInFrame")),
            isDestroyed=_coerce_bool(_get(item, "isDestroyed", "IsDestroyed")),
            threat=_coerce_float(_get(item, "threat", "Threat")),
        )
        result.append(target)
    return result


class SituationAwarenessInfoReceiver_0402(
    IFusionReceive[SituationAwarenessInfo], IsLocal, IsSingletone
):
    """0402 BattlefieldSituationAwarenessInfo 메시지 수신 리시버"""

    __namespace__ = "BattlefieldSituationAwarenessInfoReceiver_0402"

    def Receive(self, data: SituationAwarenessInfo, src):
        try:
            roi_list_raw = _get(data, "roiInfoList", "RoiInfoList")
            roi_list = _to_roi_info_model_list(roi_list_raw)

            roi_single = None
            if not roi_list:
                roi_single = _to_roi_info_model(_get(data, "roiInfo", "ROIInfo", "roiinfo"))
                if roi_single:
                    roi_list = [roi_single]
            else:
                roi_single = roi_list[0]

            target_list = _to_target_info_model_list(
                _get(data, "targetList", "TargetList", "targets", "Targets")
            )

            # .NET 객체를 Python 데이터 모델 객체로 변환
            python_data = BattlefieldSituationAwarenessInfoModel(
                timestamp=_get(data, "timestamp", "Timestamp"),
                source=_get(data, "source", "Source"),
                roiInfoList=roi_list,
                situationAwarenessInfoList=_to_situation_awareness_info_model_list(
                    _get(
                        data,
                        "situationAwarenessInfoList",
                        "SituationAwarenessInfoList",
                    )
                ),
                roiInfo=roi_single,
                targetList=target_list,
            )

            # 데이터를 중앙 저장소에 저장
            ReceiveStorage().set_data("0402", python_data)

            # Manager 및 다른 모듈에 데이터 수신 알림
            notify_to_manager("0402", python_data)

        except Exception as e:
            print(f"[ERROR][Receive-0402] traceback ↓↓↓")
            traceback.print_exc()
            print(f"[ERROR][Receive-0402] Exception: {e}")
