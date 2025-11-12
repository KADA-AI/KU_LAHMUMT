# modules/monitoring/receive/message0202_receiver.py
import json
import time
import traceback
from dataclasses import asdict
from typing import List
from System.Collections.Generic import List as CSharpList

# C# 연동 관련 클래스
from dll_files.nFusionImports import IFusionReceive, IsLocal, IsSingletone

# C# 타입을 실제 네임스페이스에서 임포트합니다.
from nFusion.Model.msg_0202 import PriorMissionInfo
from nFusion.Model.CommonType import (
    Coordinate,
    CoordinateOrientation,
    TargetOrientation,
    PriorMission,
)

# 로컬 이벤트 버스
from .receive_center import notify_to_manager

# 데이터 저장소 및 Python 데이터 모델
from data.receive_storage import ReceiveStorage
from data.message_models import (
    PriorMissionInfoModel,
    PriorMissionModel,
    CoordinateOrientationModel,
    TargetOrientationModel,
    CoordinateModel,
)
from modules.common import db_paths

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

def _to_coordinate_orientation_model(
    cs_obj: CoordinateOrientation,
) -> CoordinateOrientationModel:
    if not cs_obj:
        return None
    return CoordinateOrientationModel(
        coordinate=_to_coordinate_model(_get(cs_obj, "coordinate", "Coordinate"))
    )

def _to_target_orientation_model(cs_obj: TargetOrientation) -> TargetOrientationModel:
    if not cs_obj:
        return None
    return TargetOrientationModel(targetID=_get(cs_obj, "targetID", "TargetID"))

def _to_prior_mission_model_list(cs_list: CSharpList) -> List[PriorMissionModel]:
    if not cs_list:
        return []
    return [
        PriorMissionModel(
            priorMissionID=_get(item, "priorMissionID", "PriorMissionID"),
            missionType=_get(item, "missionType", "MissionType"),
            coordinateOrientation=_to_coordinate_orientation_model(
                _get(item, "coordinateOrientation", "CoordinateOrientation")
            ),
            targetOrientation=_to_target_orientation_model(
                _get(item, "targetOrientation", "TargetOrientation")
            ),
        )
        for item in cs_list
    ]


class PriorMissionInfoReceiver_0202(
    IFusionReceive[PriorMissionInfo], IsLocal, IsSingletone
):
    """0202 PriorMissionInfo 메시지 수신 리시버"""

    __namespace__ = "PriorMissionInfoReceiver_0202"

    def Receive(self, data: PriorMissionInfo, src):
        try:
            # .NET 객체를 Python 데이터 모델 객체로 변환
            python_data = PriorMissionInfoModel(
                timestamp=_get(data, "timestamp", "Timestamp"),
                source=_get(data, "source", "Source"),
                priorMissionList=_to_prior_mission_model_list(
                    _get(data, "priorMissionList", "PriorMissionList")
                ),
            )

            # 데이터를 중앙 저장소에 저장
            ReceiveStorage().set_data("0202", python_data)

            # DB 폴더에 스냅샷 저장
            _persist_prior_mission_snapshot(python_data)

            # Manager 및 다른 모듈에 데이터 수신 알림
            notify_to_manager("0202", python_data)

        except Exception as e:
            print(f"[ERROR][Receive-0202] traceback ↓↓↓")
            traceback.print_exc()
            print(f"[ERROR][Receive-0202] Exception: {e}")


def _persist_prior_mission_snapshot(data: PriorMissionInfoModel) -> None:
    try:
        target_dir = db_paths.get_db_subpath("PriorMissionInfo")
    except Exception:
        return
    try:
        target_dir.mkdir(parents=True, exist_ok=True)
        snapshot = asdict(data)
        for entry in snapshot.get("priorMissionList", []) or []:
            mission_id = entry.get("priorMissionID")
            if mission_id is None:
                continue
            filename = f"{int(mission_id)}.json"
            path = target_dir / filename
            with path.open("w", encoding="utf-8") as fh:
                json.dump(entry, fh, ensure_ascii=False, indent=2)
    except Exception:
        # 스냅샷 실패는 서비스 영향을 주지 않도록 조용히 무시
        return
