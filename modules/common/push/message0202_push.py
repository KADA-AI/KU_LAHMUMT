# generator/message0202_push.py
from nFusion.Model.msg_0202 import PriorMissionInfo, PriorMission, CoordinateOrientation, TargetOrientation
from generator.message0202_generator import make_msg0202_body
from System.Collections.Generic import List  # C# List[T]
import json

def _dict_to_obj(body_dict: dict) -> PriorMissionInfo:
    """
    dict(JSON, 소문자 카멜) → PriorMissionInfo(C# 객체)
    - altitude는 int로 세팅
    - missionType=1: coordinateOrientation만
    - missionType=2: targetOrientation만
    """
    info = PriorMissionInfo()
    info.timestamp = int(body_dict["timestamp"])

    pm_list = List[PriorMission]()
    for pm in body_dict.get("priorMissionList", []):
        pm_obj = PriorMission()
        pm_obj.priorMissionID = int(pm["priorMissionID"])
        pm_obj.missionType    = int(pm["missionType"])

        if pm_obj.missionType == 1:
            # CoordinateOrientation
            co = pm.get("coordinateOrientation", {})
            coord = CoordinateOrientation()
            coord.latitude  = float(co.get("latitude", 0.0))
            coord.longitude = float(co.get("longitude", 0.0))
            coord.altitude  = int(co.get("altitude", 0))   # ← int 보장
            pm_obj.coordinateOrientation = coord
        else:
            # TargetOrientation
            to = pm.get("targetOrientation", {})
            tgt = TargetOrientation()
            tgt.targetID = int(to.get("targetID", 0))
            pm_obj.targetOrientation = tgt

        pm_list.Add(pm_obj)

    info.priorMissionList = pm_list
    return info

def make_and_push(body_dict: dict, node_messenger) -> bytes:
    msg = _dict_to_obj(body_dict)
    node_messenger.Push(msg)
    log_line = (
        f"[0202] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0202] PUSH 완료"
    )
    return log_line.encode("utf-8", "ignore")

def make_random_and_push(node_messenger) -> bytes:
    return make_and_push(make_msg0202_body(), node_messenger)
