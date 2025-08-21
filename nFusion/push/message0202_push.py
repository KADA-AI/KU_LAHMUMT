from nFusion.Model.msg_0202 import *
from generator.message0202_generator import make_msg0202_body  # generator에서 메시지 바디 가져오기
from System.Collections.Generic import List  # C# List




def _dict_to_obj(body_dict: dict):
    """
    dict(JSON, 소문자 카멜) → PriorMissionInfo(C# 객체)
    """
    info = PriorMissionInfo()
    info.timestamp = body_dict["timestamp"]

    pm_list = List[PriorMission]()
    for pm in body_dict["priorMissionList"]:
        pm_obj = PriorMission()
        pm_obj.priorMissionID = pm["priorMissionID"]
        pm_obj.missionType     = pm["missionType"]

        # CoordinateOrientation
        coord = CoordinateOrientation()
        coord.latitude  = pm["coordinateOrientation"]["latitude"]
        coord.longitude = pm["coordinateOrientation"]["longitude"]
        coord.altitude  = pm["coordinateOrientation"]["altitude"]
        pm_obj.coordinateOrientation = coord

        # TargetOrientation
        tgt = TargetOrientation()
        tgt.targetID = pm["targetOrientation"]["targetID"]
        pm_obj.targetOrientation = tgt

        pm_list.Add(pm_obj)

    info.priorMissionList = pm_list
    return info


import json 
def make_and_push(body_dict: dict, node_messenger) -> None:
    msg = _dict_to_obj(body_dict)
    #print(f"Message pushed: {msg}")
    node_messenger.Push(msg)
    # ── GUI 로그에 쓰일 문자열 만들기 ───────────────────
    log_line = (
        f"[0202] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0202] PUSH 완료"

    )
    ##print(log_line)
    return log_line.encode()

def make_random_and_push(node_messenger) -> None:

    return make_and_push(make_msg0202_body(), node_messenger)


