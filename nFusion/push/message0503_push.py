# push/message0503_push.py
from System.Collections.Generic import List
from nFusion.Model.msg_0503 import *
from generator.message0503_generator import make_msg0503_body
import json


def _dict_to_obj(body_dict: dict):
    """
    dict(JSON) → MissionResults(C# 객체) 변환
    MissionID 0503 – 협업기저임무 완료 알림 (MissionResults)
    """
    mr = MissionResult()

    # ─── Top-level 필드 ──────────────────────────────
    mr.timestamp       = body_dict["timestamp"]        # ulong
    mr.type            = body_dict["type"]             # uint
    mr.inputMissionID  = body_dict["inputMissionID"]   # uint
    mr.systemRecommend = body_dict["systemRecommend"]  # uint

    # ─── IndividualMissionList ──────────────────────
    im_list = List[IndividualMission]()
    for item in body_dict["individualMissionList"]:
        im = IndividualMission()
        im.aircraftID          = item["aircraftID"]          # uint
        im.individualMissionID = item["individualMissionID"] # uint
        im_list.Add(im)
    mr.individualMissionList = im_list

    return mr


def make_and_push(body_dict: dict, node_messenger) -> bytes:
    msg = _dict_to_obj(body_dict)
    node_messenger.Push(msg)

    log_line = (
        f"[0503] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0503] PUSH 완료"
    )
    return log_line.encode()


def make_random_and_push(node_messenger) -> bytes:
    return make_and_push(make_msg0503_body(), node_messenger)
