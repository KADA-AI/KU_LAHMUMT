from System.Collections.Generic import List  # C# List
from nFusion.Model.msg_0402 import *  # msg_0501에서 메시지 타입을 import
from generator.message0402_generator import make_msg0402_body  # generator에서 메시지 바디 가져오기




def _dict_to_obj(body_dict: dict):
    """
    dict(JSON, 소문자 카멜) → SituationAwarenessInfo(C# 객체)
    """
    info = SituationAwarenessInfo()
    info.timestamp = body_dict["timestamp"]

    # ───────── ROIInfo ─────────
    roi_dict = body_dict["roiInfo"]
    roi = ROIInfo()
    roi.aircraftID = roi_dict["aircraftID"]

    coord_roi = Coordinate()
    coord_roi.latitude  = roi_dict["coordinate"]["latitude"]
    coord_roi.longitude = roi_dict["coordinate"]["longitude"]
    coord_roi.altitude  = roi_dict["coordinate"]["altitude"]
    roi.coordinate = coord_roi

    roi.fov = roi_dict["fov"]
    info.roiInfo = roi

    # ───────── TargetList ─────────
    tgt_list = List[Target]()
    for t in body_dict["targetList"]:
        tgt = Target()
        tgt.targetID      = t["targetID"]
        tgt.targetType    = t["targetType"]

        coord_t = Coordinate()
        coord_t.latitude  = t["coordinate"]["latitude"]
        coord_t.longitude = t["coordinate"]["longitude"]
        coord_t.altitude  = t["coordinate"]["altitude"]
        tgt.coordinate = coord_t

        w = Watcher()
        w.aircraftID = t["watcher"]["aircraftID"]
        tgt.watcher = w

        tgt.targetInFrame = t["targetInFrame"]
        tgt.isDestroyed   = t["isDestroyed"]
        tgt.threat        = t["threat"]

        tgt_list.Add(tgt)
    info.targetList = tgt_list

    return info


import json 
def make_and_push(body_dict: dict, node_messenger) -> None:
    msg = _dict_to_obj(body_dict)
    #print(f"Message pushed: {msg}")
    node_messenger.Push(msg)
    # ── GUI 로그에 쓰일 문자열 만들기 ───────────────────
    log_line = (

        f"[0402] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0402] PUSH 완료"

    )
    #print(log_line)
    return log_line.encode()

def make_random_and_push(node_messenger) -> None:
    return make_and_push(make_msg0402_body(), node_messenger)


