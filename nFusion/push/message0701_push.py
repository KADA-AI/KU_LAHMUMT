from System.Collections.Generic import List  # C# List
from nFusion.Model.msg_0701 import *  # msg_0701 타입 import
from generator.message0701_generator import make_msg0701_body  # 메시지 바디 생성기
import json 

def _dict_to_obj(body: dict):
    """
    dict(JSON, **소문자 키**) → MissionPlanOptionInfo(C# 객체)
    """
    mpoi = MissionPlanOptionInfo()
    mpoi.timestamp               = body["timestamp"]          # ← 필드명 오탈자 수정
    mpoi.autoExecution           = body["autoExecution"]

    # ── optionList ───────────────────────────────────────────
    opt_list = List[Option]()
    for opt in body["optionList"]:
        opt_obj = Option()
        opt_obj.optionID           = opt["optionID"]
        opt_obj.optionName         = opt["optionName"]
        opt_obj.survivalRate       = opt["survivalRate"]
        opt_obj.timeContraction    = opt["timeContraction"]
        opt_obj.recogEffectiveness = opt["recogEffectiveness"]
        opt_obj.distance           = opt["distance"]
        opt_obj.target             = opt["target"]

        # UAVMissionPlanIDList
        uav_list = List[UAVMissionPlanID]()
        for u in opt["uavMissionPlanIDList"]:
            uav = UAVMissionPlanID()
            uav.uavMissionPlanID = u["uavMissionPlanID"]
            uav_list.Add(uav)
        opt_obj.uavMissionPlanIDList = uav_list

        # LAHMissionPlanIDList
        lah_list = List[LAHMissionPlanID]()
        for l in opt["lahMissionPlanIDList"]:
            lah = LAHMissionPlanID()
            lah.LahMissionPlanID = l["lahMissionPlanID"]
            lah_list.Add(lah)
        opt_obj.LahMissionPlanIDList = lah_list

        opt_list.Add(opt_obj)

    mpoi.optionList = opt_list
    return mpoi

def make_and_push(body_dict: dict, node_messenger) -> None:
    msg = _dict_to_obj(body_dict)
    #print(f"Message pushed: {msg}")
    node_messenger.Push(msg)
    # ── GUI 로그에 쓰일 문자열 만들기 ───────────────────
    log_line = (
        f"[0701] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0701] PUSH 완료"
    )
    #print(log_line)
    return log_line.encode()

def make_random_and_push(node_messenger) -> None:
    return make_and_push(make_msg0701_body(), node_messenger)
