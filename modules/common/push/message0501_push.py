from System.Collections.Generic import List  # C# List
from nFusion.Model.msg_0501 import *  # msg_0501에서 메시지 타입을 import
from generator.message0501_generator import make_msg0501_body  # generator에서 메시지 바디 가져오기


def _dict_to_obj(body_dict: dict):
    """새 0501 규격(dict) → MissionStateInfo(C#) 변환"""
    state = MissionProgress()

    # ───────── Top-level ─────────
    state.timestamp            = body_dict["timestamp"]                 # ulong
    state.currentMissionPlanID = body_dict["currentMissionPlanID"]      # uint
    state.currentInputMissionID = body_dict["currentInputMissionID"]    # uint

    # ───────── IndividualMissionProgressStatusList ─────────
    imps_list = List[IndividualMissionProgressStatus]()

    for item in body_dict["individualMissionProgressStatusList"]:
        imps = IndividualMissionProgressStatus()
        imps.aircraftID = item["aircraftID"]                            # uint

        cim = CurrentIndividualMission()
        cim.individualMissionID = item["currentIndividualMission"]["individualMissionID"]
        imps.currentIndividualMission = cim                             # msg object

        imps.currentIndividualMissionProgress = item["currentIndividualMissionProgress"]  # uint
        imps_list.Add(imps)

    state.individualMissionProgressStatusList = imps_list
    return state

import json 
def make_and_push(body_dict: dict, node_messenger) -> None:
    msg = _dict_to_obj(body_dict)
    #print(f"Message pushed: {msg}")
    node_messenger.Push(msg)
    # ── GUI 로그에 쓰일 문자열 만들기 ───────────────────
    log_line = (
        f"[0501] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0501] PUSH 완료"
    )
    #print(log_line)
    return log_line.encode()

def make_random_and_push(node_messenger) -> None:
    return make_and_push(make_msg0501_body(), node_messenger)

