from nFusion.Model.msg_0305 import *  # msg_0305에서 메시지 타입을 import
from generator.message0305_generator import make_msg0305_body  # generator에서 메시지 바디 가져오기




def _dict_to_obj(body_dict: dict):
    replan_status = ReplanStatus()
    replan_status.timestamp              = body_dict["timestamp"]
    replan_status.missionPlanningStatus  = body_dict["missionPlanningStatus"]
    replan_status.replanReason           = body_dict["replanReason"]
    return replan_status

import json 
def make_and_push(body_dict: dict, node_messenger) -> None:
    msg = _dict_to_obj(body_dict)
    #print(f"Message pushed: {msg}")
    node_messenger.Push(msg)
    # ── GUI 로그에 쓰일 문자열 만들기 ───────────────────
    log_line = (
        f"[0305] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0305] PUSH 완료"
    )
    #print(log_line)
    return log_line.encode()

def make_random_and_push(node_messenger) -> None:
    return make_and_push(make_msg0305_body(), node_messenger)

