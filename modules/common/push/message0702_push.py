from System.Collections.Generic import List  # C# List
from nFusion.Model.msg_0702 import * 
from generator.message0702_generator import make_msg0702_body  # 메시지 바디 생성기
import json 


def _dict_to_obj(body_dict: dict):
    """
    dict(JSON, 소문자 카멜) → MissionProgress(C# 객체)
    """
    mp = PilotDecision()
    mp.timestamp     = body_dict["timestamp"]
    mp.ignore        = body_dict["ignore"]
    mp.missionPlanID = body_dict["missionPlanID"]
    return mp

def make_and_push(body_dict: dict, node_messenger) -> None:
    msg = _dict_to_obj(body_dict)
    #print(f"Message pushed: {msg}")
    node_messenger.Push(msg)
    # ── GUI 로그에 쓰일 문자열 만들기 ───────────────────
    log_line = (
        f"[0702] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0702] PUSH 완료"
    )
    #print(log_line)
    return log_line.encode()

def make_random_and_push(node_messenger) -> None:
    return make_and_push(make_msg0702_body(), node_messenger)
