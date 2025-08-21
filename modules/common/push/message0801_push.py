from nFusion.Model.msg_0801 import *  
from generator.message0801_generator import make_msg0801_body  # 메시지 바디 생성기
import json 




def _dict_to_obj(body_dict: dict):
    """
    dict(JSON, 소문자 카멜) → ReplanCommand(C# 객체)
    """
    cmd = InitialPlanCommand()
    cmd.timestamp                = body_dict["timestamp"]
    cmd.operatorReplanRequestTime = body_dict["operatorReplanRequestTime"]
    cmd.isOnGround               = body_dict["isOnGround"]
    cmd.inputMissionPackageID    = body_dict["inputMissionPackageID"]
    cmd.missionReferencePackageID = body_dict["missionReferencePackageID"]
    return cmd


def make_and_push(body_dict: dict, node_messenger) -> None:
    msg = _dict_to_obj(body_dict)
    #print(f"Message pushed: {msg}")
    node_messenger.Push(msg)
    # ── GUI 로그에 쓰일 문자열 만들기 ───────────────────
    log_line = (
        f"[0801] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0801] PUSH 완료"

    )
    #print(log_line)
    return log_line.encode()

def make_random_and_push(node_messenger) -> None:
    return make_and_push(make_msg0801_body(), node_messenger)

