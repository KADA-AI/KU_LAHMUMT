from nFusion.Model.msg_0102 import ModuleStatus
from generator.message0102_generator import make_msg0102_body  # generator에서 메시지 바디 가져오기

def _dict_to_obj(body_dict: dict):
    module_status = ModuleStatus()
    module_status.timestamp        = body_dict["timestamp"]
    module_status.status           = body_dict["status"]
    module_status.sourceModuleName = body_dict["sourceModuleName"]
    return module_status

import json 
def make_and_push(body_dict: dict, node_messenger) -> None:
    msg = _dict_to_obj(body_dict)
    #print(f"Message pushed: {msg}")
    node_messenger.Push(msg)
    # ── GUI 로그에 쓰일 문자열 만들기 ───────────────────
    log_line = (
        f"[0102] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0102] PUSH 완료"
    )
    ##print(log_line)
    return log_line.encode()

def make_random_and_push(node_messenger) -> None:
    return make_and_push(make_msg0102_body(), node_messenger)

