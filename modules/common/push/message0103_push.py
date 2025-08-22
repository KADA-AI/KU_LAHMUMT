from nFusion.Model.msg_0103 import SWStatus
from generator.message0103_generator import make_msg0103_body  # generator에서 메시지 바디 가져오기
import json

def _dict_to_obj(body_dict: dict):
    sw_status = SWStatus()
    sw_status.timestamp = body_dict["timestamp"]
    sw_status.status    = body_dict["status"]
    sw_status.mode      = body_dict["mode"]
    return sw_status

def make_and_push(body_dict: dict, node_messenger) -> bytes:
    msg = _dict_to_obj(body_dict)
    node_messenger.Push(msg)

    log_line = (
        f"[0103] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0103] PUSH 완료"
    )
    return log_line.encode()

def make_random_and_push(node_messenger) -> bytes:
    return make_and_push(make_msg0103_body(), node_messenger)
