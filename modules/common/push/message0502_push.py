# generator/message0502_push.py
from nFusion.Model.msg_0502 import EndMissionRequest
from generator.message0502_generator import make_msg0502_body
import json

def _dict_to_obj(body_dict: dict) -> EndMissionRequest:
    req = EndMissionRequest()
    req.timestamp = int(body_dict["timestamp"])
    return req

def make_and_push(body_dict: dict, node_messenger) -> bytes:
    msg = _dict_to_obj(body_dict)
    node_messenger.Push(msg)
    log_line = (
        f"[0502] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0502] PUSH 완료"
    )
    return log_line.encode("utf-8", "ignore")

def make_random_and_push(node_messenger) -> bytes:
    return make_and_push(make_msg0502_body(), node_messenger)
