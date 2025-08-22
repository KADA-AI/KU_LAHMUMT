from nFusion.Model.msg_0503 import MissionResult
from generator.message0503_generator import make_msg0503_body
import json

def _dict_to_obj(body_dict: dict):
    mr = MissionResult()
    mr.timestamp       = body_dict["timestamp"]        # ulong
    mr.source          = body_dict["source"]          # string
    mr.systemRecommend = body_dict["systemRecommend"]  # uint
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
