# generator/message0305_push.py
from nFusion.Model.msg_0305 import ReplanStatus
from generator.message0305_generator import make_msg0305_body
import json

def _dict_to_obj(body_dict: dict) -> ReplanStatus:
    obj = ReplanStatus()
    obj.timestamp             = int(body_dict["timestamp"])
    obj.missionPlanningStatus = int(body_dict["missionPlanningStatus"])
    obj.replanReason          = str(body_dict["replanReason"])
    return obj

def make_and_push(body_dict: dict, node_messenger) -> bytes:
    msg = _dict_to_obj(body_dict)
    node_messenger.Push(msg)
    log_line = (
        f"[0305] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0305] PUSH 완료"
    )
    return log_line.encode("utf-8", "ignore")

def make_random_and_push(node_messenger) -> bytes:
    return make_and_push(make_msg0305_body(), node_messenger)
