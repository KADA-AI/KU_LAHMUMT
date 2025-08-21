# push/message0601_push.py
from System.Collections.Generic import List
from nFusion.Model.msg_0601 import *
from generator.message0601_generator import make_msg0601_body  # 메시지 바디 생성기
import json




def _dict_to_obj(body: dict):
    ua = BasicAction()
    ua.timestamp    = body["timestamp"]
    ua.aircraft     = body["aircraft"]
    ua.flightMode   = body["flightMode"]
    ua.filmingMode  = body["filmingMode"]
    return ua

def make_and_push(body_dict: dict, node_messenger) -> bytes:
    msg = _dict_to_obj(body_dict)
    #print(f"Message pushed: {msg}")
    node_messenger.Push(msg)

    log_line = (
        f"[0601] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0601] PUSH 완료"
    )
    return log_line.encode()

def make_random_and_push(node_messenger) -> None:
    return make_and_push(make_msg0601_body(), node_messenger)
