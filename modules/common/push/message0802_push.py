# push/message0802_push.py

from nFusion.Model.msg_0802 import MandatoryCommand
from generator.message0802_generator import make_msg0802_body
import json




def _dict_to_obj(body: dict) -> MandatoryCommand:
    cmd = MandatoryCommand()
    cmd.timestamp     = body["timestamp"]
    cmd.aircraftID    = body["aircraftID"]
    cmd.mandatoryType = body["mandatoryType"]
    return cmd

def make_and_push(body_dict: dict, node_messenger) -> bytes:
    """
    · 메시지를 Push 한 뒤, GUI 로그에 그대로 찍을 수 있도록
      'BODY  … / PUSH 완료' 문자열을 UTF-8 바이트로 반환합니다.
    """
    msg = _dict_to_obj(body_dict)      # dict → C# 객체
    #print(f"Message pushed: {msg}")
    node_messenger.Push(msg)           # 전송

    # ── GUI 로그에 쓰일 문자열 만들기 ───────────────────
    log_line = (
        f"[0802] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0802] PUSH 완료"
    )
    return log_line.encode()

def make_random_and_push(node_messenger) -> bytes:
    return make_and_push(make_msg0802_body(), node_messenger)
