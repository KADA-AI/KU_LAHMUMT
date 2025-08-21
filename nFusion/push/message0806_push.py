# push/message0806_push.py

from nFusion.Model.msg_0806 import BootCommand
from generator.message0806_generator import make_msg0806_body
import json




def _dict_to_obj(body: dict) -> BootCommand:
    cmd = BootCommand()
    cmd.timestamp = body["timestamp"]
    cmd.command   = body["command"]
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
        f"[0806] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0806] PUSH 완료"
    )
    return log_line.encode()

def make_random_and_push(node_messenger) -> bytes:
    return make_and_push(make_msg0806_body(), node_messenger)
