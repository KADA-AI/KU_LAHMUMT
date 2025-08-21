# push/message0804_push.py

from nFusion.Model.msg_0804 import MissionRestartCommand
from generator.message0804_generator import make_msg0804_body
import json




def _dict_to_obj(body: dict) -> MissionRestartCommand:
    cmd = MissionRestartCommand()
    cmd.timestamp      = body["timestamp"]
    cmd.inputMissionID = body["inputMissionID"]
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
        f"[0804] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0804] PUSH 완료"
    )
    return log_line.encode()

def make_random_and_push(node_messenger) -> bytes:
    return make_and_push(make_msg0804_body(), node_messenger)
