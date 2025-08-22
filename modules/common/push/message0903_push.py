# push/message0903_push.py
from nFusion.Model.msg_0903 import RequestRenewMission
from generator.message0903_generator import make_msg0903_body
import json


def _dict_to_obj(body: dict) -> RequestRenewMission:
    req = RequestRenewMission()
    req.timestamp = body["timestamp"]
    req.missionPlanID = body["missionPlanID"]
    return req

def make_and_push(body_dict: dict, node_messenger) -> bytes | None:
    """
    · 메시지를 Push 한 뒤, GUI 로그에 그대로 찍을 수 있도록
      'BODY  … / PUSH 완료' 문자열을 UTF-8 바이트로 반환한다.
    """
    msg = _dict_to_obj(body_dict)            # dict → C# 객체
    node_messenger.Push(msg)                 # 전송

    # ── GUI 로그에 쓰일 문자열 만들기 ───────────────────
    log_line = (
        f"[0903] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0903] PUSH 완료"
    )
    return log_line.encode()                 # ← push_center → _mark_sent 로 전달

def make_random_and_push(node_messenger) -> bytes | None:
    return make_and_push(make_msg0903_body(), node_messenger)
