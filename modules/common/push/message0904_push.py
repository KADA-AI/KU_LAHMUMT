from nFusion.Model.msg_0904 import *        # msg_0903 타입 import
from generator.message0904_generator import make_msg0904_body # 메시지 바디 생성기
import json




def _dict_to_obj(body_dict: dict):
    req = RequestBehaviorTree()
    req.timestamp         = body_dict["timestamp"]
    req.behaviorTreeFileID = body_dict["behaviorTreeFileID"]
    return req

def make_and_push(body_dict: dict, node_messenger) -> bytes | None:
    msg = _dict_to_obj(body_dict)
    #print(f"Message pushed: {msg}")
    node_messenger.Push(msg)

    # ── GUI 로그에 쓰일 문자열 만들기 ───────────────────
    log_line = (
        f"[0904] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0904] PUSH 완료"
    )
    return log_line.encode()                 # ← push_center → _mark_sent 로 전달

def make_random_and_push(node_messenger) -> bytes | None:
    return make_and_push(make_msg0904_body(), node_messenger)
