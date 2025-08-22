from nFusion.Model.msg_0000 import *
from generator.message0000_generator import make_msg0000_body  # generator에서 메시지 바디 가져오기

def _dict_to_obj(body_dict: dict):
    from nFusion.Model.msg_0000 import Response
    resp = Response()

    # timestamp (ulong)
    resp.timestamp = int(body_dict["timestamp"])

    # source(string) ← 규격 필드명을 우선, 없으면 requestModuleName로 폴백
    val_source = body_dict.get("source", body_dict.get("requestModuleName", ""))
    # 속성 이름이 환경에 따라 다를 수 있어 양쪽 모두 시도
    if hasattr(resp, "source"):
        setattr(resp, "source", val_source)
    elif hasattr(resp, "Source"):
        setattr(resp, "Source", val_source)
    elif hasattr(resp, "requestModuleName"):
        setattr(resp, "requestModuleName", val_source)
    elif hasattr(resp, "RequestModuleName"):
        setattr(resp, "RequestModuleName", val_source)

    # messageID (uint)
    resp.messageID = int(body_dict["messageID"])
    return resp

import json 
def make_and_push(body_dict: dict, node_messenger) -> bytes:
    msg = _dict_to_obj(body_dict)
    node_messenger.Push(msg)

    import json
    log_line = (
        f"[0000] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0000] PUSH 완료"
    )
    return log_line.encode("utf-8", "ignore")

def make_random_and_push(node_messenger) -> bytes:
    from generator.message0000_generator import make_msg0000_body
    return make_and_push(make_msg0000_body(), node_messenger)

