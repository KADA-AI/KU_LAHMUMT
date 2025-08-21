from System.Collections.Generic import List                                  # C# List
from nFusion.Model.msg_0901 import *                          # msg_0901 타입 import
from generator.message0901_generator import make_msg0901_body               # 메시지 바디 생성기
import json




def _dict_to_obj(body: dict):
    req = RequestOptionInfo()

    # ── 최상위 필드 ───────────────────────────────
    req.timestamp   = body["timestamp"]       # ulong(ms)
    req.requestTime = body["requestTime"]     # ulong(ms)

    # ── OptionList 변환 ──────────────────────────
    opt_list = List[Option]()
    for itm in body["optionList"]:
        opt = Option()
        opt.optionID      = itm["optionID"]
        opt.optionName    = itm["optionName"]
        opt.missionPlanID = itm["missionPlanID"]
        opt_list.Add(opt)

    req.optionList = opt_list
    return req

def make_and_push(body_dict: dict, node_messenger) -> None:
    msg = _dict_to_obj(body_dict)
    #print(f"Message pushed: {msg}")
    node_messenger.Push(msg)

    # ── GUI 로그에 쓰일 문자열 만들기 ───────────────────
    log_line = (
        f"[0901] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0901] PUSH 완료"
    )
    return log_line.encode()

def make_random_and_push(node_messenger) -> None:
    return make_and_push(make_msg0901_body(), node_messenger)
