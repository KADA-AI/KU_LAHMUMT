# -*- coding: utf-8 -*-
from System.Collections.Generic import List
from nFusion.Model.msg_0901 import RequestOptionInfo
from nFusion.Model.CommonType import PendingOption   # ✅ CommonType 네임스페이스
from generator.message0901_generator import make_msg0901_body
import json

def _dict_to_obj(body: dict) -> RequestOptionInfo:
    req = RequestOptionInfo()

    # Top-level
    req.timestamp   = int(body["timestamp"])
    # source(string) – 대/소문자 속성명 양쪽 대비
    if hasattr(req, "source"):
        req.source = str(body.get("source", ""))
    elif hasattr(req, "Source"):
        req.Source = str(body.get("source", ""))

    req.requestTime = int(body["requestTime"])

    # pendingOptionList (List<CommonType.PendingOption>)
    lst = List[PendingOption]()
    for itm in body.get("pendingOptionList", []):
        po = PendingOption()
        # 속성명(소/대문자) 호환 세팅
        if hasattr(po, "optionID"):      po.optionID      = int(itm["optionID"])
        elif hasattr(po, "OptionID"):    po.OptionID      = int(itm["optionID"])

        if hasattr(po, "optionName"):    po.optionName    = str(itm["optionName"])
        elif hasattr(po, "OptionName"):  po.OptionName    = str(itm["optionName"])

        if hasattr(po, "missionPlanID"): po.missionPlanID = int(itm["missionPlanID"])
        elif hasattr(po, "MissionPlanID"): po.MissionPlanID = int(itm["missionPlanID"])

        lst.Add(po)

    # 속성명(소/대문자) 호환 세팅
    if hasattr(req, "pendingOptionList"):
        req.pendingOptionList = lst
    elif hasattr(req, "PendingOptionList"):
        req.PendingOptionList = lst

    return req

def make_and_push(body_dict: dict, node_messenger) -> bytes:
    msg = _dict_to_obj(body_dict)
    node_messenger.Push(msg)
    log_line = (
        f"[0901] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0901] PUSH 완료"
    )
    return log_line.encode("utf-8", "ignore")

def make_random_and_push(node_messenger) -> bytes:
    return make_and_push(make_msg0901_body(), node_messenger)
