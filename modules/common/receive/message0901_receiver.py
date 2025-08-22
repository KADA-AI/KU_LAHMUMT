# -*- coding: utf-8 -*-
# ──────────────────────────────────────────────────────────────
from dll_files.nFusionImports import *             # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0901 import RequestOptionInfo
from nFusion.Model.CommonType import PendingOption  # ✅ CommonType 네임스페이스
from .database import received_db
from receive_center import notify
import json, traceback, sys

# 안전 접근 헬퍼
def _get(obj, *names):
    for n in names:
        if hasattr(obj, n):
            return getattr(obj, n)
    return None

# CLR → dict
def _request_option_info_to_dict(req: RequestOptionInfo) -> dict:
    seq = _get(req, "pendingOptionList", "PendingOptionList") or []
    return {
        "timestamp":   _get(req, "timestamp",   "Timestamp"),
        "source":      _get(req, "source",      "Source"),
        "requestTime": _get(req, "requestTime", "RequestTime"),
        "pendingOptionList": [
            {
                "optionID":      _get(itm, "optionID",      "OptionID"),
                "optionName":    _get(itm, "optionName",    "OptionName"),
                "missionPlanID": _get(itm, "missionPlanID", "MissionPlanID"),
            } for itm in seq
        ],
    }

# Receiver
class RequestOptionInfoReceiver_0901(
    IFusionReceive[RequestOptionInfo], IsLocal, IsSingletone
):
    """0901 RequestOptionInfo 메시지 수신 리시버"""
    __namespace__ = "RequestOptionInfoReceiver_0901"

    def Receive(self, data: RequestOptionInfo, src):
        try:
            # 1) DB 저장
            received_db.set_received_0901(data)

            # 2) GUI로 JSON 바디 전달
            notify("0901", json.dumps(_request_option_info_to_dict(data), ensure_ascii=False).encode("utf-8", "ignore"))

        except Exception:
            print("[ERROR][Receive-0901] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
