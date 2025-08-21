# receive/message0901_receiver.py
# ──────────────────────────────────────────────────────────────
from dll_files.nFusionImports import *            # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0901 import *              # RequestOptionInfo, Option …
from .database import received_db
from receive_center import notify
import json, traceback, sys

# ────────── 대/소문자 안전 접근 헬퍼 ──────────
_get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)

# ────────── CLR → dict 변환 ──────────
def _request_option_info_to_dict(req: RequestOptionInfo) -> dict:
    body = {
        "timestamp":   _get(req, "timestamp",   "Timestamp"),
        "requestTime": _get(req, "requestTime", "RequestTime"),
        "optionList":  []
    }

    for itm in (_get(req, "optionList", "OptionList") or []):
        option_dict = {
            "optionID":      _get(itm, "optionID",      "OptionID"),
            "optionName":    _get(itm, "optionName",    "OptionName"),
            "missionPlanID": _get(itm, "missionPlanID", "MissionPlanID")
        }
        body["optionList"].append(option_dict)

    return body

# ────────── Receiver 클래스 ──────────
class RequestOptionInfoReceiver_0901(
    IFusionReceive[RequestOptionInfo], IsLocal, IsSingletone
):
    """0901 RequestOptionInfo 메시지 수신 리시버"""
    __namespace__ = "RequestOptionInfoReceiver_0901"

    def Receive(self, data: RequestOptionInfo, src):
        try:
            # 1) DB 저장
            received_db.set_received_0901(data)

            # 2) GUI에 JSON 바디 형태로 전달
            notify(
                "0901",
                json.dumps(_request_option_info_to_dict(data), ensure_ascii=False).encode()
            )

        except Exception:
            print("[ERROR][Receive-0901] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
