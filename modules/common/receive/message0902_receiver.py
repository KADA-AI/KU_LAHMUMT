# receive/message0902_receiver.py
# ──────────────────────────────────────────────────────────────
from dll_files.nFusionImports import *               # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0902 import *                 # ReplanRequest, ReplanRequestTime, etc.
from .database import received_db
from receive_center import notify
import json, traceback, sys

# ────────── 대/소문자 안전 접근 헬퍼 ──────────
_get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)

# ────────── CLR → dict 변환 ──────────
def _replan_request_to_dict(req: ReplanRequest) -> dict:
    body = {
        "timestamp":    _get(req, "timestamp",    "Timestamp"),
        "replanLevel":  _get(req, "replanLevel",  "ReplanLevel"),
        "replanReason": _get(req, "replanReason", "ReplanReason"),
        "replanRequestTime": {
            "replanRequestTimestamp": _get(
                _get(req, "replanRequestTime", "ReplanRequestTime"),
                "replanRequestTimestamp", "ReplanRequestTimestamp"
            )
        },
        "inputMissionIDList": [
            {"inputMissionID": _get(im, "inputMissionID", "InputMissionID")}
            for im in (_get(req, "InputMissionIDList", "inputMissionIDList") or [])
        ],
        "individualMissionIDList": [
            {"individualMissionID": _get(im, "individualMissionID", "IndividualMissionID")}
            for im in (_get(req, "individualMissionIDList", "IndividualMissionIDList") or [])
        ],
        "priorMissionList": [
            {"priorMissionID": _get(pm, "priorMissionID", "PriorMissionID")}
            for pm in (_get(req, "priorMissionList", "PriorMissionList") or [])
        ],
        "optionList": []
    }

    for op in (_get(req, "OptionList", "optionList") or []):
        body["optionList"].append({
            "optionID":      _get(op, "optionID",      "OptionID"),
            "optionName":    _get(op, "optionName",    "OptionName"),
            "missionPlanID": _get(op, "missionPlanID", "MissionPlanID")
        })

    return body

# ────────── Receiver ──────────
class ReplanRequestReceiver_0902(
    IFusionReceive[ReplanRequest], IsLocal, IsSingletone
):
    """0902 ReplanRequest 메시지 수신 리시버"""
    __namespace__ = "ReplanRequestReceiver_0902"

    def Receive(self, data: ReplanRequest, src):
        try:
            # 1) DB 저장
            received_db.set_received_0902(data)

            # 2) GUI 알림
            notify(
                "0902",
                json.dumps(_replan_request_to_dict(data), ensure_ascii=False).encode()
            )
        except Exception:
            print("[ERROR][Receive-0902] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
