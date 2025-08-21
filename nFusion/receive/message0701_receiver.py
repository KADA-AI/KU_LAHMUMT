# receive/message0701_receiver.py
# ──────────────────────────────────────────────────────────────
from dll_files.nFusionImports import *            # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0701 import *              # MissionPlanOptionInfo, Option …
from .database import received_db
from receive_center import notify
import json, traceback, sys

# ────────── 대/소문자 헬퍼 ──────────
_get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)

# ────────── CLR → dict 변환 ──────────
def _mpoi_to_dict(mpoi: MissionPlanOptionInfo) -> dict:
    body = {
        "timestamp":     _get(mpoi, "timestamp",     "Timestamp"),
        "autoExecution": _get(mpoi, "autoExecution", "AutoExecution"),
        "optionList":    []
    }

    for opt in (_get(mpoi, "optionList", "OptionList") or []):
        opt_dict = {
            "optionID":           _get(opt, "optionID",           "OptionID"),
            "optionName":         _get(opt, "optionName",         "OptionName"),
            "survivalRate":       _get(opt, "survivalRate",       "SurvivalRate"),
            "timeContraction":    _get(opt, "timeContraction",    "TimeContraction"),
            "recogEffectiveness": _get(opt, "recogEffectiveness", "RecogEffectiveness"),
            "distance":           _get(opt, "distance",           "Distance"),
            "target":             _get(opt, "target",             "Target"),
            "uavMissionPlanIDList": [],
            "lahMissionPlanIDList": []
        }

        # UAV 리스트
        for u in (_get(opt, "uavMissionPlanIDList", "UavMissionPlanIDList") or []):
            opt_dict["uavMissionPlanIDList"].append({
                "uavMissionPlanID": _get(u, "uavMissionPlanID", "UavMissionPlanID")
            })

        # LAH 리스트
        for l in (_get(opt, "lahMissionPlanIDList", "LahMissionPlanIDList") or []):
            opt_dict["lahMissionPlanIDList"].append({
                "lahMissionPlanID": _get(l, "lahMissionPlanID", "LahMissionPlanID")
            })

        body["optionList"].append(opt_dict)

    return body

# ────────── Receiver 클래스 ──────────
class MissionPlanOptionReceiver_0701(
    IFusionReceive[MissionPlanOptionInfo], IsLocal, IsSingletone
):
    """0701 MissionPlanOptionInfo 메시지 수신 리시버"""
    __namespace__ = "MissionPlanOptionReceiver_0701"

    def Receive(self, data: MissionPlanOptionInfo, src):
        try:
            # 1) DB 저장
            received_db.set_received_0701(data)

            # 2) GUI 알림 (JSON 바디)
            notify(
                "0701",
                json.dumps(_mpoi_to_dict(data), ensure_ascii=False).encode()
            )
        except Exception:
            print("[ERROR][Receive-0701] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
