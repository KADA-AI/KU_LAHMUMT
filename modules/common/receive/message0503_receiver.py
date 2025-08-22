from dll_files.nFusionImports import *
from nFusion.Model.msg_0503 import MissionResult
from .database import received_db
from receive_center import notify
import json, traceback, sys

_get = lambda obj,*names: next((getattr(obj,n) for n in names if hasattr(obj,n)),None)

def _mission_result_to_dict(mr: MissionResult) -> dict:
    return {
        "timestamp":       _get(mr, "timestamp","Timestamp"),
        "source":          _get(mr, "source","Source"),
        "systemRecommend": _get(mr, "systemRecommend","SystemRecommend"),
    }

class MissionResultReceiver_0503(
    IFusionReceive[MissionResult], IsLocal, IsSingletone
):
    """0503 MissionResult 메시지 수신 리시버"""
    __namespace__ = "MissionResultReceiver_0503"

    def Receive(self, data: MissionResult, src):
        try:
            received_db.set_received_0503(data)
            notify(
                "0503",
                json.dumps(_mission_result_to_dict(data), ensure_ascii=False).encode()
            )
        except Exception:
            print("[ERROR][Receive-0503] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
