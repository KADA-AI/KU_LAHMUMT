from dll_files.nFusionImports import *      # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0103 import *
from .database import received_db
from receive_center import notify
import json, traceback, sys

# 대/소문자 안전 접근
_get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)

def _swstatus_to_dict(sw: SWStatus) -> dict:
    return {
        "timestamp": _get(sw, "timestamp", "Timestamp"),
        "status":    _get(sw, "status", "Status"),
        "mode":      _get(sw, "mode", "Mode"),
    }

class SWStatusReceiver_0103(
    IFusionReceive[SWStatus], IsLocal, IsSingletone
):
    """0103 SWStatus 메시지 수신 리시버"""
    __namespace__ = "SWStatusReceiver_0103"

    def Receive(self, data: SWStatus, src):
        try:
            # DB 저장
            received_db.set_received_0103(data)

            # GUI 알림
            notify(
                "0103",
                json.dumps(_swstatus_to_dict(data), ensure_ascii=False).encode()
            )

        except Exception:
            print("[ERROR][Receive-0103] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
