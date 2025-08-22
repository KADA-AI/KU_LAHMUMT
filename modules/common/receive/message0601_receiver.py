# receive/message0601_receiver.py
# ─────────────────────────────────────────────────────────────
from dll_files.nFusionImports import *            # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0601 import BasicAction
from .database import received_db
from receive_center import notify
import json, traceback, sys

# 대/소문자 안전 접근
def _get(obj, *names):
    for n in names:
        if hasattr(obj, n):
            return getattr(obj, n)
    return None

# CLR → dict
def _basic_action_to_dict(ua: BasicAction) -> dict:
    return {
        "timestamp":   _get(ua, "timestamp",  "Timestamp"),
        "aircraft":    _get(ua, "aircraft",   "Aircraft"),
        "flightMode":  _get(ua, "flightMode", "FlightMode"),
        "filmingMode": _get(ua, "filmingMode","FilmingMode"),
    }

class BasicActionReceiver_0601(
    IFusionReceive[BasicAction], IsLocal, IsSingletone
):
    """0601 BasicAction 메시지 수신 리시버"""
    __namespace__ = "BasicActionReceiver_0601"

    def Receive(self, data: BasicAction, src):
        try:
            # 1) DB 저장
            received_db.set_received_0601(data)

            # 2) GUI 알림(JSON 바디)
            notify(
                "0601",
                json.dumps(_basic_action_to_dict(data), ensure_ascii=False).encode("utf-8", "ignore")
            )

        except Exception:
            print("[ERROR][Receive-0601] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
