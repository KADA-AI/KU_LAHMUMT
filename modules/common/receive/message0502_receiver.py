# receive/message0502_receiver.py
# ─────────────────────────────────────────────────────────────
from dll_files.nFusionImports import *            # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0502 import EndMissionRequest
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
def _end_mission_request_to_dict(req: EndMissionRequest) -> dict:
    return {
        "timestamp": _get(req, "timestamp", "Timestamp"),  # ms since 2000-01-01
    }

# ────────── Receiver ──────────
class EndMissionRequestReceiver_0502(
    IFusionReceive[EndMissionRequest], IsLocal, IsSingletone
):
    """0502 EndMissionRequest 메시지 수신 리시버"""
    __namespace__ = "EndMissionRequestReceiver_0502"

    def Receive(self, data: EndMissionRequest, src):
        try:
            # 1) DB 저장
            received_db.set_received_0502(data)

            # 2) GUI 알림(JSON 바디)
            notify(
                "0502",
                json.dumps(_end_mission_request_to_dict(data), ensure_ascii=False).encode("utf-8", "ignore")
            )

        except Exception:
            print("[ERROR][Receive-0502] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
