# receive/message0305_receiver.py
# ─────────────────────────────────────────────────────────────
from dll_files.nFusionImports import *            # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0305 import ReplanStatus
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
def _replan_status_to_dict(rs: ReplanStatus) -> dict:
    return {
        "timestamp":             _get(rs, "timestamp", "Timestamp"),
        "missionPlanningStatus": _get(rs, "missionPlanningStatus", "MissionPlanningStatus"),
        "replanReason":          _get(rs, "replanReason", "ReplanReason"),
    }

class ReplanStatusReceiver_0305(
    IFusionReceive[ReplanStatus], IsLocal, IsSingletone
):
    """0305 ReplanStatus 메시지 수신 리시버"""
    __namespace__ = "ReplanStatusReceiver_0305"

    def Receive(self, data: ReplanStatus, src):
        try:
            # 1) DB 저장
            received_db.set_received_0305(data)

            # 2) GUI 알림(JSON 바디)
            notify(
                "0305",
                json.dumps(_replan_status_to_dict(data), ensure_ascii=False).encode("utf-8", "ignore")
            )

        except Exception:
            print("[ERROR][Receive-0305] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
