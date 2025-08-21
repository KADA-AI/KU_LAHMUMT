# receive/message0903_receiver.py
# ──────────────────────────────────────────────────────────────
from dll_files.nFusionImports import *            # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0903 import RequestRenewMission
from .database import received_db
from receive_center import notify
import json, traceback, sys

# ────────── 대/소문자 안전 접근 헬퍼 ──────────
_get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)

# ────────── CLR → dict 변환 ──────────
def _request_renew_mission_to_dict(req: RequestRenewMission) -> dict:
    return {
        "timestamp":     _get(req, "timestamp",     "Timestamp"),
        "missionPlanID": _get(req, "missionPlanID", "MissionPlanID")
    }

# ────────── Receiver 클래스 ──────────
class RequestRenewMissionReceiver_0903(
    IFusionReceive[RequestRenewMission], IsLocal, IsSingletone
):
    """0903 RequestRenewMission 메시지 수신 리시버"""
    __namespace__ = "RequestRenewMissionReceiver_0903"

    def Receive(self, data: RequestRenewMission, src):
        try:
            # 1) DB 저장
            received_db.set_received_0903(data)

            # 2) GUI에 JSON 바디 형태로 전달
            notify(
                "0903",
                json.dumps(
                    _request_renew_mission_to_dict(data),
                    ensure_ascii=False
                ).encode()
            )

        except Exception:
            print("[ERROR][Receive-0903] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
