# receive/message0702_receiver.py
# ──────────────────────────────────────────────────────────────
from dll_files.nFusionImports import *            # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0702 import *              # MissionProgress …
from .database import received_db
from receive_center import notify
import json, traceback, sys

# ────────── 대/소문자 안전 접근 헬퍼 ──────────
_get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)

# ────────── CLR → dict 변환 ──────────
def _mission_progress_to_dict(mp: PilotDecision) -> dict:
    return {
        "timestamp":     _get(mp, "timestamp",     "Timestamp"),
        "ignore":        _get(mp, "ignore",        "Ignore"),
        "missionPlanID": _get(mp, "missionPlanID", "MissionPlanID")
    }

# ────────── Receiver 클래스 ──────────
class MissionProgressReceiver_0702(
    IFusionReceive[PilotDecision], IsLocal, IsSingletone
):
    """0702 MissionProgress 메시지 수신 리시버"""
    __namespace__ = "MissionProgressReceiver_0702"

    def Receive(self, data: PilotDecision, src):
        try:
            # 1) DB 저장
            received_db.set_received_0702(data)

            # 2) GUI에 JSON 바디 형태로 전달
            notify(
                "0702",
                json.dumps(_mission_progress_to_dict(data), ensure_ascii=False).encode()
            )

        except Exception:
            print("[ERROR][Receive-0702] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
