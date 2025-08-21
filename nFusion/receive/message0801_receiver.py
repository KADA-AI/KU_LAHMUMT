# receive/message0801_receiver.py
# ──────────────────────────────────────────────────────────────
from dll_files.nFusionImports import *            # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0801 import *              # ReplanCommand …
from .database import received_db
from receive_center import notify
import json, traceback, sys

# ────────── 대/소문자 안전 접근 헬퍼 ──────────
_get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)

# ────────── CLR → dict 변환 ──────────
def _replan_command_to_dict(cmd: InitialPlanCommand) -> dict:
    return {
        "timestamp":                 _get(cmd, "timestamp",                 "Timestamp"),
        "operatorReplanRequestTime": _get(cmd, "operatorReplanRequestTime",  "OperatorReplanRequestTime"),
        "isOnGround":                _get(cmd, "isOnGround",                "IsOnGround"),
        "inputMissionPackageID":     _get(cmd, "inputMissionPackageID",     "InputMissionPackageID"),
        "missionReferencePackageID": _get(cmd, "missionReferencePackageID", "MissionReferencePackageID")
    }

# ────────── Receiver 클래스 ──────────
class ReplanCommandReceiver_0801(
    IFusionReceive[InitialPlanCommand], IsLocal, IsSingletone
):
    """0801 ReplanCommand 메시지 수신 리시버"""
    __namespace__ = "ReplanCommandReceiver_0801"

    def Receive(self, data: InitialPlanCommand, src):
        try:
            # 1) DB 저장
            received_db.set_received_0801(data)

            # 2) GUI에 JSON 바디 형태로 전달
            notify(
                "0801",
                json.dumps(_replan_command_to_dict(data), ensure_ascii=False).encode()
            )

        except Exception:
            print("[ERROR][Receive-0801] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
