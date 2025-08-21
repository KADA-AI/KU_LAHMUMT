# receive/message0804_receiver.py
# ──────────────────────────────────────────────────────────────
from dll_files.nFusionImports import *              # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0804 import MissionRestartCommand
from .database import received_db
from receive_center import notify
import json, traceback, sys

# ────────── 대/소문자 안전 접근 헬퍼 ──────────
_get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)

# ────────── CLR → dict 변환 ──────────
def _mission_restart_command_to_dict(cmd: MissionRestartCommand) -> dict:
    return {
        "timestamp":     _get(cmd, "timestamp",   "Timestamp"),
        "inputMissionID": _get(cmd, "inputMissionID", "InputMissionID")
    }

# ────────── Receiver 클래스 ──────────
class MissionRestartCommandReceiver_0804(
    IFusionReceive[MissionRestartCommand], IsLocal, IsSingletone
):
    """0804 MissionRestartCommand 메시지 수신 리시버"""
    __namespace__ = "MissionRestartCommandReceiver_0804"

    def Receive(self, data: MissionRestartCommand, src):
        try:
            # 1) DB 저장
            received_db.set_received_0804(data)

            # 2) GUI에 JSON 바디 형태로 전달
            notify(
                "0804",
                json.dumps(
                    _mission_restart_command_to_dict(data),
                    ensure_ascii=False
                ).encode()
            )

        except Exception:
            print("[ERROR][Receive-0804] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
