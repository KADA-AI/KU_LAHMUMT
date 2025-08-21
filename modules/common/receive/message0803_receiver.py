# receive/message0803_receiver.py
# ──────────────────────────────────────────────────────────────
from dll_files.nFusionImports import *              # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0803 import ExecutionCommand
from .database import received_db
from receive_center import notify
import json, traceback, sys

# ────────── 대/소문자 안전 접근 헬퍼 ──────────
_get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)

# ────────── CLR → dict 변환 ──────────
def _start_next_mission_command_to_dict(cmd: ExecutionCommand) -> dict:
    return {
        "timestamp": _get(cmd, "timestamp", "Timestamp")
    }

# ────────── Receiver 클래스 ──────────
class StartNextMissionCommandReceiver_0803(
    IFusionReceive[ExecutionCommand], IsLocal, IsSingletone
):
    """0803 StartNextMissionCommand 메시지 수신 리시버"""
    __namespace__ = "StartNextMissionCommandReceiver_0803"

    def Receive(self, data: ExecutionCommand, src):
        try:
            # 1) DB 저장
            received_db.set_received_0803(data)

            # 2) GUI에 JSON 바디 형태로 전달
            notify(
                "0803",
                json.dumps(
                    _start_next_mission_command_to_dict(data),
                    ensure_ascii=False
                ).encode()
            )

        except Exception:
            print("[ERROR][Receive-0803] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
