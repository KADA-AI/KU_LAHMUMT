# receive/message0805_receiver.py
# ──────────────────────────────────────────────────────────────
from dll_files.nFusionImports import *              # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0805 import SystemEvent
from .database import received_db
from receive_center import notify
import json, traceback, sys

# ────────── 대/소문자 안전 접근 헬퍼 ──────────
_get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)

# ────────── CLR → dict 변환 ──────────
def _end_mission_command_to_dict(cmd: SystemEvent) -> dict:
    return {
        "timestamp": _get(cmd, "timestamp", "Timestamp")
    }

# ────────── Receiver 클래스 ──────────
class EndMissionCommandReceiver_0805(
    IFusionReceive[SystemEvent], IsLocal, IsSingletone
):
    """0805 EndMissionCommand 메시지 수신 리시버"""
    __namespace__ = "EndMissionCommandReceiver_0805"

    def Receive(self, data: SystemEvent, src):
        try:
            # 1) DB 저장
            received_db.set_received_0805(data)

            # 2) GUI에 JSON 바디 형태로 전달
            notify(
                "0805",
                json.dumps(
                    _end_mission_command_to_dict(data),
                    ensure_ascii=False
                ).encode()
            )

        except Exception:
            print("[ERROR][Receive-0805] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
