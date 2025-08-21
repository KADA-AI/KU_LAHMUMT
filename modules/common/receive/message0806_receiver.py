# receive/message0806_receiver.py
# ──────────────────────────────────────────────────────────────
from dll_files.nFusionImports import *            # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0806 import BootCommand    # EndSWCommand …
from .database import received_db
from receive_center import notify
import json, traceback, sys

# ────────── 대/소문자 안전 접근 헬퍼 ──────────
_get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)

# ────────── CLR → dict 변환 ──────────
def _end_sw_command_to_dict(cmd: BootCommand) -> dict:
    return {
        "timestamp": _get(cmd, "timestamp", "Timestamp"),
        "command":   _get(cmd, "command",   "Command")
    }

# ────────── Receiver 클래스 ──────────
class EndSWCommandReceiver_0806(
    IFusionReceive[BootCommand], IsLocal, IsSingletone
):
    """0806 EndSWCommand 메시지 수신 리시버"""
    __namespace__ = "EndSWCommandReceiver_0806"

    def Receive(self, data: BootCommand, src):
        try:
            # 1) DB 저장
            received_db.set_received_0806(data)

            # 2) GUI에 JSON 바디 형태로 전달
            notify(
                "0806",
                json.dumps(
                    _end_sw_command_to_dict(data),
                    ensure_ascii=False
                ).encode()
            )

        except Exception:
            print("[ERROR][Receive-0806] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
