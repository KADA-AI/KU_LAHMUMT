# receive/message0802_receiver.py
# ──────────────────────────────────────────────────────────────
from dll_files.nFusionImports import *            # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0802 import MandatoryCommand  # MandatoryCommand …
from .database import received_db
from receive_center import notify
import json, traceback, sys

# ────────── 대/소문자 안전 접근 헬퍼 ──────────
_get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)

# ────────── CLR → dict 변환 ──────────
def _mandatory_command_to_dict(cmd: MandatoryCommand) -> dict:
    return {
        "timestamp":     _get(cmd, "timestamp",     "Timestamp"),
        "aircraftID":    _get(cmd, "aircraftID",    "AircraftID"),
        "mandatoryType": _get(cmd, "mandatoryType", "MandatoryType")
    }

# ────────── Receiver 클래스 ──────────
class MandatoryCommandReceiver_0802(
    IFusionReceive[MandatoryCommand], IsLocal, IsSingletone
):
    """0802 MandatoryCommand 메시지 수신 리시버"""
    __namespace__ = "MandatoryCommandReceiver_0802"

    def Receive(self, data: MandatoryCommand, src):
        try:
            # 1) DB 저장
            received_db.set_received_0802(data)

            # 2) GUI에 JSON 바디 형태로 전달
            notify(
                "0802",
                json.dumps(_mandatory_command_to_dict(data), ensure_ascii=False).encode()
            )

        except Exception:
            print("[ERROR][Receive-0802] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
