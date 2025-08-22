# receive/message0101_receiver.py
# ──────────────────────────────────────────────────────────────
from dll_files.nFusionImports import *             # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0101 import *               # SystemOperationMode …
from .database import received_db
from receive_center import notify
import json, traceback, sys

# ────────── 대/소문자 안전 접근 헬퍼 ──────────
def _get(obj, *names):
    for n in names:
        if hasattr(obj, n):
            return getattr(obj, n)
    return None

# ────────── CLR → dict 변환 ──────────
def _som_to_dict(som: SystemOperationMode) -> dict:
    return {
        "timestamp":  _get(som, "timestamp",  "Timestamp"),
        # 외부 노출은 항상 'source' 키로 표준화 (DLL 내부 속성명 변동 대응)
        "source":     _get(som, "source", "Source", "requestModuleName", "RequestModuleName"),
        "systemMode": _get(som, "systemMode", "SystemMode"),
    }

# ────────── Receiver ──────────
class SystemOperationModeReceiver_0101(
    IFusionReceive[SystemOperationMode], IsLocal, IsSingletone
):
    """0101 SystemOperationMode 메시지 수신 리시버"""
    __namespace__ = "SystemOperationModeReceiver_0101"

    def Receive(self, data: SystemOperationMode, src):
        try:
            # DB 저장
            received_db.set_received_0101(data)

            # GUI 알림 (표준 키: timestamp, source, systemMode)
            notify(
                "0101",
                json.dumps(_som_to_dict(data), ensure_ascii=False).encode("utf-8", "ignore")
            )

        except Exception:
            print("[ERROR][Receive-0101] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
