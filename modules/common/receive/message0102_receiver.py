# receive/message0102_receiver.py
# ──────────────────────────────────────────────────────────────
from dll_files.nFusionImports import *             # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0102 import *    # ModuleStatus …
from .database import received_db
from receive_center import notify
import json, traceback, sys

# ────────── 안전 접근 헬퍼 ──────────
def _get(obj, *names):
    for n in names:
        if hasattr(obj, n):
            return getattr(obj, n)
    return None

# ────────── CLR → dict 변환 ──────────
def _to_dict(ms: ModuleStatus) -> dict:
    return {
        "timestamp": _get(ms, "timestamp", "Timestamp"),
        # 외부 노출은 항상 'source' 키로 표준화
        "source":    _get(ms, "source", "Source", "sourceModuleName", "SourceModuleName", "requestModuleName", "RequestModuleName"),
        "status":    _get(ms, "status", "Status"),
    }

# ────────── Receiver ──────────
class ModuleStatusReceiver_0102(
    IFusionReceive[ModuleStatus], IsLocal, IsSingletone
):
    """0102 ModuleStatus 메시지 수신 리시버"""
    __namespace__ = "ModuleStatusReceiver_0102"

    def Receive(self, data: ModuleStatus, src):
        try:
            # DB 저장
            received_db.set_received_0102(data)

            # GUI 알림 (표준 키: timestamp, source, status)
            notify(
                "0102",
                json.dumps(_to_dict(data), ensure_ascii=False).encode("utf-8", "ignore")
            )

        except Exception:
            print("[ERROR][Receive-0102] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
