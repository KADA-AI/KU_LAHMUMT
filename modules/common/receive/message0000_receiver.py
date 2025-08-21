# receive/message0000_receiver.py
# ──────────────────────────────────────────────────────────────
from dll_files.nFusionImports import *            # IFusionReceive, IsLocal, IsSingletone
from nFusion.Model.msg_0000 import *              # Response …
from .database import received_db
from receive_center import notify
import json, traceback, sys

# ────────── 대/소문자 안전 접근 헬퍼 ──────────
_get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)

# ────────── CLR → dict 변환 ──────────
def _response_to_dict(resp: RequestData) -> dict:
    return {
        "timestamp":           _get(resp, "timestamp", "Timestamp"),
        "requestModuleName":   _get(resp, "requestModuleName", "RequestModuleName"),
        "messageID": _get(resp, "messageID", "MessageID")
    }

# ────────── Receiver ──────────
class ResponseReceiver_0000(                       # import 시 이 이름을 사용
    IFusionReceive[RequestData], IsLocal, IsSingletone
):
    """0000 Response 메시지 수신 리시버"""
    __namespace__ = "ResponseReceiver_0000"

    def Receive(self, data: RequestData, src):
        try:
            # DB 저장
            received_db.set_received_0000(data)

            # GUI 알림
            notify(
                "0000",
                json.dumps(_response_to_dict(data), ensure_ascii=False).encode()
            )

        except Exception:
            print("[ERROR][Receive-0000] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
