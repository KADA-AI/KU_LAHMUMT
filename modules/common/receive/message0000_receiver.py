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
def _response_to_dict(resp) -> dict:
    # 대/소문자·이름 변동 안전 접근
    def _get(obj, *names):
        for n in names:
            if hasattr(obj, n):
                return getattr(obj, n)
        return None

    return {
        "timestamp": _get(resp, "timestamp", "Timestamp"),
        "source": _get(resp, "source", "Source", "requestModuleName", "RequestModuleName"),
        "messageID": _get(resp, "messageID", "MessageID"),
    }

# ────────── Receiver ──────────
class ResponseReceiver_0000(                       # import 시 이 이름을 사용
    IFusionReceive[RequestData], IsLocal, IsSingletone
):
    """0000 Response 메시지 수신 리시버"""
    __namespace__ = "ResponseReceiver_0000"

    def Receive(self, data, src):
        try:
            # 1) DB 저장 (원본 객체로 보존)
            received_db.set_received_0000(data)

            # 2) GUI 알림: 항상 JSON 바디는 표준 키(timestamp, source, messageID)
            import json
            notify("0000", json.dumps(_response_to_dict(data), ensure_ascii=False).encode("utf-8", "ignore"))

        except Exception:
            import traceback, sys
            print("[ERROR][Receive-0000] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)