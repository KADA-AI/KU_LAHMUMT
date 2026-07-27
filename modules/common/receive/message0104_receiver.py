# modules/common/receive/message0104_receiver.py

import json
import sys
import traceback

from dll_files.nFusionImports import *
from nFusion.Model.msg_0104 import *
from nFusion.Model.CommonType import *

from .database import received_db
from receive_center import notify

_get = lambda obj, *names: next((getattr(obj, n) for n in names if hasattr(obj, n)), None)


def _try_save_received(msgid: str, data_obj):
    try:
        fn = getattr(received_db, f"set_received_{msgid}")
        fn(data_obj)
    except Exception:
        pass


def _to_dict_TotalStatus(obj):
    d = {}
    _v = _get(obj, "timestamp", "Timestamp")
    if _v is not None:
        d["timestamp"] = int(_v)
    _sval = _get(obj, "source", "Source", "requestModuleName", "RequestModuleName")
    if _sval is not None and _sval != "":
        d["source"] = str(_sval)
    _v = _get(obj, "aircraftID", "AircraftID")
    if _v is not None:
        d["aircraftID"] = int(_v)
    _v = _get(obj, "mode", "Mode")
    if _v is not None:
        d["mode"] = int(_v)
    _v = _get(obj, "SBC1Status")
    if _v is not None:
        d["SBC1Status"] = int(_v)
    _v = _get(obj, "SBC2Status")
    if _v is not None:
        d["SBC2Status"] = int(_v)
    _v = _get(obj, "SBC3Status")
    if _v is not None:
        d["SBC3Status"] = int(_v)
    return d


class TotalStatusReceiver_0104(IFusionReceive[TotalStatus], IsLocal, IsSingletone):
    """0104 TotalStatus 메시지 수신 리시버"""

    __namespace__ = "TotalStatusReceiver_0104"

    def Receive(self, data: TotalStatus, src):
        try:
            _try_save_received("0104", data)
            body = _to_dict_TotalStatus(data)
            notify("0104", json.dumps(body, ensure_ascii=False).encode("utf-8", "ignore"))
        except Exception:
            print("[ERROR][Receive-0104] traceback ↓↓↓")
            traceback.print_exc(file=sys.stderr)
