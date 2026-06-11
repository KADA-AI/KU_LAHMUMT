# modules/common/push/message0104_push.py

import importlib
import json
from datetime import datetime, timezone

from nFusion.Model.msg_0104 import *
from nFusion.Model.CommonType import *
from System import String, UInt32, UInt64

from generator.message0104_generator import make_msg0104_body
from modules.common.source_utils import get_default_source_code, override_source_fields

MSG_ID = "0104"
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)


def _now_ms() -> int:
    return int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000)


def _try_set(obj, name: str, value) -> bool:
    for k in (name, name[:1].upper() + name[1:] if name else name):
        try:
            if hasattr(obj, k):
                setattr(obj, k, value)
                return True
        except Exception:
            pass
    return False


def _cs(name: str):
    t = globals().get(name)
    if t is not None:
        return t
    for modname in (f"nFusion.Model.msg_{MSG_ID}", "nFusion.Model.CommonType", "nFusion.Model"):
        try:
            mod = importlib.import_module(modname)
            t = getattr(mod, name, None)
            if t is not None:
                return t
        except Exception:
            pass
    return None


def _new(name: str):
    t = _cs(name)
    if t is None:
        raise NameError(f"type not found: {name}")
    return t()


def _dict_to_TotalStatus(data: dict):
    obj = _new("TotalStatus")
    if "timestamp" in data:
        _try_set(obj, "timestamp", int(data["timestamp"]))
    val_src = data.get("source", data.get("Source", data.get("requestModuleName", "")))
    if val_src != "":
        if not _try_set(obj, "source", str(val_src)):
            _try_set(obj, "Source", str(val_src))
    if "aircraftID" in data:
        _try_set(obj, "aircraftID", int(data["aircraftID"]))
    if "mode" in data:
        _try_set(obj, "mode", int(data["mode"]))
    if "SBC1Status" in data:
        _try_set(obj, "SBC1Status", int(data["SBC1Status"]))
    if "SBC2Status" in data:
        _try_set(obj, "SBC2Status", int(data["SBC2Status"]))
    if "SBC3Status" in data:
        _try_set(obj, "SBC3Status", int(data["SBC3Status"]))
    return obj


def _dict_to_obj(body_dict: dict):
    return _dict_to_TotalStatus(body_dict)


def make_and_push(body_dict: dict, node_messenger) -> bytes:
    msg = _dict_to_obj(body_dict)
    node_messenger.Push(msg)
    log_line = (
        f"[0104] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0104] PUSH 완료"
    )
    return log_line.encode("utf-8", "ignore")


def make_random_and_push(node_messenger) -> bytes:
    source = get_default_source_code()
    body = make_msg0104_body()
    override_source_fields(body, source)
    if not isinstance(body, dict):
        body = {}
    body.setdefault("timestamp", _now_ms())
    body.setdefault("source", source)
    body.setdefault("aircraftID", 1)
    body.setdefault("mode", 1)
    body.setdefault("SBC1Status", 1)
    body.setdefault("SBC2Status", 0)
    body.setdefault("SBC3Status", 0)
    return make_and_push(body, node_messenger)
