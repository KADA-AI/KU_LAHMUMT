# modules/common/push/message0504_push.py
# FuelWarning (0504) push helper with .NET integration

import json
from datetime import datetime, timezone

from System import UInt32, UInt64  # type: ignore

from nFusion.Model.msg_0504 import FuelWarning  # type: ignore
from nFusion.Model.CommonType import *  # type: ignore  # noqa: F401,F403

from modules.common.source_utils import get_default_source_code, override_source_fields
MSG_ID = "0504"

_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)

def _now_ms() -> int:
    return int((datetime.now(timezone.utc) - _EPOCH_2000).total_seconds() * 1000)


def _try_set(obj, name: str, value) -> bool:
    for attr in (name, name[:1].upper() + name[1:] if name else name):
        try:
            if hasattr(obj, attr):
                setattr(obj, attr, value)
                return True
        except Exception:
            pass
    return False


def _as_uint32(value, default: int = 0) -> UInt32:
    try:
        v = int(value)
    except Exception:
        v = int(default)
    if v < 0:
        v = 0
    return UInt32(max(0, v))


def _as_uint64(value, default: int = 0) -> UInt64:
    try:
        v = int(value)
    except Exception:
        v = int(default)
    if v < 0:
        v = 0
    return UInt64(v)


def _normalize_body(body_dict: dict | None) -> dict:
    data = dict(body_dict or {})

    ts = data.get("timestamp")
    try:
        timestamp = int(ts)
    except Exception:
        timestamp = _now_ms()
    if timestamp < 0:
        timestamp = 0

    source_val = data.get("source") or data.get("Source") or data.get("requestModuleName") or "MSM"
    source = str(source_val).strip() or "MSM"

    try:
        aircraft_id = int(data.get("aircraftID", data.get("AircraftID", 0)))
    except Exception:
        aircraft_id = 0
    if aircraft_id < 0:
        aircraft_id = 0

    try:
        fuel_level = int(data.get("fuelLevel", data.get("FuelLevel", 1)))
    except Exception:
        fuel_level = 1
    if fuel_level < 0:
        fuel_level = 0
    if fuel_level not in (0, 1, 2):
        fuel_level = 1

    return {
        "timestamp": timestamp,
        "source": source,
        "aircraftID": aircraft_id,
        "fuelLevel": fuel_level,
    }


def _dict_to_obj(body_dict: dict) -> FuelWarning:
    obj = FuelWarning()
    _try_set(obj, "timestamp", _as_uint64(body_dict.get("timestamp", _now_ms())))
    src = body_dict.get("source") or body_dict.get("Source") or "MSM"
    if src:
        _try_set(obj, "source", str(src))
    _try_set(obj, "aircraftID", _as_uint32(body_dict.get("aircraftID", 0)))
    _try_set(obj, "fuelLevel", _as_uint32(body_dict.get("fuelLevel", 1)))
    return obj


def make_and_push(body_dict: dict, node_messenger) -> bytes:
    body = _normalize_body(body_dict)
    msg = _dict_to_obj(body)
    node_messenger.Push(msg)

    body_json = json.dumps(body, ensure_ascii=False)
    log_line = f"[{MSG_ID}] BODY  : {body_json}\n[{MSG_ID}] PUSH OK"

    try:
        from receive_center import notify  # type: ignore
    except Exception:
        notify = None  # type: ignore
    if notify:
        try:
            notify(MSG_ID, body_json.encode("utf-8", "ignore"))
        except Exception:
            pass

    return log_line.encode("utf-8", "ignore")


def make_random_and_push(node_messenger) -> bytes:
    source = get_default_source_code()
    from generator.message0504_generator import make_msg0504_body

    return make_and_push(make_msg0504_body(), node_messenger)
