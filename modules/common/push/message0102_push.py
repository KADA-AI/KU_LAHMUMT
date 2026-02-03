# modules/common/push/message0102_push.py
# auto-fixed at 2025-09-19

import json, importlib
from datetime import datetime, timezone
from System.Collections.Generic import List
from nFusion.Model.msg_0102 import *    # C# 모델
from nFusion.Model.CommonType import *  # 공통 타입
from System import String, UInt32, UInt64

from modules.common.source_utils import get_default_source_code, override_source_fields
MSG_ID = "0102"
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms = lambda: int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000)

def _try_set(obj, name: str, value) -> bool:
    for k in (name, name[:1].upper()+name[1:] if name else name):
        try:
            if hasattr(obj, k):
                setattr(obj, k, value)
                return True
        except Exception:
            pass
    return False

def _cs(name: str):
    t = globals().get(name)
    if t is not None: return t
    for modname in (f'nFusion.Model.msg_{MSG_ID}', 'nFusion.Model.CommonType', 'nFusion.Model'):
        try:
            mod = importlib.import_module(modname)
            t = getattr(mod, name, None)
            if t is not None: return t
        except Exception:
            pass
    return None

def _new(name: str):
    t = _cs(name)
    if t is None:
        raise NameError(f'type not found: {name}')
    return t()

def _normalize_body_keys(body: dict) -> dict:
    """
    들어온 바디를 0102 표준 키로 정규화:
      Timestamp/timestamp -> timestamp
      Status/status       -> status
      Source/RequestModuleName/... -> source
    (값은 가공하지 않음)
    """
    if not isinstance(body, dict):
        return {}

    out = {}
    # timestamp
    for k in ("timestamp","Timestamp","timeStamp","TimeStamp","ts","TS"):
        if k in body:
            try: out["timestamp"] = int(body[k]); break
            except Exception: pass

    # status
    for k in ("status","Status","moduleStatus","ModuleStatus","state","State","healthStatus","HealthStatus"):
        if k in body:
            try: out["status"] = int(body[k]); break
            except Exception: pass

    # source (SourceModuleName 폐기 → source로 통일)
    for k in ("source","Source","requestModuleName","RequestModuleName","module","Module","sourceModule","SourceModule"):
        if k in body:
            v = str(body[k])
            if v:
                out["source"] = v
                break

    return out

def _dict_to_ModuleStatus(data: dict):
    obj = _new('ModuleStatus')
    # 표준 키 기준으로 세팅 (필드명 대소문자 모두 시도)
    if "timestamp" in data: _try_set(obj, "timestamp", int(data["timestamp"]))
    if "status"    in data: _try_set(obj, "status",    int(data["status"]))
    if "source"    in data:
        if not _try_set(obj, "source", str(data["source"])):
            _try_set(obj, "Source", str(data["source"]))
    return obj

def _dict_to_obj(body_dict: dict):
    return _dict_to_ModuleStatus(body_dict)

def make_and_push(body_dict: dict, node_messenger) -> bytes:
    # 0102는 화이트리스트 없음 → 먼저 표준 키로 정규화
    body_dict = _normalize_body_keys(body_dict)
    # 최소 세 키가 되도록(없어도 그대로 보냄: 가공/생성하지 않음)
    msg = _dict_to_obj(body_dict)
    node_messenger.Push(msg)
    # 로그에도 표준 키로 동일하게 출력
    log_line = (
        f"[0102] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0102] PUSH 완료"
    )
    return log_line.encode("utf-8", "ignore")

def make_random_and_push(node_messenger) -> bytes:
    source = get_default_source_code()
    """
    제너레이터가 비거나 누락해도 한 번 더 정규화하여
    BODY에 timestamp/status/source 3키가 표준 키로 찍히게 한다.
    """
    try:
        from generator.message0102_generator import make_msg0102_body
        raw = make_msg0102_body()
        override_source_fields(raw, source)
    except Exception:
        raw = {}

    # 폴백: 최소 세 키 (값 생성은 하지 않지만, 없으면 추가 시도)
    if not isinstance(raw, dict):
        raw = {}
    if "timestamp" not in raw and "Timestamp" not in raw:
        raw["timestamp"] = _now_ms()
    if "status" not in raw and "Status" not in raw:
        raw["status"] = 1
    if all(k not in raw for k in ("source","Source","requestModuleName","RequestModuleName")):
        raw["source"] = source

    return make_and_push(raw, node_messenger)
