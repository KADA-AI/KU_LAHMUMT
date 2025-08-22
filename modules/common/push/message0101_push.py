# generator/message0101_push.py
from nFusion.Model.msg_0101 import *  # SystemOperationMode
from generator.message0101_generator import make_msg0101_body  # 바디 생성기
import json

def _dict_to_obj(body_dict: dict):
    """
    dict(JSON 소문자 카멜) → SystemOperationMode(C# 객체)
    필드:
      - timestamp(ulong)
      - source(string)  ※ 환경에 따라 Source/requestModuleName/RequestModuleName 속성만 있는 DLL 대비
      - systemMode(uint)
    """
    obj = SystemOperationMode()

    # timestamp (ulong)
    obj.timestamp = int(body_dict["timestamp"])

    # source(string) 우선, 과거 호환키도 폴백
    val_source = body_dict.get("source", body_dict.get("requestModuleName", ""))

    # 다양한 속성명 호환 (소/대문자 및 과거 필드명)
    if hasattr(obj, "source"):
        setattr(obj, "source", val_source)
    elif hasattr(obj, "Source"):
        setattr(obj, "Source", val_source)
    elif hasattr(obj, "requestModuleName"):
        setattr(obj, "requestModuleName", val_source)
    elif hasattr(obj, "RequestModuleName"):
        setattr(obj, "RequestModuleName", val_source)

    # systemMode (uint)
    obj.systemMode = int(body_dict["systemMode"])
    return obj

def make_and_push(body_dict: dict, node_messenger) -> bytes:
    """생성된 dict를 모델 객체로 변환하여 Push"""
    msg = _dict_to_obj(body_dict)
    node_messenger.Push(msg)

    log_line = (
        f"[0101] BODY  : {json.dumps(body_dict, ensure_ascii=False)}\n"
        f"[0101] PUSH 완료"
    )
    return log_line.encode("utf-8", "ignore")

def make_random_and_push(node_messenger) -> bytes:
    """별도 입력 없이 랜덤/현재값으로 바디 생성 후 즉시 Push"""
    return make_and_push(make_msg0101_body(), node_messenger)
