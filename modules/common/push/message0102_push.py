# -*- coding: utf-8 -*-
# file: generator/message0102_push.py
"""
MSGID 0102 (운용모드 설정) 전송 유틸

요구사항:
- Timestamp: 2000-01-01 UTC 기준 ms
- Status 기본값: 1 (정상)
- SourceModuleName: KU_ROLE → 모듈명 매핑
- 텍스트/JSON 로그 및 폴백 전송 시 키 순서 고정:
  Timestamp → Status → SourceModuleName
- raw 로그에 내부 필드(sent 등) 포함 금지
"""

from nFusion.Model.msg_0102 import *           # ModuleStatus 등
from generator.message0102_generator import make_msg0102_body
import json
from datetime import datetime, timezone
from collections import OrderedDict

# ─────────────────────────────────────────────────────────────
# 2000-01-01 UTC 기준 ms
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms     = lambda: int(
    (datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000
)

def _default_source_module_name():
    """
    KU_ROLE → 모듈명 매핑
      monitoring → Mission State Monitor
      mission    → Multi-agent Mission Planner
      decision   → Mission Option Builder
    """
    import os
    role = (os.environ.get("KU_ROLE") or "").lower()
    return {
        "monitoring": "Mission State Monitor",
        "mission":    "Multi-agent Mission Planner",
        "decision":   "Mission Option Builder",
    }.get(role, "Multi-agent Mission Planner")


# ─────────────────────────────────────────────────────────────
def _dict_to_obj(body_dict: dict) -> ModuleStatus:
    """
    내부 표준 dict(JSON 소문자 키) → ModuleStatus(C# 객체)
    표준 키:
      - timestamp(ulong)
      - source(string)
      - status(uint)
    """
    obj = ModuleStatus()

    # timestamp (ulong)
    obj.timestamp = int(body_dict["timestamp"])

    # source(string): 다양한 프로퍼티 네이밍 호환
    val_source = body_dict.get("source", "")
    for name in ("source", "Source", "sourceModuleName", "SourceModuleName", "requestModuleName", "RequestModuleName"):
        if hasattr(obj, name):
            setattr(obj, name, val_source)
            break  # 한 군데만 세팅해도 .NET 측에서 바인딩됨

    # status (uint)
    obj.status = int(body_dict["status"])
    return obj


# ─────────────────────────────────────────────────────────────
def make_and_push(body_dict, messenger):
    """
    대시보드/GUI에서 넘긴 body_dict 정규화 → C# ModuleStatus → 전송
      - 입력 지원:
          A) {"Status": 0|1|2, "SourceModuleName": "...", "Timestamp"?: ulong}
          B) {"status": 0|1|2, "source": "...", "timestamp": ulong}
      - 기본값:
          status=1(정상), source=KU_ROLE 매핑명, timestamp=_now_ms()  # 2000-01-01 UTC 기준
      - 텍스트/JSON 전송 및 로그 출력 키 순서 고정:
          Timestamp → Status → SourceModuleName
    """
    # 1) 표준화
    try:
        status = int(body_dict.get("Status", body_dict.get("status", 1)))
    except Exception:
        status = 1

    source = (body_dict.get("SourceModuleName")
              or body_dict.get("source")
              or body_dict.get("requestModuleName")
              or _default_source_module_name())

    ts_in = body_dict.get("Timestamp", body_dict.get("timestamp"))
    try:
        timestamp = int(ts_in) if ts_in is not None else _now_ms()
    except Exception:
        timestamp = _now_ms()

    canon = {"timestamp": timestamp, "source": str(source), "status": int(status)}

    # 2) C# 객체로 매핑 시도 (가능하면 객체 전송)
    obj = None
    try:
        obj = _dict_to_obj(canon)
    except Exception:
        obj = None

    sent = False
    last_err = None

    # 3) 객체 전송 가능한 메서드 우선 시도
    if obj is not None:
        for fn in ("Publish", "Send", "Push", "PublishObject", "SendObject"):
            try:
                if hasattr(messenger, fn):
                    getattr(messenger, fn)("0102", obj)
                    sent = True
                    break
            except Exception as e:
                last_err = e

    # 4) JSON 전송으로 폴백 (키 순서 고정: Timestamp, Status, SourceModuleName)
    if not sent:
        jdict = OrderedDict([
            ("Timestamp", canon["timestamp"]),
            ("Status",    canon["status"]),
            ("SourceModuleName", canon["source"]),
        ])
        j = json.dumps(jdict, ensure_ascii=False)
        for fn in ("PublishJson", "PushJson", "SendJson", "SendText", "PublishText", "PublishString", "SendString"):
            try:
                if hasattr(messenger, fn):
                    getattr(messenger, fn)("0102", j)
                    sent = True
                    break
            except Exception as e:
                last_err = e

    # 5) 대시보드 로그용 RAW 반환 (키 순서 고정, 내부 필드 없음)
    raw_payload = OrderedDict([
        ("Timestamp",        canon["timestamp"]),
        ("Status",           canon["status"]),
        ("SourceModuleName", canon["source"]),
    ])
    raw = ("MSGID=0102 BODY " + json.dumps(raw_payload, ensure_ascii=False)).encode("utf-8")
    return raw


def make_random_and_push(node_messenger) -> bytes:
    """별도 입력 없이 랜덤/현재값으로 바디 생성 후 즉시 Push"""
    try:
        base = make_msg0102_body()  # 외부 제너레이터 사용 (있다면)
        if "Timestamp" not in base and "timestamp" not in base:
            base["Timestamp"] = _now_ms()
        if "SourceModuleName" not in base and "source" not in base:
            base["SourceModuleName"] = _default_source_module_name()
        if "Status" not in base and "status" not in base:
            base["Status"] = 1
        return make_and_push(base, node_messenger)
    except Exception:
        # 제너레이터가 없어도 동작하도록 폴백
        return make_and_push(
            {"Timestamp": _now_ms(), "Status": 1, "SourceModuleName": _default_source_module_name()},
            node_messenger
        )
