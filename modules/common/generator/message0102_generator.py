# generator/message0102_generator.py
import random
import json
import time
from datetime import datetime, timezone

_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms = lambda: int(
    (datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000
)

def make_msg0102_body(source: str = "CSC"):
    """ModuleStatus(0102) 메시지 바디 생성 (소문자 카멜)
    규격: timestamp(ulong), source(string), status(uint: 0=Unknown, 1=정상, 2=비정상)
    """
    return {
        "timestamp": _now_ms(),  # ms, ulong
        "source":    str(source),              # string
        "status":    random.randint(0, 2),     # uint 0..2
    }

def make_random_and_push(node_messenger) -> bytes:
    """별도 입력 없이 현재값/랜덤으로 바디 생성 후 즉시 Push"""
    from generator.message0102_push import make_and_push
    return make_and_push(make_msg0102_body(), node_messenger)

if __name__ == "__main__":
    print(json.dumps(make_msg0102_body(), ensure_ascii=False, indent=2))
