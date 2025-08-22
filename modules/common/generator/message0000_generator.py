# message0000_generator.py
import random
import json
import time
from datetime import datetime, timezone

_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms = lambda: int(
    (datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000
)

def make_msg0000_body(source: str = "DS"):
    """Response(0000) 메시지 바디 생성 (소문자 카멜) — 규격: timestamp(ulong), source(string), messageID(uint)"""
    import random, time
    return {
        "timestamp": _now_ms(),     # ms, ulong 범위
        "source": str(source),                    # string
        "messageID": random.getrandbits(32),      # uint(0..2^32-1)
    }

def make_random_and_push(node_messenger):
    from generator.message0000_push import make_and_push
    return make_and_push(make_msg0000_body(), node_messenger)

if __name__ == "__main__":
    print(json.dumps(make_msg0000_body(), ensure_ascii=False, indent=2))
