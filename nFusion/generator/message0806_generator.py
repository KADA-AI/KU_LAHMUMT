# generator/message0806_generator.py

import random
import json
from datetime import datetime, timezone

# ── 기준 시점: 2000-01-01 00:00:00 UTC ─────────────────
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms = lambda: int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000)

def make_msg0806_body() -> dict:
    """
    0806 – EndSWCommand 랜덤 바디 생성
    • Timestamp: ulong (8 bytes, ms since 2000-01-01)
    • Command  : uint  (4 bytes)
        1 = 시스템 종료
        2 = 시스템 재부팅
    """
    return {
        "timestamp": _now_ms(),
        "command"  : random.choice([1, 2])
    }

if __name__ == "__main__":
    print(json.dumps(make_msg0806_body(), ensure_ascii=False, indent=2))
