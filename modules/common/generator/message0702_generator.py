# generator/message0702_generator.py

import random
import json
from datetime import datetime, timezone

# ── 기준 시점: 2000-01-01 00:00:00 UTC ─────────────────
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms = lambda: int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000)

rand_uint32 = lambda: random.getrandbits(32)

# 1바이트 Ignore 필드: 0=None, 1=기존임무수행, 2=MissionPlanID로 선택
rand_ignore = lambda: random.choice([0, 1, 2])

def make_msg0702_body() -> dict:
    """
    0702 – MissionProgress 메시지 바디 생성
    • timestamp     : ulong      (8 bytes, ms since 2000-01-01)
    • ignore        : uint[1]    (1 byte; 0=None, 1=기존임무수행, 2=MissionPlanID로 선택)
    • missionPlanID : uint[8]    (8 bytes; 수행할 임무계획 ID)
    """
    return {
        "timestamp":     _now_ms(),
        "ignore":        rand_ignore(),
        "missionPlanID": rand_uint32()
    }

if __name__ == "__main__":
    print(json.dumps(make_msg0702_body(), ensure_ascii=False, indent=2))
