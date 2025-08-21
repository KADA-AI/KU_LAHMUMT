# generator/message0804_generator.py

import random
import json
from datetime import datetime, timezone

# ── 기준 시점: 2000-01-01 00:00:00 UTC ─────────────────
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms = lambda: int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000)

rand_uint = lambda: random.randint(0, 2**32 - 1)

def make_msg0804_body() -> dict:
    """
    0804 – MissionRestartCommand 랜덤 바디 생성
    • Timestamp     : ulong (8 bytes, ms since 2000-01-01)
    • InputMissionID: uint  (4 bytes)
    """
    return {
        "timestamp"     : _now_ms(),     # ulong(ms)
        "inputMissionID": rand_uint()    # uint (4 bytes)
    }

if __name__ == "__main__":
    print(json.dumps(make_msg0804_body(), ensure_ascii=False, indent=2))
