# generator/message0802_generator.py

import random
import json
from datetime import datetime, timezone

# ── 기준 시점: 2000-01-01 00:00:00 UTC ─────────────────
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms = lambda: int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000)

def make_msg0802_body() -> dict:
    """
    0802 – MandatoryCommand 랜덤 바디 생성
    • Timestamp     : ulong (8 bytes, ms since 2000-01-01)
    • AircraftID    : uint  (4 bytes)  – 4=무인기1, 5=무인기2, 6=무인기3 중 선택
    • MandatoryType : uint  (4 bytes)  – 1=강제대기, 2=강제귀환, 3=강제임무복귀
    """
    return {
        "timestamp"     : _now_ms(),
        "aircraftID"    : random.choice([4, 5, 6]),
        "mandatoryType" : random.choice([1, 2, 3])
    }

if __name__ == "__main__":
    print(json.dumps(make_msg0802_body(), ensure_ascii=False, indent=2))
