# generator/message0903_generator.py

import random
import json
from datetime import datetime, timezone

# ── 기준 시점: 2000-01-01 00:00:00 UTC ─────────────────
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)

def _timestamp_ms() -> int:
    """
    2000-01-01 00:00:00 UTC부터 현재 UTC까지의 밀리초(ulong) 반환
    → 64비트 범위(0~2^64−1) 내에 들어감.
    """
    return int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000)

def make_msg0903_body() -> dict:
    """
    0903 – RequestRenewMission 메시지 바디 생성
    • timestamp     : ulong(ms since 2000-01-01)
    • missionPlanID : uint  (4바이트, 0 ~ 2^32−1)
    """
    return {
        "timestamp"     : _timestamp_ms(), 
        "missionPlanID" : random.randint(0, 2**32 - 1)
    }

if __name__ == "__main__":
    print(json.dumps(make_msg0903_body(), ensure_ascii=False, indent=2))
