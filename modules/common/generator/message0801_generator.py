# generator/message0801_generator.py

import random
import json
from datetime import datetime, timezone

# ── 기준 시점: 2000-01-01 00:00:00 UTC ─────────────────
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms = lambda: int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000)

rand_uint4 = lambda: random.getrandbits(32)

# 불리언 생성 (1바이트)
rand_bool = lambda: random.choice([True, False])

def make_msg0801_body() -> dict:
    """
    0801 – ReplanCommand 메시지 바디 생성
    • Timestamp                 : ulong  (8 bytes, ms since 2000-01-01)
    • OperatorReplanRequestTime : ulong  (8 bytes, ms since 2000-01-01)
    • IsOnGround                : bool   (1 byte)
    • InputMissionPackageID     : uint[8] (8 bytes)
    • MissionReferencePackageID : uint[8] (8 bytes)
    """
    return {
        "timestamp"                 : _now_ms(),
        "operatorReplanRequestTime" : _now_ms(),
        "isOnGround"                : rand_bool(),
        "inputMissionPackageID"     : rand_uint4(),
        "missionReferencePackageID" : rand_uint4()
    }

if __name__ == "__main__":
    print(json.dumps(make_msg0801_body(), ensure_ascii=False, indent=2))
