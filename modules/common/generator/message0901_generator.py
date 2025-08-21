# generator/message0901_generator.py

import random
import string
import json
from datetime import datetime, timezone

# ── 기준 시점: 2000-01-01 00:00:00 UTC ─────────────────
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms = lambda: int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000)

rand_uint = lambda: random.randint(0, 2**32 - 1)
rand_str  = lambda n: ''.join(random.choices(string.ascii_uppercase + string.digits, k=n))

def make_msg0901_body() -> dict:
    """
    0901 – RequestOptionInfo 랜덤 바디 생성
    • Timestamp    : ulong (8 bytes, ms since 2000-01-01)
    • RequestTime  : ulong (8 bytes, ms since 2000-01-01)
    • OptionList[] : List of { OptionID: uint, OptionName: string, MissionPlanID: uint }
    """
    return {
        "timestamp"   : _now_ms(),
        "requestTime" : _now_ms(),
        "optionList": [
            {
                "optionID"     : rand_uint(),
                "optionName"   : rand_str(random.randint(5, 15)),
                "missionPlanID": rand_uint()
            }
            for _ in range(random.randint(1, 4))
        ]
    }

if __name__ == "__main__":
    print(json.dumps(make_msg0901_body(), ensure_ascii=False, indent=2))
