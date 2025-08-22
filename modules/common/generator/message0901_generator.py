# -*- coding: utf-8 -*-
"""
MessageID 0901 – 옵션정보 생성 요청 (RequestOptionInfo) 바디 생성기

스펙:
- ulong timestamp         (ms since 2000-01-01 UTC)
- string source
- ulong requestTime       (ms since 2000-01-01 UTC)
- List<CommonType.PendingOption> pendingOptionList
    - uint optionID
    - string optionName
    - uint missionPlanID
"""
import json, random, string
from datetime import datetime, timezone

# ── 2000-01-01 UTC 기준 ms ─────────────────────────────────
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
def _now_ms() -> int:
    return int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000)

# ── 랜덤 유틸 ─────────────────────────────────────────────
rand_u32 = lambda: random.randint(0, 2**32 - 1)
def rand_str(n: int) -> str:
    return ''.join(random.choices(string.ascii_uppercase + string.digits, k=n))

def make_msg0901_body() -> dict:
    """
    0901 – RequestOptionInfo 바디 생성
    """
    return {
        "timestamp":   _now_ms(),
        "source":      f"CSC{random.randint(1,5)}",
        "requestTime": _now_ms(),
        "pendingOptionList": [
            {
                "optionID":      rand_u32(),
                "optionName":    rand_str(random.randint(5, 15)),
                "missionPlanID": rand_u32(),
            }
            for _ in range(random.randint(1, 4))
        ],
    }

if __name__ == "__main__":
    print(json.dumps(make_msg0901_body(), ensure_ascii=False, indent=2))
