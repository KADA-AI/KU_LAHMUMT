# generator/message0502_generator.py

import json
from datetime import datetime, timezone

# ── 기준 시점: 2000-01-01 00:00:00 UTC ─────────────────
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms     = lambda: int((datetime.utcnow().replace(tzinfo=timezone.utc) -
                           _EPOCH_2000).total_seconds() * 1000)

def make_msg0502_body() -> dict:
    """
    0502 – EndMissionRequest 랜덤 바디 생성
    • Timestamp: ulong (8 bytes, ms since 2000-01-01)
    """
    return {
        "timestamp": _now_ms()
    }

if __name__ == "__main__":
    print(json.dumps(make_msg0502_body(), ensure_ascii=False, indent=2))
