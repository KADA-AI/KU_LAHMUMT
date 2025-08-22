# generator/message0601_generator.py
import random
import json
from datetime import datetime, timezone

# ── 공통 유틸 ─────────────────────────────────────────────
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms = lambda: int((datetime.utcnow().replace(tzinfo=timezone.utc) -
                       _EPOCH_2000).total_seconds() * 1000)

rand_int = lambda lo, hi: random.randint(lo, hi)
# ─────────────────────────────────────────────────────────

def make_msg0601_body() -> dict:
    """
    0601 – UnderlyingAction 랜덤 바디 (소문자 키)
    • timestamp   : ulong (ms since 2000-01-01)
    • aircraft    : uint (0~6)
    • flightMode  : uint (0~9)
    • filmingMode : uint (0~6)
    """
    return {
        "timestamp"   : _now_ms(),
        "aircraft"    : rand_int(0, 6),
        "flightMode"  : rand_int(0, 9),
        "filmingMode" : rand_int(0, 6),
    }

if __name__ == "__main__":
    print(json.dumps(make_msg0601_body(), ensure_ascii=False, indent=2))
