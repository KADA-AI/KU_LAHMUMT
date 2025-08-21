# message0805_generator.py

import random
import json
from datetime import datetime, timezone

# ── 기준 시점: 2000-01-01 00:00:00 UTC ─────────────────
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms = lambda: int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000)

def make_msg0805_body() -> dict:
    
    return {
        "timestamp": _now_ms()
    }

if __name__ == "__main__":
    print(json.dumps(make_msg0805_body(), ensure_ascii=False, indent=2))
