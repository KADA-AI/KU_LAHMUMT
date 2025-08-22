# MissionID 0503 – 협업기저임무 완료 알림
import json, random
from datetime import datetime, timezone

# ── Epoch (2000-01-01 UTC) ───────────────────────────────
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms = lambda: int(
    (datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000
    ).total_seconds() * 1000
)

def make_msg0503_body() -> dict:
    """0503 MissionResult 메시지 바디 (단순형)"""
    return {
        "timestamp": _now_ms(),               # ulong
        "source": f"CSC{random.randint(1,5)}",# string
        "systemRecommend": random.randint(0,1)# uint
    }

if __name__ == "__main__":
    print(json.dumps(make_msg0503_body(), ensure_ascii=False, indent=2))
