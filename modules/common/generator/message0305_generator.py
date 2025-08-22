# generator/message0305_generator.py
import random
import json
from datetime import datetime, timezone

# 2000-01-01 UTC 기준 ms
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
def _now_ms() -> int:
    return int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000)

def make_msg0305_body() -> dict:
    """ReplanStatus(0305) 메시지용 랜덤 바디 생성"""
    reasons = [
        "WeatherChange",
        "NoFlyZoneDetected",
        "HighPriorityMission",
        "FuelLow",
        "ObstacleDetected",
    ]
    return {
        "timestamp":             _now_ms(),           # ← 2000 epoch
        "missionPlanningStatus": random.randint(0, 2),
        "replanReason":          random.choice(reasons),
    }

if __name__ == "__main__":
    print(json.dumps(make_msg0305_body(), ensure_ascii=False, indent=2))
