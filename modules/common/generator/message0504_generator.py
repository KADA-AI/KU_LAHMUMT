# modules/common/generator/message0504_generator.py
# Manual generator for Message 0504 (FuelWarning)

import os, json, random
from datetime import datetime, timezone

HERE = os.path.dirname(os.path.abspath(__file__))

_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)

def now_ms_2000() -> int:
    return int((datetime.now(timezone.utc) - _EPOCH_2000).total_seconds() * 1000)

WARNING_CODES = [1, 2]  # 1=Yellow, 2=Red


def make_msg0504_body(source: str = "MSM", aircraft_id: int | None = None, fuel_level: int | None = None) -> dict:
    """Create a FuelWarning message body (MessageID: 0504)."""
    aid = int(aircraft_id) if aircraft_id is not None else random.randint(1, 99)
    level = int(fuel_level) if fuel_level in (1, 2) else random.choice(WARNING_CODES)
    return {
        "timestamp": now_ms_2000(),
        "source": source,
        "aircraftID": aid,
        "fuelLevel": level,
    }


def make_random_and_push(node_messenger):
    # ⬇️ 경로 수정: generator.message0504_push (X) → push.message0504_push (O)
    from push.message0504_push import make_and_push  # modules/common/push/message0504_push.py
    return make_and_push(make_msg0504_body(), node_messenger)


if __name__ == "__main__":
    print(json.dumps(make_msg0504_body(), ensure_ascii=False, indent=2))
