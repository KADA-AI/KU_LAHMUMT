# modules/common/generator/message0104_generator.py

import json
import random
from datetime import datetime, timezone

_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
SOURCE_ENUM_DEFAULT = ["DSC", "IDM", "MSM", "MMR", "UCC", "MOB", "CSP", "INF", "INT"]


def now_ms_2000() -> int:
    return int((datetime.now(timezone.utc) - _EPOCH_2000).total_seconds() * 1000)


def make_msg0104_body(source: str = "DS") -> dict:
    src = str(source).strip() if source else ""
    if not src:
        src = random.choice(SOURCE_ENUM_DEFAULT)
    return {
        "timestamp": now_ms_2000(),
        "source": src,
        "aircraftID": random.randint(1, 3),
        "mode": random.randint(1, 2),
        "SBC1Status": random.randint(1, 2),
        "SBC2Status": random.randint(0, 2),
        "SBC3Status": random.randint(0, 2),
    }


def make_random_and_push(node_messenger):
    from modules.common.push.message0104_push import make_and_push

    return make_and_push(make_msg0104_body(), node_messenger)


if __name__ == "__main__":
    print(json.dumps(make_msg0104_body(), ensure_ascii=False, indent=2))
