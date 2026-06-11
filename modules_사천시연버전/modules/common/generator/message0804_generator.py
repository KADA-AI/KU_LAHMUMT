# modules/common/generator/message0804_generator.py
# Manual generator for Message 0804 (PriorMissionCancelRequest)

import json
import random
from datetime import datetime, timezone

_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)


def now_ms_2000() -> int:
    return int((datetime.now(timezone.utc) - _EPOCH_2000).total_seconds() * 1000)


def make_msg0804_body(source: str = "DSC", prior_mission_id: int | None = None) -> dict:
    """Create a PriorMissionCancelRequest body (MessageID: 0804)."""
    if prior_mission_id is None:
        prior_mission_id = random.randint(1, 999999)
    return {
        "timestamp": now_ms_2000(),
        "source": source,
        "priorMission": {"priorMissionID": int(prior_mission_id)},
    }


def make_random_and_push(node_messenger):
    from push.message0804_push import make_and_push  # modules/common/push/message0804_push.py

    return make_and_push(make_msg0804_body(), node_messenger)


if __name__ == "__main__":
    print(json.dumps(make_msg0804_body(), ensure_ascii=False, indent=2))
