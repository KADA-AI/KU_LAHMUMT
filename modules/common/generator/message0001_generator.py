# modules/common/generator/message0001_generator.py
# Manual generator for Message 0001 (NoticeInfo)

import json
import random
from datetime import datetime, timezone

from modules.common.source_utils import get_default_source_code
from modules.common.string_limits import NOTICE_CONTENTS_BYTE_LIMIT, limit_utf8_bytes

_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)


def now_ms_2000() -> int:
    return int((datetime.now(timezone.utc) - _EPOCH_2000).total_seconds() * 1000)


def make_msg0001_body(source: str | None = None, contents: str | None = None) -> dict:
    """Create a NoticeInfo message body (MessageID: 0001)."""
    if source is None or not str(source).strip():
        source = get_default_source_code()
    if contents is None:
        contents = f"Notice #{random.randint(1, 9999)}"
    return {
        "timestamp": now_ms_2000(),
        "source": str(source),
        "contents": limit_utf8_bytes(contents, NOTICE_CONTENTS_BYTE_LIMIT),
    }


def make_random_and_push(node_messenger):
    from push.message0001_push import make_and_push  # modules/common/push/message0001_push.py

    source = get_default_source_code()
    return make_and_push(make_msg0001_body(source=source), node_messenger)


if __name__ == "__main__":
    print(json.dumps(make_msg0001_body(), ensure_ascii=False, indent=2))
