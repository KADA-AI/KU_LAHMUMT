# generator/message0904_generator.py
import random
import string
import json
from datetime import datetime, timezone

# ── 기준 시점: 2000-01-01 00:00:00 UTC ─────────────────
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)

def _timestamp_ms() -> int:
    """
    2000-01-01 00:00:00 UTC부터 현재 UTC까지의 밀리초(ulong) 반환
    → 64비트 범위(0~2^64-1) 내에 들어감.
    """
    return int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000)

# BehaviorTreeFileID는 정확히 8문자 (ASCII 대문자/숫자) 문자열
rand_str8 = lambda: ''.join(random.choices(string.ascii_uppercase + string.digits, k=8))


def make_msg0904_body() -> dict:
    """
    0904 – RequestBehaviorTree 메시지 바디 생성
    • Timestamp: ulong(ms since 2000-01-01)
    • BehaviorTreeFileID: 길이 8 문자(string)
    """
    return {
        "timestamp": _timestamp_ms(),         # ulong (8 bytes, ms 단위)
        "behaviorTreeFileID": rand_str8()     # string (8 bytes)
    }


if __name__ == "__main__":
    # 예시 출력 (들여쓰기 indent=2로 사람이 보기 좋게)
    print(json.dumps(make_msg0904_body(), ensure_ascii=False, indent=2))
