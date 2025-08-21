# message0102_generator.py
import random
import string
import json, time
from datetime import datetime, timezone

# ── 공통 유틸 ─────────────────────────────────────────────
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms = lambda: int((datetime.utcnow().replace(tzinfo=timezone.utc) -
                       _EPOCH_2000).total_seconds() * 1000)
# ─────────────────────────────────────────────────────────

def make_msg0102_body():
    """ModuleStatus(0102) 메시지 바디 생성 (소문자 카멜)"""
    return {
        # 0 ~ 2 범위 내 (0: Unknown, 1: 정상, 2: 비정상)
        "timestamp":         int(time.time() * 1000),
        "status":            random.randint(0, 2),
        # CSC 이름 전송 (임시로 대표적인 CSC 모듈 이름들)
        "sourceModuleName":  random.choice([
            "csc1",
            "csc2",
            "csc3",
            "csc4",
            "csc5"
        ])
    }

if __name__ == "__main__":
    print(json.dumps(make_msg0102_body(), ensure_ascii=False, indent=2))
