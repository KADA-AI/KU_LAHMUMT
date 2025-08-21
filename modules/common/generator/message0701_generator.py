# generator/message0701_generator.py

import random
import json
from datetime import datetime, timezone

# ── 기준 시점: 2000-01-01 00:00:00 UTC ─────────────────
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms      = lambda: int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000)

rand_uint8       = lambda: random.getrandbits(32)
# 32비트 부호 있는 정수(int32) 생성: -1, 0, 1 선택
rand_int32_choice = lambda: random.choice([-1, 0, 1])
# 32비트 부호 없는 정수(uint[4]) 생성: 0 ~ 2^32−1
rand_uint32      = lambda: random.randint(0, 2**32 - 1)
# 옵션명용 코드 (Uint, 값 1~5)
rand_option_name = lambda: random.choice([1, 2, 3, 4, 5])
# 불리언 생성 (1바이트)
rand_bool        = lambda: random.choice([True, False])

def make_msg0701_body() -> dict:
    body = {
        "timestamp"    : _now_ms(),
        "autoExecution": rand_bool(),
        "optionList"   : []
    }

    for _ in range(random.randint(1, 3)):
        option_item = {
            "optionID"           : rand_uint8(),
            "optionName"         : rand_option_name(),
            "survivalRate"       : rand_int32_choice(),
            "timeContraction"    : rand_int32_choice(),
            "recogEffectiveness" : rand_int32_choice(),
            "distance"           : rand_uint32(),
            "target"             : rand_uint32(),
            "uavMissionPlanIDList": [],
            "lahMissionPlanIDList": []
        }

        for _ in range(random.randint(1, 3)):
            option_item["uavMissionPlanIDList"].append({
                "uavMissionPlanID": rand_uint32()
            })
        for _ in range(random.randint(1, 3)):
            option_item["lahMissionPlanIDList"].append({
                "lahMissionPlanID": rand_uint32()
            })

        body["optionList"].append(option_item)

    return body

if __name__ == "__main__":
    print(json.dumps(make_msg0701_body(), ensure_ascii=False, indent=2))
