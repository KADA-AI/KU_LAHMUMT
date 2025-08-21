# message0202_generator.py
import random
import string
import json
import time

UINT32_MAX = (1 << 32) - 1
rand_uint32 = lambda: random.randint(0, UINT32_MAX)      # 0 ~ 4 294 967 295

rand_str8  = lambda: ''.join(random.choices(string.ascii_uppercase + string.digits, k=8))
rand_int   = lambda lo, hi: random.randint(lo, hi)
rand_float = lambda lo, hi, nd=6: round(random.uniform(lo, hi), nd)

rand_lat = lambda: rand_float(-90,   90, 6)
rand_lon = lambda: rand_float(-180, 180, 6)
rand_alt = lambda: rand_int(0, 10000)

def make_msg0202_body() -> dict:
    """
    PriorMissionInfo(0202) 메시지 바디 생성 (소문자 카멜)
    """
    now_ms = int(time.time() * 1000)
    body = {
        "timestamp":         now_ms,
        "priorMissionList": []
    }
    for _ in range(rand_int(1, 3)):
        entry = {
            "priorMissionID": rand_uint32(),
            # 1: 좌표지향, 2: 표적추적
            "missionType":    rand_int(1, 2),
            "coordinateOrientation": {
                "latitude":  rand_lat(),
                "longitude": rand_lon(),
                "altitude":  rand_alt()
            },
            "targetOrientation": {
                "targetID": rand_int(1000, 9999)
            }
        }
        body["priorMissionList"].append(entry)
    return body

if __name__ == "__main__":
    print(json.dumps(make_msg0202_body(), ensure_ascii=False, indent=2))
