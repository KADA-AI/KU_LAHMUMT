# message0402_generator.py
import random
import string
import json
import time

# ───────── 헬퍼 함수 ─────────
rand_str8  = lambda: ''.join(random.choices(string.ascii_uppercase + string.digits, k=8))
rand_int   = lambda lo=0, hi=2**16 - 1: random.randint(lo, hi)
rand_bool  = lambda: bool(random.getrandbits(1))
rand_float = lambda lo, hi, nd=6: round(random.uniform(lo, hi), nd)

def _rand_lat():
    return rand_float(-90, 90, 6)

def _rand_lon():
    return rand_float(-180, 180, 6)

def _rand_alt():
    return rand_int(0, 10000)

def _coord():
    return {
        "latitude":  _rand_lat(),
        "longitude": _rand_lon(),
        "altitude":  _rand_alt()
    }

# ───────── 바디 생성 함수 ─────────
def make_msg0402_body() -> dict:
    """
    SituationAwarenessInfo(0402) 메시지 바디 생성 (소문자 카멜)
    """
    now_ms = int(time.time() * 1000)
    body = {
        "timestamp": now_ms,
        "roiInfo": {
            "aircraftID": rand_int(),
            "coordinate": _coord(),
            "fov":        rand_float(0, 180, 2)
        },
        "targetList": []
    }

    for _ in range(random.randint(1, 4)):
        tgt = {
            "targetID":      rand_int(0, 2**16 - 1),
            "targetType":    rand_int(0, 255),
            "coordinate":    _coord(),
            "watcher":       {"aircraftID": rand_int()},
            "targetInFrame": rand_bool(),
            "isDestroyed":   rand_bool(),
            "threat":        rand_float(0, 100, 2)
        }
        body["targetList"].append(tgt)

    return body

if __name__ == "__main__":
    print(json.dumps(make_msg0402_body(), ensure_ascii=False, indent=2))
