# generator/message0202_generator.py
import random
import string
import json
import time
from datetime import datetime, timezone

_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms = lambda: int(
    (datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000
)

# ---------- 랜덤 유틸 ----------
UINT32_MAX = (1 << 32) - 1
rand_uint32 = lambda: random.randint(0, UINT32_MAX)      # 0 ~ 4,294,967,295

rand_str8  = lambda: ''.join(random.choices(string.ascii_uppercase + string.digits, k=8))
rand_int   = lambda lo, hi: random.randint(lo, hi)
def rand_float(lo, hi, nd=6): return round(random.uniform(lo, hi), nd)

rand_lat = lambda: rand_float(-90,   90, 6)
rand_lon = lambda: rand_float(-180, 180, 6)
rand_alt = lambda: rand_int(0, 10000)  # ← altitude는 int

def _make_coord_orientation() -> dict:
    """좌표지향 임무용 좌표(위·경·고도=int altitude)"""
    return {
        "latitude":  rand_lat(),
        "longitude": rand_lon(),
        "altitude":  rand_alt(),  # int
    }

def _make_target_orientation() -> dict:
    """표적추적 임무용 표적 ID"""
    return {
        "targetID": rand_int(1000, 9999)
    }

def make_msg0202_body() -> dict:
    """
    PriorMissionInfo(0202) 메시지 바디 생성 (소문자 카멜)
    - missionType=1(좌표지향): coordinateOrientation 포함
    - missionType=2(표적추적): targetOrientation 포함
    """
    body = {
        "timestamp": _now_ms(),
        "priorMissionList": []
    }

    # 1~3개 미션 생성
    for _ in range(rand_int(1, 3)):
        mtype = rand_int(1, 2)  # 1: 좌표지향, 2: 표적추적
        entry = {
            "priorMissionID": rand_uint32(),
            "missionType":    mtype,
        }
        if mtype == 1:
            entry["coordinateOrientation"] = _make_coord_orientation()
        else:
            entry["targetOrientation"] = _make_target_orientation()

        body["priorMissionList"].append(entry)
    return body

if __name__ == "__main__":
    print(json.dumps(make_msg0202_body(), ensure_ascii=False, indent=2))
