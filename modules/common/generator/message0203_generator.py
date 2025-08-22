# generator/message0203_generator.py
import random
import string
import json
from datetime import datetime, timezone

# ---------- 공통: 2000-01-01 UTC 기준 ms ----------
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
def _now_ms() -> int:
    return int((datetime.utcnow().replace(tzinfo=timezone.utc) - _EPOCH_2000).total_seconds() * 1000)

# ---------- 랜덤 유틸 ----------
UINT32_MAX = (1 << 32) - 1
rand_uint32 = lambda: random.randint(0, UINT32_MAX)      # 0 ~ 4,294,967,295

def rand_int(lo=0, hi=100) -> int:
    return random.randint(lo, hi)

def rand_float(lo: float, hi: float, nd: int = 6) -> float:
    return round(random.uniform(lo, hi), nd)

rand_lat = lambda: rand_float(-90,   90, 6)
rand_lon = lambda: rand_float(-180, 180, 6)
rand_alt = lambda: rand_int(0, 50000)   # ← altitude는 int로 고정

def _coord() -> dict:
    return {
        "latitude":  rand_lat(),
        "longitude": rand_lon(),
        "altitude":  rand_alt(),   # int
    }

def _area_lat_lon() -> dict:
    return {
        "latitude":  rand_lat(),
        "longitude": rand_lon(),
    }

def _altitude_limits() -> dict:
    a = rand_alt()
    b = rand_alt()
    lower, upper = (a, b) if a <= b else (b, a)
    return {
        "lowerLimit": lower,   # int
        "upperLimit": upper,   # int
    }

def make_msg0203_body() -> dict:
    """
    FlightReferenceInfo(0203) 메시지 바디 생성 (소문자 카멜)
    - timestamp / inputTimestamp : 2000-01-01 UTC 기준 ms
    - altitude 관련 값은 모두 int
    """
    now_ms = _now_ms()
    body = {
        "timestamp":                  now_ms,
        "missionReferencePackageID":  rand_uint32(),
        "inputTimestamp":             now_ms,
        "takeOverInfoList":           [],
        "handOverInfoList":           [],
        "rtbCoordinateList":          [],
        "flightAreaList":             [],
        "prohibitedAreaList":         [],
    }

    # TakeOverInfoList (1~3개)
    for _ in range(rand_int(1, 3)):
        body["takeOverInfoList"].append({
            "aircraftID": rand_int(0, 6),
            "coordinate": _coord(),
        })

    # HandOverInfoList (1~3개)
    for _ in range(rand_int(1, 3)):
        body["handOverInfoList"].append({
            "aircraftID": rand_int(0, 6),
            "coordinate": _coord(),
        })

    # RTBCoordinateList (1~4개)
    for _ in range(rand_int(1, 4)):
        body["rtbCoordinateList"].append(_coord())

    # FlightAreaList (1~2개)
    for _ in range(rand_int(1, 2)):
        body["flightAreaList"].append({
            "flightAreaID":   rand_uint32(),
            "areaLatLonList": [_area_lat_lon() for _ in range(rand_int(3, 6))],
            "altitudeLimits": _altitude_limits(),
        })

    # ProhibitedAreaList (0~2개)
    for _ in range(rand_int(0, 2)):
        body["prohibitedAreaList"].append({
            "prohibitedAreaID": rand_uint32(),
            "areaLatLonList":   [_area_lat_lon() for _ in range(rand_int(3, 6))],
            "altitudeLimits":   _altitude_limits(),
        })

    return body

if __name__ == "__main__":
    print(json.dumps(make_msg0203_body(), ensure_ascii=False, indent=2))
