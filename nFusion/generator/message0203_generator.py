# message0203_generator.py
import random
import string
import time
import json

UINT32_MAX = (1 << 32) - 1
rand_uint32 = lambda: random.randint(0, UINT32_MAX)      # 0 ~ 4 294 967 295

rand_str8  = lambda: ''.join(random.choices(string.ascii_uppercase + string.digits, k=8))
rand_int   = lambda lo=0, hi=100: random.randint(lo, hi)
rand_float = lambda lo, hi, nd=6: round(random.uniform(lo, hi), nd)

rand_lat = lambda: rand_float(-90,   90, 6)
rand_lon = lambda: rand_float(-180, 180, 6)
rand_alt = lambda: rand_float(0, 50000, 1)

def _coord():
    return {
        "latitude":  rand_lat(),
        "longitude": rand_lon(),
        "altitude":  rand_alt()
    }

def _area_lat_lon():
    return {
        "latitude":  rand_lat(),
        "longitude": rand_lon()
    }

def _altitude_limits():
    # lowerLimit이 upperLimit보다 작도록 정렬
    a = rand_alt()
    b = rand_alt()
    lower, upper = (a, b) if a <= b else (b, a)
    return {
        "lowerLimit": lower,
        "upperLimit": upper
    }

def make_msg0203_body() -> dict:
    """
    FlightReferenceInfo(0203) 메시지 바디 생성 (소문자 카멜)
    """
    now_ms = int(time.time() * 1000)
    body = {
        "timestamp":                  now_ms,
        "missionReferencePackageID":  rand_uint32(),
        "inputTimestamp":             now_ms,
        "takeOverInfoList":           [],
        "handOverInfoList":           [],
        "rtbCoordinateList":          [],
        "flightAreaList":             [],
        "prohibitedAreaList":         []
    }

    # TakeOverInfoList (1~3개)
    for _ in range(rand_int(1, 3)):
        entry = {
            "aircraftID": rand_int(0, 6),
            "coordinate": _coord()
        }
        body["takeOverInfoList"].append(entry)

    # HandOverInfoList (1~3개)
    for _ in range(rand_int(1, 3)):
        entry = {
            "aircraftID": rand_int(0, 6),
            "coordinate": _coord()
        }
        body["handOverInfoList"].append(entry)

    # RTBCoordinateList (1~4개)
    for _ in range(rand_int(1, 4)):
        body["rtbCoordinateList"].append(_coord())

    # FlightAreaList (1~2개)
    for _ in range(rand_int(1, 2)):
        area = {
            "flightAreaID":   rand_uint32(),
            "areaLatLonList": [_area_lat_lon() for _ in range(rand_int(3, 6))],
            "altitudeLimits": _altitude_limits()
        }
        body["flightAreaList"].append(area)

    # ProhibitedAreaList (0~2개)
    for _ in range(rand_int(0, 2)):
        area = {
            "prohibitedAreaID": rand_uint32(),
            "areaLatLonList":   [_area_lat_lon() for _ in range(rand_int(3, 6))],
            "altitudeLimits":   _altitude_limits()
        }
        body["prohibitedAreaList"].append(area)

    return body

if __name__ == "__main__":
    print(json.dumps(make_msg0203_body(), ensure_ascii=False, indent=2))
