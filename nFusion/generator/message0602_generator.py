# generator/message0602_generator.py

import random
import string
import json
from datetime import datetime, timezone

# ── 기준 시점: 2000-01-01 00:00:00 UTC ─────────────────────────────
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)
_now_ms      = lambda: int((datetime.utcnow().replace(tzinfo=timezone.utc) -
                            _EPOCH_2000).total_seconds() * 1000)

# 1바이트 uint (0 ~ 255)
rand_uint8   = lambda lo=0, hi=255: random.randint(lo, hi)
# 2바이트 uint (0 ~ 2^16−1)
rand_uint16  = lambda lo=0, hi=(2**16 - 1): random.randint(lo, hi)
# 4바이트 uint (0 ~ 2^32−1)
rand_uint32  = lambda lo=0, hi=(2**32 - 1): random.randint(lo, hi)
# 4바이트 float (32-bit float 범위, m/s 혹은 % 등)
rand_float4  = lambda lo, hi: round(random.uniform(lo, hi), 6)
# 8바이트 float (64-bit float 범위, 위도/경도)
rand_float8  = lambda lo, hi: round(random.uniform(lo, hi), 8)
# 8자 경로 ID 문자열 (ASCII 대문자 + 숫자)
rand_str8    = lambda: ''.join(random.choices(string.ascii_uppercase + string.digits, k=8))
# 4바이트 int (int32: −2^31 ~ 2^31−1)
rand_int32 = lambda lo=-(2**31), hi=(2**31 - 1): random.randint(lo, hi)

def _random_coordinate() -> dict:
    """
    • Latitude  : float64 (8 bytes, −90.0 ~ +90.0)
    • Longitude : float64 (8 bytes, −180.0 ~ +180.0)
    • Altitude  : float32 (4 bytes, 0.0 ~ 10000.0 m)
    """
    return {
        "latitude":  rand_float8(-90.0, 90.0),
        "longitude": rand_float8(-180.0, 180.0),
        "altitude":  rand_uint8(0, 10000)
    }


def make_msg0602_body() -> dict:
    return {
        "timestamp": _now_ms(),
        "uavCommandModeType": rand_uint8(0, 255),
        "aircraftID": rand_uint8(0, 255),

        "flightModeCommand": {
            "flightMode": rand_uint8(0, 255),

            "pathFollowing": {
                "pathID": rand_str8(),
                "startWaypointID": rand_uint16(0, 2**16 - 1)
            },

            "targetTracking": {
                "targetID": rand_uint32(0, 2**32 - 1)
            },

            "loiterProperty": {
                "coordinate": _random_coordinate(),
                "loiterTime": rand_float4(0.0, 3600.0),
                "loiterRadius": rand_float4(0.0, 1000.0),
                "loiterDirection": rand_uint8(0, 1),
                "loiterSpeed": rand_float4(0.0, 200.0)
            },

            "formationProperty": {
                "leaderAircraftID": rand_uint8(0, 255),
                "formation": {
                    "dX": rand_int32(-100, 100),
                    "dY": rand_int32(-100, 100),
                    "dZ": rand_int32(-50, 50)
                }
            }
        },

        "filmingModeCommand": {
            "operationMode": rand_uint8(0, 255),
            "sensorType": rand_uint8(0, 255),
            "fieldOfView": rand_float4(0.0, 180.0),

            "coordinateOrientation": {
                "coordinate": _random_coordinate()
            },

            "lineSearch": {
                "coordinateList": [
                    _random_coordinate() for _ in range(random.randint(1, 5))
                ],
                "searchSpeed": rand_float4(0.0, 50.0)
            },

            "autoTracking": {
                "targetID": rand_uint32(0, 2**32 - 1)
            },

            "aircraftFixed": {
                "gimbalPitch": rand_float4(-90.0, 90.0),
                "gimbalYaw": rand_float4(-180.0, 180.0)
            },

            "autoScan": {
                "gimbalPitch": rand_float4(-90.0, 90.0),
                "gimbalYawLimits": {
                    "leftLimit": rand_float4(-180.0, 0.0),
                    "rightLimit": rand_float4(0.0, 180.0)
                },
                "gimbalYawAngularSpeed": rand_float4(0.0, 360.0)
            }
        }
    }


if __name__ == "__main__":
    print(json.dumps(make_msg0602_body(), ensure_ascii=False, indent=2))
