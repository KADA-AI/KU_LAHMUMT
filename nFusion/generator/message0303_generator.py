# message0303_generator.py
import random
import json
import time
import os


UINT32_MAX = (1 << 32) - 1
rand_uint32 = lambda: random.randint(0, UINT32_MAX)      # 0 ~ 4 294 967 295

UINT16_MAX = (1 << 16) - 1          # 65535
rand_uint16 = lambda: random.randint(0, UINT16_MAX)

rand_int   = lambda lo=0, hi=100: random.randint(lo, hi)

# ──────── helpers ────────
def _coord():
    return {
        "latitude":  round(random.uniform(-90, 90), 6),
        "longitude": round(random.uniform(-180, 180), 6),
        "altitude":  round(random.uniform(30, 500), 1)
    }

def _formation_distances(n):
    return [{"dX": round(random.uniform(-50, 50), 1),
             "dY": round(random.uniform(-50, 50), 1),
             "dZ": round(random.uniform(-20, 20), 1)} for _ in range(n)]

def _loiter():
    return {
        "radius":    random.randint(50, 300),
        "direction": random.randint(0, 1),
        "time":      random.randint(10, 120),
        "speed":     round(random.uniform(10, 50), 1)
    }

def _gimbal_limits():
    return {"leftLimit": round(random.uniform(-90, 0), 1),
            "rightLimit": round(random.uniform(0, 90), 1)}

def _auto_scan():
    return {
        "gimbalPitch":           round(random.uniform(-45, 45), 1),
        "gimbalYawLimits":       _gimbal_limits(),
        "gimbalYawAngularSpeed": round(random.uniform(10, 90), 1)
    }

def _filming():
    return {
        "fieldOfView":        round(random.uniform(10, 60), 1),
        "sensorType":         random.randint(0, 3),
        "operationMode":      random.randint(0, 5),
        "coordinateOrientation": {"coordinate": _coord()},
        "lineSearch": {
            "coordinateList": [_coord() for _ in range(2)],
            "searchSpeed":    round(random.uniform(30, 60), 1)
        },
        "autoTracking":  {"targetID": rand_uint32()},
        "aircraftFixed": {"gimbalPitch": round(random.uniform(-45, 45), 1),
                          "gimbalYaw":   round(random.uniform(-180, 180), 1)},
        "autoScan": _auto_scan()
    }

def _waypoint(idx, total):
    return {
        "waypointID":        rand_uint16(),
        "coordinate":        _coord(),
        "speed":             round(random.uniform(10, 60), 1),
        "eta":               random.randint(1000, 10000),
        "ecf":               round(random.uniform(0, 1), 2),
        "nextWaypointID":    rand_uint16(),
        "waypointPassType":  random.randint(0, 2),
        "loiterProperty":    _loiter(),
        "filmingProperty":   _filming()
    }

_DEFAULT_PLAN_PATH = os.path.join(
    os.path.dirname(__file__),
    "..",                       # generator/ 상위
    "plannedMission",
    "MP-1_0303_flightPlan.json"  # ← 실제 저장 파일명
)

def make_msg0303_body(
    num_waypoints: int | None = None,
    wing_size: int | None = None,
    plan_path: str | None = None,
) -> list | dict:
    """
    ① plan_path(또는 기본 경로)에 JSON 파일이 있으면 → 내용 전체(list든 dict든) 그대로 반환  
    ② 없으면 기존 로직으로 더미(dict) 하나를 생성
    """
    path = plan_path or _DEFAULT_PLAN_PATH
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)            # list·dict 그대로

    # ─── fallback: 랜덤 더미 1개 ────────────────────────────
    if num_waypoints is None:
        num_waypoints = random.randint(3, 6)
    if wing_size is None:
        wing_size = random.randint(1, 3)

    now = int(time.time() * 1000)
    is_ff = random.choice([True, False])

    formation = (
        {
            "leaderAircraftID":      {"aircraftID": rand_int(0,6)},
            "formationDistanceList": _formation_distances(wing_size),
        }
        if is_ff
        else {
            "leaderAircraftID":      {"aircraftID": 0},
            "formationDistanceList": [],
        }
    )

    return {
        "timestamp":         now,
        "pathID":            rand_uint32(),
        "aircraftID":        rand_int(0,6),
        "isFormationFlight": is_ff,
        "formation":         formation,
        "waypointList":      [_waypoint(i, num_waypoints) for i in range(num_waypoints)],
    }

if __name__ == "__main__":
    print(json.dumps(make_msg0303_body(), ensure_ascii=False, indent=2))