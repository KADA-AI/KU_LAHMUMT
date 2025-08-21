# message0304_generator.py

import random
import json
import time
import os

UINT64_MAX = (1 << 64) - 1          # 18 446 744 073 709 551 615
rand_uint64 = lambda: random.randint(0, UINT64_MAX)

UINT32_MAX = (1 << 32) - 1
rand_uint32 = lambda: random.randint(0, UINT32_MAX)      # 0 ~ 4 294 967 295

UINT16_MAX = (1 << 16) - 1          # 65535
rand_uint16 = lambda: random.randint(0, UINT16_MAX)

rand_int   = lambda lo=0, hi=100: random.randint(lo, hi)


def _coord():
    return {
        "latitude":  round(random.uniform(-90, 90), 6),
        "longitude": round(random.uniform(-180, 180), 6),
        "altitude":  round(random.uniform(100, 1000), 1)
    }

def _hovering():
    return {
        "time": random.randint(5, 60)
    }

def _loiter():
    return {
        # 0: Unknown, 1: CW, 2: CCW
        "direction": random.randint(0, 2),
        "radius":    random.randint(10, 100),
        "time":      random.randint(10, 120),
        "speed":     round(random.uniform(5, 30), 1)
    }

def _attack():
    return {
        "targetID":   rand_uint32(),
        # 0: Unknown, 1: Type1, 2: Type2, 3: Type3
        "weaponType": random.randint(0, 3)
    }

def _waypoint(idx: int, total: int) -> dict:
    return {
        "waypointID":     rand_uint16(),
        "coordinate":     _coord(),
        "speed":          round(random.uniform(20, 80), 1),
        "eta":            random.randint(1000, 10000),   # 0 ~ 2^32-1 범위 내이므로 임의값 사용
        "ecf":            round(random.uniform(0, 1), 2), # 0 ~ 1000 범위 아니면 0~1로 조정
        "nextWaypointID": rand_uint16(),
        "hovering":       _hovering(),
        "loiter":         _loiter(),
        "attack":         _attack()
    }

_DEFAULT_PLAN_PATH = os.path.join(
    os.path.dirname(__file__),
    "..",                       # generator/ 상위
    "plannedMission",
    "MP-1_0304_lahFlightPlan.json"
)


def make_msg0304_body(
    num_waypoints: int | None = None,
    plan_path: str | None = None,
) -> list | dict:
    """
    ① plan_path(또는 기본 경로)에 JSON 파일이 있으면 → 내용 전체(list·dict) 그대로 반환  
    ② 파일이 없으면 랜덤-더미(dict 1개) 생성
    """
    path = plan_path or _DEFAULT_PLAN_PATH
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)          # list든 dict든 그대로 반환

    # ─── fallback: 랜덤-더미 한 개 ────────────────────────
    if num_waypoints is None:
        num_waypoints = random.randint(2, 5)

    now = int(time.time() * 1000)
    return {
        "timestamp":    now,
        "pathID":       rand_uint32(),
        "aircraftID":   rand_int(0,6),
        "waypointList": [_waypoint(i, num_waypoints) for i in range(num_waypoints)],
    }

if __name__ == "__main__":
    print(json.dumps(make_msg0304_body(), ensure_ascii=False, indent=2))