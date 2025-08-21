# message0302_generator.py
import os
import random
import json
import time

UINT32_MAX = (1 << 32) - 1
rand_uint32 = lambda: random.randint(0, UINT32_MAX)      # 0 ~ 4 294 967 295
rand_int   = lambda lo=0, hi=100: random.randint(lo, hi)
# ───────── 헬퍼 ─────────
def _make_coordinate():
    return {
        "latitude":  round(random.uniform(-90, 90), 6),
        "longitude": round(random.uniform(-180, 180), 6),
        "altitude":  round(random.uniform(50, 500), 1)
    }

def _make_line():
    return {
        "width": round(random.uniform(5, 30), 1),
        "coordinateList": [_make_coordinate() for _ in range(2)]
    }

def _make_area():
    return {
        "isHole": random.choice([False, True]),
        "coordinateList": [_make_coordinate() for _ in range(4)]
    }

def _make_individual_mission(seq: int) -> dict:
    """seq: 1,2,3…  (더미라도 ID·pathID 중복 방지)"""
    info = {
        "individualMissionType": random.randint(0, 2),   # 0 None / 1 Area / 2 Corridor
        "patternType":           random.randint(0, 3),
        "autoZoomIn":            random.choice([False, True]),
        "coordinateList":        [_make_coordinate() for _ in range(random.randint(1, 3))],
        "lineList":              [_make_line() for _ in range(random.randint(0, 2))],
        "areaList":              [_make_area() for _ in range(random.randint(0, 1))],
        "targetID":              rand_uint32(),
    }
    return {
        "individualMissionID": seq,          # uint32 -- 중복 없이
        "isDone":              False,
        "relatedMission": {
            "relatedMissionType": 0,
            "inputMissionID":     0,
            "priorMissionID":     0,
        },
        "individualMissionInfo": info,
        "pathID":               seq,         # uint32
    }


_DEFAULT_PLAN_PATH = os.path.join(
    os.path.dirname(__file__),
    "..",                       # generator/ 상위
    "plannedMission",
    "MP-1_0302_individualMissions.json"
)

# ───────── 바디 생성 ─────────
def make_msg0302_body(
    plan_path: str | None = None,
    num_aircraft: int | None = None,
    num_missions_each: int | None = None,
    *,                       # 키워드 전용
    strict: bool = True      # 파일 없으면 예외? (기본 True)
) -> list | dict:
    """
    ① plan_path(또는 _DEFAULT_PLAN_PATH)에 JSON 존재 → 그 내용 그대로 반환  
    ② 없고 strict=True  → FileNotFoundError 발생  
       없고 strict=False → 중복 없는 더미 list 생성
    """
    path = plan_path or _DEFAULT_PLAN_PATH
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)

    if strict:
        raise FileNotFoundError(f"0302 JSON not found: {path}")

    # ─── 더미 생성 (중복 없는 aircraftID) ────────────────────────
    if num_aircraft is None:
        num_aircraft = random.randint(1, 6)
    if num_missions_each is None:
        num_missions_each = random.randint(1, 3)

    aid_pool     = random.sample(range(1, 7), k=num_aircraft)   # 1-6 중복 없음
    plan_pkg_id  = rand_uint32()                                # 모든 A/C 공유
    now_ms       = int(time.time() * 1000)

    dummy_list = []
    for aid in aid_pool:
        dummy_list.append({
            "timestamp":                  now_ms,
            "individualMissionPackageID": plan_pkg_id,
            "aircraftID":                 aid,
            "individualMissionList": [
                _make_individual_mission(j + 1)
                for j in range(num_missions_each)
            ],
        })
    return dummy_list

if __name__ == "__main__":
    print(json.dumps(make_msg0302_body(), ensure_ascii=False, indent=2))
