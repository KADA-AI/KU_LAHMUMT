# message0201_generator.py
import random
import time
import json

# ───────── 헬퍼 함수 ─────────
def _rand_str8():
    return ''.join(random.choices("ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789", k=8))

def _rand_int(lo: int, hi: int) -> int:
    return random.randint(lo, hi)

def _rand_float(lo: float, hi: float, nd: int = 6) -> float:
    return round(random.uniform(lo, hi), nd)

def _rand_bool() -> bool:
    return bool(random.getrandbits(1))

def _rand_lat() -> float:
    # -90 ~ 90 범위
    return _rand_float(-90, 90, 6)

def _rand_lon() -> float:
    # -180 ~ 180 범위
    return _rand_float(-180, 180, 6)

def _rand_alt() -> float:
    # 0 ~ 50000 범위
    return _rand_float(0, 50000, 1)

def _make_coordinate() -> dict:
    return {
        "latitude":  _rand_lat(),
        "longitude": _rand_lon(),
        "altitude":  _rand_alt()
    }

def _make_line() -> dict:
    return {
        "width": _rand_alt(),  # 0 ~ 50000
        "coordinateList": [
            _make_coordinate()
            for _ in range(random.randint(1, 3))
        ]
    }

def _make_area() -> dict:
    return {
        "isHole": _rand_bool(),  # True: 해당 영역 제외, False: 해당 영역 임무지역
        "coordinateList": [
            _make_coordinate()
            for _ in range(random.randint(1, 3))
        ]
    }

def _make_mission_detail() -> dict:
    return {
        "coordinateList": [
            _make_coordinate()
            for _ in range(random.randint(1, 3))
        ],
        "lineList": [
            _make_line()
            for _ in range(random.randint(0, 2))
        ],
        "areaList": [
            _make_area()
            for _ in range(random.randint(0, 2))
        ]
    }

def _make_input_mission() -> dict:
    return {
        "inputMissionID":   _rand_int(1000, 9999),
        # 0~7 범위 내 (0: Not used, 1~7: 지정된 임무 타입, 7 이상 Reserved)
        "inputMissionType": _rand_int(0, 7),
        "isDone":           _rand_bool(),
        "missionDetail":    _make_mission_detail()
    }

# ───────── 바디 생성 함수 ─────────
def make_msg0201_body() -> dict:
    """
    InputMissionPlan(0201) 메시지 바디 생성 (소문자 카멜)
    """
    now_ms = int(time.time() * 1000)

    # 가용 항공기 ID 목록 (1~6번 중에서 1~6개 선택)
    # 1: 지휘기, 2: 편대기1, 3: 편대기2, 4: UAV#1, 5: UAV#2, 6: UAV#3
    available_ids = random.sample(range(1, 7), k=random.randint(1, 6))

    # 입력 임무 리스트 (1~3개)
    mission_count = random.randint(1, 3)
    return {
        "timestamp":               now_ms,
        "inputMissionPackageID":   _rand_int(1000, 9999),
        # 0~7 범위 내 (0: Not used, 1~6: 지정된 패키지 타입, 7: Reserved)
        "inputMissionPackageType": _rand_int(0, 7),
        
        # 0~2 범위 내 (0: Not used, 1: EO, 2: IR)
        "mainSensor":              _rand_int(0, 2),
        "availableAircraftList": [
            {"aircraftID": aid}
            for aid in available_ids
        ],
        "inputMissionList": [
            _make_input_mission()
            for _ in range(mission_count)
        ]
    }

if __name__ == "__main__":
    print(json.dumps(make_msg0201_body(), ensure_ascii=False, indent=2))
