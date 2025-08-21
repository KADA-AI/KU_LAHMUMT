import os
import json
import random
import time

# ───────── AircraftList용 랜덤 데이터 ─────────
def _make_aircraft_list(count: int):
    """AircraftList용 랜덤 데이터 생성"""
    lst = []
    for _ in range(count):
        aid = random.randint(0, 6)  # Random 항공기 ID 생성
        lst.append({
            "aircraftID": aid,  # Uint 타입: int
            "individualMissionPackageID": aid+1  # String 타입: str
        })
    return lst

_DEFAULT_PLAN_PATH = os.path.join(
    os.path.dirname(__file__),
    "..",                       # generator/ 상위
    "plannedMission",
    "MP-1_0301_missionPlan.json"
)

# ───────── MissionPlan 메시지 바디 생성 ─────────
def make_msg0301_body(
    num_aircraft: int | None = None,
    plan_path: str | None = None,
    *,                       # 키워드 전용
    strict: bool = True      # 파일이 없으면 예외? (기본 True)
) -> dict:
    """
    1) plan_path(또는 _DEFAULT_PLAN_PATH)에 JSON 파일이 존재하면 → 그 내용 그대로 반환  
    2) 파일이 없고 strict=True  → FileNotFoundError 발생  
       파일이 없고 strict=False → 랜덤-더미 데이터 생성
    """
    path = plan_path or _DEFAULT_PLAN_PATH
    if os.path.exists(path):
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)

    if strict:
        raise FileNotFoundError(f"0301 JSON not found: {path}")

    # ─── fallback: 랜덤-더미 생성 ──────────────────────────
    if num_aircraft is None:
        num_aircraft = random.randint(1, 6)

    now_ms = int(time.time() * 1000)
    id32   = now_ms & 0xFFFFFFFF
    return {
        "timestamp":                 now_ms,
        "missionPlanID":             id32,
        "missionPlanTimestamp":      now_ms,
        "planningTime":              round(random.uniform(0.1, 5.0), 3),
        "plannerID":                 random.randint(1, 10),
        "inputMissionPackageID":     (id32 + 1) & 0xFFFFFFFF,
        "missionReferencePackageID": (id32 + 2) & 0xFFFFFFFF,
        "aircraftList":              _make_aircraft_list(num_aircraft),
    }

if __name__ == "__main__":
    # 바디 출력 (디버깅용)
    print(json.dumps(make_msg0301_body(), ensure_ascii=False, indent=2))
