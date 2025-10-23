"""
MSG-0301 MissionPlan 생성 헬퍼
────────────────────────────────────────
build_mission_plan()  → 0301 JSON 한 개 반환

ID 규칙
• MissionPlanID               700 000 001부터 1씩 증가
• IndividualMissionPackageID  800 000 001부터 1씩 증가
  └ 파일 `_id_counters.json` 에 마지막 값을 기록해
    재실행·재계획 시에도 절대 중복되지 않도록 한다.
"""

from __future__ import annotations

import json, os
from pathlib import Path
from typing import List, Dict

from .mission_helpers import now_ms_since_2000



def _sw_code(default: str = "MMR") -> str:
    role = (os.environ.get("KU_ROLE") or "").lower()
    return {
        "mission": "MMR",
        "monitoring": "MSM",
        "decision": "MOB",
    }.get(role, default)


# ════════════════════════════════════════════════════════════════
# 1. Persistent ID 카운터
# ----------------------------------------------------------------
_COUNTER_FILE = Path(__file__).with_name("_id_counters.json")
_START_VALUE = {
    "missionPlanID": 700_000_001,
    "impPackageID":  800_000_001,
}

def _load_counters() -> dict[str, int]:
    if _COUNTER_FILE.is_file():
        try:
            with _COUNTER_FILE.open(encoding="utf-8") as f:
                return {k: int(v) for k, v in json.load(f).items()}
        except Exception:
            pass          # 손상되면 새로 시작
    return {}

_COUNTERS: dict[str, int] = _load_counters()

def _save_counters() -> None:
    with _COUNTER_FILE.open("w", encoding="utf-8") as f:
        json.dump(_COUNTERS, f, separators=(",", ":"))

def _next_counter(key: str) -> int:
    """
    지정 key( 'missionPlanID' | 'impPackageID' … )에 대해
    시작값부터 1씩 증가하며 유일 ID를 반환하고, 결과를 디스크에 저장.
    """
    start = _START_VALUE[key]
    nxt   = _COUNTERS.get(key, start - 1) + 1
    _COUNTERS[key] = nxt
    _save_counters()
    return nxt

# ════════════════════════════════════════════════════════════════
# 2. (선택) 외부 JSON에서 aircraft 목록 불러오기
# ----------------------------------------------------------------
def _load_aircraft_pool(filepath: os.PathLike | str) -> List[Dict]:
    """
    JSON 파일에서 aircraft 목록 추출.

    허용 형식
    1) [ { "aircraftID": 1 }, … ]          ← 최상위 리스트
    2) { "aircraftList": [ … ] }           ← 권장
    """
    fp = Path(filepath)
    if not fp.is_file():
        raise FileNotFoundError(f"[0301] no such file: {fp}")

    with fp.open(encoding="utf-8") as f:
        data = json.load(f)

    if isinstance(data, list):
        pool = data
    elif isinstance(data, dict) and "aircraftList" in data:
        pool = data["aircraftList"]
    else:
        raise ValueError(
            "[0301] JSON must be a list of aircraft dicts "
            "or contain an 'aircraftList' key."
        )

    # ── 최소 검증 ─────────────────────────────────────────
    for idx, ac in enumerate(pool, 1):
        if "aircraftID" not in ac:
            raise ValueError(f"[0301] entry #{idx} has no 'aircraftID'")
    return pool

# ─────────────────────────────────────────────────────────────
# 0) 공용 헬퍼  ─ str "1.json"  →  1   /   타입·범위 검증
# ----------------------------------------------------------------
def _parse_uint32(value: int | str | None, name: str) -> int:
    if value is None or (isinstance(value, str) and not value.strip()):
        raise ValueError(f"[0301] '{name}' is required")

    if isinstance(value, str):
        # "123.json" → 123
        value = value.rstrip(".json")
        if not value.isdigit():
            raise TypeError(f"[0301] '{name}' must be numeric or filename like '1.json'")
        value = int(value)

    if not (0 <= value <= 2**32 - 1):
        raise ValueError(f"[0301] '{name}' out of uint32 range: {value}")

    return value

# ─────────────────────────────────────────────────────────────
# 3-B) MissionPlan 스펙 검증
# ----------------------------------------------------------------
def _validate_mission_plan(mp: Dict) -> None:
    ROOT_SPECS = {
        "timestamp":                 (int,   0, 2**64 - 1),
        "missionPlanID":             (int,   0, 2**32 - 1),
        "missionPlanTimestamp":      (int,   0, 2**64 - 1),
        "planningTime":              (float, 0.0, float(2**63 - 1)),  # float(초)
        "plannerID":                 (int,   0, 6),
        "inputMissionPackageID":     (int,   0, 2**32 - 1),
        "missionReferencePackageID": (int,   0, 2**32 - 1),
    }
    for key, (typ, lo, hi) in ROOT_SPECS.items():
        if key not in mp:
            raise ValueError(f"[0301] missing '{key}'")
        # planningTime만 float 강제, 정수면 float로 승격
        if key == "planningTime" and isinstance(mp[key], int):
            mp[key] = float(mp[key])
        if not isinstance(mp[key], typ):
            raise TypeError(f"[0301] '{key}' type must be {typ.__name__}, got {type(mp[key]).__name__}")
        if not (lo <= mp[key] <= hi):
            raise ValueError(f"[0301] '{key}' out of range: {mp[key]}")

    al = mp.get("aircraftList")
    if not isinstance(al, list) or not al:
        raise ValueError("[0301] 'aircraftList' must be a non-empty list")

    seen: set[int] = set()
    for idx, ac in enumerate(al, 1):
        aid = ac.get("aircraftID")
        if not (isinstance(aid, int) and 1 <= aid <= 6):
            raise ValueError(f"[0301] aircraftList[{idx}].aircraftID must be 1..6, got: {aid}")
        if aid in seen:
            raise ValueError(f"[0301] duplicated aircraftID in aircraftList: {aid}")
        seen.add(aid)

        imp = ac.get("individualMissionPackageID")
        if not (isinstance(imp, int) and 0 <= imp <= 2**32 - 1):
            raise ValueError(f"[0301] aircraftList[{idx}].individualMissionPackageID invalid: {imp}")


# ════════════════════════════════════════════════════════════════
# 3-A) MissionPlan 빌더
# ----------------------------------------------------------------
def build_mission_plan(
    *,
    aircraft_pool: List[Dict],
    input_mission_package_id: int | str,
    mission_reference_package_id: int | str,
    mission_plan_id: int | str | None = None,
    planner_id: int = 1,
    planning_time_s: float = 0.0,
) -> Dict:
    # ── 필수 연결 ID ------------------------------------------------------
    impkg_id  = _parse_uint32(input_mission_package_id,      "inputMissionPackageID")
    refpkg_id = _parse_uint32(mission_reference_package_id,  "missionReferencePackageID")

    # ── missionPlanID -----------------------------------------------------
    if mission_plan_id is None or (isinstance(mission_plan_id, str) and not mission_plan_id.strip()):
        mission_plan_id = _next_counter("missionPlanID")
    else:
        mission_plan_id = _parse_uint32(mission_plan_id, "missionPlanID")

    # ── 공통 타임스탬프 ---------------------------------------------------
    ts_now = now_ms_since_2000()

    # ── 메시지 생성 -------------------------------------------------------
    mp = {
        "timestamp":                 ts_now,
        "Source":         _sw_code(),
        "missionPlanID":             mission_plan_id,
        "missionPlanTimestamp":      ts_now,
        "planningTime":              float(planning_time_s),  # float(초)
        "plannerID":                 planner_id,
        "inputMissionPackageID":     impkg_id,
        "missionReferencePackageID": refpkg_id,
        "aircraftList": [
            {
                "aircraftID": ac["aircraftID"],  # 1..6 가정
                "individualMissionPackageID": ac.get(
                    "individualMissionPackageID",
                    _next_counter("impPackageID"),
                ),
            }
            for ac in aircraft_pool
        ],
    }

    _validate_mission_plan(mp)
    return mp