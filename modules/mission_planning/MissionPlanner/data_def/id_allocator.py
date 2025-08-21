"""
ID allocator – 0301/0302/0303 공통
임무 재계획 시에도 중복을 막기 위해 마지막 번호를 json 파일에 저장해 둔다.
"""
from pathlib import Path
import json, threading

_LOCK = threading.Lock()
_STORE = Path(__file__).resolve().parent / "id_tracker.json"

# ── 초기 값 & 증분 규칙 ───────────────────────────────────────
BASE = {
    "missionPlanID":            700_000_001,
    "individualMissionPackage": 800_000_001,
    "individualMission":        900_000_001,
    # pathID – aircraftGroup 별
    "pathID": {
        1: 100_000_001, 2: 200_000_001, 3: 300_000_001,
        4: 400_000_001, 5: 500_000_001, 6: 600_000_001,
    },
    "waypoint": 1,             # 필요한 경우
}

def _load():
    if _STORE.exists():
        with _STORE.open() as f:
            return json.load(f)
    return {}

def _save(data):
    _STORE.write_text(json.dumps(data, indent=2))

_state = _load()              # {key: last_used}

def _next(key: str, inc: int = 1, subkey=None) -> int:
    with _LOCK:
        if subkey is None:
            cur = _state.get(key, BASE[key] - 1)
            cur += inc
            _state[key] = cur
        else:                 # pathID
            path_map = _state.setdefault(key, {})
            cur = path_map.get(subkey, BASE[key][subkey] - 1)
            cur += inc
            path_map[subkey] = cur
        _save(_state)
        return cur

# ── public helpers ──────────────────────────────────────────
def next_mission_plan_id():            return _next("missionPlanID")
def next_imp_id():                     return _next("individualMissionPackage")
def next_individual_mission_id():      return _next("individualMission")
def next_path_id(aircraft_id: int):    return _next("pathID", subkey=aircraft_id)
def next_waypoint_id():                return _next("waypoint")
