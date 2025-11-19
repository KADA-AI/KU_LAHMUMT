"""
ID allocator – 0301/0302/0303 공통
임무 재계획 시에도 중복을 막기 위해 마지막 번호를 json 파일에 저장해 둔다.
"""
from pathlib import Path
import json, threading, time

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
VOLATILE_KEYS = {"waypoint"}

def _load() -> dict:
    """
    디스크의 ID 상태를 읽어온다.
    - 파일 없음: {} 반환
    - 파일이 비었거나(JSONDecodeError) 손상: .bak로 1회 백업 후 {} 반환
    """
    try:
        if not _STORE.exists():
            return {}
        # 빈 파일(사이즈 0) 처리
        if _STORE.stat().st_size == 0:
            return {}
        with _STORE.open("r", encoding="utf-8") as f:
            data = json.load(f)
            return data if isinstance(data, dict) else {}
    except json.JSONDecodeError:
        # 손상된 경우 자동 백업(기존 .bak 없을 때만)
        try:
            bak = _STORE.with_suffix(_STORE.suffix + ".bak")
            if not bak.exists():
                _STORE.replace(bak)
        except Exception:
            pass
        return {}
    except Exception:
        return {}

def _save(state: dict) -> None:
    """
    ID 상태를 안전하게 덮어쓰다.
    """
    _STORE.parent.mkdir(parents=True, exist_ok=True)
    tmp = _STORE.with_suffix(_STORE.suffix + ".tmp")
    with tmp.open("w", encoding="utf-8") as f:
        json.dump(state, f, ensure_ascii=False, separators=(",", ":"))
    for attempt in range(5):
        try:
            tmp.replace(_STORE)
            return
        except PermissionError:
            time.sleep(0.1 * (attempt + 1))
        except Exception:
            try:
                tmp.unlink()
            except Exception:
                pass
            raise
    try:
        tmp.unlink()
    except Exception:
        pass
    raise PermissionError(f"id_tracker write failed after retries: {_STORE}")
_state = _load()              # {key: last_used}

for _volatile_key in list(VOLATILE_KEYS):
    if _volatile_key in _state:
        try:
            del _state[_volatile_key]
            _save(_state)
        except Exception:
            pass

_volatile_counters = {key: BASE[key] - 1 for key in VOLATILE_KEYS}

def _next(key: str, inc: int = 1, subkey=None) -> int:
    with _LOCK:
        if key in VOLATILE_KEYS:
            if subkey is not None:
                raise ValueError(f"volatile key '{key}' does not support subkey allocation")
            cur = _volatile_counters.get(key, BASE[key] - 1)
            cur += inc
            _volatile_counters[key] = cur
            return cur

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
