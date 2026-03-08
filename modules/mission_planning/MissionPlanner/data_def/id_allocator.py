"""
ID allocator – 0301/0302/0303 공통
임무 재계획 시에도 중복을 막기 위해 마지막 번호를 json 파일에 저장해 둔다.
"""
from pathlib import Path
import json, threading, time
from datetime import datetime, timezone
from modules.common import db_paths

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



def _read_store_state() -> dict:
    """현재 디스크 상태를 불러와 최신 값을 유지한다."""
    try:
        if not _STORE.exists() or _STORE.stat().st_size == 0:
            return {}
        with _STORE.open('r', encoding='utf-8') as f:
            data = json.load(f)
            return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def _save(state: dict) -> None:
    """
    ID 상태를 안전하게 덮어쓰다.
    """
    existing = _read_store_state()
    if existing:
        for key, value in existing.items():
            if key in VOLATILE_KEYS:
                continue
            if key == 'pathID':
                if not isinstance(value, dict):
                    continue
                target = state.setdefault('pathID', {})
                for subkey, subval in value.items():
                    try:
                        aid = int(subkey)
                        existing_val = int(subval)
                    except (TypeError, ValueError):
                        continue
                    current_val = target.get(aid)
                    try:
                        current_int = int(current_val)
                    except (TypeError, ValueError):
                        current_int = None
                    if current_int is None or existing_val > current_int:
                        target[aid] = existing_val
                continue
            try:
                existing_int = int(value)
            except (TypeError, ValueError):
                continue
            current_val = state.get(key)
            try:
                current_int = int(current_val)
            except (TypeError, ValueError):
                current_int = None
            if current_int is None or existing_int > current_int:
                state[key] = existing_int

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

def _seed_waypoint_counter() -> None:
    if "waypoint" not in _volatile_counters:
        return
    try:
        log_dir = db_paths.get_db_subpath("DSS_Internal")
        usage_file = log_dir / "waypoint_usage.json"
        if not usage_file.exists():
            return
        data = json.loads(usage_file.read_text(encoding="utf-8"))
        last_value = int(data.get("last_waypoint_id"))
        _volatile_counters["waypoint"] = last_value
    except Exception:
        return

def _seed_path_counters() -> None:
    try:
        log_dir = db_paths.get_db_subpath("DSS_Internal")
        usage_file = log_dir / "path_usage.json"
        if not usage_file.exists():
            return
        data = json.loads(usage_file.read_text(encoding="utf-8"))
    except Exception:
        return
    aircraft_map = data.get("aircraft")
    if not isinstance(aircraft_map, dict):
        return
    path_map = _state.setdefault("pathID", {})
    changed = False
    for key, value in aircraft_map.items():
        try:
            aid = int(key)
            last_value = int(value)
        except (TypeError, ValueError):
            continue
        base_value = BASE["pathID"].get(aid, 0) - 1
        current = path_map.get(aid, base_value)
        if last_value > current:
            path_map[aid] = last_value
            changed = True
    if changed:
        try:
            _save(_state)
        except Exception:
            pass

_seed_waypoint_counter()
_seed_path_counters()
def _next(key: str, inc: int = 1, subkey=None) -> int:
    with _LOCK:
        if key in VOLATILE_KEYS:
            if subkey is not None:
                raise ValueError(f"volatile key '{key}' does not support subkey allocation")
            cur = _volatile_counters.get(key, BASE[key] - 1)
            cur += inc
            _volatile_counters[key] = cur
            if key == "waypoint":
                _record_waypoint_usage(cur)
            return cur

        disk_state = _read_store_state()

        if subkey is None:
            current_base = BASE.get(key, 0) - 1
            try:
                disk_value = int(disk_state.get(key, current_base))
            except (TypeError, ValueError):
                disk_value = current_base
            try:
                mem_value = int(_state.get(key, current_base))
            except (TypeError, ValueError):
                mem_value = current_base
            cur = max(mem_value, disk_value) + inc
            _state[key] = cur
        else:                 # pathID
            path_map = _state.setdefault(key, {})
            disk_map = disk_state.get(key, {})
            try:
                disk_value = int(disk_map.get(subkey, BASE[key][subkey] - 1))
            except (TypeError, ValueError):
                disk_value = BASE[key][subkey] - 1
            try:
                mem_value = int(path_map.get(subkey, BASE[key][subkey] - 1))
            except (TypeError, ValueError):
                mem_value = BASE[key][subkey] - 1
            cur = max(mem_value, disk_value) + inc
            path_map[subkey] = cur
        _save(_state)
        if key == "pathID" and subkey is not None:
            _record_path_usage(subkey, cur)
        return cur

# ── public helpers ──────────────────────────────────────────
def next_mission_plan_id():            return _next("missionPlanID")
def next_imp_id():                     return _next("individualMissionPackage")
def next_individual_mission_id():      return _next("individualMission")
def next_path_id(aircraft_id: int):    return _next("pathID", subkey=aircraft_id)
def next_waypoint_id():                return _next("waypoint")
def reserve_waypoint_block(count: int) -> int:
    count_int = max(0, int(count or 0))
    if count_int <= 0:
        raise ValueError("count must be >= 1")
    last_id = _next("waypoint", inc=count_int)
    return int(last_id - count_int + 1)


def _record_waypoint_usage(value: int) -> None:
    try:
        log_dir = db_paths.get_db_subpath("DSS_Internal")
    except Exception:
        return
    try:
        log_dir.mkdir(parents=True, exist_ok=True)
        target = log_dir / "waypoint_usage.json"
        payload = {
            "last_waypoint_id": int(value),
            "updated_at": datetime.now(timezone.utc).isoformat(),
        }
        tmp = target.with_suffix(".tmp")
        tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        tmp.replace(target)
    except Exception:
        return

def _record_path_usage(aircraft_id: int, value: int) -> None:
    try:
        log_dir = db_paths.get_db_subpath("DSS_Internal")
    except Exception:
        return
    try:
        log_dir.mkdir(parents=True, exist_ok=True)
        target = log_dir / "path_usage.json"
        if target.exists():
            try:
                data = json.loads(target.read_text(encoding="utf-8"))
            except Exception:
                data = {}
        else:
            data = {}
        aircraft_map = data.setdefault("aircraft", {})
        aircraft_map[str(int(aircraft_id))] = int(value)
        data["updated_at"] = datetime.now(timezone.utc).isoformat()
        tmp = target.with_suffix(".tmp")
        tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
        tmp.replace(target)
    except Exception:
        return
