"""
ID allocator – 0301/0302/0303 공통
임무 재계획 시에도 중복을 막기 위해 마지막 번호를 json 파일에 저장해 둔다.
"""
from pathlib import Path
import json, logging, threading, time
from contextlib import contextmanager
from datetime import datetime, timezone
import os
from modules.common import db_paths

try:  # Windows
    import msvcrt  # type: ignore
except Exception:  # pragma: no cover - non-Windows fallback
    msvcrt = None  # type: ignore

try:  # POSIX
    import fcntl  # type: ignore
except Exception:  # pragma: no cover - Windows fallback
    fcntl = None  # type: ignore

_LOCK = threading.Lock()
_LOG = logging.getLogger(__name__)
_LEGACY_STORE = Path(__file__).resolve().parent / "id_tracker.json"
_STORE = _LEGACY_STORE
_FILE_LOCK_LOCAL = threading.local()

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
    "waypoint": 50,            # WaypointID starts at 50 by mission-planning rule.
}
VOLATILE_KEYS = {"waypoint"}
_ACTIVE_DB_ROOT = None


@contextmanager
def _store_file_lock(path: Path | None = None):
    """Serialize id_tracker read-modify-write across processes."""
    target = Path(path) if path is not None else _STORE
    depth = int(getattr(_FILE_LOCK_LOCAL, "depth", 0) or 0)
    if depth > 0:
        _FILE_LOCK_LOCAL.depth = depth + 1
        try:
            yield
        finally:
            _FILE_LOCK_LOCAL.depth = depth
        return

    target.parent.mkdir(parents=True, exist_ok=True)
    lock_path = target.with_name(f"{target.name}.lock")
    lock_file = lock_path.open("a+b")
    _FILE_LOCK_LOCAL.depth = 1
    acquired = False
    try:
        lock_file.seek(0, os.SEEK_END)
        if lock_file.tell() == 0:
            lock_file.write(b"\0")
            lock_file.flush()
        lock_file.seek(0)
        if msvcrt is not None:
            msvcrt.locking(lock_file.fileno(), msvcrt.LK_LOCK, 1)
        elif fcntl is not None:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX)
        acquired = True
        yield
    finally:
        try:
            if acquired:
                lock_file.seek(0)
                if msvcrt is not None:
                    msvcrt.locking(lock_file.fileno(), msvcrt.LK_UNLCK, 1)
                elif fcntl is not None:
                    fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)
        finally:
            _FILE_LOCK_LOCAL.depth = 0
            lock_file.close()


def _resolve_store_path() -> Path:
    try:
        return db_paths.get_db_subpath("DSS_Internal") / "id_tracker.json"
    except Exception:
        return _LEGACY_STORE

def _normalize_state(state: dict | None) -> dict:
    if not isinstance(state, dict):
        return {}
    normalized = dict(state)
    raw_path_map = normalized.get("pathID")
    if isinstance(raw_path_map, dict):
        path_map: dict[int, int] = {}
        for raw_key, raw_value in raw_path_map.items():
            try:
                aid = int(raw_key)
                value = int(raw_value)
            except (TypeError, ValueError):
                continue
            prev = path_map.get(aid)
            if prev is None or value > prev:
                path_map[aid] = value
        normalized["pathID"] = path_map
    return normalized

def _load(path: Path | None = None) -> dict:
    """
    디스크의 ID 상태를 읽어온다.
    - 파일 없음: {} 반환
    - 파일이 비었거나(JSONDecodeError) 손상: .bak로 1회 백업 후 {} 반환
    """
    target = Path(path) if path is not None else _STORE
    started = time.perf_counter()
    try:
        with _store_file_lock(target):
            if not target.exists():
                return {}
            # 빈 파일(사이즈 0) 처리
            if target.stat().st_size == 0:
                return {}
            with target.open("r", encoding="utf-8") as f:
                data = json.load(f)
                return _normalize_state(data)
    except json.JSONDecodeError:
        # 손상된 경우 자동 백업(기존 .bak 없을 때만)
        try:
            bak = target.with_suffix(target.suffix + ".bak")
            if not bak.exists():
                target.replace(bak)
        except Exception:
            pass
        _LOG.warning(
            "id_tracker JSON decode failed: path=%s elapsed_ms=%.1f",
            target,
            (time.perf_counter() - started) * 1000.0,
            exc_info=True,
        )
        return {}
    except Exception:
        _LOG.warning(
            "id_tracker load failed: path=%s elapsed_ms=%.1f",
            target,
            (time.perf_counter() - started) * 1000.0,
            exc_info=True,
        )
        return {}



def _read_store_state(path: Path | None = None) -> dict:
    """현재 디스크 상태를 불러와 최신 값을 유지한다."""
    target = path or _STORE
    try:
        with _store_file_lock(target):
            if not target.exists() or target.stat().st_size == 0:
                return {}
            with target.open('r', encoding='utf-8') as f:
                data = json.load(f)
                return _normalize_state(data)
    except Exception:
        return {}


def _save(state: dict, path: Path | None = None) -> None:
    """
    ID 상태를 안전하게 덮어쓰다.
    """
    store_path = path or _STORE
    with _store_file_lock(store_path):
        state = _normalize_state(state)
        existing = _read_store_state(store_path)
        if existing:
            for key, value in existing.items():
                if key in VOLATILE_KEYS:
                    continue
                if key == 'pathID':
                    if not isinstance(value, dict):
                        continue
                    path_state = state.setdefault('pathID', {})
                    for subkey, subval in value.items():
                        try:
                            aid = int(subkey)
                            existing_val = int(subval)
                        except (TypeError, ValueError):
                            continue
                        current_val = path_state.get(aid)
                        try:
                            current_int = int(current_val)
                        except (TypeError, ValueError):
                            current_int = None
                        if current_int is None or existing_val > current_int:
                            path_state[aid] = existing_val
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

        store_path.parent.mkdir(parents=True, exist_ok=True)
        tmp = store_path.with_name(
            f"{store_path.name}.{os.getpid()}.{threading.get_ident()}.tmp"
        )
        with tmp.open("w", encoding="utf-8") as f:
            json.dump(_normalize_state(state), f, ensure_ascii=False, separators=(",", ":"))
        for attempt in range(5):
            try:
                tmp.replace(store_path)
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
        raise PermissionError(f"id_tracker write failed after retries: {store_path}")
_state: dict = {}              # {key: last_used}

_volatile_counters = {key: BASE[key] - 1 for key in VOLATILE_KEYS}

def _read_last_waypoint_usage() -> int | None:
    try:
        log_dir = db_paths.get_db_subpath("DSS_Internal")
        usage_file = log_dir / "waypoint_usage.json"
        if not usage_file.exists():
            return None
        data = json.loads(usage_file.read_text(encoding="utf-8"))
        return int(data.get("last_waypoint_id"))
    except Exception:
        return None


def _scan_last_waypoint_id() -> int | None:
    try:
        flight_path_dir = db_paths.get_db_subpath("FlightPath")
    except Exception:
        return None
    if not flight_path_dir.exists():
        return None
    max_waypoint_id: int | None = None
    try:
        for path in flight_path_dir.glob("*.json"):
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
            except Exception:
                continue
            for list_key in ("waypointList", "lahWaypointList"):
                waypoint_list = data.get(list_key)
                if not isinstance(waypoint_list, list):
                    continue
                for waypoint in waypoint_list:
                    if not isinstance(waypoint, dict):
                        continue
                    try:
                        waypoint_id = int(waypoint.get("waypointID"))
                    except Exception:
                        continue
                    if waypoint_id <= 0:
                        continue
                    if max_waypoint_id is None or waypoint_id > max_waypoint_id:
                        max_waypoint_id = waypoint_id
    except Exception:
        return max_waypoint_id
    return max_waypoint_id


def _refresh_waypoint_counter() -> None:
    if "waypoint" not in _volatile_counters:
        return
    current = int(_volatile_counters.get("waypoint", BASE["waypoint"] - 1))
    latest_usage = _read_last_waypoint_usage()
    latest_scan = _scan_last_waypoint_id()
    candidates = [
        current,
        int(latest_usage) if latest_usage is not None else BASE["waypoint"] - 1,
        int(latest_scan) if latest_scan is not None else BASE["waypoint"] - 1,
    ]
    _volatile_counters["waypoint"] = max(candidates)


def _read_path_usage_map() -> dict[int, int]:
    try:
        log_dir = db_paths.get_db_subpath("DSS_Internal")
        usage_file = log_dir / "path_usage.json"
        if not usage_file.exists():
            return {}
        data = json.loads(usage_file.read_text(encoding="utf-8"))
    except Exception:
        return {}
    aircraft_map = data.get("aircraft")
    if not isinstance(aircraft_map, dict):
        return {}
    results: dict[int, int] = {}
    for key, value in aircraft_map.items():
        try:
            aid = int(key)
            path_id = int(value)
        except (TypeError, ValueError):
            continue
        prev = results.get(aid)
        if prev is None or path_id > prev:
            results[aid] = path_id
    return results


def _scan_numeric_stem_max(directory_name: str) -> int | None:
    try:
        directory = db_paths.get_db_subpath(directory_name)
    except Exception:
        return None
    if not directory.exists():
        return None
    last_value: int | None = None
    try:
        for path in directory.glob("*.json"):
            stem = path.stem
            if not stem.isdigit():
                continue
            value = int(stem)
            if last_value is None or value > last_value:
                last_value = value
    except Exception:
        return last_value
    return last_value


def _scan_last_individual_mission_id() -> int | None:
    try:
        imp_dir = db_paths.get_db_subpath("IndividualMissionPlan")
    except Exception:
        return None
    if not imp_dir.exists():
        return None
    last_value: int | None = None
    try:
        for path in imp_dir.glob("*.json"):
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
            except Exception:
                continue
            mission_list = data.get("individualMissionList")
            if not isinstance(mission_list, list):
                continue
            for mission in mission_list:
                if not isinstance(mission, dict):
                    continue
                try:
                    mission_id = int(mission.get("individualMissionID"))
                except Exception:
                    continue
                if mission_id <= 0:
                    continue
                if last_value is None or mission_id > last_value:
                    last_value = mission_id
    except Exception:
        return last_value
    return last_value


def _scan_last_path_ids() -> dict[int, int]:
    try:
        flight_path_dir = db_paths.get_db_subpath("FlightPath")
    except Exception:
        return {}
    if not flight_path_dir.exists():
        return {}
    results: dict[int, int] = {}
    try:
        for path in flight_path_dir.glob("*.json"):
            stem = path.stem
            if not stem.isdigit():
                continue
            path_id = int(stem)
            aircraft_id = path_id // 100_000_000
            if aircraft_id not in BASE["pathID"]:
                continue
            prev = results.get(aircraft_id)
            if prev is None or path_id > prev:
                results[aircraft_id] = path_id
    except Exception:
        return results
    return results


def _build_scope_state() -> dict:
    state = _normalize_state(_load(_STORE))
    for volatile_key in list(VOLATILE_KEYS):
        state.pop(volatile_key, None)

    for key in ("missionPlanID", "individualMissionPackage", "individualMission"):
        base_value = BASE[key] - 1
        try:
            current_value = int(state.get(key, base_value))
        except (TypeError, ValueError):
            current_value = base_value
        if key == "missionPlanID":
            scanned_value = _scan_numeric_stem_max("MissionPlan")
        elif key == "individualMissionPackage":
            scanned_value = _scan_numeric_stem_max("IndividualMissionPlan")
        else:
            scanned_value = _scan_last_individual_mission_id()
        candidates = [base_value, current_value]
        if scanned_value is not None:
            candidates.append(int(scanned_value))
        state[key] = max(candidates)

    current_path_map = state.get("pathID")
    if not isinstance(current_path_map, dict):
        current_path_map = {}
    usage_path_map = _read_path_usage_map()
    scanned_path_map = _scan_last_path_ids()
    next_path_map: dict[int, int] = {}
    for aircraft_id, base_value in BASE["pathID"].items():
        candidates = [int(base_value) - 1]
        try:
            candidates.append(int(current_path_map.get(aircraft_id, base_value - 1)))
        except (TypeError, ValueError):
            pass
        try:
            candidates.append(int(usage_path_map.get(aircraft_id, base_value - 1)))
        except (TypeError, ValueError):
            pass
        try:
            candidates.append(int(scanned_path_map.get(aircraft_id, base_value - 1)))
        except (TypeError, ValueError):
            pass
        next_path_map[aircraft_id] = max(candidates)
    state["pathID"] = next_path_map
    return state

def _sync_active_db_scope() -> None:
    global _ACTIVE_DB_ROOT, _STORE, _state
    try:
        active_root = str(db_paths.get_active_db_root())
    except Exception:
        return
    if active_root == _ACTIVE_DB_ROOT:
        return
    _ACTIVE_DB_ROOT = active_root
    _STORE = _resolve_store_path()
    _state = _build_scope_state()
    for key in VOLATILE_KEYS:
        _volatile_counters[key] = BASE[key] - 1
    try:
        _save(_state)
    except Exception:
        pass
    _seed_waypoint_counter()

def _seed_waypoint_counter() -> None:
    if "waypoint" not in _volatile_counters:
        return
    usage_value = _read_last_waypoint_usage()
    scan_value = _scan_last_waypoint_id()
    candidates = [
        int(_volatile_counters.get("waypoint", BASE["waypoint"] - 1)),
        int(usage_value) if usage_value is not None else BASE["waypoint"] - 1,
        int(scan_value) if scan_value is not None else BASE["waypoint"] - 1,
    ]
    _volatile_counters["waypoint"] = max(candidates)

_sync_active_db_scope()
def _next(key: str, inc: int = 1, subkey=None) -> int:
    with _LOCK:
        _sync_active_db_scope()
        if key in VOLATILE_KEYS:
            if subkey is not None:
                raise ValueError(f"volatile key '{key}' does not support subkey allocation")
            with _store_file_lock(_STORE):
                if key == "waypoint":
                    _refresh_waypoint_counter()
                cur = _volatile_counters.get(key, BASE[key] - 1)
                cur += inc
                _volatile_counters[key] = cur
                if key == "waypoint":
                    _record_waypoint_usage(cur)
                return cur

        with _store_file_lock(_STORE):
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


def _reserve_range(key: str, count: int, subkey=None) -> tuple[int, int]:
    count_int = max(0, int(count or 0))
    if count_int <= 0:
        raise ValueError("count must be >= 1")

    with _LOCK:
        _sync_active_db_scope()
        if key in VOLATILE_KEYS:
            if subkey is not None:
                raise ValueError(f"volatile key '{key}' does not support subkey allocation")
            with _store_file_lock(_STORE):
                if key == "waypoint":
                    _refresh_waypoint_counter()
                current_base = _volatile_counters.get(key, BASE[key] - 1)
                start = int(current_base + 1)
                end = int(current_base + count_int)
                _volatile_counters[key] = end
                if key == "waypoint":
                    _record_waypoint_usage(end)
                return start, end

        with _store_file_lock(_STORE):
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
                start = int(max(mem_value, disk_value) + 1)
                end = int(start + count_int - 1)
                _state[key] = end
            else:
                path_map = _state.setdefault(key, {})
                disk_map = disk_state.get(key, {})
                base_value = BASE[key][subkey] - 1
                try:
                    disk_value = int(disk_map.get(subkey, base_value))
                except (TypeError, ValueError):
                    disk_value = base_value
                try:
                    mem_value = int(path_map.get(subkey, base_value))
                except (TypeError, ValueError):
                    mem_value = base_value
                start = int(max(mem_value, disk_value) + 1)
                end = int(start + count_int - 1)
                path_map[subkey] = end

            _save(_state)
        if key == "pathID" and subkey is not None:
            _record_path_usage(subkey, end)
        return start, end

# ── public helpers ──────────────────────────────────────────
def next_mission_plan_id():            return _next("missionPlanID")
def next_imp_id():                     return _next("individualMissionPackage")
def next_individual_mission_id():      return _next("individualMission")
def next_path_id(aircraft_id: int):    return _next("pathID", subkey=aircraft_id)
def next_waypoint_id():                return _next("waypoint")
def reserve_mission_plan_ids(count: int):
    start, end = _reserve_range("missionPlanID", count)
    return list(range(start, end + 1))
def reserve_imp_ids(count: int):
    start, end = _reserve_range("individualMissionPackage", count)
    return list(range(start, end + 1))
def reserve_individual_mission_ids(count: int):
    start, end = _reserve_range("individualMission", count)
    return list(range(start, end + 1))
def reserve_path_ids(aircraft_id: int, count: int):
    start, end = _reserve_range("pathID", count, subkey=aircraft_id)
    return list(range(start, end + 1))
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
