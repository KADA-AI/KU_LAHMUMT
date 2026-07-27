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
_MISSION_PLANNING_ROOT = next(
    (
        parent
        for parent in Path(__file__).resolve().parents
        if (parent / "MissionPlanner" / "data_def").exists()
    ),
    Path(__file__).resolve().parents[3],
)
_LEGACY_STORE = _MISSION_PLANNING_ROOT / "MissionPlanner" / "data_def" / "id_tracker.json"
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
_WAYPOINT_SIGNATURE_CACHE: dict | None = None
_WAYPOINT_SCAN_RESULT_CACHE: dict[tuple, int | None] = {}
_RESERVATION_SNAPSHOT_LOCAL = threading.local()


def _reservation_snapshot_cache() -> dict | None:
    cache = getattr(_RESERVATION_SNAPSHOT_LOCAL, "cache", None)
    return cache if isinstance(cache, dict) else None


@contextmanager
def _reservation_snapshot_scope():
    depth = int(getattr(_RESERVATION_SNAPSHOT_LOCAL, "depth", 0) or 0)
    if depth <= 0:
        _RESERVATION_SNAPSHOT_LOCAL.cache = {}
    _RESERVATION_SNAPSHOT_LOCAL.depth = depth + 1
    try:
        yield
    finally:
        next_depth = int(getattr(_RESERVATION_SNAPSHOT_LOCAL, "depth", 1) or 1) - 1
        if next_depth <= 0:
            _RESERVATION_SNAPSHOT_LOCAL.depth = 0
            _RESERVATION_SNAPSHOT_LOCAL.cache = None
        else:
            _RESERVATION_SNAPSHOT_LOCAL.depth = next_depth


def _env_flag(name: str, default: bool = True) -> bool:
    raw = os.getenv(name)
    if raw is None:
        return bool(default)
    return str(raw).strip().lower() not in {"0", "false", "no", "off"}


def _env_float(name: str, default: float, *, minimum: float | None = None) -> float:
    raw = os.getenv(name)
    if raw is None:
        value = float(default)
    else:
        try:
            value = float(str(raw).strip())
        except (TypeError, ValueError):
            value = float(default)
    if minimum is not None and value < float(minimum):
        return float(minimum)
    return value


def _emit_id_metric(event: str, elapsed_ms: float | None = None, **fields) -> None:
    if not _env_flag("REPLAN_ID_ALLOCATOR_TIMING", True):
        return
    payload = {
        "event": str(event),
        "module": "mission_planning",
        "component": "id_allocator",
        "processId": os.getpid(),
        "threadId": threading.get_ident(),
    }
    if elapsed_ms is not None:
        payload["elapsedMs"] = round(float(elapsed_ms), 3)
    for key, value in fields.items():
        try:
            json.dumps(value)
            payload[key] = value
        except Exception:
            payload[key] = str(value)
    line = "[REPLAN][ID] " + json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
    try:
        from modules.common.process_console import emit_process_log

        emit_process_log("mission_planning", line)
    except Exception:
        try:
            _LOG.info(line)
        except Exception:
            pass


def _blocking_file_lock(lock_file) -> None:
    if msvcrt is not None:
        # Windows LK_LOCK retries only once per second.  ID reservations are
        # short transactions, so a momentary overlap otherwise adds a false
        # ~1 s pause to replanning.  Preserve the same exclusive lock and
        # timeout envelope while polling with millisecond granularity.
        deadline = time.monotonic() + 10.0
        while True:
            try:
                msvcrt.locking(lock_file.fileno(), msvcrt.LK_NBLCK, 1)
                return
            except OSError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.002)
    elif fcntl is not None:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX)


def _nonblocking_file_lock(lock_file) -> bool:
    if msvcrt is not None:
        try:
            msvcrt.locking(lock_file.fileno(), msvcrt.LK_NBLCK, 1)
            return True
        except OSError:
            return False
    if fcntl is not None:
        try:
            fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            return True
        except (BlockingIOError, OSError):
            return False
    return True


def _unlock_file_lock(lock_file) -> None:
    if msvcrt is not None:
        msvcrt.locking(lock_file.fileno(), msvcrt.LK_UNLCK, 1)
    elif fcntl is not None:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_UN)


def _acquire_store_file_lock(lock_file) -> dict[str, float | int | str]:
    if not _env_flag("REPLAN_ID_ALLOCATOR_NONBLOCKING_FILE_LOCK", False):
        _blocking_file_lock(lock_file)
        return {
            "lockMode": "blocking",
            "nonblockingAttempts": 0,
            "nonblockingSleepMs": 0.0,
            "nonblockingFallback": 0,
        }

    budget_ms = _env_float("REPLAN_ID_ALLOCATOR_NONBLOCKING_FILE_LOCK_BUDGET_MS", 20.0, minimum=0.0)
    base_sleep_ms = _env_float("REPLAN_ID_ALLOCATOR_NONBLOCKING_FILE_LOCK_SLEEP_MS", 1.0, minimum=0.0)
    max_sleep_ms = _env_float("REPLAN_ID_ALLOCATOR_NONBLOCKING_FILE_LOCK_MAX_SLEEP_MS", 5.0, minimum=0.0)
    attempts = 0
    slept_ms = 0.0
    started = time.perf_counter()
    while True:
        attempts += 1
        if _nonblocking_file_lock(lock_file):
            return {
                "lockMode": "nonblocking_retry",
                "nonblockingAttempts": attempts,
                "nonblockingSleepMs": round(slept_ms, 3),
                "nonblockingFallback": 0,
            }
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        if elapsed_ms >= budget_ms:
            break
        remaining_ms = max(0.0, budget_ms - elapsed_ms)
        sleep_ms = min(max_sleep_ms, base_sleep_ms * (2 ** min(attempts - 1, 4)), remaining_ms)
        if sleep_ms > 0.0:
            time.sleep(sleep_ms / 1000.0)
            slept_ms += sleep_ms
        else:
            time.sleep(0)

    _blocking_file_lock(lock_file)
    return {
        "lockMode": "nonblocking_retry",
        "nonblockingAttempts": attempts,
        "nonblockingSleepMs": round(slept_ms, 3),
        "nonblockingFallback": 1,
    }


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
    wait_started = time.perf_counter()
    hold_started = None
    acquire_info: dict[str, float | int | str] = {}
    try:
        lock_file.seek(0, os.SEEK_END)
        if lock_file.tell() == 0:
            lock_file.write(b"\0")
            lock_file.flush()
        lock_file.seek(0)
        acquire_info = _acquire_store_file_lock(lock_file)
        acquired = True
        hold_started = time.perf_counter()
        yield
    finally:
        hold_ms = (time.perf_counter() - hold_started) * 1000.0 if hold_started is not None else 0.0
        wait_ms = ((hold_started or time.perf_counter()) - wait_started) * 1000.0
        try:
            if acquired:
                lock_file.seek(0)
                _unlock_file_lock(lock_file)
        finally:
            _FILE_LOCK_LOCAL.depth = 0
            lock_file.close()
            _emit_id_metric(
                "file_lock",
                wait_ms + hold_ms,
                path=str(target),
                lockPath=str(lock_path),
                waitMs=round(wait_ms, 3),
                holdMs=round(hold_ms, 3),
                **acquire_info,
            )


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
    started = time.perf_counter()
    try:
        with _store_file_lock(target):
            if not target.exists() or target.stat().st_size == 0:
                return {}
            with target.open('r', encoding='utf-8') as f:
                data = json.load(f)
                return _normalize_state(data)
    except Exception:
        return {}
    finally:
        _emit_id_metric(
            "read_store_state",
            (time.perf_counter() - started) * 1000.0,
            path=str(target),
        )


def _save(state: dict, path: Path | None = None, *, existing_state: dict | None = None) -> None:
    """
    ID 상태를 안전하게 덮어쓰다.

    existing_state: 호출자가 동일한 _store_file_lock 구간 안에서 이미
    _read_store_state()로 읽어 둔 상태. 락이 read→write 전 구간 유지될 때만
    전달할 것 (그 경우 재읽기 결과와 동일함이 보장되어 중복 json.load를 생략).
    """
    store_path = path or _STORE
    started = time.perf_counter()
    with _store_file_lock(store_path):
        state = _normalize_state(state)
        existing = existing_state if existing_state is not None else _read_store_state(store_path)
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
                _emit_id_metric(
                    "save",
                    (time.perf_counter() - started) * 1000.0,
                    path=str(store_path),
                    keys=sorted(str(key) for key in state.keys()),
                )
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


def _save_current_state_locked(state: dict, path: Path | None = None) -> None:
    """Write the already-merged state while the caller holds _store_file_lock."""
    store_path = path or _STORE
    started = time.perf_counter()
    normalized = _normalize_state(state)
    store_path.parent.mkdir(parents=True, exist_ok=True)
    tmp = store_path.with_name(
        f"{store_path.name}.{os.getpid()}.{threading.get_ident()}.tmp"
    )
    with tmp.open("w", encoding="utf-8") as f:
        json.dump(normalized, f, ensure_ascii=False, separators=(",", ":"))
    for attempt in range(5):
        try:
            tmp.replace(store_path)
            _emit_id_metric(
                "save",
                (time.perf_counter() - started) * 1000.0,
                path=str(store_path),
                keys=sorted(str(key) for key in normalized.keys()),
            )
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

def _waypoint_usage_path() -> Path | None:
    try:
        log_dir = db_paths.get_db_subpath("DSS_Internal")
        return log_dir / "waypoint_usage.json"
    except Exception:
        return None


def _flight_path_signature() -> dict | None:
    try:
        flight_path_dir = db_paths.get_db_subpath("FlightPath")
    except Exception:
        return None
    try:
        directory_exists = flight_path_dir.exists()
        dir_stat = flight_path_dir.stat() if directory_exists else None
    except Exception:
        directory_exists = False
        dir_stat = None

    json_count = 0
    max_mtime_ns = 0
    total_size = 0
    if directory_exists:
        try:
            for path in flight_path_dir.glob("*.json"):
                try:
                    stat = path.stat()
                except Exception:
                    continue
                json_count += 1
                max_mtime_ns = max(max_mtime_ns, int(getattr(stat, "st_mtime_ns", 0) or 0))
                total_size += int(getattr(stat, "st_size", 0) or 0)
        except Exception:
            pass
    return {
        "version": 1,
        "path": str(flight_path_dir),
        "exists": bool(directory_exists),
        "dirMtimeNs": int(getattr(dir_stat, "st_mtime_ns", 0) or 0) if dir_stat is not None else 0,
        "jsonCount": int(json_count),
        "maxFileMtimeNs": int(max_mtime_ns),
        "totalFileSize": int(total_size),
    }


def _cached_flight_path_signature() -> dict | None:
    global _WAYPOINT_SIGNATURE_CACHE
    scoped_cache = _reservation_snapshot_cache()
    if scoped_cache is not None:
        cache_key = ("flight_path_signature", str(_ACTIVE_DB_ROOT or ""))
        cached = scoped_cache.get(cache_key)
        if isinstance(cached, dict):
            return dict(cached)
        signature = _flight_path_signature()
        if isinstance(signature, dict):
            scoped_cache[cache_key] = dict(signature)
        return signature
    if isinstance(_WAYPOINT_SIGNATURE_CACHE, dict):
        return dict(_WAYPOINT_SIGNATURE_CACHE)
    signature = _flight_path_signature()
    if isinstance(signature, dict):
        _WAYPOINT_SIGNATURE_CACHE = dict(signature)
    return signature


def _normalize_waypoint_signature(signature: object) -> dict | None:
    if not isinstance(signature, dict):
        return None
    try:
        return {
            "version": int(signature.get("version") or 0),
            "path": str(signature.get("path") or ""),
            "exists": bool(signature.get("exists")),
            "dirMtimeNs": int(signature.get("dirMtimeNs") or 0),
            "jsonCount": int(signature.get("jsonCount") or 0),
            "maxFileMtimeNs": int(signature.get("maxFileMtimeNs") or 0),
            "totalFileSize": int(signature.get("totalFileSize") or 0),
        }
    except Exception:
        return None


def _waypoint_signature_cache_key(signature: object) -> tuple | None:
    normalized = _normalize_waypoint_signature(signature)
    if normalized is None:
        return None
    return tuple(
        (key, normalized.get(key))
        for key in (
            "version",
            "path",
            "exists",
            "dirMtimeNs",
            "jsonCount",
            "maxFileMtimeNs",
            "totalFileSize",
        )
    )


def _read_waypoint_usage_record() -> dict | None:
    try:
        usage_file = _waypoint_usage_path()
        if usage_file is None or not usage_file.exists():
            return None
        data = json.loads(usage_file.read_text(encoding="utf-8"))
        last_id = int(data.get("last_waypoint_id"))
        if last_id < BASE["waypoint"] - 1:
            return None
        return {
            "last_waypoint_id": last_id,
            "flightPathSignature": _normalize_waypoint_signature(data.get("flightPathSignature")),
        }
    except Exception:
        return None


def _read_last_waypoint_usage() -> int | None:
    record = _read_waypoint_usage_record()
    if not record:
        return None
    try:
        return int(record.get("last_waypoint_id"))
    except Exception:
        return None


def _scan_last_waypoint_id(signature: dict | None = None) -> int | None:
    started = time.perf_counter()
    try:
        flight_path_dir = db_paths.get_db_subpath("FlightPath")
    except Exception:
        return None
    if not flight_path_dir.exists():
        return None
    cache_key = None
    if _env_flag("REPLAN_WAYPOINT_SCAN_CACHE", True):
        scan_signature = signature if isinstance(signature, dict) else _flight_path_signature()
        cache_key = _waypoint_signature_cache_key(scan_signature)
        if cache_key is not None and cache_key in _WAYPOINT_SCAN_RESULT_CACHE:
            cached_value = _WAYPOINT_SCAN_RESULT_CACHE.get(cache_key)
            _emit_id_metric(
                "directory_scan_cache_hit",
                (time.perf_counter() - started) * 1000.0,
                scan="waypoint",
                directory="FlightPath",
                maxValue=cached_value,
            )
            return cached_value
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
    finally:
        _emit_id_metric(
            "directory_scan",
            (time.perf_counter() - started) * 1000.0,
            scan="waypoint",
            directory="FlightPath",
            maxValue=max_waypoint_id,
        )
    if cache_key is not None:
        _WAYPOINT_SCAN_RESULT_CACHE[cache_key] = max_waypoint_id
    return max_waypoint_id


def _refresh_waypoint_counter(*, record_usage: bool = True) -> None:
    if "waypoint" not in _volatile_counters:
        return
    current = int(_volatile_counters.get("waypoint", BASE["waypoint"] - 1))
    usage_record = _read_waypoint_usage_record()
    latest_usage = None
    usage_signature = None
    if usage_record:
        try:
            latest_usage = int(usage_record.get("last_waypoint_id"))
        except Exception:
            latest_usage = None
        usage_signature = _normalize_waypoint_signature(usage_record.get("flightPathSignature"))
    current_signature = _cached_flight_path_signature()
    if (
        _env_flag("REPLAN_WAYPOINT_USAGE_FAST_PATH", True)
        and latest_usage is not None
        and usage_signature is not None
        and current_signature is not None
        and usage_signature == _normalize_waypoint_signature(current_signature)
    ):
        _volatile_counters["waypoint"] = max(current, int(latest_usage))
        _emit_id_metric(
            "waypoint_refresh_fast_path",
            0.0,
            value=int(_volatile_counters["waypoint"]),
            jsonCount=int(current_signature.get("jsonCount") or 0),
        )
        return
    latest_scan = _scan_last_waypoint_id(signature=current_signature)
    candidates = [
        current,
        int(latest_usage) if latest_usage is not None else BASE["waypoint"] - 1,
        int(latest_scan) if latest_scan is not None else BASE["waypoint"] - 1,
    ]
    _volatile_counters["waypoint"] = max(candidates)
    if record_usage and current_signature is not None:
        _record_waypoint_usage(
            int(_volatile_counters["waypoint"]),
            signature=current_signature,
            previous_record=usage_record,
        )


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
    started = time.perf_counter()
    last_value: int | None = None
    try:
        directory = db_paths.get_db_subpath(directory_name)
    except Exception:
        return None
    if not directory.exists():
        return None
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
    finally:
        _emit_id_metric(
            "directory_scan",
            (time.perf_counter() - started) * 1000.0,
            scan="numeric_stem",
            directory=directory_name,
            maxValue=last_value,
        )
    return last_value


def _scan_last_individual_mission_id() -> int | None:
    started = time.perf_counter()
    last_value: int | None = None
    try:
        imp_dir = db_paths.get_db_subpath("IndividualMissionPlan")
    except Exception:
        return None
    if not imp_dir.exists():
        return None
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
    finally:
        _emit_id_metric(
            "directory_scan",
            (time.perf_counter() - started) * 1000.0,
            scan="individualMission",
            directory="IndividualMissionPlan",
            maxValue=last_value,
        )
    return last_value


def _scan_last_path_ids() -> dict[int, int]:
    started = time.perf_counter()
    results: dict[int, int] = {}
    try:
        flight_path_dir = db_paths.get_db_subpath("FlightPath")
    except Exception:
        return {}
    if not flight_path_dir.exists():
        return {}
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
    finally:
        _emit_id_metric(
            "directory_scan",
            (time.perf_counter() - started) * 1000.0,
            scan="pathID",
            directory="FlightPath",
            maxValueByAircraft={int(k): int(v) for k, v in results.items()},
        )
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
    global _ACTIVE_DB_ROOT, _STORE, _state, _WAYPOINT_SIGNATURE_CACHE, _WAYPOINT_SCAN_RESULT_CACHE
    try:
        active_root = str(db_paths.get_active_db_root())
    except Exception:
        return
    if active_root == _ACTIVE_DB_ROOT:
        return
    _ACTIVE_DB_ROOT = active_root
    _STORE = _resolve_store_path()
    _WAYPOINT_SIGNATURE_CACHE = None
    _WAYPOINT_SCAN_RESULT_CACHE = {}
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
    usage_record = _read_waypoint_usage_record()
    usage_value = None
    usage_signature = None
    if usage_record:
        try:
            usage_value = int(usage_record.get("last_waypoint_id"))
        except Exception:
            usage_value = None
        usage_signature = _normalize_waypoint_signature(usage_record.get("flightPathSignature"))
    current_signature = _cached_flight_path_signature()
    if (
        _env_flag("REPLAN_WAYPOINT_USAGE_FAST_PATH", True)
        and usage_value is not None
        and usage_signature is not None
        and current_signature is not None
        and usage_signature == _normalize_waypoint_signature(current_signature)
    ):
        _volatile_counters["waypoint"] = max(
            int(_volatile_counters.get("waypoint", BASE["waypoint"] - 1)),
            int(usage_value),
        )
        _emit_id_metric(
            "waypoint_seed_fast_path",
            0.0,
            value=int(_volatile_counters["waypoint"]),
            jsonCount=int(current_signature.get("jsonCount") or 0),
        )
        return
    scan_value = _scan_last_waypoint_id(signature=current_signature)
    candidates = [
        int(_volatile_counters.get("waypoint", BASE["waypoint"] - 1)),
        int(usage_value) if usage_value is not None else BASE["waypoint"] - 1,
        int(scan_value) if scan_value is not None else BASE["waypoint"] - 1,
    ]
    _volatile_counters["waypoint"] = max(candidates)
    if current_signature is not None:
        _record_waypoint_usage(int(_volatile_counters["waypoint"]), signature=current_signature)

def _next(key: str, inc: int = 1, subkey=None) -> int:
    started = time.perf_counter()
    with _LOCK:
        _sync_active_db_scope()
        if key in VOLATILE_KEYS:
            if subkey is not None:
                raise ValueError(f"volatile key '{key}' does not support subkey allocation")
            with _store_file_lock(_STORE):
                if key == "waypoint":
                    _refresh_waypoint_counter(record_usage=False)
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
            _save(_state, existing_state=disk_state)
        if key == "pathID" and subkey is not None:
            _record_path_usage(subkey, cur)
        _emit_id_metric(
            "next",
            (time.perf_counter() - started) * 1000.0,
            key=key,
            subkey=subkey,
            inc=int(inc),
            value=int(cur),
        )
        return cur


def _reserve_range(key: str, count: int, subkey=None) -> tuple[int, int]:
    started = time.perf_counter()
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
                    _refresh_waypoint_counter(record_usage=False)
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

            _save(_state, existing_state=disk_state)
        if key == "pathID" and subkey is not None:
            _record_path_usage(subkey, end)
        _emit_id_metric(
            "reserve",
            (time.perf_counter() - started) * 1000.0,
            key=key,
            subkey=subkey,
            count=int(count_int),
            start=int(start),
            end=int(end),
        )
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
def reserve_path_id_blocks(count_by_aircraft: dict[int, int] | dict[str, int]):
    started = time.perf_counter()
    normalized_counts: dict[int, int] = {}
    for raw_aid, raw_count in (count_by_aircraft or {}).items():
        try:
            aid = int(raw_aid)
            count = int(raw_count)
        except (TypeError, ValueError):
            continue
        if count <= 0:
            continue
        if aid not in BASE["pathID"]:
            raise ValueError(f"unknown aircraftID for pathID allocation: {aid}")
        normalized_counts[aid] = normalized_counts.get(aid, 0) + count
    if not normalized_counts:
        return {}

    reserved: dict[int, list[int]] = {}
    with _reservation_snapshot_scope():
        with _LOCK:
            _sync_active_db_scope()
            with _store_file_lock(_STORE):
                disk_state = _read_store_state()
                path_map = _state.setdefault("pathID", {})
                disk_map = disk_state.get("pathID", {})
                for aid in sorted(normalized_counts):
                    count = int(normalized_counts[aid])
                    base_value = BASE["pathID"][aid] - 1
                    try:
                        disk_value = int(disk_map.get(aid, base_value))
                    except (TypeError, ValueError):
                        disk_value = base_value
                    try:
                        mem_value = int(path_map.get(aid, base_value))
                    except (TypeError, ValueError):
                        mem_value = base_value
                    start = int(max(mem_value, disk_value) + 1)
                    end = int(start + count - 1)
                    path_map[aid] = end
                    reserved[aid] = list(range(start, end + 1))
                _save(_state, existing_state=disk_state)
    _record_path_usage_many(
        {
            int(aid): int(values[-1])
            for aid, values in reserved.items()
            if values
        }
    )
    _emit_id_metric(
        "reserve_path_id_blocks",
        (time.perf_counter() - started) * 1000.0,
        aircraftIDs=sorted(int(aid) for aid in reserved),
        counts={str(int(aid)): len(values) for aid, values in reserved.items()},
        starts={str(int(aid)): int(values[0]) for aid, values in reserved.items() if values},
        ends={str(int(aid)): int(values[-1]) for aid, values in reserved.items() if values},
        totalCount=sum(len(values) for values in reserved.values()),
    )
    return reserved


def reserve_replan_id_bundle(
    *,
    path_count_by_aircraft: dict[int, int] | dict[str, int] | None = None,
    imp_count: int = 0,
    individual_mission_count: int = 0,
    waypoint_count: int = 0,
):
    started = time.perf_counter()
    normalized_path_counts: dict[int, int] = {}
    for raw_aid, raw_count in (path_count_by_aircraft or {}).items():
        try:
            aid = int(raw_aid)
            count = int(raw_count)
        except (TypeError, ValueError):
            continue
        if count <= 0:
            continue
        if aid not in BASE["pathID"]:
            raise ValueError(f"unknown aircraftID for pathID allocation: {aid}")
        normalized_path_counts[aid] = normalized_path_counts.get(aid, 0) + count
    imp_count_int = max(0, int(imp_count or 0))
    individual_count_int = max(0, int(individual_mission_count or 0))
    waypoint_count_int = max(0, int(waypoint_count or 0))
    if (
        not normalized_path_counts
        and imp_count_int <= 0
        and individual_count_int <= 0
        and waypoint_count_int <= 0
    ):
        return {
            "pathID": {},
            "individualMissionPackage": [],
            "individualMission": [],
            "waypoint": [],
        }

    reserved_paths: dict[int, list[int]] = {}
    reserved_imp_ids: list[int] = []
    reserved_individual_ids: list[int] = []
    reserved_waypoint_ids: list[int] = []

    def _reserve_linear_ids(disk_state: dict, key: str, count: int) -> list[int]:
        if count <= 0:
            return []
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
        end = int(start + count - 1)
        _state[key] = end
        return list(range(start, end + 1))

    with _reservation_snapshot_scope():
        with _LOCK:
            _sync_active_db_scope()
            with _store_file_lock(_STORE):
                disk_state = _read_store_state()
                path_map = _state.setdefault("pathID", {})
                disk_map = disk_state.get("pathID", {})
                for aid in sorted(normalized_path_counts):
                    count = int(normalized_path_counts[aid])
                    base_value = BASE["pathID"][aid] - 1
                    try:
                        disk_value = int(disk_map.get(aid, base_value))
                    except (TypeError, ValueError):
                        disk_value = base_value
                    try:
                        mem_value = int(path_map.get(aid, base_value))
                    except (TypeError, ValueError):
                        mem_value = base_value
                    start = int(max(mem_value, disk_value) + 1)
                    end = int(start + count - 1)
                    path_map[aid] = end
                    reserved_paths[aid] = list(range(start, end + 1))
                reserved_imp_ids = _reserve_linear_ids(
                    disk_state,
                    "individualMissionPackage",
                    imp_count_int,
                )
                reserved_individual_ids = _reserve_linear_ids(
                    disk_state,
                    "individualMission",
                    individual_count_int,
                )
                if waypoint_count_int > 0:
                    _refresh_waypoint_counter(record_usage=False)
                    current_waypoint = int(_volatile_counters.get("waypoint", BASE["waypoint"] - 1))
                    waypoint_start = int(current_waypoint + 1)
                    waypoint_end = int(current_waypoint + waypoint_count_int)
                    _volatile_counters["waypoint"] = waypoint_end
                    reserved_waypoint_ids = list(range(waypoint_start, waypoint_end + 1))
                    _record_waypoint_usage(waypoint_end)
                _save_current_state_locked(_state)

    _record_path_usage_many(
        {
            int(aid): int(values[-1])
            for aid, values in reserved_paths.items()
            if values
        }
    )
    _emit_id_metric(
        "reserve_replan_id_bundle",
        (time.perf_counter() - started) * 1000.0,
        aircraftIDs=sorted(int(aid) for aid in reserved_paths),
        pathCounts={str(int(aid)): len(values) for aid, values in reserved_paths.items()},
        pathStarts={str(int(aid)): int(values[0]) for aid, values in reserved_paths.items() if values},
        pathEnds={str(int(aid)): int(values[-1]) for aid, values in reserved_paths.items() if values},
        totalPathCount=sum(len(values) for values in reserved_paths.values()),
        impCount=len(reserved_imp_ids),
        individualMissionCount=len(reserved_individual_ids),
        waypointCount=len(reserved_waypoint_ids),
        waypointStart=int(reserved_waypoint_ids[0]) if reserved_waypoint_ids else None,
        waypointEnd=int(reserved_waypoint_ids[-1]) if reserved_waypoint_ids else None,
    )
    return {
        "pathID": reserved_paths,
        "individualMissionPackage": reserved_imp_ids,
        "individualMission": reserved_individual_ids,
        "waypoint": reserved_waypoint_ids,
    }


def reserve_waypoint_block(count: int) -> int:
    count_int = max(0, int(count or 0))
    if count_int <= 0:
        raise ValueError("count must be >= 1")
    with _reservation_snapshot_scope():
        last_id = _next("waypoint", inc=count_int)
    return int(last_id - count_int + 1)


def reserve_waypoint_blocks(counts: list[int] | tuple[int, ...]) -> list[tuple[int, int]]:
    started = time.perf_counter()
    normalized_counts: list[int] = []
    for raw_count in counts or []:
        try:
            count = int(raw_count)
        except (TypeError, ValueError):
            continue
        if count > 0:
            normalized_counts.append(count)
    if not normalized_counts:
        return []

    blocks: list[tuple[int, int]] = []
    with _reservation_snapshot_scope():
        with _LOCK:
            _sync_active_db_scope()
            with _store_file_lock(_STORE):
                _refresh_waypoint_counter(record_usage=False)
                current = int(_volatile_counters.get("waypoint", BASE["waypoint"] - 1))
                for count in normalized_counts:
                    start = int(current + 1)
                    end = int(current + count)
                    blocks.append((start, end))
                    current = end
                _volatile_counters["waypoint"] = current
                _record_waypoint_usage(current)
    _emit_id_metric(
        "reserve_waypoint_blocks",
        (time.perf_counter() - started) * 1000.0,
        counts=list(normalized_counts),
        starts=[int(start) for start, _end in blocks],
        ends=[int(end) for _start, end in blocks],
        totalCount=sum(int(count) for count in normalized_counts),
    )
    return blocks


def _record_waypoint_usage(
    value: int,
    *,
    signature: dict | None = None,
    previous_record: dict | None = None,
) -> None:
    global _WAYPOINT_SIGNATURE_CACHE
    started = time.perf_counter()
    target = _waypoint_usage_path()
    if target is None:
        return
    try:
        target.parent.mkdir(parents=True, exist_ok=True)
        previous = previous_record if isinstance(previous_record, dict) else _read_waypoint_usage_record()
        previous_value = int(previous.get("last_waypoint_id")) if previous else BASE["waypoint"] - 1
        last_value = max(int(value), previous_value)
        effective_signature = _normalize_waypoint_signature(signature)
        if effective_signature is None:
            effective_signature = _cached_flight_path_signature()
        if isinstance(effective_signature, dict):
            _WAYPOINT_SIGNATURE_CACHE = dict(effective_signature)
        payload = {
            "last_waypoint_id": int(last_value),
            "updated_at": datetime.now(timezone.utc).isoformat(),
            "flightPathSignature": effective_signature,
        }
        tmp = target.with_suffix(".tmp")
        tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
        tmp.replace(target)
    except Exception:
        return
    finally:
        _emit_id_metric(
            "usage_update",
            (time.perf_counter() - started) * 1000.0,
            usage="waypoint",
            value=int(value),
        )


def mark_waypoint_files_written(max_waypoint_id: int | None = None) -> None:
    """Mark the current FlightPath directory signature as observed for waypoint high-water checks."""
    global _WAYPOINT_SIGNATURE_CACHE
    started = time.perf_counter()
    with _LOCK:
        _sync_active_db_scope()
        current = int(_volatile_counters.get("waypoint", BASE["waypoint"] - 1))
        usage_record = _read_waypoint_usage_record()
        usage_value = None
        if usage_record:
            try:
                usage_value = int(usage_record.get("last_waypoint_id"))
            except Exception:
                usage_value = None
        candidates = [
            current,
            int(usage_value) if usage_value is not None else BASE["waypoint"] - 1,
        ]
        try:
            if max_waypoint_id is not None:
                candidates.append(int(max_waypoint_id))
        except Exception:
            pass
        value = max(candidates)
        _volatile_counters["waypoint"] = value
        signature = _flight_path_signature()
        if isinstance(signature, dict):
            _WAYPOINT_SIGNATURE_CACHE = dict(signature)
        _record_waypoint_usage(value, signature=signature, previous_record=usage_record)
    _emit_id_metric(
        "waypoint_files_written",
        (time.perf_counter() - started) * 1000.0,
        value=int(value),
        jsonCount=int((signature or {}).get("jsonCount") or 0),
    )

def _record_path_usage(aircraft_id: int, value: int) -> None:
    _record_path_usage_many({int(aircraft_id): int(value)})


def _record_path_usage_many(updates: dict[int, int]) -> None:
    started = time.perf_counter()
    normalized: dict[int, int] = {}
    for raw_aid, raw_value in (updates or {}).items():
        try:
            aid = int(raw_aid)
            value = int(raw_value)
        except Exception:
            continue
        normalized[aid] = value
    if not normalized:
        return
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
        for aid, value in sorted(normalized.items()):
            aircraft_map[str(int(aid))] = int(value)
        data["updated_at"] = datetime.now(timezone.utc).isoformat()
        tmp = target.with_suffix(".tmp")
        tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
        tmp.replace(target)
    except Exception:
        return
    finally:
        elapsed_ms = (time.perf_counter() - started) * 1000.0
        for aid, value in sorted(normalized.items()):
            _emit_id_metric(
                "usage_update",
                elapsed_ms,
                usage="pathID",
                aircraftID=int(aid),
                value=int(value),
                batchCount=len(normalized),
            )


_sync_active_db_scope()
