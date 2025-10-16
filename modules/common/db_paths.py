from __future__ import annotations

import json
import os
import shutil
import threading
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Dict, Optional

PROJECT_ROOT = Path(__file__).resolve().parents[2]
LEGACY_DB_ROOT = PROJECT_ROOT / "database"
SCENARIO_PREFIX = "Scenario_"
AGENCY_CODE_DEFAULT = os.environ.get("KU_AGENCY_CODE", "SBC3")
ENV_DB_ROOT = "KU_MISSION_DB_ROOT"
ENV_SCENARIO_ROOT = "KU_SCENARIO_ROOT"
ENV_SCENARIO_BASE_ROOT = "KU_SCENARIO_BASE_ROOT"
INFO_PATH = PROJECT_ROOT / "current_scenario.json"
_TS_OFFSET_MS = int(os.environ.get("KU_SCENARIO_TS_OFFSET_MS", "0") or 0)
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)

_lock = threading.Lock()
_cache: Dict[str, Any] = {
    "mtime": None,
    "db_root": None,
    "scenario_dir": None,
    "timestamp_ms": None,
    "iso": None,
    "agency": None,
    "source": None,
    "base_root": None,
}
_watcher_started = False


def _str_path(value: Path | str | None) -> Optional[str]:
    if value is None:
        return None
    return str(value)


def _current_info_dict() -> Dict[str, Any]:
    return {
        "timestamp_ms": _cache.get("timestamp_ms"),
        "iso": _cache.get("iso"),
        "scenario_dir": _str_path(_cache.get("scenario_dir")),
        "agency": _cache.get("agency"),
        "db_root": _str_path(_cache.get("db_root")),
        "source": _cache.get("source"),
        "base_root": _cache.get("base_root"),
    }


def _set_env_unlocked(db_root: Optional[Path], scenario_dir: Optional[Path]) -> None:
    if db_root is not None:
        os.environ[ENV_DB_ROOT] = str(db_root)
    elif ENV_DB_ROOT not in os.environ:
        os.environ[ENV_DB_ROOT] = str(LEGACY_DB_ROOT)
    if scenario_dir is not None:
        os.environ[ENV_SCENARIO_ROOT] = str(scenario_dir)
    else:
        os.environ.pop(ENV_SCENARIO_ROOT, None)
    base_root = _cache.get("base_root")
    if base_root:
        os.environ[ENV_SCENARIO_BASE_ROOT] = str(base_root)
    else:
        os.environ.pop(ENV_SCENARIO_BASE_ROOT, None)


def _write_info_unlocked(info: Dict[str, Any]) -> None:
    info = dict(info)
    if "base_root" not in info:
        info["base_root"] = _cache.get("base_root")
    INFO_PATH.parent.mkdir(parents=True, exist_ok=True)
    with INFO_PATH.open("w", encoding="utf-8") as fh:
        json.dump(info, fh, ensure_ascii=False, indent=2, sort_keys=True)


def _refresh_cache_unlocked() -> None:
    try:
        mtime = INFO_PATH.stat().st_mtime
    except FileNotFoundError:
        mtime = None
    if _cache.get("mtime") == mtime:
        return
    if mtime is None:
        _cache.update({
            "mtime": None,
            "db_root": None,
            "scenario_dir": None,
            "timestamp_ms": None,
            "iso": None,
            "agency": None,
            "source": None,
            "base_root": None,
        })
        return
    try:
        with INFO_PATH.open("r", encoding="utf-8") as fh:
            data = json.load(fh)
    except Exception:
        _cache["mtime"] = mtime
        return
    db_root = Path(data["db_root"]) if data.get("db_root") else None
    scenario_dir = Path(data["scenario_dir"]) if data.get("scenario_dir") else None
    _cache.update({
        "mtime": mtime,
        "db_root": db_root,
        "scenario_dir": scenario_dir,
        "timestamp_ms": data.get("timestamp_ms"),
        "iso": data.get("iso"),
        "agency": data.get("agency"),
        "source": data.get("source"),
        "base_root": data.get("base_root"),
    })
    _set_env_unlocked(db_root, scenario_dir)


def ensure_env_watch(interval: float | None = None) -> None:
    global _watcher_started
    if _watcher_started:
        return
    interval = float(interval or os.environ.get("KU_SCENARIO_SYNC_SEC", "1.5"))

    def _loop() -> None:
        while True:
            try:
                with _lock:
                    _refresh_cache_unlocked()
                    db_root = _cache.get("db_root")
                    scenario_dir = _cache.get("scenario_dir")
                    _set_env_unlocked(db_root, scenario_dir)
            except Exception:
                pass
            time.sleep(interval)

    thread = threading.Thread(target=_loop, name="db-path-watch", daemon=True)
    thread.start()
    _watcher_started = True


def ms_to_iso(timestamp_ms: int) -> str:
    try:
        ts_int = int(timestamp_ms)
    except Exception:
        return str(timestamp_ms)
    dt = _EPOCH_2000 + timedelta(milliseconds=ts_int - _TS_OFFSET_MS)
    try:
        local_dt = dt.astimezone()
    except Exception:
        local_dt = dt
    return local_dt.strftime("%Y-%m-%dT%H%M%S")


def bootstrap_db_root() -> Path:
    with _lock:
        _refresh_cache_unlocked()
        cached_path = _cache.get("db_root")
        if cached_path is not None:
            cached_path.mkdir(parents=True, exist_ok=True)
            _set_env_unlocked(cached_path, _cache.get("scenario_dir"))
            return cached_path
        env_path = os.environ.get(ENV_DB_ROOT)
        if env_path:
            path = Path(env_path)
            path.mkdir(parents=True, exist_ok=True)
            _cache.update({"db_root": path})
            _set_env_unlocked(path, None)
            return path
        LEGACY_DB_ROOT.mkdir(parents=True, exist_ok=True)
        _cache.update({"db_root": LEGACY_DB_ROOT})
        _set_env_unlocked(LEGACY_DB_ROOT, None)
        return LEGACY_DB_ROOT


def get_active_db_root() -> Path:
    with _lock:
        _refresh_cache_unlocked()
        path = _cache.get("db_root")
        if path is None:
            return bootstrap_db_root()
        path.mkdir(parents=True, exist_ok=True)
        return path


def get_active_db_root_str() -> str:
    return str(get_active_db_root())


def get_db_subpath(*parts: str) -> Path:
    base = get_active_db_root()
    return base.joinpath(*parts)


def _copy_legacy_into(destination: Path) -> None:
    destination.mkdir(parents=True, exist_ok=True)
    if not LEGACY_DB_ROOT.exists():
        return
    for root, _, _ in os.walk(LEGACY_DB_ROOT):
        rel = Path(root).relative_to(LEGACY_DB_ROOT) if Path(root) != LEGACY_DB_ROOT else Path('.')
        target = destination / rel
        target.mkdir(parents=True, exist_ok=True)


def _ensure_db_scaffold(db_dir: Path) -> None:
    for sub in _DB_SUBDIRS:
        (db_dir / sub).mkdir(parents=True, exist_ok=True)


def activate_scenario(timestamp_ms: int, agency: Optional[str] = None, *, copy_legacy: bool = True) -> Dict[str, Any]:
    iso = ms_to_iso(timestamp_ms)
    agency_code = agency or os.environ.get("KU_AGENCY_CODE") or AGENCY_CODE_DEFAULT
    base_override = _cache.get("base_root")
    base_root = Path(base_override) if base_override else PROJECT_ROOT
    base_root.mkdir(parents=True, exist_ok=True)
    scenario_dir = base_root / f"{SCENARIO_PREFIX}{iso}"
    agency_dir = scenario_dir / agency_code
    db_dir = agency_dir
    with _lock:
        agency_dir.mkdir(parents=True, exist_ok=True)
        copy_from_legacy = copy_legacy and not base_override
        if copy_from_legacy and not _dir_has_files(db_dir):
            _copy_legacy_into(db_dir)
        _ensure_db_scaffold(db_dir)
        info = {
            "timestamp_ms": int(timestamp_ms),
            "iso": iso,
            "scenario_dir": str(scenario_dir),
            "agency": agency_code,
            "db_root": str(db_dir),
            "source": "scenario",
            "base_root": base_override,
        }
        _write_info_unlocked(info)
        _cache.update({
            "mtime": INFO_PATH.stat().st_mtime,
            "db_root": db_dir,
            "scenario_dir": scenario_dir,
            "timestamp_ms": info["timestamp_ms"],
            "iso": iso,
            "agency": agency_code,
            "source": info["source"],
            "base_root": base_override,
        })
        _set_env_unlocked(db_dir, scenario_dir)
    return info


def set_manual_db_root(path: str | Path, *, source: str = "manual") -> Dict[str, Any]:
    dest = Path(path)
    dest.mkdir(parents=True, exist_ok=True)
    info = {
        "timestamp_ms": None,
        "iso": None,
        "scenario_dir": None,
        "agency": None,
        "db_root": str(dest),
        "source": source,
        "base_root": _cache.get("base_root"),
    }
    with _lock:
        _write_info_unlocked(info)
        _cache.update({
            "mtime": INFO_PATH.stat().st_mtime,
            "db_root": dest,
            "scenario_dir": None,
            "timestamp_ms": None,
            "iso": None,
            "agency": None,
            "source": source,
            "base_root": _cache.get("base_root"),
        })
        _set_env_unlocked(dest, None)
    return info


def set_scenario_base_root(path: str | Path | None) -> Dict[str, Any]:
    with _lock:
        base = Path(path).resolve() if path else None
        _cache.update({
            "base_root": str(base) if base else None,
        })
        info = _current_info_dict()
        info["base_root"] = _cache.get("base_root")
        _write_info_unlocked(info)
        try:
            _cache["mtime"] = INFO_PATH.stat().st_mtime
        except FileNotFoundError:
            _cache["mtime"] = None
        _set_env_unlocked(_cache.get("db_root"), _cache.get("scenario_dir"))
        return info


def get_info() -> Dict[str, Any]:
    with _lock:
        _refresh_cache_unlocked()
        info = _current_info_dict()
        return info


def _dir_has_files(path: Path) -> bool:
    try:
        return any(p.is_file() for p in path.rglob('*'))
    except FileNotFoundError:
        return False


def ensure_db_payload(name: str) -> Path:
    db_root = get_active_db_root()
    dest = db_root / name
    dest.mkdir(parents=True, exist_ok=True)
    _ensure_db_scaffold(db_root)
    if _dir_has_files(dest):
        return dest
    src = LEGACY_DB_ROOT / name
    base_override = _cache.get("base_root")
    if base_override:
        try:
            dest.resolve().relative_to(Path(base_override).resolve())
            src = None
        except ValueError:
            pass
    if src and src.exists():
        for root, _, filenames in os.walk(src):
            rel = Path(root).relative_to(src) if Path(root) != src else Path('.')
            target_dir = dest / rel
            target_dir.mkdir(parents=True, exist_ok=True)
            for fname in filenames:
                shutil.copy2(Path(root) / fname, target_dir / fname)
    return dest

_DB_SUBDIRS = (
    "cache",
    "FlightPath",
    "IndividualMissionPlan",
    "InputMissionPlan",
    "Logs",
    "MissionPlan",
    "MissionPlanOptionInfo",
    "MissionReferenceInfo",
    "mission_output",
)
