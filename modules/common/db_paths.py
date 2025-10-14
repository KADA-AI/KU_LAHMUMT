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
AGENCY_CODE_DEFAULT = os.environ.get("KU_AGENCY_CODE", "SBC2")
ENV_DB_ROOT = "KU_MISSION_DB_ROOT"
ENV_SCENARIO_ROOT = "KU_SCENARIO_ROOT"
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
}
_watcher_started = False


def _set_env_unlocked(db_root: Optional[Path], scenario_dir: Optional[Path]) -> None:
    if db_root is not None:
        os.environ[ENV_DB_ROOT] = str(db_root)
    elif ENV_DB_ROOT not in os.environ:
        os.environ[ENV_DB_ROOT] = str(LEGACY_DB_ROOT)
    if scenario_dir is not None:
        os.environ[ENV_SCENARIO_ROOT] = str(scenario_dir)
    else:
        os.environ.pop(ENV_SCENARIO_ROOT, None)


def _write_info_unlocked(info: Dict[str, Any]) -> None:
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
    if not LEGACY_DB_ROOT.exists():
        destination.mkdir(parents=True, exist_ok=True)
        return
    shutil.copytree(LEGACY_DB_ROOT, destination, dirs_exist_ok=True)


def activate_scenario(timestamp_ms: int, agency: Optional[str] = None, *, copy_legacy: bool = True) -> Dict[str, Any]:
    iso = ms_to_iso(timestamp_ms)
    agency_code = agency or os.environ.get("KU_AGENCY_CODE") or AGENCY_CODE_DEFAULT
    scenario_dir = PROJECT_ROOT / f"{SCENARIO_PREFIX}{iso}"
    agency_dir = scenario_dir / agency_code
    db_dir = agency_dir / "database"
    with _lock:
        if copy_legacy and not db_dir.exists():
            agency_dir.mkdir(parents=True, exist_ok=True)
            _copy_legacy_into(db_dir)
        else:
            db_dir.mkdir(parents=True, exist_ok=True)
        info = {
            "timestamp_ms": int(timestamp_ms),
            "iso": iso,
            "scenario_dir": str(scenario_dir),
            "agency": agency_code,
            "db_root": str(db_dir),
            "source": "scenario",
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
        })
        _set_env_unlocked(dest, None)
    return info


def get_info() -> Dict[str, Any]:
    with _lock:
        _refresh_cache_unlocked()
        return {
            "timestamp_ms": _cache.get("timestamp_ms"),
            "iso": _cache.get("iso"),
            "scenario_dir": str(_cache.get("scenario_dir")) if _cache.get("scenario_dir") else None,
            "agency": _cache.get("agency"),
            "db_root": str(_cache.get("db_root")) if _cache.get("db_root") else None,
            "source": _cache.get("source"),
        }

