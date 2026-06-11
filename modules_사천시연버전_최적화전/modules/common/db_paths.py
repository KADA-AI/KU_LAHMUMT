from __future__ import annotations

import json
import os
import shutil
import threading
import time
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Dict, Optional

from modules.common.settings_paths import scenario_info_path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SCENARIO_DIRNAME = "Logs"
LEGACY_DB_ROOT = PROJECT_ROOT / "temp" / "database"
DEFAULT_SCENARIO_BASE = PROJECT_ROOT / DEFAULT_SCENARIO_DIRNAME
SCENARIO_PREFIX = "Scenario_"
AGENCY_CODE_DEFAULT = os.environ.get("KU_AGENCY_CODE", "SBC3")
ENV_DB_ROOT = "KU_MISSION_DB_ROOT"
ENV_SCENARIO_ROOT = "KU_SCENARIO_ROOT"
ENV_SCENARIO_BASE_ROOT = "KU_SCENARIO_BASE_ROOT"
INFO_PATH = scenario_info_path()
_TS_OFFSET_MS = int(os.environ.get("KU_SCENARIO_TS_OFFSET_MS", "0") or 0)
_EPOCH_2000 = datetime(2000, 1, 1, tzinfo=timezone.utc)

_lock = threading.Lock()
def _default_base_root() -> Path:
    try:
        DEFAULT_SCENARIO_BASE.mkdir(parents=True, exist_ok=True)
    except Exception:
        pass
    return DEFAULT_SCENARIO_BASE


def _info_mtime() -> float | None:
    try:
        return INFO_PATH.stat().st_mtime
    except FileNotFoundError:
        return None


def _mkdir_if_possible(path: Path) -> bool:
    try:
        path.mkdir(parents=True, exist_ok=True)
    except OSError:
        return False
    return True


def _ensure_base_root_unlocked(persist: bool = False) -> str:
    base = _cache.get("base_root")
    if base:
        return str(base)
    base_path = _default_base_root()
    base_str = str(base_path)
    _cache["base_root"] = base_str
    if persist:
        info = _current_info_dict()
        info["base_root"] = base_str
        _write_info_unlocked(info)
    return base_str


_cache: Dict[str, Any] = {
    "mtime": None,
    "db_root": None,
    "scenario_dir": None,
    "timestamp_ms": None,
    "iso": None,
    "agency": None,
    "source": None,
    "base_root": str(_default_base_root()),
}
_watcher_started = False


def _str_path(value: Path | str | None) -> Optional[str]:
    if value is None:
        return None
    return str(value)


def _same_path(left: Path, right: Path) -> bool:
    try:
        return left.resolve() == right.resolve()
    except Exception:
        return left == right


def _path_from_raw(value: Any) -> Path | None:
    if value is None:
        return None
    try:
        text = str(value).strip()
    except Exception:
        return None
    if not text:
        return None
    try:
        return Path(text)
    except Exception:
        return None


def _scenario_db_root_from_parts(
    *,
    scenario_dir: Path | None = None,
    timestamp_ms: Any = None,
    iso: Any = None,
    agency: Any = None,
    base_root: Any = None,
) -> Path | None:
    agency_code = str(agency or os.environ.get("KU_AGENCY_CODE") or AGENCY_CODE_DEFAULT)
    if scenario_dir is not None:
        return scenario_dir / agency_code
    iso_value = str(iso).strip() if iso is not None else ""
    if not iso_value and timestamp_ms is not None:
        try:
            iso_value = ms_to_iso(int(timestamp_ms))
        except Exception:
            iso_value = ""
    if not iso_value:
        return None
    base_path = _path_from_raw(base_root) or _path_from_raw(_cache.get("base_root")) or _default_base_root()
    return base_path / f"{SCENARIO_PREFIX}{iso_value}" / agency_code


def _coerce_db_root_unlocked(
    path: Path,
    *,
    scenario_dir: Path | None = None,
    timestamp_ms: Any = None,
    iso: Any = None,
    agency: Any = None,
    base_root: Any = None,
    strict: bool = False,
) -> Path | None:
    base_path = _path_from_raw(base_root) or _path_from_raw(_cache.get("base_root")) or _default_base_root()
    unsafe_reason = ""
    if _same_path(path, PROJECT_ROOT):
        unsafe_reason = "project root"
    elif _same_path(path, DEFAULT_SCENARIO_BASE):
        unsafe_reason = "default scenario base"
    elif base_path is not None and _same_path(path, base_path):
        unsafe_reason = "scenario base"
    elif path.name.startswith(SCENARIO_PREFIX):
        repaired = path / str(agency or os.environ.get("KU_AGENCY_CODE") or AGENCY_CODE_DEFAULT)
        return repaired
    if not unsafe_reason:
        return path
    repaired = _scenario_db_root_from_parts(
        scenario_dir=scenario_dir,
        timestamp_ms=timestamp_ms,
        iso=iso,
        agency=agency,
        base_root=base_path,
    )
    if repaired is not None and not _same_path(repaired, path):
        return repaired
    if strict:
        raise ValueError(
            f"Unsafe DB root points at {unsafe_reason}: {path}. "
            f"Use a scenario agency directory such as Logs/Scenario_.../{AGENCY_CODE_DEFAULT}."
        )
    return None


def _read_info_snapshot() -> Dict[str, Any] | None:
    try:
        with INFO_PATH.open("r", encoding="utf-8") as fh:
            data = json.load(fh)
    except Exception:
        return None
    return data if isinstance(data, dict) else None


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
    base_value = info.get("base_root") or _cache.get("base_root") or str(_default_base_root())
    base_value = str(base_value)
    info["base_root"] = base_value
    _cache["base_root"] = base_value
    INFO_PATH.parent.mkdir(parents=True, exist_ok=True)
    with INFO_PATH.open("w", encoding="utf-8") as fh:
        json.dump(info, fh, ensure_ascii=False, indent=2, sort_keys=True)


def _set_cached_db_root_unlocked(
    db_root: Path,
    *,
    source: Optional[str],
    scenario_dir: Optional[Path] = None,
    timestamp_ms: Optional[int] = None,
    iso: Optional[str] = None,
    agency: Optional[str] = None,
    base_root: Path | str | None = None,
    persist: bool = False,
) -> Path:
    base_value = base_root or _cache.get("base_root") or _default_base_root()
    base_str = str(base_value)
    _cache.update({
        "db_root": db_root,
        "scenario_dir": scenario_dir,
        "timestamp_ms": timestamp_ms,
        "iso": iso,
        "agency": agency,
        "source": source,
        "base_root": base_str,
    })
    if persist:
        _write_info_unlocked(_current_info_dict())
    _cache["mtime"] = _info_mtime()
    _set_env_unlocked(db_root, scenario_dir)
    return db_root


def _reset_to_local_defaults_unlocked(source: str) -> Path:
    base_root = _default_base_root()
    if not _mkdir_if_possible(base_root):
        raise PermissionError(f"Unable to prepare local scenario root: {base_root}")
    if not _mkdir_if_possible(LEGACY_DB_ROOT):
        raise PermissionError(f"Unable to prepare local DB root: {LEGACY_DB_ROOT}")
    return _set_cached_db_root_unlocked(
        LEGACY_DB_ROOT,
        source=source,
        base_root=base_root,
        persist=True,
    )


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
            "base_root": _cache.get("base_root"),
        })
        _ensure_base_root_unlocked()
        return
    try:
        with INFO_PATH.open("r", encoding="utf-8") as fh:
            data = json.load(fh)
    except Exception:
        _cache["mtime"] = mtime
        return
    scenario_dir = Path(data["scenario_dir"]) if data.get("scenario_dir") else None
    base_root_str = data.get("base_root") or None
    raw_db_root = Path(data["db_root"]) if data.get("db_root") else None
    db_root = None
    persist_needed = not bool(base_root_str)
    if raw_db_root is not None:
        db_root = _coerce_db_root_unlocked(
            raw_db_root,
            scenario_dir=scenario_dir,
            timestamp_ms=data.get("timestamp_ms"),
            iso=data.get("iso"),
            agency=data.get("agency"),
            base_root=base_root_str,
            strict=False,
        )
        if db_root is not None and not _same_path(raw_db_root, db_root):
            data["db_root"] = str(db_root)
            persist_needed = True
    _cache.update({
        "mtime": mtime,
        "db_root": db_root,
        "scenario_dir": scenario_dir,
        "timestamp_ms": data.get("timestamp_ms"),
        "iso": data.get("iso"),
        "agency": data.get("agency"),
        "source": data.get("source"),
        "base_root": str(base_root_str) if base_root_str else None,
    })
    _ensure_base_root_unlocked(persist=persist_needed)
    if persist_needed and db_root is not None:
        _write_info_unlocked(_current_info_dict())
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
            if _mkdir_if_possible(cached_path):
                _set_env_unlocked(cached_path, _cache.get("scenario_dir"))
                return cached_path
            return _reset_to_local_defaults_unlocked("bootstrap-fallback")
        env_path = os.environ.get(ENV_DB_ROOT)
        if env_path:
            path = _coerce_db_root_unlocked(Path(env_path), strict=False)
            if path is None:
                return _reset_to_local_defaults_unlocked("env-unsafe-fallback")
            if _mkdir_if_possible(path):
                return _set_cached_db_root_unlocked(path, source="env")
            return _reset_to_local_defaults_unlocked("env-fallback")
        return _reset_to_local_defaults_unlocked("bootstrap-default")


def get_active_db_root() -> Path:
    with _lock:
        _refresh_cache_unlocked()
        path = _cache.get("db_root")
        if path is None:
            return _reset_to_local_defaults_unlocked("active-db-default")
        safe_path = _coerce_db_root_unlocked(
            Path(path),
            scenario_dir=_cache.get("scenario_dir"),
            timestamp_ms=_cache.get("timestamp_ms"),
            iso=_cache.get("iso"),
            agency=_cache.get("agency"),
            base_root=_cache.get("base_root"),
            strict=False,
        )
        if safe_path is None:
            return _reset_to_local_defaults_unlocked("active-db-unsafe-fallback")
        if not _same_path(Path(path), safe_path):
            path = safe_path
            _cache["db_root"] = path
            _write_info_unlocked(_current_info_dict())
            _set_env_unlocked(path, _cache.get("scenario_dir"))
        if _mkdir_if_possible(path):
            return path
        return _reset_to_local_defaults_unlocked("active-db-fallback")


def get_active_db_root_str() -> str:
    return str(get_active_db_root())


def peek_active_db_root(*, existing_only: bool = False, allow_legacy: bool = False) -> Optional[Path]:
    candidates: list[Path] = []

    info = _read_info_snapshot()
    if info and info.get("db_root"):
        try:
            candidates.append(Path(str(info.get("db_root"))))
        except Exception:
            pass

    env_db_root = os.environ.get(ENV_DB_ROOT)
    env_scenario_root = os.environ.get(ENV_SCENARIO_ROOT)
    if env_db_root and env_scenario_root:
        try:
            candidates.append(Path(env_db_root))
        except Exception:
            pass

    with _lock:
        cached_path = _cache.get("db_root")
        cached_source = str(_cache.get("source") or "")
    if cached_path is not None and cached_source and not cached_source.endswith("fallback"):
        try:
            candidates.append(Path(str(cached_path)))
        except Exception:
            pass

    for path in candidates:
        if not allow_legacy and _same_path(path, LEGACY_DB_ROOT):
            continue
        if existing_only and not path.exists():
            continue
        return path
    return None


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


def point_to_scenario(timestamp_ms: int, agency: Optional[str] = None) -> Dict[str, Any]:
    """폴더를 생성하지 않고 타임스탬프 기반 경로를 활성화만 한다.
    실제 폴더 생성은 다른 SW에 맡기고, DSS_KU는 경로만 지정한다."""
    iso = ms_to_iso(timestamp_ms)
    agency_code = agency or os.environ.get("KU_AGENCY_CODE") or AGENCY_CODE_DEFAULT
    base_root = Path(str(_cache.get("base_root") or _ensure_base_root_unlocked()))
    scenario_dir = base_root / f"{SCENARIO_PREFIX}{iso}"
    db_dir = scenario_dir / agency_code
    info = {
        "timestamp_ms": int(timestamp_ms),
        "iso": iso,
        "scenario_dir": str(scenario_dir),
        "agency": agency_code,
        "db_root": str(db_dir),
        "source": "scenario",
        "base_root": str(base_root),
    }
    with _lock:
        _write_info_unlocked(info)
        _cache.update({
            "mtime": _info_mtime(),
            "db_root": db_dir,
            "scenario_dir": scenario_dir,
            "timestamp_ms": info["timestamp_ms"],
            "iso": iso,
            "agency": agency_code,
            "source": info["source"],
            "base_root": str(base_root),
        })
        _set_env_unlocked(db_dir, scenario_dir)
    return info


def activate_scenario(timestamp_ms: int, agency: Optional[str] = None, *, copy_legacy: bool = True) -> Dict[str, Any]:
    iso = ms_to_iso(timestamp_ms)
    agency_code = agency or os.environ.get("KU_AGENCY_CODE") or AGENCY_CODE_DEFAULT
    base_override = _cache.get("base_root") or _ensure_base_root_unlocked()
    default_base = _default_base_root()
    base_root = Path(str(base_override))
    try:
        is_custom_base = base_root.resolve() != default_base.resolve()
    except Exception:
        is_custom_base = base_root != default_base
    if not _mkdir_if_possible(base_root):
        base_root = default_base
        is_custom_base = False
    if not _mkdir_if_possible(base_root):
        raise PermissionError(f"Unable to prepare scenario root: {base_root}")
    scenario_dir = base_root / f"{SCENARIO_PREFIX}{iso}"
    agency_dir = scenario_dir / agency_code
    db_dir = agency_dir
    with _lock:
        if not _mkdir_if_possible(agency_dir):
            if base_root != default_base:
                base_root = default_base
                scenario_dir = base_root / f"{SCENARIO_PREFIX}{iso}"
                agency_dir = scenario_dir / agency_code
                db_dir = agency_dir
                is_custom_base = False
            if not _mkdir_if_possible(agency_dir):
                raise PermissionError(f"Unable to prepare scenario DB root: {agency_dir}")
        copy_from_legacy = copy_legacy and not is_custom_base
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
            "base_root": str(base_root),
        }
        _write_info_unlocked(info)
        _cache.update({
            "mtime": _info_mtime(),
            "db_root": db_dir,
            "scenario_dir": scenario_dir,
            "timestamp_ms": info["timestamp_ms"],
            "iso": iso,
            "agency": agency_code,
            "source": info["source"],
            "base_root": str(base_root),
        })
        _set_env_unlocked(db_dir, scenario_dir)
    return info


# Backward compatibility: legacy callers must create/scaffold the scenario DB,
# not just repoint current_scenario.json.
point_to_scenario = activate_scenario


def set_manual_db_root(path: str | Path, *, source: str = "manual") -> Dict[str, Any]:
    dest = _coerce_db_root_unlocked(Path(path), strict=True)
    if dest is None:
        raise ValueError(f"Invalid DB root: {path}")
    dest.mkdir(parents=True, exist_ok=True)
    _ensure_db_scaffold(dest)
    base_root_str = _cache.get("base_root") or _ensure_base_root_unlocked()
    info = {
        "timestamp_ms": None,
        "iso": None,
        "scenario_dir": None,
        "agency": None,
        "db_root": str(dest),
        "source": source,
        "base_root": str(base_root_str),
    }
    with _lock:
        _write_info_unlocked(info)
        _cache.update({
            "mtime": _info_mtime(),
            "db_root": dest,
            "scenario_dir": None,
            "timestamp_ms": None,
            "iso": None,
            "agency": None,
            "source": source,
            "base_root": str(base_root_str),
        })
        _set_env_unlocked(dest, None)
    return info


def set_scenario_base_root(path: str | Path | None) -> Dict[str, Any]:
    with _lock:
        base = Path(path).resolve() if path else _default_base_root()
        if _same_path(base, PROJECT_ROOT):
            base = _default_base_root()
        base_str = str(base)
        _cache.update({
            "base_root": base_str,
        })
        info = _current_info_dict()
        info["base_root"] = base_str
        _write_info_unlocked(info)
        _cache["mtime"] = _info_mtime()
        _set_env_unlocked(_cache.get("db_root"), _cache.get("scenario_dir"))
        return info


def get_info() -> Dict[str, Any]:
    with _lock:
        _refresh_cache_unlocked()
        _ensure_base_root_unlocked()
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
    "DSS_Internal",
    "FlightPath",
    "IndividualMissionPlan",
    "InputMissionPlan",
    "VehicleStatus",
    "MissionPlan",
    "MissionPlanOptionInfo",
    "MissionReferenceInfo",
    "mission_output",
)
