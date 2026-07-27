from __future__ import annotations

import copy
import hashlib
import json
import os
import threading
import time
from collections import OrderedDict
from pathlib import Path
from typing import Any, Mapping

from modules.common import db_paths


KEY_VERSION = "initial-plan-template-v8-control-transfer-lah-via-acp"
DEFAULT_MAX_ENTRIES = 8

_LOCK = threading.RLock()
_CACHE: OrderedDict[str, dict[str, Any]] = OrderedDict()
_STATS = {
    "hits": 0,
    "disk_hits": 0,
    "misses": 0,
    "stores": 0,
    "disk_stores": 0,
    "evictions": 0,
    "disk_evictions": 0,
    "skips": 0,
    "errors": 0,
}


def _enabled() -> bool:
    raw = str(os.environ.get("DSS_INITIAL_PLAN_TEMPLATE_CACHE", "1") or "").strip().lower()
    return raw not in {"0", "false", "off", "no"}


def _max_entries() -> int:
    try:
        value = int(os.environ.get("DSS_INITIAL_PLAN_TEMPLATE_CACHE_MAX", str(DEFAULT_MAX_ENTRIES)))
    except Exception:
        value = DEFAULT_MAX_ENTRIES
    return max(1, int(value))


def _disk_enabled() -> bool:
    raw = str(os.environ.get("DSS_INITIAL_PLAN_TEMPLATE_CACHE_PERSIST", "1") or "").strip().lower()
    return raw not in {"0", "false", "off", "no"}


def _default_cache_dir() -> Path:
    return db_paths.get_db_subpath("DSS_Internal", "initial_plan_template_cache")


def _cache_dir() -> Path:
    override = str(os.environ.get("DSS_INITIAL_PLAN_TEMPLATE_CACHE_DIR", "") or "").strip()
    if override:
        return Path(override)
    return _default_cache_dir()


def _disk_path_for_key(key: str) -> Path:
    safe_key = "".join(ch for ch in str(key) if ch.isalnum() or ch in {"-", "_"})
    if not safe_key:
        safe_key = hashlib.sha256(str(key).encode("utf-8", "ignore")).hexdigest()
    return _cache_dir() / f"{safe_key}.json"


def _load_disk_template(key: str) -> tuple[dict[str, Any] | None, dict[str, Any]]:
    if not _disk_enabled():
        return None, {"diskEnabled": False, "diskHit": False}
    started = time.perf_counter()
    path = _disk_path_for_key(key)
    try:
        with path.open("r", encoding="utf-8") as fh:
            payload = json.load(fh)
        if not isinstance(payload, dict) or payload.get("key") != str(key):
            return None, {
                "diskEnabled": True,
                "diskHit": False,
                "elapsedMs": round((time.perf_counter() - started) * 1000.0, 3),
            }
        template = payload.get("template")
        if not isinstance(template, dict):
            return None, {
                "diskEnabled": True,
                "diskHit": False,
                "elapsedMs": round((time.perf_counter() - started) * 1000.0, 3),
            }
        try:
            os.utime(path, None)
        except Exception:
            pass
        return template, {
            "diskEnabled": True,
            "diskHit": True,
            "source": "disk",
            "path": str(path),
            "elapsedMs": round((time.perf_counter() - started) * 1000.0, 3),
        }
    except FileNotFoundError:
        return None, {
            "diskEnabled": True,
            "diskHit": False,
            "elapsedMs": round((time.perf_counter() - started) * 1000.0, 3),
        }
    except Exception as exc:
        with _LOCK:
            _STATS["errors"] += 1
        return None, {
            "diskEnabled": True,
            "diskHit": False,
            "error": str(exc),
            "elapsedMs": round((time.perf_counter() - started) * 1000.0, 3),
        }


def _prune_disk_cache() -> int:
    if not _disk_enabled():
        return 0
    try:
        cache_dir = _cache_dir()
        entries = [p for p in cache_dir.glob("*.json") if p.is_file()]
        overflow = len(entries) - _max_entries()
        if overflow <= 0:
            return 0
        entries.sort(key=lambda p: p.stat().st_mtime)
        removed = 0
        for path in entries[:overflow]:
            try:
                path.unlink()
                removed += 1
            except Exception:
                pass
        if removed:
            with _LOCK:
                _STATS["disk_evictions"] += int(removed)
        return removed
    except Exception:
        with _LOCK:
            _STATS["errors"] += 1
        return 0


def _store_disk_template(key: str, template: Mapping[str, Any]) -> dict[str, Any]:
    if not _disk_enabled():
        return {"diskEnabled": False, "diskStored": False}
    started = time.perf_counter()
    try:
        cache_dir = _cache_dir()
        cache_dir.mkdir(parents=True, exist_ok=True)
        target = _disk_path_for_key(key)
        tmp = target.with_suffix(f".tmp-{os.getpid()}-{threading.get_ident()}")
        payload = {
            "schemaVersion": 1,
            "key": str(key),
            "createdUnixMs": int(time.time() * 1000),
            "template": template,
        }
        raw = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        with tmp.open("w", encoding="utf-8") as fh:
            fh.write(raw)
        os.replace(tmp, target)
        removed = _prune_disk_cache()
        with _LOCK:
            _STATS["disk_stores"] += 1
        return {
            "diskEnabled": True,
            "diskStored": True,
            "path": str(target),
            "bytes": int(len(raw.encode("utf-8"))),
            "diskEvictions": int(removed),
            "elapsedMs": round((time.perf_counter() - started) * 1000.0, 3),
        }
    except Exception as exc:
        with _LOCK:
            _STATS["errors"] += 1
        try:
            if "tmp" in locals() and Path(tmp).exists():
                Path(tmp).unlink()
        except Exception:
            pass
        return {
            "diskEnabled": True,
            "diskStored": False,
            "error": str(exc),
            "elapsedMs": round((time.perf_counter() - started) * 1000.0, 3),
        }


def _hash_file(path: str | Path) -> dict[str, Any]:
    target = Path(path)
    digest = hashlib.sha256()
    size = 0
    try:
        with target.open("rb") as fh:
            while True:
                chunk = fh.read(1024 * 1024)
                if not chunk:
                    break
                size += len(chunk)
                digest.update(chunk)
    except Exception:
        digest.update(str(target).encode("utf-8", "ignore"))
    return {
        "pathName": target.name,
        "size": int(size),
        "sha256": digest.hexdigest(),
    }


def _fov_db_signature() -> dict[str, Any]:
    """Include the selected FOV DB contents in the initial-plan cache key."""
    try:
        from modules.mission_planning.MissionPlanner.runtime_settings import fov_db_path

        return _hash_file(fov_db_path())
    except Exception as exc:
        return {
            "pathName": "",
            "size": 0,
            "sha256": hashlib.sha256(str(exc).encode("utf-8", "ignore")).hexdigest(),
        }


def _stable_payload(value: Any) -> Any:
    if isinstance(value, Mapping):
        return {str(k): _stable_payload(v) for k, v in sorted(value.items(), key=lambda item: str(item[0]))}
    if isinstance(value, (list, tuple)):
        return [_stable_payload(item) for item in value]
    return value


def make_initial_plan_template_key(
    *,
    cmpk_path: str | Path,
    mrpk_path: str | Path,
    runtime_payload: Any,
    option_code: int,
    trust_input_aircraft: bool,
) -> str:
    payload = {
        "version": KEY_VERSION,
        "cmpk": _hash_file(cmpk_path),
        "mrpk": _hash_file(mrpk_path),
        "fovDb": _fov_db_signature(),
        "runtimePayload": _stable_payload(runtime_payload),
        "optionCode": int(option_code),
        "trustInputAircraft": bool(trust_input_aircraft),
    }
    raw = json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


def get_initial_plan_template(key: str) -> tuple[dict[str, Any] | None, dict[str, Any]]:
    if not _enabled():
        with _LOCK:
            _STATS["skips"] += 1
        return None, {"enabled": False, "hit": False}
    started = time.perf_counter()
    with _LOCK:
        cached = _CACHE.get(str(key))
        if cached is not None:
            _CACHE.move_to_end(str(key))
            _STATS["hits"] += 1
            result = copy.deepcopy(cached)
            return result, {
                "enabled": True,
                "hit": True,
                "source": "memory",
                "elapsedMs": round((time.perf_counter() - started) * 1000.0, 3),
            }

    disk_template, disk_meta = _load_disk_template(str(key))
    if disk_template is not None:
        with _LOCK:
            _CACHE[str(key)] = copy.deepcopy(disk_template)
            _CACHE.move_to_end(str(key))
            _STATS["hits"] += 1
            _STATS["disk_hits"] += 1
            max_entries = _max_entries()
            while len(_CACHE) > max_entries:
                _CACHE.popitem(last=False)
                _STATS["evictions"] += 1
        result = copy.deepcopy(disk_template)
        disk_meta.update(
            {
                "enabled": True,
                "hit": True,
                "source": "disk",
                "elapsedMs": round((time.perf_counter() - started) * 1000.0, 3),
            }
        )
        return result, disk_meta

    with _LOCK:
        _STATS["misses"] += 1
    disk_meta.update(
        {
            "enabled": True,
            "hit": False,
            "elapsedMs": round((time.perf_counter() - started) * 1000.0, 3),
        }
    )
    return None, disk_meta


def put_initial_plan_template(key: str, template: Mapping[str, Any]) -> dict[str, Any]:
    if not _enabled():
        with _LOCK:
            _STATS["skips"] += 1
        return {"enabled": False, "stored": False}
    started = time.perf_counter()
    template_copy = copy.deepcopy(dict(template))
    disk_result = _store_disk_template(str(key), template_copy)
    with _LOCK:
        _CACHE[str(key)] = copy.deepcopy(template_copy)
        _CACHE.move_to_end(str(key))
        _STATS["stores"] += 1
        max_entries = _max_entries()
        while len(_CACHE) > max_entries:
            _CACHE.popitem(last=False)
            _STATS["evictions"] += 1
        entries = len(_CACHE)
    return {
        "enabled": True,
        "stored": True,
        "entries": int(entries),
        "diskStored": bool(disk_result.get("diskStored")),
        "diskElapsedMs": float(disk_result.get("elapsedMs") or 0.0),
        "diskBytes": int(disk_result.get("bytes") or 0),
        "elapsedMs": round((time.perf_counter() - started) * 1000.0, 3),
    }


def snapshot_initial_plan_template_cache_stats() -> dict[str, Any]:
    with _LOCK:
        stats = dict(_STATS)
        stats["entries"] = len(_CACHE)
        stats["enabled"] = _enabled()
        stats["maxEntries"] = _max_entries()
        stats["diskEnabled"] = _disk_enabled()
        try:
            stats["diskEntries"] = len([p for p in _cache_dir().glob("*.json") if p.is_file()])
            stats["diskPath"] = str(_cache_dir())
        except Exception:
            stats["diskEntries"] = 0
    return stats


def clear_initial_plan_template_cache() -> None:
    with _LOCK:
        _CACHE.clear()
        for key in list(_STATS):
            _STATS[key] = 0
    try:
        for path in _cache_dir().glob("*.json"):
            try:
                path.unlink()
            except Exception:
                pass
    except Exception:
        pass
