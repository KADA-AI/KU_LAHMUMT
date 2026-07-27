from __future__ import annotations

import copy
import json
import time
from contextlib import contextmanager
from contextvars import ContextVar
from pathlib import Path
from threading import RLock
from typing import Any, Callable, Dict, Iterator, Optional

from modules.common import db_paths

try:
    from modules.common import replan_perf
except Exception:
    import sys as _sys

    _COMMON_DIR = next(
        (
            parent / "common"
            for parent in Path(__file__).resolve().parents
            if (parent / "common" / "replan_perf.py").exists()
        ),
        None,
    )
    if _COMMON_DIR is not None and str(_COMMON_DIR) not in _sys.path:
        _sys.path.insert(0, str(_COMMON_DIR))
    import replan_perf  # type: ignore


_ACTIVE_CACHE: ContextVar["SourceArtifactCache | None"] = ContextVar(
    "mission_planning_source_artifact_cache",
    default=None,
)
_FALLBACK_CACHE_LOCK = RLock()
_FALLBACK_CACHE_MAX = 128
_FALLBACK_JSON_BY_PATH: Dict[tuple[str, int, int], Any] = {}
_FALLBACK_JSON_ORDER: list[tuple[str, int, int]] = []


def _cache_path(path: str | Path) -> Path:
    raw = Path(path)
    if raw.is_absolute():
        return raw
    return raw.resolve()


def _fallback_cache_key(path: Path) -> tuple[str, int, int] | None:
    try:
        resolved = str(_cache_path(path))
        stat = path.stat()
    except Exception:
        return None
    return resolved, int(stat.st_mtime_ns), int(stat.st_size)


def _copy_payload(payload: Any) -> Any:
    copy_start = time.perf_counter() if replan_perf.is_enabled() else None
    result = copy.deepcopy(payload)
    copy_ms = 0.0
    if copy_start is not None:
        copy_ms = (time.perf_counter() - copy_start) * 1000.0
    replan_perf.add("mission_planning.source_artifact.deepcopy", elapsed_ms=copy_ms)
    return result


class SourceArtifactCache:
    """Per-replan JSON artifact cache.

    The cache is intentionally scoped to a single planning/replanning run. It
    avoids reparsing stable source artifacts while returning copies by default so
    callers can keep mutating payloads without sharing state by accident.
    """

    def __init__(self) -> None:
        self._lock = RLock()
        self._json_by_path: Dict[Path, Any] = {}
        self._path_id_by_payload_path: Dict[Path, int] = {}
        self._hits = 0
        self._misses = 0
        self._loads_by_kind: Dict[str, int] = {}

    def read_json(self, path: str | Path, *, copy_result: bool = True, kind: str = "json") -> Any:
        perf_start = replan_perf.start_timer()
        copy_ms = 0.0
        load_ms = 0.0
        resolved = _cache_path(path)
        cache_hit = False
        payload = None
        with self._lock:
            if resolved in self._json_by_path:
                self._hits += 1
                payload = self._json_by_path[resolved]
                cache_hit = True
            else:
                payload = None
        if cache_hit:
            if copy_result:
                copy_start = time.perf_counter() if replan_perf.is_enabled() else None
                result = copy.deepcopy(payload)
                if copy_start is not None:
                    copy_ms = (time.perf_counter() - copy_start) * 1000.0
            else:
                result = payload
            replan_perf.add_elapsed(
                "mission_planning.source_artifact.read_json",
                perf_start,
                hit=1,
                copy_result=copy_result,
                copy_ms=copy_ms,
            )
            replan_perf.add_elapsed(
                f"mission_planning.source_artifact.read_json.{kind}",
                perf_start,
                hit=1,
                copy_result=copy_result,
                copy_ms=copy_ms,
            )
            return result

        load_start = time.perf_counter() if replan_perf.is_enabled() else None
        payload = json.loads(resolved.read_text(encoding="utf-8"))
        if load_start is not None:
            load_ms = (time.perf_counter() - load_start) * 1000.0
        with self._lock:
            cached = self._json_by_path.setdefault(resolved, payload)
            self._misses += 1
            self._loads_by_kind[kind] = self._loads_by_kind.get(kind, 0) + 1
            if kind == "FlightPath" and isinstance(cached, dict):
                try:
                    path_id = int(cached.get("pathID"))
                    self._path_id_by_payload_path[resolved] = path_id
                except Exception:
                    pass
        if copy_result:
            copy_start = time.perf_counter() if replan_perf.is_enabled() else None
            result = copy.deepcopy(cached)
            if copy_start is not None:
                copy_ms = (time.perf_counter() - copy_start) * 1000.0
        else:
            result = cached
        replan_perf.add_elapsed(
            "mission_planning.source_artifact.read_json",
            perf_start,
            miss=1,
            copy_result=copy_result,
            load_ms=load_ms,
            copy_ms=copy_ms,
        )
        replan_perf.add_elapsed(
            f"mission_planning.source_artifact.read_json.{kind}",
            perf_start,
            miss=1,
            copy_result=copy_result,
            load_ms=load_ms,
            copy_ms=copy_ms,
        )
        return result

    def load_db_payload(
        self,
        directory: str,
        payload_id: int,
        *,
        copy_result: bool = True,
    ) -> Any:
        path = db_paths.get_db_subpath(directory, f"{int(payload_id)}.json")
        return self.read_json(path, copy_result=copy_result, kind=directory)

    def load_mission_plan(self, mission_plan_id: int, *, copy_result: bool = True) -> Dict[str, Any]:
        return self.load_db_payload("MissionPlan", int(mission_plan_id), copy_result=copy_result)

    def load_input_mission_plan(self, package_id: int, *, copy_result: bool = True) -> Dict[str, Any]:
        return self.load_db_payload("InputMissionPlan", int(package_id), copy_result=copy_result)

    def load_individual_mission_plan(self, package_id: int, *, copy_result: bool = True) -> Dict[str, Any]:
        return self.load_db_payload("IndividualMissionPlan", int(package_id), copy_result=copy_result)

    def load_flight_path(self, path_id: int, *, copy_result: bool = True) -> Dict[str, Any]:
        return self.load_db_payload("FlightPath", int(path_id), copy_result=copy_result)

    def preload_mission_plan_tree(self, mission_plan_id: int) -> Dict[str, int]:
        """Preload source MissionPlan -> IMP -> FlightPath artifacts.

        Missing child artifacts are ignored because callers may already have
        fallback behavior for partial legacy data.
        """

        loaded = {"MissionPlan": 0, "IndividualMissionPlan": 0, "FlightPath": 0}
        plan = self.load_mission_plan(int(mission_plan_id), copy_result=False)
        loaded["MissionPlan"] += 1
        for aircraft in plan.get("aircraftList") or []:
            if not isinstance(aircraft, dict):
                continue
            package_id = (
                aircraft.get("individualMissionPackageID")
                or aircraft.get("individualMissionPlanPackageID")
                or aircraft.get("individualMissionPackageId")
            )
            try:
                package_id_int = int(package_id)
            except Exception:
                continue
            try:
                imp = self.load_individual_mission_plan(package_id_int, copy_result=False)
                loaded["IndividualMissionPlan"] += 1
            except Exception:
                continue
            for mission in imp.get("individualMissionList") or []:
                if not isinstance(mission, dict):
                    continue
                try:
                    path_id = int(mission.get("pathID"))
                except Exception:
                    continue
                if path_id <= 0:
                    continue
                try:
                    self.load_flight_path(path_id, copy_result=False)
                    loaded["FlightPath"] += 1
                except Exception:
                    continue
        return loaded

    def path_waypoints(self, path_id: int) -> list[dict]:
        payload = self.load_flight_path(int(path_id), copy_result=False)
        if not isinstance(payload, dict):
            return []
        for key in ("waypointList", "uavWaypointList", "lahWaypointList"):
            rows = payload.get(key)
            if isinstance(rows, list):
                return _copy_payload(rows)
        return []

    def stats(self) -> Dict[str, Any]:
        with self._lock:
            return {
                "entries": len(self._json_by_path),
                "hits": int(self._hits),
                "misses": int(self._misses),
                "loadsByKind": dict(sorted(self._loads_by_kind.items())),
            }


@contextmanager
def use_source_artifact_cache(cache: SourceArtifactCache) -> Iterator[SourceArtifactCache]:
    token = _ACTIVE_CACHE.set(cache)
    try:
        yield cache
    finally:
        _ACTIVE_CACHE.reset(token)


def get_active_source_artifact_cache() -> Optional[SourceArtifactCache]:
    return _ACTIVE_CACHE.get()


def call_with_source_artifact_cache(
    cache: SourceArtifactCache,
    func: Callable[..., Any],
    *args: Any,
    **kwargs: Any,
) -> Any:
    with use_source_artifact_cache(cache):
        return func(*args, **kwargs)


def read_json_cached(path: str | Path, *, copy_result: bool = True, kind: str = "json") -> Any:
    cache = get_active_source_artifact_cache()
    if cache is None:
        perf_start = replan_perf.start_timer()
        resolved = Path(path)
        if copy_result:
            cache_key = _fallback_cache_key(resolved)
            if cache_key is not None:
                with _FALLBACK_CACHE_LOCK:
                    cached = _FALLBACK_JSON_BY_PATH.get(cache_key)
                    if cached is not None:
                        result = _copy_payload(cached)
                        replan_perf.add_elapsed(
                            "mission_planning.source_artifact.read_json.no_active_cache",
                            perf_start,
                            hit=1,
                            copy_result=1,
                        )
                        return result
            payload = json.loads(resolved.read_text(encoding="utf-8"))
            if cache_key is not None:
                with _FALLBACK_CACHE_LOCK:
                    if cache_key not in _FALLBACK_JSON_BY_PATH:
                        _FALLBACK_JSON_BY_PATH[cache_key] = payload
                        _FALLBACK_JSON_ORDER.append(cache_key)
                        while len(_FALLBACK_JSON_ORDER) > _FALLBACK_CACHE_MAX:
                            old_key = _FALLBACK_JSON_ORDER.pop(0)
                            _FALLBACK_JSON_BY_PATH.pop(old_key, None)
            result = _copy_payload(payload)
            replan_perf.add_elapsed(
                "mission_planning.source_artifact.read_json.no_active_cache",
                perf_start,
                miss=1,
                copy_result=1,
            )
            return result
        payload = json.loads(resolved.read_text(encoding="utf-8"))
        replan_perf.add_elapsed(
            "mission_planning.source_artifact.read_json.no_active_cache",
            perf_start,
            loaded=1,
            copy_result=0,
        )
        return payload
    return cache.read_json(path, copy_result=copy_result, kind=kind)
