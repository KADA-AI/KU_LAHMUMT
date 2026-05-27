from __future__ import annotations

import copy
import json
from contextlib import contextmanager
from contextvars import ContextVar
from pathlib import Path
from threading import RLock
from typing import Any, Callable, Dict, Iterator, Optional

from modules.common import db_paths


_ACTIVE_CACHE: ContextVar["SourceArtifactCache | None"] = ContextVar(
    "mission_planning_source_artifact_cache",
    default=None,
)


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
        resolved = Path(path).resolve()
        with self._lock:
            if resolved in self._json_by_path:
                self._hits += 1
                payload = self._json_by_path[resolved]
                return copy.deepcopy(payload) if copy_result else payload

        payload = json.loads(resolved.read_text(encoding="utf-8"))
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
            return copy.deepcopy(cached) if copy_result else cached

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
                return copy.deepcopy(rows)
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
        return json.loads(Path(path).read_text(encoding="utf-8"))
    return cache.read_json(path, copy_result=copy_result, kind=kind)
