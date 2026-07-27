# -*- coding: utf-8 -*-
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from modules.monitoring.logic.mission_update import load_db_json

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


def _coerce_int(value: object | None) -> int | None:
    try:
        if value is None:
            return None
        return int(value)
    except Exception:
        return None


def _extract_waypoint_id(value: object | None) -> int | None:
    if isinstance(value, dict):
        return _coerce_int(
            value.get("waypointID")
            or value.get("WaypointID")
            or value.get("waypointId")
        )
    return _coerce_int(value)


@dataclass
class SourceArtifactIndex:
    source_plan_id: int
    plan_data: dict[str, Any]
    input_mission_package_id: int | None
    aircraft_by_id: dict[int, dict[str, Any]]
    _imp_by_id: dict[int, dict[str, Any]] = field(default_factory=dict)
    _flight_path_by_id: dict[int, dict[str, Any]] = field(default_factory=dict)
    _waypoint_ids_by_path_id: dict[int, frozenset[int]] = field(default_factory=dict)
    _waypoint_by_path_id: dict[int, dict[int, dict[str, Any]]] = field(default_factory=dict)

    @classmethod
    def from_source_plan(cls, source_plan_id: int) -> "SourceArtifactIndex | None":
        started = replan_perf.start_timer()
        plan_data = load_db_json("MissionPlan", int(source_plan_id))
        if not plan_data:
            replan_perf.add_elapsed(
                "monitoring.source_artifact_index.from_source_plan",
                started,
                read_files=0,
                found=0,
            )
            return None
        input_mission_package_id = _coerce_int(
            plan_data.get("inputMissionPackageID")
            or plan_data.get("InputMissionPackageID")
            or plan_data.get("inputMissionPackageId")
        )
        aircraft_by_id: dict[int, dict[str, Any]] = {}
        for entry in plan_data.get("aircraftList") or []:
            if not isinstance(entry, dict):
                continue
            aircraft_id = _coerce_int(entry.get("aircraftID"))
            if aircraft_id is None:
                continue
            aircraft_by_id.setdefault(int(aircraft_id), entry)
        replan_perf.add_elapsed(
            "monitoring.source_artifact_index.from_source_plan",
            started,
            read_files=1,
            found=1,
            aircraft_entries=len(aircraft_by_id),
        )
        return cls(
            source_plan_id=int(source_plan_id),
            plan_data=plan_data,
            input_mission_package_id=input_mission_package_id,
            aircraft_by_id=aircraft_by_id,
        )

    def individual_package_id_for_aircraft(self, aircraft_id: int) -> int | None:
        aircraft_entry = self.aircraft_by_id.get(int(aircraft_id))
        if not isinstance(aircraft_entry, dict):
            return None
        package_id = _coerce_int(
            aircraft_entry.get("individualMissionPackageID")
            or aircraft_entry.get("individualMissionPlanPackageID")
            or aircraft_entry.get("individualMissionPackageId")
        )
        if package_id is None or package_id <= 0:
            return None
        return int(package_id)

    def individual_missions(self, individual_package_id: int) -> list[dict[str, Any]]:
        started = replan_perf.start_timer()
        payload = self._imp_by_id.get(int(individual_package_id))
        cache_hit = payload is not None
        if payload is None:
            payload = load_db_json("IndividualMissionPlan", int(individual_package_id))
            self._imp_by_id[int(individual_package_id)] = payload
        missions = [item for item in (payload.get("individualMissionList") or []) if isinstance(item, dict)]
        replan_perf.add_elapsed(
            "monitoring.source_artifact_index.individual_missions",
            started,
            read_files=0 if cache_hit else 1,
            cache_hit=cache_hit,
            cache_miss=not cache_hit,
            mission_entries=len(missions),
        )
        return missions

    def flight_path(self, path_id: int) -> dict[str, Any]:
        started = replan_perf.start_timer()
        payload = self._flight_path_by_id.get(int(path_id))
        cache_hit = payload is not None
        if payload is None:
            payload = load_db_json("FlightPath", int(path_id))
            self._flight_path_by_id[int(path_id)] = payload
        waypoint_list = payload.get("waypointList") or payload.get("lahWaypointList") or []
        replan_perf.add_elapsed(
            "monitoring.source_artifact_index.flight_path",
            started,
            read_files=0 if cache_hit else 1,
            cache_hit=cache_hit,
            cache_miss=not cache_hit,
            waypoint_entries=len(waypoint_list) if isinstance(waypoint_list, list) else 0,
        )
        return payload

    def waypoint_ids(self, path_id: int) -> frozenset[int]:
        cached = self._waypoint_ids_by_path_id.get(int(path_id))
        if cached is not None:
            return cached
        waypoint_map = self.waypoints_by_id(path_id)
        ids = frozenset(int(value) for value in waypoint_map.keys())
        self._waypoint_ids_by_path_id[int(path_id)] = ids
        return ids

    def waypoints_by_id(self, path_id: int) -> dict[int, dict[str, Any]]:
        started = replan_perf.start_timer()
        cached = self._waypoint_by_path_id.get(int(path_id))
        if cached is not None:
            replan_perf.add_elapsed(
                "monitoring.source_artifact_index.waypoint_scan",
                started,
                cache_hit=1,
                cache_miss=0,
                waypoint_scan=0,
                waypoint_indexed=len(cached),
            )
            return cached
        fp_data = self.flight_path(path_id)
        waypoint_list = fp_data.get("waypointList") or fp_data.get("lahWaypointList") or []
        waypoint_map: dict[int, dict[str, Any]] = {}
        for waypoint in waypoint_list:
            if not isinstance(waypoint, dict):
                continue
            waypoint_id = _extract_waypoint_id(waypoint.get("waypointID"))
            if waypoint_id is None or waypoint_id <= 0:
                continue
            waypoint_map.setdefault(int(waypoint_id), waypoint)
        self._waypoint_by_path_id[int(path_id)] = waypoint_map
        replan_perf.add_elapsed(
            "monitoring.source_artifact_index.waypoint_scan",
            started,
            cache_hit=0,
            cache_miss=1,
            waypoint_scan=len(waypoint_list) if isinstance(waypoint_list, list) else 0,
            waypoint_indexed=len(waypoint_map),
        )
        return waypoint_map

    def waypoint_for_path(self, path_id: int, waypoint_id: int) -> dict[str, Any] | None:
        waypoint = self.waypoints_by_id(path_id).get(int(waypoint_id))
        if not isinstance(waypoint, dict):
            return None
        return waypoint
