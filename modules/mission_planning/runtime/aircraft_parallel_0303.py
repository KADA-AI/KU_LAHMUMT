from __future__ import annotations

import concurrent.futures
import os
import time
from contextlib import nullcontext
from typing import Any, Dict, List, Mapping, Optional

try:
    from ..MissionPlanner.runtime_settings import runtime_override as runtime_settings_override
except Exception:
    try:
        from modules.mission_planning.MissionPlanner.runtime_settings import runtime_override as runtime_settings_override  # type: ignore
    except Exception:
        runtime_settings_override = None  # type: ignore


def _safe_int(value: Any, default: Optional[int] = None) -> Optional[int]:
    try:
        if value is None:
            return default
        return int(value)
    except Exception:
        return default


def _env_flag(name: str, default: bool, env: Optional[Mapping[str, str]] = None) -> bool:
    source = env if env is not None else os.environ
    raw = str(source.get(name, "1" if default else "0") or "").strip().lower()
    if raw in {"0", "false", "no", "off"}:
        return False
    if raw in {"1", "true", "yes", "on"}:
        return True
    return bool(default)


def _env_int(name: str, default: int, env: Optional[Mapping[str, str]] = None) -> int:
    source = env if env is not None else os.environ
    try:
        return int(source.get(name, str(default)))
    except Exception:
        return int(default)


def _runtime_context(payload: Optional[Dict[str, Any]]):
    if runtime_settings_override is None:
        return nullcontext()
    try:
        return runtime_settings_override(payload)
    except Exception:
        return nullcontext()


def _mission_aircraft_id(mission: Any) -> Optional[int]:
    if not isinstance(mission, dict):
        return None
    return _safe_int(mission.get("aircraftID"))


def _mission_path_id(mission: Any) -> Optional[int]:
    if not isinstance(mission, dict):
        return None
    return _safe_int(mission.get("pathID"))


def _line_width(info: Dict[str, Any]) -> Optional[float]:
    line_list = info.get("lineList")
    if not isinstance(line_list, list) or not line_list:
        return None
    first = line_list[0] if isinstance(line_list[0], dict) else {}
    try:
        return float(first.get("width", 0.0))
    except Exception:
        return None


def _is_formation_like_mission(mission: Any) -> bool:
    if not isinstance(mission, dict):
        return False
    info = mission.get("individualMissionInfo")
    if not isinstance(info, dict):
        return False
    width = _line_width(info)
    if width is None:
        return False
    return width <= 1.0


def _group_missions_by_aircraft(missions: List[Dict[str, Any]]) -> Dict[int, List[Dict[str, Any]]]:
    grouped: Dict[int, List[Dict[str, Any]]] = {}
    for mission in missions:
        aid = _mission_aircraft_id(mission)
        if aid is None:
            continue
        grouped.setdefault(int(aid), []).append(mission)
    return grouped


def _sort_plans_by_input_order(plans: List[Dict[str, Any]], missions: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    order_by_path: Dict[int, int] = {}
    for idx, mission in enumerate(missions):
        path_id = _mission_path_id(mission)
        if path_id is not None and path_id not in order_by_path:
            order_by_path[int(path_id)] = int(idx)

    indexed = list(enumerate(plans))
    indexed.sort(
        key=lambda item: (
            order_by_path.get(_safe_int(item[1].get("pathID"), -1) or -1, len(order_by_path) + item[0]),
            item[0],
        )
    )
    return [plan for _, plan in indexed]


def _normalize_timestamps(plans: List[Dict[str, Any]]) -> None:
    timestamps = [
        int(ts)
        for ts in (_safe_int(plan.get("timestamp")) for plan in plans if isinstance(plan, dict))
        if ts is not None and ts > 0
    ]
    if not timestamps:
        return
    timestamp = min(timestamps)
    for plan in plans:
        if isinstance(plan, dict) and "timestamp" in plan:
            plan["timestamp"] = int(timestamp)


def reassign_waypoint_ids_inplace(plans: List[Dict[str, Any]], wp_alloc: Any) -> int:
    if wp_alloc is None or not hasattr(wp_alloc, "alloc"):
        raise RuntimeError("Waypoint allocator unavailable for 0303 aircraft parallel output.")
    count = 0
    for plan in plans:
        if not isinstance(plan, dict):
            continue
        waypoints = plan.get("waypointList")
        if not isinstance(waypoints, list) or not waypoints:
            continue
        for wp in waypoints:
            if not isinstance(wp, dict):
                continue
            wp["waypointID"] = int(wp_alloc.alloc())
            count += 1
        for idx in range(len(waypoints) - 1):
            if isinstance(waypoints[idx], dict) and isinstance(waypoints[idx + 1], dict):
                waypoints[idx]["nextWaypointID"] = int(waypoints[idx + 1].get("waypointID", 0) or 0)
        if isinstance(waypoints[-1], dict):
            waypoints[-1]["nextWaypointID"] = 0
    return count


def build_0303_flight_plans_aircraft_parallel(
    d0303_module: Any,
    missions: List[Dict[str, Any]],
    *,
    runtime_payload: Optional[Dict[str, Any]],
    wp_alloc: Any,
    cruise_speed: float,
    turn_step_deg: float,
    ref0203: Optional[Dict[str, Any]],
    env: Optional[Mapping[str, str]] = None,
) -> Dict[str, Any]:
    start = time.perf_counter()
    clean_missions = [mission for mission in missions if isinstance(mission, dict)]

    def _build_sequential(reasons: List[str]) -> Dict[str, Any]:
        seq_start = time.perf_counter()
        with _runtime_context(runtime_payload):
            plans = d0303_module.build_flight_plans(
                clean_missions,
                wp_alloc,
                float(cruise_speed),
                turn_step_deg=float(turn_step_deg),
                ref0203=ref0203,
            )
        return {
            "plans": list(plans or []),
            "elapsed_ms": (time.perf_counter() - seq_start) * 1000.0,
            "mode": "sequential",
            "workers": 1,
            "aircraft": len(_group_missions_by_aircraft(clean_missions)),
            "fallback_reasons": list(reasons),
            "worker_ms_by_aircraft": {},
            "reassigned_waypoints": 0,
        }

    if not clean_missions:
        return _build_sequential(["no_missions"])
    if not _env_flag("REPLAN_0303_AIRCRAFT_PARALLEL", True, env):
        return _build_sequential(["env_disabled"])
    if any(_is_formation_like_mission(mission) for mission in clean_missions):
        return _build_sequential(["formation_mission_present"])
    if wp_alloc is None or not hasattr(wp_alloc, "alloc"):
        return _build_sequential(["wp_allocator_unavailable"])
    allocator_cls = getattr(d0303_module, "_WPAllocator", None)
    if allocator_cls is None:
        return _build_sequential(["local_wp_allocator_unavailable"])

    grouped = _group_missions_by_aircraft(clean_missions)
    if len(grouped) < 2:
        return _build_sequential(["aircraft_count_lt_2"])
    requested_workers = max(1, _env_int("REPLAN_0303_AIRCRAFT_WORKERS", 2, env))
    workers = min(len(grouped), requested_workers)
    if workers < 2:
        return _build_sequential(["worker_count_lt_2"])

    def _build_one(aid: int, group: List[Dict[str, Any]]) -> tuple[int, List[Dict[str, Any]], float]:
        worker_start = time.perf_counter()
        local_alloc = allocator_cls(start=1)
        with _runtime_context(runtime_payload):
            plans = d0303_module.build_flight_plans(
                group,
                local_alloc,
                float(cruise_speed),
                turn_step_deg=float(turn_step_deg),
                ref0203=ref0203,
            )
        return int(aid), list(plans or []), (time.perf_counter() - worker_start) * 1000.0

    worker_ms_by_aircraft: Dict[int, float] = {}
    try:
        built: List[Dict[str, Any]] = []
        with concurrent.futures.ThreadPoolExecutor(
            max_workers=workers,
            thread_name_prefix="Build0303Aircraft",
        ) as executor:
            futures = [
                executor.submit(_build_one, aid, grouped[aid])
                for aid in sorted(grouped)
            ]
            for future in concurrent.futures.as_completed(futures):
                aid, plans, worker_ms = future.result()
                worker_ms_by_aircraft[int(aid)] = float(worker_ms)
                built.extend(plans)
    except Exception:
        return _build_sequential(["aircraft_parallel_failed"])

    built = _sort_plans_by_input_order(built, clean_missions)
    _normalize_timestamps(built)
    reassigned = reassign_waypoint_ids_inplace(built, wp_alloc)
    return {
        "plans": built,
        "elapsed_ms": (time.perf_counter() - start) * 1000.0,
        "mode": "aircraft_parallel",
        "workers": int(workers),
        "aircraft": len(grouped),
        "fallback_reasons": [],
        "worker_ms_by_aircraft": dict(sorted(worker_ms_by_aircraft.items())),
        "reassigned_waypoints": int(reassigned),
    }
