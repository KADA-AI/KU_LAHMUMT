from __future__ import annotations

import concurrent.futures
import json
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


DEFAULT_AIRCRAFT_PARALLEL_WORKERS = 3
LEGACY_AIRCRAFT_PARALLEL_WORKERS = 2


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


def _payload_value(payload: Optional[Dict[str, Any]], key: str, default: Any) -> Any:
    if not isinstance(payload, dict):
        return default
    values = payload.get("values")
    if isinstance(values, dict) and key in values:
        return values.get(key)
    if key in payload:
        return payload.get(key)
    return default


def _setting_flag(
    key: str,
    default: bool,
    *,
    runtime_payload: Optional[Dict[str, Any]],
    env: Optional[Mapping[str, str]],
    env_names: List[str],
) -> bool:
    source = env if env is not None else os.environ
    for env_name in env_names:
        if env_name in source:
            return _env_flag(env_name, default, env)
    raw = _payload_value(runtime_payload, key, default)
    if isinstance(raw, str):
        lowered = raw.strip().lower()
        if lowered in {"0", "false", "no", "off"}:
            return False
        if lowered in {"1", "true", "yes", "on"}:
            return True
    return bool(raw)


def _setting_int(
    key: str,
    default: int,
    *,
    runtime_payload: Optional[Dict[str, Any]],
    env: Optional[Mapping[str, str]],
    env_names: List[str],
) -> int:
    source = env if env is not None else os.environ
    for env_name in env_names:
        if env_name in source:
            return _env_int(env_name, default, env)
    raw = _payload_value(runtime_payload, key, default)
    try:
        return int(float(raw))
    except Exception:
        return int(default)


def _runtime_context(payload: Optional[Dict[str, Any]]):
    if runtime_settings_override is None:
        return nullcontext()
    try:
        return runtime_settings_override(payload)
    except Exception:
        return nullcontext()


def _suppress_linesearch_inner_parallel_context(d0303_module: Any):
    factory = getattr(d0303_module, "suppress_linesearch_inner_parallel", None)
    if not callable(factory):
        return nullcontext()
    try:
        return factory()
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


def _formation_dependency_key(mission: Any) -> Optional[int]:
    if not isinstance(mission, dict):
        return None
    rel = mission.get("relatedMission") if isinstance(mission.get("relatedMission"), dict) else {}
    raw = rel.get("inputMissionID") or mission.get("inputMissionID") or mission.get("pathID")
    return _safe_int(raw)


def classify_0303_dependency_groups(missions: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    indexed = [(idx, mission) for idx, mission in enumerate(missions or []) if isinstance(mission, dict)]
    if not indexed:
        return []

    parent: Dict[int, int] = {idx: idx for idx, _mission in indexed}

    def find(idx: int) -> int:
        root = parent[idx]
        while root != parent[root]:
            root = parent[root]
        while idx != root:
            nxt = parent[idx]
            parent[idx] = root
            idx = nxt
        return root

    def union(left: int, right: int) -> None:
        left_root = find(left)
        right_root = find(right)
        if left_root != right_root:
            parent[right_root] = left_root

    by_aircraft: Dict[int, List[int]] = {}
    by_formation: Dict[int, List[int]] = {}
    for idx, mission in indexed:
        aid = _mission_aircraft_id(mission)
        if aid is not None:
            by_aircraft.setdefault(int(aid), []).append(idx)
        if _is_formation_like_mission(mission):
            fkey = _formation_dependency_key(mission)
            if fkey is not None:
                by_formation.setdefault(int(fkey), []).append(idx)

    for indices in by_aircraft.values():
        if len(indices) < 2:
            continue
        first = indices[0]
        for idx in indices[1:]:
            union(first, idx)

    for indices in by_formation.values():
        if len(indices) < 2:
            continue
        first = indices[0]
        for idx in indices[1:]:
            union(first, idx)

    grouped_indices: Dict[int, List[int]] = {}
    mission_by_index = {idx: mission for idx, mission in indexed}
    for idx, _mission in indexed:
        grouped_indices.setdefault(find(idx), []).append(idx)

    groups: List[Dict[str, Any]] = []
    for ordinal, indices in enumerate(
        sorted((sorted(values) for values in grouped_indices.values()), key=lambda values: values[0]),
        start=1,
    ):
        group_missions = [mission_by_index[idx] for idx in indices]
        aircraft = sorted({
            int(aid)
            for aid in (_mission_aircraft_id(mission) for mission in group_missions)
            if aid is not None
        })
        formation_keys = sorted({
            int(fkey)
            for fkey in (
                _formation_dependency_key(mission)
                for mission in group_missions
                if _is_formation_like_mission(mission)
            )
            if fkey is not None
        })
        groups.append({
            "id": f"group{ordinal}",
            "indices": list(indices),
            "missions": group_missions,
            "aircraft": aircraft,
            "pathIDs": [
                int(path_id)
                for path_id in (_mission_path_id(mission) for mission in group_missions)
                if path_id is not None
            ],
            "formationKeys": formation_keys,
            "hasFormation": bool(formation_keys),
        })
    return groups


def validate_0303_dependency_group_guard(groups: List[Dict[str, Any]]) -> Dict[str, Any]:
    if not isinstance(groups, list):
        return {"valid": False, "reason": "groups_not_list"}

    group_ids: set[str] = set()
    path_to_group: Dict[int, str] = {}
    for ordinal, group in enumerate(groups):
        if not isinstance(group, dict):
            return {"valid": False, "reason": "group_not_dict"}
        group_id = str(group.get("id") or f"group{ordinal + 1}")
        if group_id in group_ids:
            return {"valid": False, "reason": "duplicate_group_id"}
        group_ids.add(group_id)
        missions = group.get("missions")
        if not isinstance(missions, list):
            return {"valid": False, "reason": "group_missions_not_list"}
        indices = group.get("indices")
        if not isinstance(indices, list) or len(indices) != len(missions):
            return {"valid": False, "reason": "group_indices_mismatch"}
        for mission in missions:
            path_id = _mission_path_id(mission)
            if path_id is None:
                continue
            path_id = int(path_id)
            previous_group = path_to_group.get(path_id)
            if previous_group is not None and previous_group != group_id:
                return {"valid": False, "reason": "path_id_in_multiple_groups"}
            path_to_group[path_id] = group_id
    return {"valid": True, "reason": "-"}


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


def _formation_leader_aircraft_id(plan: Any) -> Optional[int]:
    if not isinstance(plan, dict):
        return None
    info = plan.get("formationInfo")
    if not isinstance(info, dict):
        return None
    leader_id = _safe_int(info.get("leaderAircraftID"))
    aircraft_id = _mission_aircraft_id(plan)
    if leader_id is None or aircraft_id is None or int(leader_id) == int(aircraft_id):
        return None
    return int(leader_id)


def _order_formation_leaders_before_followers(plans: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    if len(plans or []) < 2:
        return plans

    ordered = list(plans)
    idx = 0
    while idx < len(ordered):
        leader_id = _formation_leader_aircraft_id(ordered[idx])
        if leader_id is None:
            idx += 1
            continue
        leader_idx = None
        for candidate_idx, candidate in enumerate(ordered):
            if candidate_idx == idx:
                continue
            aid = _mission_aircraft_id(candidate)
            if aid is not None and int(aid) == int(leader_id):
                leader_idx = int(candidate_idx)
                break
        if leader_idx is not None and leader_idx > idx:
            leader_plan = ordered.pop(leader_idx)
            ordered.insert(idx, leader_plan)
            idx += 2
            continue
        idx += 1
    return ordered


def _sort_plans_for_parallel_merge(plans: List[Dict[str, Any]], missions: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    return _order_formation_leaders_before_followers(_sort_plans_by_input_order(plans, missions))


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


def _waypoint_rows_from_plans(plans: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    for plan in plans:
        if not isinstance(plan, dict):
            continue
        waypoints = plan.get("waypointList")
        if not isinstance(waypoints, list) or not waypoints:
            continue
        rows.extend(wp for wp in waypoints if isinstance(wp, dict))
    return rows


def validate_0303_parallel_merge_output(plans: List[Dict[str, Any]]) -> Dict[str, Any]:
    errors: List[str] = []
    waypoint_ids: set[int] = set()
    duplicate_waypoint_ids: set[int] = set()
    first_index_by_aircraft: Dict[int, int] = {}

    for plan_idx, plan in enumerate(plans or []):
        if not isinstance(plan, dict):
            continue
        aid = _mission_aircraft_id(plan)
        if aid is not None and int(aid) not in first_index_by_aircraft:
            first_index_by_aircraft[int(aid)] = int(plan_idx)
        waypoints = plan.get("waypointList")
        if not isinstance(waypoints, list):
            continue
        for wp_idx, wp in enumerate(waypoints):
            if not isinstance(wp, dict):
                continue
            waypoint_id = _safe_int(wp.get("waypointID"))
            if waypoint_id is not None and waypoint_id > 0:
                if int(waypoint_id) in waypoint_ids:
                    duplicate_waypoint_ids.add(int(waypoint_id))
                waypoint_ids.add(int(waypoint_id))
            expected_next = 0
            if wp_idx + 1 < len(waypoints) and isinstance(waypoints[wp_idx + 1], dict):
                expected_next = int(_safe_int(waypoints[wp_idx + 1].get("waypointID"), 0) or 0)
            actual_next = int(_safe_int(wp.get("nextWaypointID"), 0) or 0)
            if actual_next != expected_next:
                errors.append(f"nextWaypointID:{plan.get('pathID')}:{wp_idx}")

    formation_order_violations: List[str] = []
    for plan_idx, plan in enumerate(plans or []):
        leader_id = _formation_leader_aircraft_id(plan)
        if leader_id is None:
            continue
        leader_idx = first_index_by_aircraft.get(int(leader_id))
        if leader_idx is not None and int(leader_idx) > int(plan_idx):
            formation_order_violations.append(f"{plan.get('pathID')}:{leader_id}")

    if duplicate_waypoint_ids:
        errors.append("duplicateWaypointID")
    if formation_order_violations:
        errors.append("formationLeaderAfterFollower")
    return {
        "valid": not errors,
        "errors": list(errors),
        "duplicateWaypointIDs": sorted(duplicate_waypoint_ids),
        "formationLeaderOrderViolations": formation_order_violations,
    }


def summarize_aircraft_parallel_worker_policy(
    *,
    aircraft_count: int,
    requested_workers: int,
    effective_workers: int,
) -> Dict[str, Any]:
    aircraft = max(0, int(aircraft_count or 0))
    requested = max(1, int(requested_workers or 1))
    effective = max(1, int(effective_workers or 1))
    return {
        "aircraft": int(aircraft),
        "defaultWorkers": int(DEFAULT_AIRCRAFT_PARALLEL_WORKERS),
        "legacyDefaultWorkers": int(LEGACY_AIRCRAFT_PARALLEL_WORKERS),
        "requestedWorkers": int(requested),
        "effectiveWorkers": int(effective),
        "legacyDefaultTwoWorkerBottleneck": bool(aircraft > LEGACY_AIRCRAFT_PARALLEL_WORKERS),
        "effectiveWorkerBottleneck": bool(aircraft > effective),
    }


def validate_waypoint_id_sets_no_overlap(
    *,
    flight_plans_0303: List[Dict[str, Any]],
    flight_plans_0304: List[Dict[str, Any]],
) -> Dict[str, Any]:
    def _ids(plans: List[Dict[str, Any]], key: str) -> set[int]:
        found: set[int] = set()
        for plan in plans or []:
            if not isinstance(plan, dict):
                continue
            rows = plan.get(key)
            if not isinstance(rows, list):
                continue
            for wp in rows:
                if not isinstance(wp, dict):
                    continue
                wid = _safe_int(wp.get("waypointID"))
                if wid is not None and wid > 0:
                    found.add(int(wid))
        return found

    ids_0303 = _ids(flight_plans_0303, "waypointList")
    ids_0304 = _ids(flight_plans_0304, "lahWaypointList")
    overlap = sorted(ids_0303.intersection(ids_0304))
    return {
        "valid": not overlap,
        "waypointIDs0303": sorted(ids_0303),
        "waypointIDs0304": sorted(ids_0304),
        "overlapWaypointIDs": overlap,
    }


def _reset_dense_linesearch_metrics(d0303_module: Any) -> None:
    reset = getattr(d0303_module, "reset_dense_linesearch_metrics", None)
    if callable(reset):
        try:
            reset()
        except Exception:
            pass


def _get_dense_linesearch_metrics(d0303_module: Any) -> Dict[str, Any]:
    getter = getattr(d0303_module, "get_dense_linesearch_metrics", None)
    if not callable(getter):
        return {}
    try:
        value = getter(reset=False)
    except TypeError:
        try:
            value = getter()
        except Exception:
            return {}
    except Exception:
        return {}
    return dict(value) if isinstance(value, dict) else {}


def _merge_dense_linesearch_metrics(rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    merged: Dict[str, Any] = {}
    for row in rows or []:
        if not isinstance(row, dict):
            continue
        for key, value in row.items():
            if key in {
                "maxLineSearchCoords",
                "lineSearchCoordinateCap",
                "innerParallelWorkers",
                "demTileCount",
                "formationPostProcessWorkers",
            }:
                merged[key] = max(int(merged.get(key, 0) or 0), int(value or 0))
            elif key == "demBatchFallbackReason":
                text = str(value or "").strip()
                if not text:
                    continue
                existing = str(merged.get(key) or "").strip()
                if not existing:
                    merged[key] = text
                elif text not in set(existing.split(",")):
                    merged[key] = existing + "," + text
            elif key in {
                "generateLineSearchMs",
                "altitudeGuardMs",
                "altitudeApplyMs",
                "altitudeFloorMs",
                "formationLeaderBuildMs",
                "formationFollowerBuildMs",
                "formationGroupingMs",
                "missionPacketBuildMs",
                "formationDemMs",
                "formationPostProcessMs",
                "etaEcfMs",
                "searchSpeedRecalcMs",
                "lineSearchSpeedMs",
                "lineSearchMergeMs",
                "areaPostProcessMs",
                "areaRepositionMs",
                "areaPrePackAltitudeMs",
                "areaPackMs",
                "filmingNormalizeMs",
                "filmingTerrainBatchMs",
                "groundPrepassMs",
                "groundRequiredFinalPrepassFreshSkipMs",
                "groundRequiredComputeMs",
                "demBulkLookupMs",
                "jsonReadyMs",
                "outputNormalizeMs",
                "unaccounted0303Ms",
                "demTileResolveMs",
                "demPixelReadMs",
                "demCacheReadMs",
                "demCacheWriteMs",
                "demAltCacheReadMs",
                "demAltCacheWriteMs",
            }:
                merged[key] = round(float(merged.get(key, 0.0) or 0.0) + float(value or 0.0), 3)
            else:
                try:
                    merged[key] = int(merged.get(key, 0) or 0) + int(value or 0)
                except Exception:
                    pass
    return merged


def _summarize_line_search_counts(plans: List[Dict[str, Any]]) -> Dict[str, int]:
    total_coords = 0
    max_coords = 0
    line_search_count = 0
    line_search_json_bytes = 0
    path_count = 0
    for plan in plans or []:
        if not isinstance(plan, dict):
            continue
        path_count += 1
        for wp in plan.get("waypointList") or []:
            if not isinstance(wp, dict):
                continue
            filming = wp.get("filmingProperty")
            if not isinstance(filming, dict):
                continue
            line_search = filming.get("lineSearch")
            if not isinstance(line_search, dict):
                continue
            coords = line_search.get("coordinateList")
            if not isinstance(coords, list):
                continue
            count = len(coords)
            line_search_count += 1
            total_coords += count
            max_coords = max(max_coords, count)
            try:
                line_search_json_bytes += len(
                    json.dumps(line_search, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
                )
            except Exception:
                pass
    return {
        "paths": int(path_count),
        "lineSearchCount": int(line_search_count),
        "lineSearchCoordCount": int(total_coords),
        "maxLineSearchCoords": int(max_coords),
        "lineSearchJsonBytes": int(line_search_json_bytes),
    }


def reassign_waypoint_ids_inplace(plans: List[Dict[str, Any]], wp_alloc: Any) -> int:
    if wp_alloc is None or not hasattr(wp_alloc, "alloc"):
        raise RuntimeError("Waypoint allocator unavailable for 0303 aircraft parallel output.")
    waypoint_refs = _waypoint_rows_from_plans(plans)
    allocated_ids = [int(wp_alloc.alloc()) for _ in waypoint_refs]
    for wp, waypoint_id in zip(waypoint_refs, allocated_ids):
        wp["waypointID"] = int(waypoint_id)

    count = len(waypoint_refs)
    for plan in plans:
        if not isinstance(plan, dict):
            continue
        waypoints = plan.get("waypointList")
        if not isinstance(waypoints, list) or not waypoints:
            continue
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
    phase_ms: Dict[str, float] = {}

    def _phase(name: str, started_at: float) -> None:
        phase_ms[str(name)] = round((time.perf_counter() - started_at) * 1000.0, 3)

    phase_started = time.perf_counter()
    clean_missions = [mission for mission in missions if isinstance(mission, dict)]
    _phase("input_filter", phase_started)

    def _build_sequential(
        reasons: List[str],
        *,
        dag_groups: int = 0,
        dag_fallback_reason: str = "-",
    ) -> Dict[str, Any]:
        seq_start = time.perf_counter()
        _reset_dense_linesearch_metrics(d0303_module)
        with _runtime_context(runtime_payload):
            plans = d0303_module.build_flight_plans(
                clean_missions,
                wp_alloc,
                float(cruise_speed),
                turn_step_deg=float(turn_step_deg),
                ref0203=ref0203,
            )
        dense_metrics = _get_dense_linesearch_metrics(d0303_module)
        plans_list = list(plans or [])
        return {
            "plans": plans_list,
            "elapsed_ms": (time.perf_counter() - seq_start) * 1000.0,
            "mode": "sequential",
            "workers": 1,
            "aircraft": len(_group_missions_by_aircraft(clean_missions)),
            "fallback_reasons": list(reasons),
            "worker_ms_by_aircraft": {},
            "reassigned_waypoints": 0,
            "waypoint_count_prepass": 0,
            "worker_policy": summarize_aircraft_parallel_worker_policy(
                aircraft_count=len(_group_missions_by_aircraft(clean_missions)),
                requested_workers=1,
                effective_workers=1,
            ),
            "phase_ms": {"sequential_build": round((time.perf_counter() - seq_start) * 1000.0, 3)},
            "dense_linesearch_metrics": dense_metrics,
            "line_search_counts": _summarize_line_search_counts(plans_list),
            "dependency_groups": 0,
            "formation_groups": 0,
            "independent_groups": 0,
            "group_worker_ms": {},
            "dependency_parallel_fallback": "-",
            "dag_groups": int(dag_groups),
            "dag_fallback_reason": str(dag_fallback_reason or "-"),
        }

    if not clean_missions:
        return _build_sequential(["no_missions"])
    if not _env_flag("REPLAN_0303_AIRCRAFT_PARALLEL", True, env):
        return _build_sequential(["env_disabled"])
    if wp_alloc is None or not hasattr(wp_alloc, "alloc"):
        return _build_sequential(["wp_allocator_unavailable"])
    allocator_cls = getattr(d0303_module, "_WPAllocator", None)
    if allocator_cls is None:
        return _build_sequential(["local_wp_allocator_unavailable"])

    def _build_dependency_parallel() -> Dict[str, Any]:
        dep_start = time.perf_counter()
        phase_started_dep = time.perf_counter()
        groups = classify_0303_dependency_groups(clean_missions)
        _phase("dependency_group_missions", phase_started_dep)
        dag_guard = validate_0303_dependency_group_guard(groups)
        if not bool(dag_guard.get("valid")):
            dag_reason = str(dag_guard.get("reason") or "guard_failed")
            return _build_sequential(
                ["dependency_parallel_guard_failed", dag_reason],
                dag_groups=len(groups),
                dag_fallback_reason=dag_reason,
            )
        formation_groups = sum(1 for group in groups if bool(group.get("hasFormation")))
        independent_groups = max(0, len(groups) - formation_groups)
        requested_workers = max(
            1,
            _setting_int(
                "replan_0303_dependency_workers",
                DEFAULT_AIRCRAFT_PARALLEL_WORKERS,
                runtime_payload=runtime_payload,
                env=env,
                env_names=[
                    "REPLAN_0303_DEPENDENCY_WORKERS",
                    "MISSION_PLAN_REPLAN_0303_DEPENDENCY_WORKERS",
                ],
            ),
        )
        workers = max(1, min(len(groups) or 1, requested_workers))
        aircraft_count = len(_group_missions_by_aircraft(clean_missions))
        worker_policy = summarize_aircraft_parallel_worker_policy(
            aircraft_count=aircraft_count,
            requested_workers=requested_workers,
            effective_workers=workers,
        )

        def _result_common(
            *,
            plans: List[Dict[str, Any]],
            elapsed_ms: float,
            fallback: str,
            worker_ms: Dict[str, float],
            dense_metrics: Dict[str, Any],
            reassigned: int,
            waypoint_count_prepass: int,
        ) -> Dict[str, Any]:
            return {
                "plans": plans,
                "elapsed_ms": elapsed_ms,
                "mode": "dependency_parallel",
                "workers": int(workers),
                "aircraft": aircraft_count,
                "fallback_reasons": [],
                "worker_ms_by_aircraft": {},
                "group_worker_ms": dict(sorted(worker_ms.items())),
                "reassigned_waypoints": int(reassigned),
                "waypoint_count_prepass": int(waypoint_count_prepass),
                "worker_policy": worker_policy,
                "phase_ms": dict(phase_ms),
                "dense_linesearch_metrics": dense_metrics,
                "line_search_counts": _summarize_line_search_counts(plans),
                "dependency_groups": int(len(groups)),
                "formation_groups": int(formation_groups),
                "independent_groups": int(independent_groups),
                "dependency_parallel_fallback": str(fallback or "-"),
                "dag_groups": int(len(groups)),
                "dag_fallback_reason": str(fallback or "-"),
            }

        if len(groups) < 2 or workers < 2:
            seq_group_start = time.perf_counter()
            _reset_dense_linesearch_metrics(d0303_module)
            with _runtime_context(runtime_payload):
                plans = d0303_module.build_flight_plans(
                    clean_missions,
                    wp_alloc,
                    float(cruise_speed),
                    turn_step_deg=float(turn_step_deg),
                    ref0203=ref0203,
                )
            plans_list = list(plans or [])
            _normalize_timestamps(plans_list)
            elapsed = (time.perf_counter() - seq_group_start) * 1000.0
            phase_ms["dependency_single_group_build"] = round(elapsed, 3)
            return _result_common(
                plans=plans_list,
                elapsed_ms=(time.perf_counter() - dep_start) * 1000.0,
                fallback="single_dependency_group" if len(groups) < 2 else "worker_count_lt_2",
                worker_ms={str((groups[0] or {}).get("id") or "group1"): elapsed} if groups else {},
                dense_metrics=_get_dense_linesearch_metrics(d0303_module),
                reassigned=0,
                waypoint_count_prepass=0,
            )

        def _build_one_group(group: Dict[str, Any]) -> tuple[str, List[Dict[str, Any]], float, Dict[str, Any]]:
            group_id = str(group.get("id") or "group")
            worker_start = time.perf_counter()
            local_alloc = allocator_cls(start=1)
            _reset_dense_linesearch_metrics(d0303_module)
            with _runtime_context(runtime_payload), _suppress_linesearch_inner_parallel_context(d0303_module):
                plans = d0303_module.build_flight_plans(
                    list(group.get("missions") or []),
                    local_alloc,
                    float(cruise_speed),
                    turn_step_deg=float(turn_step_deg),
                    ref0203=ref0203,
                )
            return (
                group_id,
                list(plans or []),
                (time.perf_counter() - worker_start) * 1000.0,
                _get_dense_linesearch_metrics(d0303_module),
            )

        group_worker_ms: Dict[str, float] = {}
        dense_metrics_by_group: Dict[str, Dict[str, Any]] = {}
        try:
            built: List[Dict[str, Any]] = []
            phase_started_dep = time.perf_counter()
            with concurrent.futures.ThreadPoolExecutor(
                max_workers=workers,
                thread_name_prefix="Build0303Dependency",
            ) as executor:
                futures = [executor.submit(_build_one_group, group) for group in groups]
                for future in concurrent.futures.as_completed(futures):
                    group_id, plans, worker_ms, dense_metrics = future.result()
                    group_worker_ms[group_id] = float(worker_ms)
                    dense_metrics_by_group[group_id] = dict(dense_metrics or {})
                    built.extend(plans)
            _phase("dependency_group_build", phase_started_dep)
        except Exception:
            return _build_sequential(["dependency_parallel_failed"])

        phase_started_dep = time.perf_counter()
        built = _sort_plans_for_parallel_merge(built, clean_missions)
        _phase("dependency_sort_by_input_order", phase_started_dep)
        phase_started_dep = time.perf_counter()
        _normalize_timestamps(built)
        _phase("dependency_normalize_timestamps", phase_started_dep)
        phase_started_dep = time.perf_counter()
        reassigned = reassign_waypoint_ids_inplace(built, wp_alloc)
        _phase("dependency_reassign_waypoint_ids", phase_started_dep)
        merge_validation = validate_0303_parallel_merge_output(built)
        if not bool(merge_validation.get("valid")):
            return _build_sequential(["dependency_parallel_output_invalid"])
        return _result_common(
            plans=built,
            elapsed_ms=(time.perf_counter() - dep_start) * 1000.0,
            fallback="-",
            worker_ms=group_worker_ms,
            dense_metrics=_merge_dense_linesearch_metrics(list(dense_metrics_by_group.values())),
            reassigned=int(reassigned),
            waypoint_count_prepass=int(reassigned),
        )

    if any(_is_formation_like_mission(mission) for mission in clean_missions):
        if not _setting_flag(
            "replan_0303_dependency_parallel_enabled",
            True,
            runtime_payload=runtime_payload,
            env=env,
            env_names=[
                "REPLAN_0303_DEPENDENCY_PARALLEL",
                "MISSION_PLAN_REPLAN_0303_DEPENDENCY_PARALLEL_ENABLED",
            ],
        ):
            return _build_sequential(["dependency_parallel_disabled"])
        return _build_dependency_parallel()

    phase_started = time.perf_counter()
    grouped = _group_missions_by_aircraft(clean_missions)
    _phase("group_missions", phase_started)
    if len(grouped) < 2:
        return _build_sequential(["aircraft_count_lt_2"])
    requested_workers = max(
        1,
        _setting_int(
            "replan_0303_aircraft_workers",
            DEFAULT_AIRCRAFT_PARALLEL_WORKERS,
            runtime_payload=runtime_payload,
            env=env,
            env_names=[
                "REPLAN_0303_AIRCRAFT_WORKERS",
                "MISSION_PLAN_REPLAN_0303_AIRCRAFT_WORKERS",
            ],
        ),
    )
    workers = min(len(grouped), requested_workers)
    worker_policy = summarize_aircraft_parallel_worker_policy(
        aircraft_count=len(grouped),
        requested_workers=requested_workers,
        effective_workers=workers,
    )
    if workers < 2:
        return _build_sequential(["worker_count_lt_2"])

    def _build_one(aid: int, group: List[Dict[str, Any]]) -> tuple[int, List[Dict[str, Any]], float, Dict[str, Any]]:
        worker_start = time.perf_counter()
        local_alloc = allocator_cls(start=1)
        _reset_dense_linesearch_metrics(d0303_module)
        with _runtime_context(runtime_payload), _suppress_linesearch_inner_parallel_context(d0303_module):
            plans = d0303_module.build_flight_plans(
                group,
                local_alloc,
                float(cruise_speed),
                turn_step_deg=float(turn_step_deg),
                ref0203=ref0203,
            )
        return int(aid), list(plans or []), (time.perf_counter() - worker_start) * 1000.0, _get_dense_linesearch_metrics(d0303_module)

    worker_ms_by_aircraft: Dict[int, float] = {}
    dense_metrics_by_aircraft: Dict[int, Dict[str, Any]] = {}
    try:
        built: List[Dict[str, Any]] = []
        phase_started = time.perf_counter()
        with concurrent.futures.ThreadPoolExecutor(
            max_workers=workers,
            thread_name_prefix="Build0303Aircraft",
        ) as executor:
            futures = [
                executor.submit(_build_one, aid, grouped[aid])
                for aid in sorted(grouped)
            ]
            for future in concurrent.futures.as_completed(futures):
                aid, plans, worker_ms, dense_metrics = future.result()
                worker_ms_by_aircraft[int(aid)] = float(worker_ms)
                dense_metrics_by_aircraft[int(aid)] = dict(dense_metrics or {})
                built.extend(plans)
        _phase("per_aircraft_build", phase_started)
    except Exception:
        return _build_sequential(["aircraft_parallel_failed"])

    phase_started = time.perf_counter()
    built = _sort_plans_for_parallel_merge(built, clean_missions)
    _phase("sort_by_input_order", phase_started)
    phase_started = time.perf_counter()
    _normalize_timestamps(built)
    _phase("normalize_timestamps", phase_started)
    phase_started = time.perf_counter()
    reassigned = reassign_waypoint_ids_inplace(built, wp_alloc)
    _phase("reassign_waypoint_ids", phase_started)
    merge_validation = validate_0303_parallel_merge_output(built)
    if not bool(merge_validation.get("valid")):
        return _build_sequential(["aircraft_parallel_output_invalid"])
    return {
        "plans": built,
        "elapsed_ms": (time.perf_counter() - start) * 1000.0,
        "mode": "aircraft_parallel",
        "workers": int(workers),
        "aircraft": len(grouped),
        "fallback_reasons": [],
        "worker_ms_by_aircraft": dict(sorted(worker_ms_by_aircraft.items())),
        "reassigned_waypoints": int(reassigned),
        "waypoint_count_prepass": int(reassigned),
        "worker_policy": worker_policy,
        "phase_ms": dict(phase_ms),
        "dense_linesearch_metrics": _merge_dense_linesearch_metrics(list(dense_metrics_by_aircraft.values())),
        "dense_linesearch_metrics_by_aircraft": dict(sorted(dense_metrics_by_aircraft.items())),
        "line_search_counts": _summarize_line_search_counts(built),
    }
