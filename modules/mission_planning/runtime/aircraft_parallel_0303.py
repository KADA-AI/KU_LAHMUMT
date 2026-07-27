from __future__ import annotations

import concurrent.futures
import importlib
import json
import os
import time
from contextlib import nullcontext
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional

try:
    from .persistent_process_pool import (
        acquire_process_pool as _acquire_persistent_process_pool,
        invalidate_process_pool as _invalidate_persistent_process_pool,
        shutdown_process_pools as _shutdown_persistent_process_pools,
    )
except Exception:
    try:
        from modules.mission_planning.runtime.persistent_process_pool import (
            acquire_process_pool as _acquire_persistent_process_pool,
            invalidate_process_pool as _invalidate_persistent_process_pool,
            shutdown_process_pools as _shutdown_persistent_process_pools,
        )
    except Exception:
        _acquire_persistent_process_pool = None  # type: ignore
        _invalidate_persistent_process_pool = None  # type: ignore
        _shutdown_persistent_process_pools = None  # type: ignore

try:
    from ..MissionPlanner.runtime_settings import runtime_override as runtime_settings_override
except Exception:
    try:
        from modules.mission_planning.MissionPlanner.runtime_settings import runtime_override as runtime_settings_override  # type: ignore
    except Exception:
        runtime_settings_override = None  # type: ignore

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


DEFAULT_AIRCRAFT_PARALLEL_WORKERS = 3
LEGACY_AIRCRAFT_PARALLEL_WORKERS = 2
CANONICAL_D0303_MODULE = (
    "modules.mission_planning.engine.mission_generation."
    "artifacts_0301_0302_0303_0304.d0303"
)
# A reload gives the parent a new generation and therefore a new Windows spawn
# pool.  The registry itself is intentionally not hot-reloaded.
_PROCESS_POOL_GENERATION = f"{os.getpid()}:{time.time_ns()}"
_PROCESS_POOL_FAMILY = "mission-planning-0303"
_PLANNING_ENV_PREFIXES = ("KU_", "MISSION_PLAN_", "REPLAN_", "DSS_")
_D0303_RUNTIME_GLOBAL_NAMES = (
    "FOV_DEG",
    "AREA_CUSTOM_FOV_DEG",
    "AREA_OUTPUT_FOV_SCALE",
    "LINE_SWEEP_DENSITY_SCALE",
    "FOV_DB_SEP_SAFETY_FACTOR",
    "AREA_SWEEP_DENSITY_SCALE",
    "LINE_ROUTE_OFFSET_SCALE",
    "AREA_ROUTE_OFFSET_SCALE",
    "LINE_SEARCH_SPEED_WEIGHT",
    "AREA_SEARCH_SPEED_WEIGHT",
    "AREA_FIRST_PACKET_SEARCH_SPEED_SCALE",
    "AREA_FIRST_PACKET_SWEEP_GROUP_SCALE",
    "SWEEP_ROUTE_WP_SPACING_M",
    "AREA_SWEEP_ROUTE_WP_SPACING_M",
    "DUBINS_TURN_RADIUS_M",
    "DB_FOV_WEIGHT",
    "Altitude",
    "ALTITUDE_LAYERS_M",
    "SWEEP_MERGE_HEADING_DEG",
    "SWEEP_LINE_INTERP_POINTS",
    "MAX_LINESEARCH_COORDS_PER_WAYPOINT",
    "LINESEARCH_INNER_PARALLEL_MIN_STRIPS",
    "LINESEARCH_INNER_PARALLEL_MIN_COORDS",
    "LINESEARCH_INNER_PARALLEL_WORKERS",
    "FORMATION_FOLLOWER_POSTPROCESS_PARALLEL_MIN_FOLLOWERS",
    "FORMATION_FOLLOWER_POSTPROCESS_WORKERS",
    "MIN_SWEEP_LEN_M",
    "MIN_ROUTE_SPACING_M",
    "AREA_DUBINS_ENTRY_LINKS_ENABLED",
    "DEFAULT_SEARCH_SPEED_MULTIPLIER",
    "POINT_FOV_DEG",
    "AREA_NADIR_FOV_DEG",
    "ENTRY_HOLD_FOV_DEG",
    "ENTRY_HOLD_GIMBAL_PITCH",
    "ENTRY_HOLD_GIMBAL_YAW",
    "LOITER_RADIUS_M",
    "LOITER_DIRECTION",
    "LOITER_TIME_S",
    "LOITER_SPEED_MPS",
    "FLYOVER_ENTRY_OFFSET",
    "FLYOVER_DUBINS_PREFIX",
    "FLYOVER_LAST_POINT",
    "FLYOVER_ALL_WPS",
)
_SEARCH_SPEED_MODULE_NAME = "modules.mission_planning.MissionPlanner.data_def.search_speed"
_MP_CONFIG_MODULE_NAME = "modules.mission_planning.MissionPlanner.config"
_DENSE_LINESEARCH_MAX_METRIC_KEYS = frozenset(
    {
        "maxLineSearchCoords",
        "lineSearchCoordinateCap",
        "innerParallelWorkers",
        "demTileCount",
        "formationPostProcessWorkers",
    }
)
_DENSE_LINESEARCH_REASON_METRIC_KEYS = frozenset({"demBatchFallbackReason"})
_DENSE_LINESEARCH_FLOAT_METRIC_KEYS = frozenset(
    {
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
    }
)


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


def _runtime_state_value(value: Any) -> Any:
    """Copy the small primitive-only subset that is safe to send to workers."""

    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, tuple):
        return tuple(_runtime_state_value(item) for item in value)
    if isinstance(value, list):
        return [_runtime_state_value(item) for item in value]
    if isinstance(value, dict):
        return {
            _runtime_state_value(key): _runtime_state_value(item)
            for key, item in value.items()
            if key is None or isinstance(key, (bool, int, float, str))
        }
    raise TypeError(f"unsupported 0303 runtime state value: {type(value).__name__}")


def _snapshot_d0303_runtime_state(d0303_module: Any) -> Dict[str, Any]:
    """Capture the exact parent-side settings used by the in-process builder."""

    globals_snapshot: Dict[str, Any] = {}
    for name in _D0303_RUNTIME_GLOBAL_NAMES:
        if not hasattr(d0303_module, name):
            continue
        globals_snapshot[name] = _runtime_state_value(getattr(d0303_module, name))

    sweep_geometry: Optional[Dict[str, float]] = None
    geometry = getattr(d0303_module, "SWEEP_GEOMETRY", None)
    try:
        sweep_geometry = {
            "separation_m": float(getattr(geometry, "separation_m")),
            "fov_deg": float(getattr(geometry, "fov_deg")),
        }
    except Exception:
        sweep_geometry = None

    companion_snapshot: Dict[str, Dict[str, Any]] = {}
    for module_name, names in (
        (_SEARCH_SPEED_MODULE_NAME, ("_CFG_WEIGHT",)),
        (
            _MP_CONFIG_MODULE_NAME,
            ("DEFAULT_SWEEP_SEPARATION_M", "SEARCH_SPEED_WEIGHT", "DB_FOV_WEIGHT"),
        ),
    ):
        module = importlib.import_module(module_name)
        values: Dict[str, Any] = {}
        for name in names:
            if not hasattr(module, name):
                continue
            values[name] = _runtime_state_value(getattr(module, name))
        if values:
            companion_snapshot[module_name] = values

    return {
        "d0303": globals_snapshot,
        "sweep_geometry": sweep_geometry,
        "companions": companion_snapshot,
    }


def _apply_d0303_runtime_state(
    d0303_module: Any,
    runtime_state: Optional[Dict[str, Any]],
) -> None:
    """Make a spawned/persistent worker behave exactly like its parent module."""

    if not isinstance(runtime_state, dict):
        return
    globals_snapshot = runtime_state.get("d0303")
    if isinstance(globals_snapshot, dict):
        for name in _D0303_RUNTIME_GLOBAL_NAMES:
            if name not in globals_snapshot:
                continue
            setattr(d0303_module, name, _runtime_state_value(globals_snapshot[name]))

    geometry = runtime_state.get("sweep_geometry")
    sweep_cls = getattr(d0303_module, "SweepConfig", None)
    if isinstance(geometry, dict) and callable(sweep_cls):
        d0303_module.SWEEP_GEOMETRY = sweep_cls(
            separation_m=float(geometry["separation_m"]),
            fov_deg=float(geometry["fov_deg"]),
        )

    companions = runtime_state.get("companions")
    if not isinstance(companions, dict):
        return
    for module_name in (_SEARCH_SPEED_MODULE_NAME, _MP_CONFIG_MODULE_NAME):
        values = companions.get(module_name)
        if not isinstance(values, dict):
            continue
        module = importlib.import_module(module_name)
        for name, value in values.items():
            setattr(module, str(name), _runtime_state_value(value))
def _suppress_linesearch_inner_parallel_context(d0303_module: Any):
    factory = getattr(d0303_module, "suppress_linesearch_inner_parallel", None)
    if not callable(factory):
        return nullcontext()
    try:
        return factory()
    except Exception:
        return nullcontext()


def _planning_process_context_snapshot() -> tuple[Dict[str, str], Optional[str]]:
    """Capture only planner-relevant environment plus the authoritative DB root."""

    planning_env: Dict[str, str] = {}
    try:
        planning_env = {
            str(key): str(value)
            for key, value in os.environ.items()
            if str(key).startswith(_PLANNING_ENV_PREFIXES)
        }
    except Exception:
        planning_env = {}

    db_root: Optional[str] = None
    try:
        from modules.common import db_paths

        db_root = str(db_paths.get_active_db_root())
        planning_env["KU_MISSION_DB_ROOT"] = db_root
    except Exception:
        raw_root = planning_env.get("KU_MISSION_DB_ROOT") or os.environ.get("KU_MISSION_DB_ROOT")
        db_root = str(raw_root) if raw_root else None
    return planning_env, db_root


def _apply_planning_process_context(
    planning_env: Optional[Dict[str, str]],
    db_root: Optional[str],
) -> None:
    """Refresh mutable parent context in a persistent worker before every task."""

    snapshot = {
        str(key): str(value)
        for key, value in (planning_env or {}).items()
        if str(key).startswith(_PLANNING_ENV_PREFIXES)
    }
    try:
        for key in list(os.environ):
            if str(key).startswith(_PLANNING_ENV_PREFIXES) and key not in snapshot:
                os.environ.pop(key, None)
        os.environ.update(snapshot)
    except Exception:
        pass

    if not db_root:
        return
    try:
        from modules.common import db_paths

        root_path = Path(str(db_root))
        scenario_raw = snapshot.get("KU_SCENARIO_ROOT")
        base_raw = snapshot.get("KU_SCENARIO_BASE_ROOT")
        lock = getattr(db_paths, "_lock", None)
        setter = getattr(db_paths, "_set_cached_db_root_unlocked", None)
        if lock is not None and callable(setter):
            with lock:
                setter(
                    root_path,
                    source="process-task",
                    scenario_dir=Path(scenario_raw) if scenario_raw else None,
                    agency=snapshot.get("KU_AGENCY_CODE"),
                    base_root=Path(base_raw) if base_raw else None,
                    persist=False,
                )
    except Exception:
        # The explicit environment root remains a safe fallback for modules that
        # do not use db_paths' in-memory cache.
        pass


def _initialize_0303_process_worker(
    planning_env: Optional[Dict[str, str]],
    db_root: Optional[str],
    terrain_points: Optional[List[Any]] = None,
    runtime_payload: Optional[Dict[str, Any]] = None,
    runtime_state: Optional[Dict[str, Any]] = None,
) -> None:
    """Spawn initializer: import the expensive 0303 stack in every worker."""

    _apply_planning_process_context(planning_env, db_root)
    d0303_module = importlib.import_module(CANONICAL_D0303_MODULE)
    _warm_0303_module_caches(
        d0303_module,
        terrain_points=terrain_points,
        runtime_payload=runtime_payload,
        runtime_state=runtime_state,
    )
    try:
        setattr(d0303_module, "_RECOMPUTE_LINE_SEARCH_SPEED_CACHE", None)
    except Exception:
        pass


def _warm_0303_module_caches(
    d0303_module: Any,
    *,
    terrain_points: Optional[List[Any]],
    runtime_payload: Optional[Dict[str, Any]],
    runtime_state: Optional[Dict[str, Any]],
) -> None:
    """Warm deterministic read-only caches in the worker that will use them."""

    _apply_d0303_runtime_state(d0303_module, runtime_state)
    points = list(terrain_points or [])
    with _runtime_context(runtime_payload):
        try:
            load_fov_rows = getattr(d0303_module, "_load_fov_db_rows", None)
            if callable(load_fov_rows):
                load_fov_rows()
        except Exception:
            pass
        if not points:
            return
        try:
            from modules.mission_planning.MissionPlanner.data_def.mission_helpers import (
                warm_terrain_cache,
            )

            warm_terrain_cache(points)
        except Exception:
            pass
        try:
            dem_alt_many = getattr(d0303_module, "_dem_alt_many", None)
            if callable(dem_alt_many):
                dem_alt_many(points)
        except Exception:
            pass


def _warm_0303_process_worker(
    terrain_points: Optional[List[Any]] = None,
    runtime_payload: Optional[Dict[str, Any]] = None,
    planning_env: Optional[Dict[str, str]] = None,
    db_root: Optional[str] = None,
    runtime_state: Optional[Dict[str, Any]] = None,
) -> int:
    """Pickle-safe worker warm-up for imports and optional DEM pixels."""

    _apply_planning_process_context(planning_env, db_root)
    d0303_module = importlib.import_module(CANONICAL_D0303_MODULE)
    _warm_0303_module_caches(
        d0303_module,
        terrain_points=terrain_points,
        runtime_payload=runtime_payload,
        runtime_state=runtime_state,
    )
    # A short hold gives all eagerly-created workers a chance to accept one
    # warm task instead of one fast worker consuming the whole queue.
    time.sleep(0.05)
    return int(os.getpid())


def warm_persistent_0303_process_pool(
    *,
    max_workers: int = DEFAULT_AIRCRAFT_PARALLEL_WORKERS,
    timeout_s: float = 30.0,
    env: Optional[Mapping[str, str]] = None,
    terrain_points: Optional[List[Any]] = None,
    runtime_payload: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    """Spawn/import persistent 0303 workers ahead of the first replan."""

    started = time.perf_counter()
    workers = max(1, int(max_workers))
    if workers < 2:
        return {"warmed": False, "reason": "worker_count_lt_2", "workers": workers}
    if not _setting_flag(
        "replan_0303_persistent_process_pool_enabled",
        True,
        runtime_payload=None,
        env=env,
        env_names=[
            "REPLAN_0303_PERSISTENT_PROCESS_POOL",
            "MISSION_PLAN_REPLAN_0303_PERSISTENT_PROCESS_POOL_ENABLED",
        ],
    ):
        return {"warmed": False, "reason": "persistent_pool_disabled", "workers": workers}
    if not callable(_acquire_persistent_process_pool):
        return {"warmed": False, "reason": "registry_unavailable", "workers": workers}

    executor: Any = None
    futures: List[concurrent.futures.Future] = []
    try:
        planning_env, db_root = _planning_process_context_snapshot()
        parent_d0303_module = importlib.import_module(CANONICAL_D0303_MODULE)
        runtime_state = _snapshot_d0303_runtime_state(parent_d0303_module)
        executor = _acquire_persistent_process_pool(
            family=_PROCESS_POOL_FAMILY,
            generation=_PROCESS_POOL_GENERATION,
            max_workers=workers,
            initializer=_initialize_0303_process_worker,
            initargs=(
                planning_env,
                db_root,
                list(terrain_points or []),
                runtime_payload,
                runtime_state,
            ),
        )
        futures = [
            executor.submit(
                _warm_0303_process_worker,
                list(terrain_points or []),
                runtime_payload,
                planning_env,
                db_root,
                runtime_state,
            )
            for _ in range(workers)
        ]
        done, not_done = concurrent.futures.wait(
            futures,
            timeout=max(0.1, float(timeout_s)),
        )
        if not_done:
            raise TimeoutError(f"0303 process warm-up timed out ({len(not_done)} pending)")
        task_pids = {int(future.result()) for future in done}
        process_rows = getattr(executor, "_processes", None)
        if isinstance(process_rows, dict):
            for process in process_rows.values():
                pid = getattr(process, "pid", None)
                if pid is not None:
                    task_pids.add(int(pid))
        worker_pids = sorted(task_pids)
        return {
            "warmed": True,
            "reason": "ok",
            "workers": workers,
            "workerPIDs": worker_pids,
            "terrainPointCount": len(terrain_points or []),
            "elapsed_ms": round((time.perf_counter() - started) * 1000.0, 3),
        }
    except Exception as exc:
        for future in futures:
            future.cancel()
        if executor is not None and callable(_invalidate_persistent_process_pool):
            try:
                _invalidate_persistent_process_pool(executor)
            except Exception:
                pass
        return {
            "warmed": False,
            "reason": "warm_failed",
            "workers": workers,
            "error": str(exc),
            "elapsed_ms": round((time.perf_counter() - started) * 1000.0, 3),
        }


def shutdown_persistent_0303_process_pools(
    *,
    wait: bool = False,
    cancel_futures: bool = True,
) -> int:
    """Public GUI/process-exit hook; atexit remains the fallback."""

    if not callable(_shutdown_persistent_process_pools):
        return 0
    try:
        return int(
            _shutdown_persistent_process_pools(
                wait=bool(wait),
                cancel_futures=bool(cancel_futures),
            )
            or 0
        )
    except Exception:
        return 0


def _mission_aircraft_id(mission: Any) -> Optional[int]:
    if not isinstance(mission, dict):
        return None
    return _safe_int(mission.get("aircraftID"))


def _build_aircraft_group_in_process(
    module_name: str,
    aid: int,
    group: List[Dict[str, Any]],
    runtime_payload: Optional[Dict[str, Any]],
    cruise_speed: float,
    turn_step_deg: float,
    ref0203: Optional[Dict[str, Any]],
    planning_env: Optional[Dict[str, str]] = None,
    db_root: Optional[str] = None,
    runtime_state: Optional[Dict[str, Any]] = None,
) -> tuple[int, List[Dict[str, Any]], float, Dict[str, Any], List[Dict[str, Any]]]:
    worker_start = time.perf_counter()
    _apply_planning_process_context(planning_env, db_root)
    try:
        d0303_module = importlib.import_module(str(module_name or CANONICAL_D0303_MODULE))
    except Exception:
        d0303_module = importlib.import_module(CANONICAL_D0303_MODULE)
    _apply_d0303_runtime_state(d0303_module, runtime_state)
    # This cache is intentionally process-wide in the ordinary short-lived
    # worker model.  A persistent worker must re-resolve it from this task's
    # runtime override/environment.
    try:
        setattr(d0303_module, "_RECOMPUTE_LINE_SEARCH_SPEED_CACHE", None)
    except Exception:
        pass
    allocator_cls = getattr(d0303_module, "_WPAllocator")
    local_alloc = allocator_cls(start=1)
    _reset_dense_linesearch_metrics(d0303_module)
    with _runtime_context(runtime_payload), _suppress_linesearch_inner_parallel_context(d0303_module):
        plans = d0303_module.build_flight_plans(
            list(group or []),
            local_alloc,
            float(cruise_speed),
            turn_step_deg=float(turn_step_deg),
            ref0203=ref0203,
        )
    return (
        int(aid),
        list(plans or []),
        (time.perf_counter() - worker_start) * 1000.0,
        _get_dense_linesearch_metrics(d0303_module),
        _get_mission_plan_timings(d0303_module),
    )


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


def _get_mission_plan_timings(d0303_module: Any) -> List[Dict[str, Any]]:
    getter = getattr(d0303_module, "get_last_mission_plan_timings", None)
    if not callable(getter):
        return []
    try:
        value = getter(reset=True)
    except TypeError:
        try:
            value = getter()
        except Exception:
            return []
    except Exception:
        return []
    if not isinstance(value, list):
        return []
    return [dict(row) for row in value if isinstance(row, dict)]


def _merge_dense_linesearch_metrics(rows: List[Dict[str, Any]]) -> Dict[str, Any]:
    merged: Dict[str, Any] = {}
    for row in rows or []:
        if not isinstance(row, dict):
            continue
        for key, value in row.items():
            if key in _DENSE_LINESEARCH_MAX_METRIC_KEYS:
                merged[key] = max(int(merged.get(key, 0) or 0), int(value or 0))
            elif key in _DENSE_LINESEARCH_REASON_METRIC_KEYS:
                text = str(value or "").strip()
                if not text:
                    continue
                existing = str(merged.get(key) or "").strip()
                if not existing:
                    merged[key] = text
                elif text not in set(existing.split(",")):
                    merged[key] = existing + "," + text
            elif key in _DENSE_LINESEARCH_FLOAT_METRIC_KEYS:
                merged[key] = round(float(merged.get(key, 0.0) or 0.0) + float(value or 0.0), 3)
            else:
                try:
                    merged[key] = int(merged.get(key, 0) or 0) + int(value or 0)
                except Exception:
                    pass
    return merged


def _summarize_line_search_counts(plans: List[Dict[str, Any]]) -> Dict[str, int]:
    perf_start = replan_perf.start_timer()
    total_coords = 0
    max_coords = 0
    line_search_count = 0
    line_search_json_bytes = 0
    path_count = 0
    json_serialize_ms = 0.0
    json_dumps = json.dumps
    compact_separators = (",", ":")
    measure_json = replan_perf.is_enabled()
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
                json_started = time.perf_counter() if measure_json else None
                line_search_json_bytes += len(
                    json_dumps(line_search, ensure_ascii=False, separators=compact_separators).encode("utf-8")
                )
                if json_started is not None:
                    json_serialize_ms += (time.perf_counter() - json_started) * 1000.0
            except Exception:
                pass
    replan_perf.add_elapsed(
        "mission_planning.0303.line_search_summary",
        perf_start,
        paths=path_count,
        line_search_count=line_search_count,
        line_search_coord_count=total_coords,
        line_search_json_bytes=line_search_json_bytes,
    )
    replan_perf.add(
        "mission_planning.0303.line_search_summary.json_serialize",
        elapsed_ms=json_serialize_ms,
        line_search_count=line_search_count,
        line_search_json_bytes=line_search_json_bytes,
    )
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
        mission_timings = _get_mission_plan_timings(d0303_module)
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
            "mission_timings": mission_timings,
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
            mission_timings: List[Dict[str, Any]],
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
                "mission_timings": list(mission_timings or []),
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
                mission_timings=_get_mission_plan_timings(d0303_module),
                reassigned=0,
                waypoint_count_prepass=0,
            )

        def _build_one_group(group: Dict[str, Any]) -> tuple[str, List[Dict[str, Any]], float, Dict[str, Any], List[Dict[str, Any]]]:
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
                _get_mission_plan_timings(d0303_module),
            )

        group_worker_ms: Dict[str, float] = {}
        dense_metrics_by_group: Dict[str, Dict[str, Any]] = {}
        mission_timings: List[Dict[str, Any]] = []
        try:
            built: List[Dict[str, Any]] = []
            phase_started_dep = time.perf_counter()
            with concurrent.futures.ThreadPoolExecutor(
                max_workers=workers,
                thread_name_prefix="Build0303Dependency",
            ) as executor:
                futures = [executor.submit(_build_one_group, group) for group in groups]
                for future in concurrent.futures.as_completed(futures):
                    group_id, plans, worker_ms, dense_metrics, timing_rows = future.result()
                    group_worker_ms[group_id] = float(worker_ms)
                    dense_metrics_by_group[group_id] = dict(dense_metrics or {})
                    mission_timings.extend(dict(row) for row in (timing_rows or []) if isinstance(row, dict))
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
            mission_timings=mission_timings,
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

    process_parallel_enabled = _setting_flag(
        "replan_0303_aircraft_process_parallel_enabled",
        False,
        runtime_payload=runtime_payload,
        env=env,
        env_names=[
            "REPLAN_0303_AIRCRAFT_PROCESS_PARALLEL",
            "MISSION_PLAN_REPLAN_0303_AIRCRAFT_PROCESS_PARALLEL_ENABLED",
        ],
    )
    if process_parallel_enabled:
        process_requested_workers = max(
            1,
            _setting_int(
                "replan_0303_aircraft_process_workers",
                workers,
                runtime_payload=runtime_payload,
                env=env,
                env_names=[
                    "REPLAN_0303_AIRCRAFT_PROCESS_WORKERS",
                    "MISSION_PLAN_REPLAN_0303_AIRCRAFT_PROCESS_WORKERS",
                ],
            ),
        )
        process_workers = max(1, min(len(grouped), process_requested_workers))
        if process_workers >= 2:
            process_worker_policy = summarize_aircraft_parallel_worker_policy(
                aircraft_count=len(grouped),
                requested_workers=process_requested_workers,
                effective_workers=process_workers,
            )
            worker_ms_by_aircraft_proc: Dict[int, float] = {}
            dense_metrics_by_aircraft_proc: Dict[int, Dict[str, Any]] = {}
            mission_timings_proc: List[Dict[str, Any]] = []
            module_name = CANONICAL_D0303_MODULE
            process_futures: List[concurrent.futures.Future] = []
            persistent_executor: Any = None
            persistent_enabled = _setting_flag(
                "replan_0303_persistent_process_pool_enabled",
                True,
                runtime_payload=runtime_payload,
                env=env,
                env_names=[
                    "REPLAN_0303_PERSISTENT_PROCESS_POOL",
                    "MISSION_PLAN_REPLAN_0303_PERSISTENT_PROCESS_POOL_ENABLED",
                ],
            ) and callable(_acquire_persistent_process_pool)
            try:
                built_proc: List[Dict[str, Any]] = []
                phase_started = time.perf_counter()
                planning_env, process_db_root = _planning_process_context_snapshot()
                process_runtime_state = _snapshot_d0303_runtime_state(d0303_module)
                if persistent_enabled:
                    persistent_executor = _acquire_persistent_process_pool(
                        family=_PROCESS_POOL_FAMILY,
                        generation=_PROCESS_POOL_GENERATION,
                        max_workers=process_workers,
                        initializer=_initialize_0303_process_worker,
                        initargs=(planning_env, process_db_root),
                    )
                executor_context = (
                    nullcontext(persistent_executor)
                    if persistent_executor is not None
                    else concurrent.futures.ProcessPoolExecutor(max_workers=process_workers)
                )
                with executor_context as executor:
                    process_futures = [
                        executor.submit(
                            _build_aircraft_group_in_process,
                            module_name,
                            aid,
                            grouped[aid],
                            runtime_payload,
                            float(cruise_speed),
                            float(turn_step_deg),
                            ref0203,
                            planning_env,
                            process_db_root,
                            process_runtime_state,
                        )
                        for aid in sorted(grouped)
                    ]
                    for future in concurrent.futures.as_completed(process_futures):
                        aid, plans, worker_ms, dense_metrics, timing_rows = future.result()
                        worker_ms_by_aircraft_proc[int(aid)] = float(worker_ms)
                        dense_metrics_by_aircraft_proc[int(aid)] = dict(dense_metrics or {})
                        mission_timings_proc.extend(
                            dict(row) for row in (timing_rows or []) if isinstance(row, dict)
                        )
                        built_proc.extend(plans)
                _phase("per_aircraft_process_build", phase_started)
                phase_started = time.perf_counter()
                built_proc = _sort_plans_for_parallel_merge(built_proc, clean_missions)
                _phase("process_sort_by_input_order", phase_started)
                phase_started = time.perf_counter()
                _normalize_timestamps(built_proc)
                _phase("process_normalize_timestamps", phase_started)
                phase_started = time.perf_counter()
                reassigned_proc = reassign_waypoint_ids_inplace(built_proc, wp_alloc)
                _phase("process_reassign_waypoint_ids", phase_started)
                merge_validation = validate_0303_parallel_merge_output(built_proc)
                if bool(merge_validation.get("valid")):
                    return {
                        "plans": built_proc,
                        "elapsed_ms": (time.perf_counter() - start) * 1000.0,
                        "mode": "aircraft_process_parallel",
                        "workers": int(process_workers),
                        "aircraft": len(grouped),
                        "fallback_reasons": [],
                        "worker_ms_by_aircraft": dict(sorted(worker_ms_by_aircraft_proc.items())),
                        "reassigned_waypoints": int(reassigned_proc),
                        "waypoint_count_prepass": int(reassigned_proc),
                        "worker_policy": process_worker_policy,
                        "phase_ms": dict(phase_ms),
                        "dense_linesearch_metrics": _merge_dense_linesearch_metrics(
                            list(dense_metrics_by_aircraft_proc.values())
                        ),
                        "dense_linesearch_metrics_by_aircraft": dict(
                            sorted(dense_metrics_by_aircraft_proc.items())
                        ),
                        "mission_timings": mission_timings_proc,
                        "line_search_counts": _summarize_line_search_counts(built_proc),
                    }
                phase_ms["process_parallel_invalid_output"] = 1.0
            except Exception as exc:
                if persistent_executor is not None:
                    # Match the old context-manager behavior: do not overlap a
                    # local retry with unfinished work from this same request.
                    for future in process_futures:
                        future.cancel()
                    if process_futures:
                        try:
                            concurrent.futures.wait(process_futures)
                        except Exception:
                            pass
                    if callable(_invalidate_persistent_process_pool):
                        try:
                            _invalidate_persistent_process_pool(persistent_executor)
                        except Exception:
                            pass
                phase_ms["process_parallel_failed"] = 1.0
                phase_ms["process_parallel_error_len"] = float(len(str(exc)))

    def _build_one(aid: int, group: List[Dict[str, Any]]) -> tuple[int, List[Dict[str, Any]], float, Dict[str, Any], List[Dict[str, Any]]]:
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
        return (
            int(aid),
            list(plans or []),
            (time.perf_counter() - worker_start) * 1000.0,
            _get_dense_linesearch_metrics(d0303_module),
            _get_mission_plan_timings(d0303_module),
        )

    worker_ms_by_aircraft: Dict[int, float] = {}
    dense_metrics_by_aircraft: Dict[int, Dict[str, Any]] = {}
    mission_timings: List[Dict[str, Any]] = []
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
                aid, plans, worker_ms, dense_metrics, timing_rows = future.result()
                worker_ms_by_aircraft[int(aid)] = float(worker_ms)
                dense_metrics_by_aircraft[int(aid)] = dict(dense_metrics or {})
                mission_timings.extend(dict(row) for row in (timing_rows or []) if isinstance(row, dict))
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
        "mission_timings": mission_timings,
        "line_search_counts": _summarize_line_search_counts(built),
    }
