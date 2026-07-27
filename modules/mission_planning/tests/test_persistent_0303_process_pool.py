from __future__ import annotations

import concurrent.futures
from types import SimpleNamespace

import pytest

from modules.mission_planning.runtime import aircraft_parallel_0303 as parallel
from modules.mission_planning.runtime import persistent_process_pool as registry


class _FakeExecutor:
    def __init__(self, max_workers: int = 1, **_kwargs):
        self.max_workers = int(max_workers)
        self.shutdown_calls: list[tuple[bool, bool]] = []

    def submit(self, fn, *args, **kwargs):
        future: concurrent.futures.Future = concurrent.futures.Future()
        try:
            future.set_result(fn(*args, **kwargs))
        except BaseException as exc:  # pragma: no cover - mirrors Executor behavior
            future.set_exception(exc)
        return future

    def shutdown(self, wait: bool = True, *, cancel_futures: bool = False):
        self.shutdown_calls.append((bool(wait), bool(cancel_futures)))

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.shutdown(wait=True, cancel_futures=False)
        return False


@pytest.fixture(autouse=True)
def _clear_registry():
    registry.shutdown_process_pools(wait=False, cancel_futures=True)
    yield
    registry.shutdown_process_pools(wait=False, cancel_futures=True)


def test_registry_reuses_generation_and_retires_hot_reload(monkeypatch):
    created: list[_FakeExecutor] = []

    def _factory(max_workers: int, **_kwargs):
        executor = _FakeExecutor(max_workers=max_workers)
        created.append(executor)
        return executor

    monkeypatch.setattr(registry.concurrent.futures, "ProcessPoolExecutor", _factory)

    first = registry.acquire_process_pool(family="0303", generation="gen-1", max_workers=3)
    assert registry.acquire_process_pool(
        family="0303", generation="gen-1", max_workers=3
    ) is first
    second = registry.acquire_process_pool(family="0303", generation="gen-2", max_workers=3)

    assert second is not first
    assert len(created) == 2
    assert created[0].shutdown_calls == [(False, False)]
    assert registry.process_pool_registry_size() == 1


def test_registry_invalidates_only_exact_failed_pool(monkeypatch):
    monkeypatch.setattr(registry.concurrent.futures, "ProcessPoolExecutor", _FakeExecutor)
    failed = registry.acquire_process_pool(family="0303", generation="gen", max_workers=2)
    other = registry.acquire_process_pool(family="0303", generation="gen", max_workers=3)

    assert registry.invalidate_process_pool(failed) is True
    assert registry.invalidate_process_pool(failed) is False
    assert failed.shutdown_calls == [(False, True)]
    assert other.shutdown_calls == []
    assert registry.process_pool_registry_size() == 1


def test_worker_context_replaces_only_planning_environment(monkeypatch):
    monkeypatch.setenv("MISSION_PLAN_OLD_VALUE", "stale")
    monkeypatch.setenv("UNRELATED_PROCESS_VALUE", "keep")

    parallel._apply_planning_process_context(
        {
            "MISSION_PLAN_NEW_VALUE": "fresh",
            "KU_ROLE": "planner",
            "UNRELATED_PROCESS_VALUE": "replace-attempt",
        },
        None,
    )

    assert "MISSION_PLAN_OLD_VALUE" not in parallel.os.environ
    assert parallel.os.environ["MISSION_PLAN_NEW_VALUE"] == "fresh"
    assert parallel.os.environ["KU_ROLE"] == "planner"
    assert parallel.os.environ["UNRELATED_PROCESS_VALUE"] == "keep"


class _Allocator:
    def __init__(self, start: int = 1, **_kwargs):
        self.next_id = int(start)

    def alloc(self) -> int:
        value = self.next_id
        self.next_id += 1
        return value


def _worker_stub(
    _module_name,
    aid,
    group,
    _runtime_payload,
    _cruise_speed,
    _turn_step_deg,
    _ref0203,
    _planning_env=None,
    _db_root=None,
    _runtime_state=None,
):
    mission = group[0]
    return (
        int(aid),
        [
            {
                "aircraftID": int(aid),
                "pathID": int(mission["pathID"]),
                "timestamp": 1234,
                "waypointList": [{"waypointID": 1, "nextWaypointID": 0}],
            }
        ],
        1.0,
        {},
        [],
    )


def _build_with_persistent_setting(enabled: bool):
    runtime_payload = {
        "values": {
            "replan_0303_aircraft_workers": 2,
            "replan_0303_aircraft_process_parallel_enabled": True,
            "replan_0303_aircraft_process_workers": 2,
            "replan_0303_persistent_process_pool_enabled": bool(enabled),
        }
    }
    fake_d0303 = SimpleNamespace(
        _WPAllocator=_Allocator,
        build_flight_plans=lambda *_args, **_kwargs: [],
    )
    return parallel.build_0303_flight_plans_aircraft_parallel(
        fake_d0303,
        [
            {"aircraftID": 11, "pathID": 101},
            {"aircraftID": 12, "pathID": 102},
        ],
        runtime_payload=runtime_payload,
        wp_alloc=_Allocator(start=900),
        cruise_speed=40.0,
        turn_step_deg=15.0,
        ref0203=None,
        env={},
    )


def test_persistent_and_ephemeral_process_paths_keep_plan_and_ids(monkeypatch):
    ephemeral_instances: list[_FakeExecutor] = []
    persistent = _FakeExecutor(max_workers=2)

    def _ephemeral_factory(max_workers: int):
        executor = _FakeExecutor(max_workers=max_workers)
        ephemeral_instances.append(executor)
        return executor

    monkeypatch.setattr(parallel.concurrent.futures, "ProcessPoolExecutor", _ephemeral_factory)
    monkeypatch.setattr(parallel, "_build_aircraft_group_in_process", _worker_stub)
    monkeypatch.setattr(parallel, "_planning_process_context_snapshot", lambda: ({}, None))
    monkeypatch.setattr(
        parallel,
        "_acquire_persistent_process_pool",
        lambda **_kwargs: persistent,
    )

    ephemeral_result = _build_with_persistent_setting(False)
    persistent_result = _build_with_persistent_setting(True)

    assert ephemeral_result["mode"] == persistent_result["mode"] == "aircraft_process_parallel"
    assert ephemeral_result["plans"] == persistent_result["plans"]
    assert ephemeral_result["reassigned_waypoints"] == persistent_result["reassigned_waypoints"] == 2
    assert [wp["waypointID"] for plan in persistent_result["plans"] for wp in plan["waypointList"]] == [900, 901]
    assert ephemeral_instances[0].shutdown_calls == [(True, False)]
    assert persistent.shutdown_calls == []


def test_worker_runtime_state_copies_parent_planning_globals(monkeypatch):
    class _SweepConfig:
        def __init__(self, *, separation_m, fov_deg):
            self.separation_m = float(separation_m)
            self.fov_deg = float(fov_deg)

    parent = SimpleNamespace(
        LINE_SEARCH_SPEED_WEIGHT=1.0,
        FOV_DEG=2.4,
        ALTITUDE_LAYERS_M=(1000.0, 1010.0, 1020.0),
        FLYOVER_LAST_POINT=True,
        SWEEP_GEOMETRY=_SweepConfig(separation_m=875.0, fov_deg=2.4),
    )
    child = SimpleNamespace(
        LINE_SEARCH_SPEED_WEIGHT=1.1,
        FOV_DEG=9.9,
        ALTITUDE_LAYERS_M=(1.0,),
        FLYOVER_LAST_POINT=False,
        SWEEP_GEOMETRY=_SweepConfig(separation_m=1000.0, fov_deg=9.9),
        SweepConfig=_SweepConfig,
    )
    search_speed = SimpleNamespace(_CFG_WEIGHT=1.1)
    config = SimpleNamespace(
        DEFAULT_SWEEP_SEPARATION_M=1000.0,
        SEARCH_SPEED_WEIGHT=1.1,
        DB_FOV_WEIGHT=1.0,
    )

    modules = {
        parallel._SEARCH_SPEED_MODULE_NAME: search_speed,
        parallel._MP_CONFIG_MODULE_NAME: config,
    }
    monkeypatch.setattr(parallel.importlib, "import_module", lambda name: modules[name])

    state = parallel._snapshot_d0303_runtime_state(parent)
    state["companions"][parallel._SEARCH_SPEED_MODULE_NAME]["_CFG_WEIGHT"] = 1.0
    state["companions"][parallel._MP_CONFIG_MODULE_NAME]["SEARCH_SPEED_WEIGHT"] = 1.0
    parallel._apply_d0303_runtime_state(child, state)

    assert child.LINE_SEARCH_SPEED_WEIGHT == 1.0
    assert child.FOV_DEG == 2.4
    assert child.ALTITUDE_LAYERS_M == (1000.0, 1010.0, 1020.0)
    assert child.FLYOVER_LAST_POINT is True
    assert child.SWEEP_GEOMETRY.separation_m == 875.0
    assert child.SWEEP_GEOMETRY.fov_deg == 2.4
    assert search_speed._CFG_WEIGHT == 1.0
    assert config.SEARCH_SPEED_WEIGHT == 1.0


def test_reused_worker_refreshes_runtime_globals_before_each_build(monkeypatch):
    child = SimpleNamespace(LINE_SEARCH_SPEED_WEIGHT=1.1)
    search_speed = SimpleNamespace(_CFG_WEIGHT=1.1)
    config = SimpleNamespace(
        DEFAULT_SWEEP_SEPARATION_M=1000.0,
        SEARCH_SPEED_WEIGHT=1.1,
        DB_FOV_WEIGHT=1.0,
    )
    modules = {
        parallel._SEARCH_SPEED_MODULE_NAME: search_speed,
        parallel._MP_CONFIG_MODULE_NAME: config,
    }
    monkeypatch.setattr(parallel.importlib, "import_module", lambda name: modules[name])

    for weight in (1.0, 0.8):
        parallel._apply_d0303_runtime_state(
            child,
            {
                "d0303": {"LINE_SEARCH_SPEED_WEIGHT": weight},
                "companions": {
                    parallel._SEARCH_SPEED_MODULE_NAME: {"_CFG_WEIGHT": weight},
                    parallel._MP_CONFIG_MODULE_NAME: {"SEARCH_SPEED_WEIGHT": weight},
                },
            },
        )
        assert child.LINE_SEARCH_SPEED_WEIGHT == weight
        assert search_speed._CFG_WEIGHT == weight
        assert config.SEARCH_SPEED_WEIGHT == weight


def test_public_warm_helper_forces_lazy_worker_start(monkeypatch):
    persistent = _FakeExecutor(max_workers=3)
    acquire_args = {}

    def _acquire(**kwargs):
        acquire_args.update(kwargs)
        return persistent

    monkeypatch.setattr(parallel, "_acquire_persistent_process_pool", _acquire)
    monkeypatch.setattr(parallel, "_planning_process_context_snapshot", lambda: ({}, None))
    warm_calls = []

    def _warm_worker(points, runtime_payload, planning_env, db_root, runtime_state):
        warm_calls.append((points, runtime_payload, planning_env, db_root, runtime_state))
        return 77

    monkeypatch.setattr(parallel, "_warm_0303_process_worker", _warm_worker)

    result = parallel.warm_persistent_0303_process_pool(
        max_workers=3,
        timeout_s=1.0,
        env={},
        terrain_points=[(37.0, 127.0)],
        runtime_payload={"values": {"dem_alt_cache_round_decimals": 0}},
    )

    assert result["warmed"] is True
    assert result["workerPIDs"] == [77]
    assert result["terrainPointCount"] == 1
    assert len(warm_calls) == 3
    assert all(call[0] == [(37.0, 127.0)] for call in warm_calls)
    assert acquire_args["max_workers"] == 3
    assert acquire_args["initializer"] is parallel._initialize_0303_process_worker
    assert len(acquire_args["initargs"]) == 5
    assert persistent.shutdown_calls == []
