from __future__ import annotations

import threading
import time
from dataclasses import replace
from types import SimpleNamespace

import pytest

from modules.common.terrain_los import (
    ENEMY_OBSERVER_HEIGHT_M,
    LAH_UAV_COMMUNICATION_RANGE_M,
    LOS_CLEARANCE_M,
    LOS_SAMPLES_PER_CELL,
)
from modules.monitoring.logic.dem_cover.config import CoverConfig
from modules.sim.runtime.geo import GeoConverter
from modules.sim.runtime import sim_service as sim_service_module
from modules.sim.runtime.sim_service import (
    SimulationService,
    _EnemyLahLosResult,
    _EnemyLahLosWorker,
)


def _vehicle(
    label: str,
    aircraft_id: int,
    airframe: str,
    *,
    x: float,
    y: float,
    z: float,
    alive: bool = True,
) -> SimpleNamespace:
    return SimpleNamespace(
        label=label,
        aircraft_id=aircraft_id,
        airframe=airframe,
        alive=alive,
        vehicle=SimpleNamespace(s=SimpleNamespace(x=x, y=y, z=z)),
    )


def _mark_targets_discovered(sim: SimulationService, *targets) -> None:
    for index, target in enumerate(targets, start=1):
        raw_target_id = int(target.id)
        sim._target_id_map_0402[raw_target_id] = 1000 + index
        sim._target_watcher_0402[raw_target_id] = 4
        sim._discovered_raw_target_ids.add(raw_target_id)


def test_sim_and_cover_planner_share_los_policy() -> None:
    config = CoverConfig()

    assert config.enemy_height_m == pytest.approx(ENEMY_OBSERVER_HEIGHT_M)
    assert config.los_clearance_m == pytest.approx(LOS_CLEARANCE_M)
    assert config.los_samples_per_cell == pytest.approx(LOS_SAMPLES_PER_CELL)
    assert sim_service_module._TERRAIN_LOS_TARGET_HEIGHT_M == pytest.approx(
        config.enemy_height_m
    )
    assert sim_service_module._TERRAIN_LOS_CLEARANCE_M == pytest.approx(
        config.los_clearance_m
    )
    assert sim_service_module._LAH_UAV_COMM_MAX_RANGE_M == pytest.approx(
        LAH_UAV_COMMUNICATION_RANGE_M
    )


def test_sim_wrapper_matches_canonical_planner_on_real_native_dem() -> None:
    dem_path = sim_service_module._CANONICAL_LOS_DEM_DIR / "Inje_10m.tif"
    if not dem_path.is_file():
        pytest.skip("operational Inje DEM is not installed")

    geo = GeoConverter(128.27, 37.85)
    evaluator = SimpleNamespace(geo=geo)
    lah_lat, lah_lon, lah_alt = 37.8500, 128.2500, 900.0
    enemy_lat, enemy_lon, enemy_alt = 37.8600, 128.2900, 0.0
    lah_x, lah_y = geo.lonlat_to_xy(lah_lon, lah_lat)
    enemy_x, enemy_y = geo.lonlat_to_xy(enemy_lon, enemy_lat)

    sim_status = sim_service_module._compute_dem_los_status(
        evaluator,
        sx=lah_x,
        sy=lah_y,
        sz=lah_alt,
        tx=enemy_x,
        ty=enemy_y,
        tz=enemy_alt,
        clearance_m=LOS_CLEARANCE_M,
        sample_step_m=10.0,
        target_height_m=ENEMY_OBSERVER_HEIGHT_M,
    )
    planner_status = sim_service_module.evaluate_regional_los(
        resource_dir=sim_service_module._CANONICAL_LOS_DEM_DIR,
        observer_latitude=enemy_lat,
        observer_longitude=enemy_lon,
        observer_altitude_m=enemy_alt,
        observer_height_m=ENEMY_OBSERVER_HEIGHT_M,
        target_latitude=lah_lat,
        target_longitude=lah_lon,
        target_altitude_m=lah_alt,
        target_height_m=0.0,
        reject_nodata=False,
    )

    assert sim_status["visible"] is planner_status["visible"]
    assert sim_status["reason"] == planner_status["reason"]
    assert sim_status["demPath"] == planner_status["demPath"]
    assert sim_status["targetRayAlt"] == pytest.approx(
        planner_status["observerRayAlt"]
    )


def test_enemy_lah_los_uses_only_discovered_live_targets() -> None:
    sim = SimulationService()
    sim.geo = GeoConverter(127.0, 38.0)
    lah = _vehicle("LAH1", 1, "lah", x=0.0, y=0.0, z=300.0)
    discovered = sim._build_target(
        type_id=1,
        x=1_000.0,
        y=0.0,
        z=0.0,
        id_override=11,
    )
    undiscovered = sim._build_target(
        type_id=2,
        x=500.0,
        y=0.0,
        z=0.0,
        id_override=12,
    )
    destroyed = sim._build_target(
        type_id=3,
        x=250.0,
        y=0.0,
        z=0.0,
        id_override=13,
    )
    destroyed.alive = False
    sim._target_id_map_0402[int(discovered.id)] = 1001
    sim._target_id_map_0402[int(undiscovered.id)] = 1002
    sim._target_watcher_0402[int(discovered.id)] = 4
    sim._target_watcher_0402[int(undiscovered.id)] = 4
    sim._target_id_map_0402["invalid"] = 9999
    sim._emit_0402(
        body={
            "targetList": [
                {
                    "targetID": 1001,
                    "targetInFrame": 1,
                    "isDestroyed": 0,
                }
            ]
        },
        aircraft_id=4,
        target_id=1001,
    )
    _mark_targets_discovered(sim, destroyed)

    job = sim._capture_enemy_lah_los_job_locked(
        geo=sim.geo,
        vehicles=[lah],
        targets=[discovered, undiscovered, destroyed],
        step_count=1,
    )

    assert [pair.target_id for pair in job.pairs] == [11]
    sim.shutdown()


def test_enemy_lah_los_links_cover_live_pairs_and_cache_dem_result(monkeypatch) -> None:
    sim = SimulationService()
    sim.geo = GeoConverter(127.0, 38.0)
    lah1 = _vehicle("LAH1", 1, "lah", x=0.0, y=0.0, z=300.0)
    lah2 = _vehicle("LAH2", 2, "lah", x=100.0, y=0.0, z=320.0)
    dead_lah = _vehicle("LAH3", 3, "lah", x=200.0, y=0.0, z=330.0, alive=False)
    uav = _vehicle("UAV1", 4, "uav", x=20.0, y=0.0, z=1000.0)
    target1 = sim._build_target(type_id=1, x=1000.0, y=0.0, z=0.0, id_override=11)
    target2 = sim._build_target(type_id=2, x=1500.0, y=100.0, z=0.0, id_override=12)
    dead_target = sim._build_target(type_id=3, x=2000.0, y=0.0, z=0.0, id_override=13)
    dead_target.alive = False
    _mark_targets_discovered(sim, target1, target2, dead_target)

    calls: list[tuple[str, int]] = []

    # The first snapshot must be observed before the worker publishes, so the
    # worker is held until the pending assertion has run.
    release = threading.Event()

    def compute(job):
        release.wait(timeout=5.0)
        statuses = {}
        for pair in job.pairs:
            calls.append(pair.key)
            statuses[pair.key] = {
                "visible": pair.source_x < 50.0,
                "demAvailable": True,
                "demSources": ["Inje_10m.tif"],
                "targetRayAlt": 83.0,
            }
        return _EnemyLahLosResult(
            generation=job.generation,
            sequence=job.sequence,
            step=job.step,
            pair_keys=job.pair_keys,
            statuses=statuses,
            completed_wall=time.monotonic(),
        )

    sim._enemy_lah_los_worker.close()
    sim._enemy_lah_los_worker = _EnemyLahLosWorker(
        refresh_sec=60.0,
        compute=compute,
    )
    vehicles = [lah1, lah2, dead_lah, uav]
    targets = [target1, target2, dead_target]

    pending_links = sim._build_enemy_lah_los_links(
        geo=sim.geo,
        vehicles=vehicles,
        targets=targets,
    )
    assert len(pending_links) == 4
    assert all(link["visible"] is None for link in pending_links)
    release.set()
    assert sim._enemy_lah_los_worker.wait_for_result(
        sim._enemy_lah_los_generation,
    ) is not None
    links = sim._build_enemy_lah_los_links(
        geo=sim.geo,
        vehicles=vehicles,
        targets=targets,
    )

    assert len(links) == 4
    assert {link["id"] for link in links} == {
        "LAH1:11",
        "LAH2:11",
        "LAH1:12",
        "LAH2:12",
    }
    assert len(calls) == 4
    assert all(link["to"]["alt"] == pytest.approx(83.0) for link in links)
    assert all(link["demAvailable"] is True for link in links)
    assert all(link["visible"] is True for link in links if link["aircraft"] == "LAH1")
    assert all(link["terrainBlocked"] is True for link in links if link["aircraft"] == "LAH2")

    # Endpoint positions still update on every snapshot, while the native DEM
    # result remains cached inside the 0.8 second overlay refresh interval.
    old_lon = next(link for link in links if link["id"] == "LAH1:11")["from"]["lon"]
    lah1.vehicle.s.x = 25.0
    cached_links = sim._build_enemy_lah_los_links(
        geo=sim.geo,
        vehicles=vehicles,
        targets=targets,
    )
    new_lon = next(link for link in cached_links if link["id"] == "LAH1:11")["from"]["lon"]
    assert new_lon != old_lon
    assert len(calls) == 4
    sim.shutdown()


def test_enemy_lah_los_reports_unknown_without_complete_dem_coverage(monkeypatch) -> None:
    sim = SimulationService()
    sim.geo = GeoConverter(127.0, 38.0)
    lah = _vehicle("LAH1", 1, "lah", x=0.0, y=0.0, z=300.0)
    target = sim._build_target(type_id=1, x=1000.0, y=0.0, z=0.0)
    _mark_targets_discovered(sim, target)

    class MissingDemEvaluator:
        geo = sim.geo

        @staticmethod
        def _terrain_source_name(_lat, _lon):
            return None

        @staticmethod
        def _ground_height(_x, _y):
            return 0.0

        @staticmethod
        def _check_los_terrain(*_args):
            raise AssertionError("LOS must stay unknown")

    monkeypatch.setattr(
        sim_service_module,
        "_make_enemy_lah_los_evaluator",
        lambda _job: MissingDemEvaluator(),
    )

    links = sim._build_enemy_lah_los_links(
        geo=sim.geo,
        vehicles=[lah],
        targets=[target],
    )

    assert len(links) == 1
    assert sim._enemy_lah_los_worker.wait_for_result(
        sim._enemy_lah_los_generation,
    ) is not None
    links = sim._build_enemy_lah_los_links(
        geo=sim.geo,
        vehicles=[lah],
        targets=[target],
    )
    assert links[0]["demAvailable"] is False
    assert links[0]["visible"] is None
    assert links[0]["terrainBlocked"] is None
    sim.shutdown()


def test_enemy_lah_los_does_not_block_snapshot_on_slow_dem() -> None:
    sim = SimulationService()
    sim.geo = GeoConverter(127.0, 38.0)
    lah = _vehicle("LAH1", 1, "lah", x=0.0, y=0.0, z=300.0)
    target = sim._build_target(type_id=1, x=10_000.0, y=0.0, z=0.0)
    _mark_targets_discovered(sim, target)
    started = threading.Event()
    release = threading.Event()

    def slow_compute(job):
        started.set()
        release.wait(timeout=2.0)
        return _EnemyLahLosResult(
            generation=job.generation,
            sequence=job.sequence,
            step=job.step,
            pair_keys=job.pair_keys,
            statuses={},
            completed_wall=time.monotonic(),
        )

    sim._enemy_lah_los_worker.close()
    sim._enemy_lah_los_worker = _EnemyLahLosWorker(
        refresh_sec=60.0,
        compute=slow_compute,
    )
    try:
        started_at = time.perf_counter()
        links = sim._build_enemy_lah_los_links(
            geo=sim.geo,
            vehicles=[lah],
            targets=[target],
        )
        elapsed = time.perf_counter() - started_at
        assert links
        assert all(link["visible"] is None for link in links)
        assert elapsed < 0.2
        assert started.wait(timeout=1.0)
    finally:
        release.set()
        sim.shutdown()


def test_enemy_lah_los_keeps_all_discovered_targets_within_shared_cap() -> None:
    sim = SimulationService()
    sim.geo = GeoConverter(127.0, 38.0)
    lah1 = _vehicle("LAH1", 1, "lah", x=0.0, y=0.0, z=300.0)
    lah2 = _vehicle("LAH2", 2, "lah", x=10_000.0, y=0.0, z=300.0)
    targets = [
        sim._build_target(type_id=1, x=x, y=0.0, z=0.0, id_override=target_id)
        for target_id, x in ((11, 500.0), (12, 1_000.0), (13, 2_000.0), (14, 8_000.0), (15, 9_500.0))
    ]
    _mark_targets_discovered(sim, *targets)

    links = sim._build_enemy_lah_los_links(
        geo=sim.geo,
        vehicles=[lah1, lah2],
        targets=targets,
    )

    assert len(links) == 10
    assert {
        link["targetID"] for link in links if link["aircraft"] == "LAH1"
    } == {11, 12, 13, 14, 15}
    assert {
        link["targetID"] for link in links if link["aircraft"] == "LAH2"
    } == {11, 12, 13, 14, 15}
    sim.shutdown()


def test_enemy_lah_los_worker_single_flights_concurrent_polling() -> None:
    sim = SimulationService()
    sim.geo = GeoConverter(127.0, 38.0)
    lah = _vehicle("LAH1", 1, "lah", x=0.0, y=0.0, z=300.0)
    target = sim._build_target(type_id=1, x=1_000.0, y=0.0, z=0.0)
    _mark_targets_discovered(sim, target)
    release = threading.Event()
    started = threading.Event()
    call_lock = threading.Lock()
    call_count = 0

    def slow_compute(job):
        nonlocal call_count
        with call_lock:
            call_count += 1
        started.set()
        release.wait(timeout=2.0)
        return _EnemyLahLosResult(
            generation=job.generation,
            sequence=job.sequence,
            step=job.step,
            pair_keys=job.pair_keys,
            statuses={},
            completed_wall=time.monotonic(),
        )

    sim._enemy_lah_los_worker.close()
    sim._enemy_lah_los_worker = _EnemyLahLosWorker(
        refresh_sec=60.0,
        compute=slow_compute,
    )
    errors: list[BaseException] = []

    def poll() -> None:
        try:
            sim._build_enemy_lah_los_links(
                geo=sim.geo,
                vehicles=[lah],
                targets=[target],
            )
        except BaseException as exc:  # pragma: no cover - diagnostic capture
            errors.append(exc)

    threads = [threading.Thread(target=poll) for _ in range(8)]
    try:
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=1.0)
        assert started.wait(timeout=1.0)
        assert not errors
        assert call_count == 1
    finally:
        release.set()
        sim.shutdown()


def test_enemy_lah_los_discards_result_from_previous_generation() -> None:
    sim = SimulationService()
    sim.geo = GeoConverter(127.0, 38.0)
    lah = _vehicle("LAH1", 1, "lah", x=0.0, y=0.0, z=300.0)
    target = sim._build_target(type_id=1, x=1_000.0, y=0.0, z=0.0)
    _mark_targets_discovered(sim, target)
    first_started = threading.Event()
    release_first = threading.Event()
    first_completed = threading.Event()
    call_count = 0

    def compute(job):
        nonlocal call_count
        call_count += 1
        if call_count == 1:
            first_started.set()
            release_first.wait(timeout=2.0)
            first_completed.set()
        return _EnemyLahLosResult(
            generation=job.generation,
            sequence=job.sequence,
            step=job.step,
            pair_keys=job.pair_keys,
            statuses={
                pair.key: {
                    "visible": True,
                    "demAvailable": True,
                    "demSources": ["Inje_10m.tif"],
                    "targetRayAlt": 3.0,
                }
                for pair in job.pairs
            },
            completed_wall=time.monotonic(),
        )

    sim._enemy_lah_los_worker.close()
    sim._enemy_lah_los_worker = _EnemyLahLosWorker(
        refresh_sec=60.0,
        compute=compute,
    )
    try:
        sim._build_enemy_lah_los_links(
            geo=sim.geo,
            vehicles=[lah],
            targets=[target],
        )
        assert first_started.wait(timeout=1.0)
        with sim._lock:
            sim._invalidate_enemy_lah_los_locked()
            current_generation = sim._enemy_lah_los_generation
        release_first.set()
        assert first_completed.wait(timeout=1.0)
        assert sim._enemy_lah_los_worker.latest(current_generation) is None

        sim._build_enemy_lah_los_links(
            geo=sim.geo,
            vehicles=[lah],
            targets=[target],
        )
        result = sim._enemy_lah_los_worker.wait_for_result(
            current_generation,
            timeout=1.0,
        )
        assert result is not None
        assert result.generation == current_generation
        assert call_count == 2
    finally:
        release_first.set()
        sim.shutdown()


def test_lah_uav_communication_capture_uses_fixed_live_pairs_and_current_uav_altitude() -> None:
    sim = SimulationService()
    sim.geo = GeoConverter(127.0, 38.0)
    lah1 = _vehicle("LAH1", 1, "lah", x=0.0, y=0.0, z=300.0)
    lah2 = _vehicle("LAH2", 2, "lah", x=100.0, y=0.0, z=320.0)
    lah3 = _vehicle("LAH3", 3, "lah", x=200.0, y=0.0, z=340.0)
    uav1 = _vehicle("UAV1", 4, "uav", x=1_000.0, y=0.0, z=1_111.0)
    uav2 = _vehicle("UAV2", 5, "uav", x=2_000.0, y=0.0, z=1_222.0)
    uav3 = _vehicle("UAV3", 6, "uav", x=3_000.0, y=0.0, z=1_333.0)
    unrelated_lah = _vehicle("LAH9", 9, "lah", x=0.0, y=1_000.0, z=350.0)
    unrelated_uav = _vehicle("UAV9", 9, "uav", x=0.0, y=2_000.0, z=1_500.0)
    vehicles = [
        lah1,
        lah2,
        lah3,
        uav1,
        uav2,
        uav3,
        unrelated_lah,
        unrelated_uav,
    ]

    job = sim._capture_enemy_lah_los_job_locked(
        geo=sim.geo,
        vehicles=vehicles,
        targets=[],
        step_count=17,
    )

    assert [
        (pair.manned_aircraft_id, pair.uav_aircraft_id)
        for pair in job.communication_pairs
    ] == [(1, 4), (1, 5), (1, 6)]
    assert [pair.target_z for pair in job.communication_pairs] == pytest.approx(
        [1_111.0, 1_222.0, 1_333.0]
    )

    # Missing and dead endpoints are not exposed as communication links.
    uav2.alive = False
    lah3.alive = False
    job = sim._capture_enemy_lah_los_job_locked(
        geo=sim.geo,
        vehicles=[vehicle for vehicle in vehicles if vehicle is not uav3],
        targets=[],
        step_count=18,
    )
    assert [
        (pair.manned_aircraft_id, pair.uav_aircraft_id)
        for pair in job.communication_pairs
    ] == [(1, 4)]
    sim.shutdown()


def test_lah_uav_communication_uses_canonical_planner_10km_range(
    monkeypatch,
) -> None:
    sim = SimulationService()
    sim.geo = GeoConverter(127.0, 38.0)
    lah1 = _vehicle("LAH1", 1, "lah", x=0.0, y=0.0, z=310.0)
    uav1 = _vehicle("UAV1", 4, "uav", x=10_000.0, y=0.0, z=1_401.0)
    uav2 = _vehicle("UAV2", 5, "uav", x=10_000.01, y=0.0, z=1_502.0)
    job = sim._capture_enemy_lah_los_job_locked(
        geo=sim.geo,
        vehicles=[lah1, uav1, uav2],
        targets=[],
        step_count=27,
    )
    los_calls: list[dict] = []

    def fake_evaluate_regional_los(**kwargs):
        los_calls.append(dict(kwargs))
        within_range = len(los_calls) == 1
        return {
            "visible": bool(within_range),
            "demAvailable": True,
            "demSources": ["Inje_10m.tif"],
            "demPath": "resource/Inje_10m.tif",
            "observerRayAlt": float(kwargs["observer_altitude_m"]),
            "horizontalDistanceM": 10_000.0 if within_range else 10_000.01,
            "policyVersion": "regional-dem-los-v1",
            "reason": "VISIBLE" if within_range else "OUT_OF_RANGE",
        }

    monkeypatch.setattr(
        sim_service_module,
        "evaluate_regional_los",
        fake_evaluate_regional_los,
    )
    # Hold the worker so the pending snapshot cannot race the published one.
    release = threading.Event()

    def gated_compute(job_arg):
        release.wait(timeout=5.0)
        return sim_service_module._compute_enemy_lah_los_job(job_arg)

    sim._enemy_lah_los_worker.close()
    sim._enemy_lah_los_worker = _EnemyLahLosWorker(
        refresh_sec=60.0,
        compute=gated_compute,
    )

    pending = sim._build_lah_uav_communication_links(job=job)
    assert {link["status"] for link in pending} == {"pending", "outOfRange"}
    release.set()
    assert sim._enemy_lah_los_worker.wait_for_result(
        sim._enemy_lah_los_generation,
    ) is not None
    links = sim._build_lah_uav_communication_links(job=job)
    by_id = {link["id"]: link for link in links}

    boundary = by_id["LAH1:UAV1"]
    assert boundary["horizontalDistanceM"] == pytest.approx(10_000.0)
    assert boundary["withinRange"] is True
    assert boundary["status"] == "connected"
    assert boundary["terrainVisible"] is True
    assert boundary["to"]["alt"] == pytest.approx(1_401.0)

    beyond = by_id["LAH1:UAV2"]
    assert beyond["horizontalDistanceM"] == pytest.approx(10_000.0, abs=0.1)
    assert beyond["withinRange"] is False
    assert beyond["status"] == "outOfRange"
    assert beyond["terrainVisible"] is False
    assert beyond["demAvailable"] is True
    assert beyond["to"]["alt"] == pytest.approx(1_502.0)

    # Both pairs reach the canonical UTM/native-DEM API so the same projected
    # distance decides the 10 km boundary in planner and SIM.
    assert len(los_calls) == 2
    kwargs = los_calls[0]
    assert kwargs["observer_altitude_m"] == pytest.approx(1_401.0)
    assert kwargs["target_altitude_m"] == pytest.approx(310.0)
    assert kwargs["observer_height_m"] == pytest.approx(0.0)
    assert kwargs["reject_nodata"] is True
    assert all(
        call["max_range_m"] == pytest.approx(LAH_UAV_COMMUNICATION_RANGE_M)
        for call in los_calls
    )
    sim.shutdown()


def test_lah_uav_communication_statuses_keep_reported_disconnect_priority() -> None:
    sim = SimulationService()
    sim.geo = GeoConverter(127.0, 38.0)
    lah1 = _vehicle("LAH1", 1, "lah", x=0.0, y=0.0, z=300.0)
    uav1 = _vehicle("UAV1", 4, "uav", x=25_000.0, y=0.0, z=1_100.0)
    lah2 = _vehicle("LAH2", 2, "lah", x=0.0, y=1_000.0, z=310.0)
    uav2 = _vehicle("UAV2", 5, "uav", x=2_000.0, y=1_000.0, z=1_200.0)
    lah3 = _vehicle("LAH3", 3, "lah", x=0.0, y=2_000.0, z=320.0)
    uav3 = _vehicle("UAV3", 6, "uav", x=3_000.0, y=2_000.0, z=1_300.0)
    sim._agent_overrides["LAH1"] = {"datalink": {"uav1": False}}
    job = sim._capture_enemy_lah_los_job_locked(
        geo=sim.geo,
        vehicles=[lah1, uav1, lah2, uav2, lah3, uav3],
        targets=[],
        step_count=37,
    )

    def compute(current_job):
        communication_statuses = {}
        for pair in current_job.communication_pairs:
            if pair.uav_aircraft_id == 5:
                communication_statuses[pair.key] = {
                    "visible": False,
                    "demAvailable": True,
                    "demSources": ["Inje_10m.tif"],
                    "targetRayAlt": pair.target_z,
                }
            elif pair.uav_aircraft_id == 6:
                communication_statuses[pair.key] = {
                    "visible": None,
                    "demAvailable": False,
                    "demSources": [],
                    "targetRayAlt": pair.target_z,
                }
        return _EnemyLahLosResult(
            generation=current_job.generation,
            sequence=current_job.sequence,
            step=current_job.step,
            pair_keys=current_job.pair_keys,
            statuses={},
            completed_wall=time.monotonic(),
            communication_statuses=communication_statuses,
        )

    sim._enemy_lah_los_worker.close()
    sim._enemy_lah_los_worker = _EnemyLahLosWorker(
        refresh_sec=60.0,
        compute=compute,
    )
    sim._build_lah_uav_communication_links(job=job)
    assert sim._enemy_lah_los_worker.wait_for_result(
        sim._enemy_lah_los_generation,
    ) is not None
    links = sim._build_lah_uav_communication_links(job=job)
    by_id = {link["id"]: link for link in links}

    manual_down = by_id["LAH1:UAV1"]
    assert manual_down["reportedConnected"] is False
    assert manual_down["withinRange"] is False
    assert manual_down["status"] == "reportedDisconnected"
    assert manual_down["communicationAvailable"] is False

    blocked = by_id["LAH1:UAV2"]
    assert blocked["reportedConnected"] is True
    assert blocked["terrainVisible"] is False
    assert blocked["terrainBlocked"] is True
    assert blocked["physicalAvailable"] is False
    assert blocked["status"] == "terrainBlocked"

    unknown = by_id["LAH1:UAV3"]
    assert unknown["reportedConnected"] is True
    assert unknown["terrainVisible"] is None
    assert unknown["terrainBlocked"] is None
    assert unknown["demAvailable"] is False
    assert unknown["physicalAvailable"] is None
    assert unknown["communicationAvailable"] is None
    assert unknown["status"] == "demUnknown"
    sim.shutdown()


def test_vertical_air_to_air_los_uses_endpoint_clearance_not_altitude_order(
    monkeypatch,
) -> None:
    sim = SimulationService()
    sim.geo = None
    monkeypatch.setattr(
        sim,
        "_build_footprint_terrain_context",
        lambda _x, _y: (lambda _px, _py: 100.0, 1.0),
    )

    # LAH is directly below the UAV.  This is a clear vertical air-to-air ray,
    # even though the source altitude is lower than the target altitude.
    assert sim._check_los_terrain(
        0.0,
        0.0,
        300.0,
        0.0,
        0.0,
        1_000.0,
        clearance_m=10.0,
        sample_step_m=30.0,
        target_height_m=0.0,
    ) is True

    # Endpoint terrain clearance still applies to very short/vertical rays.
    assert sim._check_los_terrain(
        0.0,
        0.0,
        300.0,
        0.0,
        0.0,
        105.0,
        clearance_m=10.0,
        sample_step_m=30.0,
        target_height_m=0.0,
    ) is False
    sim.shutdown()


def test_los_worker_rejects_old_generation_and_out_of_order_sequence() -> None:
    sim = SimulationService()
    sim.geo = GeoConverter(127.0, 38.0)
    lah = _vehicle("LAH1", 1, "lah", x=0.0, y=0.0, z=300.0)
    uav = _vehicle("UAV1", 4, "uav", x=1_000.0, y=0.0, z=1_000.0)
    base_job = sim._capture_enemy_lah_los_job_locked(
        geo=sim.geo,
        vehicles=[lah, uav],
        targets=[],
        step_count=1,
    )
    release = threading.Event()

    def blocked_compute(job):
        release.wait(timeout=2.0)
        return _EnemyLahLosResult(
            generation=job.generation,
            sequence=job.sequence,
            step=job.step,
            pair_keys=job.pair_keys,
            statuses={},
            completed_wall=time.monotonic(),
        )

    worker = _EnemyLahLosWorker(refresh_sec=60.0, compute=blocked_compute)
    try:
        assert worker.invalidate(2) is True
        stale_generation = replace(base_job, generation=1, sequence=30)
        newest = replace(base_job, generation=2, sequence=20)
        stale_sequence = replace(base_job, generation=2, sequence=10)

        assert worker.offer(stale_generation) is False
        assert worker.offer(newest) is True
        assert worker.offer(stale_sequence) is False
        assert worker.invalidate(1, sequence=40) is False
        assert worker.invalidate(2, sequence=10) is False
        assert worker.offer(newest) is False
        assert worker._desired_generation == 2
        assert worker._desired_sequence == 20
    finally:
        release.set()
        worker.close()
        sim.shutdown()


def test_los_worker_publishes_slow_same_fingerprint_result_during_refresh() -> None:
    sim = SimulationService()
    sim.geo = GeoConverter(127.0, 38.0)
    lah = _vehicle("LAH1", 1, "lah", x=0.0, y=0.0, z=300.0)
    uav = _vehicle("UAV1", 4, "uav", x=1_000.0, y=0.0, z=1_000.0)
    base_job = sim._capture_enemy_lah_los_job_locked(
        geo=sim.geo,
        vehicles=[lah, uav],
        targets=[],
        step_count=1,
    )
    first_job = replace(base_job, generation=2, sequence=1)
    second_job = replace(base_job, generation=2, sequence=2, step=2)
    first_started = threading.Event()
    release_first = threading.Event()
    second_started = threading.Event()
    release_second = threading.Event()

    def slow_compute(job):
        if job.sequence == first_job.sequence:
            first_started.set()
            release_first.wait(timeout=2.0)
        else:
            second_started.set()
            release_second.wait(timeout=2.0)
        return _EnemyLahLosResult(
            generation=job.generation,
            sequence=job.sequence,
            step=job.step,
            pair_keys=job.pair_keys,
            statuses={},
            completed_wall=time.monotonic(),
        )

    worker = _EnemyLahLosWorker(refresh_sec=0.1, compute=slow_compute)
    try:
        assert worker.offer(first_job) is True
        assert first_started.wait(timeout=1.0)
        time.sleep(0.11)
        assert worker.offer(second_job) is True
        release_first.set()
        assert second_started.wait(timeout=1.0)

        # The first result remains useful because the generation/fingerprint is
        # unchanged.  It must be published while the fresher sample computes.
        result = worker.latest(2, pair_keys=first_job.pair_keys)
        assert result is not None
        assert result.sequence == first_job.sequence
    finally:
        release_first.set()
        release_second.set()
        worker.close()
        sim.shutdown()


def test_los_worker_does_not_resurrect_result_after_fingerprint_cycles() -> None:
    sim = SimulationService()
    sim.geo = GeoConverter(127.0, 38.0)
    lah = _vehicle("LAH1", 1, "lah", x=0.0, y=0.0, z=300.0)
    uav = _vehicle("UAV1", 4, "uav", x=1_000.0, y=0.0, z=1_000.0)
    base_job = sim._capture_enemy_lah_los_job_locked(
        geo=sim.geo,
        vehicles=[lah, uav],
        targets=[],
        step_count=1,
    )
    inside_pair = base_job.communication_pairs[0]
    outside_pair = replace(inside_pair, target_x=25_000.0)
    first_job = replace(base_job, generation=2, sequence=1)
    outside_job = replace(
        base_job,
        generation=2,
        sequence=2,
        step=2,
        communication_pairs=(outside_pair,),
    )
    returned_job = replace(base_job, generation=2, sequence=3, step=3)
    assert first_job.pair_keys != outside_job.pair_keys
    assert first_job.pair_keys == returned_job.pair_keys

    first_started = threading.Event()
    release_first = threading.Event()
    returned_started = threading.Event()
    release_returned = threading.Event()

    def slow_compute(job):
        if job.sequence == first_job.sequence:
            first_started.set()
            release_first.wait(timeout=2.0)
        elif job.sequence == returned_job.sequence:
            returned_started.set()
            release_returned.wait(timeout=2.0)
        return _EnemyLahLosResult(
            generation=job.generation,
            sequence=job.sequence,
            step=job.step,
            pair_keys=job.pair_keys,
            statuses={},
            completed_wall=time.monotonic(),
        )

    worker = _EnemyLahLosWorker(refresh_sec=60.0, compute=slow_compute)
    try:
        assert worker.offer(first_job) is True
        assert first_started.wait(timeout=1.0)
        assert worker.offer(outside_job) is True
        assert worker.offer(returned_job) is True
        release_first.set()
        assert returned_started.wait(timeout=1.0)

        # Although the pair fingerprint returned to its original value, the
        # pre-transition result belongs to an older lineage and stays rejected.
        assert worker.latest(2, pair_keys=returned_job.pair_keys) is None
    finally:
        release_first.set()
        release_returned.set()
        worker.close()
        sim.shutdown()


def test_los_worker_drops_completed_result_when_fingerprint_cycles_back() -> None:
    sim = SimulationService()
    sim.geo = GeoConverter(127.0, 38.0)
    lah = _vehicle("LAH1", 1, "lah", x=0.0, y=0.0, z=300.0)
    uav = _vehicle("UAV1", 4, "uav", x=1_000.0, y=0.0, z=1_000.0)
    base_job = sim._capture_enemy_lah_los_job_locked(
        geo=sim.geo,
        vehicles=[lah, uav],
        targets=[],
        step_count=1,
    )
    outside_pair = replace(base_job.communication_pairs[0], target_x=25_000.0)
    first_job = replace(base_job, generation=2, sequence=1)
    outside_job = replace(
        base_job,
        generation=2,
        sequence=2,
        step=2,
        communication_pairs=(outside_pair,),
    )
    returned_job = replace(base_job, generation=2, sequence=3, step=3)
    outside_started = threading.Event()
    release_outside = threading.Event()
    returned_started = threading.Event()
    release_returned = threading.Event()

    def compute(job):
        if job.sequence == outside_job.sequence:
            outside_started.set()
            release_outside.wait(timeout=2.0)
        elif job.sequence == returned_job.sequence:
            returned_started.set()
            release_returned.wait(timeout=2.0)
        return _EnemyLahLosResult(
            generation=job.generation,
            sequence=job.sequence,
            step=job.step,
            pair_keys=job.pair_keys,
            statuses={},
            completed_wall=time.monotonic(),
        )

    worker = _EnemyLahLosWorker(refresh_sec=60.0, compute=compute)
    try:
        assert worker.offer(first_job) is True
        first_result = worker.wait_for_result(2, timeout=1.0)
        assert first_result is not None
        assert first_result.sequence == first_job.sequence

        assert worker.offer(outside_job) is True
        assert outside_started.wait(timeout=1.0)
        assert worker.offer(returned_job) is True

        # A completed result from the earlier A lineage is not valid after
        # A -> B -> A, even though its fingerprint matches the current A again.
        assert worker.latest(2, pair_keys=returned_job.pair_keys) is None
        release_outside.set()
        assert returned_started.wait(timeout=1.0)
        assert worker.latest(2, pair_keys=returned_job.pair_keys) is None
    finally:
        release_outside.set()
        release_returned.set()
        worker.close()
        sim.shutdown()


def test_range_transition_never_reuses_previous_terrain_status() -> None:
    sim = SimulationService()
    sim.geo = GeoConverter(127.0, 38.0)
    lah = _vehicle("LAH1", 1, "lah", x=0.0, y=0.0, z=300.0)
    uav = _vehicle("UAV1", 4, "uav", x=1_000.0, y=0.0, z=1_000.0)
    first_job = sim._capture_enemy_lah_los_job_locked(
        geo=sim.geo,
        vehicles=[lah, uav],
        targets=[],
        step_count=1,
    )
    second_started = threading.Event()
    release_second = threading.Event()
    call_count = 0

    def compute(job):
        nonlocal call_count
        call_count += 1
        if call_count == 2:
            second_started.set()
            release_second.wait(timeout=2.0)
        statuses = {
            pair.key: {
                "visible": False,
                "demAvailable": True,
                "demSources": ["Inje_10m.tif"],
                "targetRayAlt": pair.target_z,
            }
            for pair in job.communication_pairs
        }
        return _EnemyLahLosResult(
            generation=job.generation,
            sequence=job.sequence,
            step=job.step,
            pair_keys=job.pair_keys,
            statuses={},
            completed_wall=time.monotonic(),
            communication_statuses=statuses,
        )

    sim._enemy_lah_los_worker.close()
    sim._enemy_lah_los_worker = _EnemyLahLosWorker(
        refresh_sec=60.0,
        compute=compute,
    )
    try:
        sim._build_lah_uav_communication_links(job=first_job)
        assert sim._enemy_lah_los_worker.wait_for_result(
            first_job.generation,
        ) is not None
        first_link = sim._build_lah_uav_communication_links(job=first_job)[0]
        assert first_link["status"] == "terrainBlocked"

        uav.vehicle.s.x = 10_000.01
        second_job = sim._capture_enemy_lah_los_job_locked(
            geo=sim.geo,
            vehicles=[lah, uav],
            targets=[],
            step_count=2,
        )
        second_links = sim._build_lah_uav_communication_links(job=second_job)
        assert second_started.wait(timeout=1.0)
        assert second_links[0]["status"] == "outOfRange"
        assert second_links[0]["terrainVisible"] is None
        assert second_links[0]["terrainBlocked"] is None
        assert second_links[0]["demAvailable"] is None
    finally:
        release_second.set()
        sim.shutdown()


def test_snapshot_schema_always_exposes_los_links() -> None:
    sim = SimulationService()

    snapshot = sim.build_snapshot()

    assert snapshot["losLinks"] == []
    assert snapshot["communicationLinks"] == []
    assert snapshot["terrainLosModel"]["nativePlannerKernel"] is True
    assert snapshot["terrainLosModel"]["lahUavRangeM"] == pytest.approx(10_000.0)
    assert snapshot["terrainLosModel"]["communicationAffects0401"] is False
