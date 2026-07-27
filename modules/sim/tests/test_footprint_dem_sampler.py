from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest

from modules.common.regional_dem import REGIONAL_DEM_SPECS
from modules.mission_planning.MissionPlanner.data_def import mission_helpers
from modules.sim.runtime import sim_service
from modules.sim.runtime.geo import GeoConverter


RESOURCE_DIR = Path(__file__).resolve().parents[3] / "resource"


@pytest.mark.parametrize("spec", REGIONAL_DEM_SPECS, ids=lambda spec: spec.filename)
def test_footprint_sampler_matches_shared_operational_dem(spec) -> None:
    tile = sim_service._load_footprint_dem_tile(RESOURCE_DIR / spec.filename)

    for lat_fraction in (0.15, 0.37, 0.61, 0.85):
        for lon_fraction in (0.15, 0.43, 0.72, 0.85):
            latitude = spec.south + ((spec.north - spec.south) * lat_fraction)
            longitude = spec.west + ((spec.east - spec.west) * lon_fraction)
            assert tile.sample(latitude, longitude) == pytest.approx(
                mission_helpers.terrain_elev(latitude, longitude),
                abs=1e-9,
            )


def test_footprint_sampler_hot_path_does_not_call_shared_scalar_lookup(monkeypatch) -> None:
    spec = REGIONAL_DEM_SPECS[0]
    tile = sim_service._load_footprint_dem_tile(RESOURCE_DIR / spec.filename)
    latitude = (spec.south + spec.north) / 2.0
    longitude = (spec.west + spec.east) / 2.0
    expected = tile.sample(latitude, longitude)

    monkeypatch.setattr(
        mission_helpers,
        "terrain_elev",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(
            AssertionError("footprint hot path delegated to terrain_elev")
        ),
    )

    assert tile.sample(latitude, longitude) == expected


def test_terrain_los_uses_dem_ridge_and_caches_exact_ray(monkeypatch) -> None:
    sim = sim_service.SimulationService()
    samples: list[tuple[float, float]] = []

    def ridge_height(x: float, y: float) -> float:
        samples.append((float(x), float(y)))
        return 80.0 if 45.0 <= float(x) <= 55.0 else 0.0

    monkeypatch.setattr(
        sim,
        "_build_footprint_terrain_context",
        lambda _x, _y: (ridge_height, 5.0),
    )
    assert sim._check_los_terrain(0.0, 0.0, 100.0, 100.0, 0.0, 0.0) is False
    sampled_once = len(samples)
    assert sampled_once > 2

    # An identical sensor/target ray reuses the per-step result.
    assert sim._check_los_terrain(0.0, 0.0, 100.0, 100.0, 0.0, 0.0) is False
    assert len(samples) == sampled_once

    sim._terrain_los_cache.clear()
    monkeypatch.setattr(
        sim,
        "_build_footprint_terrain_context",
        lambda _x, _y: (lambda _gx, _gy: 0.0, 5.0),
    )
    assert sim._check_los_terrain(0.0, 0.0, 100.0, 100.0, 0.0, 0.0) is True


def test_camera_detection_uses_displayed_footprint_as_single_truth(monkeypatch) -> None:
    sim = sim_service.SimulationService()
    state = SimpleNamespace(x=0.0, y=0.0, z=100.0, yaw=0.0, pitch=0.0, roll=0.0)
    simv = SimpleNamespace(
        label="UAV1",
        vehicle=SimpleNamespace(s=state),
    )
    sim.uav_detection_range_m = 1.0
    monkeypatch.setattr(sim, "_check_los_terrain", lambda *_args, **_kwargs: False)
    monkeypatch.setattr(sim, "_point_in_camera_frustum", lambda *_args, **_kwargs: False)

    footprint = [(-10.0, -10.0), (10.0, -10.0), (10.0, 10.0), (-10.0, 10.0)]
    assert sim._point_in_camera_view_with_polygon(
        simv,
        x=0.0,
        y=0.0,
        z=100_000.0,
        fov_deg=20.0,
        polygon=footprint,
    )
    assert sim._point_in_camera_view_with_polygon(
        simv,
        x=10.0,
        y=0.0,
        z=-100_000.0,
        fov_deg=20.0,
        polygon=footprint,
    )
    assert not sim._point_in_camera_view_with_polygon(
        simv,
        x=10.1,
        y=0.0,
        z=0.0,
        fov_deg=20.0,
        polygon=footprint,
    )


def test_visible_target_is_detected_when_footprint_covers_its_xy(monkeypatch) -> None:
    sim = sim_service.SimulationService()
    sim.uav_detection_range_m = 1.0
    target = SimpleNamespace(id=1, alive=True, x=0.0, y=0.0, z=100_000.0)
    sim.targets = [target]
    state = SimpleNamespace(x=0.0, y=0.0, z=100.0)
    simv = SimpleNamespace(
        label="UAV1",
        aircraft_id=4,
        vehicle=SimpleNamespace(s=state),
    )
    footprint = [(-10.0, -10.0), (10.0, -10.0), (10.0, 10.0), (-10.0, 10.0)]
    monkeypatch.setattr(sim, "_camera_view_polygon", lambda *_args, **_kwargs: footprint)
    monkeypatch.setattr(sim, "_check_los_terrain", lambda *_args, **_kwargs: False)
    monkeypatch.setattr(sim, "_point_in_camera_frustum", lambda *_args, **_kwargs: False)
    monkeypatch.setattr(
        sim,
        "_target_info_visibility_context",
        lambda: {
            "entries_by_id": {},
            "ignored_ids": set(),
            "ignored_xy": [],
            "destroyed_xy": [],
        },
    )

    assert sim._visible_tracking_targets(simv, 20.0) == [target]


def test_active_camera_emits_0402_detection_when_footprint_covers_target(monkeypatch) -> None:
    sim = sim_service.SimulationService()
    sim.auto_track_always = True
    sim.auto_track_takeover = False
    target = SimpleNamespace(id=7, alive=True, x=0.0, y=0.0, z=100_000.0)
    sim.targets = [target]
    current = SimpleNamespace(
        filming={"operationMode": 1, "fieldOfView": 20.0},
    )
    state = SimpleNamespace(x=0.0, y=0.0, z=100.0)
    simv = SimpleNamespace(
        label="UAV1",
        aircraft_id=4,
        airframe="uav",
        controller=SimpleNamespace(current_target=lambda: current),
        vehicle=SimpleNamespace(s=state),
    )
    footprint = [(-10.0, -10.0), (10.0, -10.0), (10.0, 10.0), (-10.0, 10.0)]
    monkeypatch.setattr(sim, "_camera_view_polygon", lambda *_args, **_kwargs: footprint)
    monkeypatch.setattr(
        sim,
        "_target_info_visibility_context",
        lambda: {
            "entries_by_id": {},
            "ignored_ids": set(),
            "ignored_xy": [],
            "destroyed_xy": [],
        },
    )
    detected: list[tuple[object, object, float]] = []
    monkeypatch.setattr(
        sim,
        "_record_0402_target_list",
        lambda vehicle, found, fov: detected.append((vehicle, found, fov)),
    )

    sim._update_tracking(simv, 0.1)

    assert detected == [(simv, target, 20.0)]


def test_visual_boundary_vertices_are_actual_inje_dem_intersections() -> None:
    latitude = 37.85
    longitude = 128.25
    tile = sim_service._resolve_footprint_dem_tile(latitude, longitude)
    assert tile is not None and tile.path.name == "Inje_10m.tif"
    ground = tile.sample(latitude, longitude)

    sim = sim_service.SimulationService()
    sim.geo = GeoConverter(longitude, latitude)
    state = SimpleNamespace(
        x=0.0,
        y=0.0,
        z=ground + 1000.0,
        yaw=0.0,
        pitch=0.0,
        roll=0.0,
    )
    simv = SimpleNamespace(label="UAV1", vehicle=SimpleNamespace(s=state))

    projection = sim._resolve_footprint_projection(
        simv,
        (0.0, 0.0, ground),
        5.7,
    )

    assert projection is not None
    assert len(projection.corners) == 4
    assert len(projection.boundary) == 8
    for x, y, altitude in projection.boundary:
        lon, lat = sim.geo.xy_to_lonlat(x, y)
        assert altitude == pytest.approx(tile.sample(lat, lon), abs=1e-6)
