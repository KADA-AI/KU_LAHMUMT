"""A ground target stands on the terrain, not at sea level.

Targets used to spawn at z=0. The LOS kernels hid it - they clamp the ray to
ground+height - but slant range, the 0402 report altitude and the map marker
all took z at face value, which put the enemy hundreds of metres underground.
"""

from __future__ import annotations

from typing import Any

import pytest

from modules.sim.runtime.geo import GeoConverter
from modules.sim.runtime.sim_service import (
    _GROUND_TARGET_SPAWN_HEIGHT_M,
    SimulationService,
)

ORIGIN_LON = 128.20
ORIGIN_LAT = 37.87
GROUND_M = 583.0


def _sim(*, terrain: Any = GROUND_M, with_geo: bool = True) -> SimulationService:
    sim = SimulationService()
    if with_geo:
        sim.geo = GeoConverter(ORIGIN_LON, ORIGIN_LAT)
    if terrain is None:
        # No terrain source loaded at all.
        sim._terrain_elev_fn = False
    else:
        sim._terrain_elev_fn = lambda _lat, _lon, _v=float(terrain): float(_v)
    return sim


def _xy(sim: SimulationService) -> tuple[float, float]:
    return sim.geo.lonlat_to_xy(128.2099, 37.8664)


def test_a_target_spawns_one_metre_above_the_terrain() -> None:
    sim = _sim()
    try:
        x, y = _xy(sim)
        target = sim._build_target(type_id=1, x=x, y=y, z=0.0)

        assert target.z == pytest.approx(GROUND_M + _GROUND_TARGET_SPAWN_HEIGHT_M)
        assert _GROUND_TARGET_SPAWN_HEIGHT_M == 1.0
    finally:
        sim.shutdown()


def test_the_raised_altitude_reaches_everything_downstream() -> None:
    """Slant range, the 0402 report and the map marker all read target.z."""

    sim = _sim()
    try:
        x, y = _xy(sim)
        target = sim._build_target(type_id=1, x=x, y=y, z=0.0)
        payload = sim._target_to_dict(target, sim.geo)

        assert payload["alt"] == pytest.approx(GROUND_M + _GROUND_TARGET_SPAWN_HEIGHT_M)
        assert payload["alt"] > 0.0
    finally:
        sim.shutdown()


def test_an_explicitly_placed_altitude_is_not_overridden() -> None:
    sim = _sim()
    try:
        x, y = _xy(sim)
        target = sim._build_target(type_id=1, x=x, y=y, z=1234.0)

        assert target.z == pytest.approx(1234.0)
    finally:
        sim.shutdown()


def test_a_run_without_a_terrain_source_behaves_exactly_as_before() -> None:
    sim = _sim(terrain=None)
    try:
        x, y = _xy(sim)
        target = sim._build_target(type_id=1, x=x, y=y, z=0.0)

        assert target.z == pytest.approx(0.0)
    finally:
        sim.shutdown()


def test_a_nonfinite_terrain_sample_is_not_trusted() -> None:
    sim = _sim(terrain=float("nan"))
    try:
        x, y = _xy(sim)
        target = sim._build_target(type_id=1, x=x, y=y, z=0.0)

        assert target.z == pytest.approx(0.0)
    finally:
        sim.shutdown()


def test_no_projection_yet_keeps_the_requested_altitude() -> None:
    """Queued targets are built again once geo exists; do not guess early."""

    sim = _sim(with_geo=False)
    try:
        assert sim._ground_target_spawn_altitude(0.0, 0.0, 0.0) == pytest.approx(0.0)
    finally:
        sim.shutdown()


def test_every_target_type_spawns_on_the_terrain() -> None:
    sim = _sim()
    try:
        x, y = _xy(sim)
        for type_id in range(1, 7):
            target = sim._build_target(type_id=type_id, x=x, y=y, z=0.0)
            assert target.z == pytest.approx(
                GROUND_M + _GROUND_TARGET_SPAWN_HEIGHT_M
            ), f"target type {type_id} spawned at {target.z}"
    finally:
        sim.shutdown()


def test_targets_do_not_drift_off_the_terrain() -> None:
    """Every configured type is stationary, so the spawn altitude is final."""

    sim = _sim()
    try:
        x, y = _xy(sim)
        target = sim._build_target(type_id=1, x=x, y=y, z=0.0)
        spawn_z = float(target.z)
        for _ in range(200):
            target.step(0.5)

        assert target.z == pytest.approx(spawn_z)
        assert target.x == pytest.approx(x)
        assert target.y == pytest.approx(y)
    finally:
        sim.shutdown()
