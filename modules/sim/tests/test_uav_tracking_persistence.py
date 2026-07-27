"""Track on first sighting, release on death - and nothing in between.

Releasing a live target put it straight back into the auto-acquire pool, so the
aircraft flipped between tracking and sweeping every time a planned loiter
duration expired.
"""

from __future__ import annotations

from types import SimpleNamespace
from typing import Any

from modules.sim.runtime.geo import GeoConverter
from modules.sim.runtime.sim_service import SimulationService, TrackingState


def _controller() -> SimpleNamespace:
    return SimpleNamespace(
        current_target=lambda: None,
        _advance_wp=lambda: None,
        speed_target=30.0,
    )


def _vehicle(label: str = "UAV1", aircraft_id: int = 4) -> SimpleNamespace:
    return SimpleNamespace(
        label=label,
        aircraft_id=aircraft_id,
        airframe="uav",
        alive=True,
        vehicle=SimpleNamespace(
            s=SimpleNamespace(x=0.0, y=0.0, z=1000.0, u=30.0, yaw=0.0, roll=0.0, pitch=0.0)
        ),
        controller=_controller(),
        shinil_profile_phase="baseline",
    )


def _tracking_state(sim: SimulationService, target: Any, **overrides: Any) -> TrackingState:
    controller = _controller()
    defaults: dict[str, Any] = {
        "target_id": int(target.id),
        "target": target,
        "saved_controller": controller,
        "saved_wp_id": None,
        "tracking_controller": controller,
        "loiter_wp": SimpleNamespace(filming=None),
        "fov_deg": 5.0,
        "stage": 1,
        "start_step": 0,
        "last_seen": 0.0,
        "filming_prop": {"operationMode": 3, "fieldOfView": 5.0},
        "end_time": None,
        "advance_on_complete": True,
        "manual": True,
        "track_radius": 180.0,
        "track_speed": 30.0,
    }
    defaults.update(overrides)
    return TrackingState(**defaults)


def _sim_with_tracked_target(**state_overrides: Any):
    sim = SimulationService()
    sim.geo = GeoConverter(127.0, 38.0)
    # targetInfo persists across scenario runs; a leftover 'destroyed' row
    # would release the track for reasons unrelated to this test.
    sim._load_target_info_map = lambda: {}
    vehicle = _vehicle()
    sim.vehicles = {vehicle.label: vehicle}
    target = sim._build_target(type_id=1, x=500.0, y=0.0, z=0.0, id_override=11)
    sim.targets = [target]
    sim._tracking_state[vehicle.label] = _tracking_state(sim, target, **state_overrides)
    sim._tracking_target_owner[int(target.id)] = vehicle.label
    return sim, vehicle, target


def test_an_expired_loiter_duration_no_longer_drops_a_live_target() -> None:
    """The oscillation source: a timer used to release a target that was fine."""

    sim, vehicle, target = _sim_with_tracked_target(end_time=1.0)
    try:
        sim.sim_time = 9999.0  # long past any planned dwell
        sim._update_tracking(vehicle, 0.1)

        assert vehicle.label in sim._tracking_state
        assert sim._tracking_target_owner.get(int(target.id)) == vehicle.label
    finally:
        sim.shutdown()


def test_tracking_survives_many_updates_without_releasing_ownership() -> None:
    sim, vehicle, target = _sim_with_tracked_target()
    try:
        for _ in range(200):
            sim.sim_time += 0.5
            sim._update_tracking(vehicle, 0.5)

        assert vehicle.label in sim._tracking_state
        assert sim._tracking_target_owner.get(int(target.id)) == vehicle.label
    finally:
        sim.shutdown()


def test_killing_the_target_releases_the_track() -> None:
    sim, vehicle, target = _sim_with_tracked_target()
    try:
        target.alive = False
        sim._update_tracking(vehicle, 0.1)

        assert vehicle.label not in sim._tracking_state
        assert int(target.id) not in sim._tracking_target_owner
    finally:
        sim.shutdown()


def test_a_destroyed_report_also_releases_the_track() -> None:
    sim, vehicle, target = _sim_with_tracked_target()
    try:
        sim._load_target_info_map = lambda: {
            "11-4": {"targetID": 11, "isDestroyed": True, "watcherID": 4}
        }
        sim._update_tracking(vehicle, 0.1)

        assert vehicle.label not in sim._tracking_state
    finally:
        sim.shutdown()


def test_a_dead_target_is_never_re_acquired() -> None:
    sim = SimulationService()
    sim.geo = GeoConverter(127.0, 38.0)
    sim._load_target_info_map = lambda: {}
    vehicle = _vehicle()
    sim.vehicles = {vehicle.label: vehicle}
    try:
        target = sim._build_target(type_id=1, x=200.0, y=0.0, z=0.0, id_override=11)
        target.alive = False
        sim.targets = [target]

        assert sim._visible_tracking_targets(vehicle, 30.0) == []
    finally:
        sim.shutdown()
