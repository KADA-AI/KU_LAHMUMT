from __future__ import annotations

from modules.sim.runtime.controllers.waypoint_pid import (
    WaypointPIDController,
    WaypointTarget,
)
from modules.sim.runtime.sim_service import SimulationService
from modules.sim.runtime.uav import UAV, UAVParams


def _guard_target(wp_id: int, set_id: str) -> WaypointTarget:
    return WaypointTarget(
        pos=(float(wp_id), 0.0, 500.0),
        wp_id=wp_id,
        input_mission_id=5,
        individual_mission_id=900000000 + wp_id,
        path_id=500000000 + wp_id,
        boundary_guard_loop=True,
        boundary_guard_loop_version=1,
        boundary_guard_set_id=set_id,
        boundary_guard_cycle_first_wp_id=wp_id,
        boundary_guard_cycle_last_wp_id=wp_id,
    )


def _controller(*targets: WaypointTarget) -> WaypointPIDController:
    return WaypointPIDController(UAV(UAVParams()), list(targets))


def test_changed_waypoint_signature_restores_only_stable_guard_set_counter() -> None:
    service = SimulationService()
    previous = _controller(
        _guard_target(10, "guard:stable"),
        _guard_target(20, "guard:retired"),
    )
    previous.boundary_guard_cycle_counts = {
        "guard:stable": 2,
        "guard:retired": 7,
    }
    snapshot = service._capture_controller_progress(previous)
    assert snapshot is not None

    current = _controller(
        _guard_target(110, "guard:stable"),
        _guard_target(120, "guard:new"),
    )

    fully_restored = service._restore_controller_progress(current, snapshot)

    assert fully_restored is False
    assert current.curr_idx == 0
    assert current.boundary_guard_cycle_counts == {"guard:stable": 2}


def test_matching_signature_still_restores_full_controller_progress() -> None:
    service = SimulationService()
    previous = _controller(_guard_target(10, "guard:stable"))
    previous.curr_idx = 0
    previous.boundary_guard_cycle_counts = {"guard:stable": 3}
    snapshot = service._capture_controller_progress(previous)
    assert snapshot is not None

    current = _controller(_guard_target(10, "guard:stable"))

    fully_restored = service._restore_controller_progress(current, snapshot)

    assert fully_restored is True
    assert current.boundary_guard_cycle_counts == {"guard:stable": 3}
