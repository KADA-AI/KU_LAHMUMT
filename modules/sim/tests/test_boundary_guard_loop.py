from __future__ import annotations

from types import SimpleNamespace

from modules.sim.mission.mission_plan_loader import build_features_from_flight_paths
from modules.sim.mission.mission_validator import validate_mission_payload
from modules.sim.runtime.controllers.waypoint_pid import (
    WaypointPIDController,
    WaypointTarget,
)
from modules.sim.runtime.sim_service import (
    SimVehicle,
    SimulationService,
    _boundary_guard_wp_fields,
    _resolve_boundary_guard_sequence_contracts,
)
from modules.sim.runtime.uav import UAV, UAVParams


def _waypoint(wp_id: int, lon: float) -> dict:
    return {
        "waypointID": wp_id,
        "nextWaypointID": 0,
        "coordinate": {
            "latitude": 38.0,
            "longitude": lon,
            "altitude": 500,
        },
    }


def _path(path_id: int, aircraft_id: int, wp_ids: tuple[int, int]) -> dict:
    return {
        "pathID": path_id,
        "aircraftID": aircraft_id,
        "waypointList": [
            _waypoint(wp_ids[0], 127.0 + wp_ids[0] * 0.00001),
            _waypoint(wp_ids[1], 127.0 + wp_ids[1] * 0.00001),
        ],
    }


def _guard_target(
    wp_id: int,
    *,
    first: int,
    last: int,
    is_done: bool = False,
) -> WaypointTarget:
    return WaypointTarget(
        pos=(float(wp_id), 0.0, 500.0),
        wp_id=wp_id,
        next_wp_id=first if wp_id == last else None,
        is_done=is_done,
        boundary_guard_loop=True,
        boundary_guard_loop_version=1,
        boundary_guard_set_id="guard-A",
        boundary_guard_cycle_first_wp_id=first,
        boundary_guard_cycle_last_wp_id=last,
    )


def test_type2_region7_fallback_resolves_one_full_child_set_loop() -> None:
    flight_by_path = {
        400000001: _path(400000001, 4, (10, 11)),
        400000002: _path(400000002, 4, (20, 21)),
    }
    sequence = [
        {
            "path_id": 400000001,
            "input_mission_id": 5,
            "package_type": 2,
            "input_mission_type": 3,
            "region_type": 7,
        },
        {
            "path_id": 400000002,
            "input_mission_id": 5,
            "package_type": 2,
            "input_mission_type": 3,
            "region_type": 7,
        },
    ]

    resolved = _resolve_boundary_guard_sequence_contracts(
        4,
        sequence,
        flight_by_path,
    )

    assert set(resolved) == {400000001, 400000002}
    assert resolved[400000001]["sequence"] == 1
    assert resolved[400000002]["sequence"] == 2
    assert resolved[400000001]["sequence_count"] == 2
    assert resolved[400000001]["cycle_first_wp_id"] == 10
    assert resolved[400000002]["cycle_last_wp_id"] == 21
    assert resolved[400000001]["set_id"] == resolved[400000002]["set_id"]


def test_resolved_path_contract_reaches_waypoint_runtime_fields() -> None:
    resolved_contract = {
        "loop_present": True,
        "loop": True,
        "loop_version": 1,
        "set_id": "guard-A",
        "sequence": 2,
        "sequence_count": 2,
        "duration_s": 600.0,
        "cycle_first_wp_id": 10,
        "cycle_last_wp_id": 21,
    }

    fields = _boundary_guard_wp_fields(
        {"waypointID": 21, "nextWaypointID": 10},
        resolved_contract,
    )

    assert fields == {
        "next_wp_id": 10,
        "boundary_guard_loop": True,
        "boundary_guard_loop_version": 1,
        "boundary_guard_set_id": "guard-A",
        "boundary_guard_sequence": 2,
        "boundary_guard_sequence_count": 2,
        "boundary_guard_duration_s": 600.0,
        "boundary_guard_cycle_first_wp_id": 10,
        "boundary_guard_cycle_last_wp_id": 21,
    }


def test_waypoint_explicit_false_overrides_resolved_path_loop() -> None:
    fields = _boundary_guard_wp_fields(
        {
            "waypointID": 21,
            "nextWaypointID": 10,
            "boundaryGuardLoop": False,
        },
        {
            "loop_present": True,
            "loop": True,
            "set_id": "guard-A",
            "cycle_first_wp_id": 10,
            "cycle_last_wp_id": 21,
        },
    )

    assert fields["boundary_guard_loop"] is False
    assert fields["boundary_guard_set_id"] == "guard-A"


def test_sim_load_preserves_guard_contract_and_restarts_completed_pass() -> None:
    first = _path(400000001, 4, (10, 11))
    second = _path(400000002, 4, (20, 21))
    for sequence, path in enumerate((first, second), start=1):
        path.update(
            {
                "boundaryGuardLoop": True,
                "boundaryGuardLoopVersion": 1,
                "boundaryGuardSetID": "guard-A",
                "boundaryGuardSequence": sequence,
                "boundaryGuardSequenceCount": 2,
                "boundaryGuardDurationS": 600.0,
                "boundaryGuardCycleFirstWaypointID": 10,
                "boundaryGuardCycleLastWaypointID": 21,
            }
        )
        for waypoint in path["waypointList"]:
            waypoint.update(
                {
                    "speed": 40.0,
                    "waypointPassType": 1,
                    "isDone": True,
                }
            )
    first["waypointList"][-1]["nextWaypointID"] = 20
    second["waypointList"][-1].update(
        {
            "nextWaypointID": 10,
            "isDone": False,
        }
    )

    service = SimulationService()
    result = service.load_mission(
        {
            "missionPlanID": 700000001,
            "inputMissionPlans": [
                {
                    "timestamp": 1,
                    "inputMissionList": [
                        {
                            "inputMissionID": 5,
                            "inputMissionType": 3,
                            "regionType": 7,
                        }
                    ],
                }
            ],
            "individualMissionPlans": [
                {
                    "aircraftID": 4,
                    "individualMissionList": [
                        {
                            "individualMissionID": 800000001,
                            "pathID": 400000001,
                            "isDone": False,
                            "relatedMission": {"inputMissionID": 5},
                        },
                        {
                            "individualMissionID": 800000002,
                            "pathID": 400000002,
                            "isDone": False,
                            "relatedMission": {"inputMissionID": 5},
                        },
                    ],
                }
            ],
            "flightPaths": [first, second],
        }
    )

    assert result["ok"] is True
    controller = service.vehicles["UAV1"].controller
    assert [target.wp_id for target in controller.targets] == [10, 11, 20, 21]
    assert all(target.boundary_guard_loop for target in controller.targets)
    assert all(
        target.boundary_guard_set_id == "guard-A"
        for target in controller.targets
    )

    controller.curr_idx = 3
    controller._advance_wp()

    assert controller.curr_idx == 0
    assert controller.current_target().wp_id == 10
    assert controller.finished is False
    assert controller.advance_reason == "boundary_guard_loop"
    assert all(target.is_done is False for target in controller.targets)


def test_region7_fallback_is_disabled_outside_type2_package() -> None:
    path = _path(400000001, 4, (10, 11))
    sequence = [
        {
            "path_id": 400000001,
            "input_mission_id": 5,
            "package_type": 3,
            "input_mission_type": 3,
            "region_type": 7,
        }
    ]

    assert (
        _resolve_boundary_guard_sequence_contracts(
            4,
            sequence,
            {400000001: path},
        )
        == {}
    )


def test_guard_tail_wraps_to_set_first_and_bypasses_input_block() -> None:
    uav = UAV(UAVParams())
    controller = WaypointPIDController(
        uav,
        [
            _guard_target(10, first=10, last=21),
            _guard_target(11, first=10, last=21),
            _guard_target(20, first=10, last=21),
            _guard_target(21, first=10, last=21),
        ],
    )
    controller.curr_idx = 3
    controller.set_block_indices({3: 5})

    assert controller._should_enter_input_block() is False
    controller._advance_wp()

    assert controller.curr_idx == 0
    assert controller.finished is False
    assert controller.advance_reason == "boundary_guard_loop"
    assert controller.boundary_guard_cycle_counts == {"guard-A": 1}


def test_guard_wrap_reopens_waypoints_completed_during_the_previous_pass() -> None:
    controller = WaypointPIDController(
        UAV(UAVParams()),
        [
            _guard_target(10, first=10, last=21, is_done=True),
            _guard_target(11, first=10, last=21, is_done=True),
            _guard_target(20, first=10, last=21, is_done=True),
            _guard_target(21, first=10, last=21),
        ],
    )
    controller.curr_idx = 3
    controller.set_block_indices({3: 5})

    controller._advance_wp()

    assert controller.curr_idx == 0
    assert controller.finished is False
    assert controller.advance_reason == "boundary_guard_loop"
    assert all(target.is_done is False for target in controller.targets)


def test_guard_contract_does_not_loop_when_tail_next_id_is_zero() -> None:
    controller = WaypointPIDController(
        UAV(UAVParams()),
        [
            _guard_target(10, first=10, last=21),
            _guard_target(21, first=10, last=21),
        ],
    )
    controller.targets[-1].next_wp_id = 0
    controller.curr_idx = 1

    controller._advance_wp()

    assert controller.finished is True
    assert controller.advance_reason != "boundary_guard_loop"


def test_ordinary_terminal_waypoint_behavior_is_unchanged() -> None:
    uav = UAV(UAVParams())
    controller = WaypointPIDController(
        uav,
        [
            WaypointTarget(pos=(0.0, 0.0, 500.0), wp_id=1),
            WaypointTarget(pos=(100.0, 0.0, 500.0), wp_id=2),
        ],
    )
    controller.curr_idx = 1
    controller.set_block_indices({1: 9})

    assert controller._should_enter_input_block() is True
    controller.set_block_indices({})
    controller._advance_wp()

    assert controller.finished is True
    assert controller.curr_idx == 2
    assert controller.boundary_guard_cycle_counts == {}


def test_guard_status_stays_active_while_tracking_uses_saved_controller() -> None:
    controller = WaypointPIDController(
        UAV(UAVParams()),
        [_guard_target(10, first=10, last=21)],
    )
    controller.boundary_guard_cycle_counts["guard-A"] = 2
    tracking = SimpleNamespace(saved_controller=controller, stage=1)
    simv = SimVehicle(
        label="UAV1",
        aircraft_id=4,
        airframe="uav",
        vehicle=controller.uav,
        controller=WaypointPIDController(
            controller.uav,
            [WaypointTarget(pos=(0.0, 0.0, 500.0), wp_id=None)],
        ),
        path_id=None,
    )

    status = SimulationService()._boundary_guard_runtime_status(simv, tracking)

    assert status["boundaryGuardLoopActive"] is True
    assert status["boundaryGuardSetID"] == "guard-A"
    assert status["boundaryGuardCycleCount"] == 2


def test_visualization_adds_one_finite_closure_without_duplicate_waypoints() -> None:
    first = _path(400000001, 4, (10, 11))
    second = _path(400000002, 4, (20, 21))
    for sequence, path in enumerate((first, second), start=1):
        path.update(
            {
                "boundaryGuardLoop": True,
                "boundaryGuardLoopVersion": 1,
                "boundaryGuardSetID": "guard-A",
                "boundaryGuardSequence": sequence,
                "boundaryGuardSequenceCount": 2,
                "boundaryGuardCycleFirstWaypointID": 10,
                "boundaryGuardCycleLastWaypointID": 21,
            }
        )

    features, _ = build_features_from_flight_paths([first, second])

    assert len(features) == 2
    assert [len(feature["coords"]) for feature in features] == [2, 2]
    assert sum("loopClosureCoord" in feature for feature in features) == 1
    assert "loopClosureCoord" not in features[0]
    assert features[1]["loopClosureCoord"] == features[0]["coords"][0]
    assert features[1]["boundaryGuardSequenceCount"] == 2


def test_sim_validator_accepts_only_the_declared_guard_cross_path_tails() -> None:
    first = _path(400000001, 4, (10, 11))
    second = _path(400000002, 4, (20, 21))
    for sequence, path in enumerate((first, second), start=1):
        path.update(
            {
                "timestamp": 1,
                "isFormationFlight": False,
                "boundaryGuardLoop": True,
                "boundaryGuardLoopVersion": 1,
                "boundaryGuardSetID": "guard-A",
                "boundaryGuardSequence": sequence,
                "boundaryGuardSequenceCount": 2,
                "boundaryGuardCycleFirstWaypointID": 10,
                "boundaryGuardCycleLastWaypointID": 21,
            }
        )
        for waypoint in path["waypointList"]:
            waypoint.update(
                {
                    "speed": 40.0,
                    "eta": 0,
                    "ecf": 0.0,
                    "waypointPassType": 1,
                    "isDone": False,
                }
            )
    first["waypointList"][-1]["nextWaypointID"] = 20
    second["waypointList"][-1]["nextWaypointID"] = 10

    valid = validate_mission_payload(
        {
            "flightPaths": [first, second],
            "inputMissionPlans": [],
            "individualMissionPlans": [],
        }
    )
    assert not any(
        issue.get("code") == "next_waypoint_missing"
        for issue in valid["issues"]
    )

    first["waypointList"][-1]["nextWaypointID"] = 21
    invalid = validate_mission_payload(
        {
            "flightPaths": [first, second],
            "inputMissionPlans": [],
            "individualMissionPlans": [],
        }
    )
    assert any(
        issue.get("code") == "next_waypoint_missing"
        and issue.get("actual") == 21
        for issue in invalid["issues"]
    )
