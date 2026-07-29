from __future__ import annotations

import unittest

from modules.sim.runtime.geo import GeoConverter
from modules.sim.runtime.sim_service import (
    PathDefinition,
    SimulationService,
    _mission_execution_blocked_for_load,
)


class SkipToMissionStartTest(unittest.TestCase):
    def test_preserved_active_input_ignores_only_its_stale_collab_block(
        self,
    ) -> None:
        blocked = {"executionBlockedUntilNextCollab": True}

        self.assertFalse(
            _mission_execution_blocked_for_load(
                blocked,
                input_mission_id=5,
                start_input_mission_id=None,
                preserved_active_input_mission_id=5,
            )
        )
        self.assertTrue(
            _mission_execution_blocked_for_load(
                blocked,
                input_mission_id=6,
                start_input_mission_id=None,
                preserved_active_input_mission_id=5,
            )
        )
        self.assertFalse(
            _mission_execution_blocked_for_load(
                blocked,
                input_mission_id=6,
                start_input_mission_id=6,
                preserved_active_input_mission_id=5,
            )
        )

    def _build_service(self) -> SimulationService:
        service = SimulationService()
        service.geo = GeoConverter(127.0, 38.0)
        path = PathDefinition(
            label="UAV1",
            aircraft_id=4,
            airframe="uav",
            path_id=4001,
            waypoints=[
                {
                    "lat": 38.001,
                    "lon": 127.001,
                    "alt": 300.0,
                    "wp_id": 1001,
                    "input_mission_id": 101,
                    "speed": 60.0,
                },
                {
                    "lat": 38.010,
                    "lon": 127.010,
                    "alt": 420.0,
                    "wp_id": 2001,
                    "input_mission_id": 202,
                    "speed": 70.0,
                },
                {
                    "lat": 38.020,
                    "lon": 127.020,
                    "alt": 450.0,
                    "wp_id": 2002,
                    "input_mission_id": 202,
                    "speed": 70.0,
                },
            ],
        )
        service._paths = [path]
        service._spawn_by_aircraft = {4: (0.0, 0.0, 300.0)}
        service.input_mission_order_by_aircraft = {4: [101, 202]}
        service.current_input_mission_idx_by_aircraft = {4: 1}
        service._build_vehicles([path])
        return service

    def test_teleports_to_first_wp_of_active_input_mission(self) -> None:
        service = self._build_service()
        simv = service.vehicles["UAV1"]
        controller = simv.controller
        controller.curr_idx = 2
        controller._closest_wp_idx = 2
        controller.blocked = True
        controller.blocked_input_id = 202
        controller.is_loitering = True
        controller.finished = True
        simv.vehicle.s.x = -9000.0
        simv.vehicle.s.y = 5000.0
        simv.vehicle.s.z = 100.0
        simv.vehicle.s.pitch = 12.0
        simv.vehicle.s.roll = -20.0
        service._history.append({"step": 1})

        result = service.skip_to_current_mission_start()

        self.assertTrue(result["ok"])
        self.assertEqual(result["count"], 1)
        self.assertEqual(result["teleported"][0]["waypointID"], 2001)
        self.assertEqual(result["teleported"][0]["inputMissionID"], 202)
        self.assertEqual(controller.curr_idx, 1)
        self.assertFalse(controller.finished)
        self.assertFalse(controller.blocked)
        self.assertFalse(controller.is_loitering)
        self.assertAlmostEqual(simv.vehicle.s.x, controller.targets[1].pos[0])
        self.assertAlmostEqual(simv.vehicle.s.y, controller.targets[1].pos[1])
        self.assertAlmostEqual(simv.vehicle.s.z, controller.targets[1].pos[2])
        self.assertEqual(simv.vehicle.s.pitch, 0.0)
        self.assertEqual(simv.vehicle.s.roll, 0.0)
        self.assertEqual(len(service._history), 0)

        service._step_once(service.dt)
        self.assertEqual(controller.curr_idx, 2)
        self.assertFalse(controller.finished)

    def test_returns_error_without_loaded_vehicles(self) -> None:
        service = SimulationService()

        result = service.skip_to_current_mission_start()

        self.assertFalse(result["ok"])
        self.assertEqual(result["count"], 0)

    def test_single_terminal_loiter_loads_and_finishes_after_its_duration(self) -> None:
        service = SimulationService()
        result = service.load_mission(
            {
                "missionPlanID": 7002,
                "inputMissionPlans": [
                    {
                        "timestamp": 1,
                        "inputMissionList": [
                            {
                                "inputMissionID": 101,
                                "inputMissionType": 1,
                                "regionType": 4,
                            }
                        ],
                    }
                ],
                "individualMissionPlans": [
                    {
                        "aircraftID": 4,
                        "individualMissionList": [
                            {
                                "individualMissionID": 8002,
                                "pathID": 4002,
                                "isDone": False,
                                "relatedMission": {"inputMissionID": 101},
                            }
                        ],
                    }
                ],
                "flightPaths": [
                    {
                        "pathID": 4002,
                        "aircraftID": 4,
                        "waypointList": [
                            {
                                "waypointID": 1002,
                                "nextWaypointID": 0,
                                "isDone": False,
                                "waypointPassType": 2,
                                "coordinate": {
                                    "latitude": 38.001,
                                    "longitude": 127.001,
                                    "altitude": 300.0,
                                },
                                "loiterProperty": {
                                    "radius": 180.0,
                                    "direction": 1,
                                    "time": 15.0,
                                    "speed": 30.0,
                                },
                            }
                        ],
                    }
                ],
            }
        )

        self.assertTrue(result["ok"])
        self.assertEqual(result["count"], 1)
        self.assertIn("UAV1", service.vehicles)
        simv = service.vehicles["UAV1"]
        controller = simv.controller
        controller.update(0.5)
        self.assertTrue(controller.is_loitering)
        self.assertFalse(controller.finished)
        self.assertEqual(service._on_mission_for(simv), 1)

        # The 15-second timer begins only after the waypoint is acquired, so
        # the arrival update above does not consume any of the hold duration.
        for _ in range(30):
            controller.update(0.5)

        self.assertTrue(controller.finished)
        self.assertEqual(controller.advance_reason, "loiter")
        self.assertEqual(service._on_mission_for(simv), 2)

    def test_postattack_loiter_finishes_by_assignment_time_before_waypoint_arrival(self) -> None:
        service = SimulationService()
        result = service.load_mission(
            {
                "missionPlanID": 7005,
                "inputMissionPlans": [
                    {
                        "timestamp": 1,
                        "inputMissionList": [
                            {
                                "inputMissionID": 101,
                                "inputMissionType": 1,
                                "regionType": 4,
                            }
                        ],
                    }
                ],
                "individualMissionPlans": [
                    {
                        "aircraftID": 4,
                        "individualMissionList": [
                            {
                                "individualMissionID": 8005,
                                "pathID": 4005,
                                "isDone": False,
                                "postAttackBoundaryHold": True,
                                "relatedMission": {"inputMissionID": 101},
                            }
                        ],
                    }
                ],
                "flightPaths": [
                    {
                        "pathID": 4005,
                        "aircraftID": 4,
                        "waypointList": [
                            {
                                "waypointID": 1005,
                                "nextWaypointID": 0,
                                "isDone": False,
                                "waypointPassType": 2,
                                "coordinate": {
                                    "latitude": 38.01,
                                    "longitude": 127.01,
                                    "altitude": 300.0,
                                },
                                "loiterProperty": {
                                    "radius": 180.0,
                                    "direction": 1,
                                    "time": 15.0,
                                    "speed": 30.0,
                                },
                            }
                        ],
                    }
                ],
            }
        )

        self.assertTrue(result["ok"])
        simv = service.vehicles["UAV1"]
        controller = simv.controller
        target = controller.current_target()
        self.assertTrue(target.complete_loiter_after_assignment)

        # Keep the UAV far outside the waypoint capture radius for the entire
        # timer.  Completion must depend only on assignment age.
        simv.vehicle.s.x = target.pos[0] - 20_000.0
        simv.vehicle.s.y = target.pos[1] - 20_000.0
        for _ in range(29):
            controller.update(0.5)

        self.assertFalse(controller.finished)
        self.assertEqual(service._on_mission_for(simv), 1)
        controller.update(0.5)

        self.assertTrue(controller.finished)
        self.assertEqual(controller.advance_reason, "loiter")
        self.assertEqual(service._on_mission_for(simv), 2)

    def test_attack_completion_release_transit_uses_eta_without_loiter(self) -> None:
        service = SimulationService()
        result = service.load_mission(
            {
                "missionPlanID": 7006,
                "inputMissionPlans": [
                    {
                        "timestamp": 1,
                        "inputMissionList": [
                            {
                                "inputMissionID": 101,
                                "inputMissionType": 1,
                                "regionType": 4,
                            }
                        ],
                    }
                ],
                "individualMissionPlans": [
                    {
                        "aircraftID": 6,
                        "individualMissionList": [
                            {
                                "individualMissionID": 900000384,
                                "pathID": 600000062,
                                "isDone": False,
                                "attackCompletionBoundaryHold": True,
                                "relatedMission": {"inputMissionID": 101},
                            }
                        ],
                    }
                ],
                "flightPaths": [
                    {
                        "pathID": 600000062,
                        "aircraftID": 6,
                        "attackCompletionBoundaryHold": True,
                        "waypointList": [
                            {
                                "waypointID": 29015,
                                "nextWaypointID": 0,
                                "isDone": False,
                                "waypointPassType": 3,
                                "eta": 10,
                                "coordinate": {
                                    "latitude": 38.107216,
                                    "longitude": 127.312194,
                                    "altitude": 1312.0,
                                },
                                "loiterProperty": {},
                            }
                        ],
                    }
                ],
            }
        )

        self.assertTrue(result["ok"])
        simv = service.vehicles["UAV3"]
        controller = simv.controller
        target = controller.current_target()
        self.assertTrue(target.complete_loiter_after_assignment)
        self.assertEqual(target.assignment_completion_seconds, 10.0)

        simv.vehicle.s.x = target.pos[0] - 20_000.0
        simv.vehicle.s.y = target.pos[1] - 20_000.0
        for _ in range(19):
            controller.update(0.5)

        self.assertFalse(controller.finished)
        self.assertEqual(service._on_mission_for(simv), 1)
        controller.update(0.5)

        self.assertTrue(controller.finished)
        self.assertEqual(controller.advance_reason, "loiter")
        self.assertEqual(service._on_mission_for(simv), 2)

    def test_single_non_loiter_waypoint_remains_non_executable(self) -> None:
        service = SimulationService()
        service.geo = GeoConverter(127.0, 38.0)
        path = PathDefinition(
            label="UAV1",
            aircraft_id=4,
            airframe="uav",
            path_id=4003,
            waypoints=[
                {
                    "lat": 38.001,
                    "lon": 127.001,
                    "alt": 300.0,
                    "wp_id": 1003,
                    "is_done": False,
                    "pass_type": 3,
                }
            ],
        )

        service._build_vehicles([path])

        self.assertNotIn("UAV1", service.vehicles)

    def test_single_line_sweep_replaces_preserved_tracking_controller(self) -> None:
        service = SimulationService()
        initial = service.load_mission(
            {
                "missionPlanID": 7003,
                "inputMissionPlans": [
                    {
                        "timestamp": 1,
                        "inputMissionList": [
                            {"inputMissionID": 101, "inputMissionType": 1, "regionType": 4}
                        ],
                    }
                ],
                "individualMissionPlans": [
                    {
                        "aircraftID": 6,
                        "individualMissionList": [
                            {
                                "individualMissionID": 9001,
                                "pathID": 6001,
                                "isDone": False,
                                "relatedMission": {"inputMissionID": 101},
                            }
                        ],
                    }
                ],
                "flightPaths": [
                    {
                        "pathID": 6001,
                        "aircraftID": 6,
                        "waypointList": [
                            {
                                "waypointID": 11562,
                                "nextWaypointID": 0,
                                "isDone": False,
                                "waypointPassType": 2,
                                "coordinate": {
                                    "latitude": 38.001,
                                    "longitude": 127.001,
                                    "altitude": 300.0,
                                },
                                "loiterProperty": {
                                    "radius": 400.0,
                                    "direction": 1,
                                    "time": 61.0,
                                    "speed": 30.0,
                                },
                                "filmingProperty": {
                                    "operationMode": 3,
                                    "autoTracking": {"targetID": 8},
                                },
                            }
                        ],
                    }
                ],
            }
        )
        self.assertTrue(initial["ok"])
        self.assertEqual(service.vehicles["UAV3"].controller.current_target().wp_id, 11562)

        replanned = service.load_mission(
            {
                "missionPlanID": 7004,
                "preserveState": True,
                "inputMissionPlans": [
                    {
                        "timestamp": 2,
                        "inputMissionList": [
                            {"inputMissionID": 101, "inputMissionType": 1, "regionType": 4}
                        ],
                    }
                ],
                "individualMissionPlans": [
                    {
                        "aircraftID": 6,
                        "individualMissionList": [
                            {
                                "individualMissionID": 9002,
                                "pathID": 6002,
                                "isDone": False,
                                "relatedMission": {"inputMissionID": 101},
                            }
                        ],
                    }
                ],
                "flightPaths": [
                    {
                        "pathID": 6002,
                        "aircraftID": 6,
                        "waypointList": [
                            {
                                "waypointID": 12337,
                                "nextWaypointID": 0,
                                "isDone": False,
                                "waypointPassType": 1,
                                "coordinate": {
                                    "latitude": 38.047131,
                                    "longitude": 127.272403,
                                    "altitude": 1423.0,
                                },
                                "filmingProperty": {
                                    "operationMode": 2,
                                    "lineSearch": {
                                        "coordinateList": [
                                            {
                                                "latitude": 38.047131,
                                                "longitude": 127.272403,
                                                "altitude": 120.0,
                                            },
                                            {
                                                "latitude": 38.048000,
                                                "longitude": 127.273000,
                                                "altitude": 120.0,
                                            },
                                        ]
                                    },
                                },
                            }
                        ],
                    }
                ],
            }
        )

        self.assertTrue(replanned["ok"])
        self.assertIn("UAV3", service.vehicles)
        current = service.vehicles["UAV3"].controller.current_target()
        self.assertEqual(current.wp_id, 12337)
        self.assertEqual(current.path_id, 6002)
        self.assertEqual(current.filming["operationMode"], 2)


if __name__ == "__main__":
    unittest.main()
