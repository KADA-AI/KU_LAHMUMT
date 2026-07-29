from __future__ import annotations

import unittest
import threading
from copy import deepcopy
from types import SimpleNamespace
from unittest.mock import patch

from modules.monitoring.logic.boundary_guard_progress import (
    BoundaryGuardProgressGate,
)
from modules.monitoring.logic.mission_progress import MissionProgressTracker
from modules.monitoring.logic.mission_update import extract_0401_agent_states
from modules.monitoring.logic import mission_update
from modules.monitoring.gui.tabs.monitoring_visualization_tab import (
    _recommendation_state_after_plan_switch,
)


INPUT_ID = 700000005


def _guard_view(
    *,
    aircraft_ids: tuple[int, ...] = (4, 5, 6),
    duration_s: float = 10.0,
) -> dict[str, object]:
    entries: list[dict[str, object]] = []
    for index, aircraft_id in enumerate(aircraft_ids):
        set_id = f"guard:{aircraft_id}"
        first_wp = 1000 + index * 10
        last_wp = first_wp + 1
        entries.append(
            {
                "aircraft_id": aircraft_id,
                "individual_mission_package_id": 8000 + aircraft_id,
                "current_individual_mission_id": 9000 + aircraft_id,
                "missions": [
                    {
                        "individual_mission_id": 9000 + aircraft_id,
                        "input_id": INPUT_ID,
                        "input_mission_type": 3,
                        "region_type": 7,
                        "eta_seconds": 2.0,
                        "is_done": False,
                        "boundary_guard_loop": True,
                        "boundary_guard_loop_version": 1,
                        "boundary_guard_set_id": set_id,
                        "boundary_guard_sequence": 1,
                        "boundary_guard_sequence_count": 1,
                        "boundary_guard_duration_s": duration_s,
                        "boundary_guard_cycle_first_waypoint_id": first_wp,
                        "boundary_guard_cycle_last_waypoint_id": last_wp,
                        "waypoints": [
                            {
                                "waypoint_id": first_wp,
                                "eta": 1.0,
                                "line_search_point_count": 1,
                            },
                            {
                                "waypoint_id": last_wp,
                                "eta": 2.0,
                                "line_search_point_count": 1,
                            },
                        ],
                    }
                ],
            }
        )
    return {
        "mission_plan_id": 7001,
        "input_mission_package_id": 6001,
        "input_missions": [
            {
                "input_mission_id": INPUT_ID,
                "input_mission_type": 3,
                "region_type": 7,
                "is_done": False,
            }
        ],
        "uav_entries": entries,
    }


def _state(
    aircraft_id: int,
    *,
    cycle_count: int,
    filming: int = 1,
    flight_mode: int = 8,
    waypoint_offset: int = 0,
) -> dict[str, object]:
    index = aircraft_id - 4
    return {
        "aircraft_id": aircraft_id,
        "current_waypoint_id": 1000 + index * 10 + waypoint_offset,
        "flying": 1,
        "filming": filming,
        "flight_mode": flight_mode,
        "boundary_guard_set_id": f"guard:{aircraft_id}",
        "boundary_guard_cycle_count": cycle_count,
        "boundary_guard_loop_active": True,
    }


class BoundaryGuardGateTests(unittest.TestCase):
    def test_clock_starts_only_when_guard_is_really_filming(self) -> None:
        gate = BoundaryGuardProgressGate(default_duration_s=10.0)
        gate.configure(_guard_view(aircraft_ids=(4,)))

        gate.update(
            timestamp_ms=1_000,
            agent_states=[_state(4, cycle_count=0, filming=0)],
        )
        self.assertIsNone(gate.status(INPUT_ID)["started_at_ms"])

        gate.update(
            timestamp_ms=2_000,
            agent_states=[_state(4, cycle_count=0, filming=1)],
        )
        self.assertEqual(gate.status(INPUT_ID)["started_at_ms"], 2_000)

    def test_duration_and_every_set_first_cycle_are_both_required(self) -> None:
        gate = BoundaryGuardProgressGate(default_duration_s=10.0)
        gate.configure(_guard_view())
        gate.update(
            timestamp_ms=1_000,
            agent_states=[
                _state(4, cycle_count=0),
                _state(5, cycle_count=0),
                _state(6, cycle_count=0),
            ],
        )
        gate.update(
            timestamp_ms=12_000,
            agent_states=[
                _state(4, cycle_count=1),
                _state(5, cycle_count=1),
                _state(6, cycle_count=0),
            ],
        )
        self.assertFalse(gate.is_ready(INPUT_ID))
        self.assertEqual(gate.status(INPUT_ID)["phase"], "waiting_first_cycle")

        gate.update(
            timestamp_ms=12_100,
            agent_states=[
                _state(4, cycle_count=1),
                _state(5, cycle_count=1),
                _state(6, cycle_count=1),
            ],
        )
        self.assertTrue(gate.is_ready(INPUT_ID))

    def test_tracking_blocks_ready_but_does_not_pause_wall_clock(self) -> None:
        gate = BoundaryGuardProgressGate(default_duration_s=10.0)
        gate.configure(_guard_view(aircraft_ids=(4,)))
        gate.update(
            timestamp_ms=1_000,
            agent_states=[_state(4, cycle_count=0)],
        )
        gate.update(
            timestamp_ms=12_000,
            agent_states=[
                _state(4, cycle_count=1, flight_mode=9, waypoint_offset=1)
            ],
            active_tracking_assignments=[
                {
                    "aircraft_id": 4,
                    "current_input_mission_id": INPUT_ID,
                    "active": True,
                }
            ],
        )
        status = gate.status(INPUT_ID)
        self.assertGreaterEqual(status["elapsed_s"], 11.0)
        self.assertFalse(status["ready"])
        self.assertEqual(status["phase"], "waiting_tracking")

        gate.update(
            timestamp_ms=12_100,
            agent_states=[_state(4, cycle_count=1)],
        )
        self.assertTrue(gate.is_ready(INPUT_ID))

    def test_reconfigure_replaces_removed_set_requirements(self) -> None:
        gate = BoundaryGuardProgressGate(default_duration_s=10.0)
        gate.configure(_guard_view())
        gate.update(
            timestamp_ms=1_000,
            agent_states=[
                _state(4, cycle_count=1),
                _state(5, cycle_count=0),
                _state(6, cycle_count=0),
            ],
        )

        gate.configure(_guard_view(aircraft_ids=(4,)))
        status = gate.status(INPUT_ID)
        self.assertEqual(status["expected_set_ids"], ["guard:4"])
        self.assertEqual(status["cycle_count_by_set"], {"guard:4": 1})

    def test_already_completed_guard_input_is_not_rearmed_on_plan_load(self) -> None:
        view = _guard_view(aircraft_ids=(4,))
        view["input_missions"][0]["is_done"] = True
        gate = BoundaryGuardProgressGate(default_duration_s=10.0)
        gate.configure(view)

        self.assertFalse(gate.is_guard_input(INPUT_ID))
        self.assertIsNone(gate.status(INPUT_ID))

    def test_explicit_reexecute_rearms_clock_and_cycle_baseline(self) -> None:
        gate = BoundaryGuardProgressGate(default_duration_s=10.0)
        gate.configure(_guard_view(aircraft_ids=(4,)))
        gate.update(
            timestamp_ms=1_000,
            agent_states=[_state(4, cycle_count=2)],
        )
        gate.update(
            timestamp_ms=12_000,
            agent_states=[_state(4, cycle_count=2)],
        )
        self.assertTrue(gate.is_ready(INPUT_ID))

        self.assertTrue(gate.reset_input(INPUT_ID))
        gate.update(
            timestamp_ms=13_000,
            agent_states=[_state(4, cycle_count=2)],
        )
        gate.update(
            timestamp_ms=24_000,
            agent_states=[_state(4, cycle_count=2)],
        )
        self.assertFalse(gate.is_ready(INPUT_ID))
        self.assertEqual(
            gate.status(INPUT_ID)["cycle_count_by_set"],
            {"guard:4": 0},
        )

        gate.update(
            timestamp_ms=24_100,
            agent_states=[_state(4, cycle_count=3)],
        )
        self.assertTrue(gate.is_ready(INPUT_ID))


class BoundaryGuardMissionProgressTests(unittest.TestCase):
    def test_gate_completes_normal_monitor_path_without_flying_two(self) -> None:
        tracker = MissionProgressTracker()
        tracker.reset(_guard_view())
        with patch(
            "modules.monitoring.logic.mission_progress.list_active_tracking_assignments",
            return_value=[],
        ):
            first = tracker.update(
                1_000,
                [
                    _state(4, cycle_count=0),
                    _state(5, cycle_count=0),
                    _state(6, cycle_count=0),
                ],
            )
            self.assertEqual(
                first["input_progress"][INPUT_ID]["planned_seconds"],
                10,
            )
            self.assertFalse(first["input_progress"][INPUT_ID]["done"])

            completed = tracker.update(
                12_000,
                [
                    _state(4, cycle_count=1),
                    _state(5, cycle_count=1),
                    _state(6, cycle_count=1),
                ],
            )

        self.assertEqual(completed["new_completed_input"], [INPUT_ID])
        self.assertTrue(completed["input_progress"][INPUT_ID]["done"])
        self.assertEqual(
            completed["input_progress"][INPUT_ID]["progress_percent"],
            100,
        )
        self.assertEqual(
            len(completed["new_completed_individual"]),
            3,
        )

    def test_legacy_flying_two_cannot_finish_guard_before_gate(self) -> None:
        tracker = MissionProgressTracker()
        tracker.reset(_guard_view(aircraft_ids=(4,)))
        row = _state(4, cycle_count=0)
        row["flying"] = 2
        row["filming"] = 2
        snapshot = tracker.update(1_000, [row])

        self.assertFalse(snapshot["input_progress"][INPUT_ID]["done"])
        self.assertEqual(snapshot["new_completed_input"], [])
        self.assertEqual(snapshot["new_completed_individual"], [])

    def test_destroyed_target_replan_then_elapsed_guard_completes_immediately(self) -> None:
        tracker = MissionProgressTracker()
        source_view = _guard_view(aircraft_ids=(4,))
        tracker.reset(source_view)
        active_assignments = []

        with patch(
            "modules.monitoring.logic.mission_progress.list_active_tracking_assignments",
            side_effect=lambda: list(active_assignments),
        ):
            tracker.update(1_000, [_state(4, cycle_count=0)])
            tracker.update(9_000, [_state(4, cycle_count=1)])

            active_assignments[:] = [
                {
                    "aircraft_id": 4,
                    "current_input_mission_id": INPUT_ID,
                    "active": True,
                }
            ]
            blocked = tracker.update(
                12_000,
                [_state(4, cycle_count=1, flight_mode=9, waypoint_offset=1)],
            )
            self.assertEqual(blocked["new_completed_input"], [])
            self.assertEqual(
                blocked["input_progress"][INPUT_ID]["boundary_guard"]["phase"],
                "waiting_tracking",
            )

            # attackClosedDestroyed writes/applies a new MissionPlan but keeps
            # the same portable guard-set identity and InputMissionPackage.
            post_attack_view = deepcopy(source_view)
            post_attack_view["mission_plan_id"] = 7002
            tracker.reset(post_attack_view)
            active_assignments.clear()
            completed = tracker.update(
                12_100,
                [_state(4, cycle_count=0)],
            )

        self.assertEqual(completed["new_completed_input"], [INPUT_ID])
        self.assertTrue(completed["input_progress"][INPUT_ID]["done"])
        self.assertEqual(
            completed["input_progress"][INPUT_ID]["progress_percent"],
            100,
        )


class BoundaryGuardRecommendationCarryTests(unittest.TestCase):
    def test_unsent_completion_survives_same_package_post_attack_plan_switch(self) -> None:
        state = _recommendation_state_after_plan_switch(
            same_input_package=True,
            input_ids={INPUT_ID, INPUT_ID + 1},
            done_input_ids={INPUT_ID},
            observed_completion_inputs={INPUT_ID},
            observed_execute_ready_inputs=set(),
            pending_completion_inputs=[INPUT_ID],
            pending_execute_inputs=[],
            sent_completion_inputs=set(),
            sent_execute_inputs=set(),
        )

        self.assertEqual(state["pending_completion_inputs"], [INPUT_ID])
        self.assertNotIn(INPUT_ID, state["sent_completion_inputs"])

    def test_already_sent_completion_is_not_requeued_after_plan_switch(self) -> None:
        state = _recommendation_state_after_plan_switch(
            same_input_package=True,
            input_ids={INPUT_ID, INPUT_ID + 1},
            done_input_ids={INPUT_ID},
            observed_completion_inputs={INPUT_ID},
            observed_execute_ready_inputs=set(),
            pending_completion_inputs=[],
            pending_execute_inputs=[],
            sent_completion_inputs={INPUT_ID},
            sent_execute_inputs=set(),
        )

        self.assertEqual(state["pending_completion_inputs"], [])
        self.assertIn(INPUT_ID, state["sent_completion_inputs"])

    def test_different_input_package_drops_old_pending_recommendation(self) -> None:
        state = _recommendation_state_after_plan_switch(
            same_input_package=False,
            input_ids={INPUT_ID},
            done_input_ids={INPUT_ID},
            observed_completion_inputs={INPUT_ID},
            observed_execute_ready_inputs=set(),
            pending_completion_inputs=[INPUT_ID],
            pending_execute_inputs=[],
            sent_completion_inputs=set(),
            sent_execute_inputs=set(),
        )

        self.assertEqual(state["pending_completion_inputs"], [])
        self.assertEqual(state["sent_completion_inputs"], {INPUT_ID})


class BoundaryGuardAreaWorkerQueueTests(unittest.TestCase):
    def test_0401_normalization_preserves_nested_guard_cycle_fields(self) -> None:
        timestamp_ms, rows = extract_0401_agent_states(
            {
                "timestamp": 12_345,
                "agentStateList": [
                    {
                        "aircraftID": 4,
                        "unmannedInfo": {
                            "boundaryGuardSetID": "guard:4",
                            "boundaryGuardCycleCount": 3,
                            "boundaryGuardLoopActive": True,
                        },
                    }
                ],
            }
        )

        self.assertEqual(timestamp_ms, 12_345)
        self.assertEqual(rows[0]["boundary_guard_set_id"], "guard:4")
        self.assertEqual(rows[0]["boundary_guard_cycle_count"], 3)
        self.assertTrue(rows[0]["boundary_guard_loop_active"])

    def test_mission_view_preserves_portable_guard_contract(self) -> None:
        coordinates = [
            {"latitude": 35.0, "longitude": 127.0, "altitude": 100.0},
            {"latitude": 35.0, "longitude": 127.01, "altitude": 100.0},
            {"latitude": 35.01, "longitude": 127.01, "altitude": 100.0},
        ]
        payloads = {
            ("MissionPlan", 7001): {
                "inputMissionPackageID": 6001,
                "aircraftList": [
                    {
                        "aircraftID": 4,
                        "individualMissionPackageID": 8004,
                    }
                ],
            },
            ("InputMissionPlan", 6001): {
                "inputMissionList": [
                    {
                        "inputMissionID": INPUT_ID,
                        "inputMissionType": 3,
                        "regionType": 7,
                        "isDone": False,
                        "missionDetail": {
                            "areaList": [{"coordinateList": coordinates}]
                        },
                    }
                ]
            },
            ("IndividualMissionPlan", 8004): {
                "individualMissionList": [
                    {
                        "individualMissionID": 9004,
                        "pathID": 5004,
                        "isDone": False,
                        "relatedMission": {"inputMissionID": INPUT_ID},
                        "individualMissionInfo": {
                            "individualMissionType": 3,
                            "areaList": [{"coordinateList": coordinates}],
                        },
                    }
                ]
            },
            ("FlightPath", 5004): {
                "boundaryGuardLoop": True,
                "boundaryGuardLoopVersion": 1,
                "boundaryGuardSetID": "guard:4",
                "boundaryGuardSequence": 1,
                "boundaryGuardSequenceCount": 1,
                "boundaryGuardDurationS": 600.0,
                "boundaryGuardCycleFirstWaypointID": 100,
                "boundaryGuardCycleLastWaypointID": 101,
                "waypointList": [
                    {
                        "waypointID": 100,
                        "eta": 1.0,
                        "coordinate": coordinates[0],
                    },
                    {
                        "waypointID": 101,
                        "eta": 2.0,
                        "coordinate": coordinates[1],
                    },
                ],
            },
        }
        with patch.object(
            mission_update,
            "load_db_json",
            side_effect=lambda folder, identifier, db_root=None: payloads.get(
                (folder, identifier),
                {},
            ),
        ):
            view = mission_update.build_uav_mission_view(
                7001,
                uav_ids=(4,),
            )

        self.assertEqual(view["input_missions"][0]["region_type"], 7)
        mission = view["uav_entries"][0]["missions"][0]
        self.assertTrue(mission["boundary_guard_loop"])
        self.assertEqual(mission["boundary_guard_loop_version"], 1)
        self.assertEqual(mission["boundary_guard_set_id"], "guard:4")
        self.assertEqual(mission["boundary_guard_duration_s"], 600.0)
        self.assertEqual(
            mission["boundary_guard_cycle_first_waypoint_id"],
            100,
        )
        self.assertEqual(
            mission["boundary_guard_cycle_last_waypoint_id"],
            101,
        )

    def test_queue_preserves_guard_cycle_contract_fields(self) -> None:
        from modules.monitoring.monitoring_gui import MainWindow

        target = SimpleNamespace(
            _area_snapshot_enabled=True,
            _area_snapshot_lock=threading.Lock(),
            _area_snapshot_latest_status=None,
            _area_snapshot_event=threading.Event(),
        )
        MainWindow._queue_area_snapshot_status_update(
            target,
            timestamp_ms=12_345,
            agent_states=[
                {
                    "aircraft_id": 4,
                    "boundary_guard_set_id": "guard:4",
                    "boundary_guard_cycle_count": 2,
                    "boundary_guard_loop_active": True,
                    "ignored": "value",
                }
            ],
        )

        timestamp_ms, rows = target._area_snapshot_latest_status
        self.assertEqual(timestamp_ms, 12_345)
        self.assertEqual(
            rows,
            [
                {
                    "aircraft_id": 4,
                    "boundary_guard_set_id": "guard:4",
                    "boundary_guard_cycle_count": 2,
                    "boundary_guard_loop_active": True,
                }
            ],
        )


if __name__ == "__main__":
    unittest.main()
