from __future__ import annotations

import tempfile
import unittest
from copy import deepcopy
from pathlib import Path
from unittest.mock import patch

from modules.mission_planning.replanning.triggers.post_attack.pipeline import (
    _POST_ATTACK_COMPLETE_HOLD_SECONDS,
    _allow_active_suffix_latest_plan_fallback,
    _apply_collab_unavailable_return_only_fallback,
    _apply_post_attack_collab_entry_policy,
    _build_post_attack_tracking_return_only_update,
    _can_resume_line_directly_after_attack,
    _finalize_post_attack_tracking_boundary_guard_graph,
    _line_scan_progress_entry_is_current,
    _mark_post_attack_followups_execution_blocked,
    _post_attack_authoritative_source_plan_id,
    _post_attack_follow_up_source_missions,
    _restore_type2_line_carriers_from_original,
    _resolve_imaging_entry_flight_coordinate,
)
from modules.mission_planning.pipelines.type2_boundary_guard_loop import (
    BOUNDARY_GUARD_CONTRACT_KEYS,
    apply_boundary_guard_contract,
    boundary_guard_contract,
    validate_boundary_guard_flight_path_sets,
)
from modules.sim.runtime.sim_service import (
    _mission_execution_blocked_until_next_collab,
)


def _mission(mission_id: int, input_id: int) -> dict[str, object]:
    return {
        "individualMissionID": int(mission_id),
        "isDone": False,
        "relatedMission": {
            "relatedMissionType": 1,
            "inputMissionID": int(input_id),
            "priorMissionID": 0,
        },
        "pathID": int(mission_id) * 10,
    }


class PostAttackFollowupRetentionTests(unittest.TestCase):
    def test_pending_line_scan_progress_is_not_treated_as_live_resume_progress(
        self,
    ) -> None:
        pending_entry = {
            "progress_source": "line_scan",
            "is_current": False,
            "line_scan": {
                "source": "line_scan_progress_monitor",
                "isCurrent": False,
                "sweepPointCount": 124,
                "progressPoints": 89,
            },
        }
        missing_marker_entry = {
            "progress_source": "line_scan",
            "line_scan": {
                "source": "line_scan_progress_monitor",
                "sweepPointCount": 124,
                "progressPoints": 89,
            },
        }
        current_entry = deepcopy(pending_entry)
        current_entry["is_current"] = True
        current_entry["line_scan"]["isCurrent"] = True

        self.assertFalse(_line_scan_progress_entry_is_current(pending_entry))
        self.assertFalse(_line_scan_progress_entry_is_current(missing_marker_entry))
        self.assertTrue(_line_scan_progress_entry_is_current(current_entry))

    def test_type2_active_suffix_rejects_newer_unselected_plan_fallback(
        self,
    ) -> None:
        self.assertFalse(
            _allow_active_suffix_latest_plan_fallback(
                force_type2_individual_suffix_refresh=True,
            )
        )
        self.assertTrue(
            _allow_active_suffix_latest_plan_fallback(
                force_type2_individual_suffix_refresh=False,
            )
        )

    @staticmethod
    def _single_capture_rejoin_path() -> dict[str, object]:
        entry = {
            "waypointID": 101,
            "nextWaypointID": 102,
            "coordinate": {
                "latitude": 37.1,
                "longitude": 127.1,
                "altitude": 900,
            },
            "filmingProperty": {
                "operationMode": 1,
                "coordinateOrientation": {
                    "coordinate": {
                        "latitude": 37.1005,
                        "longitude": 127.1005,
                        "altitude": 0,
                    }
                },
            },
        }
        capture = {
            "waypointID": 102,
            "nextWaypointID": 0,
            "coordinate": {
                "latitude": 37.11,
                "longitude": 127.11,
                "altitude": 900,
            },
            "filmingProperty": {
                "operationMode": 2,
                "lineSearch": {
                    "coordinateList": [
                        {
                            "latitude": 37.1005,
                            "longitude": 127.1005,
                            "altitude": 0,
                        },
                        {
                            "latitude": 37.1105,
                            "longitude": 127.1105,
                            "altitude": 0,
                        },
                    ],
                    "searchSpeed": 25,
                },
            },
        }
        waypoints = [entry, capture]
        return {
            "pathID": 500000001,
            "waypointList": deepcopy(waypoints),
            "lahWaypointList": deepcopy(waypoints),
        }

    def test_returning_uav_keeps_assigned_area_entry_before_single_capture_wp(self) -> None:
        payload = self._single_capture_rejoin_path()
        messages: list[str] = []

        result = _apply_post_attack_collab_entry_policy(
            4,
            500000001,
            payload,
            returning_aircraft_ids={4},
            emit=messages.append,
        )

        waypoints = result["waypointList"]
        self.assertEqual([waypoint["waypointID"] for waypoint in waypoints], [101, 102])
        self.assertNotIn("lineSearch", waypoints[0]["filmingProperty"])
        self.assertEqual(
            len(waypoints[1]["filmingProperty"]["lineSearch"]["coordinateList"]),
            2,
        )
        self.assertEqual(result["lahWaypointList"], waypoints)
        self.assertTrue(
            any("assigned-area entry waypoint retained" in message for message in messages)
        )

    def test_returning_uav_keeps_one_entry_before_multiple_capture_wps(self) -> None:
        payload = self._single_capture_rejoin_path()
        second_capture = deepcopy(payload["waypointList"][-1])
        second_capture["waypointID"] = 103
        second_capture["nextWaypointID"] = 0
        second_capture["coordinate"]["latitude"] = 37.12
        payload["waypointList"][-1]["nextWaypointID"] = 103
        payload["waypointList"].append(second_capture)
        payload["lahWaypointList"] = deepcopy(payload["waypointList"])

        result = _apply_post_attack_collab_entry_policy(
            4,
            500000001,
            payload,
            returning_aircraft_ids={4},
            emit=lambda _message: None,
        )

        waypoints = result["waypointList"]
        self.assertEqual(
            [waypoint["waypointID"] for waypoint in waypoints],
            [101, 102, 103],
        )
        self.assertEqual(
            sum(
                1
                for waypoint in waypoints
                if "lineSearch" not in waypoint["filmingProperty"]
            ),
            1,
        )
        self.assertTrue(
            all(
                "lineSearch" in waypoint["filmingProperty"]
                for waypoint in waypoints[1:]
            )
        )

    def test_stale_line_carrier_anchor_moves_to_capture_area_at_flight_altitude(self) -> None:
        payload = self._single_capture_rejoin_path()
        payload["waypointList"] = [payload["waypointList"][-1]]
        capture = payload["waypointList"][0]
        capture["coordinate"] = {
            "latitude": 37.0,
            "longitude": 127.0,
            "altitude": 1450,
        }
        capture["filmingProperty"]["lineSearch"]["coordinateList"] = [
            {"latitude": 37.03, "longitude": 127.03, "altitude": 210},
            {"latitude": 37.04, "longitude": 127.04, "altitude": 220},
        ]

        entry, capture_index, corrected, offset_m = (
            _resolve_imaging_entry_flight_coordinate(payload["waypointList"])
        )

        self.assertTrue(corrected)
        self.assertEqual(capture_index, 0)
        self.assertGreater(offset_m, 2000.0)
        self.assertEqual(
            entry,
            {"latitude": 37.03, "longitude": 127.03, "altitude": 1450},
        )

    def test_active_uav_still_consumes_entry_before_single_capture_wp(self) -> None:
        payload = self._single_capture_rejoin_path()

        result = _apply_post_attack_collab_entry_policy(
            5,
            500000001,
            payload,
            returning_aircraft_ids={4},
            emit=lambda _message: None,
        )

        waypoints = result["waypointList"]
        self.assertEqual(len(waypoints), 1)
        self.assertEqual(waypoints[0]["waypointID"], 102)
        self.assertIn("lineSearch", waypoints[0]["filmingProperty"])
        self.assertEqual(result["lahWaypointList"], waypoints)

    def test_collab_failure_replaces_destroyed_tracking_package_with_return_only(self) -> None:
        plan = {
            "aircraftList": [
                {
                    "aircraftID": 6,
                    "individualMissionPackageID": 800000042,
                }
            ]
        }
        release_update = {
            "aircraft_id": 6,
            "individualMissionPackageID": 800000046,
            "generatedPathIDs": [600000046],
            "reservationSummaries": [{"scope": "test-return-only"}],
        }
        reservations: list[dict[str, object]] = []

        with patch(
            "modules.mission_planning.replanning.triggers.post_attack.pipeline."
            "_build_post_attack_tracking_return_only_update",
            return_value=release_update,
        ) as build_return:
            result = _apply_collab_unavailable_return_only_fallback(
                attack_plan_id=700000005,
                current_input_id=1,
                group_assignments=[
                    {
                        "aircraft_id": 6,
                        "tracking_individual_mission_id": 900000250,
                        "resume_individual_mission_id": 900000251,
                        "tracking_path_id": 600000043,
                        "resume_path_id": 600000044,
                    }
                ],
                agent_state_map={
                    6: {
                        "coordinate": {
                            "latitude": 38.0223535,
                            "longitude": 127.3027138,
                            "altitude": 1697,
                        }
                    }
                },
                new_plan_data=plan,
                now_ms=1,
                emit=lambda _message: None,
                run_cache=None,
                reservation_summaries=reservations,
            )

        self.assertEqual(
            plan["aircraftList"][0]["individualMissionPackageID"],
            800000046,
        )
        self.assertEqual(result["released_aircraft_ids"], [6])
        self.assertEqual(result["failed_aircraft_ids"], [])
        self.assertEqual(result["generated_imp_ids"], [800000046])
        self.assertEqual(result["generated_path_ids"], [600000046])
        self.assertEqual(reservations, [{"scope": "test-return-only"}])
        self.assertTrue(
            build_return.call_args.kwargs["block_follow_up_until_reassignment"]
        )
        self.assertEqual(
            build_return.call_args.kwargs["hold_seconds"],
            int(_POST_ATTACK_COMPLETE_HOLD_SECONDS),
        )

    def test_follow_up_source_missions_are_not_deleted_at_collaboration_boundary(self) -> None:
        missions = [
            _mission(1, 70000000),
            _mission(2, 70000001),
            _mission(3, 70000002),
        ]

        follow_ups = _post_attack_follow_up_source_missions(missions, 1)

        self.assertEqual(
            [row["individualMissionID"] for row in follow_ups],
            [2, 3],
        )

    def test_future_missions_are_retained_but_execution_blocked(self) -> None:
        current_resume = _mission(1, 70000000)
        future_one = _mission(2, 70000001)
        future_two = _mission(3, 70000002)
        missions = [current_resume, future_one, future_two]

        blocked = _mark_post_attack_followups_execution_blocked(
            missions,
            current_input_id=70000000,
        )

        self.assertEqual(blocked, 2)
        self.assertNotIn("executionBlockedUntilNextCollab", current_resume)
        self.assertTrue(future_one["executionBlockedUntilNextCollab"])
        self.assertTrue(future_two["executionBlockedUntilNextCollab"])
        self.assertFalse(_mission_execution_blocked_until_next_collab(current_resume))
        self.assertTrue(_mission_execution_blocked_until_next_collab(future_one))

    def test_tracking_return_excludes_transit_clone_from_boundary_guard_count(
        self,
    ) -> None:
        """Reproduce the latest 5-declared/6-present post-attack failure."""

        set_id = "type2-boundary:3:5:aircraft-5"
        guard_missions: list[dict[str, object]] = []
        guard_paths: list[dict[str, object]] = []
        for sequence in range(1, 6):
            mission_id = 900000100 + sequence
            path_id = 500000100 + sequence
            first_waypoint_id = 1000 + sequence * 10
            contract = boundary_guard_contract(
                set_id=set_id,
                sequence=sequence,
                sequence_count=5,
                duration_s=600,
            )
            mission = _mission(mission_id, 5)
            mission["aircraftID"] = 5
            mission["pathID"] = path_id
            mission["individualMissionInfo"] = {
                "individualMissionType": 5,
                "coordinateList": [],
            }
            apply_boundary_guard_contract(
                mission,
                contract,
                include_individual_mission_info=True,
            )
            flight_path = {
                "pathID": path_id,
                "aircraftID": 5,
                "individualMissionID": mission_id,
                "waypointList": [
                    {
                        "waypointID": first_waypoint_id,
                        "nextWaypointID": first_waypoint_id + 1,
                        "eta": 0,
                    },
                    {
                        "waypointID": first_waypoint_id + 1,
                        "nextWaypointID": 0,
                        "eta": 0,
                    },
                ],
            }
            apply_boundary_guard_contract(flight_path, contract)
            guard_missions.append(mission)
            guard_paths.append(flight_path)

        # The transit-only return is cloned from the current AREA template,
        # so it starts with the same boundary metadata and caused validation
        # to see a sixth child in the five-child owner set.
        return_mission = deepcopy(guard_missions[0])
        return_mission["individualMissionID"] = 900000199
        return_mission["pathID"] = 500000199
        return_mission["individualMissionInfo"]["individualMissionType"] = 7
        return_path = deepcopy(guard_paths[0])
        return_path["pathID"] = 500000199
        return_path["individualMissionID"] = 900000199
        return_path["waypointList"] = [
            {"waypointID": 1999, "nextWaypointID": 0, "eta": 0}
        ]

        summary = _finalize_post_attack_tracking_boundary_guard_graph(
            missions=[return_mission, *guard_missions],
            flight_paths=[return_path, *guard_paths],
            synthetic_return_path_id=500000199,
            emit=lambda _message: None,
            log_prefix="[TEST]",
        )

        for key in BOUNDARY_GUARD_CONTRACT_KEYS:
            self.assertNotIn(key, return_mission)
            self.assertNotIn(key, return_mission["individualMissionInfo"])
            self.assertNotIn(key, return_path)
        self.assertEqual(summary[set_id]["sequenceCount"], 5)
        self.assertEqual(
            [mission["boundaryGuardSequence"] for mission in guard_missions],
            [1, 2, 3, 4, 5],
        )
        self.assertEqual(
            {mission["boundaryGuardSequenceCount"] for mission in guard_missions},
            {5},
        )
        for index, flight_path in enumerate(guard_paths):
            expected_next = guard_paths[(index + 1) % len(guard_paths)][
                "waypointList"
            ][0]["waypointID"]
            self.assertEqual(
                flight_path["waypointList"][-1]["nextWaypointID"],
                expected_next,
            )
        self.assertEqual(guard_paths[-1]["waypointList"][-1]["eta"], 600)
        validate_boundary_guard_flight_path_sets(guard_paths)

    def test_post_attack_remaining_geometry_uses_current_applied_plan(self) -> None:
        self.assertEqual(
            _post_attack_authoritative_source_plan_id(700000015, 700000018),
            700000018,
        )
        self.assertEqual(
            _post_attack_authoritative_source_plan_id(700000015, 0),
            700000015,
        )

    def test_direct_line_resume_rule_applies_to_general_line_and_not_area(self) -> None:
        line_mission = _mission(900000010, 4)
        line_mission["individualMissionInfo"] = {
            "individualMissionType": 6,
            "patternType": 8,
        }
        line_waypoints = [
            {
                "waypointID": 100,
                "filmingProperty": {
                    "operationMode": 2,
                    "lineSearch": {
                        "coordinateList": [
                            {"latitude": 38.0, "longitude": 127.0, "altitude": 0},
                            {"latitude": 38.01, "longitude": 127.01, "altitude": 0},
                        ]
                    },
                },
            }
        ]

        self.assertTrue(
            _can_resume_line_directly_after_attack(
                line_mission,
                line_waypoints,
                current_input_id=4,
                block_follow_up_until_reassignment=False,
            )
        )
        # The live current LINE stays executable even while future inputs are held.
        self.assertTrue(
            _can_resume_line_directly_after_attack(
                line_mission,
                line_waypoints,
                current_input_id=4,
                block_follow_up_until_reassignment=True,
            )
        )

        future_line = deepcopy(line_mission)
        future_line["relatedMission"]["inputMissionID"] = 5
        self.assertFalse(
            _can_resume_line_directly_after_attack(
                future_line,
                line_waypoints,
                current_input_id=4,
                block_follow_up_until_reassignment=True,
            )
        )

        area_mission = deepcopy(line_mission)
        area_mission["individualMissionInfo"]["individualMissionType"] = 3
        self.assertFalse(
            _can_resume_line_directly_after_attack(
                area_mission,
                line_waypoints,
                current_input_id=4,
                block_follow_up_until_reassignment=False,
            )
        )

    def test_tracking_release_return_uses_same_terminal_boundary_loiter(self) -> None:
        imp = {
            "individualMissionPackageID": 800000001,
            "individualMissionList": [
                {
                    "individualMissionID": 900000001,
                    "isDone": False,
                    "relatedMission": {"inputMissionID": 70000001},
                    "individualMissionInfo": {
                        "individualMissionType": 1,
                        "targetID": 8,
                    },
                    "pathID": 400000001,
                },
                {
                    "individualMissionID": 900000002,
                    "isDone": False,
                    "relatedMission": {"inputMissionID": 70000001},
                    "individualMissionInfo": {
                        "individualMissionType": 7,
                        "targetID": None,
                    },
                    "pathID": 400000002,
                },
                {
                    "individualMissionID": 900000004,
                    "isDone": False,
                    "relatedMission": {"inputMissionID": 70000002},
                    "individualMissionInfo": {
                        "individualMissionType": 6,
                        "targetID": None,
                    },
                    "pathID": 400000004,
                },
            ],
        }
        tracking_path = {
            "pathID": 400000001,
            "aircraftID": 4,
            "individualMissionID": 900000001,
            "waypointList": [
                {
                    "waypointID": 101,
                    "coordinate": {
                        "latitude": 38.0,
                        "longitude": 127.0,
                        "altitude": 1200,
                    },
                    "isDone": True,
                }
            ],
        }
        resume_path = {
            "pathID": 400000002,
            "aircraftID": 4,
            "individualMissionID": 900000002,
            "waypointList": [
                {
                    "waypointID": 102,
                    "coordinate": {
                        # Deliberately far away: with no capture remaining this
                        # stale resume endpoint must not create a two-WP return.
                        "latitude": 38.1,
                        "longitude": 127.1,
                        "altitude": 1200,
                    },
                    "waypointPassType": 3,
                    "loiterProperty": {},
                    "isDone": False,
                }
            ],
        }

        class _Reservation:
            def __init__(self) -> None:
                self._waypoint = 1000

            def next_waypoint(self) -> int:
                self._waypoint += 1
                return self._waypoint

            def next_individual(self) -> int:
                return 900000003

            def next_path(self, _aircraft_id: int) -> int:
                return 400000003

            def next_imp(self) -> int:
                return 800000002

        written: list[tuple[Path, dict[str, object]]] = []
        cloned_future_mission = deepcopy(imp["individualMissionList"][2])
        cloned_future_mission["individualMissionID"] = 900000005
        cloned_future_mission["pathID"] = 400000005
        cloned_future_path = {
            "pathID": 400000005,
            "aircraftID": 4,
            "individualMissionID": 900000005,
            "waypointList": [
                {
                    "waypointID": 1002,
                    "coordinate": {
                        "latitude": 38.2,
                        "longitude": 127.2,
                        "altitude": 1200,
                    },
                    "isDone": False,
                }
            ],
        }

        def _capture_write(entries, **_kwargs) -> None:
            written.extend(entries)

        with tempfile.TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)

            def _db_path(kind: str, filename: str | None = None) -> Path:
                directory = root / kind
                return directory / filename if filename else directory

            with (
                patch(
                    "modules.mission_planning.replanning.triggers.post_attack.pipeline."
                    "_load_imp_package_for_aircraft_cached",
                    return_value=imp,
                ),
                patch(
                    "modules.mission_planning.replanning.triggers.post_attack.pipeline."
                    "_load_path_payload",
                    side_effect=lambda path_id, **_kwargs: (
                        resume_path if int(path_id) == 400000002 else tracking_path
                    ),
                ),
                patch(
                    "modules.mission_planning.replanning.triggers.post_attack.pipeline."
                    "_input_area_internal_hold_coordinate",
                    return_value=None,
                ),
                patch(
                    "modules.mission_planning.replanning.triggers.post_attack.pipeline."
                    "ReplanIdReservation.reserve",
                    return_value=_Reservation(),
                ),
                patch(
                    "modules.mission_planning.replanning.triggers.post_attack.pipeline."
                    "validate_generated_artifact_payloads",
                ),
                patch(
                    "modules.mission_planning.replanning.triggers.post_attack.pipeline."
                    "write_json_batch",
                    side_effect=_capture_write,
                ),
                patch(
                    "modules.mission_planning.replanning.triggers.post_attack.pipeline."
                    "db_paths.get_db_subpath",
                    side_effect=_db_path,
                ),
                patch(
                    "modules.mission_planning.replanning.triggers.post_attack.pipeline."
                    "_clone_follow_up_replan_artifacts",
                    return_value=(
                        [cloned_future_mission],
                        [
                            (
                                root / "FlightPath" / "400000005.json",
                                cloned_future_path,
                            )
                        ],
                    ),
                ),
            ):
                result = _build_post_attack_tracking_return_only_update(
                    attack_plan_id=700000001,
                    current_input_id=70000001,
                    assignment={
                        "aircraft_id": 4,
                        "tracking_individual_mission_id": 900000001,
                        "resume_individual_mission_id": 900000002,
                        "original_path_id": 400000001,
                        "resume_path_id": 400000002,
                    },
                    current_state={
                        "coordinate": {
                            "latitude": 38.0005,
                            "longitude": 127.0005,
                            "altitude": 1200,
                        }
                    },
                    now_ms=1,
                    emit=lambda _message: None,
                    log_prefix="[TEST]",
                    block_follow_up_until_reassignment=True,
                )
                point_only_written = [
                    (path, deepcopy(payload)) for path, payload in written
                ]
                written.clear()
                executable_future_result = (
                    _build_post_attack_tracking_return_only_update(
                        attack_plan_id=700000001,
                        current_input_id=70000001,
                        assignment={
                            "aircraft_id": 4,
                            "tracking_individual_mission_id": 900000001,
                            "resume_individual_mission_id": 900000002,
                            "original_path_id": 400000001,
                            "resume_path_id": 400000002,
                        },
                        current_state={
                            "coordinate": {
                                "latitude": 38.0005,
                                "longitude": 127.0005,
                                "altitude": 1200,
                            }
                        },
                        now_ms=2,
                        emit=lambda _message: None,
                        log_prefix="[TEST]",
                        block_follow_up_until_reassignment=False,
                    )
                )
                executable_future_written = [
                    (path, deepcopy(payload)) for path, payload in written
                ]

        written = point_only_written
        self.assertIsNotNone(result)
        path_payload = next(
            payload
            for path, payload in written
            if path.name == "400000003.json"
        )
        self.assertEqual(len(path_payload["waypointList"]), 1)
        terminal = path_payload["waypointList"][-1]
        self.assertEqual(terminal["waypointPassType"], 2)
        # The completion-boundary hold is one shared constant; assert against
        # it so a deliberate change to the duration cannot silently diverge.
        self.assertEqual(
            terminal["loiterProperty"]["time"],
            int(_POST_ATTACK_COMPLETE_HOLD_SECONDS),
        )
        self.assertEqual(terminal["loiterProperty"]["radius"], 180)
        self.assertEqual(terminal["loiterProperty"]["speed"], 30)
        self.assertEqual(terminal["coordinate"]["latitude"], 38.0005)
        self.assertEqual(terminal["coordinate"]["longitude"], 127.0005)
        self.assertTrue(terminal["noCaptureCompletionLoiter"])
        self.assertTrue(terminal["postAttackBoundaryHold"])
        self.assertTrue(path_payload["postAttackBoundaryHold"])
        imp_payload = next(
            payload
            for path, payload in written
            if path.name == "800000002.json"
        )
        self.assertTrue(
            imp_payload["individualMissionList"][0]["postAttackBoundaryHold"]
        )
        return_mission = imp_payload["individualMissionList"][0]
        self.assertTrue(return_mission["noCaptureCompletionLoiter"])
        self.assertEqual(
            [
                mission["individualMissionID"]
                for mission in imp_payload["individualMissionList"]
            ],
            [900000003, 900000005],
        )
        self.assertNotIn(
            900000001,
            [
                mission["individualMissionID"]
                for mission in imp_payload["individualMissionList"]
            ],
        )
        self.assertNotIn(
            900000002,
            [
                mission["individualMissionID"]
                for mission in imp_payload["individualMissionList"]
            ],
        )
        self.assertTrue(
            imp_payload["individualMissionList"][1][
                "executionBlockedUntilNextCollab"
            ]
        )
        self.assertEqual(
            return_mission["individualMissionInfo"]["coordinateList"],
            [
                {
                    "latitude": 38.0005,
                    "longitude": 127.0005,
                    "altitude": 1200.0,
                }
            ],
        )
        self.assertEqual(return_mission["individualMissionInfo"]["lineList"], [])

        self.assertIsNotNone(executable_future_result)
        self.assertFalse(executable_future_result["completionBoundaryHold"])
        connector_path = next(
            payload
            for path, payload in executable_future_written
            if path.name == "400000003.json"
        )
        connector_terminal = connector_path["waypointList"][-1]
        self.assertNotEqual(connector_terminal.get("waypointPassType"), 2)
        self.assertFalse(connector_terminal.get("loiterProperty"))
        executable_imp = next(
            payload
            for path, payload in executable_future_written
            if path.name == "800000002.json"
        )
        self.assertNotIn(
            "executionBlockedUntilNextCollab",
            executable_imp["individualMissionList"][-1],
        )

    def test_type2_partial_branch_line_omits_return_and_blocks_future_area(
        self,
    ) -> None:
        current_input_id = 4
        future_input_id = 5
        sweep_coords = [
            {
                "latitude": 38.0 + (index * 0.001),
                "longitude": 127.0,
                "altitude": 900,
            }
            for index in range(6)
        ]

        def _imaging_waypoint(waypoint_id: int, coords: list[dict[str, object]]) -> dict[str, object]:
            # Keep the aircraft route visibly separate from the sensor's ground
            # sweep.  A post-attack return must use this flight coordinate, not
            # the first coordinate in lineSearch.coordinateList.
            flight_coord = deepcopy(coords[0])
            flight_coord["longitude"] = float(flight_coord["longitude"]) + 0.01
            return {
                "waypointID": int(waypoint_id),
                "coordinate": flight_coord,
                "isDone": True,
                "filmingProperty": {
                    "operationMode": 2,
                    "lineSearch": {
                        "coordinateList": deepcopy(coords),
                        "searchSpeed": 25,
                    },
                },
            }

        tracking_mission = _mission(900000001, current_input_id)
        tracking_mission["pathID"] = 500000001
        tracking_mission["individualMissionInfo"] = {
            "individualMissionType": 1,
            "targetID": 7,
        }
        resume_mission = _mission(900000002, current_input_id)
        resume_mission["pathID"] = 500000002
        resume_mission["individualMissionInfo"] = {
            "individualMissionType": 6,
            "targetID": None,
            "lineList": [{"coordinateList": deepcopy(sweep_coords), "width": 300}],
            "areaList": [],
            "coordinateList": deepcopy(sweep_coords),
        }
        # Reproduce the real Plan34 provenance: the current LINE stays
        # executable, while the later AREA remains behind the group handoff.
        resume_mission["executionBlockedUntilNextCollab"] = True
        future_area_mission = _mission(900000003, future_input_id)
        future_area_mission["pathID"] = 500000003
        future_area_mission["executionBlockedUntilNextCollab"] = True
        future_area_mission["individualMissionInfo"] = {
            "individualMissionType": 3,
            "targetID": None,
            "lineList": [],
            "areaList": [
                {
                    "coordinateList": [
                        {"latitude": 38.5, "longitude": 127.5, "altitude": 900}
                    ]
                }
            ],
        }
        imp = {
            "individualMissionPackageID": 800000001,
            "individualMissionList": [
                tracking_mission,
                resume_mission,
                future_area_mission,
            ],
        }
        tracking_path = {
            "pathID": 500000001,
            "aircraftID": 5,
            "individualMissionID": 900000001,
            "waypointList": [
                {
                    "waypointID": 101,
                    "coordinate": {"latitude": 37.99, "longitude": 127.0, "altitude": 900},
                    "isDone": True,
                }
            ],
        }
        resume_path = {
            "pathID": 500000002,
            "aircraftID": 5,
            "individualMissionID": 900000002,
            # This reproduces the observed log: carrier WPs say done although
            # the nested lineSearch geometry is only partly photographed.
            "waypointList": [
                _imaging_waypoint(102, sweep_coords[:4]),
                _imaging_waypoint(103, sweep_coords[4:]),
            ],
        }
        future_area_path = {
            "pathID": 500000003,
            "aircraftID": 5,
            "individualMissionID": 900000003,
            "waypointList": [
                {
                    "waypointID": 104,
                    "coordinate": {"latitude": 38.5, "longitude": 127.5, "altitude": 900},
                    "isDone": False,
                }
            ],
        }
        path_payloads = {
            500000001: tracking_path,
            500000002: resume_path,
            500000003: future_area_path,
        }

        class _Reservation:
            def __init__(self) -> None:
                self._waypoint = 2000
                self._individual = 900000009
                self._path = 500000009

            def next_waypoint(self) -> int:
                self._waypoint += 1
                return self._waypoint

            def next_individual(self) -> int:
                self._individual += 1
                return self._individual

            def next_path(self, _aircraft_id: int) -> int:
                self._path += 1
                return self._path

            def next_imp(self) -> int:
                return 800000002

        written: list[tuple[Path, dict[str, object]]] = []
        type2_messages: list[str] = []

        def _capture_write(entries, **_kwargs) -> None:
            written.extend(entries)

        with tempfile.TemporaryDirectory() as tmp_dir:
            root = Path(tmp_dir)

            def _db_path(kind: str, filename: str | None = None) -> Path:
                directory = root / kind
                return directory / filename if filename else directory

            def _read_json(path: Path, **_kwargs) -> dict[str, object]:
                path_id = int(Path(path).stem)
                return deepcopy(path_payloads[path_id])

            with (
                patch(
                    "modules.mission_planning.replanning.triggers.post_attack.pipeline."
                    "_load_imp_package_for_aircraft_cached",
                    return_value=deepcopy(imp),
                ),
                patch(
                    "modules.mission_planning.replanning.triggers.post_attack.pipeline."
                    "_load_path_payload",
                    side_effect=lambda path_id, **_kwargs: deepcopy(
                        path_payloads[int(path_id)]
                    ),
                ),
                patch(
                    "modules.mission_planning.replanning.triggers.post_attack.pipeline."
                    "_load_sweep_progress_safe",
                    return_value={
                        500000002: {
                            "path_id": 500000002,
                            "sweep_point_count": 6,
                            "progress_points": 2,
                            "progress_percent": 33,
                            "remaining_seconds": 40,
                            "buffer_points": 2,
                            "progress_source": "line_scan",
                            "line_scan": {
                                "source": "line_scan_progress_monitor",
                                "isCurrent": True,
                                "sweepPointCount": 6,
                                "progressPoints": 2,
                                "progressPercent": 33,
                            },
                        }
                    },
                ),
                patch(
                    "modules.mission_planning.replanning.triggers.post_attack.pipeline."
                    "_source_type2_self_reliance_phase",
                    return_value="outbound_line",
                ),
                patch(
                    "modules.mission_planning.replanning.triggers.post_attack.pipeline."
                    "ReplanIdReservation.reserve",
                    return_value=_Reservation(),
                ),
                patch(
                    "modules.mission_planning.replanning.triggers.post_attack.pipeline."
                    "validate_generated_artifact_payloads",
                ),
                patch(
                    "modules.mission_planning.replanning.triggers.post_attack.pipeline."
                    "write_json_batch",
                    side_effect=_capture_write,
                ),
                patch(
                    "modules.mission_planning.replanning.triggers.post_attack.pipeline."
                    "read_json_cached",
                    side_effect=_read_json,
                ),
                patch(
                    "modules.mission_planning.replanning.triggers.prior.pipeline."
                    "read_json_cached",
                    side_effect=_read_json,
                ),
                patch(
                    "modules.mission_planning.replanning.triggers.post_attack.pipeline."
                    "db_paths.get_db_subpath",
                    side_effect=_db_path,
                ),
            ):
                result = _build_post_attack_tracking_return_only_update(
                    attack_plan_id=700000027,
                    current_input_id=current_input_id,
                    assignment={
                        "aircraft_id": 5,
                        "tracking_individual_mission_id": 900000001,
                        "resume_individual_mission_id": 900000002,
                        "original_path_id": 500000001,
                        "resume_path_id": 500000002,
                    },
                    current_state={
                        "coordinate": {
                            "latitude": 37.995,
                            "longitude": 127.0,
                            "altitude": 900,
                        },
                        "heading": 0.0,
                        "speed": 30.0,
                    },
                    now_ms=1,
                    emit=type2_messages.append,
                    log_prefix="[TEST]",
                    block_follow_up_until_reassignment=True,
                )

        self.assertIsNotNone(result, "\n".join(type2_messages))
        self.assertTrue(result["followUpsBlockedUntilNextCollab"])
        self.assertFalse(result["completionBoundaryHold"])
        self.assertTrue(result["directLineResume"])
        self.assertTrue(result["returnMissionOmitted"])
        self.assertEqual(
            result["returnMissionOmitReason"],
            "direct_live_line_follow_up",
        )
        self.assertIsNone(result["returnMission"])

        imp_payload = next(
            payload for path, payload in written if path.name == "800000002.json"
        )
        missions = imp_payload["individualMissionList"]
        self.assertEqual(
            [mission["relatedMission"]["inputMissionID"] for mission in missions],
            [current_input_id, future_input_id],
        )
        self.assertFalse(missions[0].get("executionBlockedUntilNextCollab", False))
        self.assertTrue(missions[1].get("executionBlockedUntilNextCollab", False))
        self.assertTrue(
            all(
                (
                    (mission.get("individualMissionInfo") or {}).get(
                        "individualMissionType"
                    ),
                    (mission.get("individualMissionInfo") or {}).get("patternType"),
                )
                != (7, 10)
                for mission in missions
            )
        )
        self.assertEqual(
            missions[0]["individualMissionInfo"]["individualMissionType"],
            6,
        )

        cloned_line_path_id = missions[0]["pathID"]
        cloned_line_path = next(
            payload
            for path, payload in written
            if path.name == f"{cloned_line_path_id}.json"
        )
        resumed_flight_coord = cloned_line_path["waypointList"][0]["coordinate"]
        resumed_sweep_coord = (
            cloned_line_path["waypointList"][0]["filmingProperty"]["lineSearch"][
                "coordinateList"
            ][0]
        )
        self.assertGreater(
            abs(
                float(resumed_flight_coord["longitude"])
                - float(resumed_sweep_coord["longitude"])
            ),
            0.005,
        )
        remaining_sweep_points = sum(
            len(
                (((waypoint.get("filmingProperty") or {}).get("lineSearch") or {}).get("coordinateList") or [])
            )
            for waypoint in cloned_line_path["waypointList"]
        )
        self.assertEqual(remaining_sweep_points, 4)
        self.assertTrue(
            all(not waypoint.get("isDone") for waypoint in cloned_line_path["waypointList"])
        )
        written_flight_path_ids = {
            int(path.stem)
            for path, payload in written
            if path.parent.name == "FlightPath"
            and isinstance(payload, dict)
        }
        self.assertEqual(
            written_flight_path_ids,
            {int(mission["pathID"]) for mission in missions},
        )
        self.assertEqual(
            set(result["generatedPathIDs"]),
            written_flight_path_ids,
        )
        self.assertTrue(
            any(
                "redundant return-only boundary omitted" in message
                for message in type2_messages
            )
        )

    def test_type2_return_restores_legacy_reanchored_carriers_from_original_path(
        self,
    ) -> None:
        first_source_sweep = [
            {"latitude": 38.0538, "longitude": 127.3642, "altitude": 352},
            {"latitude": 38.0561, "longitude": 127.3727, "altitude": 235},
            {"latitude": 38.0465, "longitude": 127.3854, "altitude": 193},
        ]
        second_source_sweep = [
            {"latitude": 38.0466, "longitude": 127.3857, "altitude": 194},
            {"latitude": 38.0589, "longitude": 127.4076, "altitude": 479},
        ]

        def _capture(
            waypoint_id: int,
            coordinate: dict[str, object],
            sweep: list[dict[str, object]],
        ) -> dict[str, object]:
            return {
                "waypointID": waypoint_id,
                "coordinate": deepcopy(coordinate),
                "isDone": False,
                "filmingProperty": {
                    "operationMode": 2,
                    "lineSearch": {"coordinateList": deepcopy(sweep)},
                },
            }

        original_first = {
            "latitude": 38.047971,
            "longitude": 127.384717,
            "altitude": 1352,
        }
        original_second = {
            "latitude": 38.054501,
            "longitude": 127.409464,
            "altitude": 1568,
        }
        original_path = {
            "waypointList": [
                {
                    "waypointID": 11167,
                    "coordinate": {
                        "latitude": 38.043699,
                        "longitude": 127.368532,
                        "altitude": 1522,
                    },
                },
                _capture(11168, original_first, first_source_sweep),
                _capture(11169, original_second, second_source_sweep),
            ]
        }
        # Reproduce the observed legacy corruption: sensor-based anchors made
        # the carrier run west although the source branch runs north-east.
        legacy_resume = [
            _capture(
                12292,
                {
                    "latitude": 38.043397,
                    "longitude": 127.378290,
                    "altitude": 1265,
                },
                first_source_sweep[1:],
            ),
            _capture(
                12293,
                {
                    "latitude": 38.046047,
                    "longitude": 127.358929,
                    "altitude": 1568,
                },
                second_source_sweep,
            ),
        ]

        restored = _restore_type2_line_carriers_from_original(
            legacy_resume,
            original_path,
            original_current_waypoint_id=11168,
        )

        self.assertEqual(restored, 2)
        self.assertEqual(legacy_resume[0]["coordinate"], original_first)
        self.assertEqual(legacy_resume[1]["coordinate"], original_second)
        self.assertGreater(
            legacy_resume[1]["coordinate"]["longitude"],
            legacy_resume[0]["coordinate"]["longitude"],
        )


if __name__ == "__main__":
    unittest.main()
