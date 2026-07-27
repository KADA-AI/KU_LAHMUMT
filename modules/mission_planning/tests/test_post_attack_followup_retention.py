from __future__ import annotations

import tempfile
import unittest
from copy import deepcopy
from pathlib import Path
from unittest.mock import patch

from modules.mission_planning.replanning.triggers.post_attack.pipeline import (
    _build_post_attack_tracking_return_only_update,
    _mark_post_attack_followups_execution_blocked,
    _post_attack_authoritative_source_plan_id,
    _post_attack_follow_up_source_missions,
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

    def test_post_attack_remaining_geometry_uses_current_applied_plan(self) -> None:
        self.assertEqual(
            _post_attack_authoritative_source_plan_id(700000015, 700000018),
            700000018,
        )
        self.assertEqual(
            _post_attack_authoritative_source_plan_id(700000015, 0),
            700000015,
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
                        "latitude": 38.001,
                        "longitude": 127.001,
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
                    block_follow_up_until_reassignment=False,
                )

        self.assertIsNotNone(result)
        path_payload = next(
            payload
            for path, payload in written
            if path.name == "400000003.json"
        )
        terminal = path_payload["waypointList"][-1]
        self.assertEqual(terminal["waypointPassType"], 2)
        self.assertEqual(terminal["loiterProperty"]["time"], 15)
        self.assertEqual(terminal["loiterProperty"]["radius"], 180)
        self.assertEqual(terminal["loiterProperty"]["speed"], 30)
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

    def test_type2_partial_branch_line_keeps_future_area_executable(self) -> None:
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
        future_area_mission = _mission(900000003, future_input_id)
        future_area_mission["pathID"] = 500000003
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
                        }
                    },
                    now_ms=1,
                    emit=type2_messages.append,
                    log_prefix="[TEST]",
                    block_follow_up_until_reassignment=False,
                )

        self.assertIsNotNone(result, "\n".join(type2_messages))
        self.assertFalse(result["followUpsBlockedUntilNextCollab"])
        self.assertNotAlmostEqual(
            result["returnMission"]["finalCoordinate"]["latitude"],
            38.5,
            places=3,
        )

        imp_payload = next(
            payload for path, payload in written if path.name == "800000002.json"
        )
        missions = imp_payload["individualMissionList"]
        self.assertEqual(
            [mission["relatedMission"]["inputMissionID"] for mission in missions],
            [current_input_id, current_input_id, future_input_id],
        )
        self.assertFalse(missions[1].get("executionBlockedUntilNextCollab", False))
        self.assertFalse(missions[2].get("executionBlockedUntilNextCollab", False))

        cloned_line_path_id = missions[1]["pathID"]
        cloned_line_path = next(
            payload
            for path, payload in written
            if path.name == f"{cloned_line_path_id}.json"
        )
        return_coord = result["returnMission"]["finalCoordinate"]
        resumed_flight_coord = cloned_line_path["waypointList"][0]["coordinate"]
        resumed_sweep_coord = (
            cloned_line_path["waypointList"][0]["filmingProperty"]["lineSearch"][
                "coordinateList"
            ][0]
        )
        self.assertAlmostEqual(
            return_coord["latitude"], resumed_flight_coord["latitude"], places=7
        )
        self.assertAlmostEqual(
            return_coord["longitude"], resumed_flight_coord["longitude"], places=7
        )
        self.assertGreater(
            abs(float(return_coord["longitude"]) - float(resumed_sweep_coord["longitude"])),
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


if __name__ == "__main__":
    unittest.main()
