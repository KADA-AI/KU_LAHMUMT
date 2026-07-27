from __future__ import annotations

import unittest
from copy import deepcopy
from pathlib import Path
from unittest.mock import patch

from modules.mission_planning.replanning.triggers.post_attack.pipeline import (
    _build_lah_path_payload_from_waypoints,
    _copy_post_attack_imp_shell,
)
from modules.mission_planning.replanning.triggers.prior.pipeline import (
    _build_uav_release_resume_waypoints,
    _clone_follow_up_replan_artifacts,
)
from modules.mission_planning.runtime.replan_transaction import (
    _mark_written_flight_paths,
)


class ReplanPerformanceInvariantTests(unittest.TestCase):
    def test_release_resume_can_defer_ids_without_changing_geometry(self) -> None:
        kwargs = {
            "start_coord": {"latitude": 38.0, "longitude": 127.0, "altitude": 1000},
            "end_coord": {"latitude": 38.1, "longitude": 127.1, "altitude": 1000},
            "release_eta_s": 0,
            "target_finish_eta_s": 100,
        }
        deferred, deferred_speed = _build_uav_release_resume_waypoints(
            **kwargs,
            assign_waypoint_ids=False,
        )
        ids = iter((101, 102))
        assigned, assigned_speed = _build_uav_release_resume_waypoints(
            **kwargs,
            waypoint_id_provider=lambda: next(ids),
        )

        self.assertEqual([row["waypointID"] for row in deferred], [0, 0])
        self.assertEqual([row["waypointID"] for row in assigned], [101, 102])
        self.assertEqual([row["nextWaypointID"] for row in assigned], [102, 0])
        self.assertEqual(deferred_speed, assigned_speed)
        for deferred_wp, assigned_wp in zip(deferred, assigned):
            deferred_payload = {k: v for k, v in deferred_wp.items() if k not in {"waypointID", "nextWaypointID"}}
            assigned_payload = {k: v for k, v in assigned_wp.items() if k not in {"waypointID", "nextWaypointID"}}
            self.assertEqual(deferred_payload, assigned_payload)

    def test_follow_up_clone_consumes_pre_reserved_ids_and_keeps_source_unchanged(self) -> None:
        source = {
            "pathID": 400000001,
            "aircraftID": 4,
            "individualMissionID": 900000001,
            "waypointList": [
                {"waypointID": 11, "nextWaypointID": 12, "isDone": True},
                {"waypointID": 12, "nextWaypointID": 0, "isDone": True},
            ],
        }
        mission = {
            "individualMissionID": 900000001,
            "pathID": 400000001,
            "isDone": False,
            "relatedMission": {"inputMissionID": 70000001},
        }
        waypoint_ids = iter((201, 202))

        with patch(
            "modules.mission_planning.replanning.triggers.prior.pipeline.read_json_cached",
            side_effect=lambda *_args, **_kwargs: deepcopy(source),
        ), patch(
            "modules.mission_planning.replanning.triggers.prior.pipeline.db_paths.get_db_subpath",
            side_effect=lambda kind, filename=None: Path("C:/replan-test") / kind / (filename or ""),
        ):
            result = _clone_follow_up_replan_artifacts(
                missions=[mission],
                aircraft_id=4,
                now_ms=123,
                emit=lambda _message: None,
                log_prefix="[TEST]",
                individual_id_provider=lambda: 900000101,
                path_id_provider=lambda _aircraft_id: 400000101,
                waypoint_id_provider=lambda: next(waypoint_ids),
            )

        self.assertIsNotNone(result)
        cloned_missions, cloned_paths = result or ([], [])
        self.assertEqual(cloned_missions[0]["individualMissionID"], 900000101)
        cloned = cloned_paths[0][1]
        self.assertEqual([wp["waypointID"] for wp in cloned["waypointList"]], [201, 202])
        self.assertEqual([wp["nextWaypointID"] for wp in cloned["waypointList"]], [202, 0])
        self.assertTrue(all(not wp["isDone"] for wp in cloned["waypointList"]))
        self.assertEqual(source["waypointList"][0]["waypointID"], 11)
        self.assertTrue(source["waypointList"][0]["isDone"])

    def test_flight_path_write_marker_uses_payload_high_water(self) -> None:
        entries = [
            (
                Path("C:/scenario/FlightPath/400000001.json"),
                {"waypointList": [{"waypointID": 301}, {"waypointID": 305}]},
            ),
            (Path("C:/scenario/MissionPlan/700000001.json"), {"missionPlanID": 700000001}),
        ]
        results = [{"written": True}, {"written": True}]
        with patch(
            "modules.mission_planning.engine.mission_generation.id_allocation.allocator."
            "mark_waypoint_files_written"
        ) as marker:
            _mark_written_flight_paths(entries, results)
        marker.assert_called_once_with(max_waypoint_id=305)

    def test_post_attack_shell_and_lah_builder_preserve_payload_values(self) -> None:
        imp = {
            "individualMissionPackageID": 800000001,
            "timestamp": 1,
            "meta": {"value": [1, 2, 3]},
            "individualMissionList": [{"individualMissionID": 900000001}],
        }
        shell = _copy_post_attack_imp_shell(imp)
        self.assertNotIn("individualMissionList", shell)
        self.assertEqual(shell["meta"], imp["meta"])
        shell["meta"]["value"].append(4)
        self.assertEqual(imp["meta"]["value"], [1, 2, 3])

        template = {
            "pathID": 100000001,
            "Source": "MMR",
            "meta": {"x": 1},
            "lahWaypointList": [{"waypointID": 1, "isDone": True}],
            "waypointList": [{"waypointID": 1, "isDone": True}],
        }
        ids = iter((401,))
        payload = _build_lah_path_payload_from_waypoints(
            template_path=template,
            aircraft_id=1,
            path_id=100000101,
            individual_mission_id=900000101,
            waypoints=[{"waypointID": 1, "nextWaypointID": 0, "isDone": True}],
            now_ms=456,
            waypoint_id_provider=lambda: next(ids),
        )
        self.assertEqual(payload["pathID"], 100000101)
        self.assertEqual(payload["lahWaypointList"][0]["waypointID"], 401)
        self.assertEqual(payload["waypointList"], payload["lahWaypointList"])
        self.assertEqual(template["lahWaypointList"][0]["waypointID"], 1)

    def test_post_attack_lah_builder_normalizes_legacy_float_altitude_without_mutating_source(
        self,
    ) -> None:
        template = {
            "pathID": 100000001,
            "Source": "MMR",
            "lahWaypointList": [],
            "waypointList": [],
        }
        source_waypoint = {
            "waypointID": 1,
            "nextWaypointID": 0,
            "isDone": True,
            "coordinate": {
                "latitude": 37.87,
                "longitude": 128.12,
                "altitude": 795.373,
            },
        }

        payload = _build_lah_path_payload_from_waypoints(
            template_path=template,
            aircraft_id=1,
            path_id=100000101,
            individual_mission_id=900000101,
            waypoints=[source_waypoint],
            now_ms=456,
            waypoint_id_provider=lambda: 401,
        )

        for list_key in ("lahWaypointList", "waypointList"):
            altitude = payload[list_key][0]["coordinate"]["altitude"]
            self.assertEqual(altitude, 795)
            self.assertIs(type(altitude), int)
        self.assertEqual(source_waypoint["coordinate"]["altitude"], 795.373)


if __name__ == "__main__":
    unittest.main()
