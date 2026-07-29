from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from modules.mission_planning.runtime import next_collab_replan_store
from modules.mission_planning.replanning.triggers.attack.pipeline import (
    _resolve_plan_artifacts_cached,
    _split_done_resume_path,
    _type2_branch_line_completion_confirmed,
)
from modules.mission_planning.replanning.triggers.prior.pipeline import (
    _resolve_plan_artifacts,
)


class Type2ReplanCollaborationBarrierTests(unittest.TestCase):
    def test_authoritative_complete_line_collapses_to_terminal_hold(self) -> None:
        sweep = [
            {
                "latitude": 38.0 + index * 0.00001,
                "longitude": 127.0,
                "altitude": 0,
            }
            for index in range(6)
        ]
        carrier = {
            "waypointID": 12001,
            "coordinate": {
                "latitude": 38.0,
                "longitude": 127.01,
                "altitude": 1400,
            },
            "isDone": False,
            "filmingProperty": {
                "operationMode": 2,
                "lineSearch": {
                    "coordinateList": sweep,
                    "searchSpeed": 20.0,
                },
            },
        }
        progress = {
            400000112: {
                "sweep_point_count": 6,
                "progress_points": 6,
                "progress_percent": 100,
                "remaining_seconds": 0,
            }
        }

        with patch(
            "modules.mission_planning.replanning.triggers.attack.pipeline."
            "_source_type2_self_reliance_phase",
            return_value="outbound_line",
        ):
            completion_confirmed = _type2_branch_line_completion_confirmed(
                source_plan_id=700000023,
                input_mission_id=4,
                path_id=400000112,
                sweep_progress=progress,
            )
            _done, resume, _removed = _split_done_resume_path(
                {
                    "pathID": 400000112,
                    "waypointList": [carrier],
                },
                artifacts=SimpleNamespace(
                    path_id=400000112,
                    current_waypoint_id=12001,
                    previous_waypoint_id=None,
                ),
                sweep_progress=progress,
                emit=lambda _message: None,
                force_nonempty_resume=not completion_confirmed,
            )

        self.assertTrue(completion_confirmed)
        self.assertEqual(resume, [])

    def test_latest_next_collab_detail_follows_derived_plan_lineage(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            (root / "next_collab_detail_700000015.json").write_text(
                json.dumps(
                    {
                        "missionPlanID": 700000015,
                        "inputMissionPackageID": 3,
                        "targetInputMissionID": 4,
                    }
                ),
                encoding="utf-8",
            )
            (root / "next_collab_detail_700000020.json").write_text(
                json.dumps(
                    {
                        "missionPlanID": 700000020,
                        "inputMissionPackageID": 99,
                        "targetInputMissionID": 8,
                    }
                ),
                encoding="utf-8",
            )
            with patch.object(
                next_collab_replan_store.replan_store,
                "_store_dir",
                return_value=root,
            ):
                detail = next_collab_replan_store.load_latest_detail_at_or_before(
                    700000024,
                    input_mission_package_id=3,
                )

        self.assertIsNotNone(detail)
        self.assertEqual(detail["missionPlanID"], 700000015)
        self.assertEqual(detail["targetInputMissionID"], 4)

    def test_prior_fallback_keeps_completed_line_before_blocked_area(self) -> None:
        plan = {
            "aircraftList": [
                {
                    "aircraftID": 4,
                    "individualMissionPackageID": 800000130,
                }
            ]
        }
        package = {
            "individualMissionList": [
                {
                    "individualMissionID": 900000585,
                    "pathID": 400000112,
                    "isDone": True,
                    "relatedMission": {"inputMissionID": 4},
                },
                {
                    "individualMissionID": 900000586,
                    "pathID": 400000113,
                    "isDone": False,
                    "executionBlockedUntilNextCollab": True,
                    "relatedMission": {"inputMissionID": 5},
                },
            ]
        }
        emitted: list[str] = []

        def _read_json(_path: Path, *, kind: str, **_kwargs: object) -> dict:
            if kind == "MissionPlan":
                return plan
            if kind == "IndividualMissionPlan":
                return package
            raise AssertionError(kind)

        with (
            patch(
                "modules.mission_planning.replanning.triggers.prior.pipeline."
                "db_paths.get_db_subpath",
                side_effect=lambda folder, name=None: Path(folder) / str(name or ""),
            ),
            patch(
                "modules.mission_planning.replanning.triggers.prior.pipeline."
                "read_json_cached",
                side_effect=_read_json,
            ),
            patch(
                "modules.mission_planning.replanning.triggers.prior.pipeline."
                "_load_waypoint_ids",
                side_effect=lambda path_id: {
                    400000112: [12001, 12002],
                    400000113: [13001],
                }.get(int(path_id), []),
            ),
        ):
            artifacts = _resolve_plan_artifacts(
                source_plan_id=700000023,
                aircraft_id=4,
                current_waypoint_id=0,
                emit=emitted.append,
            )

        self.assertIsNotNone(artifacts)
        self.assertEqual(artifacts.individual_mission_id, 900000585)
        self.assertEqual(artifacts.path_id, 400000112)
        self.assertEqual(artifacts.current_waypoint_id, 12002)
        self.assertEqual(artifacts.previous_waypoint_id, 12001)
        self.assertTrue(
            any("before collaboration barrier" in message for message in emitted),
            emitted,
        )

    def test_attack_fallback_rejects_exact_wp_inside_blocked_area(self) -> None:
        package_id = 800000130
        cache = {
            "aircraft_entries": {
                4: {
                    "aircraftID": 4,
                    "individualMissionPackageID": package_id,
                }
            },
            "imp_payloads": {
                package_id: {
                    "individualMissionList": [
                        {
                            "individualMissionID": 900000586,
                            "pathID": 400000113,
                            "isDone": False,
                            "executionBlockedUntilNextCollab": True,
                            "relatedMission": {"inputMissionID": 5},
                        },
                        {
                            "individualMissionID": 900000585,
                            "pathID": 400000112,
                            "isDone": True,
                            "relatedMission": {"inputMissionID": 4},
                        },
                    ]
                }
            },
            "fp_payloads": {},
            "waypoint_ids": {
                400000112: [12001, 12002],
                400000113: [13001],
            },
        }
        emitted: list[str] = []

        artifacts = _resolve_plan_artifacts_cached(
            source_plan_id=700000023,
            aircraft_id=4,
            current_waypoint_id=13001,
            cache=cache,
            emit=emitted.append,
        )

        self.assertIsNotNone(artifacts)
        self.assertEqual(artifacts.individual_mission_id, 900000585)
        self.assertEqual(artifacts.path_id, 400000112)
        self.assertEqual(artifacts.current_waypoint_id, 12002)
        self.assertTrue(
            any("before collaboration barrier" in message for message in emitted),
            emitted,
        )


if __name__ == "__main__":
    unittest.main()
