from __future__ import annotations

import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from modules.mission_status_monitoring.service import (
    MissionStatusService,
    _latest_option_payload_from_db,
    build_option_assignment_snapshot,
)


def _write_json(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")


class OptionAssignmentSnapshotTests(unittest.TestCase):
    def test_builds_area_and_line_assignments_per_uav(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            plan_id = 700000101
            _write_json(
                root / "MissionPlan" / f"{plan_id}.json",
                {
                    "missionPlanID": plan_id,
                    "aircraftList": [
                        {"aircraftID": 4, "individualMissionPackageID": 800000104},
                        {"aircraftID": 5, "individualMissionPackageID": 800000105},
                        {"aircraftID": 6, "individualMissionPackageID": 800000106},
                    ],
                },
            )
            _write_json(
                root / "IndividualMissionPlan" / "800000104.json",
                {
                    "individualMissionList": [
                        {
                            "individualMissionID": 8101,
                            "relatedMission": {"inputMissionID": 101},
                            "individualMissionInfo": {
                                "individualMissionType": 2,
                                "areaList": [
                                    {
                                        "isHole": False,
                                        "coordinateList": [
                                            {"latitude": 38.0, "longitude": 127.0},
                                            {"latitude": 38.0, "longitude": 127.01},
                                            {"latitude": 38.01, "longitude": 127.01},
                                        ],
                                    }
                                ],
                            },
                        }
                    ]
                },
            )
            _write_json(
                root / "IndividualMissionPlan" / "800000105.json",
                {
                    "individualMissionList": [
                        {
                            "individualMissionID": 8201,
                            "relatedMission": {"inputMissionID": 102},
                            "individualMissionInfo": {
                                "individualMissionType": 1,
                                "lineList": [
                                    {
                                        "width": 120,
                                        "coordinateList": [
                                            {"latitude": 38.02, "longitude": 127.02},
                                            {"latitude": 38.03, "longitude": 127.04},
                                        ],
                                    }
                                ],
                            },
                        }
                    ]
                },
            )
            _write_json(
                root / "IndividualMissionPlan" / "800000106.json",
                {
                    "individualMissionList": [
                        {
                            "individualMissionID": 8301,
                            "relatedMission": {"inputMissionID": 103},
                            "individualMissionInfo": {
                                "individualMissionType": 9,
                                "coordinateList": [
                                    {"latitude": 38.04, "longitude": 127.05}
                                ],
                            },
                        }
                    ]
                },
            )
            payload = {
                "timestamp": 837510000000,
                "optionList": [
                    {
                        "optionID": 17,
                        "optionName": 2,
                        "recommend": True,
                        "missionPlanID": plan_id,
                    }
                ],
            }

            snapshot = build_option_assignment_snapshot(payload, db_root=root)

            self.assertTrue(snapshot["available"])
            self.assertEqual(snapshot["optionCount"], 1)
            option = snapshot["options"][0]
            self.assertTrue(option["available"])
            self.assertTrue(option["complete"])
            self.assertTrue(option["recommend"])
            by_aircraft = {row["aircraftID"]: row for row in option["aircraft"]}
            self.assertEqual((by_aircraft[4]["areaCount"], by_aircraft[4]["lineCount"]), (1, 0))
            self.assertEqual((by_aircraft[5]["areaCount"], by_aircraft[5]["lineCount"]), (0, 1))
            self.assertEqual((by_aircraft[6]["areaCount"], by_aircraft[6]["lineCount"]), (0, 0))
            self.assertEqual(by_aircraft[4]["inputMissionIDs"], [101])
            self.assertEqual(by_aircraft[5]["inputMissionIDs"], [102])
            self.assertEqual(by_aircraft[6]["inputMissionIDs"], [])
            shapes = {
                (feature["properties"]["aircraftID"], feature["properties"]["assignmentShape"])
                for feature in option["geojson"]["features"]
            }
            self.assertIn((4, "AREA"), shapes)
            self.assertIn((5, "LINE"), shapes)
            self.assertFalse(any(aircraft_id == 6 for aircraft_id, _shape in shapes))

    def test_loads_newest_persisted_option_message(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            older = root / "MissionPlanOptionInfo" / "1.json"
            newer = root / "MissionPlanOptionInfo" / "2.json"
            _write_json(older, {"timestamp": 100, "optionList": [{"optionID": 1}]})
            _write_json(newer, {"timestamp": 200, "optionList": [{"optionID": 2}]})
            os.utime(older, ns=(1_000_000_000, 1_000_000_000))
            os.utime(newer, ns=(2_000_000_000, 2_000_000_000))

            payload = _latest_option_payload_from_db(root)

            self.assertEqual(payload["timestamp"], 200)
            self.assertEqual(payload["optionList"][0]["optionID"], 2)

            service = MissionStatusService(integration=None)
            with patch(
                "modules.mission_status_monitoring.service.db_paths.get_active_db_root",
                return_value=root,
            ):
                snapshot = service._option_assignment_state()
            self.assertEqual(snapshot["timestamp"], 200)
            self.assertEqual(snapshot["optionCount"], 1)
            self.assertEqual(snapshot["options"][0]["optionID"], 2)


if __name__ == "__main__":
    unittest.main()
