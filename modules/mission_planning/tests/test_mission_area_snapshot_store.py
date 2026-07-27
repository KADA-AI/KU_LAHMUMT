from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from modules.common import mission_area_replan_store


class MissionAreaSnapshotStoreTests(unittest.TestCase):
    @staticmethod
    def _unfinished_area_entry(
        *,
        mission_plan_id: int = 700000013,
        input_mission_id: int = 70000003,
    ) -> dict:
        return {
            "missionPlanID": int(mission_plan_id),
            "inputMissionID": int(input_mission_id),
            "missionType": "area",
            "isDone": False,
            "remainingAreaM2": 1250.0,
            "remainingDetail": {
                "coordinateList": [],
                "lineList": [],
                "areaList": [
                    {
                        "isHole": False,
                        "coordinateList": [
                            {"latitude": 38.0, "longitude": 127.0, "altitude": 100.0},
                            {"latitude": 38.0, "longitude": 127.01, "altitude": 100.0},
                            {"latitude": 38.01, "longitude": 127.01, "altitude": 100.0},
                        ],
                    }
                ],
            },
        }

    @staticmethod
    def _line_only_live_snapshot(*, mission_plan_id: int = 700000015) -> dict:
        missions = [
            {
                "missionPlanID": int(mission_plan_id),
                "inputMissionID": input_mission_id,
                "missionType": "line",
                "isDone": False,
            }
            for input_mission_id in (70000004, 70000005, 70000009)
        ]
        return {
            "missionPlanID": int(mission_plan_id),
            "snapshotOrigin": "monitor",
            "missionCount": len(missions),
            "missions": missions,
        }

    def test_late_carry_forward_does_not_overwrite_existing_live_target(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir, patch.object(
            mission_area_replan_store,
            "_detail_dir",
            return_value=Path(temp_dir),
        ):
            source_path = mission_area_replan_store._detail_path(700000010)
            target_path = mission_area_replan_store._detail_path(700000011)
            mission_area_replan_store._write_snapshot_file(
                source_path,
                {
                    "missionPlanID": 700000010,
                    "snapshotOrigin": "monitor",
                    "missionCount": 1,
                    "missions": [
                        {
                            "missionPlanID": 700000010,
                            "inputMissionID": 70000008,
                            "missionType": "line",
                            "isDone": False,
                        }
                    ],
                },
            )
            live_target = {
                "missionPlanID": 700000011,
                "snapshotOrigin": "monitor",
                "missionCount": 1,
                "missions": [
                    {
                        "missionPlanID": 700000011,
                        "inputMissionID": 70000008,
                        "missionType": "line",
                        "isDone": True,
                        "remainingAreaM2": 0.0,
                    }
                ],
            }
            mission_area_replan_store._write_snapshot_file(target_path, live_target)

            result = mission_area_replan_store.carry_forward_snapshot(
                700000010,
                700000011,
                reason="late_async_attack_seed",
            )

            self.assertEqual(result, target_path)
            self.assertEqual(
                json.loads(target_path.read_text(encoding="utf-8")),
                live_target,
            )
            audit_rows = [
                json.loads(line)
                for line in mission_area_replan_store._audit_path()
                .read_text(encoding="utf-8")
                .splitlines()
                if line.strip()
            ]
            self.assertEqual(
                audit_rows[-1]["event"],
                "snapshot_carry_skipped_existing_target",
            )

    def test_snapshot_origins_distinguish_live_update_and_carry_seed(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir, patch.object(
            mission_area_replan_store,
            "_detail_dir",
            return_value=Path(temp_dir),
        ), patch.object(
            mission_area_replan_store,
            "_target_plan_input_kinds",
            return_value={},
        ):
            mission_area_replan_store.save_snapshot(
                700000020,
                {"missionCount": 0, "missions": []},
            )
            carried_path = mission_area_replan_store.carry_forward_snapshot(
                700000020,
                700000021,
                reason="new_plan_seed",
            )

            self.assertIsNotNone(carried_path)
            live = json.loads(
                mission_area_replan_store._detail_path(700000020).read_text(encoding="utf-8")
            )
            carried = json.loads(Path(carried_path).read_text(encoding="utf-8"))
            self.assertEqual(live.get("snapshotOrigin"), "monitor")
            self.assertEqual(carried.get("snapshotOrigin"), "carry_forward_seed")

    def test_live_line_only_save_restores_missing_unfinished_area_from_central_ledger(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir, patch.object(
            mission_area_replan_store,
            "_detail_dir",
            return_value=Path(temp_dir),
        ), patch.object(
            mission_area_replan_store,
            "_target_plan_input_kinds",
            return_value={
                70000003: "area",
                70000004: "line",
                70000005: "line",
                70000009: "line",
            },
        ), patch.object(
            mission_area_replan_store,
            "_target_plan_active_area_input_ids",
            return_value={70000003},
        ):
            area_entry = self._unfinished_area_entry()
            mission_area_replan_store._write_central_ledger(
                {
                    "entries": {
                        "area:70000003": {
                            "mission": area_entry,
                        }
                    }
                }
            )
            mission_area_replan_store._write_snapshot_file(
                mission_area_replan_store._detail_path(700000015),
                {
                    "missionPlanID": 700000015,
                    "snapshotOrigin": "carry_forward_seed",
                    "missionCount": 4,
                    "missions": [
                        {
                            **area_entry,
                            "missionPlanID": 700000015,
                        },
                        *self._line_only_live_snapshot()["missions"],
                    ],
                },
            )

            written_path = mission_area_replan_store.save_snapshot(
                700000015,
                self._line_only_live_snapshot(),
            )

            saved = json.loads(written_path.read_text(encoding="utf-8"))
            saved_by_input = {
                int(mission["inputMissionID"]): mission
                for mission in saved.get("missions") or []
                if isinstance(mission, dict) and mission.get("inputMissionID") is not None
            }
            self.assertEqual(set(saved_by_input), {70000003, 70000004, 70000005, 70000009})
            self.assertFalse(saved_by_input[70000003]["isDone"])
            self.assertEqual(saved_by_input[70000003]["missionPlanID"], 700000015)
            self.assertEqual(saved_by_input[70000003]["remainingAreaM2"], 1250.0)

    def test_live_save_does_not_restore_area_removed_from_target_plan(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir, patch.object(
            mission_area_replan_store,
            "_detail_dir",
            return_value=Path(temp_dir),
        ), patch.object(
            mission_area_replan_store,
            "_target_plan_input_kinds",
            return_value={
                70000004: "line",
                70000005: "line",
                70000009: "line",
            },
        ), patch.object(
            mission_area_replan_store,
            "_target_plan_active_area_input_ids",
            return_value={70000003},
        ):
            mission_area_replan_store._write_central_ledger(
                {
                    "entries": {
                        "area:70000003": {
                            "mission": self._unfinished_area_entry(),
                        }
                    }
                }
            )

            written_path = mission_area_replan_store.save_snapshot(
                700000015,
                self._line_only_live_snapshot(),
            )

            saved = json.loads(written_path.read_text(encoding="utf-8"))
            self.assertEqual(
                [int(mission["inputMissionID"]) for mission in saved.get("missions") or []],
                [70000004, 70000005, 70000009],
            )

    def test_live_save_does_not_restore_completed_area_from_central_ledger(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir, patch.object(
            mission_area_replan_store,
            "_detail_dir",
            return_value=Path(temp_dir),
        ), patch.object(
            mission_area_replan_store,
            "_target_plan_input_kinds",
            return_value={70000003: "area", 70000004: "line"},
        ), patch.object(
            mission_area_replan_store,
            "_target_plan_active_area_input_ids",
            return_value={70000003},
        ):
            completed_area = self._unfinished_area_entry()
            completed_area.update(
                {
                    "isDone": True,
                    "remainingAreaM2": 0.0,
                    "remainingDetail": {
                        "coordinateList": [],
                        "lineList": [],
                        "areaList": [],
                    },
                }
            )
            mission_area_replan_store._write_central_ledger(
                {
                    "entries": {
                        "area:70000003": {
                            "mission": completed_area,
                        }
                    }
                }
            )

            written_path = mission_area_replan_store.save_snapshot(
                700000015,
                self._line_only_live_snapshot(),
            )

            saved = json.loads(written_path.read_text(encoding="utf-8"))
            self.assertNotIn(
                70000003,
                {
                    int(mission["inputMissionID"])
                    for mission in saved.get("missions") or []
                    if isinstance(mission, dict) and mission.get("inputMissionID") is not None
                },
            )

    def test_live_save_does_not_restore_stale_area_when_target_input_is_done(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir, patch.object(
            mission_area_replan_store,
            "_detail_dir",
            return_value=Path(temp_dir),
        ), patch.object(
            mission_area_replan_store,
            "_target_plan_input_kinds",
            return_value={70000003: "area", 70000004: "line"},
        ), patch.object(
            mission_area_replan_store,
            "_target_plan_active_area_input_ids",
            return_value=set(),
        ):
            mission_area_replan_store._write_central_ledger(
                {
                    "entries": {
                        "area:70000003": {
                            # The ledger is deliberately stale; the target
                            # input package is the completion authority here.
                            "mission": self._unfinished_area_entry(),
                        }
                    }
                }
            )

            written_path = mission_area_replan_store.save_snapshot(
                700000015,
                self._line_only_live_snapshot(),
            )

            saved = json.loads(written_path.read_text(encoding="utf-8"))
            self.assertNotIn(
                70000003,
                {
                    int(mission["inputMissionID"])
                    for mission in saved.get("missions") or []
                    if isinstance(mission, dict) and mission.get("inputMissionID") is not None
                },
            )

    def test_live_save_restores_current_area_but_not_future_blocked_area(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir, patch.object(
            mission_area_replan_store,
            "_detail_dir",
            return_value=Path(temp_dir),
        ), patch.object(
            mission_area_replan_store,
            "_target_plan_input_kinds",
            return_value={
                70000003: "area",
                70000011: "area",
                70000004: "line",
            },
        ), patch.object(
            mission_area_replan_store,
            "_target_plan_active_area_input_ids",
            # 70000011 is unfinished but execution-blocked in the target IMP.
            return_value={70000003},
        ):
            mission_area_replan_store._write_central_ledger(
                {
                    "entries": {
                        "area:70000003": {
                            "mission": self._unfinished_area_entry(),
                        },
                        "area:70000011": {
                            "mission": self._unfinished_area_entry(input_mission_id=70000011),
                        },
                    }
                }
            )

            written_path = mission_area_replan_store.save_snapshot(
                700000015,
                self._line_only_live_snapshot(),
            )

            saved = json.loads(written_path.read_text(encoding="utf-8"))
            saved_ids = {
                int(mission["inputMissionID"])
                for mission in saved.get("missions") or []
                if isinstance(mission, dict) and mission.get("inputMissionID") is not None
            }
            self.assertIn(70000003, saved_ids)
            self.assertNotIn(70000011, saved_ids)

    def test_explicit_live_area_completion_is_not_replaced_by_stale_ledger(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir, patch.object(
            mission_area_replan_store,
            "_detail_dir",
            return_value=Path(temp_dir),
        ), patch.object(
            mission_area_replan_store,
            "_target_plan_input_kinds",
            return_value={70000003: "area", 70000004: "line"},
        ), patch.object(
            mission_area_replan_store,
            "_target_plan_active_area_input_ids",
            return_value={70000003},
        ):
            mission_area_replan_store._write_central_ledger(
                {
                    "entries": {
                        "area:70000003": {
                            "mission": self._unfinished_area_entry(),
                        }
                    }
                }
            )
            live = self._line_only_live_snapshot()
            live["missions"].append(
                {
                    "missionPlanID": 700000015,
                    "inputMissionID": 70000003,
                    "missionType": "area",
                    "isDone": True,
                    "remainingAreaM2": 0.0,
                    "remainingDetail": {
                        "coordinateList": [],
                        "lineList": [],
                        "areaList": [],
                    },
                }
            )
            live["missionCount"] = len(live["missions"])

            written_path = mission_area_replan_store.save_snapshot(700000015, live)

            saved = json.loads(written_path.read_text(encoding="utf-8"))
            area_entries = [
                mission
                for mission in saved.get("missions") or []
                if isinstance(mission, dict) and mission.get("inputMissionID") == 70000003
            ]
            self.assertEqual(len(area_entries), 1)
            self.assertTrue(area_entries[0]["isDone"])
            self.assertEqual(area_entries[0]["remainingAreaM2"], 0.0)

    def test_target_plan_active_area_ids_select_only_current_uav_input(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            db_root = Path(temp_dir)

            def write_db_json(section: str, filename: str, payload: dict) -> None:
                path = db_root / section / filename
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(
                    json.dumps(payload, ensure_ascii=False, indent=2),
                    encoding="utf-8",
                )

            write_db_json(
                "MissionPlan",
                "700000015.json",
                {
                    "missionPlanID": 700000015,
                    "inputMissionPackageID": 102,
                    "aircraftList": [
                        {
                            "aircraftID": 1,
                            "individualMissionPackageID": 800000001,
                        },
                        {
                            "aircraftID": 4,
                            "individualMissionPackageID": 800000004,
                        },
                    ],
                },
            )
            write_db_json(
                "InputMissionPlan",
                "102.json",
                {
                    "inputMissionPackageID": 102,
                    "inputMissionList": [
                        {
                            "inputMissionID": 70000002,
                            "inputMissionType": 2,
                            "isDone": True,
                        },
                        {
                            "inputMissionID": 70000003,
                            "inputMissionType": 2,
                            "isDone": False,
                        },
                        {
                            "inputMissionID": 70000011,
                            "inputMissionType": 2,
                            "isDone": False,
                        },
                    ],
                },
            )
            write_db_json(
                "IndividualMissionPlan",
                "800000001.json",
                {
                    "aircraftID": 1,
                    "individualMissionList": [
                        {
                            "individualMissionID": 900000001,
                            "isDone": False,
                            "relatedMission": {"inputMissionID": 70000011},
                        }
                    ],
                },
            )
            write_db_json(
                "IndividualMissionPlan",
                "800000004.json",
                {
                    "aircraftID": 4,
                    "individualMissionList": [
                        {
                            "individualMissionID": 900000002,
                            "isDone": False,
                            "relatedMission": {"inputMissionID": 70000002},
                        },
                        {
                            # A completed boundary hold must not erase the still
                            # unfinished input-level AREA obligation.
                            "individualMissionID": 900000003,
                            "isDone": True,
                            "relatedMission": {"inputMissionID": 70000003},
                        },
                        {
                            "individualMissionID": 900000011,
                            "isDone": False,
                            "executionBlockedUntilNextCollab": True,
                            "relatedMission": {"inputMissionID": 70000011},
                        },
                    ],
                },
            )

            def db_path(section: str, filename: str) -> Path:
                return db_root / section / filename

            with patch.object(
                mission_area_replan_store.db_paths,
                "get_db_subpath",
                side_effect=db_path,
            ):
                active_ids = (
                    mission_area_replan_store._target_plan_active_area_input_ids(
                        700000015
                    )
                )

            self.assertEqual(active_ids, {70000003})


if __name__ == "__main__":
    unittest.main()
