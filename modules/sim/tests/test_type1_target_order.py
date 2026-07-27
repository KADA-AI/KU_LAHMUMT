from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from modules.monitoring.logic.anti_armor_air_strike_review import (
    build_anti_armor_target_order_payload,
    describe_anti_armor_target_order,
    detect_anti_armor_target_order_change,
)
from modules.sim.mission.input_mission_reissue import (
    load_type1_target_order_candidates,
    prepare_type1_new_target_input_mission_0201,
    prepare_type1_target_order_input_mission_0201,
)
from modules.sim.mission.mission_plan_loader import normalize_input_mission_plan_float_fields


def _coord(latitude: float, longitude: float) -> dict[str, float]:
    return {"latitude": latitude, "longitude": longitude, "altitude": 300.0}


def _line(
    mission_id: int,
    region_type: int,
    start: tuple[float, float],
    end: tuple[float, float],
) -> dict[str, object]:
    return {
        "inputMissionID": mission_id,
        "inputMissionType": 1,
        "regionType": region_type,
        "isDone": True,
        "missionDetail": {
            "coordinateList": None,
            "lineList": [
                {
                    "width": 1000,
                    "coordinateList": [_coord(*start), _coord(*end)],
                }
            ],
            "areaList": None,
        },
    }


def _area(
    mission_id: int,
    region_type: int,
    center: tuple[float, float],
) -> dict[str, object]:
    latitude, longitude = center
    coordinates = [
        _coord(latitude - 0.004, longitude - 0.004),
        _coord(latitude - 0.004, longitude + 0.004),
        _coord(latitude + 0.004, longitude + 0.004),
        _coord(latitude + 0.004, longitude - 0.004),
    ]
    return {
        "inputMissionID": mission_id,
        "inputMissionType": 2,
        "regionType": region_type,
        "isDone": True,
        "missionDetail": {
            "coordinateList": None,
            "lineList": None,
            "areaList": [{"isHole": False, "coordinateList": coordinates}],
        },
    }


def _reviewed_payload(*, package_id: int = 103, include_second_target: bool = True) -> dict[str, object]:
    attack_wait = (38.070, 127.245)
    battle_one = (38.100, 127.300)
    target_one = (38.142, 127.294)
    battle_two = (38.098, 127.270)
    target_two = (38.135, 127.248)
    acp = (38.054, 127.346)

    missions: list[dict[str, object]] = [
        _line(100, 3, (38.017, 127.293), (38.054, 127.268)),
        _line(101, 4, (38.054, 127.268), attack_wait),
        _area(102, 4, attack_wait),
        _line(103, 5, attack_wait, battle_one),
        _area(104, 5, battle_one),
        _line(105, 6, battle_one, target_one),
        _area(106, 6, target_one),
        _line(107, 5, target_one, battle_one),
    ]
    if include_second_target:
        missions.extend(
            [
                _line(108, 4, battle_one, attack_wait),
                _line(109, 5, attack_wait, battle_two),
                _area(110, 5, battle_two),
                _line(111, 6, battle_two, target_two),
                _area(112, 6, target_two),
                _line(113, 5, target_two, battle_two),
            ]
        )
        last_battle = battle_two
    else:
        last_battle = battle_one
    missions.extend(
        [
            _line(114, 3, last_battle, acp),
            _line(115, 2, acp, (38.080, 127.365)),
        ]
    )
    return {
        "timestamp": 1,
        "inputMissionPackageID": package_id,
        "inputMissionPackageType": 1,
        "inputMissionList": missions,
        "reviewSource": "MSM",
        "reviewKind": "antiArmorNewTargetRefresh",
        "reviewedFromInputMissionPackageID": package_id - 1,
    }


class Type1TargetOrderTests(unittest.TestCase):
    def test_reorders_complete_target_bundles_and_is_detected(self) -> None:
        source = _reviewed_payload()
        description = describe_anti_armor_target_order(source)
        self.assertEqual(description["targetInputMissionIDs"], [106, 112])

        result = build_anti_armor_target_order_payload(
            source,
            ordered_target_input_mission_ids=[112, 106],
            new_package_id=104,
            timestamp_ms=777,
        )

        missions = result.payload["inputMissionList"]
        self.assertEqual(
            [mission["inputMissionID"] for mission in missions],
            [100, 101, 102, 109, 110, 111, 112, 113, 108, 103, 104, 105, 106, 107, 114, 115],
        )
        self.assertEqual(describe_anti_armor_target_order(result.payload)["targetInputMissionIDs"], [112, 106])
        self.assertEqual(result.payload["inputMissionPackageID"], 104)
        self.assertEqual(result.payload["timestamp"], 777)
        self.assertTrue(all(mission["isDone"] is False for mission in missions))
        self.assertNotIn("reviewSource", result.payload)
        self.assertNotIn("reviewKind", result.payload)
        self.assertNotIn("reviewedFromInputMissionPackageID", result.payload)
        detected = detect_anti_armor_target_order_change(source, result.payload)
        self.assertIsNotNone(detected)
        self.assertEqual(detected["currentTargetInputMissionIDs"], [112, 106])

    def test_candidate_loading_and_0201_file_creation(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            input_dir = root / "InputMissionPlan"
            input_dir.mkdir(parents=True)
            original = _reviewed_payload(package_id=101, include_second_target=False)
            source = _reviewed_payload(package_id=103, include_second_target=True)
            (input_dir / "101.json").write_text(json.dumps(original), encoding="utf-8")
            (input_dir / "103.json").write_text(json.dumps(source), encoding="utf-8")

            candidates = load_type1_target_order_candidates(db_root=root)
            self.assertTrue(candidates["ok"])
            self.assertEqual(candidates["sourcePackageID"], 103)
            self.assertEqual(
                [(row["targetInputMissionID"], row["isNew"]) for row in candidates["targets"]],
                [(106, False), (112, True)],
            )

            prepared = prepare_type1_target_order_input_mission_0201(
                ordered_target_input_mission_ids=[112, 106],
                source_package_id=103,
                db_root=root,
                now_ms=lambda: 888,
            )
            self.assertTrue(prepared["ok"])
            self.assertEqual(prepared["newPackageID"], 104)
            output = json.loads((input_dir / "104.json").read_text(encoding="utf-8"))
            self.assertEqual(output["timestamp"], 888)
            self.assertEqual(describe_anti_armor_target_order(output)["targetInputMissionIDs"], [112, 106])
            self.assertIsNotNone(detect_anti_armor_target_order_change(source, output))
            output_altitudes = [
                coord["altitude"]
                for mission in output["inputMissionList"]
                for area in (mission.get("missionDetail") or {}).get("areaList") or []
                for coord in area.get("coordinateList") or []
            ]
            self.assertTrue(output_altitudes)
            self.assertTrue(all(type(altitude) is int for altitude in output_altitudes))

    def test_new_target_and_legacy_integral_altitudes_are_saved_as_int(self) -> None:
        legacy = {
            "inputMissionList": [
                {
                    "missionDetail": {
                        "coordinateList": [
                            {"latitude": 38.0, "longitude": 127.0, "altitude": 0.0},
                            {"latitude": 38.0, "longitude": 127.1, "altitude": 12.5},
                        ]
                    }
                }
            ]
        }
        normalize_input_mission_plan_float_fields(legacy)
        coordinates = legacy["inputMissionList"][0]["missionDetail"]["coordinateList"]
        self.assertIs(type(coordinates[0]["altitude"]), int)
        self.assertIs(type(coordinates[1]["altitude"]), float)

        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            input_dir = root / "InputMissionPlan"
            input_dir.mkdir(parents=True)
            source = _reviewed_payload(package_id=101, include_second_target=False)
            (input_dir / "101.json").write_text(json.dumps(source), encoding="utf-8")
            prepared = prepare_type1_new_target_input_mission_0201(
                coordinate_list=[
                    {"latitude": 38.130, "longitude": 127.240, "altitude": 0.0},
                    {"latitude": 38.130, "longitude": 127.250, "altitude": 0.0},
                    {"latitude": 38.140, "longitude": 127.250, "altitude": 0.0},
                ],
                source_package_id=101,
                db_root=root,
                now_ms=lambda: 999,
            )
            self.assertTrue(prepared["ok"])
            output = json.loads(Path(prepared["outputPath"]).read_text(encoding="utf-8"))
            target = output["inputMissionList"][8]
            altitudes = [
                coord["altitude"]
                for area in target["missionDetail"]["areaList"]
                for coord in area["coordinateList"]
            ]
            self.assertEqual(altitudes, [0, 0, 0])
            self.assertTrue(all(type(altitude) is int for altitude in altitudes))

    def test_rejects_unchanged_or_incomplete_order(self) -> None:
        source = _reviewed_payload()
        with self.assertRaisesRegex(ValueError, "unchanged"):
            build_anti_armor_target_order_payload(
                source,
                ordered_target_input_mission_ids=[106, 112],
                new_package_id=104,
                timestamp_ms=777,
            )
        with self.assertRaisesRegex(ValueError, "every target"):
            build_anti_armor_target_order_payload(
                source,
                ordered_target_input_mission_ids=[112],
                new_package_id=104,
                timestamp_ms=777,
            )


if __name__ == "__main__":
    unittest.main()
