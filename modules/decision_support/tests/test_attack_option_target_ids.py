from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from modules.common.Tabs.csc_tab_base import CSCTabBase
from modules.common.Tabs.decision_support_tab import DecisionSupportTab
from modules.decision_support.core.option_processing import OptionPayloadBuilder


class _TemporaryDbPaths:
    def __init__(self, root: Path) -> None:
        self.root = root

    def get_db_subpath(self, *parts: str) -> Path:
        return self.root.joinpath(*parts)


class AttackOptionTargetIdTests(unittest.TestCase):
    def test_decision_tab_does_not_dispatch_0901_business_logic_twice(self) -> None:
        self.assertIs(DecisionSupportTab.mark_received, CSCTabBase.mark_received)

    def test_attack_option_recovers_target_from_generated_0302(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            plan_dir = root / "MissionPlan"
            imp_dir = root / "IndividualMissionPlan"
            plan_dir.mkdir()
            imp_dir.mkdir()

            (plan_dir / "700000002.json").write_text(
                json.dumps(
                    {
                        "missionPlanID": 700000002,
                        "aircraftList": [
                            {
                                "aircraftID": 2,
                                "individualMissionPackageID": 800000008,
                            },
                            {
                                "aircraftID": 1,
                                "individualMissionPackageID": 800000009,
                            },
                        ],
                    }
                ),
                encoding="utf-8",
            )
            (imp_dir / "800000008.json").write_text(
                json.dumps(
                    {
                        "individualMissionList": [
                            {
                                "individualMissionInfo": {
                                    "individualMissionType": 2,
                                    "targetID": 7,
                                }
                            }
                        ]
                    }
                ),
                encoding="utf-8",
            )
            # Support hold/resume missions must not invent additional attack targets.
            (imp_dir / "800000009.json").write_text(
                json.dumps(
                    {
                        "individualMissionList": [
                            {
                                "individualMissionInfo": {
                                    "individualMissionType": 9,
                                    "targetID": 7,
                                }
                            }
                        ]
                    }
                ),
                encoding="utf-8",
            )

            builder = OptionPayloadBuilder(_TemporaryDbPaths(root))
            option_list = builder.build_option_list(
                [
                    {
                        "optionID": 1,
                        "optionName": 2,
                        "missionPlanID": 700000002,
                    }
                ]
            )

            self.assertEqual(option_list[0]["targetIDListN"], 1)
            self.assertEqual(option_list[0]["targetIDList"], [{"targetID": 7}])

    def test_attack_pipeline_metadata_shape_is_supported(self) -> None:
        builder = OptionPayloadBuilder(_TemporaryDbPaths(Path("missing")))
        option_list = builder.build_option_list(
            [
                {
                    "optionID": 1,
                    "optionName": 2,
                    "missionPlanID": 700000002,
                    "optionMeta": {
                        "attack": True,
                        "attackTargetCount": 1,
                        "attackTargets": [{"targetID": 23}],
                        "primaryTarget": {"targetID": 23},
                    },
                }
            ]
        )

        self.assertEqual(option_list[0]["targetIDListN"], 1)
        self.assertEqual(option_list[0]["targetIDList"], [{"targetID": 23}])


if __name__ == "__main__":
    unittest.main()
