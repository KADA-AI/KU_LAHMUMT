from __future__ import annotations

import importlib
import os
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
PACKAGE = "modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304"


def configure_import_paths() -> None:
    for path in reversed(
        (
            PROJECT_ROOT,
            PROJECT_ROOT / "modules",
            PROJECT_ROOT / "modules" / "mission_planning",
            PROJECT_ROOT / "modules" / "mission_planning" / "MissionPlanner",
        )
    ):
        path_str = str(path)
        if path.exists() and path_str not in sys.path:
            sys.path.insert(0, path_str)


def fail(message: str) -> None:
    raise AssertionError(message)


def import_artifact_module(module_name: str):
    return importlib.import_module(f"{PACKAGE}.{module_name}")


def check_sw_code_mapping() -> None:
    previous_role = os.environ.get("KU_ROLE")
    try:
        os.environ["KU_ROLE"] = "mission"
        for module_name in ("d0301", "d0302", "d0303", "d0304"):
            module = import_artifact_module(module_name)
            if module._sw_code() != "MMR":
                fail(f"{module_name} _sw_code() did not resolve mission role to MMR")

            old = importlib.import_module(f"modules.mission_planning.MissionPlanner.data_def.{module_name}")
            bare = importlib.import_module(f"data_def.{module_name}")
            if old is not module:
                fail(f"{module_name} old import identity split")
            if bare is not module:
                fail(f"{module_name} bare import identity split")
            if old._sw_code() != "MMR" or bare._sw_code() != "MMR":
                fail(f"{module_name} compatibility path did not preserve mission role SW code")
    finally:
        if previous_role is None:
            os.environ.pop("KU_ROLE", None)
        else:
            os.environ["KU_ROLE"] = previous_role


def check_minimal_0301_0302_outputs() -> None:
    previous_role = os.environ.get("KU_ROLE")
    try:
        os.environ["KU_ROLE"] = "mission"
        d0301 = import_artifact_module("d0301")
        d0302 = import_artifact_module("d0302")

        original_d0301_reserve_mission_plan_ids = d0301.reserve_mission_plan_ids
        original_d0301_reserve_imp_ids = d0301.reserve_imp_ids
        original_d0302_reserve_imp_ids = d0302.reserve_imp_ids
        original_d0302_reserve_individual_mission_ids = d0302.reserve_individual_mission_ids
        original_d0302_reserve_path_ids = d0302.reserve_path_ids
        original_d0302_reserve_path_id_blocks = d0302.reserve_path_id_blocks
        try:
            d0301.reserve_mission_plan_ids = lambda count: [700_000_001 + idx for idx in range(count)]
            d0301.reserve_imp_ids = lambda count: [800_000_001 + idx for idx in range(count)]
            d0302.reserve_imp_ids = lambda count: [800_000_001 + idx for idx in range(count)]
            d0302.reserve_individual_mission_ids = lambda count: [900_000_001 + idx for idx in range(count)]
            d0302.reserve_path_ids = lambda aircraft_id, count: [
                aircraft_id * 100_000_000 + 1 + idx for idx in range(count)
            ]
            d0302.reserve_path_id_blocks = lambda request: {
                int(aircraft_id): [int(aircraft_id) * 100_000_000 + 1 + idx for idx in range(count)]
                for aircraft_id, count in dict(request).items()
            }

            plan = d0301.build_mission_plan(
                aircraft_pool=[{"aircraftID": 4, "individualMissionPackageID": 800_000_001}],
                input_mission_package_id=1,
                mission_reference_package_id=2,
                mission_plan_id=700_000_001,
            )
            if plan.get("Source") != "MMR":
                fail(f"0301 Source baseline changed: {plan.get('Source')!r}")

            packages = d0302.build_mission_packages(
                [
                    {
                        "aircraftID": 4,
                        "pathID": 400_000_001,
                        "isDone": False,
                        "relatedMission": {
                            "relatedMissionType": 1,
                            "inputMissionID": 1,
                            "priorMissionID": 0,
                        },
                        "individualMissionInfo": {
                            "individualMissionType": 5,
                            "inputMissionID": 1,
                            "coordinateList": [
                                {"latitude": 37.0, "longitude": 127.0, "altitude": 1000}
                            ],
                        },
                    }
                ],
                cmpk_id=1,
                plan_pkg_map={4: 800_000_001},
                reserved_individual_mission_ids=[900_000_001],
            )
            if len(packages) != 1:
                fail(f"0302 package baseline count changed: {len(packages)}")
            if packages[0].get("Source") != "MMR":
                fail(f"0302 Source baseline changed: {packages[0].get('Source')!r}")
        finally:
            d0301.reserve_mission_plan_ids = original_d0301_reserve_mission_plan_ids
            d0301.reserve_imp_ids = original_d0301_reserve_imp_ids
            d0302.reserve_imp_ids = original_d0302_reserve_imp_ids
            d0302.reserve_individual_mission_ids = original_d0302_reserve_individual_mission_ids
            d0302.reserve_path_ids = original_d0302_reserve_path_ids
            d0302.reserve_path_id_blocks = original_d0302_reserve_path_id_blocks
    finally:
        if previous_role is None:
            os.environ.pop("KU_ROLE", None)
        else:
            os.environ["KU_ROLE"] = previous_role


def check_source_field_assembly_contract() -> None:
    expected_fragments = {
        "d0301": '"Source":_sw_code()',
        "d0302": '("Source",_sw_code())',
        "d0303": '("Source",_sw_code())',
        "d0304": '("Source",_sw_code())',
    }
    for module_name, fragment in expected_fragments.items():
        module = import_artifact_module(module_name)
        path = Path(module.__file__)
        source = "".join(path.read_text(encoding="utf-8", errors="ignore").split())
        if fragment not in source:
            fail(f"{module_name} Source assembly no longer uses _sw_code()")


def main() -> int:
    configure_import_paths()
    check_sw_code_mapping()
    check_minimal_0301_0302_outputs()
    check_source_field_assembly_contract()
    print("0301-0304 mission-role SW code baseline smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
