from __future__ import annotations

import argparse
import importlib
import inspect
import os
import sys
from dataclasses import dataclass
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
ARTIFACT_PACKAGE = "modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304"


@dataclass(frozen=True)
class BuilderSignatureCase:
    module: str
    attr: str
    signature: str

    @property
    def canonical(self) -> str:
        return f"{ARTIFACT_PACKAGE}.{self.module}"

    @property
    def wrappers(self) -> tuple[str, str]:
        return (
            f"modules.mission_planning.MissionPlanner.data_def.{self.module}",
            f"data_def.{self.module}",
        )


SIGNATURE_CASES: tuple[BuilderSignatureCase, ...] = (
    BuilderSignatureCase(
        "d0301",
        "build_mission_plan",
        "(*, aircraft_pool: 'List[Dict]', input_mission_package_id: 'int | str', mission_reference_package_id: 'int | str', mission_plan_id: 'int | str | None' = None, planner_id: 'int' = 1, planning_time_s: 'float' = 0.0) -> 'Dict'",
    ),
    BuilderSignatureCase(
        "d0302",
        "build_mission_packages",
        "(missions: 'list[dict]', *, cmpk_id: 'int', plan_pkg_map: 'dict[int, int] | None' = None, reserved_individual_mission_ids: 'list[int] | tuple[int, ...] | None' = None) -> 'list[dict]'",
    ),
    BuilderSignatureCase(
        "d0303",
        "build_flight_plans",
        "(missions: 'list[dict]', wp_alloc: '_WPAllocator | None' = None, cruise_speed: 'float' = 30.0, turn_step_deg: 'float' = 45.0, ref0203: 'dict | None' = None) -> 'list[dict]'",
    ),
    BuilderSignatureCase(
        "d0303",
        "set_flyover_options",
        "(*, entry_offset: 'bool' = False, dubins_prefix: 'bool' = False, last_point: 'bool' = False, all_wps: 'bool' = False) -> 'None'",
    ),
    BuilderSignatureCase(
        "d0303",
        "reset_dense_linesearch_metrics",
        "() -> 'None'",
    ),
    BuilderSignatureCase(
        "d0303",
        "get_dense_linesearch_metrics",
        "(*, reset: 'bool' = False) -> 'dict'",
    ),
    BuilderSignatureCase(
        "d0304",
        "build_lah_flight_plans_fixed",
        "(missions: 'List[dict]', *, cruise_speed: 'float' = 30.0, wp_interval_m: 'float' = 3000.0, manned_plan_mode: 'str' = 'normal', lah_path_mode: 'str' = 'linear', lah_rl_hex_step: 'int' = 50, lah_rl_area_km: 'float' = 10.0, wp_alloc: '_WPAllocator | None' = None) -> 'List[dict]'",
    ),
    BuilderSignatureCase(
        "d0304",
        "build_lah_flight_plans_from_mrpk",
        "(missions: 'List[dict]', mrpk: 'dict', *, cruise_speed: 'float' = 30.0, wp_interval_m: 'float' = 3000.0, manned_plan_mode: 'str' = 'normal', wp_alloc: '_WPAllocator | None' = None) -> 'List[dict]'",
    ),
    BuilderSignatureCase(
        "d0304",
        "apply_uav_eta_follow_speed_plan",
        "(lah_packets: 'List[dict]', uav_packets: 'List[dict]') -> 'List[dict]'",
    ),
)


def configure_import_paths(project_root: Path = PROJECT_ROOT) -> None:
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    os.environ.setdefault("KU_ROLE", "mission")
    desired = (
        project_root,
        project_root / "modules",
        project_root / "modules" / "mission_planning",
        project_root / "modules" / "mission_planning" / "MissionPlanner",
    )
    for path in reversed(desired):
        path_str = str(path)
        if not path.exists():
            continue
        while path_str in sys.path:
            sys.path.remove(path_str)
        sys.path.insert(0, path_str)


def current_signature(module_name: str, attr: str) -> str:
    module = importlib.import_module(module_name)
    value = getattr(module, attr)
    if not callable(value):
        raise RuntimeError(f"{module_name}.{attr} is not callable")
    return str(inspect.signature(value))


def check_case(case: BuilderSignatureCase) -> None:
    canonical_module = importlib.import_module(case.canonical)
    canonical_value = getattr(canonical_module, case.attr)
    actual = current_signature(case.canonical, case.attr)
    if actual != case.signature:
        raise RuntimeError(
            f"{case.canonical}.{case.attr} signature changed: {actual!r} != {case.signature!r}"
        )
    for wrapper_name in case.wrappers:
        wrapper_module = importlib.import_module(wrapper_name)
        wrapper_value = getattr(wrapper_module, case.attr)
        if wrapper_value is not canonical_value:
            raise RuntimeError(f"{wrapper_name}.{case.attr} identity split from canonical")
        wrapper_sig = str(inspect.signature(wrapper_value))
        if wrapper_sig != case.signature:
            raise RuntimeError(
                f"{wrapper_name}.{case.attr} signature changed: {wrapper_sig!r} != {case.signature!r}"
            )


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke snapshot d0301-d0304 artifact builder signatures.")
    parser.add_argument("--print-current", action="store_true")
    args = parser.parse_args()

    try:
        configure_import_paths()
        for case in SIGNATURE_CASES:
            if args.print_current:
                print(f"{case.module}\t{case.attr}\t{current_signature(case.canonical, case.attr)}")
            check_case(case)
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    if not args.print_current:
        print(f"artifact builder signature smoke ok ({len(SIGNATURE_CASES)} functions)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
