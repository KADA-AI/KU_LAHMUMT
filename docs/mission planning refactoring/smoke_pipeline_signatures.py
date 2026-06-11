from __future__ import annotations

import argparse
import importlib
import inspect
import os
import sys
from dataclasses import dataclass
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]


@dataclass(frozen=True)
class SignatureCase:
    name: str
    module: str
    attr: str
    signature: str


SIGNATURE_CASES: tuple[SignatureCase, ...] = (
    SignatureCase(
        "attack",
        "modules.mission_planning.replanning.triggers.attack.pipeline",
        "run_attack_plan_pipeline",
        "(ctx: 'Dict[str, Any]', log_callback: 'Optional[LogCallback]' = None) -> 'Dict[str, Any]'",
    ),
    SignatureCase(
        "prior",
        "modules.mission_planning.replanning.triggers.prior.pipeline",
        "run_prior_mission_pipeline",
        "(ctx: 'Dict[str, Any]', detail: 'Dict[str, Any]', reason: 'str', *, log: 'Callable[[str], None]') -> 'Optional[PriorMissionPipelineResult]'",
    ),
    SignatureCase(
        "next_collab",
        "modules.mission_planning.replanning.triggers.next_collab.pipeline",
        "run_next_collab_replan_pipeline",
        "(ctx: 'Dict[str, Any]', detail: 'Dict[str, Any]', reason: 'str', *, log: 'Callable[[str], None]') -> 'Optional[NextCollabPipelineResult]'",
    ),
    SignatureCase(
        "path_deviation",
        "modules.mission_planning.replanning.triggers.path_deviation.pipeline",
        "run_path_deviation_replan_pipeline",
        "(ctx: 'Dict[str, Any]', detail: 'Dict[str, Any]', reason: 'str', *, log: 'Callable[[str], None]') -> 'Optional[PathDeviationPipelineResult]'",
    ),
    SignatureCase(
        "imaging_schedule",
        "modules.mission_planning.replanning.triggers.imaging_schedule.pipeline",
        "run_imaging_schedule_replan_pipeline",
        "(ctx: 'Dict[str, Any]', detail: 'Dict[str, Any]', reason: 'str', *, log: 'Callable[[str], None]') -> 'Optional[ImagingSchedulePipelineResult]'",
    ),
    SignatureCase(
        "post_attack",
        "modules.mission_planning.replanning.triggers.post_attack.pipeline",
        "run_post_attack_rejoin_pipeline",
        "(ctx: 'Dict[str, Any]', detail: 'Dict[str, Any]', reason: 'str', *, log: 'Optional[LogCallback]' = None) -> 'PostAttackRejoinPipelineResult'",
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


def current_signature(case: SignatureCase) -> str:
    module = importlib.import_module(case.module)
    func = getattr(module, case.attr)
    if not callable(func):
        raise RuntimeError(f"{case.module}.{case.attr} is not callable")
    return str(inspect.signature(func))


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke snapshot public run_* pipeline signatures.")
    parser.add_argument("--print-current", action="store_true")
    args = parser.parse_args()

    try:
        configure_import_paths()
        failures: list[str] = []
        for case in SIGNATURE_CASES:
            actual = current_signature(case)
            if args.print_current:
                print(f"{case.name}\t{case.module}.{case.attr}\t{actual}")
            if actual != case.signature:
                failures.append(
                    f"{case.name}: {case.module}.{case.attr} signature changed: {actual!r} != {case.signature!r}"
                )
        if failures:
            raise RuntimeError("\n".join(failures))
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    if not args.print_current:
        print(f"mission pipeline signature smoke ok ({len(SIGNATURE_CASES)} functions)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
