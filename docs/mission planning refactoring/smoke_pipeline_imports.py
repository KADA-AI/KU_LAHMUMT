from __future__ import annotations

import argparse
import importlib
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


PROJECT_ROOT = Path(__file__).resolve().parents[2]


@dataclass(frozen=True)
class WrapperCase:
    module: str
    attrs: tuple[str, ...] | None = None


@dataclass(frozen=True)
class PipelineCase:
    name: str
    canonical: str
    callables: tuple[str, ...] = ()
    classes: tuple[str, ...] = ()
    values: tuple[str, ...] = ()
    wrappers: tuple[WrapperCase, ...] = ()

    @property
    def attrs(self) -> tuple[str, ...]:
        return self.callables + self.classes + self.values


PIPELINE_CASES: tuple[PipelineCase, ...] = (
    PipelineCase(
        "attack",
        "modules.mission_planning.replanning.triggers.attack.pipeline",
        callables=(
            "run_attack_exclusion_pipeline",
            "run_attack_plan_pipeline",
            "warm_attack_plan_pipeline",
        ),
        wrappers=(
            WrapperCase("modules.mission_planning.replanning.triggers.attack"),
            WrapperCase("modules.mission_planning.attack_plan_pipeline"),
            WrapperCase("modules.mission_planning.legacy.wrappers.attack_plan_pipeline"),
            WrapperCase("modules.mission_planning.pipelines.attack_plan_pipeline"),
        ),
    ),
    PipelineCase(
        "prior",
        "modules.mission_planning.replanning.triggers.prior.pipeline",
        callables=(
            "run_prior_mission_pipeline",
            "warm_prior_mission_pipeline",
            "run_prior_post_rejoin_pipeline",
            "warm_prior_post_rejoin_pipeline",
        ),
        classes=(
            "PriorMissionPipelineResult",
            "PriorPostRejoinPipelineResult",
        ),
        wrappers=(
            WrapperCase(
                "modules.mission_planning.replanning.triggers.prior",
                (
                    "run_prior_mission_pipeline",
                    "warm_prior_mission_pipeline",
                    "run_prior_post_rejoin_pipeline",
                    "warm_prior_post_rejoin_pipeline",
                ),
            ),
            WrapperCase(
                "modules.mission_planning.prior_mission_pipeline",
                ("run_prior_mission_pipeline", "warm_prior_mission_pipeline"),
            ),
            WrapperCase("modules.mission_planning.prior_mission_pipeline_impl"),
            WrapperCase(
                "modules.mission_planning.legacy.wrappers.prior_mission_pipeline",
                ("run_prior_mission_pipeline", "warm_prior_mission_pipeline"),
            ),
            WrapperCase("modules.mission_planning.legacy.wrappers.prior_mission_pipeline_impl"),
            WrapperCase("modules.mission_planning.pipelines.prior_mission_pipeline_impl"),
        ),
    ),
    PipelineCase(
        "next_collab",
        "modules.mission_planning.replanning.triggers.next_collab.pipeline",
        callables=(
            "run_next_collab_replan_pipeline",
            "warm_next_collab_replan_pipeline",
            "prepare_next_collab_input_replacements",
        ),
        classes=("NextCollabPipelineResult",),
        wrappers=(
            WrapperCase(
                "modules.mission_planning.replanning.triggers.next_collab",
                ("run_next_collab_replan_pipeline", "warm_next_collab_replan_pipeline"),
            ),
            WrapperCase(
                "modules.mission_planning.next_collab_replan_pipeline",
                ("run_next_collab_replan_pipeline", "warm_next_collab_replan_pipeline"),
            ),
            WrapperCase(
                "modules.mission_planning.legacy.wrappers.next_collab_replan_pipeline",
                ("run_next_collab_replan_pipeline", "warm_next_collab_replan_pipeline"),
            ),
            WrapperCase(
                "modules.mission_planning.pipelines.next_collab_replan_pipeline",
                (
                    "NextCollabPipelineResult",
                    "run_next_collab_replan_pipeline",
                    "warm_next_collab_replan_pipeline",
                ),
            ),
            WrapperCase("modules.mission_planning.pipelines.next_collab_replan_pipeline_impl"),
        ),
    ),
    PipelineCase(
        "imaging_schedule",
        "modules.mission_planning.replanning.triggers.imaging_schedule.pipeline",
        callables=(
            "run_imaging_schedule_replan_pipeline",
            "warm_imaging_schedule_replan_pipeline",
        ),
        classes=("ImagingSchedulePipelineResult",),
        values=("IMAGING_TRIGGER_TYPE", "QUALITY_TRIGGER_TYPE"),
        wrappers=(
            WrapperCase(
                "modules.mission_planning.replanning.triggers.imaging_schedule",
                ("run_imaging_schedule_replan_pipeline", "warm_imaging_schedule_replan_pipeline"),
            ),
            WrapperCase(
                "modules.mission_planning.imaging_schedule_replan_pipeline",
                ("run_imaging_schedule_replan_pipeline", "warm_imaging_schedule_replan_pipeline"),
            ),
            WrapperCase(
                "modules.mission_planning.legacy.wrappers.imaging_schedule_replan_pipeline",
                ("run_imaging_schedule_replan_pipeline", "warm_imaging_schedule_replan_pipeline"),
            ),
            WrapperCase("modules.mission_planning.pipelines.imaging_schedule_replan_pipeline_impl"),
        ),
    ),
    PipelineCase(
        "path_deviation",
        "modules.mission_planning.replanning.triggers.path_deviation.pipeline",
        callables=(
            "run_path_deviation_replan_pipeline",
            "warm_path_deviation_replan_pipeline",
        ),
        classes=("PathDeviationPipelineResult",),
        wrappers=(
            WrapperCase(
                "modules.mission_planning.replanning.triggers.path_deviation",
                ("run_path_deviation_replan_pipeline", "warm_path_deviation_replan_pipeline"),
            ),
            WrapperCase(
                "modules.mission_planning.path_deviation_replan_pipeline",
                ("run_path_deviation_replan_pipeline", "warm_path_deviation_replan_pipeline"),
            ),
            WrapperCase(
                "modules.mission_planning.legacy.wrappers.path_deviation_replan_pipeline",
                ("run_path_deviation_replan_pipeline", "warm_path_deviation_replan_pipeline"),
            ),
            WrapperCase("modules.mission_planning.pipelines.path_deviation_replan_pipeline_impl"),
        ),
    ),
    PipelineCase(
        "post_attack",
        "modules.mission_planning.replanning.triggers.post_attack.pipeline",
        callables=(
            "run_post_attack_rejoin_pipeline",
            "warm_post_attack_rejoin_pipeline",
        ),
        classes=("PostAttackRejoinPipelineResult",),
        wrappers=(
            WrapperCase(
                "modules.mission_planning.replanning.triggers.post_attack",
                ("run_post_attack_rejoin_pipeline", "warm_post_attack_rejoin_pipeline"),
            ),
            WrapperCase("modules.mission_planning.pipelines.post_attack_rejoin_pipeline"),
        ),
    ),
    PipelineCase(
        "remaining_current",
        "modules.mission_planning.replanning.triggers.remaining_hybrid.current",
        callables=(
            "build_current_remaining_hybrid",
            "filter_generic_flightpath_missions_for_hybrid",
            "merge_current_remaining_hybrid",
            "validate_current_remaining_hybrid_paths",
            "validate_current_remaining_hybrid_request",
        ),
        classes=("CurrentRemainingHybridRequest",),
        wrappers=(
            WrapperCase(
                "modules.mission_planning.replanning.triggers.remaining_hybrid",
                (
                    "build_current_remaining_hybrid",
                    "filter_generic_flightpath_missions_for_hybrid",
                    "merge_current_remaining_hybrid",
                    "validate_current_remaining_hybrid_paths",
                    "validate_current_remaining_hybrid_request",
                    "CurrentRemainingHybridRequest",
                ),
            ),
            WrapperCase("modules.mission_planning.pipelines.current_remaining_hybrid"),
        ),
    ),
    PipelineCase(
        "remaining_current_replan",
        "modules.mission_planning.replanning.triggers.remaining_hybrid.current_replan",
        callables=("prepare_current_remaining_hybrid_replacements",),
        classes=("CurrentRemainingHybridResult",),
        wrappers=(
            WrapperCase("modules.mission_planning.pipelines.current_remaining_hybrid_replan"),
        ),
    ),
    PipelineCase(
        "remaining_general",
        "modules.mission_planning.replanning.triggers.remaining_hybrid.general",
        callables=(
            "apply_remaining_hybrid_replan",
            "validate_remaining_hybrid_source_geometry",
        ),
        classes=("RemainingHybridResult",),
        wrappers=(
            WrapperCase(
                "modules.mission_planning.replanning.triggers.remaining_hybrid",
                (
                    "RemainingHybridResult",
                    "apply_remaining_hybrid_replan",
                    "validate_remaining_hybrid_source_geometry",
                ),
            ),
            WrapperCase("modules.mission_planning.pipelines.general_remaining_hybrid_replan"),
        ),
    ),
    PipelineCase(
        "reexecute_first",
        "modules.mission_planning.replanning.triggers.remaining_hybrid.reexecute_first",
        callables=(
            "prepare_reexecute_first_mission_replacements",
            "reexecute_first_mission_generic_skip_policy",
            "summarize_reexecute_first_mission_option_effect",
            "validate_reexecute_first_mission_inputs",
        ),
        wrappers=(
            WrapperCase(
                "modules.mission_planning.replanning.triggers.remaining_hybrid",
                (
                    "prepare_reexecute_first_mission_replacements",
                    "reexecute_first_mission_generic_skip_policy",
                    "summarize_reexecute_first_mission_option_effect",
                    "validate_reexecute_first_mission_inputs",
                ),
            ),
            WrapperCase("modules.mission_planning.pipelines.reexecute_first_mission_hybrid"),
        ),
    ),
    PipelineCase(
        "recon_specialized",
        "modules.mission_planning.replanning.triggers.recon_specialized.pipeline",
        callables=(
            "build_recon_specialized_runtime_payload",
            "compare_recon_option_path_signatures",
            "is_recon_specialized_option",
            "summarize_recon_area_review_guard",
            "summarize_recon_expected_path_quality",
        ),
        wrappers=(
            WrapperCase("modules.mission_planning.replanning.triggers.recon_specialized"),
            WrapperCase("modules.mission_planning.pipelines.recon_specialized_pipeline"),
        ),
    ),
)


SUPPORT_CASES: tuple[PipelineCase, ...] = (
    PipelineCase(
        "mission_path_trim",
        "modules.mission_planning.pipelines.mission_path_trim",
        callables=(
            "merge_small_adjacent_line_search_waypoints",
            "sweep_cut_points",
            "load_sweep_progress",
        ),
        wrappers=(
            WrapperCase("modules.mission_planning.legacy.wrappers.mission_path_trim"),
        ),
    ),
    PipelineCase(
        "attack_helpers",
        "modules.mission_planning.pipelines.mission_planning_attack_helpers",
        callables=(
            "apply_attack_customizations",
            "build_attack_context_from_replan_detail",
            "compute_attack_waypoint",
            "load_attack_context",
        ),
        wrappers=(
            WrapperCase("modules.mission_planning.legacy.wrappers.mission_planning_attack_helpers"),
        ),
    ),
    PipelineCase(
        "next_collab_path_builder",
        "modules.mission_planning.pipelines.next_collab_path_builder",
        callables=(
            "build_flight_path_from_planned_row",
            "build_formation_flight_path_from_template",
            "build_mission_info_from_planned_row",
        ),
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


def _expect_module(module_name: str):
    try:
        return importlib.import_module(module_name)
    except Exception as exc:
        raise RuntimeError(f"import failed for {module_name}: {exc!r}") from exc


def _check_all_exports(module: object, attrs: Iterable[str], label: str) -> None:
    exported = getattr(module, "__all__", None)
    if exported is None:
        return
    exported_set = set(exported)
    missing = [attr for attr in attrs if attr not in exported_set]
    if missing:
        raise RuntimeError(f"{label} __all__ missing: {', '.join(missing)}")


def _check_case(case: PipelineCase) -> None:
    canonical = _expect_module(case.canonical)
    _check_all_exports(canonical, case.attrs, case.canonical)

    for attr in case.callables:
        value = getattr(canonical, attr, None)
        if not callable(value):
            raise RuntimeError(f"{case.canonical}.{attr} is not callable")

    for attr in case.classes:
        value = getattr(canonical, attr, None)
        if not isinstance(value, type):
            raise RuntimeError(f"{case.canonical}.{attr} is not a class")

    for attr in case.values:
        if not hasattr(canonical, attr):
            raise RuntimeError(f"{case.canonical}.{attr} missing")

    for wrapper_case in case.wrappers:
        wrapper = _expect_module(wrapper_case.module)
        attrs = case.attrs if wrapper_case.attrs is None else wrapper_case.attrs
        _check_all_exports(wrapper, attrs, wrapper_case.module)
        for attr in attrs:
            if not hasattr(wrapper, attr):
                raise RuntimeError(f"{wrapper_case.module}.{attr} missing")
            if getattr(wrapper, attr) is not getattr(canonical, attr):
                raise RuntimeError(
                    f"{wrapper_case.module}.{attr} identity split from {case.canonical}.{attr}"
                )


def selected_cases(group: str) -> tuple[PipelineCase, ...]:
    if group == "trigger":
        return PIPELINE_CASES
    if group == "support":
        return SUPPORT_CASES
    return PIPELINE_CASES + SUPPORT_CASES


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke import mission-planning pipeline entrypoints.")
    parser.add_argument("--group", choices=("all", "trigger", "support"), default="all")
    parser.add_argument("--list", action="store_true")
    args = parser.parse_args()

    cases = selected_cases(args.group)
    if args.list:
        for case in cases:
            print(f"{case.name}\t{case.canonical}\t{','.join(case.attrs)}")
        return 0

    try:
        configure_import_paths()
        for case in cases:
            _check_case(case)
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print(f"mission pipeline import smoke ok ({len(cases)} cases)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
