from __future__ import annotations

import argparse
import importlib
import os
import sys
from dataclasses import dataclass
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]


@dataclass(frozen=True)
class ImportCase:
    group: str
    module: str
    attrs: tuple[str, ...] = ()


ACTIVE_IMPORTS: tuple[ImportCase, ...] = (
    ImportCase("app", "modules.mission_planning.app.bootstrap", ("configure_mission_role",)),
    ImportCase("app", "modules.mission_planning.app.message_handlers.system_mode", ("extract_system_mode_code",)),
    ImportCase("app", "modules.mission_planning.app.message_handlers.input_packages", ("get_input_message_spec",)),
    ImportCase("app", "modules.mission_planning.app.message_handlers.replan_requests", ("parse_replan_payload",)),
    ImportCase("app", "modules.mission_planning.app.delivery.mission_plan_delivery", ("sort_plan_delivery_entries",)),
    ImportCase("app", "modules.mission_planning.app.visualization.mission_visualization_tab", ("MissionVisualizationTab",)),
    ImportCase("mission_control", "modules.mission_planning.mission_control.planner_runtime", ("refresh_live_planning_helpers",)),
    ImportCase("mission_control", "modules.mission_planning.mission_control.plan_metrics", ("count_replan_options",)),
    ImportCase("replanning", "modules.mission_planning.replanning.dispatcher", ("should_use_attack_pipeline",)),
    ImportCase("replanning", "modules.mission_planning.replanning.triggers.attack.pipeline", ("run_attack_plan_pipeline",)),
    ImportCase("replanning", "modules.mission_planning.replanning.triggers.prior.pipeline", ("run_prior_mission_pipeline",)),
    ImportCase("replanning", "modules.mission_planning.replanning.triggers.next_collab.pipeline", ("run_next_collab_replan_pipeline",)),
    ImportCase("replanning", "modules.mission_planning.replanning.triggers.path_deviation.pipeline", ("run_path_deviation_replan_pipeline",)),
    ImportCase("replanning", "modules.mission_planning.replanning.triggers.imaging_schedule.pipeline", ("run_imaging_schedule_replan_pipeline",)),
    ImportCase("replanning", "modules.mission_planning.replanning.triggers.post_attack.pipeline", ("run_post_attack_rejoin_pipeline",)),
    ImportCase("replanning", "modules.mission_planning.replanning.triggers.remaining_hybrid.current", ("build_current_remaining_hybrid",)),
    ImportCase("replanning", "modules.mission_planning.replanning.triggers.remaining_hybrid.current_replan", ("prepare_current_remaining_hybrid_replacements",)),
    ImportCase("replanning", "modules.mission_planning.replanning.triggers.remaining_hybrid.general", ("apply_remaining_hybrid_replan",)),
    ImportCase("replanning", "modules.mission_planning.replanning.triggers.remaining_hybrid.reexecute_first", ("prepare_reexecute_first_mission_replacements",)),
    ImportCase("replanning", "modules.mission_planning.replanning.triggers.recon_specialized.pipeline", ("build_recon_specialized_runtime_payload",)),
    ImportCase("runtime", "modules.mission_planning.runtime.json_io", ("write_json",)),
    ImportCase("runtime", "modules.mission_planning.runtime.debug_artifacts", ("write_debug_json",)),
    ImportCase("runtime", "modules.mission_planning.runtime.aircraft_parallel_0303", ("build_0303_flight_plans_aircraft_parallel",)),
    ImportCase("runtime", "modules.mission_planning.runtime.next_collab_line_runner", ("run_next_collab_line_plan",)),
    ImportCase("runtime", "modules.mission_planning.runtime.next_collab_division_runner", ("run_next_collab_division_plan",)),
    ImportCase("runtime", "modules.mission_planning.runtime.next_collab_heading", ("monitor_heading_to_planner_bearing_deg",)),
    ImportCase("runtime", "modules.mission_planning.runtime.next_collab_replan_runtime", ("load_next_collab_detail",)),
    ImportCase("runtime", "modules.mission_planning.runtime.next_collab_replan_store", ("save_detail",)),
    ImportCase("runtime", "modules.mission_planning.runtime.replan_store", ("load_detail",)),
    ImportCase("runtime", "modules.mission_planning.runtime.state.attack_assignment", ("get_last_assigned_manned_id",)),
    ImportCase("runtime", "modules.mission_planning.runtime.state.attack_tracking", ("resolve_plan_lineage_ids",)),
    ImportCase("runtime", "modules.mission_planning.runtime.state.prior_tracking", ("register_prior_assignment",)),
    ImportCase("runtime", "modules.mission_planning.runtime.cache.source_artifacts", ("SourceArtifactCache",)),
    ImportCase("runtime", "modules.mission_planning.runtime.cache.latest_input", ("update_from_payload",)),
    ImportCase("runtime", "modules.mission_planning.runtime.logging.pipeline_events", ("PipelineLogManager",)),
    ImportCase("runtime", "modules.mission_planning.runtime.logging.plan_file_logger", ("MissionPlanFileLogger",)),
    ImportCase("runtime", "modules.mission_planning.runtime.validation.replan_payloads", ("validate_replan_payloads",)),
    ImportCase("runtime", "modules.mission_planning.runtime.ids.replan_reservation", ("ReplanIdReservation",)),
    ImportCase("engine", "modules.mission_planning.engine.mission_generation.id_allocation.allocator", ("reserve_mission_plan_ids",)),
    ImportCase("engine", "modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304.d0301", ("build_mission_plan",)),
    ImportCase("engine", "modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304.d0302", ("build_mission_packages",)),
    ImportCase("engine", "modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304.d0303", ("build_flight_plans",)),
    ImportCase("engine", "modules.mission_planning.engine.mission_generation.artifacts_0301_0302_0303_0304.d0304", ("build_lah_flight_plans_fixed",)),
    ImportCase("planner", "modules.mission_planning.MissionPlanner.AnS", ("run_divide_and_pattern",)),
    ImportCase("planner", "modules.mission_planning.MissionPlanner.AnS.mission_pipeline", ("build_mission_plan_0301",)),
    ImportCase("planner", "modules.mission_planning.MissionPlanner.data_def", ("build_lah_flight_plans_fixed",)),
    ImportCase("planner", "modules.mission_planning.MissionPlanner.planning_enhanced", ("run_enhanced_divide_and_pattern",)),
    ImportCase("planner", "modules.mission_planning.MissionPlanner.planning_enhanced.io", ("build_0303_0304_from_0302_packages",)),
    ImportCase("planner", "modules.mission_planning.MissionPlanner.planning_enhanced.pathing", ("generate_expected_paths",)),
    ImportCase("planner", "modules.mission_planning.MissionPlanner.planning_enhanced.type_decider", ("apply_logic_type_decider",)),
)


def configure_import_paths(project_root: Path = PROJECT_ROOT) -> None:
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
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


def selected_cases(group: str | None, module: str | None) -> tuple[ImportCase, ...]:
    cases = ACTIVE_IMPORTS
    if group:
        cases = tuple(case for case in cases if case.group == group)
    if module:
        cases = tuple(case for case in cases if case.module == module)
    return cases


def run_import_cases(cases: tuple[ImportCase, ...]) -> list[str]:
    failures: list[str] = []
    for case in cases:
        try:
            imported = importlib.import_module(case.module)
        except Exception as exc:
            failures.append(f"{case.group}: import {case.module} failed: {exc!r}")
            continue
        for attr in case.attrs:
            if not hasattr(imported, attr):
                failures.append(f"{case.group}: {case.module} missing {attr}")
    logging_mod = importlib.import_module("logging")
    if not hasattr(logging_mod, "getLogger"):
        failures.append("stdlib logging was shadowed after active imports")
    return failures


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke import active mission_planning modules.")
    parser.add_argument("--group", choices=sorted({case.group for case in ACTIVE_IMPORTS}))
    parser.add_argument("--module")
    parser.add_argument("--list", action="store_true", help="print import cases without importing")
    args = parser.parse_args()

    cases = selected_cases(args.group, args.module)
    if args.list:
        for case in cases:
            attrs = ",".join(case.attrs) if case.attrs else "-"
            print(f"{case.group}\t{case.module}\t{attrs}")
        return 0

    if not cases:
        print("no active import cases selected", file=sys.stderr)
        return 2

    configure_import_paths()
    failures = run_import_cases(cases)
    if failures:
        print("active mission_planning import smoke failed:", file=sys.stderr)
        for failure in failures:
            print(f"- {failure}", file=sys.stderr)
        return 1
    print(f"active mission_planning import smoke ok ({len(cases)} modules)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
