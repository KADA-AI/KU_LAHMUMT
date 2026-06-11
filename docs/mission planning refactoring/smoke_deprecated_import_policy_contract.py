from __future__ import annotations

import argparse
import ast
import importlib
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
DECISION_DOC = PROJECT_ROOT / "docs" / "mission planning refactoring" / "74-deprecated-import-policy-decision.md"


DEPRECATED_IMPORT_ALIAS_MODULES = (
    "modules.mission_planning.attack_assignment_state",
    "modules.mission_planning.attack_plan_pipeline",
    "modules.mission_planning.id_relationship_tab",
    "modules.mission_planning.imaging_schedule_replan_pipeline",
    "modules.mission_planning.json_io",
    "modules.mission_planning.latest_input_cache",
    "modules.mission_planning.mission_path_trim",
    "modules.mission_planning.mission_plan_file_logger",
    "modules.mission_planning.mission_planning_attack_helpers",
    "modules.mission_planning.mission_planning_gui_env",
    "modules.mission_planning.mission_planning_log_tab",
    "modules.mission_planning.mission_planning_pipeline_logging",
    "modules.mission_planning.next_collab_replan_pipeline",
    "modules.mission_planning.path_deviation_replan_pipeline",
    "modules.mission_planning.prior_mission_pipeline",
    "modules.mission_planning.prior_mission_pipeline_impl",
)


DEPRECATED_IMPORT_SURFACE_FILES = (
    "modules/mission_planning/lah_rl_planner_gui.py",
    "modules/mission_planning/legacy/wrappers/attack_assignment_state.py",
    "modules/mission_planning/legacy/wrappers/attack_plan_pipeline.py",
    "modules/mission_planning/legacy/wrappers/id_relationship_tab.py",
    "modules/mission_planning/legacy/wrappers/imaging_schedule_replan_pipeline.py",
    "modules/mission_planning/legacy/wrappers/json_io.py",
    "modules/mission_planning/legacy/wrappers/latest_input_cache.py",
    "modules/mission_planning/legacy/wrappers/mission_path_trim.py",
    "modules/mission_planning/legacy/wrappers/mission_plan_file_logger.py",
    "modules/mission_planning/legacy/wrappers/mission_planning_attack_helpers.py",
    "modules/mission_planning/legacy/wrappers/mission_planning_gui_env.py",
    "modules/mission_planning/legacy/wrappers/mission_planning_log_tab.py",
    "modules/mission_planning/legacy/wrappers/mission_planning_pipeline_logging.py",
    "modules/mission_planning/legacy/wrappers/next_collab_replan_pipeline.py",
    "modules/mission_planning/legacy/wrappers/path_deviation_replan_pipeline.py",
    "modules/mission_planning/legacy/wrappers/prior_mission_pipeline.py",
    "modules/mission_planning/legacy/wrappers/prior_mission_pipeline_impl.py",
    "modules/mission_planning/pipelines/attack_plan_pipeline.py",
    "modules/mission_planning/pipelines/prior_mission_pipeline_impl.py",
    "modules/mission_planning/pipelines/next_collab_replan_pipeline.py",
    "modules/mission_planning/pipelines/next_collab_replan_pipeline_impl.py",
    "modules/mission_planning/pipelines/current_remaining_hybrid.py",
    "modules/mission_planning/pipelines/current_remaining_hybrid_replan.py",
    "modules/mission_planning/pipelines/general_remaining_hybrid_replan.py",
    "modules/mission_planning/pipelines/reexecute_first_mission_hybrid.py",
    "modules/mission_planning/pipelines/recon_specialized_pipeline.py",
    "modules/mission_planning/pipelines/imaging_schedule_replan_pipeline_impl.py",
    "modules/mission_planning/pipelines/path_deviation_replan_pipeline_impl.py",
    "modules/mission_planning/pipelines/post_attack_rejoin_pipeline.py",
    "modules/mission_planning/runtime/attack_assignment_state.py",
    "modules/mission_planning/runtime/attack_tracking_state.py",
    "modules/mission_planning/runtime/prior_tracking_state.py",
    "modules/mission_planning/runtime/source_artifact_cache.py",
    "modules/mission_planning/runtime/latest_input_cache.py",
    "modules/mission_planning/runtime/mission_planning_pipeline_logging.py",
    "modules/mission_planning/runtime/mission_plan_file_logger.py",
    "modules/mission_planning/runtime/replan_validation.py",
    "modules/mission_planning/runtime/replan_id_reservation.py",
    "modules/mission_planning/MissionPlanner/data_def/d0301.py",
    "modules/mission_planning/MissionPlanner/data_def/d0302.py",
    "modules/mission_planning/MissionPlanner/data_def/d0303.py",
    "modules/mission_planning/MissionPlanner/data_def/d0304.py",
    "modules/mission_planning/MissionPlanner/data_def/id_allocator.py",
)


FORBIDDEN_IMPORT_SIDE_EFFECT_MODULES = {
    "warnings",
    "logging",
    "modules.common.process_console",
    "modules.mission_planning.runtime.logging.pipeline_events",
    "modules.mission_planning.runtime.logging.plan_file_logger",
}


FORBIDDEN_TOP_LEVEL_CALLS = {
    "warn",
    "warning",
    "getLogger",
    "basicConfig",
    "emit_process_log",
    "emit_process_lifecycle_event",
    "PipelineLogManager",
    "MissionPlanFileLogger",
}


def fail(message: str) -> None:
    raise AssertionError(message)


def read_source(rel_path: str) -> str:
    path = PROJECT_ROOT / rel_path
    if not path.exists():
        fail(f"{rel_path} is missing")
    return path.read_text(encoding="utf-8", errors="ignore")


def parse_source(rel_path: str) -> ast.Module:
    try:
        return ast.parse(read_source(rel_path))
    except SyntaxError as exc:
        fail(f"{rel_path} is not parseable: {exc}")


def import_module_name(node: ast.ImportFrom) -> str:
    prefix = "." * int(getattr(node, "level", 0) or 0)
    return prefix + str(node.module or "")


def top_level_call_names(tree: ast.Module) -> set[str]:
    names: set[str] = set()

    def visit_statement(node: ast.AST) -> None:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            return
        for child in ast.iter_child_nodes(node):
            if isinstance(child, ast.Call):
                func = child.func
                if isinstance(func, ast.Name):
                    names.add(func.id)
                elif isinstance(func, ast.Attribute):
                    names.add(func.attr)
            visit_statement(child)

    for statement in tree.body:
        visit_statement(statement)
    return names


def check_decision_doc() -> None:
    text = DECISION_DOC.read_text(encoding="utf-8", errors="ignore")
    required = (
        "Documentation-only deprecated import policy",
        "Do not emit runtime deprecation warnings",
        "No runtime implementation changed",
        "bootstrap import-order contract",
    )
    missing = [snippet for snippet in required if snippet not in text]
    if missing:
        fail(f"{DECISION_DOC.relative_to(PROJECT_ROOT)} missing deprecated import policy markers: {missing!r}")


def check_wrapper_imports_have_no_deprecation_side_effects() -> None:
    offenders: list[str] = []
    alias_source = read_source("modules/mission_planning/__init__.py")
    if "DeprecationWarning" in alias_source or "deprecated import" in alias_source.lower():
        offenders.append("modules/mission_planning/__init__.py: contains deprecation warning text")
    for module_name in DEPRECATED_IMPORT_ALIAS_MODULES:
        short_name = module_name.rsplit(".", 1)[1]
        if (PROJECT_ROOT / "modules" / "mission_planning" / f"{short_name}.py").exists():
            offenders.append(f"{module_name}: alias still has root wrapper file")
        try:
            importlib.import_module(module_name)
        except Exception as exc:
            offenders.append(f"{module_name}: import failed: {exc}")
    for rel_path in DEPRECATED_IMPORT_SURFACE_FILES:
        tree = parse_source(rel_path)
        source = read_source(rel_path)
        if "DeprecationWarning" in source or "deprecated import" in source.lower():
            offenders.append(f"{rel_path}: contains deprecation warning text")
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom):
                module_name = import_module_name(node)
                if module_name in FORBIDDEN_IMPORT_SIDE_EFFECT_MODULES:
                    offenders.append(f"{rel_path}: imports {module_name}")
            elif isinstance(node, ast.Import):
                for alias in node.names:
                    if alias.name in FORBIDDEN_IMPORT_SIDE_EFFECT_MODULES:
                        offenders.append(f"{rel_path}: imports {alias.name}")
        calls = top_level_call_names(tree).intersection(FORBIDDEN_TOP_LEVEL_CALLS)
        if calls:
            offenders.append(f"{rel_path}: top-level side-effect calls {sorted(calls)!r}")
    if offenders:
        fail("deprecated import logging side effects found:\n" + "\n".join(offenders))


def check_policy_references_existing_contracts() -> None:
    references = (
        (
            "docs/mission planning refactoring/41-bootstrap-import-order-contract-progress.md",
            "console/file logging",
        ),
        (
            "docs/mission planning refactoring/73-compat-root-strategy-decision.md",
            "root compatibility paths",
        ),
        (
            "docs/mission planning refactoring/10-wrapper-support-matrix.md",
            "Supported compatibility paths",
        ),
    )
    for rel_path, marker in references:
        if marker not in read_source(rel_path):
            fail(f"{rel_path} missing policy reference marker {marker!r}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke deprecated import logging/documentation policy.")
    parser.parse_args()

    try:
        check_decision_doc()
        check_wrapper_imports_have_no_deprecation_side_effects()
        check_policy_references_existing_contracts()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("deprecated import policy smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
