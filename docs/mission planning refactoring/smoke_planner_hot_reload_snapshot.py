from __future__ import annotations

import ast
import importlib
import sys
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


EXPECTED_IMPORT_PATHS = (
    ".",
    "modules",
    "modules/mission_planning",
    "modules/mission_planning/MissionPlanner",
)

EXPECTED_WATCH_PATHS = (
    "modules/mission_planning/MissionPlanner/AnS/__init__.py",
    "modules/mission_planning/MissionPlanner/AnS/coord_transform.py",
    "modules/mission_planning/MissionPlanner/AnS/task_patterns_ver2.py",
    "modules/mission_planning/MissionPlanner/AnS/mission_effectiveness_ver2.py",
    "modules/mission_planning/MissionPlanner/AnS/env_patternselection.py",
    "modules/mission_planning/MissionPlanner/AnS/mission_pipeline.py",
    "modules/mission_planning/MissionPlanner/data_def/d0302.py",
    "modules/mission_planning/MissionPlanner/data_def/d0303.py",
    "modules/mission_planning/MissionPlanner/data_def/d0304.py",
    "modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0301.py",
    "modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0302.py",
    "modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0303.py",
    "modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0304.py",
    "modules/mission_planning/replanning/triggers/remaining_hybrid/current.py",
    "modules/mission_planning/replanning/triggers/remaining_hybrid/current_replan.py",
    "modules/mission_planning/replanning/triggers/remaining_hybrid/general.py",
    "modules/mission_planning/replanning/triggers/remaining_hybrid/reexecute_first.py",
    "modules/mission_planning/replanning/triggers/recon_specialized/pipeline.py",
    "modules/mission_planning/pipelines/next_collab_path_builder.py",
    "modules/mission_planning/replanning/triggers/next_collab/pipeline.py",
    "modules/mission_planning/runtime/next_collab_line_runner.py",
    "modules/mission_planning/runtime/aircraft_parallel_0303.py",
    "modules/mission_planning/replanning/triggers/prior/pipeline.py",
    "modules/mission_planning/replanning/triggers/imaging_schedule/pipeline.py",
    "modules/mission_planning/replanning/triggers/path_deviation/pipeline.py",
    "modules/mission_planning/replanning/triggers/attack/pipeline.py",
    "modules/mission_planning/replanning/triggers/post_attack/pipeline.py",
)

EXPECTED_RELOAD_ORDER = (
    "modules.mission_planning.MissionPlanner.AnS.coord_transform",
    "modules.mission_planning.MissionPlanner.AnS.task_patterns_ver2",
    "modules.mission_planning.MissionPlanner.AnS.mission_effectiveness_ver2",
    "modules.mission_planning.MissionPlanner.AnS.env_patternselection",
    "modules.mission_planning.MissionPlanner.AnS.mission_pipeline",
    "modules.mission_planning.MissionPlanner.AnS",
    "modules.mission_planning.pipelines.next_collab_path_builder",
    "modules.mission_planning.runtime.next_collab_line_runner",
    "modules.mission_planning.replanning.triggers.next_collab.pipeline",
    "modules.mission_planning.replanning.triggers.remaining_hybrid.reexecute_first",
    "modules.mission_planning.replanning.triggers.recon_specialized.pipeline",
    "modules.mission_planning.replanning.triggers.remaining_hybrid.current_replan",
    "modules.mission_planning.replanning.triggers.remaining_hybrid.current",
    "modules.mission_planning.replanning.triggers.remaining_hybrid.general",
    "modules.mission_planning.runtime.aircraft_parallel_0303",
    "modules.mission_planning.replanning.triggers.prior.pipeline",
    "modules.mission_planning.replanning.triggers.imaging_schedule.pipeline",
    "modules.mission_planning.replanning.triggers.path_deviation.pipeline",
    "modules.mission_planning.replanning.triggers.attack.pipeline",
    "modules.mission_planning.replanning.triggers.post_attack.pipeline",
)

EXPECTED_BINDINGS = {
    "modules.mission_planning.MissionPlanner.AnS": (
        "run_divide_and_pattern",
        "run_pulp_scheduling",
        "build_mission_plan_0301",
        "get_last_divide_and_pattern_metrics",
    ),
    "modules.mission_planning.replanning.triggers.prior.pipeline": (
        "run_prior_mission_pipeline",
        "warm_prior_mission_pipeline",
        "run_prior_post_rejoin_pipeline",
        "warm_prior_post_rejoin_pipeline",
    ),
    "modules.mission_planning.replanning.triggers.imaging_schedule.pipeline": (
        "run_imaging_schedule_replan_pipeline",
        "warm_imaging_schedule_replan_pipeline",
    ),
    "modules.mission_planning.replanning.triggers.path_deviation.pipeline": (
        "run_path_deviation_replan_pipeline",
        "warm_path_deviation_replan_pipeline",
    ),
    "modules.mission_planning.replanning.triggers.next_collab.pipeline": (
        "run_next_collab_replan_pipeline",
        "warm_next_collab_replan_pipeline",
    ),
    "modules.mission_planning.replanning.triggers.attack.pipeline": (
        "run_attack_exclusion_pipeline",
        "run_attack_plan_pipeline",
        "warm_attack_plan_pipeline",
    ),
    "modules.mission_planning.replanning.triggers.post_attack.pipeline": (
        "run_post_attack_rejoin_pipeline",
        "warm_post_attack_rejoin_pipeline",
    ),
}

EXPECTED_NOT_IN_PLANNER_RUNTIME_RELOAD_ORDER_PATHS = (
    "modules/mission_planning/MissionPlanner/data_def/d0302.py",
    "modules/mission_planning/MissionPlanner/data_def/d0303.py",
    "modules/mission_planning/MissionPlanner/data_def/d0304.py",
    "modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0301.py",
    "modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0302.py",
    "modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0303.py",
    "modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0304.py",
)


class SmokeFailure(RuntimeError):
    pass


def fail(message: str) -> None:
    raise SmokeFailure(message)


def rel_tuple(paths: tuple[Path, ...]) -> tuple[str, ...]:
    return tuple(str(path).replace("\\", "/") for path in paths)


def check_runtime_snapshot() -> None:
    runtime = importlib.import_module("modules.mission_planning.mission_control.planner_runtime")
    import_paths = rel_tuple(runtime.MISSION_PLANNER_IMPORT_RELATIVE_PATHS)
    watch_paths = rel_tuple(runtime.PLANNER_RUNTIME_WATCH_RELATIVE_PATHS)
    reload_order = tuple(runtime.PLANNER_RUNTIME_RELOAD_ORDER)
    bindings = {key: tuple(value) for key, value in runtime.PIPELINE_RELOAD_BINDINGS.items()}

    if import_paths != EXPECTED_IMPORT_PATHS:
        fail(f"mission planner import paths changed:\n{import_paths!r}")
    if watch_paths != EXPECTED_WATCH_PATHS:
        fail(f"planner hot-reload watch list changed:\n{watch_paths!r}")
    if reload_order != EXPECTED_RELOAD_ORDER:
        fail(f"planner hot-reload reload order changed:\n{reload_order!r}")
    if bindings != EXPECTED_BINDINGS:
        fail(f"planner hot-reload binding map changed:\n{bindings!r}")
    for rel_path in EXPECTED_NOT_IN_PLANNER_RUNTIME_RELOAD_ORDER_PATHS:
        if rel_path not in watch_paths:
            fail(f"expected watch path missing from non-runtime-reload set: {rel_path}")

    missing = [rel for rel in EXPECTED_WATCH_PATHS if not (PROJECT_ROOT / rel).exists()]
    if missing:
        fail("planner hot-reload watched file(s) missing: " + ", ".join(missing))

    signature = runtime.planner_runtime_source_signature(PROJECT_ROOT)
    signature_keys = tuple(key for key, _sig in signature)
    if signature_keys != EXPECTED_WATCH_PATHS:
        fail(f"planner source signature order changed:\n{signature_keys!r}")
    none_keys = [key for key, sig in signature if sig is None]
    if none_keys:
        fail("planner source signature missing file(s): " + ", ".join(none_keys))
    for key, sig in signature:
        if (
            not isinstance(sig, tuple)
            or len(sig) != 2
            or not all(isinstance(value, int) for value in sig)
        ):
            fail(f"planner source signature shape changed for {key}: {sig!r}")


def check_gui_hot_reload_bridge() -> None:
    gui_path = PROJECT_ROOT / "modules" / "mission_planning" / "mission_planning_gui.py"
    source = gui_path.read_text(encoding="utf-8-sig", errors="ignore")
    tree = ast.parse(source)
    imports_runtime_helpers = False
    assigns_watch_alias = False
    source_signature_uses_project_root = False
    refresh_uses_globals = False
    build_runtime_force_reload_shape = False
    direct_reload_snippets = (
        "mp_ans = importlib.reload(mp_ans)",
        "d0302 = importlib.reload(d0302)",
        "d0303 = importlib.reload(d0303)",
        "d0304 = importlib.reload(d0304)",
        "mp_config = importlib.reload(mp_config)",
        "mp_search_speed = importlib.reload(mp_search_speed)",
    )

    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom) and node.module == "modules.mission_planning.mission_control.planner_runtime":
            imported_names = {alias.name for alias in node.names}
            imports_runtime_helpers = {
                "PLANNER_RUNTIME_WATCH_RELATIVE_PATHS",
                "planner_runtime_source_signature",
                "refresh_live_planning_helpers",
                "reload_planning_module",
            }.issubset(imported_names)
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if (
                    isinstance(target, ast.Name)
                    and target.id == "_PLANNER_RUNTIME_WATCH_RELATIVE_PATHS"
                    and isinstance(node.value, ast.Name)
                    and node.value.id == "PLANNER_RUNTIME_WATCH_RELATIVE_PATHS"
                ):
                    assigns_watch_alias = True

    for node in tree.body:
        if not isinstance(node, ast.FunctionDef):
            continue
        if node.name == "_planner_runtime_source_signature":
            for child in ast.walk(node):
                if (
                    isinstance(child, ast.Call)
                    and isinstance(child.func, ast.Name)
                    and child.func.id == "_planner_runtime_source_signature_impl"
                    and child.args
                    and isinstance(child.args[0], ast.Name)
                    and child.args[0].id == "PROJECT_ROOT"
                ):
                    source_signature_uses_project_root = True
        if node.name == "_refresh_live_planning_helpers":
            for child in ast.walk(node):
                if (
                    isinstance(child, ast.Call)
                    and isinstance(child.func, ast.Name)
                    and child.func.id == "_refresh_live_planning_helpers_impl"
                    and child.args
                    and isinstance(child.args[0], ast.Call)
                    and isinstance(child.args[0].func, ast.Name)
                    and child.args[0].func.id == "globals"
                ):
                    refresh_uses_globals = True

    for node in ast.walk(tree):
        if isinstance(node, ast.FunctionDef) and node.name == "_build_planner_runtime":
            text = ast.get_source_segment(source, node) or ""
            build_runtime_force_reload_shape = (
                "force_reload = previous_signature is not None and previous_signature != source_signature" in text
                and "_ensure_mission_planner_import_paths()" in text
                and "_refresh_live_planning_helpers()" in text
                and "import AnS as mp_ans" in text
                and "from data_def import d0302, d0303, d0304" in text
                and all(snippet in text for snippet in direct_reload_snippets)
            )

    checks = {
        "runtime helper import": imports_runtime_helpers,
        "watch alias": assigns_watch_alias,
        "source signature project root": source_signature_uses_project_root,
        "refresh globals bridge": refresh_uses_globals,
        "build runtime force-reload shape": build_runtime_force_reload_shape,
    }
    failed = [name for name, ok in checks.items() if not ok]
    if failed:
        fail("mission_planning_gui hot-reload bridge changed: " + ", ".join(failed))


def main() -> int:
    try:
        check_runtime_snapshot()
        check_gui_hot_reload_bridge()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("planner hot-reload watch/reload snapshot smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
