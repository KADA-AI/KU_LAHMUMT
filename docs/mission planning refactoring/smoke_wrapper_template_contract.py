from __future__ import annotations

import argparse
import ast
import sys
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


BROAD_PROJECT_BOOTSTRAP_WRAPPERS = {
    "modules/mission_planning/lah_rl_planner_gui.py": (
        "modules.mission_planning.manual.lah_rl_planner_gui"
    ),
    "modules/mission_planning/legacy/wrappers/attack_assignment_state.py": (
        "modules.mission_planning.runtime.state.attack_assignment"
    ),
    "modules/mission_planning/legacy/wrappers/attack_plan_pipeline.py": (
        "modules.mission_planning.replanning.triggers.attack.pipeline"
    ),
    "modules/mission_planning/legacy/wrappers/id_relationship_tab.py": (
        "modules.mission_planning.legacy.ui.id_relationship_tab"
    ),
    "modules/mission_planning/legacy/wrappers/json_io.py": "modules.mission_planning.runtime.json_io",
    "modules/mission_planning/legacy/wrappers/latest_input_cache.py": (
        "modules.mission_planning.runtime.cache.latest_input"
    ),
    "modules/mission_planning/legacy/wrappers/mission_path_trim.py": (
        "modules.mission_planning.pipelines.mission_path_trim"
    ),
    "modules/mission_planning/legacy/wrappers/mission_plan_file_logger.py": (
        "modules.mission_planning.runtime.logging.plan_file_logger"
    ),
    "modules/mission_planning/legacy/wrappers/mission_planning_attack_helpers.py": (
        "modules.mission_planning.pipelines.mission_planning_attack_helpers"
    ),
    "modules/mission_planning/legacy/wrappers/mission_planning_gui_env.py": (
        "modules.mission_planning.ui.mission_planning_gui_env"
    ),
    "modules/mission_planning/legacy/wrappers/mission_planning_log_tab.py": (
        "modules.mission_planning.legacy.ui.mission_planning_log_tab"
    ),
    "modules/mission_planning/legacy/wrappers/mission_planning_pipeline_logging.py": (
        "modules.mission_planning.runtime.logging.pipeline_events"
    ),
    "modules/mission_planning/legacy/wrappers/prior_mission_pipeline_impl.py": (
        "modules.mission_planning.replanning.triggers.prior.pipeline"
    ),
}


BROAD_PACKAGE_WRAPPERS = {
    "modules/mission_planning/pipelines/attack_plan_pipeline.py": (
        "modules.mission_planning.replanning.triggers.attack.pipeline"
    ),
    "modules/mission_planning/pipelines/prior_mission_pipeline_impl.py": (
        "modules.mission_planning.replanning.triggers.prior.pipeline"
    ),
    "modules/mission_planning/pipelines/next_collab_replan_pipeline_impl.py": (
        "modules.mission_planning.replanning.triggers.next_collab.pipeline"
    ),
    "modules/mission_planning/pipelines/current_remaining_hybrid.py": (
        "modules.mission_planning.replanning.triggers.remaining_hybrid.current"
    ),
    "modules/mission_planning/pipelines/current_remaining_hybrid_replan.py": (
        "modules.mission_planning.replanning.triggers.remaining_hybrid.current_replan"
    ),
    "modules/mission_planning/pipelines/general_remaining_hybrid_replan.py": (
        "modules.mission_planning.replanning.triggers.remaining_hybrid.general"
    ),
    "modules/mission_planning/pipelines/reexecute_first_mission_hybrid.py": (
        "modules.mission_planning.replanning.triggers.remaining_hybrid.reexecute_first"
    ),
    "modules/mission_planning/pipelines/recon_specialized_pipeline.py": (
        "modules.mission_planning.replanning.triggers.recon_specialized.pipeline"
    ),
    "modules/mission_planning/pipelines/post_attack_rejoin_pipeline.py": (
        "modules.mission_planning.replanning.triggers.post_attack.pipeline"
    ),
}


RUNTIME_SAFE_BROAD_WRAPPERS = {
    "modules/mission_planning/runtime/attack_assignment_state.py": (
        "modules.mission_planning.runtime.state.attack_assignment"
    ),
    "modules/mission_planning/runtime/attack_tracking_state.py": (
        "modules.mission_planning.runtime.state.attack_tracking"
    ),
    "modules/mission_planning/runtime/prior_tracking_state.py": (
        "modules.mission_planning.runtime.state.prior_tracking"
    ),
    "modules/mission_planning/runtime/source_artifact_cache.py": (
        "modules.mission_planning.runtime.cache.source_artifacts"
    ),
    "modules/mission_planning/runtime/latest_input_cache.py": (
        "modules.mission_planning.runtime.cache.latest_input"
    ),
    "modules/mission_planning/runtime/mission_planning_pipeline_logging.py": (
        "modules.mission_planning.runtime.logging.pipeline_events"
    ),
    "modules/mission_planning/runtime/mission_plan_file_logger.py": (
        "modules.mission_planning.runtime.logging.plan_file_logger"
    ),
    "modules/mission_planning/runtime/replan_validation.py": (
        "modules.mission_planning.runtime.validation.replan_payloads"
    ),
    "modules/mission_planning/runtime/replan_id_reservation.py": (
        "modules.mission_planning.runtime.ids.replan_reservation"
    ),
}


EXPLICIT_BOOTSTRAP_WRAPPERS = {
    "modules/mission_planning/legacy/wrappers/prior_mission_pipeline.py": (
        "modules.mission_planning.replanning.triggers.prior.pipeline",
        ("run_prior_mission_pipeline", "warm_prior_mission_pipeline"),
    ),
    "modules/mission_planning/legacy/wrappers/next_collab_replan_pipeline.py": (
        "modules.mission_planning.replanning.triggers.next_collab.pipeline",
        ("run_next_collab_replan_pipeline", "warm_next_collab_replan_pipeline"),
    ),
    "modules/mission_planning/legacy/wrappers/imaging_schedule_replan_pipeline.py": (
        "modules.mission_planning.replanning.triggers.imaging_schedule.pipeline",
        ("run_imaging_schedule_replan_pipeline", "warm_imaging_schedule_replan_pipeline"),
    ),
    "modules/mission_planning/legacy/wrappers/path_deviation_replan_pipeline.py": (
        "modules.mission_planning.replanning.triggers.path_deviation.pipeline",
        ("run_path_deviation_replan_pipeline", "warm_path_deviation_replan_pipeline"),
    ),
}


EXPLICIT_PACKAGE_WRAPPERS = {
    "modules/mission_planning/pipelines/next_collab_replan_pipeline.py": (
        "modules.mission_planning.replanning.triggers.next_collab.pipeline",
        (
            "NextCollabPipelineResult",
            "run_next_collab_replan_pipeline",
            "warm_next_collab_replan_pipeline",
        ),
    ),
    "modules/mission_planning/pipelines/imaging_schedule_replan_pipeline_impl.py": (
        "modules.mission_planning.replanning.triggers.imaging_schedule.pipeline",
        (
            "IMAGING_TRIGGER_TYPE",
            "QUALITY_TRIGGER_TYPE",
            "ImagingSchedulePipelineResult",
            "run_imaging_schedule_replan_pipeline",
            "warm_imaging_schedule_replan_pipeline",
        ),
    ),
    "modules/mission_planning/pipelines/path_deviation_replan_pipeline_impl.py": (
        "modules.mission_planning.replanning.triggers.path_deviation.pipeline",
        (
            "PathDeviationPipelineResult",
            "run_path_deviation_replan_pipeline",
            "warm_path_deviation_replan_pipeline",
        ),
    ),
}


SYS_MODULE_ALIAS_WRAPPERS = {
    **{
        f"modules/mission_planning/MissionPlanner/data_def/{name}.py": (
            "modules.mission_planning.engine.mission_generation."
            f"artifacts_0301_0302_0303_0304.{name}"
        )
        for name in ("d0301", "d0302", "d0303", "d0304")
    },
}


SPECIAL_PROXY_WRAPPERS = {
    "modules/mission_planning/MissionPlanner/data_def/id_allocator.py": (
        "modules.mission_planning.engine.mission_generation.id_allocation.allocator",
        "_AllocatorProxy",
    ),
}


PACKAGE_SHIM_WRAPPERS = {}


PACKAGE_ROOT_ALIAS_MODULES = {
    "attack_assignment_state": (
        "modules.mission_planning.runtime.state.attack_assignment",
        None,
    ),
    "attack_plan_pipeline": (
        "modules.mission_planning.replanning.triggers.attack.pipeline",
        None,
    ),
    "id_relationship_tab": ("modules.mission_planning.ui.id_relationship_tab", None),
    "imaging_schedule_replan_pipeline": (
        "modules.mission_planning.replanning.triggers.imaging_schedule.pipeline",
        ("run_imaging_schedule_replan_pipeline", "warm_imaging_schedule_replan_pipeline"),
    ),
    "json_io": ("modules.mission_planning.runtime.json_io", None),
    "latest_input_cache": ("modules.mission_planning.runtime.cache.latest_input", None),
    "mission_path_trim": ("modules.mission_planning.pipelines.mission_path_trim", None),
    "mission_plan_file_logger": (
        "modules.mission_planning.runtime.logging.plan_file_logger",
        None,
    ),
    "mission_planning_attack_helpers": (
        "modules.mission_planning.pipelines.mission_planning_attack_helpers",
        None,
    ),
    "mission_planning_gui_env": (
        "modules.mission_planning.ui.mission_planning_gui_env",
        None,
    ),
    "mission_planning_log_tab": (
        "modules.mission_planning.ui.mission_planning_log_tab",
        None,
    ),
    "mission_planning_pipeline_logging": (
        "modules.mission_planning.runtime.logging.pipeline_events",
        None,
    ),
    "next_collab_replan_pipeline": (
        "modules.mission_planning.replanning.triggers.next_collab.pipeline",
        ("run_next_collab_replan_pipeline", "warm_next_collab_replan_pipeline"),
    ),
    "path_deviation_replan_pipeline": (
        "modules.mission_planning.replanning.triggers.path_deviation.pipeline",
        ("run_path_deviation_replan_pipeline", "warm_path_deviation_replan_pipeline"),
    ),
    "prior_mission_pipeline": (
        "modules.mission_planning.replanning.triggers.prior.pipeline",
        ("run_prior_mission_pipeline", "warm_prior_mission_pipeline"),
    ),
    "prior_mission_pipeline_impl": (
        "modules.mission_planning.replanning.triggers.prior.pipeline",
        None,
    ),
}


class SmokeFailure(RuntimeError):
    pass


def fail(message: str) -> None:
    raise SmokeFailure(message)


def read_source(rel_path: str) -> str:
    path = PROJECT_ROOT / rel_path
    if not path.exists():
        fail(f"{rel_path} is missing")
    return path.read_text(encoding="utf-8", errors="ignore")


def parse_source(rel_path: str) -> ast.Module:
    text = read_source(rel_path)
    try:
        return ast.parse(text)
    except SyntaxError as exc:
        fail(f"{rel_path} is not parseable: {exc}")


def assert_contains(rel_path: str, *snippets: str) -> None:
    text = read_source(rel_path)
    missing = [snippet for snippet in snippets if snippet not in text]
    if missing:
        fail(f"{rel_path} missing wrapper template markers: {missing!r}")


def top_level_definitions(tree: ast.Module) -> list[str]:
    return [
        node.name
        for node in tree.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef))
    ]


def assert_no_top_level_definitions(rel_path: str) -> None:
    definitions = top_level_definitions(parse_source(rel_path))
    if definitions:
        fail(f"{rel_path} should be a wrapper only, but defines: {', '.join(definitions)}")


def literal_all(rel_path: str) -> list[str]:
    tree = parse_source(rel_path)
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        if any(isinstance(target, ast.Name) and target.id == "__all__" for target in node.targets):
            try:
                value = ast.literal_eval(node.value)
            except Exception as exc:
                fail(f"{rel_path} __all__ must be a literal explicit export list: {exc}")
            if not isinstance(value, list) or not all(isinstance(item, str) for item in value):
                fail(f"{rel_path} __all__ must be a list[str]")
            return list(value)
    fail(f"{rel_path} missing __all__")


def assert_project_bootstrap(rel_path: str) -> None:
    assert_contains(
        rel_path,
        "import sys",
        "from pathlib import Path",
        "Path(__file__).resolve()",
        "_PROJECT_ROOT_STR = str(_PROJECT_ROOT)",
        "sys.path.insert(0, _PROJECT_ROOT_STR)",
    )


def assert_broad_reexport(rel_path: str, canonical_name: str, *, project_bootstrap: bool = False) -> None:
    assert_no_top_level_definitions(rel_path)
    if project_bootstrap:
        assert_project_bootstrap(rel_path)
    assert_contains(
        rel_path,
        "from importlib import import_module",
        f'_MODULE = import_module("{canonical_name}")',
        "for _name, _value in vars(_MODULE).items():",
        'if _name.startswith("__") and _name.endswith("__"):',
        "globals()[_name] = _value",
        'if hasattr(_MODULE, "__all__"):',
        "__all__ = list(_MODULE.__all__)",
    )


def assert_runtime_safe_broad_reexport(rel_path: str, canonical_name: str) -> None:
    assert_no_top_level_definitions(rel_path)
    assert_contains(
        rel_path,
        "import_runtime_compat_module",
        "from ._compat_import import import_runtime_compat_module",
        "from _compat_import import import_runtime_compat_module",
        f'_MODULE = import_runtime_compat_module("{canonical_name}", __file__)',
        "for _name, _value in vars(_MODULE).items():",
        "globals()[_name] = _value",
        'if hasattr(_MODULE, "__all__"):',
        "__all__ = list(_MODULE.__all__)",
    )


def assert_explicit_wrapper(
    rel_path: str,
    canonical_name: str,
    export_names: tuple[str, ...],
    *,
    project_bootstrap: bool = False,
) -> None:
    assert_no_top_level_definitions(rel_path)
    if project_bootstrap:
        assert_project_bootstrap(rel_path)
    assert_contains(rel_path, f"from {canonical_name} import (")
    for name in export_names:
        assert_contains(rel_path, name)
    expect_all = list(export_names)
    actual_all = literal_all(rel_path)
    if actual_all != expect_all:
        fail(f"{rel_path} __all__ changed: expected {expect_all!r}, got {actual_all!r}")


def assert_sys_module_alias(rel_path: str, canonical_name: str) -> None:
    assert_no_top_level_definitions(rel_path)
    assert_contains(
        rel_path,
        "from importlib import import_module",
        "import sys",
        "sys.modules[__name__] = import_module(",
        canonical_name,
    )


def assert_special_proxy(rel_path: str, canonical_name: str, proxy_class_name: str) -> None:
    definitions = top_level_definitions(parse_source(rel_path))
    if definitions != [proxy_class_name]:
        fail(f"{rel_path} proxy definitions changed: expected {[proxy_class_name]!r}, got {definitions!r}")
    assert_contains(
        rel_path,
        "from types import ModuleType",
        f'_MODULE = import_module("{canonical_name}")',
        f"class {proxy_class_name}(ModuleType):",
        "def __getattribute__(self, name: str):",
        "def __getattr__(self, name: str):",
        "def __setattr__(self, name: str, value):",
        "for _name, _value in vars(_MODULE).items():",
        "globals()[_name] = _value",
        'if hasattr(_MODULE, "__all__"):',
        "sys.modules[__name__].__class__ =",
    )


def assert_package_shim(rel_path: str, canonical_name: str) -> None:
    assert_no_top_level_definitions(rel_path)
    assert_contains(
        rel_path,
        "from importlib import import_module",
        canonical_name,
        "__path__",
        "for _name, _value in vars(_MODULE).items():",
        "globals()[_name] = _value",
        'if hasattr(_MODULE, "__all__"):',
    )


def assert_package_root_aliases() -> None:
    rel_path = "modules/mission_planning/__init__.py"
    source = read_source(rel_path)
    assert_contains(
        rel_path,
        "_COMPAT_ALIASES",
        "_CompatAliasFinder",
        "_CompatAliasLoader",
        "_install_compat_alias_finder()",
    )
    for alias_name, (canonical_name, exports) in PACKAGE_ROOT_ALIAS_MODULES.items():
        if f'"{alias_name}"' not in source:
            fail(f"{rel_path} missing package root alias {alias_name!r}")
        if canonical_name not in source:
            fail(f"{rel_path} missing package root alias target {canonical_name!r}")
        root_file = PROJECT_ROOT / "modules" / "mission_planning" / f"{alias_name}.py"
        if root_file.exists():
            fail(f"{root_file.relative_to(PROJECT_ROOT)} should be represented by package alias, not a root wrapper file")
        module = __import__(f"modules.mission_planning.{alias_name}", fromlist=["*"])
        if exports is not None:
            actual_all = list(getattr(module, "__all__", []))
            if actual_all != list(exports):
                fail(f"{alias_name} alias __all__ changed: expected {list(exports)!r}, got {actual_all!r}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke mission-planning wrapper template contracts.")
    parser.parse_args()

    try:
        for rel_path, canonical_name in BROAD_PROJECT_BOOTSTRAP_WRAPPERS.items():
            assert_broad_reexport(rel_path, canonical_name, project_bootstrap=True)
        for rel_path, canonical_name in BROAD_PACKAGE_WRAPPERS.items():
            assert_broad_reexport(rel_path, canonical_name)
        for rel_path, canonical_name in RUNTIME_SAFE_BROAD_WRAPPERS.items():
            assert_runtime_safe_broad_reexport(rel_path, canonical_name)
        for rel_path, (canonical_name, export_names) in EXPLICIT_BOOTSTRAP_WRAPPERS.items():
            assert_explicit_wrapper(rel_path, canonical_name, export_names, project_bootstrap=True)
        for rel_path, (canonical_name, export_names) in EXPLICIT_PACKAGE_WRAPPERS.items():
            assert_explicit_wrapper(rel_path, canonical_name, export_names)
        for rel_path, canonical_name in SYS_MODULE_ALIAS_WRAPPERS.items():
            assert_sys_module_alias(rel_path, canonical_name)
        for rel_path, (canonical_name, proxy_class_name) in SPECIAL_PROXY_WRAPPERS.items():
            assert_special_proxy(rel_path, canonical_name, proxy_class_name)
        for rel_path, canonical_name in PACKAGE_SHIM_WRAPPERS.items():
            assert_package_shim(rel_path, canonical_name)
        assert_package_root_aliases()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("wrapper template contract smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
