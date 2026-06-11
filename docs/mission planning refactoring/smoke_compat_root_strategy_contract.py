from __future__ import annotations

import argparse
import importlib
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
DECISION_DOC = PROJECT_ROOT / "docs" / "mission planning refactoring" / "73-compat-root-strategy-decision.md"


ROOT_COMPATIBILITY_ALIAS_MODULES = (
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


ROOT_COMPATIBILITY_FILES = (
    "modules/mission_planning/lah_rl_planner_gui.py",
)


INTERNAL_BARE_IMPORT_FILES = (
    "modules/mission_planning/MissionPlanner/AnS/__init__.py",
    "modules/mission_planning/MissionPlanner/data_def/__init__.py",
    "modules/mission_planning/MissionPlanner/data_def/d0301.py",
    "modules/mission_planning/MissionPlanner/data_def/d0302.py",
    "modules/mission_planning/MissionPlanner/data_def/d0303.py",
    "modules/mission_planning/MissionPlanner/data_def/d0304.py",
    "modules/mission_planning/MissionPlanner/data_def/id_allocator.py",
    "modules/mission_planning/MissionPlanner/data_def/mission_helpers.py",
    "modules/mission_planning/MissionPlanner/data_def/search_speed.py",
    "modules/mission_planning/MissionPlanner/config.py",
)


FORBIDDEN_PROJECT_ROOT_BARE_SHIMS = (
    "AnS",
    "data_def",
    "config.py",
)


ACTIVE_CANONICAL_UI_DEPENDENCIES = {
    "modules/mission_planning/manual/MissionVisualizer/main_visualizer.py": (
        "from modules.mission_planning.ui.id_relationship_tab import RelationshipCache",
    ),
    "modules/mission_planning/__init__.py": (
        '"MissionVisualizer": "modules.mission_planning.manual.MissionVisualizer"',
    ),
    "modules/mission_planning/MissionPlanner/tools/main_visualizer.py": (
        'import_module("modules.mission_planning.manual.MissionVisualizer.main_visualizer")',
    ),
}


FORBIDDEN_ACTIVE_ROOT_WRAPPER_IMPORTS = {
    "modules/mission_planning/manual/MissionVisualizer/main_visualizer.py": (
        "from modules.mission_planning.id_relationship_tab import RelationshipCache",
    ),
    "modules/mission_planning/MissionPlanner/tools/main_visualizer.py": (
        "from modules.mission_planning.id_relationship_tab import RelationshipCache",
    ),
}


ACTIVE_BARE_SHIM_DEPENDENCIES = {
    "modules/mission_planning/mission_planning_gui.py": (
        "import AnS as mp_ans",
        "from data_def import d0302, d0303, d0304",
        "import config as mp_config",
        "from data_def.id_allocator import mark_waypoint_files_written",
    ),
    "modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0303.py": (
        "from config import",
    ),
    "modules/mission_planning/MissionPlanner/AnS/mission_pipeline.py": (
        "from data_def.id_allocator import",
    ),
    "modules/mission_planning/MissionPlanner/corridor_planner.py": (
        "from data_def.mission_helpers import now_ms_since_2000",
        "import config as mp_config",
    ),
}

ACTIVE_EXTERNAL_SURFACE_DEPENDENCIES = {
    "run.py": (
        '"mission": "mission_planning_gui.py"',
        '"mission_planning_gui.py": "mission"',
        "from modules.mission_planning.MissionPlanner.data_def import id_allocator",
    ),
    "modules/common/button_wiring.py": (
        '"assignment": "mission_planning_gui.py"',
    ),
    "modules/monitoring/gui/tabs/monitoring_visualization_tab.py": (
        "from modules.mission_planning.pipelines.mission_path_trim import DEFAULT_SWEEP_SPLIT_LOOKAHEAD_SECONDS",
    ),
}


def fail(message: str) -> None:
    raise AssertionError(message)


def read_text(rel_path: str) -> str:
    path = PROJECT_ROOT / rel_path
    if not path.exists():
        fail(f"{rel_path} is missing")
    return path.read_text(encoding="utf-8", errors="ignore")


def assert_contains(rel_path: str, *snippets: str) -> None:
    text = read_text(rel_path)
    missing = [snippet for snippet in snippets if snippet not in text]
    if missing:
        fail(f"{rel_path} missing root compatibility markers: {missing!r}")


def check_decision_doc() -> None:
    text = DECISION_DOC.read_text(encoding="utf-8", errors="ignore")
    required = (
        "Decision",
        "Keep root compatibility paths for the current refactor",
        "do not move public compatibility imports under `compat/`",
        "manual/lah_rl_planner_gui.py",
        "internal bare-import bootstrap behavior",
    )
    missing = [snippet for snippet in required if snippet not in text]
    if missing:
        fail(f"{DECISION_DOC.relative_to(PROJECT_ROOT)} missing decision markers: {missing!r}")


def check_root_files_remain() -> None:
    missing = [rel_path for rel_path in ROOT_COMPATIBILITY_FILES if not (PROJECT_ROOT / rel_path).exists()]
    if missing:
        fail("root compatibility files moved or deleted: " + ", ".join(missing))
    alias_source = read_text("modules/mission_planning/__init__.py")
    for module_name in ROOT_COMPATIBILITY_ALIAS_MODULES:
        short_name = module_name.rsplit(".", 1)[1]
        if f'"{short_name}"' not in alias_source:
            fail(f"root compatibility alias missing from __init__.py: {module_name}")
        if (PROJECT_ROOT / "modules" / "mission_planning" / f"{short_name}.py").exists():
            fail(f"root compatibility alias still has loose wrapper file: {short_name}.py")
        importlib.import_module(module_name)
    missing_bare = [rel_path for rel_path in INTERNAL_BARE_IMPORT_FILES if not (PROJECT_ROOT / rel_path).exists()]
    if missing_bare:
        fail("internal bare import files moved or deleted: " + ", ".join(missing_bare))
    forbidden = [rel_path for rel_path in FORBIDDEN_PROJECT_ROOT_BARE_SHIMS if (PROJECT_ROOT / rel_path).exists()]
    if forbidden:
        fail("project-root bare import shims must stay absent: " + ", ".join(forbidden))


def check_active_dependencies() -> None:
    for rel_path, snippets in ACTIVE_CANONICAL_UI_DEPENDENCIES.items():
        assert_contains(rel_path, *snippets)
    for rel_path, snippets in FORBIDDEN_ACTIVE_ROOT_WRAPPER_IMPORTS.items():
        text = read_text(rel_path)
        offenders = [snippet for snippet in snippets if snippet in text]
        if offenders:
            fail(f"{rel_path} still imports root compatibility wrappers: {offenders!r}")
    for rel_path, snippets in ACTIVE_BARE_SHIM_DEPENDENCIES.items():
        assert_contains(rel_path, *snippets)
    for rel_path, snippets in ACTIVE_EXTERNAL_SURFACE_DEPENDENCIES.items():
        assert_contains(rel_path, *snippets)


def check_no_public_compat_import_surface() -> None:
    compat_dir = PROJECT_ROOT / "modules" / "mission_planning" / "compat"
    if compat_dir.exists():
        fail("modules/mission_planning/compat exists; current decision keeps public wrappers at root")

    offenders: list[str] = []
    for base in (
        PROJECT_ROOT / "app",
        PROJECT_ROOT / "modules" / "common",
        PROJECT_ROOT / "modules" / "monitoring",
        PROJECT_ROOT / "modules" / "mission_planning",
    ):
        if not base.exists():
            continue
        for path in base.rglob("*.py"):
            text = path.read_text(encoding="utf-8", errors="ignore")
            if "modules.mission_planning.compat" in text:
                offenders.append(str(path.relative_to(PROJECT_ROOT)))
    if offenders:
        fail("public compat imports introduced unexpectedly: " + ", ".join(sorted(offenders)))


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke root-vs-compat compatibility strategy.")
    parser.parse_args()

    try:
        check_decision_doc()
        check_root_files_remain()
        check_active_dependencies()
        check_no_public_compat_import_surface()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("compat root strategy smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
