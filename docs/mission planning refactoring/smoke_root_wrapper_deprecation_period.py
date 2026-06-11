from __future__ import annotations

import argparse
import ast
import importlib
import sys
from datetime import date, timedelta
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
DECISION_DOC = PROJECT_ROOT / "docs" / "mission planning refactoring" / "79-root-wrapper-deprecation-period-decision.md"


ROOT_COMPAT_ALIAS_MODULES = (
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

ROOT_COMPAT_PACKAGE_ALIAS_MODULES = (
    "modules.mission_planning.MissionVisualizer.main_visualizer",
    "modules.mission_planning.logic_test.division_test.main",
    "modules.mission_planning.logic_test.dubins_test.dubins_turn_link_logic",
)


ACTIVE_CANONICAL_CALLER_MARKERS = {
    "modules/mission_planning/manual/MissionVisualizer/main_visualizer.py": (
        "from modules.mission_planning.ui.id_relationship_tab import RelationshipCache",
    ),
    "modules/mission_planning/__init__.py": (
        '"MissionVisualizer": "modules.mission_planning.manual.MissionVisualizer"',
        '"logic_test": "modules.mission_planning.manual.logic_test"',
    ),
    "modules/mission_planning/MissionPlanner/tools/main_visualizer.py": (
        'import_module("modules.mission_planning.manual.MissionVisualizer.main_visualizer")',
    ),
    "run.py": (
        '"mission": "mission_planning_gui.py"',
        '"mission_planning_gui.py": "mission"',
    ),
    "modules/common/button_wiring.py": (
        '"assignment": "mission_planning_gui.py"',
    ),
    "modules/monitoring/gui/tabs/monitoring_visualization_tab.py": (
        "from modules.mission_planning.pipelines.mission_path_trim import DEFAULT_SWEEP_SPLIT_LOOKAHEAD_SECONDS",
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


FORBIDDEN_DEPRECATION_MARKERS = (
    "DeprecationWarning",
    "deprecated import",
    "warnings.warn",
)


def fail(message: str) -> None:
    raise AssertionError(message)


def read_source(rel_path: str | Path) -> str:
    path = rel_path if isinstance(rel_path, Path) else PROJECT_ROOT / rel_path
    if not path.exists():
        fail(f"{path.relative_to(PROJECT_ROOT)} is missing")
    return path.read_text(encoding="utf-8", errors="ignore")


def check_decision_doc() -> None:
    text = read_source(DECISION_DOC)
    notice_date = date(2026, 6, 5)
    earliest_removal = notice_date + timedelta(days=30)
    required = (
        "The deprecation clock is not active in this refactor phase.",
        "Old root import paths remain supported compatibility paths.",
        "package-level lazy aliases",
        "minimum deprecation period is 30 calendar days",
        str(notice_date),
        str(earliest_removal),
        "old import surface remains intact",
    )
    missing = [marker for marker in required if marker not in text]
    if missing:
        fail(f"{DECISION_DOC.relative_to(PROJECT_ROOT)} missing deprecation-period markers: {missing!r}")


def check_root_wrappers_remain_import_safe() -> None:
    offenders: list[str] = []
    source = read_source("modules/mission_planning/__init__.py")
    try:
        ast.parse(source)
    except SyntaxError as exc:
        offenders.append(f"modules/mission_planning/__init__.py: parse failed: {exc}")
    for marker in FORBIDDEN_DEPRECATION_MARKERS:
        if marker.lower() in source.lower():
            offenders.append(f"modules/mission_planning/__init__.py: contains runtime deprecation marker {marker!r}")
    for module_name in ROOT_COMPAT_ALIAS_MODULES:
        short_name = module_name.rsplit(".", 1)[1]
        if f'"{short_name}"' not in source:
            offenders.append(f"{module_name}: missing from package alias map")
        if (PROJECT_ROOT / "modules" / "mission_planning" / f"{short_name}.py").exists():
            offenders.append(f"{module_name}: still has loose root wrapper file")
        try:
            importlib.import_module(module_name)
        except Exception as exc:
            offenders.append(f"{module_name}: import failed: {exc}")
    for module_name in ROOT_COMPAT_PACKAGE_ALIAS_MODULES:
        root_name = module_name.split(".")[2]
        if (PROJECT_ROOT / "modules" / "mission_planning" / root_name).exists():
            offenders.append(f"{module_name}: still has loose root directory")
        try:
            importlib.import_module(module_name)
        except Exception as exc:
            offenders.append(f"{module_name}: import failed: {exc}")
    if offenders:
        fail("root wrapper deprecation guard failed:\n" + "\n".join(offenders))


def check_active_root_surface_still_needed() -> None:
    for rel_path, markers in ACTIVE_CANONICAL_CALLER_MARKERS.items():
        text = read_source(rel_path)
        missing = [marker for marker in markers if marker not in text]
        if missing:
            fail(f"{rel_path} missing active caller markers: {missing!r}")
    for rel_path, markers in FORBIDDEN_ACTIVE_ROOT_WRAPPER_IMPORTS.items():
        text = read_source(rel_path)
        offenders = [marker for marker in markers if marker in text]
        if offenders:
            fail(f"{rel_path} still imports root compatibility wrappers: {offenders!r}")


def check_no_public_compat_surface_replaces_root_wrappers() -> None:
    compat_dir = PROJECT_ROOT / "modules" / "mission_planning" / "compat"
    if compat_dir.exists():
        fail("modules/mission_planning/compat exists before root-wrapper deprecation starts")

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
        fail("public compat imports introduced before deprecation starts: " + ", ".join(sorted(offenders)))


def check_policy_chain() -> None:
    references = {
        "docs/mission planning refactoring/73-compat-root-strategy-decision.md": (
            "Keep root compatibility paths for the current refactor",
            "package-level lazy aliases",
        ),
        "docs/mission planning refactoring/74-deprecated-import-policy-decision.md": (
            "Documentation-only deprecated import policy",
            "Do not emit runtime deprecation warnings",
        ),
        "docs/mission planning refactoring/77-deletion-owner-manual-workflow-progress.md": (
            "root compatibility wrappers",
            "Delete only after a documented deprecation period",
        ),
        "docs/mission planning refactoring/smoke_wrapper_template_contract.py": (
            "PACKAGE_ROOT_ALIAS_MODULES",
            "SPECIAL_PROXY_WRAPPERS",
        ),
        "docs/mission planning refactoring/smoke_import_contract.py": (
            "check_wrapper_identity",
            "check_moved_pipeline_wrapper_shape",
            "check_moved_runtime_wrapper_shape",
        ),
    }
    for rel_path, markers in references.items():
        text = read_source(rel_path)
        missing = [marker for marker in markers if marker not in text]
        if missing:
            fail(f"{rel_path} missing policy-chain markers: {missing!r}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke root wrapper deprecation-period policy.")
    parser.parse_args()

    try:
        check_decision_doc()
        check_root_wrappers_remain_import_safe()
        check_active_root_surface_still_needed()
        check_no_public_compat_surface_replaces_root_wrappers()
        check_policy_chain()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("root wrapper deprecation period smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
