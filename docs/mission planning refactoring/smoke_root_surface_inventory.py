from __future__ import annotations

import argparse
import importlib
import subprocess
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
MISSION_ROOT = PROJECT_ROOT / "modules" / "mission_planning"
README = MISSION_ROOT / "README.md"


ALLOWED_CATEGORIES = {
    "public",
    "active-package",
    "compat-wrapper",
    "manual-tool",
    "archive",
    "cleanup-candidate",
}


ROOT_SURFACE = {
    "__init__.py": "public",
    "README.md": "public",
    "_paths.py": "public",
    "mission_planning_gui.py": "public",
    "app": "active-package",
    "engine": "active-package",
    "mission_control": "active-package",
    "MissionPlanner": "active-package",
    "manual": "manual-tool",
    "next_area_mode": "active-package",
    "pipelines": "active-package",
    "planners": "active-package",
    "replanning": "active-package",
    "runtime": "active-package",
    "ui": "active-package",
    "lah_rl_planner_gui.py": "compat-wrapper",
    "legacy": "archive",
    "__pycache__": "cleanup-candidate",
}


OPTIONAL_ITEMS = {
    "__pycache__",
}


COMPAT_WRAPPER_MARKERS = (
    "import_compat_module(",
    "import_runtime_compat_module(",
    "import_module(",
    "from modules.mission_planning.",
)


ROOT_COMPAT_ALIAS_NAMES = (
    "attack_assignment_state",
    "attack_plan_pipeline",
    "id_relationship_tab",
    "imaging_schedule_replan_pipeline",
    "json_io",
    "latest_input_cache",
    "mission_path_trim",
    "mission_plan_file_logger",
    "mission_planning_attack_helpers",
    "mission_planning_gui_env",
    "mission_planning_log_tab",
    "mission_planning_pipeline_logging",
    "next_collab_replan_pipeline",
    "path_deviation_replan_pipeline",
    "prior_mission_pipeline",
    "prior_mission_pipeline_impl",
)


ROOT_COMPAT_PACKAGE_ALIAS_MODULES = (
    "modules.mission_planning.MissionVisualizer",
    "modules.mission_planning.MissionVisualizer.main_visualizer",
    "modules.mission_planning.logic_test.division_test.main",
    "modules.mission_planning.logic_test.dubins_test.dubins_turn_link_logic",
)


def fail(message: str) -> None:
    raise AssertionError(message)


def read_text(path: Path) -> str:
    if not path.exists():
        fail(f"{path.relative_to(PROJECT_ROOT)} is missing")
    return path.read_text(encoding="utf-8", errors="ignore")


def git_ls_files(*patterns: str) -> list[str]:
    result = subprocess.run(
        ["git", "ls-files", "--", *patterns],
        cwd=PROJECT_ROOT,
        capture_output=True,
        text=True,
        timeout=30,
    )
    if result.returncode != 0:
        fail(f"git ls-files failed: {result.stderr}")
    return [line.strip().replace("\\", "/") for line in result.stdout.splitlines() if line.strip()]


def check_every_root_item_classified() -> None:
    existing = {path.name for path in MISSION_ROOT.iterdir()}
    missing_classification = sorted(existing.difference(ROOT_SURFACE))
    if missing_classification:
        fail("unclassified mission_planning root items: " + ", ".join(missing_classification))

    missing_required = sorted(
        item
        for item in ROOT_SURFACE
        if item not in OPTIONAL_ITEMS and not (MISSION_ROOT / item).exists()
    )
    if missing_required:
        fail("classified mission_planning root items are missing: " + ", ".join(missing_required))

    invalid_categories = {
        item: category
        for item, category in ROOT_SURFACE.items()
        if category not in ALLOWED_CATEGORIES
    }
    if invalid_categories:
        fail(f"invalid root surface categories: {invalid_categories!r}")


def check_shape_by_category() -> None:
    offenders: list[str] = []
    for item, category in ROOT_SURFACE.items():
        path = MISSION_ROOT / item
        if not path.exists():
            continue
        if category in {"active-package", "manual-tool", "archive", "cleanup-candidate"}:
            if item.endswith((".py", ".md", ".html")):
                continue
            if not path.is_dir():
                offenders.append(f"{item}: expected directory-like category, got file")
        if category in {"public", "compat-wrapper"} and not path.is_file():
            offenders.append(f"{item}: expected file for {category}, got directory")
    if offenders:
        fail("root surface category shape mismatch:\n" + "\n".join(offenders))


def check_compat_wrappers_are_obvious_wrappers() -> None:
    offenders: list[str] = []
    for item, category in ROOT_SURFACE.items():
        if category != "compat-wrapper":
            continue
        path = MISSION_ROOT / item
        text = read_text(path)
        if not any(marker in text for marker in COMPAT_WRAPPER_MARKERS):
            offenders.append(f"{item}: no compatibility wrapper marker found")
        if "class " in text or "\ndef " in text:
            offenders.append(f"{item}: contains implementation class/function definitions")
    if offenders:
        fail("root compatibility wrapper shape failed:\n" + "\n".join(offenders))


def check_root_compat_aliases_are_importable() -> None:
    add_project_root = str(PROJECT_ROOT)
    if add_project_root not in sys.path:
        sys.path.insert(0, add_project_root)
    offenders: list[str] = []
    for name in ROOT_COMPAT_ALIAS_NAMES:
        if (MISSION_ROOT / f"{name}.py").exists():
            offenders.append(f"{name}.py: compatibility alias should not have a root wrapper file")
            continue
        try:
            importlib.import_module(f"modules.mission_planning.{name}")
        except Exception as exc:
            offenders.append(f"{name}: import failed: {exc}")
    for module_name in ROOT_COMPAT_PACKAGE_ALIAS_MODULES:
        root_name = module_name.split(".")[2]
        if (MISSION_ROOT / root_name).exists():
            offenders.append(f"{root_name}: compatibility alias should not have a root directory")
            continue
        try:
            importlib.import_module(module_name)
        except Exception as exc:
            offenders.append(f"{module_name}: import failed: {exc}")
    if offenders:
        fail("root compatibility alias contract failed:\n" + "\n".join(offenders))


def check_cache_policy() -> None:
    gitignore = read_text(PROJECT_ROOT / ".gitignore")
    if "__pycache__/" not in gitignore:
        fail(".gitignore must keep __pycache__/ ignored")

    tracked_cache = git_ls_files("modules/mission_planning/__pycache__", "modules/mission_planning/**/*.pyc")
    if tracked_cache:
        fail("mission_planning cache files are tracked unexpectedly: " + ", ".join(tracked_cache))


def check_readme_documents_inventory_policy() -> None:
    text = read_text(README)
    required = (
        "Root surface final cleanup TODO",
        "`public`, `active-package`, `compat-wrapper`, `manual-tool`, `archive`, or `cleanup-candidate`",
        "package-level aliases preserve old `MissionVisualizer` and `logic_test` import paths",
        "root inventory smoke fails if a new loose file appears without classification",
    )
    missing = [marker for marker in required if marker not in text]
    if missing:
        fail(f"{README.relative_to(PROJECT_ROOT)} missing root inventory markers: {missing!r}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke mission_planning root surface inventory.")
    parser.parse_args()

    try:
        check_every_root_item_classified()
        check_shape_by_category()
        check_compat_wrappers_are_obvious_wrappers()
        check_root_compat_aliases_are_importable()
        check_cache_policy()
        check_readme_documents_inventory_policy()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("mission planning root surface inventory smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
