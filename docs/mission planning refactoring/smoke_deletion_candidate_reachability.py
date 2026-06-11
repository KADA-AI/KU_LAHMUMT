from __future__ import annotations

import argparse
import hashlib
import importlib
import subprocess
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


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


LEGACY_WRAPPER_FILES = (
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
)


LEGACY_ARCHIVE_FILES = (
    "modules/mission_planning/legacy/apps/MissionVisualizer/main_visualizer.py",
    "modules/mission_planning/legacy/MissionPlanner_tools/main_visualizer.py",
    "modules/mission_planning/legacy/compat_packages/MissionVisualizer/main_visualizer.py",
    "modules/mission_planning/legacy/compat_packages/next_area_mode/main.py",
    "modules/mission_planning/legacy/tests/division_test/main.py",
    "modules/mission_planning/legacy/tests/dubins_test/dubins_turn_link_gui.py",
)


ACTIVE_HOLD_FILES = (
    "modules/mission_planning/mission_planning_gui.py",
    "modules/mission_planning/manual/MissionVisualizer/main_visualizer.py",
    "modules/mission_planning/MissionPlanner/tools/main_visualizer.py",
    "modules/mission_planning/lah_rl_planner_gui.py",
    "modules/mission_planning/manual/lah_rl_planner_gui.py",
    "modules/mission_planning/MissionPlanner/portable_mission_bundle/app.py",
    "modules/mission_planning/MissionPlanner/portable_mission_bundle/run_portable.bat",
    "modules/mission_planning/MissionPlanner/portable_mission_bundle/models/latest_model.zip",
    "modules/mission_planning/MissionPlanner/portable_mission_bundle/models/model_config.json",
    "modules/mission_planning/manual/logic_test/division_test/main.py",
    "modules/mission_planning/manual/logic_test/division_test/division_planner_gui.py",
    "modules/mission_planning/manual/logic_test/dubins_test/dubins_turn_link_logic.py",
    "modules/mission_planning/manual/logic_test/dubins_test/dubins_turn_link_gui.py",
    "modules/mission_planning/MissionPlanner/data_def/dubins_turn_link.py",
    "modules/mission_planning/MissionPlanner/tools/UAV_pattern/Nadir_BF/area_nadir_bf_planner.py",
)


SOURCE_MARKERS = {
    "modules/mission_planning/manual/MissionVisualizer/main_visualizer.py": (
        "from modules.mission_planning.ui.id_relationship_tab import RelationshipCache",
        "def main(",
    ),
    "modules/mission_planning/__init__.py": (
        '"MissionVisualizer": "modules.mission_planning.manual.MissionVisualizer"',
        '"logic_test": "modules.mission_planning.manual.logic_test"',
        "class _CompatTargetModuleLoader",
    ),
    "modules/mission_planning/MissionPlanner/tools/main_visualizer.py": (
        'import_module("modules.mission_planning.manual.MissionVisualizer.main_visualizer")',
        "MissionPlanVisualizer = _MODULE.MissionPlanVisualizer",
        "main = _MODULE.main",
    ),
    "modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0303.py": (
        "from modules.mission_planning.MissionPlanner.tools.UAV_pattern.Nadir_BF.area_nadir_bf_planner import",
        "from tools.UAV_pattern.Nadir_BF.area_nadir_bf_planner import",
        "from modules.mission_planning.MissionPlanner.data_def.dubins_turn_link import",
    ),
    "modules/mission_planning/MissionPlanner/tools/UAV_pattern/Nadir_BF/area_nadir_bf_planner.py": (
        "from .BF import BFPlanner",
        "from .area_nadir_planner import build_nadir_overflight_coords",
    ),
    "modules/mission_planning/MissionPlanner/tools/UAV_pattern/Nadir_BF/BF.py": (
        "from .Dubins_Path import DubinsPath",
        "from .BF_DB_Generate import BFDBGenerate",
        "from Dubins_Path import DubinsPath",
        "from BF_DB_Generate import BFDBGenerate",
    ),
    "modules/mission_planning/engine/mission_generation/artifacts_0301_0302_0303_0304/d0304.py": (
        'bundle_root = _MISSION_PLANNER_DIR / "portable_mission_bundle"',
    ),
    "modules/mission_planning/lah_rl_planner_gui.py": (
        'import_module("modules.mission_planning.manual.lah_rl_planner_gui")',
    ),
    "modules/mission_planning/manual/lah_rl_planner_gui.py": (
        '_BUNDLE_ROOT = _MISSION_ROOT / "MissionPlanner" / "portable_mission_bundle"',
        '"models" / "latest_model.zip"',
    ),
    "modules/mission_planning/mission_planning_gui.py": (
        "from modules.mission_planning.manual.lah_rl_planner_gui import LAHPlannerWindow",
        "from lah_rl_planner_gui import LAHPlannerWindow",
    ),
    "modules/mission_planning/runtime/next_collab_line_runner.py": (
        "from modules.mission_planning.next_area_mode.config import",
        "from modules.mission_planning.next_area_mode.planner_window import",
    ),
    "modules/mission_planning/runtime/next_collab_division_runner.py": (
        "from modules.mission_planning.planners.next_collab_division._planner_window import",
    ),
    "modules/mission_planning/legacy/compat_packages/next_area_mode/main.py": (
        "from modules.mission_planning.legacy.apps.next_area_mode.main import main",
    ),
    "modules/mission_planning/legacy/compat_packages/MissionVisualizer/main_visualizer.py": (
        "from modules.mission_planning.legacy.apps.MissionVisualizer.main_visualizer import",
    ),
    "modules/mission_planning/legacy/logic_test/division_test/main.py": (
        "from modules.mission_planning.planners.next_collab_division.main import main as _legacy_main",
    ),
}


DOCUMENT_MARKERS = {
    "docs/mission planning refactoring/04-deletion-candidates.md": (
        "삭제 확정 목록이 아니다",
        "root compatibility wrappers",
        "legacy/wrappers",
        "logic_test/*/output",
        "d0304 copy.py",
        "duplicate visualizer copy",
        "Nadir_BF",
    ),
    "docs/mission planning refactoring/67-manual-workflow-owner-decisions-progress.md": (
        "| `manual/logic_test/division_test/**` | delete-hold |",
        "| `manual/logic_test/dubins_test/**` | wrapper candidate |",
        "| `MissionPlanner/tools/main_visualizer.py` | wrapper |",
        "No delete action is approved by this checkpoint.",
    ),
    "docs/mission planning refactoring/73-compat-root-strategy-decision.md": (
        "Keep root compatibility paths for the current refactor",
    ),
}


TRACKED_EXPECTED = (
    "modules/mission_planning/MissionPlanner/data_def/d0304 copy.py",
    "modules/mission_planning/MissionPlanner/AnS/Training/TensorBoard Logs/PPO_BalancedCase_1/events.out.tfevents.1734704170.LAHMUMT4.9964.2",
    "modules/mission_planning/MissionPlanner/AnS/Training/TensorBoard Logs-Assign/PPO_1/events.out.tfevents.1734861413.LAHMUMT4.8772.0",
)


def fail(message: str) -> None:
    raise AssertionError(message)


def read_source(rel_path: str) -> str:
    path = PROJECT_ROOT / rel_path
    if not path.exists():
        fail(f"{rel_path} is missing")
    return path.read_text(encoding="utf-8", errors="ignore")


def expect_paths(label: str, rel_paths: tuple[str, ...]) -> None:
    missing = [rel_path for rel_path in rel_paths if not (PROJECT_ROOT / rel_path).exists()]
    if missing:
        fail(f"{label} paths missing: {missing!r}")


def expect_source_contains(rel_path: str, *markers: str) -> None:
    text = read_source(rel_path)
    missing = [marker for marker in markers if marker not in text]
    if missing:
        fail(f"{rel_path} missing reachability markers: {missing!r}")


def file_sha256(rel_path: str) -> str:
    return hashlib.sha256((PROJECT_ROOT / rel_path).read_bytes()).hexdigest()


def count_json_files(rel_dir: str) -> int:
    path = PROJECT_ROOT / rel_dir
    if not path.exists():
        fail(f"{rel_dir} is missing")
    return sum(1 for item in path.rglob("*.json") if item.is_file())


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
    return [line.strip() for line in result.stdout.splitlines() if line.strip()]


def check_wrapper_reachability() -> None:
    alias_source = read_source("modules/mission_planning/__init__.py")
    for module_name in ROOT_COMPAT_ALIAS_MODULES:
        short_name = module_name.rsplit(".", 1)[1]
        if f'"{short_name}"' not in alias_source:
            fail(f"root compatibility alias missing: {module_name}")
        if (PROJECT_ROOT / "modules" / "mission_planning" / f"{short_name}.py").exists():
            fail(f"root compatibility alias still has loose wrapper file: {short_name}.py")
        importlib.import_module(module_name)
    expect_paths("legacy wrapper", LEGACY_WRAPPER_FILES)
    for rel_path in LEGACY_WRAPPER_FILES:
        text = read_source(rel_path)
        if "import_module(" not in text and "from " not in text:
            fail(f"{rel_path} no longer looks like a compatibility wrapper")


def check_archive_and_manual_surfaces() -> None:
    expect_paths("legacy archive", LEGACY_ARCHIVE_FILES)
    expect_paths("active hold", ACTIVE_HOLD_FILES)
    for rel_path, markers in SOURCE_MARKERS.items():
        expect_source_contains(rel_path, *markers)

    for rel_path in (
        "modules/mission_planning/MissionPlanner/tools/main_visualizer.py",
    ):
        text = read_source(rel_path)
        if 'import_module("modules.mission_planning.manual.MissionVisualizer.main_visualizer")' not in text:
            fail(f"{rel_path} no longer delegates to the manual visualizer")


def check_generated_output_candidates() -> None:
    expected_counts = {
        "modules/mission_planning/manual/logic_test/division_test/output/auto_0302": 3,
        "modules/mission_planning/manual/logic_test/division_test/output/auto_0303": 16,
        "modules/mission_planning/legacy/tests/division_test/output/auto_0302": 3,
        "modules/mission_planning/legacy/tests/division_test/output/auto_0303": 13,
    }
    for rel_dir, expected_count in expected_counts.items():
        actual_count = count_json_files(rel_dir)
        if actual_count != expected_count:
            fail(f"{rel_dir} JSON count changed: expected {expected_count}, got {actual_count}")


def check_prototype_and_backup_candidates() -> None:
    prototype_paths = (
        "modules/mission_planning/MissionPlanner/tools/UAV_pattern/Standoff_Sweep_ROI/Dubins_Path.py",
        "modules/mission_planning/MissionPlanner/tools/UAV_pattern/Standoff_Sweep_ROI/main_example.py",
        "modules/mission_planning/MissionPlanner/tools/UAV_pattern/Standoff_Sweep_ROI/testtest.py",
        "modules/mission_planning/MissionPlanner/tools/UAV_pattern/Standoff_BF/Dubins_Path.py",
        "modules/mission_planning/MissionPlanner/tools/UAV_pattern/Interval_Round_Trip_BF/Dubins_Path.py",
        "modules/mission_planning/MissionPlanner/tools/UAV_pattern/Interval_Round_Trip_BF/Inerval_Round_Flight_BF.py",
        "modules/mission_planning/MissionPlanner/tools/UAV_pattern/Nadir_BF/Dubins_Path.py",
    )
    expect_paths("prototype", prototype_paths)
    expect_paths(
        "backup/training",
        (
            "modules/mission_planning/MissionPlanner/data_def/d0304 copy.py",
            "modules/mission_planning/MissionPlanner/AnS/Training/TensorBoard Logs/PPO_BalancedCase_1/events.out.tfevents.1734704170.LAHMUMT4.9964.2",
            "modules/mission_planning/MissionPlanner/AnS/Training/TensorBoard Logs-Assign/PPO_1/events.out.tfevents.1734861413.LAHMUMT4.8772.0",
        ),
    )

    tracked = set(git_ls_files(*TRACKED_EXPECTED))
    missing_tracked = [rel_path for rel_path in TRACKED_EXPECTED if rel_path not in tracked]
    if missing_tracked:
        fail(f"expected tracked deletion candidates missing from git index: {missing_tracked!r}")

    tracked_pycache = git_ls_files("*__pycache__*", "*.pyc")
    if tracked_pycache:
        fail(f"tracked pycache/pyc files should not be deletion candidates: {tracked_pycache!r}")

    active_reference_patterns = (
        "d0304 copy.py",
        "events.out.tfevents",
        "TensorBoard Logs",
    )
    allowed_reference_roots = (
        "docs/mission planning refactoring/",
        "modules/mission_planning/MissionPlanner/AnS/Training/",
        "modules/mission_planning/MissionPlanner/data_def/d0304 copy.py",
    )
    for path in (PROJECT_ROOT / "modules" / "mission_planning").rglob("*"):
        if not path.is_file():
            continue
        if path.suffix.lower() not in {".py", ".md", ".txt", ".json", ".bat", ".ps1", ".yml", ".yaml"}:
            continue
        rel_path = str(path.relative_to(PROJECT_ROOT)).replace("\\", "/")
        if rel_path.startswith(allowed_reference_roots):
            continue
        try:
            text = path.read_text(encoding="utf-8", errors="ignore")
        except Exception:
            continue
        for pattern in active_reference_patterns:
            if pattern in text:
                fail(f"{rel_path} unexpectedly references backup/training candidate marker {pattern!r}")


def check_deletion_documents() -> None:
    for rel_path, markers in DOCUMENT_MARKERS.items():
        expect_source_contains(rel_path, *markers)


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke deletion candidate reachability.")
    parser.parse_args()

    try:
        check_deletion_documents()
        check_wrapper_reachability()
        check_archive_and_manual_surfaces()
        check_generated_output_candidates()
        check_prototype_and_backup_candidates()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("deletion candidate reachability smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
