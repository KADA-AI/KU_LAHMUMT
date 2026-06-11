from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DECISION_DOC = PROJECT_ROOT / "docs" / "mission planning refactoring" / "80-legacy-bucket-archive-strategy-decision.md"


LEGACY_DIRS = (
    "modules/mission_planning/legacy/apps",
    "modules/mission_planning/legacy/compat_packages",
    "modules/mission_planning/legacy/docs",
    "modules/mission_planning/legacy/logic_test",
    "modules/mission_planning/legacy/MissionPlanner_tools",
    "modules/mission_planning/legacy/static",
    "modules/mission_planning/legacy/tests",
    "modules/mission_planning/legacy/ui",
    "modules/mission_planning/legacy/wrappers",
)


REPRESENTATIVE_LEGACY_FILES = (
    "modules/mission_planning/legacy/README.md",
    "modules/mission_planning/legacy/__init__.py",
    "modules/mission_planning/legacy/wrappers/attack_plan_pipeline.py",
    "modules/mission_planning/legacy/wrappers/id_relationship_tab.py",
    "modules/mission_planning/legacy/compat_packages/MissionVisualizer/main_visualizer.py",
    "modules/mission_planning/legacy/compat_packages/next_area_mode/main.py",
    "modules/mission_planning/legacy/apps/MissionVisualizer/main_visualizer.py",
    "modules/mission_planning/legacy/tests/division_test/main.py",
    "modules/mission_planning/legacy/tests/dubins_test/dubins_turn_link_gui.py",
    "modules/mission_planning/legacy/logic_test/division_test/main.py",
    "modules/mission_planning/legacy/ui/id_relationship_tab.py",
    "modules/mission_planning/legacy/static/map.html",
)


LEGACY_WRAPPER_MARKERS = {
    "modules/mission_planning/legacy/wrappers/attack_plan_pipeline.py": (
        "modules.mission_planning.replanning.triggers.attack.pipeline",
    ),
    "modules/mission_planning/legacy/wrappers/id_relationship_tab.py": (
        "modules.mission_planning.legacy.ui.id_relationship_tab",
    ),
    "modules/mission_planning/legacy/compat_packages/MissionVisualizer/main_visualizer.py": (
        "modules.mission_planning.legacy.apps.MissionVisualizer.main_visualizer",
    ),
    "modules/mission_planning/legacy/compat_packages/next_area_mode/main.py": (
        "modules.mission_planning.legacy.apps.next_area_mode.main",
    ),
}


def fail(message: str) -> None:
    raise AssertionError(message)


def read_source(rel_path: str | Path) -> str:
    path = rel_path if isinstance(rel_path, Path) else PROJECT_ROOT / rel_path
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


def check_decision_doc() -> None:
    text = read_source(DECISION_DOC)
    required = (
        "archive-hold",
        "Do not delete or relocate the `legacy` bucket in this refactor phase.",
        "`legacy/wrappers` | keep-hold",
        "`legacy/compat_packages` | archive-hold",
        "`legacy/apps` | archive-hold",
        "`legacy/tests` | archive-hold",
        "`legacy/MissionPlanner_tools` | archive-hold",
        "`legacy/ui` | archive-hold",
        "No runtime implementation changed.",
    )
    missing = [marker for marker in required if marker not in text]
    if missing:
        fail(f"{DECISION_DOC.relative_to(PROJECT_ROOT)} missing legacy archive markers: {missing!r}")


def check_legacy_bucket_remains_in_place() -> None:
    missing_dirs = [rel_path for rel_path in LEGACY_DIRS if not (PROJECT_ROOT / rel_path).is_dir()]
    if missing_dirs:
        fail(f"legacy archive dirs missing: {missing_dirs!r}")
    missing_files = [rel_path for rel_path in REPRESENTATIVE_LEGACY_FILES if not (PROJECT_ROOT / rel_path).exists()]
    if missing_files:
        fail(f"representative legacy archive files missing: {missing_files!r}")

    tracked = git_ls_files("modules/mission_planning/legacy")
    if len(tracked) < 80:
        fail(f"tracked legacy archive unexpectedly shrank: {len(tracked)} tracked files")
    for rel_path in REPRESENTATIVE_LEGACY_FILES[:10]:
        if rel_path not in tracked:
            fail(f"representative legacy file no longer tracked: {rel_path}")


def check_legacy_compat_markers() -> None:
    for rel_path, markers in LEGACY_WRAPPER_MARKERS.items():
        text = read_source(rel_path)
        missing = [marker for marker in markers if marker not in text]
        if missing:
            fail(f"{rel_path} missing legacy compatibility markers: {missing!r}")


def check_existing_policy_chain() -> None:
    references = {
        "docs/mission planning refactoring/77-deletion-owner-manual-workflow-progress.md": (
            "| `legacy/wrappers` | keep-hold |",
            "| `legacy/compat_packages` | archive-hold |",
            "| `legacy/apps` | archive-hold |",
            "| `legacy/tests` | archive-hold |",
            "No deletion is approved by this checkpoint.",
        ),
        "docs/mission planning refactoring/78-generated-output-fixture-policy-progress.md": (
            "legacy/tests/division_test",
            "fixture-hold",
        ),
        "docs/mission planning refactoring/79-root-wrapper-deprecation-period-decision.md": (
            "The deprecation clock is not active",
            "No root wrapper removal is approved now.",
        ),
    }
    for rel_path, markers in references.items():
        text = read_source(rel_path)
        missing = [marker for marker in markers if marker not in text]
        if missing:
            fail(f"{rel_path} missing legacy strategy guardrails: {missing!r}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke legacy bucket archive strategy.")
    parser.parse_args()

    try:
        check_decision_doc()
        check_legacy_bucket_remains_in_place()
        check_legacy_compat_markers()
        check_existing_policy_chain()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("legacy bucket archive strategy smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
