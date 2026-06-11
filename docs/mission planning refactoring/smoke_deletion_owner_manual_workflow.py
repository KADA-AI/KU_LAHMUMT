from __future__ import annotations

import argparse
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DECISION_DOC = PROJECT_ROOT / "docs" / "mission planning refactoring" / "77-deletion-owner-manual-workflow-progress.md"


OWNER_MATRIX_MARKERS = (
    "| root compatibility wrappers | keep | compatibility/import-surface owner |",
    "| `legacy/wrappers` | keep-hold | compatibility/archive owner |",
    "| `legacy/compat_packages` | archive-hold | compatibility/archive owner |",
    "| `legacy/apps` | archive-hold | manual app archive owner |",
    "| `legacy/tests` | archive-hold | manual/golden fixture owner |",
    "| `manual/logic_test/division_test/**` | delete-hold | next-collab/planning-enhanced owner |",
    "| `manual/logic_test/dubins_test/**` | wrapper candidate | Dubins/flight-path owner |",
    "| division-test generated output JSON | fixture-hold | generated fixture owner |",
    "| duplicate visualizer import path | package-alias | operator/manual visualization owner |",
    "| `MissionPlanner/tools/UAV_pattern/Nadir_BF/**` | keep | mission-generation owner |",
    "| other UAV-pattern prototype scripts | archive-hold | manual prototype owner needed |",
    "| portable mission bundle | keep | portable/RL workflow owner |",
    "| `d0304 copy.py` | backup-hold | artifact-builder owner |",
    "| TensorBoard training logs/models | artifact-hold | training/model owner |",
    "| tracked `__pycache__`/`.pyc` | delete-if-present | repository hygiene owner |",
)


SOURCE_GUARD_MARKERS = {
    "docs/mission planning refactoring/67-manual-workflow-owner-decisions-progress.md": (
        "| `manual/logic_test/division_test/**` | delete-hold | next-collab/planning-enhanced owner |",
        "| `MissionPlanner/tools/main_visualizer.py` | wrapper | operator/manual visualization |",
        "No delete action is approved by this checkpoint.",
    ),
    "docs/mission planning refactoring/73-compat-root-strategy-decision.md": (
        "Keep root compatibility paths for the current refactor",
    ),
    "docs/mission planning refactoring/76-deletion-candidate-reachability-progress.md": (
        "This smoke does not approve any deletion.",
        "Next incomplete TODO: confirm deletion candidate owner/manual workflow.",
    ),
}


def fail(message: str) -> None:
    raise AssertionError(message)


def read_source(rel_path: str | Path) -> str:
    path = rel_path if isinstance(rel_path, Path) else PROJECT_ROOT / rel_path
    if not path.exists():
        fail(f"{path.relative_to(PROJECT_ROOT)} is missing")
    return path.read_text(encoding="utf-8", errors="ignore")


def check_owner_matrix() -> None:
    text = read_source(DECISION_DOC)
    missing = [marker for marker in OWNER_MATRIX_MARKERS if marker not in text]
    if missing:
        fail(f"{DECISION_DOC.relative_to(PROJECT_ROOT)} missing owner matrix rows: {missing!r}")
    for required in (
        "No deletion is approved by this checkpoint.",
        "manual workflow",
        "owner bucket",
        "future delete/archive batch",
    ):
        if required not in text:
            fail(f"{DECISION_DOC.relative_to(PROJECT_ROOT)} missing decision marker {required!r}")


def check_existing_guardrails() -> None:
    for rel_path, markers in SOURCE_GUARD_MARKERS.items():
        text = read_source(rel_path)
        missing = [marker for marker in markers if marker not in text]
        if missing:
            fail(f"{rel_path} missing owner/manual guardrails: {missing!r}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke deletion owner/manual workflow decisions.")
    parser.parse_args()

    try:
        check_owner_matrix()
        check_existing_guardrails()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("deletion owner/manual workflow smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
