from __future__ import annotations

import argparse
import ast
import subprocess
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
DECISION_DOC = PROJECT_ROOT / "docs" / "mission planning refactoring" / "81-backup-style-file-deletion-decision.md"


BACKUP_STYLE_FILES = (
    "modules/mission_planning/MissionPlanner/data_def/d0304 copy.py",
)


TRAINING_ARTIFACT_PATTERNS = (
    "modules/mission_planning/MissionPlanner/AnS/Training/TensorBoard Logs",
    "modules/mission_planning/MissionPlanner/AnS/Training/TensorBoard Logs-Assign",
)


ACTIVE_REFERENCE_PATTERNS = (
    "d0304 copy.py",
    "events.out.tfevents",
    "TensorBoard Logs",
)


ALLOWED_REFERENCE_ROOTS = (
    "docs/mission planning refactoring/",
    "modules/mission_planning/MissionPlanner/AnS/Training/",
    "modules/mission_planning/MissionPlanner/data_def/d0304 copy.py",
)


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
        "backup-hold and artifact-hold",
        "Do not delete backup-style files or tracked training artifacts in this refactor phase.",
        "`MissionPlanner/data_def/d0304 copy.py` | backup-hold",
        "`MissionPlanner/AnS/Training/TensorBoard Logs*` event files | artifact-hold",
        "No runtime implementation changed.",
    )
    missing = [marker for marker in required if marker not in text]
    if missing:
        fail(f"{DECISION_DOC.relative_to(PROJECT_ROOT)} missing backup-style policy markers: {missing!r}")


def check_backup_style_candidates_exist_and_parse() -> None:
    tracked = set(git_ls_files(*BACKUP_STYLE_FILES))
    for rel_path in BACKUP_STYLE_FILES:
        path = PROJECT_ROOT / rel_path
        if not path.exists():
            fail(f"backup-style file missing: {rel_path}")
        if rel_path not in tracked:
            fail(f"backup-style file no longer tracked: {rel_path}")
        try:
            ast.parse(path.read_text(encoding="utf-8", errors="ignore"))
        except SyntaxError as exc:
            fail(f"backup-style file is not parseable: {rel_path}: {exc}")


def check_training_artifacts_remain_tracked() -> None:
    tracked = git_ls_files(*TRAINING_ARTIFACT_PATTERNS)
    event_files = [rel_path for rel_path in tracked if "/events.out.tfevents." in rel_path]
    if len(event_files) < 11:
        fail(f"tracked TensorBoard artifact set unexpectedly shrank: {len(event_files)} event files")
    for rel_path in event_files:
        if not (PROJECT_ROOT / rel_path).exists():
            fail(f"tracked TensorBoard artifact missing from worktree: {rel_path}")


def check_no_active_source_references_backup_artifacts() -> None:
    offenders: list[str] = []
    for path in (PROJECT_ROOT / "modules" / "mission_planning").rglob("*"):
        if not path.is_file():
            continue
        rel_path = str(path.relative_to(PROJECT_ROOT)).replace("\\", "/")
        if rel_path.startswith(ALLOWED_REFERENCE_ROOTS):
            continue
        if path.suffix.lower() not in {".py", ".md", ".txt", ".json", ".bat", ".ps1", ".yml", ".yaml"}:
            continue
        text = path.read_text(encoding="utf-8", errors="ignore")
        for pattern in ACTIVE_REFERENCE_PATTERNS:
            if pattern in text:
                offenders.append(f"{rel_path}: references {pattern!r}")
    if offenders:
        fail("active source references backup/training artifact candidates:\n" + "\n".join(offenders))


def check_existing_policy_chain() -> None:
    references = {
        "docs/mission planning refactoring/04-deletion-candidates.md": (
            "`MissionPlanner/data_def/d0304 copy.py`",
            "`MissionPlanner/AnS/Training/TensorBoard Logs*`",
        ),
        "docs/mission planning refactoring/77-deletion-owner-manual-workflow-progress.md": (
            "| `d0304 copy.py` | backup-hold | artifact-builder owner |",
            "| TensorBoard training logs/models | artifact-hold | training/model owner |",
        ),
        "docs/mission planning refactoring/76-deletion-candidate-reachability-progress.md": (
            "`d0304 copy.py`",
            "TensorBoard training artifacts",
        ),
    }
    for rel_path, markers in references.items():
        text = read_source(rel_path)
        missing = [marker for marker in markers if marker not in text]
        if missing:
            fail(f"{rel_path} missing backup-style guardrails: {missing!r}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke backup-style file deletion policy.")
    parser.parse_args()

    try:
        check_decision_doc()
        check_backup_style_candidates_exist_and_parse()
        check_training_artifacts_remain_tracked()
        check_no_active_source_references_backup_artifacts()
        check_existing_policy_chain()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("backup-style file policy smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
