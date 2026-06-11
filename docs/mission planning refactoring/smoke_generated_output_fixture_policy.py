from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[2]
POLICY_DOC = PROJECT_ROOT / "docs" / "mission planning refactoring" / "78-generated-output-fixture-policy-progress.md"


OUTPUT_BUCKETS = {
    "logic_test_active_0302": (
        "modules/mission_planning/manual/logic_test/division_test/output/auto_0302",
        3,
        "individualMissionPackageID",
    ),
    "logic_test_active_0303": (
        "modules/mission_planning/manual/logic_test/division_test/output/auto_0303",
        16,
        "pathID",
    ),
    "legacy_tests_0302": (
        "modules/mission_planning/legacy/tests/division_test/output/auto_0302",
        3,
        "individualMissionPackageID",
    ),
    "legacy_tests_0303": (
        "modules/mission_planning/legacy/tests/division_test/output/auto_0303",
        13,
        "pathID",
    ),
}


OUTPUT_WRITER_MARKERS = {
    "modules/mission_planning/manual/logic_test/division_test/division_planner_gui.py": (
        'root = Path(__file__).resolve().parent / "output"',
        'out_0302 = out_root / "auto_0302"',
        'out_0303 = out_root / "auto_0303"',
        "paths_0302 = save_0302_packages(packages_0302, out_0302)",
        "paths_0303 = save_0303_plans(fp_0303, out_0303)",
        "paths_0304 = save_0304_plans(fp_0304, out_0304)",
    ),
    "modules/mission_planning/legacy/tests/division_test/_planner_window.py": (
        'root = Path(__file__).resolve().parent / "output"',
        'out_0302 = out_root / "auto_0302"',
        'out_0303 = out_root / "auto_0303"',
        "paths_0302 = save_0302_packages(packages_0302, out_0302)",
        "paths_0303 = save_0303_plans(fp_0303, out_0303)",
        "paths_0304 = save_0304_plans(fp_0304, out_0304)",
    ),
}


def fail(message: str) -> None:
    raise AssertionError(message)


def read_source(rel_path: str | Path) -> str:
    path = rel_path if isinstance(rel_path, Path) else PROJECT_ROOT / rel_path
    if not path.exists():
        fail(f"{path.relative_to(PROJECT_ROOT)} is missing")
    return path.read_text(encoding="utf-8", errors="ignore")


def load_json(path: Path) -> Any:
    return json.loads(path.read_text(encoding="utf-8"))


def check_policy_doc() -> None:
    text = read_source(POLICY_DOC)
    required = (
        "fixture-hold",
        "Do not delete generated output JSON in this refactor phase.",
        "No deletion is approved",
        "manual/golden fixture candidate",
    )
    missing = [marker for marker in required if marker not in text]
    if missing:
        fail(f"{POLICY_DOC.relative_to(PROJECT_ROOT)} missing generated-output policy markers: {missing!r}")


def check_output_buckets() -> None:
    for label, (rel_dir, expected_count, required_key) in OUTPUT_BUCKETS.items():
        path = PROJECT_ROOT / rel_dir
        if not path.exists():
            fail(f"{label} output dir missing: {rel_dir}")
        files = sorted(item for item in path.glob("*.json") if item.is_file())
        if len(files) != int(expected_count):
            fail(f"{label} JSON count changed: expected {expected_count}, got {len(files)}")
        payload = load_json(files[0])
        if not isinstance(payload, dict):
            fail(f"{label} sample payload is not an object: {files[0]}")
        if required_key not in payload:
            fail(f"{label} sample payload missing {required_key}: {files[0]}")
        if "Source" not in payload:
            fail(f"{label} sample payload missing Source: {files[0]}")
        if required_key == "pathID":
            if "waypointList" not in payload:
                fail(f"{label} 0303 sample payload missing waypointList: {files[0]}")
        else:
            if "individualMissionList" not in payload:
                fail(f"{label} 0302 sample payload missing individualMissionList: {files[0]}")


def check_output_writer_markers() -> None:
    for rel_path, markers in OUTPUT_WRITER_MARKERS.items():
        text = read_source(rel_path)
        missing = [marker for marker in markers if marker not in text]
        if missing:
            fail(f"{rel_path} missing generated-output writer markers: {missing!r}")


def check_existing_deletion_holds() -> None:
    references = {
        "docs/mission planning refactoring/76-deletion-candidate-reachability-progress.md": (
            "manual/logic_test/division_test/output",
            "generated/golden candidates",
            "This smoke does not approve any deletion.",
        ),
        "docs/mission planning refactoring/77-deletion-owner-manual-workflow-progress.md": (
            "division-test generated output JSON",
            "fixture-hold",
            "No deletion is approved by this checkpoint.",
        ),
    }
    for rel_path, markers in references.items():
        text = read_source(rel_path)
        missing = [marker for marker in markers if marker not in text]
        if missing:
            fail(f"{rel_path} missing generated-output hold markers: {missing!r}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Smoke generated output fixture/delete policy.")
    parser.parse_args()

    try:
        check_policy_doc()
        check_output_buckets()
        check_output_writer_markers()
        check_existing_deletion_holds()
    except Exception as exc:
        print(str(exc), file=sys.stderr)
        return 1

    print("generated output fixture policy smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
