"""Backward-compatible wrapper for the division planner entrypoint."""

from __future__ import annotations

import sys
from pathlib import Path


HERE = Path(__file__).resolve()

def _find_project_root(start: Path) -> Path:
    for candidate in (start, *start.parents):
        if (candidate / "modules").exists() and (candidate / "run.py").exists():
            return candidate
    return start.parents[5]

PROJECT_ROOT = _find_project_root(HERE)

if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from modules.mission_planning.legacy.tests.division_test.main import *  # type: ignore
from modules.mission_planning.legacy.tests.division_test.main import main as _legacy_main


if __name__ == "__main__":
    raise SystemExit(_legacy_main())
