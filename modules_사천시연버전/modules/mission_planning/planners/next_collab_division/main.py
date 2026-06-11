from __future__ import annotations

import multiprocessing
import os
import sys
from pathlib import Path


HERE = Path(__file__).resolve()
PKG_DIR = HERE.parent


def _find_project_root(start: Path) -> Path:
    for candidate in (start, *start.parents):
        if (candidate / "modules").exists() and (candidate / "run.py").exists():
            return candidate
    return start.parents[4]


PROJECT_ROOT = _find_project_root(HERE)
MISSION_PLANNER_DIR = PROJECT_ROOT / "modules" / "mission_planning" / "MissionPlanner"


def _prepare_path() -> None:
    for candidate in (PKG_DIR, PROJECT_ROOT, MISSION_PLANNER_DIR):
        candidate_str = str(candidate)
        if candidate_str not in sys.path:
            sys.path.insert(0, candidate_str)


def main() -> int:
    if os.name == "nt":
        multiprocessing.freeze_support()

    _prepare_path()
    # Match the production initial mission-planning pipeline by default.
    os.environ.setdefault("DIVISION_TEST_FLOW_MODE", "initial")

    from modules.mission_planning.planners.next_collab_division.division_planner_gui import main as gui_main

    return gui_main()


if __name__ == "__main__":
    raise SystemExit(main())
