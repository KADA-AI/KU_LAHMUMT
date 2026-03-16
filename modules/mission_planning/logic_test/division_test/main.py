from __future__ import annotations

import multiprocessing
import os
import sys
from pathlib import Path


HERE = Path(__file__).resolve()
PKG_DIR = HERE.parent
PROJECT_ROOT = HERE.parents[4]
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

    from division_planner_gui import main as gui_main

    return gui_main()


if __name__ == "__main__":
    raise SystemExit(main())
