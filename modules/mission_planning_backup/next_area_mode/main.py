from __future__ import annotations

import multiprocessing
import os
import sys
from pathlib import Path

HERE = Path(__file__).resolve()
PKG_DIR = HERE.parent
PROJECT_ROOT = HERE.parents[3]
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
    from modules.mission_planning.next_area_mode.config import FLOW_MODE_ENV_KEY
    os.environ.setdefault(FLOW_MODE_ENV_KEY, "initial")

    from modules.mission_planning.next_area_mode.planner_window import main as gui_main

    return gui_main()


if __name__ == "__main__":
    raise SystemExit(main())
