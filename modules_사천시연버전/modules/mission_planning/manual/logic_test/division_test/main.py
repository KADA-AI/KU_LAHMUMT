from __future__ import annotations

import multiprocessing
import os
import sys
from pathlib import Path

from modules.mission_planning._paths import mission_planner_root, project_root


HERE = Path(__file__).resolve()
PKG_DIR = HERE.parent
PROJECT_ROOT = project_root()
MISSION_PLANNER_DIR = mission_planner_root()


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

    from division_planner_gui import main as gui_main

    return gui_main()


if __name__ == "__main__":
    raise SystemExit(main())
