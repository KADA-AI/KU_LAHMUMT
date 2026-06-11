"""Division planner GUI -- thin re-export facade.

All implementation lives in sub-modules:
  _constants, _geo_utils, _canvas_state, _planning_canvas, _gantt_chart, _planner_window
"""
from __future__ import annotations

import sys

from PyQt5.QtWidgets import QApplication

# Re-export public names so existing importers keep working.
from ._constants import *          # noqa: F401,F403
from ._geo_utils import *          # noqa: F401,F403
from ._canvas_state import *       # noqa: F401,F403
from ._planning_canvas import *    # noqa: F401,F403
from ._gantt_chart import *        # noqa: F401,F403
from ._planner_window import *     # noqa: F401,F403

# Explicit re-exports for the names callers rely on.
from ._planner_window import DivisionPlannerWindow  # noqa: F811
from ._planning_canvas import PlanningCanvas  # noqa: F811
from ._gantt_chart import GanttChartWidget  # noqa: F811
from ._canvas_state import CanvasState  # noqa: F811


def main() -> int:
    app = QApplication.instance() or QApplication(sys.argv)
    win = DivisionPlannerWindow()
    win.show()
    return app.exec_()


__all__ = [
    "DivisionPlannerWindow",
    "PlanningCanvas",
    "GanttChartWidget",
    "CanvasState",
    "main",
]


if __name__ == "__main__":
    raise SystemExit(main())
