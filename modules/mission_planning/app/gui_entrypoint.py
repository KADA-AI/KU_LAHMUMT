"""Entrypoint helpers for the public mission_planning_gui launcher."""

from __future__ import annotations

import os
import sys
from types import ModuleType
from typing import Any, MutableMapping, Sequence


SMOKE_ENV_KEY = "MISSION_PLANNING_GUI_SMOKE_LAUNCH"


def smoke_launch_main(gui_module: ModuleType | Any) -> int:
    app = gui_module.QApplication.instance()
    if app is None:
        app = gui_module.QApplication([sys.argv[0]])
    gui_module.load_shared_stylesheet(app, gui_module.PROJECT_ROOT)
    if not issubclass(gui_module.MainWindow, gui_module.QMainWindow):
        print("mission_planning_gui smoke launch failed: MainWindow is not a QMainWindow", file=sys.stderr)
        return 1
    try:
        gui_module._planner_runtime_source_signature()
    except Exception as exc:
        print(f"mission_planning_gui smoke launch failed: planner runtime signature error: {exc!r}", file=sys.stderr)
        return 1
    gui_module.QTimer.singleShot(0, app.quit)
    result = int(app.exec_())
    if result == 0:
        print("mission_planning_gui smoke launch ok")
    return result


def run_gui(gui_module: ModuleType | Any, argv: Sequence[str] | None = None) -> int:
    app = gui_module.QApplication(list(sys.argv if argv is None else argv))
    gui_module.load_shared_stylesheet(app, gui_module.PROJECT_ROOT)
    win = gui_module.MainWindow()
    gui_module.apply_initial_visibility(app, win, gui_module.position_window_from_env)
    return int(app.exec_())


def run_public_gui_entrypoint(
    gui_module: ModuleType | Any,
    *,
    argv: Sequence[str] | None = None,
    environ: MutableMapping[str, str] | None = None,
) -> int:
    env = os.environ if environ is None else environ
    if env.get(SMOKE_ENV_KEY) == "1":
        return smoke_launch_main(gui_module)
    return run_gui(gui_module, argv=argv)
