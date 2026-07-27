from __future__ import annotations

import multiprocessing
import os
import sys
from pathlib import Path


HERE = Path(__file__).resolve()
PKG_DIR = HERE.parent
PROJECT_ROOT = HERE.parents[3]


def _prepare_path() -> None:
    for candidate in (PKG_DIR, PROJECT_ROOT):
        candidate_str = str(candidate)
        if candidate_str not in sys.path:
            sys.path.insert(0, candidate_str)


def main() -> int:
    if os.name == "nt":
        multiprocessing.freeze_support()

    _prepare_path()

    from PyQt5.QtWidgets import QApplication
    from planning_enhanced.gui import MainWindow

    app = QApplication.instance() or QApplication(sys.argv)
    win = MainWindow()
    win.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
