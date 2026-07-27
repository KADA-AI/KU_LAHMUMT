# -*- coding: utf-8 -*-
"""Minimal KU dashboard launcher.

The dashboard implementation lives in ``modules.run`` so future deployments
only need the ``modules`` directory. Keep this root file stable as the public
operator entrypoint.
"""

from __future__ import annotations

import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parent
PROJECT_ROOT_TEXT = str(PROJECT_ROOT)
if PROJECT_ROOT_TEXT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT_TEXT)


def main() -> None:
    from modules.run import main as dashboard_main

    dashboard_main()


if __name__ == "__main__":
    main()
