"""Backward-compatible wrapper for the path-deviation replan pipeline entrypoint."""

from __future__ import annotations

import sys
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
_PROJECT_ROOT_STR = str(_PROJECT_ROOT)
if _PROJECT_ROOT_STR not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT_STR)

from modules.mission_planning.replanning.triggers.path_deviation.pipeline import (
    run_path_deviation_replan_pipeline,
    warm_path_deviation_replan_pipeline,
)

__all__ = ["run_path_deviation_replan_pipeline", "warm_path_deviation_replan_pipeline"]
