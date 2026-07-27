"""Backward-compatible wrapper for the imaging-schedule replan pipeline entrypoint."""

from __future__ import annotations

import sys
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
_PROJECT_ROOT_STR = str(_PROJECT_ROOT)
if _PROJECT_ROOT_STR not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT_STR)

from modules.mission_planning.replanning.triggers.imaging_schedule.pipeline import (
    run_imaging_schedule_replan_pipeline,
    warm_imaging_schedule_replan_pipeline,
)

__all__ = ["run_imaging_schedule_replan_pipeline", "warm_imaging_schedule_replan_pipeline"]
