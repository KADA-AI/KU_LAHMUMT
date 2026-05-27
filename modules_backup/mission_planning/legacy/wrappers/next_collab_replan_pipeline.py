"""Backward-compatible wrapper for the next-collaborative-mission replan pipeline entrypoint."""

from __future__ import annotations

import sys
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
_PROJECT_ROOT_STR = str(_PROJECT_ROOT)
if _PROJECT_ROOT_STR not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT_STR)

from modules.mission_planning.pipelines.next_collab_replan_pipeline import (
    run_next_collab_replan_pipeline,
    warm_next_collab_replan_pipeline,
)

__all__ = ["run_next_collab_replan_pipeline", "warm_next_collab_replan_pipeline"]
