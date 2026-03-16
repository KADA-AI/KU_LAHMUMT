"""Backward-compatible wrapper for the prior mission pipeline entrypoint."""

from __future__ import annotations

import sys
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
_PROJECT_ROOT_STR = str(_PROJECT_ROOT)
if _PROJECT_ROOT_STR not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT_STR)

from modules.mission_planning.pipelines.prior_mission_pipeline_impl import (
    run_prior_mission_pipeline,
    warm_prior_mission_pipeline,
)

__all__ = ["run_prior_mission_pipeline", "warm_prior_mission_pipeline"]
