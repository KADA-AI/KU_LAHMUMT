from __future__ import annotations

from pathlib import Path


_MISSION_PLANNING_ROOT = Path(__file__).resolve().parent
_PROJECT_ROOT = _MISSION_PLANNING_ROOT.parents[1]
_MISSION_PLANNER_ROOT = _MISSION_PLANNING_ROOT / "MissionPlanner"
_DATA_DEF_ROOT = _MISSION_PLANNER_ROOT / "data_def"


def mission_planning_root() -> Path:
    return _MISSION_PLANNING_ROOT


def project_root() -> Path:
    return _PROJECT_ROOT


def mission_planner_root() -> Path:
    return _MISSION_PLANNER_ROOT


def mission_planner_data_def_root() -> Path:
    return _DATA_DEF_ROOT
