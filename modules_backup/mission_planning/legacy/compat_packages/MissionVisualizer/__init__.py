"""Backward-compatible wrapper for the MissionVisualizer app package."""

from modules.mission_planning.legacy.apps.MissionVisualizer import MissionPlanVisualizer, main

__all__ = ["MissionPlanVisualizer", "main"]
