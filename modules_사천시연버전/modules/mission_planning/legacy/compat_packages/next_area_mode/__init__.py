"""Backward-compatible wrapper for the next-area-mode app package."""

from modules.mission_planning.legacy.apps.next_area_mode import NextAreaModeLauncherTab, NextAreaPlanningWindow, main

__all__ = ["NextAreaModeLauncherTab", "NextAreaPlanningWindow", "main"]
