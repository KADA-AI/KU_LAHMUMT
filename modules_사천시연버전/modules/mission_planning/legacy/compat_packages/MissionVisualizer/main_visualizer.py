"""Backward-compatible wrapper for the MissionVisualizer entry point."""

from modules.mission_planning.legacy.apps.MissionVisualizer.main_visualizer import MissionPlanVisualizer, main

__all__ = ["MissionPlanVisualizer", "main"]


if __name__ == "__main__":
    main()
