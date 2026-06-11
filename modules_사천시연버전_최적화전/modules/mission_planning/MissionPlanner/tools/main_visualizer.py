"""Compatibility wrapper for the canonical mission visualizer entrypoint."""
from __future__ import annotations

from importlib import import_module


_MODULE = import_module("modules.mission_planning.manual.MissionVisualizer.main_visualizer")

MissionPlanVisualizer = _MODULE.MissionPlanVisualizer
main = _MODULE.main

__all__ = ["MissionPlanVisualizer", "main"]


if __name__ == "__main__":
    main()
