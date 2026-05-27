# MissionPlanner Folder Notes

This folder is still the core mission-generation engine.

## Active runtime path

The current runtime flow is effectively:

1. `modules/mission_planning/mission_planning_gui.py`
2. `MissionPlanner/AnS/mission_pipeline.py`
3. `MissionPlanner/data_def/d0302.py`
4. `MissionPlanner/data_def/d0303.py`
5. `MissionPlanner/data_def/d0304.py`

That means `AnS/` and `data_def/` should be treated as the high-risk zone for refactors.

## Subfolders / files

- `AnS/`
  - assignment / division / pattern selection pipeline
- `data_def/`
  - 0301/0302/0303/0304 generation logic
  - allocators, helpers, attack support
- `planning_enhanced/`
  - productionized copy of the stronger line/area planning pipeline from
    `test_mission_planning`
  - owns split refinement, expected-path generation, expected-velocity selection,
    mission type/pattern decision, and enhanced 0302 export
- `tools/UAV_pattern/`
  - reusable geometry / path-pattern prototypes
- `main_MP.py`
  - standalone/manual mission-planning app
- `corridor_*`, `tools/main_visualizer.py`
  - auxiliary tools, not the main runtime path

## Refactor guidance

Prefer:

- improving imports around this folder
- documenting active vs auxiliary files
- isolating replan/runtime helpers outside this folder

Avoid:

- moving `AnS/` or `data_def/` without a full runtime regression pass
- changing `planning_enhanced/` interfaces without checking both `run_divide_and_pattern`
  and `d0303.build_flight_plans(...)`
- renaming core files that are imported dynamically
