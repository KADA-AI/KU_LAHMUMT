# Mission Planning Structure

`modules/mission_planning` keeps only the active mission-planning runtime at the package root.

## Active root layout

- `mission_planning_gui.py`
  - main GUI entrypoint used by the system
  - orchestrates initial planning and replan flow
- `MissionPlanner/`
  - core mission-planning engine and production 0302/0303/0304 generation path
  - `data_def/` remains the primary execution path
  - `planning_enhanced/` remains production logic, not test code
- `pipelines/`
  - runtime pipeline implementations
  - attack, prior-mission, and replan helper logic
  - public runtime entrypoints such as `next_collab_replan_pipeline.py`
- `runtime/`
  - runtime JSON I/O
  - latest input cache
  - mission-planning state and logging helpers
  - next-collab replan store/runtime helpers used by monitoring and GUI
- `ui/`
  - active GUI widgets and environment helpers
- `legacy/`
  - archived wrappers, standalone tools, tests, documents, and static leftovers
- `_paths.py`
  - shared path helpers for runtime modules

## Preset cleanup

- The GUI exposes only the base preset: `dubins_mode`
- General mission-planning options now all run through the same base preset path
- Manual FOV is a mode inside the base preset, not a separate preset
- Removed preset/profile branches are documented under `legacy/`
- Runtime behavior that still matters follows downstream values such as:
  - `area_sweep_mode`
  - `area_split_mode`
  - `uav_plan_mode`
  - auto/manual FOV mode

In other words, preset-specific branching was removed from the general mission-planning path without rewriting the attack, prior-mission, or replan execution logic.

## Refactor boundary

Safe to archive or reorganize:

- wrapper modules no longer imported by active runtime code
- standalone app folders not used by the active runtime
- tests, exploratory tools, and documents
- static/generated leftovers

Current public entrypoints:

- next collaborative mission replan:
  - pipeline: `pipelines/next_collab_replan_pipeline.py`
  - runtime store/helpers: `runtime/next_collab_replan_store.py`, `runtime/next_collab_replan_runtime.py`
  - `modules/common/next_collab_replan_store.py` is compatibility-only

Do not move casually:

- `MissionPlanner/AnS/`
- `MissionPlanner/data_def/` except for narrowly-scoped runtime-safe helper extraction
- `MissionPlanner/planning_enhanced/`
- `mission_planning_gui.py`
- attack, prior-mission, and replan pipeline behavior

These remain the primary runtime path for mission generation.
