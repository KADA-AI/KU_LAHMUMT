# Mission Planning Structure

`modules/mission_planning` is organized conservatively so runtime behavior stays intact.

## Runtime entrypoints

- `mission_planning_gui.py`
  - actual GUI entrypoint used by the system
  - orchestrates initial planning / replan flow
  - imports:
    - `MissionPlanner/AnS/mission_pipeline.py`
    - `MissionPlanner/data_def/d0302.py`
    - `MissionPlanner/data_def/d0303.py`
    - `MissionPlanner/data_def/d0304.py`

- `prior_mission_pipeline.py`
- `attack_plan_pipeline.py`
  - top-level compatibility entrypoints kept for existing callers

## Main folders

- `MissionPlanner/`
  - core legacy mission-planning engine
  - keep this stable: most 0302/0303/0304 behavior still depends on it
  - `planning_enhanced/` now contains the migrated line/area split, expected-path,
    expected-velocity, and 0302 export logic copied out of `test_mission_planning`
    and adapted for production use
- `MissionVisualizer/`
  - standalone visualization tools
- `pipelines/`
  - attack/prior replan pipeline implementations
  - path trimming / attack helper logic
- `runtime/`
  - JSON I/O
  - latest input cache
  - attack assignment state
  - plan/file logging helpers
- `ui/`
  - GUI bootstrap helpers
  - support widgets / relationship explorer

## Top-level wrappers

Legacy import paths are preserved through thin wrapper modules at the top level.

Examples:

- `modules.mission_planning.attack_plan_pipeline`
- `modules.mission_planning.prior_mission_pipeline`
- `modules.mission_planning.json_io`
- `modules.mission_planning.attack_assignment_state`

The real implementations now live under `pipelines/`, `runtime/`, or `ui/`.

## Safe refactor boundary

Safe to reorganize:

- replan helper modules
- runtime/state/cache helpers
- UI helper modules
- documentation

Do not move casually:

- `MissionPlanner/AnS/`
- `MissionPlanner/data_def/`
- `mission_planning_gui.py`
- `MissionPlanner/planning_enhanced/`

Those are still the primary execution path for mission generation.
