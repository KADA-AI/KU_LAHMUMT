# Planning Enhanced Import Graph

Date: 2026-06-04, Asia/Seoul

## Completed

- Reviewed `MissionPlanner/planning_enhanced` import graph before any physical move.
- Added smoke coverage for the active non-GUI public surface:
  - `planning_enhanced.run_enhanced_divide_and_pattern`
  - `algo` split/review exports
  - `io` 0301/0302/0303/0304 exports
  - `pathing` expected path/velocity exports
  - `scheduling` scheduling exports
  - `type_decider` profile exports
  - `models` dataclasses
  - `runtime.next_collab_line_runner`
- Fixed bare `import planning_enhanced` from `MissionPlanner` cwd by adding project-root bootstrap in `planning_enhanced/__init__.py`.

## Current Import Graph

| Area | Depends On | Active Callers |
| --- | --- | --- |
| `planning_enhanced.pipeline` | `modules.common.db_paths`, `MissionPlanner.runtime_settings`, `assignment`, `algo`, `io`, `pathing`, `type_decider`, recon-specialized summaries | `AnS/mission_pipeline.py`, `MissionPlanner/main_MP.py`, smoke contract |
| `algo/*` | shapely/numpy geometry helpers, `models`, `assignment`, `scheduling.simple_scheduler` | next-collab trigger, next-area planner, division planner GUI/tests |
| `io/export_0301.py` | `data_def.id_allocator` compatibility path | enhanced pipeline/manual exports |
| `io/export_0302.py` | `models`, generated 0302 package contracts | enhanced pipeline/manual exports |
| `io/export_0303_0304.py` | `runtime.json_io`, `runtime_settings`, `data_def.d0303`, `data_def.d0304`, `data_def.search_speed`, `MissionPlanner.config` | enhanced pipeline/manual exports |
| `pathing/*` | shapely geometry, `models` | next-collab line runner, next-collab trigger, planner UI |
| `scheduling/*` | `models`, optional `pulp` fallback | planner UI/tests, scheduling package export |
| `type_decider/*` | `models` | enhanced pipeline, next-collab trigger |
| `gui/*`, `map/*` | PyQt5/folium UI dependencies | manual planner UI, not part of non-GUI smoke |

## Move Feasibility

`planning_enhanced` can move toward `engine/optimization`, but not as a blind directory rename.

Required before physical move:

- Keep `MissionPlanner/planning_enhanced` as a package wrapper, not a single-file wrapper, because callers import deep modules such as `planning_enhanced.algo.split_runner`, `planning_enhanced.io.export_0303_0304`, and `planning_enhanced.pathing.expected_path`.
- Preserve bare `import planning_enhanced` from `MissionPlanner` cwd.
- Decide whether PyQt/folium `gui` and `map` modules move with optimization logic or stay under a UI/manual-tools namespace.
- Move or wrap `data_def.d0301-d0304`, `MissionPlanner.config`, and `runtime_settings` contracts first or together, because `io/export_0303_0304.py` still reaches those modules.
- Update `AnS/mission_pipeline.py` and manual `main_MP.py` only after wrapper package coverage exists.

## Verification Passed

- `python "docs\mission planning refactoring\smoke_import_contract.py"`
- `python -m compileall modules\mission_planning\MissionPlanner\planning_enhanced "docs\mission planning refactoring\smoke_import_contract.py"`

## Progress Snapshot

- Starting point before this pass: 36 / 99 complete, 63 remaining, 36.4% complete.
- Starting Phase 5 point: 2 / 6 complete, 4 remaining, 33.3% complete.
- Overall roadmap after this checkbox: 37 / 99 complete, 62 remaining, 37.4% complete.
- Phase 5: 3 / 6 complete, 3 remaining, 50.0% complete.
