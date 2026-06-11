# Trigger Move Progress

Date: 2026-06-04, Asia/Seoul

## Completed In This Pass

- Added executable contract smoke:
  - `docs/mission planning refactoring/smoke_import_contract.py`
  - Default mode verifies current worktree imports, wrapper identity, planner runtime canonical reload paths, import order, and stale moved-impl imports.
  - `--require-git-tracked` is reserved for PR/staging checks.
- Added wrapper support matrix:
  - `docs/mission planning refactoring/10-wrapper-support-matrix.md`
- Physically moved prior implementation:
  - Canonical: `modules.mission_planning.replanning.triggers.prior.pipeline`
  - Compatibility wrapper: `modules.mission_planning.pipelines.prior_mission_pipeline_impl`
  - Root/legacy wrappers now import canonical trigger path.
  - Planner runtime watch/reload/bindings now use canonical trigger path.
- Physically moved next-collab implementation:
  - Canonical: `modules.mission_planning.replanning.triggers.next_collab.pipeline`
  - Compatibility wrapper: `modules.mission_planning.pipelines.next_collab_replan_pipeline_impl`
  - Public wrapper `modules.mission_planning.pipelines.next_collab_replan_pipeline` remains supported.
  - Root/legacy wrappers now import canonical trigger path.
  - Planner runtime watch/reload/bindings now use canonical trigger path.

## Verification Passed

- `python "docs\mission planning refactoring\smoke_import_contract.py"`
- `python -m compileall` for moved trigger packages, wrappers, planner runtime, and touched hybrid support files.
- GUI import surface smoke:
  - prior
  - prior post-rejoin
  - next-collab
  - attack
  - post-attack
  - imaging schedule
  - path deviation
- Planner runtime smoke:
  - prior and next-collab watch paths use canonical trigger files.
  - old prior/next-collab implementation files are not watched or reloaded.
  - bindings use canonical trigger modules.
- Stale import search:
  - no direct imports of `modules.mission_planning.pipelines.prior_mission_pipeline_impl`
  - no direct imports of `modules.mission_planning.pipelines.next_collab_replan_pipeline_impl`
- `git diff --check` passed for the touched files. Only line-ending warnings were reported.

## Remaining Phase 3 Work

- `current/general remaining hybrid` still needs a `replanning/triggers/remaining_hybrid/` organization plan.
- `recon_specialized_pipeline.py` and `reexecute_first_mission_hybrid.py` still need clearer trigger placement.
- Support modules still intentionally remain under `pipelines/`:
  - `next_collab_path_builder.py`
  - `mission_path_trim.py`
  - current/general hybrid helpers
  - recon/reexecute helpers

Do not delete these support modules until the allow-list and reachability checks are expanded.
