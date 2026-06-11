# Remaining Hybrid Progress

Date: 2026-06-04, Asia/Seoul

## Completed

- Created `modules/mission_planning/replanning/triggers/remaining_hybrid/`.
- Moved current/general remaining hybrid implementations:
  - `current.py` from `pipelines/current_remaining_hybrid.py`
  - `current_replan.py` from `pipelines/current_remaining_hybrid_replan.py`
  - `general.py` from `pipelines/general_remaining_hybrid_replan.py`
- Restored the old `pipelines/*remaining*hybrid*.py` paths as compatibility wrappers.
- Updated `mission_planning_gui.py` to import current/general remaining hybrid helpers from the canonical trigger package.
- Updated `mission_control/planner_runtime.py` watch and reload paths to use canonical remaining hybrid modules.
- Updated `smoke_import_contract.py` so old remaining-hybrid implementation imports are forbidden and wrapper identity is verified.

## Verification Passed

- `python "docs\mission planning refactoring\smoke_import_contract.py"`
- `python -m compileall` for remaining_hybrid package, compatibility wrappers, planner runtime, GUI, and smoke script.
- Remaining hybrid wrapper identity smoke:
  - `CurrentRemainingHybridRequest`
  - `build_current_remaining_hybrid`
  - `merge_current_remaining_hybrid`
  - `CurrentRemainingHybridResult`
  - `prepare_current_remaining_hybrid_replacements`
  - `RemainingHybridResult`
  - `apply_remaining_hybrid_replan`
- Stale import search found no direct imports of:
  - `modules.mission_planning.pipelines.current_remaining_hybrid`
  - `modules.mission_planning.pipelines.current_remaining_hybrid_replan`
  - `modules.mission_planning.pipelines.general_remaining_hybrid_replan`

## Updated Caution

- `reexecute_first_mission_hybrid.py` and `recon_specialized_pipeline.py` were moved after this note; see `13-recon-reexecute-progress.md`.
- Support modules such as `mission_path_trim.py` and `next_collab_path_builder.py` are still active and should not be deleted.
