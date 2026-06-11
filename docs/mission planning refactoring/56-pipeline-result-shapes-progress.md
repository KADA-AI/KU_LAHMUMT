# Pipeline Result Shapes Progress

## Scope

This checkpoint freezes the current mission-planning pipeline result shapes that `mission_planning_gui.py` consumes.

## Added

- `smoke_pipeline_result_shapes.py`

## Contract Captured

- Dataclass field order and default/factory markers for:
  - `PriorMissionPipelineResult`
  - `PriorPostRejoinPipelineResult`
  - `PostAttackRejoinPipelineResult`
  - `NextCollabPipelineResult`
  - `ImagingSchedulePipelineResult`
  - `PathDeviationPipelineResult`
  - `remaining_hybrid.current_replan.CurrentRemainingHybridResult`
  - `RemainingHybridResult`
  - `CurrentRemainingHybridRequest`
  - `CurrentRemainingHybridGeometry`
  - `CurrentRemainingHybridRuntimeResult`
  - `remaining_hybrid.current.CurrentRemainingHybridResult`
  - `GenericFlightPathSkipResult`
- Compatibility wrapper class identity for import paths that currently export result classes.
- GUI consumption assumptions for common result fields: `plan_ids`, `option_names`, `plan_meta_map`, `generated_imp_ids`, `generated_path_ids`, and `log_path`.
- GUI consumption assumptions for trigger-specific fields:
  - next-collab `new_input_package_id`
  - imaging/quality `replaced_waypoint_id`, `new_waypoint_id`
  - path-deviation `removed_waypoint_id`, `inserted_waypoint_id`
  - rejoin `summary` and `status`
- Attack dict shape source markers:
  - top-level log/result/timing keys
  - nested `result.missionUpdates`
  - failure `failure_code` and `failure_notice`
  - target/weapon/aircraft keys consumed by GUI follow-up logic
- Attack-exclusion dict shape source markers:
  - top-level `logMessages`, `timingMs`, `result`
  - success `result.missionPlanID`, `planPath`, `missionUpdates`
  - failure `result.error`, including `no_updates` and validation errors

## Boundary

This smoke does not execute any mission pipeline. It imports modules, snapshots dataclass metadata, checks wrapper identity, validates GUI-style fixture consumption, and inspects attack source markers. This avoids DB writes, ID counter changes, and external runtime state mutations.

## Why This Is Safe

No runtime code changed. The smoke is read-only and constructor/inspection based. It is intended to catch accidental field renames, wrapper identity splits, and attack dict nesting changes before large folder moves continue.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_pipeline_result_shapes.py"
python "docs\mission planning refactoring\smoke_pipeline_result_shapes.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
git diff --check -- "docs/mission planning refactoring"
```

Expected result:

```text
pipeline result shape smoke ok (13 dataclasses + attack dicts)
mission planning refactor import-contract smoke ok
```

## Next

Next incomplete TODO: `ID allocator cold-reset/concurrent-reserve parity test`.
