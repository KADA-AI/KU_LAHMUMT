# 2026-05-20 Area Remaining Replan TODO

Purpose: prevent already-covered area/line work from being revived after repeated attack, post-attack return, path deviation, or other additional replans.

## Current Findings

- The monitoring area tab writes the current remaining-work snapshot to `DSS_Internal/mission_area_replan/mission_area_snapshot_<MissionPlanID>.json`.
- The planner consumes that snapshot when rebuilding a current collaborative mission.
- The first replan is usually correct because the source `MissionPlanID` and snapshot file match.
- Later replans create a new `MissionPlanID`, but the remaining snapshot was not always carried forward.
- When a planner-side snapshot lookup missed, one path returned the original `InputMissionPlan` current mission, which could revive already-covered area.
- Path deviation and post-attack flows also generate new mission plans, so they must preserve the snapshot lineage.

## TODO

- [x] Add a store-level monotonic guard so remaining area for the same input mission cannot grow when saving a snapshot.
- [x] Add a helper to find the latest compatible snapshot entry for an input mission when the exact plan snapshot is missing.
- [x] Stop collaborative remaining replan from silently falling back to the full original input mission when no snapshot entry exists.
- [x] Carry remaining snapshots from source plan to generated plan in attack replan.
- [x] Carry remaining snapshots from source plan to generated plan in path-deviation replan.
- [x] Carry remaining snapshots from source plan to generated plan in post-attack rejoin.
- [x] Carry remaining snapshots from source plan to generated plan in prior post-rejoin.
- [ ] Runtime validation: run an attack -> return -> path-deviation sequence and confirm `mission_area_snapshot_<newPlanID>.json` is created for each generated plan.
- [ ] Runtime validation: confirm snapshot audit logs do not show `missing_snapshot` before a collaborative remaining replan.

## Changes Made

- `modules/common/mission_area_replan_store.py`
  - Added per-input monotonic merge on `save_snapshot()`.
  - Added latest-compatible snapshot lookup by `inputMissionID`.
  - Added snapshot carry-forward helper.
  - Added lightweight audit JSONL logging under `DSS_Internal/mission_area_replan/mission_area_snapshot_audit.jsonl`.
- `modules/mission_planning/pipelines/prior_mission_pipeline_impl.py`
  - Collaborative remaining mission construction now uses exact snapshot first, then latest compatible snapshot.
  - If no snapshot entry exists, it now returns unavailable instead of using the full original mission.
  - Area remaining detail no longer convex-hulls disjoint polygons into a larger revived area.
- `modules/mission_planning/pipelines/attack_plan_pipeline.py`
  - Carries the source remaining snapshot to the generated attack plan.
- `modules/mission_planning/pipelines/path_deviation_replan_pipeline_impl.py`
  - Carries the source remaining snapshot to the generated path-deviation plan.
- `modules/mission_planning/pipelines/post_attack_rejoin_pipeline.py`
  - Carries the current remaining snapshot to the generated post-attack plan.
- `modules/mission_planning/pipelines/prior_mission_pipeline_impl.py`
  - Carries the current remaining snapshot to generated prior/post-rejoin plans.

## Verification

- [x] `python -m py_compile` passed for the touched modules:
  - `modules/common/mission_area_replan_store.py`
  - `modules/mission_planning/mission_planning_gui.py`
  - `modules/mission_planning/pipelines/prior_mission_pipeline_impl.py`
  - `modules/mission_planning/pipelines/attack_plan_pipeline.py`
  - `modules/mission_planning/pipelines/path_deviation_replan_pipeline_impl.py`
  - `modules/mission_planning/pipelines/post_attack_rejoin_pipeline.py`
