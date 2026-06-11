# ID/State JSON Artifact Manifest Progress

## Scope

This checkpoint freezes ID and runtime state JSON artifacts before further movement of allocator and state modules.

## Added

- `smoke_id_state_json_artifacts.py`

## Contract Captured

- Legacy ID files remain under `modules/mission_planning/MissionPlanner/data_def/`:
  - `id_tracker.json`
  - `id_tracker_0202.json`
  - `_id_counters.json`
- The legacy AnS counter file remains `modules/mission_planning/MissionPlanner/AnS/_id_counters.json`.
- `run.py` still resets legacy ID files and clears active `DSS_Internal/path_usage.json` and `DSS_Internal/waypoint_usage.json`.
- Runtime active ID allocator artifacts remain under active DB `DSS_Internal/`:
  - `id_tracker.json`
  - `id_tracker.json.lock`
  - `path_usage.json`
  - `waypoint_usage.json`
- `id_tracker.json` stores high-water marks for `missionPlanID`, `individualMissionPackage`, `individualMission`, and `pathID`.
- `path_usage.json` stores top-level `aircraft` and `updated_at`.
- `waypoint_usage.json` stores `last_waypoint_id`, `updated_at`, and `flightPathSignature`.
- Runtime state artifacts remain under active DB `DSS_Internal/`:
  - `attack_assignment_state.json`
  - `attack_tracking_state.json`
  - `prior_tracking_state.json`
- `attack_assignment_state.json` preserves:
  - `last_manned_aircraft_id`
  - `used_manned_by_input_package`
  - `pending_manned_by_plan_id`
  - `deferred_attack_targets_by_input_package`
- `attack_tracking_state.json` and `prior_tracking_state.json` preserve the top-level `assignments` map and assignment lifecycle fields.
- Atomic tmp files for attack/prior tracking state and ID usage sidecars are not left behind after writes.

## Boundary

This smoke does not run mission planning, delivery, monitoring, dashboard reset, or nFusion. It calls allocator/state APIs directly against a temporary DB root.

## Why This Is Safe

No runtime code changed. The smoke monkeypatches `modules.common.db_paths` to an isolated temporary DB root, writes only inside that root, then removes it.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_id_state_json_artifacts.py"
python "docs\mission planning refactoring\smoke_id_state_json_artifacts.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
git diff --check -- "docs/mission planning refactoring"
```

Expected result:

```text
ID/state JSON artifact manifest smoke ok
mission planning refactor import-contract smoke ok
```

## Next

Next incomplete TODO: `runtime DB state artifact manifest`.
