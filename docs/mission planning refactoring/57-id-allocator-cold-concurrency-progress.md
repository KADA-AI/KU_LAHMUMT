# ID Allocator Cold/Concurrency Progress

## Scope

This checkpoint freezes the current ID allocator behavior for memory cold-reset recovery and concurrent reserve calls.

## Added

- `smoke_id_allocator_cold_concurrency.py`

## Contract Captured

- Linear ID reservations continue from the on-disk `id_tracker.json` high-water mark after in-memory allocator state is reset.
- Existing on-disk high-water values win over fresh memory state for:
  - `missionPlanID`
  - `individualMissionPackage`
  - `individualMission`
  - per-aircraft `pathID`
- `pathID` string keys from JSON are normalized and continue correctly after cold reset.
- `waypoint` remains volatile and is not persisted into `id_tracker.json`; after memory reset it starts again from the waypoint base unless waypoint usage scanning/records seed it.
- The store lock sidecar file remains `id_tracker.json.lock`.
- `run.py` launch reset semantics are captured without importing the full GUI entrypoint:
  - `_force_reset_id_files()` writes legacy `id_tracker.json`, `id_tracker_0202.json`, and `_id_counters.json` with hardened `*_000` seeds.
  - `_force_reset_id_files()` clears active DB `path_usage.json` and `waypoint_usage.json` but does not write active DB `id_tracker.json`.
  - `_reset_id_counters()` writes legacy `id_tracker.json`, resets allocator `_state`, resets volatile waypoint state to `49`, writes `id_tracker_0202.json`, and writes `_id_counters.json` with `*_001` seeds.
- Concurrent `reserve_mission_plan_ids(...)` calls return contiguous per-call blocks whose union exactly matches a sequential range from `BASE`.
- Concurrent `reserve_path_id_blocks(...)` calls preserve per-aircraft contiguous, duplicate-free ranges.
- Concurrent `reserve_replan_id_bundle(...)` calls preserve atomic ranges across path IDs, IMP IDs, and individual mission IDs.
- Concurrent waypoint block reservation remains duplicate-free and contiguous for both `reserve_waypoint_block(...)` and `reserve_waypoint_blocks(...)`.
- Cross-process workers sharing one temp `_STORE` preserve duplicate-free, gap-free ranges through the `id_tracker.json.lock` file.

## Boundary

This smoke does not use the active mission DB. It monkeypatches the allocator store to a temporary directory and disables metric/path/waypoint side-effect hooks during the test, then restores all allocator globals. For `run.py`, it AST-loads only reset-related constants/functions into a synthetic namespace so the GUI entrypoint is not imported.

## Why This Is Safe

No runtime code changed. The smoke exercises public allocator APIs against an isolated temp store, so it does not mutate real `DSS_Internal/id_tracker.json`, path usage, waypoint usage, or generated mission artifacts.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_id_allocator_cold_concurrency.py"
python "docs\mission planning refactoring\smoke_id_allocator_cold_concurrency.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
git diff --check -- "docs/mission planning refactoring"
```

Expected result:

```text
id allocator cold-reset/concurrent-reserve parity smoke ok
mission planning refactor import-contract smoke ok
```

## Next

Next incomplete TODO: `runtime artifact/resource path manifest`.
