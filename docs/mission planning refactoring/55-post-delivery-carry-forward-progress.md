# Post Delivery Carry-Forward Progress

## Scope

This checkpoint freezes the current post-delivery waypoint mark and mission-area snapshot carry-forward behavior.

## Added

- `smoke_post_delivery_carry_forward.py`

## Contract Captured

- Waypoint mark normalization accepts positive `max_waypoint_id` or `variants`, clamps negative variants to `0`, and ignores empty payloads.
- Waypoint mark merge keeps max `max_waypoint_id`, sums variants, and prefers the incoming reason.
- Snapshot carry-forward normalization accepts single-item payloads or `items`, supports snake_case and camelCase plan IDs, clamps negative variant to `0`, and ignores invalid items.
- Snapshot carry-forward merge deduplicates by `(sourceMissionPlanID, targetMissionPlanID, reason)`, fills missing variants with `0`, and prefers the incoming batch reason.
- Pending `_schedule_plan_delivery(...)` merge carries and merges waypoint/snapshot post-delivery payloads.
- `_start_push_sequence()` schedules post-delivery payloads after `0301` and `0305 status=2` success, before option-mode `mode_ready` flush.
- `0301` failure and `0305` completion failure do not schedule post-delivery waypoint/snapshot work.
- Waypoint worker calls `mark_waypoint_files_written(max_waypoint_id)` and records `post_delivery_waypoint_mark`.
- Snapshot worker calls `mission_area_replan_store.carry_forward_snapshot(...)` for each item and records carried/skipped counts.
- Current worker thread names are `PostDelivery-WaypointMark` and `PostDelivery-SnapshotCarry`.
- Invalid waypoint/snapshot post-delivery payloads do not start worker threads.

## Boundary

This smoke does not snapshot pipeline result shapes. It only validates the post-delivery payload normalization, merge, scheduling, and worker-call contracts.

## Why This Is Safe

No runtime code changed. The smoke uses fake `push_message`, immediate fake threads, and monkeypatched waypoint/snapshot store calls, then restores all patches after execution.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_post_delivery_carry_forward.py"
python "docs\mission planning refactoring\smoke_post_delivery_carry_forward.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
git diff --check -- "docs/mission planning refactoring"
```

Result:

```text
post delivery carry-forward smoke ok
mission planning refactor import-contract smoke ok
```

## Next

Next incomplete TODO: `pipeline result shape snapshot`.
