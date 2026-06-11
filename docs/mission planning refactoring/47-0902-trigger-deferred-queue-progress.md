# 0902 Trigger Deferred Queue Progress

## Scope

This checkpoint freezes trigger-specific 0902 delay consumption through immediate scheduling and deferred queue handling.

## Added

- `smoke_0902_trigger_deferred_queue.py`

## Contract Captured

- `_replan_delay_ms_for_payload(payload)` returns the current trigger-specific delay values:
  - `collabReexecuteInputRefresh`: runtime setting key with default `30` ms
  - `0402`: `0` ms
  - `attackClosedDestroyed`: `0` ms
  - `0401` + RTB trigger types: `55000` ms
  - unknown or missing detail: `100` ms
- Collab runtime override uses `int(float(raw))`.
- Negative collab runtime override clamps to `0`.
- Invalid collab runtime override currently raises `ValueError`; it does not fall back to default in the active imported GUI module.
- `_queue_deferred_replan_request(ctx, delay_ms=...)` normalizes negative/invalid delay values, deep-copies `ctx`, records `deferred_queued`, logs, and sorts by `due_at`.
- `_resume_deferred_replan_request()` waits while `_initplan_running` is true.
- Resume pops the earliest queued request, sets `_active_plan_context`, computes remaining delay from `due_at`, records `deferred_resumed`, logs, and schedules the pipeline with the remaining delay.
- `_handle_replan_received_impl()` computes `delay_ms = self._replan_delay_ms_for_payload(payload)` before both deferred queueing and immediate scheduling, and passes that same value to `_queue_deferred_replan_request(...)` or `_schedule_replan_pipeline(...)`.

## Why This Is Safe

No runtime code changed. The smoke imports `mission_planning_gui.py` but does not initialize the Qt window. It uses `MainWindow.__new__` and stubs logging/timing/scheduling methods on that test instance.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_0902_trigger_deferred_queue.py"
python "docs\mission planning refactoring\smoke_0902_trigger_deferred_queue.py"
```

Result:

```text
0902 trigger deferred queue smoke ok
```

## Next

Next incomplete TODO: `init planning 중 0902 terrain warmup/timing marker가 deferred queue보다 먼저 예약되는 순서 fixture 작성`.
