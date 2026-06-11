# 0902 Pre-Deferred Order Progress

## Scope

This checkpoint freezes the order used when a 0902 request arrives while initial planning is still running. Timing and terrain warmup markers must be scheduled before the request is placed into the deferred queue.

## Added

- `smoke_0902_predefer_order.py`

## Contract Captured

- `_handle_replan_received_impl()` calls `_start_replan_timing(ctx, payload)` before `_schedule_replan_terrain_warmup(ctx, payload)`.
- Terrain warmup scheduling happens before `delay_ms = self._replan_delay_ms_for_payload(payload)`.
- Delay calculation happens before the `_initplan_running` deferred branch.
- When `_initplan_running` is true, the handler calls `_queue_deferred_replan_request(ctx, delay_ms=delay_ms)` after timing/terrain work and does not schedule the pipeline immediately.
- Replay capture still happens after deferred queueing.
- When `_initplan_running` is false, the same timing/terrain/delay order is preserved and the handler calls `_schedule_replan_pipeline(delay_ms=delay_ms)`.

## Why This Is Safe

No runtime code changed. The smoke imports `mission_planning_gui.py` but does not initialize the Qt window. It uses `MainWindow.__new__` and stubs parse/timing/terrain/delay/queue/schedule/capture methods on the test instance.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_0902_predefer_order.py"
python "docs\mission planning refactoring\smoke_0902_predefer_order.py"
```

Result:

```text
0902 pre-deferred timing/terrain order smoke ok
```

## Next

Next incomplete TODO: `captured 0902 replay와 store-backed detail fixture 작성`.
