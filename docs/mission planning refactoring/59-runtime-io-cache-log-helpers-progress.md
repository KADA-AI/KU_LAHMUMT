# Runtime I/O/Cache/Log Helpers Progress

## Scope

This checkpoint freezes the current behavior of the small runtime support helpers used by mission planning:

- `json_io`
- `latest_input_cache`
- `mission_plan_file_logger`
- `mission_planning_pipeline_logging`

## Added

- `smoke_runtime_io_cache_log_helpers.py`

## Contract Captured

- `json_io`:
  - compact JSON serialization preserves deterministic sorted-key output when requested.
  - FlightPath payloads with waypoint lists normalize legacy `Source` to `source`.
  - Existing lower-case `source` wins if both `Source` and `source` are present.
  - unchanged writes are skipped when `skip_if_unchanged=True`.
  - atomic write temp files use the `.json.tmp` suffix and are removed after replace.
  - `write_json_bytes()` preserves byte payloads and skips unchanged bytes.
  - batch write results report `written` and `skipped` flags and emit log messages.
- `latest_input_cache`:
  - `reset_latest_inputs()` clears both 0201 and 0203 snapshots.
  - 0201 reads `inputMissionPackageID`; 0203 reads `missionReferencePackageID`.
  - payload keys are resolved across exact, lower-case, and upper-case forms.
  - unsupported message IDs are ignored for updates and return `None` for package lookup.
  - `get_latest_snapshot()` returns a top-level payload copy and raises `KeyError` for unsupported IDs.
  - `resolve_path_from_cache()` resolves `<cached package id>.json` only when the file exists.
- `mission_plan_file_logger`:
  - `MissionPlanFileLogger.start_run()` writes under `db_paths.ensure_db_payload("DSS_Internal")`.
  - invalid `plan_ids` are ignored while valid plan IDs are retained.
  - `replanLevel` and `replan_level` are normalized to integer `replan_level`.
  - finalized run logs use `missionPlan_<planID>.json`.
  - duplicate plan logs use tokenized `missionPlan_<planID>_<token>.json` paths.
  - runs without plan IDs use tokenized `missionPlan_pending_<token>.json` paths.
  - blocked logs record `status=blocked`, stop reason, summary, and clear the active run.
- `mission_planning_pipeline_logging`:
  - checkpoint phase names and outcome inference stay stable.
  - `PipelinePhaseTimer` records phase timings and total timing.
  - `emit_pipeline_event()` and `emit_replan_checkpoint()` emit `[REPLAN][EVENT]` JSON through `process_console.emit_process_log`.
  - `PipelineLogManager` opens, appends, closes, and dispatches events to a log tab object.

## Boundary

This smoke does not duplicate wrapper identity checks from `smoke_import_contract.py`. It imports the canonical runtime helper modules and validates behavior.

It intentionally does not cover `json_io` UAV speed-weight/ETA mutation because that path depends on runtime settings and ETA helpers. That can be covered in a later mission-artifact behavior smoke if needed.

It also avoids active mission DB side effects:

- `db_paths.ensure_db_payload()` is monkeypatched to a temporary directory during file logger checks.
- `process_console.emit_process_log()` is monkeypatched to an in-memory capture list during pipeline event checks.
- `REPLAN_RUNTIME_ARTIFACT_MODE` is set only inside a temporary env patch while writing file logger artifacts.

## Why This Is Safe

No runtime code changed. The smoke writes only under a `TemporaryDirectory` and restores all monkeypatches/env overrides after each check.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_runtime_io_cache_log_helpers.py"
python "docs\mission planning refactoring\smoke_runtime_io_cache_log_helpers.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
git diff --check -- "docs/mission planning refactoring"
```

Expected result:

```text
runtime I/O/cache/log helper smoke ok
mission planning refactor import-contract smoke ok
```

## Next

Next incomplete TODO: `0101 parsing allow-list fixture`.
