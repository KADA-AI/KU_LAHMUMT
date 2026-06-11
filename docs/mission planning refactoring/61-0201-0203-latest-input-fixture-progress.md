# 0201/0203 Latest Input Fixture Progress

## Scope

This checkpoint freezes the current latest-input behavior for 0201 and 0203 before further message-handler and GUI-shell refactoring.

## Added

- `smoke_0201_0203_latest_input_fixture.py`

## Contract Captured

- `0202` remains outside latest-input cache/materialization handling.
- 0201 and 0203 payloads with only package IDs and empty core lists are cached for latest ID tracking but are not materialized to JSON files.
- 0201 and 0203 materialization requires at least one non-empty core list, not every core list.
- 0201 payloads with non-empty `inputMissionList` or `availableAircraftList` materialize under `InputMissionPlan/<id>.json`.
- 0203 payloads with non-empty `takeOverInfoList`, `flightAreaList`, or `handOverInfoList` materialize under `MissionReferenceInfo/<id>.json`.
- 0203 must not be accidentally written under `InputMissionPlan`.
- `Source`/`source` extraction preserves the current truthy raw value and preserves whitespace in GUI log text.
- When both `Source` and `source` exist, truthy `Source` still wins.
- GUI latest-input handling keeps the current cache/log/warmup/id-tab update split:
  - `_handle_latest_input_payload()` updates the latest-input cache, refreshes the banner, logs a newly seen ID, schedules warmup, and submits id-tab updates.
  - `_prime_latest_input_file()` performs cache-payload materialization.
- The banner resolves existing cached files as `InputMissionPlan/<id>.json` and `MissionReferenceInfo/<id>.json`.

## Boundary

This smoke does not instantiate the Qt window, register nFusion listeners, run initial planning, or execute planner warmup. It creates a `MainWindow.__new__()` test instance with only the fields and callbacks needed by the latest-input methods.

## Why This Is Safe

No runtime code changed. The smoke uses a temporary DB root by monkeypatching `modules.common.db_paths.get_active_db_root`, restores the patch, resets latest-input state, and removes the temporary directory.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_0201_0203_latest_input_fixture.py"
python "docs\mission planning refactoring\smoke_0201_0203_latest_input_fixture.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
git diff --check -- "docs/mission planning refactoring"
```

Expected result:

```text
0201/0203 latest input fixture smoke ok
mission planning refactor import-contract smoke ok
```

## Next

Next incomplete TODO: `ID/state JSON artifact manifest`.
