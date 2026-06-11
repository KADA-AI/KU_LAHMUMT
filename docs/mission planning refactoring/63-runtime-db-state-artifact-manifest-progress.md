# Runtime DB State Artifact Manifest Progress

## Scope

This checkpoint freezes runtime DB state artifacts that are used as planning/replanning input state rather than logs.

## Added

- `smoke_runtime_db_state_artifacts.py`

## Contract Captured

- `VehicleStatus/status.json`:
  - is written under the active DB root.
  - stores `updated_at`, sorted `available`, `manned`, and `unmanned`.
  - is read by enhanced planning through the same active DB path.
- `DSS_Internal/targetInfo.json`:
  - stores a top-level `targetList` map.
  - target entries preserve `targetID`, `targetType`, `watcherID` or watcher fallback, `coordinate`, destroyed/used/ignored/in-frame flags, threat, timestamps, and raw entry access.
  - attack/prior replanning readers reject non-map `targetList`.
- `DSS_Internal/sweep_progress.json`:
  - stores `timestamp_ms` and an `entries` list.
  - entries are keyed by numeric `path_id` for mission path trimming and post-attack replanning.
  - progress point, explicit buffer, and elapsed-time buffer semantics are preserved.
- `DSS_Internal/coverage_progress.json`:
  - stores `timestamp_ms`, `mission_plan_id`, `plan_coverage`, `input_coverage`, `package_coverage`, and `missions`.
  - post-attack replanning loads the object as-is.
- `DSS_Internal/mission_progress/*.json`:
  - prior replanning selects the newest JSON by mtime and reads `missionPlanID`.

## Boundary

This does not retest 0401 snapshot/log files, replan sidecars, ID allocator state, attack/prior state modules, or HTML/PNG artifacts. Those are covered by adjacent roadmap items.

## Why This Is Safe

No runtime code changed. The smoke monkeypatches `modules.common.db_paths` to an isolated temporary DB root, writes only inside that root, and removes it afterward.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_runtime_db_state_artifacts.py"
python "docs\mission planning refactoring\smoke_runtime_db_state_artifacts.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
git diff --check -- "docs/mission planning refactoring"
```

Expected result:

```text
runtime DB state artifact manifest smoke ok
mission planning refactor import-contract smoke ok
```

## Next

Next incomplete TODO: `HTML/PNG output manifest`.
