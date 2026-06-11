# Deprecated Import Policy Decision

## Decision

Documentation-only deprecated import policy for this refactor phase.

Do not emit runtime deprecation warnings, process-console logs, or logging records from compatibility wrappers during import.

## Rationale

- The bootstrap import-order contract requires `KU_ROLE=mission` and mission console/file logging setup to happen before logging/runtime side-effect imports.
- Compatibility wrappers are imported from launchers, manual tools, GUI warmup paths, and legacy entrypoints; adding warnings or logs would change import behavior and may create noisy console/file output.
- Existing smoke coverage already documents and verifies supported old paths, wrapper identity, and stale moved implementation import bans.
- Deprecation timing is a separate roadmap item. Until that period is decided, old paths remain supported compatibility paths, not noisy deprecated paths.

## Boundary

No runtime implementation changed. The decision only records that deprecated import paths are documented in the refactoring docs and support matrix. Runtime warnings/logging can be reconsidered only after a deprecation period and import-order-safe logging mechanism are agreed.

## Smoke

`smoke_deprecated_import_policy_contract.py` verifies:

- this decision remains documentation-only.
- compatibility wrapper files do not import `warnings`, stdlib `logging`, process-console, or mission runtime logging helpers just to report deprecation.
- compatibility wrapper files do not make top-level warning/logging/process-console calls.
- the policy remains tied to the existing bootstrap import-order and root compatibility strategy docs.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_deprecated_import_policy_contract.py"
python "docs\mission planning refactoring\smoke_deprecated_import_policy_contract.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
git diff --check -- "docs/mission planning refactoring"
```

Expected result:

```text
deprecated import policy smoke ok
mission planning refactor import-contract smoke ok
```

## Next

Next incomplete TODO: keep `mission_planning_gui.py` as the public launcher and prepare an internal import target handoff structure.
