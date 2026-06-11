# Root Wrapper Deprecation Period Decision

## Scope

This checkpoint records the compatibility period for old root mission-planning import paths.

## Decision

The deprecation clock is not active in this refactor phase.

Old root import paths remain supported compatibility paths. Import-only root
wrapper files have been replaced by package-level lazy aliases in
`modules/mission_planning/__init__.py`; direct operator launchers remain as
files.

If a later PR starts root-wrapper deprecation, the minimum deprecation period is 30 calendar days from the merged notice date. If the notice were merged on 2026-06-05, the earliest possible removal date would be 2026-07-05.

## Rationale

- Old root import paths are still documented as supported compatibility paths.
- Package-level aliases remove loose root files while preserving imports such as
  `modules.mission_planning.attack_plan_pipeline`.
- Active callers still use root mission-planning surfaces, including visualizer, launcher, dashboard wiring, and monitoring paths.
- The deprecated import policy is documentation-only, so runtime warnings/logging must not be added to start a deprecation clock.
- Removing import compatibility before external import cleanup would change public import behavior.

## Removal Gate

Final old import path removal needs a separate, explicit batch after all of these are true:

- a deprecation notice has been merged and aged at least 30 calendar days.
- external import scans show zero required root-wrapper callers.
- alias/import smokes are updated to the replacement public paths.
- `smoke_import_contract.py` and wrapper/deprecation policy smokes pass.
- the owner approves the removal batch.

## Boundary

Runtime compatibility changed from loose wrapper files to package-level aliases.
This decision records the minimum deprecation period and keeps the old import
surface intact.
The old import surface remains intact.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_root_wrapper_deprecation_period.py"
python "docs\mission planning refactoring\smoke_root_wrapper_deprecation_period.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
git diff --check -- "docs/mission planning refactoring"
```

Expected result:

```text
root wrapper deprecation period smoke ok
mission planning refactor import-contract smoke ok
```

## Next

Next incomplete TODO: decide the `legacy` bucket deletion or archive strategy.
