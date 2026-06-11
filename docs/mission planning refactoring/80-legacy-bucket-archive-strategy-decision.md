# Legacy Bucket Archive Strategy Decision

## Scope

This checkpoint decides whether the `modules/mission_planning/legacy` bucket should be deleted or archived.

## Decision

archive-hold.

Do not delete or relocate the `legacy` bucket in this refactor phase.

## Archive Buckets

| Bucket | Policy | Reason |
| --- | --- | --- |
| `legacy/wrappers` | keep-hold | compatibility wrappers still participate in import/wrapper smoke coverage. |
| `legacy/compat_packages` | archive-hold | old package compatibility paths need a separate owner decision before removal. |
| `legacy/apps` | archive-hold | manual visualizer/next-area app archive copies may still be operator/demo references. |
| `legacy/tests` | archive-hold | archived planner tests and output JSON are fixture candidates. |
| `legacy/logic_test` | archive-hold | archived logic-test entrypoints are tied to manual workflow checks. |
| `legacy/MissionPlanner_tools` | archive-hold | archived visualizer/planner tool copies may still be manual references. |
| `legacy/ui` | archive-hold | archived UI tab implementations are still used by compatibility wrappers. |
| `legacy/docs` and `legacy/static` | archive-hold | archived operator/static materials need a manifest before deletion. |

## Rationale

- Previous reachability and owner/manual workflow checks did not approve deletion.
- The generated-output policy keeps archived division output JSON as fixture candidates.
- The root wrapper deprecation clock is not active, so legacy compatibility wrappers should not be removed as a side effect.
- Moving the bucket to a different archive path would still change import paths for existing compatibility modules.

## Future Archive Gate

A later archive/delete batch needs a manifest of files being kept, external import checks, manual workflow owner approval, and green wrapper/import/manual entrypoint smokes.

## Boundary

No runtime implementation changed. This decision only records that the legacy bucket remains in place as an archive-hold bucket.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_legacy_bucket_archive_strategy.py"
python "docs\mission planning refactoring\smoke_legacy_bucket_archive_strategy.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
git diff --check -- "docs/mission planning refactoring"
```

Expected result:

```text
legacy bucket archive strategy smoke ok
mission planning refactor import-contract smoke ok
```

## Next

Next incomplete TODO: decide backup-style file deletion.
