# Backup-Style File Deletion Decision

## Scope

This checkpoint decides whether backup-style mission-planning files and training artifacts should be deleted.

## Decision

backup-hold and artifact-hold.

Do not delete backup-style files or tracked training artifacts in this refactor phase.

## Held Candidates

| Candidate | Policy | Reason |
| --- | --- | --- |
| `MissionPlanner/data_def/d0304 copy.py` | backup-hold | backup-style artifact-builder file; deletion needs artifact-builder owner approval. |
| `MissionPlanner/AnS/Training/TensorBoard Logs*` event files | artifact-hold | tracked training/evaluation artifacts; deletion needs model/training owner approval. |

## Rationale

- Existing reachability checks classify these files as deletion candidates, not approved deletion targets.
- No active runtime reference requires these backup/training paths, but removing tracked historical artifacts can still affect reproducibility, audit, or manual comparison workflows.
- The current refactor target is functional module organization, not training artifact cleanup.

## Future Delete Gate

A later cleanup batch may delete or move these files only after owner approval, artifact manifest update, active reference scan, and green import/artifact-builder smokes.

## Boundary

No runtime implementation changed. No backup-style or training artifact files were rewritten or deleted by this checkpoint.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_backup_style_file_policy.py"
python "docs\mission planning refactoring\smoke_backup_style_file_policy.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
git diff --check -- "docs/mission planning refactoring"
```

Expected result:

```text
backup-style file policy smoke ok
mission planning refactor import-contract smoke ok
```

## Next

Next incomplete TODO: PR preflight with `smoke_import_contract.py --require-git-tracked`.
