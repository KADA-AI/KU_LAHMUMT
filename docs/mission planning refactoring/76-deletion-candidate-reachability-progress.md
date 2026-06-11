# Deletion Candidate Reachability Progress

## Scope

This checkpoint verifies reachability for deletion/archive candidates without deleting, moving, or importing side-effect-prone tool modules.

## Added

- `smoke_deletion_candidate_reachability.py`

## Contract Captured

- Root compatibility wrappers and `legacy/wrappers` still exist and remain wrapper surfaces.
- Legacy app/test/compat package buckets still have reachable wrapper or archive references, so they are not deletion-approved.
- `MissionVisualizer` old import paths are package-level aliases for `manual/MissionVisualizer`; `MissionPlanner/tools/main_visualizer.py` remains a compatibility wrapper.
- `manual/logic_test/division_test/output` and `legacy/tests/division_test/output` JSON counts are fixed as generated/golden candidates until fixture policy is decided.
- Active Nadir BF and Dubins helpers remain reachable from the 0303 builder.
- The portable mission bundle remains reachable from `d0304.py`, canonical `manual/lah_rl_planner_gui.py`, root `lah_rl_planner_gui.py` compatibility wrapper, and the public GUI.
- Prototype UAV-pattern scripts, `d0304 copy.py`, and TensorBoard training artifacts are recorded as candidates requiring later owner/output/backup decisions.
- No tracked `__pycache__`/`.pyc` files are present.

## Boundary

This smoke does not approve any deletion. It reads files, checks source markers, counts JSON outputs, and uses `git ls-files` only to identify tracked candidate artifacts.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_deletion_candidate_reachability.py"
python "docs\mission planning refactoring\smoke_deletion_candidate_reachability.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
git diff --check -- "docs/mission planning refactoring"
```

Expected result:

```text
deletion candidate reachability smoke ok
mission planning refactor import-contract smoke ok
```

## Next

Next incomplete TODO: confirm deletion candidate owner/manual workflow.
