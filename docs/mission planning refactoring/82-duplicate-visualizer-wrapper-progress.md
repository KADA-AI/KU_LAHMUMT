# Duplicate Visualizer Wrapper Progress

## Scope

This checkpoint removes one active duplicate implementation and then lets the
operator-facing `MissionVisualizer` import path live through package-level
aliases.

## Changed

- `modules/mission_planning/manual/MissionVisualizer/main_visualizer.py` is now
  the canonical implementation.
- The old `modules.mission_planning.MissionVisualizer.*` import path is now
  handled by `modules/mission_planning/__init__.py`.
- `modules/mission_planning/MissionPlanner/tools/main_visualizer.py` is now a
  thin compatibility wrapper.
- Manual/operator and owner-decision smokes now lock the canonical-plus-wrapper
  contract instead of requiring duplicate file hashes.

## Preserved Behavior

- Imports of `MissionPlanVisualizer` and `main` from the old tool path still
  resolve.
- The old `MissionVisualizer` package import path is preserved as a package
  alias.

## Boundary

No `Logs`, project root, `resource`, `settings`, or backup folders were touched.
`logic_test/` now lives under `manual/logic_test/`; old imports are preserved by
the same package-level alias mechanism.

## Verification

```powershell
python -m py_compile "modules\mission_planning\__init__.py"
python -m py_compile "modules\mission_planning\MissionPlanner\tools\main_visualizer.py"
python -m py_compile "docs\mission planning refactoring\smoke_manual_operator_entrypoints.py"
python -m py_compile "docs\mission planning refactoring\smoke_manual_workflow_owner_decisions.py"
python "docs\mission planning refactoring\smoke_manual_operator_entrypoints.py"
python "docs\mission planning refactoring\smoke_manual_workflow_owner_decisions.py"
```

Expected result:

```text
manual/operator entrypoint inventory smoke ok
manual workflow owner decision smoke ok
```
