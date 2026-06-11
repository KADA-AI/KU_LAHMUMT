# Root Manual Bucket Finalization Progress

## Scope

This checkpoint removes the remaining manual/operator folders from the
`modules/mission_planning` root without changing runtime mission-planning
behavior.

## Changed

- Moved `logic_test/` to `manual/logic_test/`.
- Removed the root `MissionVisualizer/` wrapper package.
- Kept canonical visualizer code at `manual/MissionVisualizer/main_visualizer.py`.
- Kept `MissionPlanner/tools/main_visualizer.py` as the direct tool wrapper.
- Added package-level aliases in `modules/mission_planning/__init__.py` for:
  - `modules.mission_planning.MissionVisualizer`
  - `modules.mission_planning.logic_test`

## Preserved Behavior

- Old `modules.mission_planning.MissionVisualizer.*` imports resolve to the
  canonical manual visualizer modules.
- Old `modules.mission_planning.logic_test.*` imports resolve to
  `modules.mission_planning.manual.logic_test.*`.
- `manual/logic_test/division_test/output` generated JSON fixture candidates
  are kept intact.
- `bkup`, `Logs`, project-root settings/resource folders, and
  `modules_bkup` are outside this change.

## Verification

```powershell
python -m py_compile modules\mission_planning\__init__.py
python -m py_compile modules\mission_planning\manual\logic_test\division_test\main.py
python -m py_compile modules\mission_planning\manual\logic_test\dubins_test\dubins_turn_link_logic.py
python "docs\mission planning refactoring\smoke_root_surface_inventory.py"
python "docs\mission planning refactoring\smoke_manual_operator_entrypoints.py"
python "docs\mission planning refactoring\smoke_manual_workflow_owner_decisions.py"
```

Expected result:

```text
mission planning root surface inventory smoke ok
manual/operator entrypoint inventory smoke ok
manual workflow owner decision smoke ok
```
