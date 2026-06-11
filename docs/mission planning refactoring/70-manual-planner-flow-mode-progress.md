# Manual Planner Flow Mode Progress

## Scope

This checkpoint freezes the manual planner flow-mode contracts for next-area and next-collab division tools without launching Qt windows.

## Added

- `smoke_manual_planner_flow_modes.py`

## Contract Captured

- `next_area_mode`:
  - flow environment key remains `MISSION_NEXT_AREA_FLOW_MODE`.
  - launcher default remains `initial`.
  - tab UI exposes `Initial` and `Replan`.
  - planner window normalizes any non-`replan` value back to `initial`.
  - `use_replan_flow` controls assignment behavior and stage-log flow text.
  - runtime headless line runner still imports the next-area config/window helpers.
- `planners/next_collab_division`:
  - launcher default remains `DIVISION_TEST_FLOW_MODE=initial`.
  - `division_planner_gui.py` remains the public Qt wrapper around `_planner_window.DivisionPlannerWindow`.
  - planner window normalizes any non-`replan` value back to `initial`.
  - `use_replan_flow` controls assignment behavior and stage-log flow text.
  - mid-line no-split mode remains a production division planner branch.
  - runtime headless division runner still imports `_planner_window` helpers.
- `logic_test/division_test`:
  - mirror launcher default remains `DIVISION_TEST_FLOW_MODE=initial`.
  - mirror GUI keeps the same `_flow_mode` / `_is_replan_flow` behavior.

## Boundary

This smoke does not launch PyQt, instantiate planner windows, or execute planning. It is a source-level contract for manual flow-mode behavior before folder moves or wrapper conversion.

## Why This Is Safe

No runtime code changed. The smoke reads source files only and checks environment keys, default values, public wrapper paths, and runtime headless import dependencies.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_manual_planner_flow_modes.py"
python "docs\mission planning refactoring\smoke_manual_planner_flow_modes.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
git diff --check -- "docs/mission planning refactoring"
```

Expected result:

```text
manual planner flow-mode smoke ok
mission planning refactor import-contract smoke ok
```

## Next

Next incomplete TODO: `current-remaining hybrid failure fallback/pathID mapping fixture`.
