# Bootstrap Import-Order Contract Progress

## Scope

This checkpoint freezes the mission-planning bootstrap import-order contract. The goal is to keep `KU_ROLE=mission` setup and mission process console/file logging setup ahead of role-sensitive planner imports and logging/runtime side-effect imports.

## Added

- `smoke_bootstrap_import_order_contract.py`

## Contract Captured

- Importing `modules.mission_planning.app.bootstrap` does not import `modules.common.process_console` at module import time.
- `configure_mission_role(environ)` sets `KU_ROLE` to `mission` and returns `mission`.
- `configure_mission_process_console(environ)` lazy-imports `ensure_console` and `install_process_file_logging` inside the function.
- Mission process console setup order is `ensure_console(...)` before `install_process_file_logging("mission_planning")`.
- In `mission_planning_gui.py`, `configure_mission_role()` runs before `configure_mission_process_console()`.
- Before `configure_mission_process_console()`, `mission_planning_gui.py` may import only non-install process-console helpers: `emit_process_lifecycle_event`, `emit_process_log`.
- `configure_mission_role()` runs before `MissionPlanner`, bare `data_def`, bare `AnS`, and mission-generation engine imports.
- `configure_mission_process_console()` runs before runtime logging/debug/json I/O, Qt, and visualization-tab imports.
- Local modules directly imported before `configure_mission_role()` must not read/write `KU_ROLE` or import role-sensitive planner modules.
- Local modules directly imported before `configure_mission_process_console()` must not import logging/runtime side-effect modules or make top-level console/logging calls.
- Before `configure_mission_process_console()`, there are no top-level calls to `emit_process_log`, `emit_process_lifecycle_event`, `PipelineLogManager`, or `MissionPlanFileLogger`.

## Why This Is Safe

No runtime code changed. The smoke checks source order with AST and uses a fake `modules.common.process_console` module to validate `configure_mission_process_console()` call order without allocating a real console or installing real file logging.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_bootstrap_import_order_contract.py" "docs\mission planning refactoring\smoke_import_contract.py"
python "docs\mission planning refactoring\smoke_bootstrap_import_order_contract.py"
python "docs\mission planning refactoring\smoke_sw_code_baseline.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
git diff --check -- "docs/mission planning refactoring"
```

Result:

```text
py_compile: pass
mission bootstrap import-order contract smoke ok
0301-0304 mission-role SW code baseline smoke ok
mission planning refactor import-contract smoke ok
git diff --check: pass
```

Read-only sub-agent review flagged early `process_console` import and transitive early-import risk; the smoke was updated to cover both.

## Next

Next incomplete TODO: `0902 normalization fixture 작성`.
