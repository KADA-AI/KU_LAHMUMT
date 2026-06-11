# Mission Planning GUI Public Launcher Handoff Progress

## Scope

This checkpoint prepares `mission_planning_gui.py` to stay the public launcher while moving launcher execution mechanics into an internal app helper.

## Added

- `modules/mission_planning/app/gui_entrypoint.py`
- `smoke_mission_planning_gui_public_launcher_handoff.py`

## Changed

- `_smoke_launch_main()` in `mission_planning_gui.py` now delegates to `app.gui_entrypoint.smoke_launch_main(...)`.
- The `if __name__ == "__main__"` block in `mission_planning_gui.py` now delegates to `app.gui_entrypoint.run_public_gui_entrypoint(...)`.
- `MainWindow`, `PROJECT_ROOT`, planner runtime bindings, and all existing public imports remain on `mission_planning_gui.py`.

## Contract Captured

- `mission_planning_gui.py` remains the public import and executable script.
- `app/gui_entrypoint.py` does not import PyQt or `mission_planning_gui.py` at module import time.
- Smoke launch mode still uses `MISSION_PLANNING_GUI_SMOKE_LAUNCH`.
- Normal launch still creates `QApplication`, loads the shared stylesheet, creates `MainWindow`, applies initial visibility, and enters `app.exec_()`.

## Boundary

This is not the large GUI split. It only moves entrypoint mechanics behind an internal helper so the later implementation handoff has a stable target.

## Verification

```powershell
python -m py_compile "modules\mission_planning\app\gui_entrypoint.py" "modules\mission_planning\mission_planning_gui.py" "docs\mission planning refactoring\smoke_mission_planning_gui_public_launcher_handoff.py"
python "docs\mission planning refactoring\smoke_mission_planning_gui_public_launcher_handoff.py"
python "docs\mission planning refactoring\smoke_mission_planning_gui.py" --mode all --timeout-s 60
python "docs\mission planning refactoring\smoke_import_contract.py"
git diff --check -- "modules/mission_planning" "docs/mission planning refactoring"
```

Expected result:

```text
mission_planning_gui public launcher handoff smoke ok
mission_planning_gui all smoke ok
mission planning refactor import-contract smoke ok
```

## Next

Next incomplete TODO: verify deletion candidates by reachability.
