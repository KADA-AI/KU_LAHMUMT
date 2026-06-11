# Manual/Operator Entrypoint Inventory Progress

## Scope

This checkpoint freezes the runnable manual/operator entrypoint surface that must not be deleted, renamed, or silently converted during the mission planning refactor.

## Added

- `smoke_manual_operator_entrypoints.py`

## Contract Captured

- Public operator launchers:
  - `mission_planning_gui.py` remains the active mission planning GUI launcher.
  - import-time `configure_mission_role()` remains part of the launcher contract.
  - `MISSION_PLANNING_GUI_SMOKE_LAUNCH=1` remains the lightweight smoke-launch branch.
  - `manual/lah_rl_planner_gui.py` remains the canonical operator GUI tied to `portable_mission_bundle/models/latest_model.zip`.
  - root `lah_rl_planner_gui.py` remains a thin compatibility launcher/import wrapper.
  - the main GUI still opens `LAHPlannerWindow` through both package and bare-import fallback paths.
- Manual visualizers:
  - `manual/MissionVisualizer/main_visualizer.py` is the canonical implementation.
  - `MissionVisualizer/main_visualizer.py` and `MissionPlanner/tools/main_visualizer.py` remain runnable compatibility entrypoints.
  - legacy visualizer copies and the compatibility package wrapper remain documented so duplicate cleanup can happen by decision, not accident.
- Next-area and next-collab manual planner launchers:
  - `next_area_mode/main.py` prepares `sys.path`, calls `freeze_support()` on Windows, defaults flow mode to `initial`, then launches `planner_window.main`.
  - the next-area flow mode environment key remains `MISSION_NEXT_AREA_FLOW_MODE`.
  - `planners/next_collab_division/main.py` and `logic_test/division_test/main.py` default `DIVISION_TEST_FLOW_MODE` to `initial`.
  - the GUI windows still instantiate `QApplication`, create the planner window, show it, and execute the app loop.
- Auxiliary tools:
  - `MissionPlanner/main_MP.py`, `corridor_gui.py`, `corridor_planner.py`, `tools/test_div_area.py`, `tools/turn_link_visualizer.py`, and `tools/DTA.py` remain manually runnable tool entrypoints.
  - `lah_attack_assistance.py` remains a CLI entrypoint with coordinate args and `--output-json`.
  - `MissionPlanner/data_def/dubins_turn_link.py` remains both a CLI utility and an active helper imported by the 0303 builder.
- Portable bundle:
  - `portable_mission_bundle/app.py` keeps `create_app(ROOT)`, `MISSION_APP_HOST`, and `MISSION_APP_PORT`.
  - `run_portable.bat` still changes to the bundle directory and runs `python app.py`.
  - portable service model/config paths remain `latest_model.zip` and `model_config.json`.

## Boundary

This does not decide which manual workflows to keep, wrap, archive, or delete. That is the adjacent owner-decision TODO.

This does not launch Qt windows, start the portable Flask server, or execute attack assistance. The dedicated portable and subprocess smoke TODOs remain separate.

Prototype UAV-pattern scripts under `MissionPlanner/tools/UAV_pattern/**` are not promoted to supported operator entrypoints by this checkpoint; they remain subject to the deletion/owner workflow.

Known import-side-effect risks are recorded but not fixed here: `MissionPlanner/FFAR_list_class.py` and `MissionPlanner/tools/UAV_pattern/Standoff_BF/visualization.py` can open matplotlib windows if imported directly.

## Why This Is Safe

No runtime code changed. The smoke reads source files only and checks stable launch markers, environment defaults, wrappers, and model/path dependencies.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_manual_operator_entrypoints.py"
python "docs\mission planning refactoring\smoke_manual_operator_entrypoints.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
git diff --check -- "docs/mission planning refactoring"
```

Expected result:

```text
manual/operator entrypoint inventory smoke ok
mission planning refactor import-contract smoke ok
```

## Next

Next incomplete TODO: `monitoring/common/app external import contract inventory`.
