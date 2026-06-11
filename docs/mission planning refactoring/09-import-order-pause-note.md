# Import Order Pause Note

Date: 2026-06-04, Asia/Seoul
Status: paused after the import-order fix requested by the user.

## What Was Fixed In This Stop

- Canonical trigger imports were tightened:
  - `modules/mission_planning/replanning/triggers/post_attack/pipeline.py` now imports attack helper internals from `modules.mission_planning.replanning.triggers.attack.pipeline`, not the old `pipelines.attack_plan_pipeline` wrapper.
  - `modules/mission_planning/pipelines/prior_mission_pipeline_impl.py` now loads post-attack rejoin from `modules.mission_planning.replanning.triggers.post_attack.pipeline`, not the old `pipelines.post_attack_rejoin_pipeline` wrapper.
- Import-order regression was reproduced and fixed:
  - Repro: importing `modules.mission_planning.mission_planning_gui` first, then `app.ui.main_window`, failed with `ModuleNotFoundError: No module named 'app.ui'`.
  - Root cause: the new `modules/mission_planning/app/__init__.py` made `modules/mission_planning/app` a regular top-level `app` package whenever `modules/mission_planning` was on `sys.path`.
  - Fix: removed `modules/mission_planning/app/__init__.py`. The folder still works as a namespace package for fully qualified imports such as `modules.mission_planning.app.bootstrap`.

## Verification Run

- `python -m compileall modules\mission_planning\replanning modules\mission_planning\app modules\mission_planning\mission_control modules\mission_planning\mission_planning_gui.py modules\mission_planning\pipelines\prior_mission_pipeline_impl.py`
- Wrapper identity smoke for moved canonical modules:
  - attack
  - imaging schedule
  - path deviation
  - post-attack rejoin
- Root and legacy wrapper identity smoke for supported old entrypoints.
- GUI import surface smoke for:
  - `run_attack_plan_pipeline`
  - `run_post_attack_rejoin_pipeline`
  - `run_imaging_schedule_replan_pipeline`
  - `run_path_deviation_replan_pipeline`
  - `MissionVisualizationTab`
  - `_planner_runtime_source_signature`
  - `_reload_planning_module`
- Planner runtime canonical reload smoke:
  - watch paths use the new trigger files
  - reload order uses the new trigger modules
  - bindings use the new trigger modules
  - old moved implementation modules are not watched/reloaded
- Import-order smoke passed:
  - `modules.mission_planning.mission_planning_gui -> app.ui.main_window`
  - `app.ui.main_window -> modules.mission_planning.mission_planning_gui`
  - `run -> modules.mission_planning.mission_planning_gui -> app.ui.main_window`

## Sub-Agent Findings Captured

- Important P1 risk: extracted refactor files are still untracked in git. A clean checkout or incomplete PR staging would fail because `mission_planning_gui.py` imports the new `app`, `mission_control`, and `replanning` modules.
- Important P1 risk found and fixed in this stop: top-level `app` import collision caused by `modules/mission_planning/app/__init__.py`.
- Prior and next-collab trigger folders exist, but they are compatibility entrypoints only. The main implementations still live under `pipelines/`.
- Current/general remaining hybrid, recon, and reexecute code still lives under `pipelines/` and remains active.
- Post-attack has no historical root wrapper at `modules.mission_planning.post_attack_rejoin_pipeline`; the supported compatibility path is `modules.mission_planning.pipelines.post_attack_rejoin_pipeline`.
- Wrapper contracts differ by path. Attack/post-attack pipeline wrappers re-export broad non-dunder symbols; imaging/path root wrappers only expose run/warm functions. This should be documented and tested before deletion or cleanup.
- Moved trigger implementations still intentionally depend on shared `pipelines.*` helper modules. Future cleanup needs an allow-list so shared helpers are not removed by mistake.

## TODO Additions For Resume

- Add a wrapper support matrix: supported old paths, unsupported root paths, `__all__`, and star-import policy.
- Add old/new import object identity snapshots for moved trigger pipelines.
- Add a clean-checkout or staging smoke so extracted refactor files cannot be omitted.
- Add a stale-import guard: moved old impl paths are forbidden; remaining shared `pipelines.*` helper dependencies need an allow-list.
- Keep deletion candidates conservative until file-by-file reachability is proven.

## Pause Point

- Work intentionally stops here.
- The third sub-agent review was still running and was closed because the user asked to pause after this fix.
- Next recommended resume step: do not move more code yet. First add the wrapper/path support matrix and clean-checkout/staging smoke so the untracked-file and import-contract risks are explicit.
