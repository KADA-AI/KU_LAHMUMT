# Mission Planning Structure

`modules/mission_planning` keeps active entrypoints and package directories at
the package root. Old import-only root modules are served by lazy aliases in
`__init__.py` instead of one wrapper file per import path.

## Active root layout

- `mission_planning_gui.py`
  - main GUI entrypoint used by the system
  - orchestrates initial planning and replan flow
- `MissionPlanner/`
  - core mission-planning engine and production 0302/0303/0304 generation path
  - `data_def/` remains the primary execution path
  - `planning_enhanced/` remains production logic, not test code
- `pipelines/`
  - runtime pipeline implementations
  - attack, prior-mission, and replan helper logic
  - public runtime entrypoints such as `next_collab_replan_pipeline.py`
- `runtime/`
  - runtime JSON I/O
  - latest input cache
  - mission-planning state and logging helpers
  - next-collab replan store/runtime helpers used by monitoring and GUI
- `ui/`
  - active GUI widgets and environment helpers
- `legacy/`
  - archived wrappers, standalone tools, tests, documents, and static leftovers
- `_paths.py`
  - shared path helpers for runtime modules

## Root surface final cleanup TODO

The current root still exposes compatibility wrappers, manual tools, archived notes, generated/static files, and active entrypoints together. The final cleanup should make the root boring and predictable without breaking old imports.

Target root surface:

- public entrypoints: `mission_planning_gui.py`, `__init__.py`, `README.md`, `_paths.py`
- compatibility aliases: `__init__.py` preserves old import paths without loose root `.py` files
- active packages: `app/`, `engine/`, `mission_control/`, `MissionPlanner/`, `next_area_mode/`, `pipelines/`, `planners/`, `replanning/`, `runtime/`, `ui/`
- manual bucket: `manual/`
- archive bucket: `legacy/`

TODO:

- [x] Remove local cache folders such as `__pycache__/` from the worktree and keep them ignored.
- [x] Write a root surface inventory smoke that classifies every root item as `public`, `active-package`, `compat-wrapper`, `manual-tool`, `archive`, or `cleanup-candidate`.
- [x] Update external callers to canonical paths before deleting root wrappers.
  - `run.py` and dashboard launch code keep `mission_planning_gui.py` until a launcher replacement is explicitly approved.
  - monitoring/common imports should move from old wrapper paths to canonical `runtime/`, `replanning/`, or `app/` paths.
- [x] Convert root import-only compatibility wrappers into a single obvious pattern.
  - no implementation logic in loose root wrappers.
  - old import paths are mapped in `__init__.py` with lazy aliases.
  - no runtime deprecation warnings or logging at import time.
  - wrapper identity smokes must keep passing.
  - enforced by `smoke_wrapper_template_contract.py` and `smoke_root_surface_inventory.py`.
- [x] Move manual/operator tools into a clear manual bucket.
  - selected bucket: `manual/`
  - moved: `manual/archive/logic_memo/`, `manual/reference/logic_ref.md`, `manual/reference/map.html`
  - moved: `manual/lah_rl_planner_gui.py`, with root `lah_rl_planner_gui.py` kept as a compatibility launcher wrapper.
  - moved: `manual/MissionVisualizer/main_visualizer.py` is the canonical visualizer implementation.
  - moved: `manual/logic_test/` now owns manual division/dubins test tools and checked-in fixture candidates.
  - package-level aliases preserve old `MissionVisualizer` and `logic_test` import paths without root folders.
  - converted duplicate: `MissionPlanner/tools/main_visualizer.py` now wraps canonical `manual/MissionVisualizer/main_visualizer.py`.
  - active package, not manual: `next_area_mode/` stays at root because runtime next-collab code imports it directly.
  - keep thin launch wrappers only for paths that operators still run directly.
- [x] Decide the public location for `lah_rl_planner_gui.py`.
  - canonical implementation: `manual/lah_rl_planner_gui.py`
  - compatibility launcher/import wrapper: root `lah_rl_planner_gui.py`
- [x] Keep `MissionPlanner/` in place until engine migration is ready.
  - do not casually move `MissionPlanner/AnS/`, `MissionPlanner/data_def/`, `MissionPlanner/planning_enhanced/`, DEM/model/config files, or ID counter files.
  - project-root `AnS/`, `data_def/`, and `config.py` shims are intentionally absent; bare imports resolve through `MissionPlanner` path bootstrap.
- [ ] After external imports are clean, start the old-import-path deprecation clock.
  - minimum period: 30 calendar days from merged notice.
  - no old import path removal before the deprecation policy smoke is updated and green.
- [x] Remove import-only root wrappers in a single alias-backed batch.
  - removed loose wrapper files for pipeline, runtime, UI, and helper imports.
  - old module import paths remain supported by package aliases.
  - each batch must pass import contract, GUI launch smoke, manual entrypoint smoke, and relevant scenario smoke.
- [ ] Final root acceptance check.
  - `modules/mission_planning` root contains only public entrypoints, active package directories, and documented temporary wrappers.
  - every remaining root file has an owner and a reason in this README.
  - root inventory smoke fails if a new loose file appears without classification.

## Preset cleanup

- The GUI exposes only the base preset: `dubins_mode`
- General mission-planning options now all run through the same base preset path
- Manual FOV is a mode inside the base preset, not a separate preset
- Removed preset/profile branches are documented under `legacy/`
- Runtime behavior that still matters follows downstream values such as:
  - `area_sweep_mode`
  - `area_split_mode`
  - `uav_plan_mode`
  - auto/manual FOV mode

In other words, preset-specific branching was removed from the general mission-planning path without rewriting the attack, prior-mission, or replan execution logic.

## Refactor boundary

Safe to archive or reorganize:

- wrapper modules no longer imported by active runtime code
- standalone app folders not used by the active runtime
- tests, exploratory tools, and documents
- static/generated leftovers

Current public entrypoints:

- next collaborative mission replan:
  - pipeline: `pipelines/next_collab_replan_pipeline.py`
  - runtime store/helpers: `runtime/next_collab_replan_store.py`, `runtime/next_collab_replan_runtime.py`
  - `modules/common/next_collab_replan_store.py` is compatibility-only

Do not move casually:

- `MissionPlanner/AnS/`
- `MissionPlanner/data_def/` except for narrowly-scoped runtime-safe helper extraction
- `MissionPlanner/planning_enhanced/`
- `mission_planning_gui.py`
- attack, prior-mission, and replan pipeline behavior

These remain the primary runtime path for mission generation.
