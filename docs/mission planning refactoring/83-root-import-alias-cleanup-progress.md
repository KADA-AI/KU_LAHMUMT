# Root Import Alias Cleanup Progress

## Scope

This checkpoint removes loose import-only compatibility wrapper files from
`modules/mission_planning` root while preserving their old import paths.

## Changed

- Removed root wrapper files for pipeline, runtime, UI, and helper compatibility
  imports.
- Added lazy package-level aliases in `modules/mission_planning/__init__.py`.
- Kept direct operator/public files at root:
  - `mission_planning_gui.py`
  - `lah_rl_planner_gui.py`
  - `__init__.py`
  - `_paths.py`
  - `README.md`

## Preserved Import Paths

Examples that still import:

- `modules.mission_planning.attack_plan_pipeline`
- `modules.mission_planning.prior_mission_pipeline`
- `modules.mission_planning.next_collab_replan_pipeline`
- `modules.mission_planning.json_io`
- `modules.mission_planning.id_relationship_tab`

## Boundary

No project root, `Logs`, `resource`, `settings`, or backup folder was touched.
No `MissionPlanner/AnS`, `MissionPlanner/data_def`, or `logic_test` move is
included in this checkpoint.

## Verification

```powershell
python "docs\mission planning refactoring\smoke_import_contract.py"
python "docs\mission planning refactoring\smoke_root_surface_inventory.py"
python "docs\mission planning refactoring\smoke_wrapper_template_contract.py"
python "docs\mission planning refactoring\smoke_compat_root_strategy_contract.py"
python "docs\mission planning refactoring\smoke_deprecated_import_policy_contract.py"
python "docs\mission planning refactoring\smoke_deletion_candidate_reachability.py"
python "docs\mission planning refactoring\smoke_root_wrapper_deprecation_period.py"
```

Expected result:

```text
mission planning refactor import-contract smoke ok
mission planning root surface inventory smoke ok
wrapper template contract smoke ok
compat root strategy smoke ok
deprecated import policy smoke ok
deletion candidate reachability smoke ok
root wrapper deprecation period smoke ok
```
