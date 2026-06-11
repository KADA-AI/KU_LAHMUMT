# Manual Workflow Owner Decisions Progress

## Scope

This checkpoint records conservative owner/status decisions for `manual/logic_test`, tool GUI, and portable bundle workflows. Current code keeps manual tools under `manual/` and preserves old imports with package-level aliases.

## Added

- `smoke_manual_workflow_owner_decisions.py`

## Decision Matrix

| Surface | Decision | Owner bucket | Rationale |
| --- | --- | --- | --- |
| `manual/logic_test/division_test/**` | delete-hold | next-collab/planning-enhanced owner | Manual/golden fixture candidate. It launches a large division planner GUI and has 19 checked-in JSON outputs under `output/`. Delete only after fixture ownership is decided. |
| `manual/logic_test/dubins_test/**` | wrapper candidate | Dubins/flight-path owner | Manual GUI/CLI mirror for Dubins turn-link behavior. Canonical active helper remains `MissionPlanner/data_def/dubins_turn_link.py`; later cleanup should delegate here instead of keeping duplicate logic. |
| `manual/MissionVisualizer/main_visualizer.py` | keep canonical | operator/manual visualization | Public/manual visualizer implementation. |
| `MissionVisualizer` old import path | package-alias | operator/manual visualization | Public compatibility import path mapped by `modules/mission_planning/__init__.py`; no root folder is required. |
| `MissionPlanner/tools/main_visualizer.py` | wrapper | operator/manual visualization | Thin compatibility wrapper that delegates to canonical `manual/MissionVisualizer/main_visualizer.py`. |
| legacy visualizer copies | archive-hold | compatibility/archive owner | Compatibility/archive copies are not active canonicals. Do not delete until legacy strategy is decided. |
| `MissionPlanner/main_MP.py` | keep-hold | mission planning refactor owner | Manual planning-enhanced GUI launcher. Keep until GUI launch ownership is clarified. |
| `MissionPlanner/corridor_gui.py`, `MissionPlanner/corridor_planner.py` | keep-hold | manual corridor tool owner | Auxiliary corridor tools with runnable/manual entrypoints and map output contracts. |
| `MissionPlanner/tools/test_div_area.py`, `turn_link_visualizer.py`, `DTA.py` | keep-hold | manual tool owner | Matplotlib/manual analysis tools. Not active runtime, but not safe to delete without owner confirmation. |
| `MissionPlanner/portable_mission_bundle/**` | keep | portable/RL operator workflow | Standalone portable web/RL bundle with app, batch launcher, model/config, static/templates, data folders, and service code. |
| `manual/lah_rl_planner_gui.py` | keep canonical | portable/RL operator workflow | Operator GUI directly references portable bundle and `latest_model.zip`. |
| root `lah_rl_planner_gui.py` | wrapper | portable/RL compatibility launcher | Keep the old import/direct-launch path as a thin wrapper during migration. |
| `MissionPlanner/tools/UAV_pattern/Nadir_BF/**` | keep active support | mission generation/runtime | Active 0303 builder imports `area_nadir_bf_planner.build_nadir_bf_overflight_coords`. |
| Other `MissionPlanner/tools/UAV_pattern/**` demos | archive-hold | manual prototype owner needed | Prototype/demo/database scripts are not promoted to supported operator entrypoints, but deletion needs owner/reachability review. |

No delete action is approved by this checkpoint.

## Guardrails

- Keep `manual/MissionVisualizer/main_visualizer.py` as canonical.
- Keep old `modules.mission_planning.MissionVisualizer.*` imports alive through package-level aliases.
- Keep `MissionPlanner/tools/main_visualizer.py` runnable as a thin wrapper.
- Keep `manual/logic_test/division_test/output/*.json` as manual/golden candidates until generated output fixture policy is decided.
- Treat `manual/logic_test/dubins_test` as a wrapper/delegation candidate, not as disposable test-only code.
- Keep portable bundle root files and model/config artifacts intact until `python app.py`/`run_portable.bat` smoke passes.
- Do not treat all `UAV_pattern/**` files equally: `Nadir_BF/**` has an active 0303 dependency, while other demo folders are archive-hold candidates.

## Boundary

This is an owner/status decision record, not a deletion batch. Remaining deletion TODOs must still run reachability checks and owner confirmation before removing any file.

This does not perform the later portable server smoke, attack-assistance subprocess smoke, or next-area/next-collab flow-mode smoke.

## Why This Is Safe

The smoke reads source files, checks the manual visualizer wrapper contract, verifies portable bundle files exist, and records deletion holds.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_manual_workflow_owner_decisions.py"
python "docs\mission planning refactoring\smoke_manual_workflow_owner_decisions.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
git diff --check -- "docs/mission planning refactoring"
```

Expected result:

```text
manual workflow owner decision smoke ok
mission planning refactor import-contract smoke ok
```

## Next

Next incomplete TODO: `lah_attack_assistance.py subprocess smoke`.
