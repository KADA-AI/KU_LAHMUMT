# Deletion Owner Manual Workflow Progress

## Scope

This checkpoint confirms owner bucket and manual workflow status for deletion/archive candidates after reachability verification.

## Added

- `smoke_deletion_owner_manual_workflow.py`

## Owner Matrix

| Candidate surface | Decision | Owner bucket | Manual workflow / deletion condition |
| --- | --- | --- | --- |
| root compatibility wrappers | keep | compatibility/import-surface owner | Keep through the current refactor. Delete only after a documented deprecation period and external import smoke update. |
| `legacy/wrappers` | keep-hold | compatibility/archive owner | Keep until the legacy archive strategy is decided; they still participate in wrapper/import smoke. |
| `legacy/compat_packages` | archive-hold | compatibility/archive owner | Keep as archive compatibility packages until legacy strategy decides whether to preserve or remove them. |
| `legacy/apps` | archive-hold | manual app archive owner | Keep archived visualizer/next-area app copies until manual archive expectations are decided. |
| `legacy/tests` | archive-hold | manual/golden fixture owner | Keep archived division/dubins tests and outputs until fixture/archive strategy is decided. |
| `manual/logic_test/division_test/**` | delete-hold | next-collab/planning-enhanced owner | Manual/golden fixture candidate. Delete only after generated output fixture policy and owner signoff. |
| `manual/logic_test/dubins_test/**` | wrapper candidate | Dubins/flight-path owner | Convert toward canonical `MissionPlanner/data_def/dubins_turn_link.py` before considering deletion. |
| division-test generated output JSON | fixture-hold | generated fixture owner | Decide fixture-vs-delete policy before removing checked-in output JSON. |
| duplicate visualizer import path | package-alias | operator/manual visualization owner | `MissionVisualizer` old imports now map to canonical `manual/MissionVisualizer`; `MissionPlanner/tools/main_visualizer.py` remains a thin wrapper. |
| `MissionPlanner/tools/UAV_pattern/Nadir_BF/**` | keep | mission-generation owner | Active 0303 builder dependency. Excluded from deletion candidates. |
| other UAV-pattern prototype scripts | archive-hold | manual prototype owner needed | Archive or delete only after owner confirms no manual research/demo workflow depends on them. |
| portable mission bundle | keep | portable/RL workflow owner | Active portable Flask/RL workflow. Keep root files, model/config, data folders, and batch launcher. |
| `d0304 copy.py` | backup-hold | artifact-builder owner | Backup-style file. Decide in the backup-style file policy before deletion. |
| TensorBoard training logs/models | artifact-hold | training/model owner | Treat as training artifacts; decide generated-output/artifact policy before deletion. |
| tracked `__pycache__`/`.pyc` | delete-if-present | repository hygiene owner | None are currently tracked; delete only if future tracked bytecode appears. |

## Decision

No deletion is approved by this checkpoint. Every candidate has an owner bucket and a future delete/archive batch condition.

## Boundary

This is an owner/manual workflow confirmation record. It does not delete files, rewrite wrappers, or import side-effect-prone manual tools.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_deletion_owner_manual_workflow.py"
python "docs\mission planning refactoring\smoke_deletion_owner_manual_workflow.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
git diff --check -- "docs/mission planning refactoring"
```

Expected result:

```text
deletion owner/manual workflow smoke ok
mission planning refactor import-contract smoke ok
```

## Next

Next incomplete TODO: decide whether generated output should be kept as fixtures or deleted.
