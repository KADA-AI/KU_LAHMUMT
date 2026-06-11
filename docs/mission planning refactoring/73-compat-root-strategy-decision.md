# Compat Root Strategy Decision

## Decision

Keep root compatibility paths for the current refactor, and do not move public compatibility imports under `compat/`.
Import-only root wrapper files are represented by package-level lazy aliases in
`modules/mission_planning/__init__.py`.

## Rationale

- Existing root import paths are already part of the supported import surface documented in `10-wrapper-support-matrix.md`.
- Import-only root wrappers are consolidated into package-level lazy aliases so
  the folder surface stays smaller without changing import paths.
- Manual visualizer tools now import canonical `modules.mission_planning.ui.id_relationship_tab`, while the root wrapper remains as a supported compatibility path for old callers.
- `manual/lah_rl_planner_gui.py` is now the canonical LAH RL planner implementation, while root `lah_rl_planner_gui.py` remains as a supported compatibility launcher/import wrapper.
- Bare imports `AnS`, `data_def`, and `config` are still used by active mission-planning code and MissionPlanner tools, but they are resolved through `MissionPlanner` path bootstrap instead of project-root shim folders/files.
- External launch surfaces still refer to existing mission-planning script names and paths, including `run.py`, `modules/common/button_wiring.py`, and monitoring visualization imports.
- Moving these paths into `compat/` now would require callers to change imports, which is the opposite of the compatibility phase.
- `compat/` can still become an internal helper namespace later, but it is not a replacement for the existing public root paths during this refactor.

## Boundary

This decision preserves existing root import paths and internal bare-import bootstrap behavior until a separate deprecation policy is documented and verified.

## Smoke

`smoke_compat_root_strategy_contract.py` verifies:

- old root import paths still import through package-level aliases.
- internal `MissionPlanner` bare-import targets still exist, while project-root `AnS/`, `data_def/`, and `config.py` shims stay absent.
- active manual/runtime code uses canonical visualizer imports while root/bare compatibility surfaces remain available.
- no `modules.mission_planning.compat` public import surface has been introduced.

## Verification

```powershell
python -m py_compile "docs\mission planning refactoring\smoke_compat_root_strategy_contract.py"
python "docs\mission planning refactoring\smoke_compat_root_strategy_contract.py"
python "docs\mission planning refactoring\smoke_import_contract.py"
git diff --check -- "docs/mission planning refactoring"
```

Expected result:

```text
compat root strategy smoke ok
mission planning refactor import-contract smoke ok
```

## Next

Next incomplete TODO: decide whether deprecated import paths should log or remain documentation-only.
